//! A loopback-only USB/IP server. The framing follows
//! <https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html>.

use crate::clock::Clock;
use crate::device::{IsoTiming, Setup, UsbDevice, EPIPE};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap};
use std::io::{self, Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread;
use std::time::Duration;

const VERSION: u16 = 0x0111;
const OP_REQ_DEVLIST: u16 = 0x8005;
const OP_REQ_IMPORT: u16 = 0x8003;
const CMD_SUBMIT: u32 = 1;
const CMD_UNLINK: u32 = 2;
const RET_SUBMIT: u32 = 3;
const RET_UNLINK: u32 = 4;
const ECONNRESET: i32 = -104;
const MAX_TRANSFER: usize = 4 * 1024 * 1024;
const MAX_ISO_PACKETS: usize = 1024;
/// An endpoint's transfers continue the previous one's timing, unless that ended longer ago than this (the
/// stream was stopped): then they restart from the present.
const RESYNC_NS: u64 = 20_000_000;
static URB_ERRORS_LOGGED: AtomicU32 = AtomicU32::new(0);

struct Exported {
    device: Arc<dyn UsbDevice>,
    devnum: u32,
    active: Arc<AtomicBool>,
}

type Devices = Arc<RwLock<BTreeMap<String, Exported>>>;

/// Serves [`UsbDevice`]s on `127.0.0.1`, to be attached with `usbip.exe --tcp-port <port> attach ...`.
pub struct Server {
    devices: Devices,
    stop: Arc<AtomicBool>,
    listener: Option<thread::JoinHandle<()>>,
    port: u16,
    next_devnum: AtomicU32,
}

impl Server {
    /// Listens on `port` of the loopback interface, 0 for any free port (see [`Server::port`]).
    pub fn start(port: u16, clock: Arc<dyn Clock>) -> io::Result<Self> {
        let listener = TcpListener::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, port))?;
        let port = listener.local_addr()?.port();
        let devices: Devices = Default::default();
        let stop = Arc::new(AtomicBool::new(false));
        #[cfg(windows)]
        // sleeps wake up within a millisecond, not a 15.6 ms scheduler tick
        unsafe {
            windows_sys::Win32::Media::timeBeginPeriod(1);
        }
        let (worker_devices, worker_stop) = (devices.clone(), stop.clone());
        let worker = thread::Builder::new().name("usbip-listener".into()).spawn(move || {
            while !worker_stop.load(Ordering::Acquire) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        if worker_stop.load(Ordering::Acquire) {
                            break;
                        }
                        let (devices, stop, clock) = (worker_devices.clone(), worker_stop.clone(), clock.clone());
                        let _ = thread::Builder::new().name("usbip-client".into()).spawn(move || {
                            if let Err(error) = serve_client(stream, &devices, &stop, &clock) {
                                if !matches!(
                                    error.kind(),
                                    io::ErrorKind::UnexpectedEof
                                        | io::ErrorKind::ConnectionReset
                                        | io::ErrorKind::ConnectionAborted
                                ) {
                                    eprintln!("USB/IP client ended: {error}");
                                }
                            }
                        });
                    }
                    Err(error) => {
                        eprintln!("USB/IP accept failed: {error}");
                        thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        })?;
        Ok(Self { devices, stop, listener: Some(worker), port, next_devnum: AtomicU32::new(1) })
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    /// Exports `device` as `busid` (at most 31 bytes, e.g. `1-1`), replacing (and unplugging) a device
    /// exported as `busid` before.
    pub fn add(&self, busid: &str, device: Arc<dyn UsbDevice>) {
        assert!(busid.len() < 32, "USB/IP bus ids are at most 31 bytes");
        let devnum = self.next_devnum.fetch_add(1, Ordering::Relaxed);
        let exported = Exported { device, devnum, active: Arc::new(AtomicBool::new(true)) };
        if let Some(previous) = self.devices.write().unwrap().insert(busid.to_owned(), exported) {
            previous.active.store(false, Ordering::Release);
        }
    }

    /// Stops exporting `busid`: its connection is closed, so Windows sees it unplugged.
    pub fn remove(&self, busid: &str) {
        if let Some(previous) = self.devices.write().unwrap().remove(busid) {
            previous.active.store(false, Ordering::Release);
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        // wake up the blocking accept
        let _ = TcpStream::connect((Ipv4Addr::LOCALHOST, self.port));
        if let Some(worker) = self.listener.take() {
            let _ = worker.join();
        }
        for exported in self.devices.write().unwrap().values() {
            exported.active.store(false, Ordering::Release);
        }
        #[cfg(windows)]
        unsafe {
            windows_sys::Win32::Media::timeEndPeriod(1);
        }
    }
}

fn put_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn be32(bytes: &[u8]) -> u32 {
    u32::from_be_bytes(bytes[..4].try_into().unwrap())
}

fn fixed(out: &mut Vec<u8>, text: &str, width: usize) {
    let bytes = text.as_bytes();
    let used = bytes.len().min(width - 1);
    out.extend_from_slice(&bytes[..used]);
    out.resize(out.len() + width - used, 0);
}

/// The interface (class, subclass, protocol) of each interface's first alternate setting.
fn interfaces(configuration: &[u8]) -> Vec<[u8; 3]> {
    let mut found = vec![];
    let mut at = 0;
    while at + 1 < configuration.len() && configuration[at] >= 2 {
        let descriptor = &configuration[at..(at + configuration[at] as usize).min(configuration.len())];
        if descriptor.len() >= 9 && descriptor[1] == 4 && descriptor[3] == 0 {
            found.push([descriptor[5], descriptor[6], descriptor[7]]);
        }
        at += configuration[at] as usize;
    }
    found
}

/// `usbip_usb_device`, followed by `usbip_usb_interface`s for a device list.
fn device_info(busid: &str, exported: &Exported, with_interfaces: bool) -> Vec<u8> {
    let device = exported.device.device_descriptor();
    let configuration = exported.device.configuration_descriptor();
    let interfaces = interfaces(&configuration);
    let mut out = Vec::with_capacity(312 + 4 * interfaces.len());
    fixed(&mut out, &format!("/virtual/usbip-audio/{busid}"), 256);
    fixed(&mut out, busid, 32);
    put_u32(&mut out, 1); // busnum
    put_u32(&mut out, exported.devnum);
    put_u32(&mut out, exported.device.speed().usbip_value());
    put_u16(&mut out, u16::from_le_bytes([device[8], device[9]]));
    put_u16(&mut out, u16::from_le_bytes([device[10], device[11]]));
    put_u16(&mut out, u16::from_le_bytes([device[12], device[13]]));
    out.extend_from_slice(&[device[4], device[5], device[6], configuration[5], device[17]]);
    out.push(interfaces.len() as u8);
    if with_interfaces {
        for interface in interfaces {
            out.extend_from_slice(&interface);
            out.push(0);
        }
    }
    out
}

fn op_reply_header(code: u16, status: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(12);
    put_u16(&mut out, VERSION);
    put_u16(&mut out, code);
    put_u32(&mut out, status);
    out
}

fn serve_client(mut stream: TcpStream, devices: &Devices, stop: &AtomicBool, clock: &Arc<dyn Clock>) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(Duration::from_millis(500)))?;
    stream.set_nodelay(true)?;
    let mut preamble = [0; 8];
    read_exact_active(&mut stream, &mut preamble, stop, None)?;
    if u16::from_be_bytes([preamble[0], preamble[1]]) != VERSION {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "unsupported USB/IP version"));
    }
    match u16::from_be_bytes([preamble[2], preamble[3]]) {
        OP_REQ_DEVLIST => {
            let devices = devices.read().unwrap();
            let active: Vec<_> = devices.iter().filter(|(_, e)| e.active.load(Ordering::Acquire)).collect();
            let mut out = op_reply_header(0x0005, 0);
            put_u32(&mut out, active.len() as u32);
            for (busid, exported) in active {
                out.extend_from_slice(&device_info(busid, exported, true));
            }
            stream.write_all(&out)
        }
        OP_REQ_IMPORT => {
            let mut field = [0; 32];
            read_exact_active(&mut stream, &mut field, stop, None)?;
            let end = field.iter().position(|&byte| byte == 0).unwrap_or(32);
            let busid = std::str::from_utf8(&field[..end]).map_err(|_| io::ErrorKind::InvalidData)?;
            let found = devices
                .read()
                .unwrap()
                .get(busid)
                .filter(|e| e.active.load(Ordering::Acquire))
                .map(|e| (device_info(busid, e, false), e.device.clone(), e.active.clone()));
            match found {
                Some((info, device, active)) => {
                    let mut out = op_reply_header(0x0003, 0);
                    out.extend_from_slice(&info);
                    stream.write_all(&out)?;
                    serve_imported(stream, device, &active, stop, clock)
                }
                None => stream.write_all(&op_reply_header(0x0003, 1)),
            }
        }
        _ => Err(io::Error::new(io::ErrorKind::InvalidData, "unknown USB/IP operation")),
    }
}

fn read_exact_active(
    stream: &mut TcpStream,
    bytes: &mut [u8],
    stop: &AtomicBool,
    active: Option<&AtomicBool>,
) -> io::Result<()> {
    let mut offset = 0;
    while offset < bytes.len() {
        if stop.load(Ordering::Acquire) || active.is_some_and(|a| !a.load(Ordering::Acquire)) {
            return Err(io::ErrorKind::ConnectionAborted.into());
        }
        match stream.read(&mut bytes[offset..]) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(count) => offset += count,
            Err(error) if matches!(error.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

/// An isochronous transfer waiting for the end of the time it covers.
struct PendingUrb {
    address: u8,
    timing: IsoTiming,
    start_frame: u32,
    /// the submitted packet descriptors (offset, length, actual length, status)
    descriptors: Vec<[u8; 16]>,
    /// OUT: each packet's length
    out_lengths: Vec<usize>,
}

#[derive(Default)]
struct Pending {
    due: BinaryHeap<Reverse<(u64, u32)>>,
    urbs: HashMap<u32, PendingUrb>,
    closed: bool,
}

struct Connection {
    writer: Mutex<TcpStream>,
    pending: Mutex<Pending>,
    changed: Condvar,
}

impl Connection {
    fn send(&self, bytes: &[u8]) -> io::Result<()> {
        self.writer.lock().unwrap().write_all(bytes)
    }
}

fn ret_submit(seq: u32, status: i32, actual: usize, start_frame: u32, packets: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(48);
    for value in [RET_SUBMIT, seq, 0, 0, 0, status as u32, actual as u32, start_frame, packets, 0] {
        put_u32(&mut out, value);
    }
    out.extend_from_slice(&[0; 8]);
    out
}

/// The timing of the next transfer of `packets` on an endpoint whose previous transfer ended at `next`.
pub(crate) fn schedule(next: &mut Option<u64>, now: u64, packets: u32, packet_ns: u64) -> IsoTiming {
    let start = match *next {
        Some(next) if next.saturating_add(RESYNC_NS) >= now => next,
        _ => now - now % packet_ns,
    };
    *next = Some(start + packets as u64 * packet_ns);
    IsoTiming { start_ns: start, packet_ns }
}

fn serve_imported(
    mut stream: TcpStream,
    device: Arc<dyn UsbDevice>,
    active: &AtomicBool,
    stop: &AtomicBool,
    clock: &Arc<dyn Clock>,
) -> io::Result<()> {
    let connection = Arc::new(Connection {
        writer: Mutex::new(stream.try_clone()?),
        pending: Default::default(),
        changed: Condvar::new(),
    });
    let completer = {
        let (connection, device, clock) = (connection.clone(), device.clone(), clock.clone());
        thread::Builder::new().name("usbip-iso".into()).spawn(move || complete_iso(&connection, &*device, &*clock))?
    };
    let result = read_urbs(&mut stream, &connection, &*device, active, stop, &**clock);
    connection.pending.lock().unwrap().closed = true;
    connection.changed.notify_all();
    let _ = completer.join();
    let _ = stream.shutdown(std::net::Shutdown::Both);
    result
}

fn read_urbs(
    stream: &mut TcpStream,
    connection: &Connection,
    device: &dyn UsbDevice,
    active: &AtomicBool,
    stop: &AtomicBool,
    clock: &dyn Clock,
) -> io::Result<()> {
    let mut next_due: HashMap<u8, Option<u64>> = HashMap::new();
    loop {
        let mut header = [0; 48];
        read_exact_active(stream, &mut header, stop, Some(active))?;
        let command = be32(&header[0..4]);
        let seq = be32(&header[4..8]);
        let direction = be32(&header[12..16]);
        let ep = be32(&header[16..20]);
        match command {
            CMD_SUBMIT => {
                let length = be32(&header[24..28]) as usize;
                let start_frame = be32(&header[28..32]);
                let packets = be32(&header[32..36]);
                let iso_packets = if packets == u32::MAX { 0 } else { packets as usize };
                if length > MAX_TRANSFER || iso_packets > MAX_ISO_PACKETS || direction > 1 || ep > 15 {
                    return Err(io::Error::new(io::ErrorKind::InvalidData, "oversized USB/IP transfer"));
                }
                let mut payload = vec![0; if direction == 0 { length } else { 0 }];
                read_exact_active(stream, &mut payload, stop, Some(active))?;
                let mut iso = vec![0; iso_packets * 16];
                read_exact_active(stream, &mut iso, stop, Some(active))?;
                let address = ep as u8 | ((direction as u8) << 7);

                if ep == 0 {
                    let setup = Setup::parse(header[40..48].try_into().unwrap());
                    let reply = match device.control(setup, &payload) {
                        Ok(mut data) => {
                            data.truncate(length);
                            let actual = if direction == 0 { payload.len() } else { data.len() };
                            let mut out = ret_submit(seq, 0, actual, 0, packets);
                            if direction == 1 {
                                out.extend_from_slice(&data);
                            }
                            out
                        }
                        Err(status) => {
                            log_failure(&format!("control {setup:x?} status {status}"));
                            ret_submit(seq, status, 0, 0, packets)
                        }
                    };
                    connection.send(&reply)?;
                    continue;
                }
                let Some(endpoint) = device.iso_endpoint(address).filter(|_| iso_packets > 0) else {
                    log_failure(&format!("transfer to endpoint {address:#04x}, {iso_packets} iso packets"));
                    connection.send(&ret_submit(seq, EPIPE, 0, start_frame, packets))?;
                    continue;
                };
                let descriptors: Vec<[u8; 16]> = iso.as_chunks::<16>().0.to_vec();
                let timing =
                    schedule(next_due.entry(address).or_default(), clock.now_ns(), packets, endpoint.packet_ns);
                let mut out_lengths = vec![];
                if direction == 0 {
                    let slices: Vec<&[u8]> = descriptors
                        .iter()
                        .map(|d| {
                            let offset = (be32(&d[0..4]) as usize).min(payload.len());
                            let len = (be32(&d[4..8]) as usize).min(payload.len() - offset);
                            &payload[offset..offset + len]
                        })
                        .collect();
                    out_lengths = slices.iter().map(|s| s.len()).collect();
                    device.iso_out(address, timing, &slices);
                }
                let due = timing.start_ns + iso_packets as u64 * timing.packet_ns;
                let mut pending = connection.pending.lock().unwrap();
                pending.due.push(Reverse((due, seq)));
                pending.urbs.insert(seq, PendingUrb { address, timing, start_frame, descriptors, out_lengths });
                connection.changed.notify_all();
            }
            CMD_UNLINK => {
                let unlinked = be32(&header[20..24]);
                // a transfer still waiting is never completed; one already completed is acknowledged with 0
                let status = match connection.pending.lock().unwrap().urbs.remove(&unlinked) {
                    Some(_) => ECONNRESET,
                    None => 0,
                };
                let mut out = Vec::with_capacity(48);
                for value in [RET_UNLINK, seq, 0, 0, 0, status as u32] {
                    put_u32(&mut out, value);
                }
                out.resize(48, 0);
                connection.send(&out)?;
            }
            _ => return Err(io::Error::new(io::ErrorKind::InvalidData, "unknown USB/IP command")),
        }
    }
}

fn log_failure(what: &str) {
    if URB_ERRORS_LOGGED.fetch_add(1, Ordering::Relaxed) < 50 {
        eprintln!("USB/IP: refused {what}");
    }
}

/// Completes isochronous transfers at the end of the time they cover.
fn complete_iso(connection: &Connection, device: &dyn UsbDevice, clock: &dyn Clock) {
    let mut pending = connection.pending.lock().unwrap();
    loop {
        if pending.closed {
            return;
        }
        let Some(&Reverse((due, seq))) = pending.due.peek() else {
            pending = connection.changed.wait(pending).unwrap();
            continue;
        };
        let now = clock.now_ns();
        if now < due {
            pending = connection.changed.wait_timeout(pending, Duration::from_nanos(due - now)).unwrap().0;
            continue;
        }
        pending.due.pop();
        let Some(urb) = pending.urbs.remove(&seq) else {
            continue; // unlinked
        };
        drop(pending);

        let mut data = vec![];
        let lengths = if urb.address & 0x80 != 0 {
            let capacity: Vec<usize> = urb.descriptors.iter().map(|d| be32(&d[4..8]) as usize).collect();
            let mut lengths = device.iso_in(urb.address, urb.timing, &capacity, &mut data);
            lengths.resize(capacity.len(), 0);
            lengths
        } else {
            urb.out_lengths
        };
        let mut reply = ret_submit(seq, 0, lengths.iter().sum(), urb.start_frame, urb.descriptors.len() as u32);
        // IN data is sent packed; the client moves each packet to its offset
        reply.extend_from_slice(&data);
        for (descriptor, &length) in urb.descriptors.iter().zip(&lengths) {
            reply.extend_from_slice(&descriptor[0..8]);
            put_u32(&mut reply, length as u32);
            put_u32(&mut reply, 0);
        }
        if connection.send(&reply).is_err() {
            return;
        }
        pending = connection.pending.lock().unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transfers_continue_the_endpoint_timing() {
        let mut next = None;
        let t = schedule(&mut next, 1_000_100, 8, 125_000);
        assert_eq!(t.start_ns, 1_000_000, "aligned to a packet");
        assert_eq!(next, Some(2_000_000));
        let t = schedule(&mut next, 1_200_000, 8, 125_000);
        assert_eq!(t.start_ns, 2_000_000, "submitted ahead");
        let t = schedule(&mut next, 3_000_000 + RESYNC_NS - 1, 8, 125_000);
        assert_eq!(t.start_ns, 3_000_000, "late, but catching up");
        let now = 100_000_000_000;
        let t = schedule(&mut next, now, 8, 125_000);
        assert_eq!(t.start_ns, now, "restarted after a pause");
    }

    #[test]
    fn interfaces_of_first_alternate_settings() {
        let configuration = [
            9, 2, 0, 0, 2, 1, 0, 0x80, 50, //
            9, 4, 0, 0, 0, 1, 1, 0x20, 0, //
            9, 4, 1, 0, 0, 1, 2, 0x20, 0, //
            9, 4, 1, 1, 1, 1, 2, 0x20, 0, //
        ];
        assert_eq!(interfaces(&configuration), vec![[1, 1, 0x20], [1, 2, 0x20]]);
    }
}
