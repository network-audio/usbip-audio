//! Drives the server over TCP as a USB/IP client (usbip-win2) does.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use usbip_audio::uac2::{Uac2Config, Uac2Device, Uac2Streams};
use usbip_audio::{MonotonicClock, Server};

#[derive(Default)]
struct Streams {
    played: Mutex<Vec<(u64, Vec<i32>)>>,
    captured_at: AtomicU64,
}

impl Uac2Streams for Streams {
    fn playback(&self, frame: u64, samples: &[i32]) {
        self.played.lock().unwrap().push((frame, samples.to_vec()));
    }
    fn capture(&self, frame: u64, samples: &mut [i32]) {
        self.captured_at.store(frame, Ordering::Relaxed);
        samples.fill(0x0100);
    }
}

fn device(streams: Arc<Streams>) -> Arc<Uac2Device<Arc<Streams>>> {
    Arc::new(Uac2Device::new(config(false), streams).unwrap())
}

fn config(capture_connector: bool) -> Uac2Config {
    Uac2Config {
        vendor_id: 0x1209,
        product_id: 0x0001,
        bcd_device: 0x0100,
        manufacturer: "Test".into(),
        product: "Test device".into(),
        serial: "0001".into(),
        sample_rate: 48000,
        playback_channels: 2,
        capture_channels: 4,
        playback_name: None,
        capture_name: None,
        capture_connector,
    }
}

fn be32(bytes: &[u8]) -> u32 {
    u32::from_be_bytes(bytes[..4].try_into().unwrap())
}

fn op(stream: &mut TcpStream, code: u16) {
    let mut out = 0x0111u16.to_be_bytes().to_vec();
    out.extend_from_slice(&code.to_be_bytes());
    out.extend_from_slice(&[0; 4]);
    stream.write_all(&out).unwrap();
}

fn read(stream: &mut TcpStream, len: usize) -> Vec<u8> {
    let mut bytes = vec![0; len];
    stream.read_exact(&mut bytes).unwrap();
    bytes
}

#[allow(clippy::too_many_arguments)]
fn submit(
    seq: u32,
    direction: u32,
    ep: u32,
    length: u32,
    packets: u32,
    setup: [u8; 8],
    payload: &[u8],
    iso: &[(u32, u32)],
) -> Vec<u8> {
    let mut out = vec![];
    for value in [1, seq, 0x0001_0001, direction, ep, 0, length, 0, packets, 0] {
        out.extend_from_slice(&u32::to_be_bytes(value));
    }
    out.extend_from_slice(&setup);
    out.extend_from_slice(payload);
    for &(offset, len) in iso {
        for value in [offset, len, 0, 0] {
            out.extend_from_slice(&value.to_be_bytes());
        }
    }
    out
}

fn import(server: &Server, busid: &str) -> TcpStream {
    let mut stream = TcpStream::connect(("127.0.0.1", server.port())).unwrap();
    stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    stream.set_nodelay(true).unwrap();
    op(&mut stream, 0x8003);
    let mut field = busid.as_bytes().to_vec();
    field.resize(32, 0);
    stream.write_all(&field).unwrap();
    let reply = read(&mut stream, 8 + 312);
    assert_eq!(be32(&reply[4..8]), 0, "import status");
    stream
}

#[test]
fn device_list_describes_the_device() {
    let server = Server::start(0, Arc::new(MonotonicClock::new())).unwrap();
    server.add("1-1", device(Default::default()));
    let mut stream = TcpStream::connect(("127.0.0.1", server.port())).unwrap();
    op(&mut stream, 0x8005);
    let header = read(&mut stream, 12);
    assert_eq!(be32(&header[8..12]), 1, "one device");
    let info = read(&mut stream, 312 + 3 * 4);
    assert!(info[256..].starts_with(b"1-1\0"));
    assert_eq!(be32(&info[296..300]), 3, "high speed");
    assert_eq!(u16::from_be_bytes([info[300], info[301]]), 0x1209);
    assert_eq!(&info[306..309], &[0xef, 0x02, 0x01], "interface association class");
    assert_eq!(info[311], 3, "control, playback and capture interfaces");
    assert_eq!(&info[312..315], &[1, 1, 0x20], "AudioControl, UAC2");
    assert_eq!(&info[316..319], &[1, 2, 0x20], "AudioStreaming, UAC2");
}

#[test]
fn control_and_isochronous_transfers() {
    let streams = Arc::new(Streams::default());
    let server = Server::start(0, Arc::new(MonotonicClock::new())).unwrap();
    server.add("1-1", device(streams.clone()));
    let mut stream = import(&server, "1-1");

    // GET_DESCRIPTOR(configuration), first 9 bytes
    stream.write_all(&submit(1, 1, 0, 9, u32::MAX, [0x80, 6, 0, 2, 0, 0, 9, 0], &[], &[])).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[0..4]), be32(&reply[4..8]), be32(&reply[20..24])), (3, 1, 0));
    assert_eq!(be32(&reply[24..28]), 9);
    let descriptor = read(&mut stream, 9);
    assert_eq!(descriptor[1], 2);

    // an unknown request stalls
    stream.write_all(&submit(2, 1, 0, 4, u32::MAX, [0xc0, 0x55, 0, 0, 0, 0, 4, 0], &[], &[])).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!(be32(&reply[20..24]) as i32, -32);

    // 8 microframes (1 ms) of stereo playback: completes after its time, not before
    let packet: Vec<u8> = (0..12i32).flat_map(|s| (s << 8).to_le_bytes()).collect();
    let payload: Vec<u8> = packet.iter().copied().cycle().take(8 * 48).collect();
    let iso: Vec<(u32, u32)> = (0..8).map(|i| (i * 48, 48)).collect();
    let sent = Instant::now();
    stream.write_all(&submit(3, 0, 1, payload.len() as u32, 8, [0; 8], &payload, &iso)).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[4..8]), be32(&reply[20..24]), be32(&reply[24..28])), (3, 0, 384));
    let descriptors = read(&mut stream, 8 * 16);
    assert!(descriptors.chunks(16).all(|d| be32(&d[8..12]) == 48), "each packet's actual length");
    assert!(sent.elapsed() >= Duration::from_micros(500), "paced by the clock: {:?}", sent.elapsed());
    {
        let played = streams.played.lock().unwrap();
        assert_eq!(played.len(), 1);
        assert_eq!(played[0].1.len(), 8 * 12);
        assert_eq!(played[0].1[1], 1 << 8);
    }

    // 4 microframes of 4-channel capture, data packed in the reply
    let iso: Vec<(u32, u32)> = (0..4).map(|i| (i * 96, 96)).collect();
    stream.write_all(&submit(4, 1, 2, 4 * 96, 4, [0; 8], &[], &iso)).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[4..8]), be32(&reply[24..28]), be32(&reply[32..36])), (4, 384, 4));
    let data = read(&mut stream, 384);
    assert!(data.chunks(4).all(|s| s == 0x0100i32.to_le_bytes()));
    let descriptors = read(&mut stream, 4 * 16);
    assert!(descriptors.chunks(16).all(|d| be32(&d[8..12]) == 96));

    // a capture transfer unlinked before its time never completes
    stream.write_all(&submit(5, 1, 2, 8 * 96, 8, [0; 8], &[], &iso.repeat(2))).unwrap();
    let mut unlink = vec![];
    for value in [2, 6, 0x0001_0001, 0, 2, 5] {
        unlink.extend_from_slice(&u32::to_be_bytes(value));
    }
    unlink.resize(48, 0);
    stream.write_all(&unlink).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[0..4]), be32(&reply[4..8]), be32(&reply[20..24]) as i32), (4, 6, -104));
    stream.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
    assert!(stream.read(&mut [0; 1]).is_err(), "no reply for the unlinked transfer");
}

#[test]
fn removing_a_device_closes_its_connection() {
    let server = Server::start(0, Arc::new(MonotonicClock::new())).unwrap();
    server.add("1-1", device(Default::default()));
    let mut stream = import(&server, "1-1");
    server.remove("1-1");
    let mut byte = [0; 1];
    let closed = stream.read(&mut byte);
    assert!(matches!(closed, Ok(0) | Err(_)), "connection closed: {closed:?}");
}

fn unlink(seq: u32, unlinked: u32) -> Vec<u8> {
    let mut out = vec![];
    for value in [2, seq, 0x0001_0001, 0, 3, unlinked] {
        out.extend_from_slice(&u32::to_be_bytes(value));
    }
    out.resize(48, 0);
    out
}

/// GET_CUR of the capture terminal's connector control.
fn connector(stream: &mut TcpStream, seq: u32) -> Vec<u8> {
    stream.write_all(&submit(seq, 1, 0, 6, u32::MAX, [0xa1, 1, 0, 2, 0, 4, 6, 0], &[], &[])).unwrap();
    let reply = read(stream, 48);
    assert_eq!((be32(&reply[4..8]), be32(&reply[20..24])), (seq, 0));
    read(stream, be32(&reply[24..28]) as usize)
}

fn assert_silent(stream: &mut TcpStream) {
    stream.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
    assert!(stream.read(&mut [0; 1]).is_err(), "no reply yet");
    stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
}

/// A connector change, as an AudioControl interrupt: CUR of the capture terminal's connector control.
const CONNECTOR_CHANGED: [u8; 6] = [0, 1, 0, 2, 0, 4];

#[test]
fn interrupt_transfers_wait_for_connector_changes() {
    let device = Arc::new(Uac2Device::new(config(true), Arc::new(Streams::default())).unwrap());
    let server = Server::start(0, Arc::new(MonotonicClock::new())).unwrap();
    server.add("1-1", device.clone());
    let mut stream = import(&server, "1-1");
    assert_eq!(connector(&mut stream, 1), [0; 6], "unplugged until connected");

    // interrupt transfers stay pending until there is a change; usbip-win2 queues several
    for seq in [2, 3] {
        stream.write_all(&submit(seq, 1, 3, 8, 0, [0; 8], &[], &[])).unwrap();
    }
    assert_silent(&mut stream);
    device.set_capture_connected(true);
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[0..4]), be32(&reply[4..8]), be32(&reply[20..24])), (3, 2, 0), "the oldest first");
    assert_eq!(be32(&reply[24..28]), 6);
    assert_eq!(read(&mut stream, 6), CONNECTOR_CHANGED);
    assert_eq!(connector(&mut stream, 4), [4, 0, 0, 0, 0, 0], "all 4 channels connected");
    assert_silent(&mut stream);

    // the transfer still pending reports the next change
    device.set_capture_connected(false);
    let reply = read(&mut stream, 48);
    assert_eq!(be32(&reply[4..8]), 3);
    assert_eq!(read(&mut stream, 6), CONNECTOR_CHANGED);

    // changes while none is pending are reported by the next one, at once
    device.set_capture_connected(true);
    device.set_capture_connected(false);
    stream.write_all(&submit(5, 1, 3, 8, 0, [0; 8], &[], &[])).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!(be32(&reply[4..8]), 5);
    assert_eq!(read(&mut stream, 6), CONNECTOR_CHANGED);
    assert_eq!(connector(&mut stream, 6), [0; 6]);

    // an unlinked interrupt transfer is never completed
    stream.write_all(&submit(7, 1, 3, 8, 0, [0; 8], &[], &[])).unwrap();
    stream.write_all(&unlink(8, 7)).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[0..4]), be32(&reply[4..8]), be32(&reply[20..24]) as i32), (4, 8, -104));
    device.set_capture_connected(true);
    assert_silent(&mut stream);
    stream.write_all(&submit(9, 1, 3, 8, 0, [0; 8], &[], &[])).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!(be32(&reply[4..8]), 9, "the change waited for a transfer");
    assert_eq!(read(&mut stream, 6), CONNECTOR_CHANGED);
}

#[test]
fn interrupt_endpoints_need_a_connector() {
    let server = Server::start(0, Arc::new(MonotonicClock::new())).unwrap();
    server.add("1-1", device(Default::default()));
    let mut stream = import(&server, "1-1");
    stream.write_all(&submit(1, 1, 3, 8, 0, [0; 8], &[], &[])).unwrap();
    let reply = read(&mut stream, 48);
    assert_eq!((be32(&reply[4..8]), be32(&reply[20..24]) as i32), (1, -32));
}
