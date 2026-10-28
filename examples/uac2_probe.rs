//! Exports test devices and checks Windows plays and records through them (see the Windows CI job).
//!
//! - `uac2_probe serve --devices 2x2,8x8,2x2@44100 --seconds 120 [--attach] [--stop-file F] [--expect]`:
//!   exports one device per `<playback>x<capture>[@<rate>]` (48 kHz by default), named `Probe <spec>`.
//!   Capture channels carry a 1 kHz sine. With `--expect`, exits with an error unless every device's playback
//!   channels received audio and its capture was read.
//! - Windows only: `uac2_probe list`, `uac2_probe render-test --endpoint NAME [--exclusive]`,
//!   `uac2_probe capture-test --endpoint NAME [--exclusive]`.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use usbip_audio::uac2::{Uac2Config, Uac2Device, Uac2Streams};
use usbip_audio::{usbip_exe, MonotonicClock, Server};

const DEFAULT_RATE: u32 = 48000;
const CAPTURE_HZ: f64 = 1000.0;

#[derive(Default)]
struct Stats {
    playback_frames: AtomicU64,
    /// per playback channel: largest sample, sign changes
    channels: Mutex<Vec<(i64, u64, bool)>>,
    capture_frames: AtomicU64,
    last_playback_frame: AtomicU64,
    playback_gaps: AtomicU64,
}

struct Probe {
    playback_channels: usize,
    capture_channels: usize,
    rate: u32,
    stats: Stats,
}

impl Uac2Streams for Probe {
    fn playback(&self, frame: u64, samples: &[i32]) {
        let frames = samples.len() / self.playback_channels;
        let last = self.stats.last_playback_frame.swap(frame + frames as u64, Ordering::Relaxed);
        if last != 0 && last != frame {
            self.stats.playback_gaps.fetch_add(1, Ordering::Relaxed);
        }
        self.stats.playback_frames.fetch_add(frames as u64, Ordering::Relaxed);
        let mut channels = self.stats.channels.lock().unwrap();
        channels.resize(self.playback_channels, (0, 0, false));
        for frame in samples.chunks(self.playback_channels) {
            for (stat, &s) in channels.iter_mut().zip(frame) {
                stat.0 = stat.0.max((s as i64).abs());
                let positive = s > 0;
                if positive != stat.2 && s != 0 {
                    stat.1 += 1;
                    stat.2 = positive;
                }
            }
        }
    }

    fn capture(&self, frame: u64, samples: &mut [i32]) {
        for (i, chunk) in samples.chunks_mut(self.capture_channels).enumerate() {
            let t = (frame + i as u64) as f64 / self.rate as f64;
            let value = ((t * CAPTURE_HZ * std::f64::consts::TAU).sin() * 0.25 * i32::MAX as f64) as i32;
            chunk.fill(value & !0xff);
        }
        self.stats.capture_frames.fetch_add((samples.len() / self.capture_channels) as u64, Ordering::Relaxed);
    }
}

fn arg<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).map(|s| s.as_str())
}

fn serve(args: &[String]) -> Result<(), String> {
    let specs = arg(args, "--devices").unwrap_or("2x2");
    let seconds: u64 = arg(args, "--seconds").unwrap_or("60").parse().map_err(|_| "--seconds")?;
    let port: u16 = arg(args, "--port").unwrap_or("0").parse().map_err(|_| "--port")?;
    let stop_file = arg(args, "--stop-file").map(std::path::PathBuf::from);
    let server = Server::start(port, Arc::new(MonotonicClock::new())).map_err(|e| e.to_string())?;
    println!("USB/IP server on port {}", server.port());

    let mut devices = BTreeMap::new();
    for (index, spec) in specs.split(',').enumerate() {
        let (channels, rate) = spec.split_once('@').unwrap_or((spec, ""));
        let rate: u32 = if rate.is_empty() { DEFAULT_RATE } else { rate.parse().map_err(|_| "rate")? };
        let (playback, capture) = channels.split_once('x').ok_or(format!("bad device {spec:?}"))?;
        let (playback, capture): (u16, u16) =
            (playback.parse().map_err(|_| "channels")?, capture.parse().map_err(|_| "channels")?);
        let config = Uac2Config {
            vendor_id: 0x1209,
            product_id: 0x0001, // pid.codes test PID
            bcd_device: 0x0100,
            manufacturer: "usbip-audio".into(),
            product: format!("Probe {spec}"),
            serial: format!("probe-{spec}"),
            sample_rate: rate,
            playback_channels: playback,
            capture_channels: capture,
            playback_name: None,
            capture_name: None,
        };
        let probe = Arc::new(Probe {
            playback_channels: playback as usize,
            capture_channels: capture as usize,
            rate,
            stats: Default::default(),
        });
        let device = Arc::new(Uac2Device::new(config, probe.clone())?);
        let busid = format!("1-{}", index + 1);
        server.add(&busid, device);
        devices.insert(spec.to_owned(), (busid, probe, None::<u32>));
    }

    let usbip = args.iter().any(|a| a == "--attach").then(|| usbip_exe::find().ok_or("usbip.exe not found"));
    let usbip = usbip.transpose()?;
    if let Some(usbip) = &usbip {
        for (spec, (busid, _, attached)) in devices.iter_mut() {
            let port = usbip_exe::attach(usbip, server.port(), busid)?;
            println!("attached {spec} ({busid}) to usbip port {port}");
            *attached = Some(port);
        }
    }
    println!("ready");

    let started = Instant::now();
    while started.elapsed() < Duration::from_secs(seconds) && !stop_file.as_ref().is_some_and(|f| f.exists()) {
        std::thread::sleep(Duration::from_millis(200));
    }

    let mut failures = vec![];
    for (spec, (_, probe, attached)) in &devices {
        let stats = &probe.stats;
        let (playback, capture) =
            (stats.playback_frames.load(Ordering::Relaxed), stats.capture_frames.load(Ordering::Relaxed));
        let channels = stats.channels.lock().unwrap().clone();
        let silent: Vec<usize> =
            channels.iter().enumerate().filter(|(_, c)| c.0 < 1 << 20).map(|(i, _)| i + 1).collect();
        println!(
            "{spec}: playback {playback} frames ({} s), {} discontinuities, capture {capture} frames ({} s)",
            playback / probe.rate as u64,
            stats.playback_gaps.load(Ordering::Relaxed),
            capture / probe.rate as u64
        );
        for (i, (peak, crossings, _)) in channels.iter().enumerate() {
            let hz = *crossings as f64 / 2.0 / (playback.max(1) as f64 / probe.rate as f64);
            println!("  playback channel {}: peak {:.3}, ~{hz:.0} Hz", i + 1, *peak as f64 / i32::MAX as f64);
        }
        if probe.playback_channels > 0 && (channels.len() < probe.playback_channels || !silent.is_empty()) {
            failures
                .push(format!("{spec}: playback channels without audio: {silent:?} of {}", probe.playback_channels));
        }
        if probe.capture_channels > 0 && capture < probe.rate as u64 {
            failures.push(format!("{spec}: capture read for less than a second"));
        }
        if let (Some(usbip), Some(port)) = (&usbip, attached) {
            if let Err(e) = usbip_exe::detach(usbip, *port) {
                eprintln!("{e}");
            }
        }
    }
    if args.iter().any(|a| a == "--expect") && !failures.is_empty() {
        return Err(failures.join("\n"));
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    #[cfg(windows)]
    let exclusive = args.iter().any(|a| a == "--exclusive");
    let result = match args.first().map(|s| s.as_str()) {
        Some("serve") => serve(&args),
        #[cfg(windows)]
        Some("list") => wasapi::list(),
        #[cfg(windows)]
        Some("render-test") => wasapi::render_test(arg(&args, "--endpoint").unwrap_or_default(), exclusive),
        #[cfg(windows)]
        Some("capture-test") => wasapi::capture_test(arg(&args, "--endpoint").unwrap_or_default(), exclusive),
        _ => Err("usage: uac2_probe serve|list|render-test|capture-test ...".into()),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

#[cfg(windows)]
mod wasapi {
    use std::time::{Duration, Instant};
    use windows::core::GUID;
    use windows::Win32::Devices::FunctionDiscovery::PKEY_Device_FriendlyName;
    use windows::Win32::Media::Audio::{
        eCapture, eRender, EDataFlow, IAudioCaptureClient, IAudioClient, IAudioRenderClient, IMMDevice,
        IMMDeviceEnumerator, MMDeviceEnumerator, AUDCLNT_SHAREMODE_EXCLUSIVE, AUDCLNT_SHAREMODE_SHARED,
        DEVICE_STATE_ACTIVE, WAVEFORMATEX, WAVEFORMATEXTENSIBLE, WAVEFORMATEXTENSIBLE_0,
    };
    use windows::Win32::System::Com::{CoCreateInstance, CoInitializeEx, CLSCTX_ALL, COINIT_MULTITHREADED, STGM_READ};

    const FLOAT: GUID = GUID::from_u128(0x00000003_0000_0010_8000_00aa00389b71);

    fn e(error: windows::core::Error) -> String {
        error.to_string()
    }

    fn at(call: &'static str) -> impl Fn(windows::core::Error) -> String {
        move |error| format!("{call}: {error}")
    }

    fn endpoints(flow: EDataFlow) -> Result<Vec<(String, IMMDevice)>, String> {
        unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) }.ok().map_err(e)?;
        let enumerator: IMMDeviceEnumerator =
            unsafe { CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL) }.map_err(e)?;
        let collection = unsafe { enumerator.EnumAudioEndpoints(flow, DEVICE_STATE_ACTIVE) }.map_err(e)?;
        let mut found = vec![];
        for index in 0..unsafe { collection.GetCount() }.map_err(e)? {
            let device = unsafe { collection.Item(index) }.map_err(e)?;
            let store = unsafe { device.OpenPropertyStore(STGM_READ) }.map_err(e)?;
            let name = unsafe { store.GetValue(&PKEY_Device_FriendlyName) }.map_err(e)?.to_string();
            found.push((name, device));
        }
        Ok(found)
    }

    fn endpoint(flow: EDataFlow, name: &str) -> Result<(String, IMMDevice), String> {
        let all = endpoints(flow)?;
        let names: Vec<String> = all.iter().map(|(n, _)| n.clone()).collect();
        all.into_iter().find(|(n, _)| n.contains(name)).ok_or(format!("no endpoint containing {name:?} in {names:?}"))
    }

    /// The shared-mode mix format: channels, and whether it is 32-bit float.
    fn mix_format(client: &IAudioClient) -> Result<(*mut WAVEFORMATEX, usize, bool), String> {
        let format = unsafe { client.GetMixFormat() }.map_err(e)?;
        let f = unsafe { format.read_unaligned() };
        let (tag, bits, channels) = (f.wFormatTag, f.wBitsPerSample, f.nChannels);
        let sub_format =
            unsafe { std::ptr::addr_of!((*(format as *const WAVEFORMATEXTENSIBLE)).SubFormat).read_unaligned() };
        let float = tag == 3 || (tag == 0xfffe && sub_format == FLOAT);
        if !float || bits != 32 {
            return Err(format!("mix format is not 32-bit float: tag {tag:#x}, {bits} bits"));
        }
        Ok((format, channels as usize, true))
    }

    pub fn list() -> Result<(), String> {
        for (flow, label) in [(eRender, "render"), (eCapture, "capture")] {
            for (name, device) in endpoints(flow)? {
                let client: IAudioClient = unsafe { device.Activate(CLSCTX_ALL, None) }.map_err(e)?;
                let format = unsafe { client.GetMixFormat() }.map_err(e)?;
                let f = unsafe { format.read_unaligned() };
                let (channels, rate, bits) = (f.nChannels, f.nSamplesPerSec, f.wBitsPerSample);
                println!("{label}: {name}: {channels} channels, {rate} Hz, {bits} bits");
            }
        }
        Ok(())
    }

    const PCM: GUID = GUID::from_u128(0x00000001_0000_0010_8000_00aa00389b71);

    /// An initialised stream: shared mode in the mix format (32-bit float), or exclusive mode in the
    /// device's own format (24 bits in 32-bit integers).
    struct Stream {
        client: IAudioClient,
        channels: usize,
        rate: f64,
        exclusive: bool,
    }

    impl Stream {
        fn open(device: &IMMDevice, exclusive: bool) -> Result<Self, String> {
            let client: IAudioClient = unsafe { device.Activate(CLSCTX_ALL, None) }.map_err(at("Activate"))?;
            let mix = unsafe { client.GetMixFormat() }.map_err(at("GetMixFormat"))?;
            let m = unsafe { mix.read_unaligned() };
            let (channels, rate) = (m.nChannels, m.nSamplesPerSec);
            if exclusive {
                let format = WAVEFORMATEXTENSIBLE {
                    Format: WAVEFORMATEX {
                        wFormatTag: 0xfffe,
                        nChannels: channels,
                        nSamplesPerSec: rate,
                        nAvgBytesPerSec: rate * 4 * channels as u32,
                        nBlockAlign: 4 * channels,
                        wBitsPerSample: 32,
                        cbSize: 22,
                    },
                    Samples: WAVEFORMATEXTENSIBLE_0 { wValidBitsPerSample: 24 },
                    dwChannelMask: if channels == 2 { 3 } else { 0 },
                    SubFormat: PCM,
                };
                let (mut default, mut minimum) = (0i64, 0i64);
                unsafe { client.GetDevicePeriod(Some(&mut default), Some(&mut minimum)) }
                    .map_err(at("GetDevicePeriod"))?;
                let format = &format as *const WAVEFORMATEXTENSIBLE as *const WAVEFORMATEX;
                unsafe { client.Initialize(AUDCLNT_SHAREMODE_EXCLUSIVE, 0, 1_000_000, default, format, None) }
                    .map_err(at("Initialize (exclusive)"))?;
            } else {
                mix_format(&client)?;
                unsafe { client.Initialize(AUDCLNT_SHAREMODE_SHARED, 0, 1_000_000, 0, mix, None) }
                    .map_err(at("Initialize"))?;
            }
            Ok(Self { client, channels: channels as usize, rate: rate as f64, exclusive })
        }

        fn mode(&self) -> &'static str {
            if self.exclusive {
                "exclusive"
            } else {
                "shared"
            }
        }
    }

    pub fn render_test(name: &str, exclusive: bool) -> Result<(), String> {
        let (name, device) = endpoint(eRender, name)?;
        let stream = Stream::open(&device, exclusive)?;
        let (client, channels) = (&stream.client, stream.channels);
        let render: IAudioRenderClient = unsafe { client.GetService() }.map_err(at("GetService"))?;
        let buffer = unsafe { client.GetBufferSize() }.map_err(at("GetBufferSize"))?;
        unsafe { client.Start() }.map_err(at("Start"))?;
        let started = Instant::now();
        let mut frame = 0u64;
        while started.elapsed() < Duration::from_secs(4) {
            let available = buffer - unsafe { client.GetCurrentPadding() }.map_err(at("GetCurrentPadding"))?;
            if available > 0 {
                let data = unsafe { render.GetBuffer(available) }.map_err(at("GetBuffer"))?;
                for i in 0..available as usize {
                    let value = (frame as f64 * 440.0 * std::f64::consts::TAU / stream.rate).sin() * 0.5;
                    for ch in 0..channels {
                        let at = i * channels + ch;
                        unsafe {
                            if stream.exclusive {
                                *(data as *mut i32).add(at) = ((value * i32::MAX as f64) as i32) & !0xff;
                            } else {
                                *(data as *mut f32).add(at) = value as f32;
                            }
                        }
                    }
                    frame += 1;
                }
                unsafe { render.ReleaseBuffer(available, 0) }.map_err(at("ReleaseBuffer"))?;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        unsafe { client.Stop() }.map_err(at("Stop"))?;
        println!("rendered {frame} frames of 440 Hz to {name} ({channels} channels, {})", stream.mode());
        Ok(())
    }

    pub fn capture_test(name: &str, exclusive: bool) -> Result<(), String> {
        let (name, device) = endpoint(eCapture, name)?;
        let stream = Stream::open(&device, exclusive)?;
        let (client, channels, rate) = (&stream.client, stream.channels, stream.rate);
        let capture: IAudioCaptureClient = unsafe { client.GetService() }.map_err(at("GetService"))?;
        unsafe { client.Start() }.map_err(at("Start"))?;
        // per channel: sum of squares, sign changes, last sign
        let mut stats = vec![(0f64, 0u64, false); channels];
        let mut frames = 0u64;
        let started = Instant::now();
        while started.elapsed() < Duration::from_secs(3) {
            loop {
                let mut data = std::ptr::null_mut();
                let (mut count, mut flags) = (0u32, 0u32);
                unsafe { capture.GetBuffer(&mut data, &mut count, &mut flags, None, None) }.map_err(at("GetBuffer"))?;
                if count == 0 {
                    break;
                }
                // skip the first half second: the stream starting up
                if started.elapsed() > Duration::from_millis(500) {
                    for i in 0..count as usize * channels {
                        let s = unsafe {
                            if stream.exclusive {
                                *(data as *const i32).add(i) as f64 / i32::MAX as f64
                            } else {
                                *(data as *const f32).add(i) as f64
                            }
                        };
                        let stat = &mut stats[i % channels];
                        stat.0 += s * s;
                        let positive = s > 0.0;
                        if positive != stat.2 && s != 0.0 {
                            stat.1 += 1;
                            stat.2 = positive;
                        }
                    }
                    frames += count as u64;
                }
                unsafe { capture.ReleaseBuffer(count) }.map_err(at("ReleaseBuffer"))?;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        unsafe { client.Stop() }.map_err(at("Stop"))?;
        println!("captured {frames} frames from {name} ({channels} channels, {})", stream.mode());
        if frames < rate as u64 {
            return Err(format!("captured only {frames} frames"));
        }
        let mut bad = vec![];
        for (i, (squares, crossings, _)) in stats.iter().enumerate() {
            let rms = (squares / frames as f64).sqrt();
            let hz = *crossings as f64 / 2.0 / (frames as f64 / rate);
            if i < 2 || i + 1 == channels {
                println!("  channel {}: rms {rms:.3}, ~{hz:.0} Hz", i + 1);
            }
            if rms < 0.05 || (hz - 1000.0).abs() > 50.0 {
                bad.push(i + 1);
            }
        }
        if !bad.is_empty() {
            return Err(format!("channels without the 1 kHz test signal: {bad:?}"));
        }
        Ok(())
    }
}
