//! A high-speed USB Audio Class 2.0 device, which Windows binds to `usbaudio2.sys`.
//!
//! It has one fixed internal clock, and up to one playback stream (Windows to the device, an OUT endpoint) and
//! one capture stream (device to Windows, an IN endpoint) of any number of channels. Samples are 24 bits in
//! 32-bit (4-byte) subslots, i.e. `i32`s with the 24 most significant bits valid. Both endpoints are
//! synchronous: every 125 µs microframe carries `sample_rate / 8000` frames, and the [`Server`](crate::Server)
//! completes the transfers by its [`Clock`](crate::Clock), which so is the clock of both streams.

use crate::device::{IsoEndpoint, IsoTiming, Setup, Speed, UsbDevice, EPIPE};
use std::sync::Mutex;

const MICROFRAME_NS: u64 = 125_000;
const SUBSLOT_BYTES: usize = 4;
const BIT_RESOLUTION: u8 = 24;
/// Most bytes a high-speed isochronous endpoint can move per microframe (3 transactions of 1024).
const MAX_HS_ISO_BYTES: usize = 3 * 1024;

const PLAYBACK_EP: u8 = 0x01;
const CAPTURE_EP: u8 = 0x82;
const CLOCK_ID: u8 = 1;
const PLAYBACK_INPUT_TERMINAL: u8 = 2;
const PLAYBACK_OUTPUT_TERMINAL: u8 = 3;
const CAPTURE_INPUT_TERMINAL: u8 = 4;
const CAPTURE_OUTPUT_TERMINAL: u8 = 5;

const STRING_MANUFACTURER: u8 = 1;
const STRING_PRODUCT: u8 = 2;
const STRING_SERIAL: u8 = 3;
const STRING_PLAYBACK: u8 = 4;
const STRING_CAPTURE: u8 = 5;

/// What a [`Uac2Device`] looks like to Windows.
#[derive(Clone, Debug)]
pub struct Uac2Config {
    pub vendor_id: u16,
    pub product_id: u16,
    pub bcd_device: u16,
    pub manufacturer: String,
    /// The device's name, which Windows shows in its endpoints' names
    pub product: String,
    /// Distinguishes devices of the same vendor and product id: Windows remembers each one's settings by it
    pub serial: String,
    /// A multiple of 8000 Hz (e.g. 48000, 96000)
    pub sample_rate: u32,
    /// Channels Windows plays to the device (0 for none). Windows' shared-mode audio engine opens at most 32
    /// channels: applications can only open more in exclusive mode.
    pub playback_channels: u16,
    /// Channels Windows records from the device (0 for none); at most 32 in shared mode, as for playback
    pub capture_channels: u16,
    /// Name of the playback terminal (shown by Windows as the output's name), if any
    pub playback_name: Option<String>,
    /// Name of the capture terminal (shown by Windows as the input's name), if any
    pub capture_name: Option<String>,
}

impl Uac2Config {
    fn frames_per_microframe(&self) -> usize {
        (self.sample_rate / 8000) as usize
    }
    fn packet_bytes(&self, channels: u16) -> usize {
        self.frames_per_microframe() * channels as usize * SUBSLOT_BYTES
    }
    /// Checks the format fits into high-speed isochronous transfers.
    pub fn validate(&self) -> Result<(), String> {
        if self.sample_rate == 0 || !self.sample_rate.is_multiple_of(8000) {
            return Err(format!("sample rate {} is not a multiple of 8000 Hz", self.sample_rate));
        }
        if self.playback_channels == 0 && self.capture_channels == 0 {
            return Err("a device needs playback or capture channels".into());
        }
        for channels in [self.playback_channels, self.capture_channels] {
            if channels > 255 {
                return Err(format!("{channels} channels: at most 255"));
            }
            if self.packet_bytes(channels) > MAX_HS_ISO_BYTES {
                return Err(format!(
                    "{channels} channels at {} Hz need {} bytes per microframe, more than the {MAX_HS_ISO_BYTES} \
                     of a high-speed endpoint",
                    self.sample_rate,
                    self.packet_bytes(channels)
                ));
            }
        }
        Ok(())
    }
}

/// The application's side of a [`Uac2Device`]'s streams. Frame positions are the server clock's time
/// converted to frames at the sample rate (`ns * rate / 10^9`), so they are continuous across transfers.
pub trait Uac2Streams: Send + Sync {
    /// Samples Windows played, interleaved, for the frames starting at `frame`. They arrive ahead of time: by
    /// the length of the transfers Windows queues.
    fn playback(&self, frame: u64, samples: &[i32]);
    /// Fills the interleaved samples Windows records, for the frames starting at `frame`, which have just
    /// passed.
    fn capture(&self, frame: u64, samples: &mut [i32]);
}

/// Lets the application keep a reference to its streams.
impl<T: Uac2Streams + ?Sized> Uac2Streams for std::sync::Arc<T> {
    fn playback(&self, frame: u64, samples: &[i32]) {
        (**self).playback(frame, samples)
    }
    fn capture(&self, frame: u64, samples: &mut [i32]) {
        (**self).capture(frame, samples)
    }
}

#[derive(Default)]
struct State {
    configuration: u8,
    alternates: [u8; 3],
}

pub struct Uac2Device<S: Uac2Streams> {
    config: Uac2Config,
    streams: S,
    state: Mutex<State>,
}

/// Interface numbers: AudioControl is 0, then playback, then capture streaming (each only if it has channels).
fn interface_numbers(config: &Uac2Config) -> (Option<u8>, Option<u8>) {
    let playback = (config.playback_channels > 0).then_some(1);
    let capture = (config.capture_channels > 0).then_some(1 + playback.is_some() as u8);
    (playback, capture)
}

fn channel_config(channels: u16) -> u32 {
    // front left and right for stereo, so Windows treats it as such; other counts are non-spatial
    if channels == 2 {
        0x3
    } else {
        0
    }
}

fn frame_at(ns: u64, rate: u32) -> u64 {
    (ns as u128 * rate as u128 / 1_000_000_000) as u64
}

impl<S: Uac2Streams> Uac2Device<S> {
    pub fn new(config: Uac2Config, streams: S) -> Result<Self, String> {
        config.validate()?;
        Ok(Self { config, streams, state: Default::default() })
    }

    pub fn config(&self) -> &Uac2Config {
        &self.config
    }

    pub fn streams(&self) -> &S {
        &self.streams
    }

    fn string(&self, index: u8) -> Option<&str> {
        let c = &self.config;
        match index {
            STRING_MANUFACTURER => Some(&c.manufacturer),
            STRING_PRODUCT => Some(&c.product),
            STRING_SERIAL => Some(&c.serial),
            STRING_PLAYBACK => c.playback_name.as_deref(),
            STRING_CAPTURE => c.capture_name.as_deref(),
            _ => None,
        }
    }

    fn string_descriptor(&self, index: u8) -> Result<Vec<u8>, i32> {
        if index == 0 {
            return Ok(vec![4, 3, 0x09, 0x04]); // English (US)
        }
        let utf16: Vec<u16> = self.string(index).ok_or(EPIPE)?.encode_utf16().take(126).collect();
        let mut out = vec![(utf16.len() * 2 + 2) as u8, 3];
        for unit in utf16 {
            out.extend_from_slice(&unit.to_le_bytes());
        }
        Ok(out)
    }

    fn device_qualifier(&self) -> Vec<u8> {
        let d = self.device_descriptor();
        vec![10, 6, d[2], d[3], d[4], d[5], d[6], d[7], 1, 0]
    }

    /// Clock source requests: sampling frequency and validity.
    fn clock_request(&self, setup: Setup, data: &[u8]) -> Result<Vec<u8>, i32> {
        let selector = (setup.value >> 8) as u8;
        let rate = self.config.sample_rate;
        match (setup.request_type, setup.request, selector) {
            (0xa1, 0x01, 0x01) => Ok(rate.to_le_bytes().to_vec()), // CUR SAM_FREQ
            (0xa1, 0x02, 0x01) => {
                // RANGE SAM_FREQ: one subrange, min = max = rate
                let mut out = 1u16.to_le_bytes().to_vec();
                for value in [rate, rate, 0] {
                    out.extend_from_slice(&value.to_le_bytes());
                }
                Ok(out)
            }
            (0xa1, 0x01, 0x02) => Ok(vec![1]), // CUR CLOCK_VALID
            (0x21, 0x01, 0x01) if data.len() >= 4 && u32::from_le_bytes(data[..4].try_into().unwrap()) == rate => {
                Ok(vec![])
            }
            _ => Err(EPIPE),
        }
    }
}

fn streaming_interface(out: &mut Vec<u8>, number: u8, terminal: u8, channels: u16, endpoint: u8, packet_bytes: usize) {
    out.extend_from_slice(&[9, 4, number, 0, 0, 0x01, 0x02, 0x20, 0]); // alternate 0: no bandwidth
    out.extend_from_slice(&[9, 4, number, 1, 1, 0x01, 0x02, 0x20, 0]); // alternate 1: streaming
                                                                       // class-specific AS_GENERAL: PCM
    out.extend_from_slice(&[16, 0x24, 0x01, terminal, 0, 0x01, 0x01, 0, 0, 0, channels as u8]);
    out.extend_from_slice(&channel_config(channels).to_le_bytes());
    out.push(0);
    out.extend_from_slice(&[6, 0x24, 0x02, 0x01, SUBSLOT_BYTES as u8, BIT_RESOLUTION]); // FORMAT_TYPE_I
                                                                                        // high bandwidth: up to 3 transactions of at most 1024 bytes per microframe
    let transactions = packet_bytes.div_ceil(1024).max(1);
    let max_packet = (packet_bytes.div_ceil(transactions) | ((transactions - 1) << 11)) as u16;
    out.extend_from_slice(&[7, 5, endpoint, 0x0d]); // isochronous, synchronous, data
    out.extend_from_slice(&max_packet.to_le_bytes());
    out.push(1); // every microframe
    out.extend_from_slice(&[8, 0x25, 0x01, 0, 0, 0, 0, 0]); // class-specific endpoint: no controls, no lock delay
}

fn input_terminal(out: &mut Vec<u8>, id: u8, terminal_type: u16, channels: u16, name: u8) {
    out.extend_from_slice(&[17, 0x24, 0x02, id]);
    out.extend_from_slice(&terminal_type.to_le_bytes());
    out.extend_from_slice(&[0, CLOCK_ID, channels as u8]);
    out.extend_from_slice(&channel_config(channels).to_le_bytes());
    out.extend_from_slice(&[0, 0, 0, name]);
}

fn output_terminal(out: &mut Vec<u8>, id: u8, terminal_type: u16, source: u8, name: u8) {
    out.extend_from_slice(&[12, 0x24, 0x03, id]);
    out.extend_from_slice(&terminal_type.to_le_bytes());
    out.extend_from_slice(&[0, source, CLOCK_ID, 0, 0, name]);
}

impl<S: Uac2Streams> UsbDevice for Uac2Device<S> {
    fn speed(&self) -> Speed {
        Speed::High
    }

    fn device_descriptor(&self) -> Vec<u8> {
        let c = &self.config;
        let mut out = vec![18, 1, 0x00, 0x02, 0xef, 0x02, 0x01, 64]; // USB 2.0, interface association
        out.extend_from_slice(&c.vendor_id.to_le_bytes());
        out.extend_from_slice(&c.product_id.to_le_bytes());
        out.extend_from_slice(&c.bcd_device.to_le_bytes());
        out.extend_from_slice(&[STRING_MANUFACTURER, STRING_PRODUCT, STRING_SERIAL, 1]);
        out
    }

    fn configuration_descriptor(&self) -> Vec<u8> {
        let c = &self.config;
        let (playback, capture) = interface_numbers(c);
        let interfaces = 1 + playback.is_some() as u8 + capture.is_some() as u8;
        let name = |s: &Option<String>, index| if s.is_some() { index } else { 0 };

        let mut ac = vec![];
        ac.extend_from_slice(&[8, 0x24, 0x0a, CLOCK_ID, 0x01, 0x05, 0, 0]); // internal fixed clock, read-only
        if playback.is_some() {
            input_terminal(&mut ac, PLAYBACK_INPUT_TERMINAL, 0x0101, c.playback_channels, 0); // USB streaming
            let label = name(&c.playback_name, STRING_PLAYBACK);
            output_terminal(&mut ac, PLAYBACK_OUTPUT_TERMINAL, 0x0301, PLAYBACK_INPUT_TERMINAL, label);
            // speaker
        }
        if capture.is_some() {
            let label = name(&c.capture_name, STRING_CAPTURE);
            input_terminal(&mut ac, CAPTURE_INPUT_TERMINAL, 0x0201, c.capture_channels, label); // microphone
            output_terminal(&mut ac, CAPTURE_OUTPUT_TERMINAL, 0x0101, CAPTURE_INPUT_TERMINAL, 0);
        }
        let ac_total = (9 + ac.len()) as u16;

        let mut out = vec![9, 2, 0, 0, interfaces, 1, 0, 0x80, 50];
        out.extend_from_slice(&[8, 0x0b, 0, interfaces, 0x01, 0x00, 0x20, STRING_PRODUCT]); // association
        out.extend_from_slice(&[9, 4, 0, 0, 0, 0x01, 0x01, 0x20, 0]); // AudioControl
        out.extend_from_slice(&[9, 0x24, 0x01, 0x00, 0x02, 0x08]); // AC header, ADC 2.0, I/O box
        out.extend_from_slice(&ac_total.to_le_bytes());
        out.push(0);
        out.extend_from_slice(&ac);
        if let Some(number) = playback {
            let bytes = c.packet_bytes(c.playback_channels);
            streaming_interface(&mut out, number, PLAYBACK_INPUT_TERMINAL, c.playback_channels, PLAYBACK_EP, bytes);
        }
        if let Some(number) = capture {
            let bytes = c.packet_bytes(c.capture_channels);
            streaming_interface(&mut out, number, CAPTURE_OUTPUT_TERMINAL, c.capture_channels, CAPTURE_EP, bytes);
        }
        let total = out.len() as u16;
        out[2..4].copy_from_slice(&total.to_le_bytes());
        out
    }

    fn control(&self, setup: Setup, data: &[u8]) -> Result<Vec<u8>, i32> {
        let mut state = self.state.lock().unwrap();
        let interface = setup.index as u8;
        match (setup.request_type, setup.request) {
            (0x80, 6) => {
                let descriptor = match (setup.value >> 8) as u8 {
                    1 => self.device_descriptor(),
                    2 => self.configuration_descriptor(),
                    3 => self.string_descriptor(setup.value as u8)?,
                    6 => self.device_qualifier(),
                    _ => return Err(EPIPE),
                };
                Ok(descriptor[..descriptor.len().min(setup.length as usize)].to_vec())
            }
            (0x00, 5) => Ok(vec![]), // SET_ADDRESS: virtual
            (0x00, 9) if setup.value <= 1 => {
                state.configuration = setup.value as u8;
                Ok(vec![])
            }
            (0x80, 8) => Ok(vec![state.configuration]),
            (0x01, 11) if (interface as usize) < state.alternates.len() && setup.value <= 1 => {
                state.alternates[interface as usize] = setup.value as u8;
                Ok(vec![])
            }
            (0x81, 10) if (interface as usize) < state.alternates.len() => {
                Ok(vec![state.alternates[interface as usize]])
            }
            (0x80, 0) | (0x81, 0) | (0x82, 0) => Ok(vec![0, 0]), // GET_STATUS
            (0x02, 1) => Ok(vec![]),                             // CLEAR_FEATURE(ENDPOINT_HALT)
            (0x21 | 0xa1, _) if interface == 0 && (setup.index >> 8) as u8 == CLOCK_ID => {
                drop(state);
                self.clock_request(setup, data)
            }
            _ => Err(EPIPE),
        }
    }

    fn iso_endpoint(&self, address: u8) -> Option<IsoEndpoint> {
        let (playback, capture) = interface_numbers(&self.config);
        let exists = match address {
            PLAYBACK_EP => playback.is_some(),
            CAPTURE_EP => capture.is_some(),
            _ => false,
        };
        exists.then_some(IsoEndpoint { packet_ns: MICROFRAME_NS })
    }

    fn iso_out(&self, address: u8, timing: IsoTiming, packets: &[&[u8]]) {
        if address != PLAYBACK_EP {
            return;
        }
        let channels = self.config.playback_channels as usize;
        let frames = self.config.frames_per_microframe();
        let start = frame_at(timing.start_ns, self.config.sample_rate);
        let mut samples = Vec::with_capacity(packets.len() * frames * channels);
        for packet in packets {
            // a short packet is padded with silence, so positions stay continuous
            let (whole, _) = packet.as_chunks::<SUBSLOT_BYTES>();
            let count = whole.len().min(frames * channels) / channels * channels;
            samples.extend(whole[..count].iter().map(|b| i32::from_le_bytes(*b)));
            samples.resize(samples.len() + frames * channels - count, 0);
        }
        self.streams.playback(start, &samples);
    }

    fn iso_in(&self, address: u8, timing: IsoTiming, capacity: &[usize], data: &mut Vec<u8>) -> Vec<usize> {
        if address != CAPTURE_EP {
            return vec![0; capacity.len()];
        }
        let channels = self.config.capture_channels as usize;
        let frames = self.config.frames_per_microframe();
        let start = frame_at(timing.start_ns, self.config.sample_rate);
        let mut samples = vec![0i32; capacity.len() * frames * channels];
        self.streams.capture(start, &mut samples);
        let packet_bytes = frames * channels * SUBSLOT_BYTES;
        capacity
            .iter()
            .zip(samples.chunks(frames * channels))
            .map(|(&capacity, packet)| {
                let len = packet_bytes.min(capacity / (channels * SUBSLOT_BYTES) * channels * SUBSLOT_BYTES);
                for sample in &packet[..len / SUBSLOT_BYTES] {
                    data.extend_from_slice(&sample.to_le_bytes());
                }
                len
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[derive(Default)]
    struct Recorder {
        played: Mutex<Vec<(u64, Vec<i32>)>>,
    }

    impl Uac2Streams for Recorder {
        fn playback(&self, frame: u64, samples: &[i32]) {
            self.played.lock().unwrap().push((frame, samples.to_vec()));
        }
        fn capture(&self, frame: u64, samples: &mut [i32]) {
            for (i, s) in samples.iter_mut().enumerate() {
                *s = (frame as i32 + i as i32) << 8;
            }
        }
    }

    fn config(playback: u16, capture: u16) -> Uac2Config {
        Uac2Config {
            vendor_id: 0x1209,
            product_id: 0x0001,
            bcd_device: 0x0100,
            manufacturer: "Test".into(),
            product: "Test device".into(),
            serial: "0001".into(),
            sample_rate: 48000,
            playback_channels: playback,
            capture_channels: capture,
            playback_name: Some("Out".into()),
            capture_name: None,
        }
    }

    /// Walks a configuration descriptor: every descriptor's length must be in bounds and the total must match.
    fn descriptors(configuration: &[u8]) -> Vec<&[u8]> {
        let total = u16::from_le_bytes([configuration[2], configuration[3]]) as usize;
        assert_eq!(total, configuration.len());
        let mut found = vec![];
        let mut at = 0;
        while at < configuration.len() {
            let len = configuration[at] as usize;
            assert!(len >= 2 && at + len <= configuration.len(), "descriptor at {at} has length {len}");
            found.push(&configuration[at..at + len]);
            at += len;
        }
        found
    }

    #[test]
    fn descriptors_are_well_formed() {
        for (playback, capture) in [(2, 2), (8, 0), (0, 8), (64, 64)] {
            let device = Uac2Device::new(config(playback, capture), Recorder::default()).unwrap();
            let configuration = device.configuration_descriptor();
            let all = descriptors(&configuration);
            let interfaces = all.iter().filter(|d| d[1] == 4 && d[3] == 0).count();
            assert_eq!(configuration[4] as usize, interfaces);
            // the AudioControl header's total covers the class-specific AudioControl descriptors
            let header = all.iter().position(|d| d[1] == 0x24 && d[2] == 0x01).unwrap();
            let ac_total = u16::from_le_bytes([all[header][6], all[header][7]]) as usize;
            let ac_len: usize = all[header..].iter().take_while(|d| d[1] == 0x24).map(|d| d.len()).sum();
            assert_eq!(ac_total, ac_len);
            let endpoints: Vec<_> = all.iter().filter(|d| d[1] == 5).collect();
            assert_eq!(endpoints.len(), (playback > 0) as usize + (capture > 0) as usize);
            for endpoint in endpoints {
                let max = u16::from_le_bytes([endpoint[4], endpoint[5]]) as usize;
                let (size, transactions) = (max & 0x7ff, (max >> 11) + 1);
                assert!(size <= 1024 && transactions <= 3);
                let channels = if endpoint[2] == PLAYBACK_EP { playback } else { capture } as usize;
                assert!(size * transactions >= 6 * channels * 4);
            }
        }
        assert_eq!(Uac2Device::new(config(2, 2), Recorder::default()).unwrap().device_descriptor().len(), 18);
    }

    #[test]
    fn too_many_channels_are_refused() {
        assert!(config(130, 0).validate().is_err());
        assert!(config(128, 128).validate().is_ok(), "exactly 3 transactions of 1024 bytes");
        assert!(config(0, 0).validate().is_err());
        assert!(config(64, 64).validate().is_ok());
    }

    #[test]
    fn clock_reports_the_sample_rate() {
        let device = Uac2Device::new(config(2, 2), Recorder::default()).unwrap();
        let setup = |request_type, request, selector: u16, length| Setup {
            request_type,
            request,
            value: selector << 8,
            index: (CLOCK_ID as u16) << 8,
            length,
        };
        assert_eq!(device.control(setup(0xa1, 1, 1, 4), &[]).unwrap(), 48000u32.to_le_bytes());
        let range = device.control(setup(0xa1, 2, 1, 14), &[]).unwrap();
        assert_eq!(range.len(), 14);
        assert_eq!(device.control(setup(0xa1, 1, 2, 1), &[]).unwrap(), vec![1]);
        assert!(device.control(setup(0x21, 1, 1, 4), &44100u32.to_le_bytes()).is_err());
    }

    #[test]
    fn playback_frames_are_positioned_by_time() {
        let device = Uac2Device::new(config(2, 0), Recorder::default()).unwrap();
        let packet: Vec<u8> = (0..12i32).flat_map(|s| s.to_le_bytes()).collect();
        let short = &packet[..16]; // 2 of 6 frames
        let timing = IsoTiming { start_ns: 1_000_000, packet_ns: MICROFRAME_NS };
        device.iso_out(PLAYBACK_EP, timing, &[&packet, short]);
        let played = device.streams().played.lock().unwrap();
        assert_eq!(played[0].0, 48, "1 ms in");
        assert_eq!(played[0].1.len(), 24);
        assert_eq!(&played[0].1[12..16], &[0, 1, 2, 3]);
        assert!(played[0].1[16..].iter().all(|&s| s == 0), "padded with silence");
    }

    #[test]
    fn capture_fills_whole_frames_up_to_capacity() {
        let device = Uac2Device::new(config(0, 2), Recorder::default()).unwrap();
        let timing = IsoTiming { start_ns: 0, packet_ns: MICROFRAME_NS };
        let mut data = vec![];
        let lengths = device.iso_in(CAPTURE_EP, timing, &[48, 20, 1000], &mut data);
        assert_eq!(lengths, vec![48, 16, 48]);
        assert_eq!(data.len(), 112);
        let second_packet_first = i32::from_le_bytes(data[48..52].try_into().unwrap());
        assert_eq!(second_packet_first, 12 << 8, "positions continue across packets");
    }
}
