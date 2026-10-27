//! A full-speed USB Audio Class 1.0 speaker (44.1 kHz, stereo, 16-bit), which Windows binds to
//! `usbaudio.sys`, with Feature Unit mute and volume (master and per channel) controls.

use crate::device::{IsoEndpoint, IsoTiming, Setup, Speed, UsbDevice, EPIPE};
use std::sync::Mutex;

const FRAME_NS: u64 = 1_000_000;
const SPEAKER_EP: u8 = 0x01;
const FEATURE_UNIT: u16 = 0x0200; // entity 2, interface 0

/// Windows' volume and mute settings for the speaker.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Volume {
    /// Master volume, 0 to 1
    pub scalar: f32,
    pub muted: bool,
    /// Left and right channel gains, 0 to 1
    pub channels: [f32; 2],
}

impl Default for Volume {
    fn default() -> Self {
        Self { scalar: 0.5, muted: false, channels: [1.0, 1.0] }
    }
}

#[derive(Clone, Debug)]
pub struct Uac1SpeakerConfig {
    pub vendor_id: u16,
    pub product_id: u16,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
    /// Reported in the AudioStreaming descriptor's `bDelay`, in milliseconds
    pub delay_ms: u8,
}

/// The application's side of a [`Uac1Speaker`].
pub trait Uac1SpeakerStreams: Send + Sync {
    /// 16-bit little-endian stereo PCM at 44.1 kHz, as Windows sent it (one transfer at a time, ahead of its time).
    fn pcm(&self, data: &[u8]);
    /// Windows changed the volume or mute setting.
    fn volume(&self, volume: Volume);
}

impl<T: Uac1SpeakerStreams + ?Sized> Uac1SpeakerStreams for std::sync::Arc<T> {
    fn pcm(&self, data: &[u8]) {
        (**self).pcm(data)
    }
    fn volume(&self, volume: Volume) {
        (**self).volume(volume)
    }
}

#[derive(Default)]
struct State {
    configuration: u8,
    alternate: u8,
    volume: Volume,
}

pub struct Uac1Speaker<S: Uac1SpeakerStreams> {
    config: Uac1SpeakerConfig,
    streams: S,
    state: Mutex<State>,
}

impl<S: Uac1SpeakerStreams> Uac1Speaker<S> {
    pub fn new(config: Uac1SpeakerConfig, streams: S) -> Self {
        Self { config, streams, state: Default::default() }
    }

    pub fn streams(&self) -> &S {
        &self.streams
    }

    pub fn volume(&self) -> Volume {
        self.state.lock().unwrap().volume
    }

    fn string_descriptor(&self, index: u8) -> Result<Vec<u8>, i32> {
        if index == 0 {
            return Ok(vec![4, 3, 0x09, 0x04]);
        }
        let value = match index {
            1 => &self.config.manufacturer,
            2 => &self.config.product,
            3 => &self.config.serial,
            _ => return Err(EPIPE),
        };
        let utf16: Vec<u16> = value.encode_utf16().take(126).collect();
        let mut out = vec![(utf16.len() * 2 + 2) as u8, 3];
        for unit in utf16 {
            out.extend_from_slice(&unit.to_le_bytes());
        }
        Ok(out)
    }
}

fn db_to_scalar(payload: &[u8]) -> f32 {
    let db = i16::from_le_bytes([payload[0], payload[1]]) as f32 / 256.0;
    10f32.powf(db / 20.0).clamp(0.0, 1.0)
}

fn scalar_to_db(scalar: f32) -> Vec<u8> {
    let db = if scalar <= 0.001 { -96.0 } else { 20.0 * scalar.log10() };
    ((db.clamp(-96.0, 0.0) * 256.0) as i16).to_le_bytes().to_vec()
}

impl<S: Uac1SpeakerStreams> UsbDevice for Uac1Speaker<S> {
    fn speed(&self) -> Speed {
        Speed::Full
    }

    fn device_descriptor(&self) -> Vec<u8> {
        let mut out = vec![18, 1, 0x00, 0x02, 0, 0, 0, 64];
        out.extend_from_slice(&self.config.vendor_id.to_le_bytes());
        out.extend_from_slice(&self.config.product_id.to_le_bytes());
        out.extend_from_slice(&[0x00, 0x01, 1, 2, 3, 1]);
        out
    }

    fn configuration_descriptor(&self) -> Vec<u8> {
        let mut out = vec![
            9,
            2,
            0,
            0,
            2,
            1,
            0,
            0x80,
            50, // configuration
            9,
            4,
            0,
            0,
            0,
            1,
            1,
            0,
            0, // AudioControl interface
            9,
            0x24,
            1,
            0,
            1,
            40,
            0,
            1,
            1, // AC header, streaming interface 1
            12,
            0x24,
            2,
            1,
            0x01,
            0x01,
            0,
            2,
            3,
            0,
            0,
            0, // USB-streaming input terminal
            10,
            0x24,
            6,
            2,
            1,
            1,
            3,
            2,
            2,
            0, // mute/volume Feature Unit
            9,
            0x24,
            3,
            3,
            0x01,
            0x03,
            0,
            2,
            0, // speaker output terminal
            9,
            4,
            1,
            0,
            0,
            1,
            2,
            0,
            0, // AudioStreaming alt 0
            9,
            4,
            1,
            1,
            1,
            1,
            2,
            0,
            0, // AudioStreaming alt 1
            7,
            0x24,
            1,
            1,
            self.config.delay_ms,
            1,
            0, // AS general, PCM; bDelay is USB frames (ms)
            11,
            0x24,
            2,
            1,
            2,
            2,
            16,
            1,
            0x44,
            0xac,
            0, // 44.1 kHz stereo 16-bit
            9,
            5,
            SPEAKER_EP,
            0x0d,
            180,
            0,
            1,
            0,
            0, // synchronous isochronous OUT
            7,
            0x25,
            1,
            0,
            0,
            0,
            0, // class-specific endpoint
        ];
        let length = out.len() as u16;
        out[2..4].copy_from_slice(&length.to_le_bytes());
        out
    }

    fn control(&self, setup: Setup, payload: &[u8]) -> Result<Vec<u8>, i32> {
        let (value, index) = (setup.value, setup.index);
        let mut state = self.state.lock().unwrap();
        match (setup.request_type, setup.request) {
            (0x80, 6) => {
                let descriptor = match (value >> 8) as u8 {
                    1 => self.device_descriptor(),
                    2 => self.configuration_descriptor(),
                    3 => self.string_descriptor(value as u8)?,
                    _ => return Err(EPIPE),
                };
                Ok(descriptor[..descriptor.len().min(setup.length as usize)].to_vec())
            }
            (0x00, 5) => Ok(vec![]), // SET_ADDRESS is virtualized by the host controller
            (0x00, 9) if value <= 1 => {
                state.configuration = value as u8;
                Ok(vec![])
            }
            (0x80, 8) => Ok(vec![state.configuration]),
            (0x02, 1) if index == 1 && value == 0 => Ok(vec![]), // CLEAR_FEATURE(ENDPOINT_HALT)
            (0x01, 11) if index == 1 && value <= 1 => {
                state.alternate = value as u8;
                Ok(vec![])
            }
            (0x81, 10) if index == 1 => Ok(vec![state.alternate]),
            (0x80, 0) | (0x81, 0) | (0x82, 0) => Ok(vec![0, 0]), // GET_STATUS
            (0x21, 1) if index == FEATURE_UNIT => {
                let (selector, channel) = ((value >> 8) as u8, value as u8);
                match (selector, channel) {
                    (1, 0) if !payload.is_empty() => state.volume.muted = payload[0] != 0,
                    (2, 0) if payload.len() >= 2 => state.volume.scalar = db_to_scalar(payload),
                    (2, 1..=2) if payload.len() >= 2 => {
                        state.volume.channels[channel as usize - 1] = db_to_scalar(payload)
                    }
                    _ => return Err(EPIPE),
                }
                let volume = state.volume;
                drop(state);
                self.streams.volume(volume);
                Ok(vec![])
            }
            (0xa1, 0x81..=0x84) if index == FEATURE_UNIT => {
                let (selector, channel) = ((value >> 8) as u8, value as u8);
                match (selector, setup.request, channel) {
                    (1, 0x81, 0) => Ok(vec![u8::from(state.volume.muted)]),
                    (2, 0x81, 0) => Ok(scalar_to_db(state.volume.scalar)),
                    (2, 0x81, 1..=2) => Ok(scalar_to_db(state.volume.channels[channel as usize - 1])),
                    (2, 0x82, 0..=2) => Ok((-96_i16 * 256).to_le_bytes().to_vec()), // MIN
                    (2, 0x83, 0..=2) => Ok(0_i16.to_le_bytes().to_vec()),           // MAX
                    (2, 0x84, 0..=2) => Ok(256_i16.to_le_bytes().to_vec()),         // RES
                    _ => Err(EPIPE),
                }
            }
            _ => Err(EPIPE),
        }
    }

    fn iso_endpoint(&self, address: u8) -> Option<IsoEndpoint> {
        (address == SPEAKER_EP).then_some(IsoEndpoint { packet_ns: FRAME_NS })
    }

    fn iso_out(&self, address: u8, _timing: IsoTiming, packets: &[&[u8]]) {
        if address != SPEAKER_EP {
            return;
        }
        // usbip-win2 can handle SET_INTERFACE itself: a transfer is evidence of the streaming alternate setting
        self.state.lock().unwrap().alternate = 1;
        let pcm = packets.concat();
        if !pcm.is_empty() {
            self.streams.pcm(&pcm);
        }
    }

    fn iso_in(&self, _address: u8, _timing: IsoTiming, capacity: &[usize], _data: &mut Vec<u8>) -> Vec<usize> {
        vec![0; capacity.len()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Recorder {
        pcm: Mutex<Vec<u8>>,
        volumes: Mutex<Vec<Volume>>,
    }

    impl Uac1SpeakerStreams for Recorder {
        fn pcm(&self, data: &[u8]) {
            self.pcm.lock().unwrap().extend_from_slice(data);
        }
        fn volume(&self, volume: Volume) {
            self.volumes.lock().unwrap().push(volume);
        }
    }

    fn speaker() -> Uac1Speaker<Recorder> {
        let config = Uac1SpeakerConfig {
            vendor_id: 0xcafe,
            product_id: 0x4001,
            manufacturer: "Win AirPlay".into(),
            product: "Kitchen".into(),
            serial: "0123".into(),
            delay_ms: 180,
        };
        Uac1Speaker::new(config, Recorder::default())
    }

    #[test]
    fn descriptors_are_well_formed() {
        let device = speaker();
        let d = device.device_descriptor();
        assert_eq!((d.len(), d[8], d[9]), (18, 0xfe, 0xca));
        let c = device.configuration_descriptor();
        assert_eq!(u16::from_le_bytes([c[2], c[3]]) as usize, c.len());
        let mut at = 0;
        while at < c.len() {
            assert!(c[at] >= 2 && at + c[at] as usize <= c.len());
            at += c[at] as usize;
        }
        assert!(c.windows(7).any(|w| w == [7, 0x24, 1, 1, 180, 1, 0]), "bDelay");
    }

    #[test]
    fn volume_controls_round_trip() {
        let device = speaker();
        let set = |selector: u16, channel: u16, data: &[u8]| {
            let setup = Setup {
                request_type: 0x21,
                request: 1,
                value: selector << 8 | channel,
                index: 0x0200,
                length: data.len() as u16,
            };
            device.control(setup, data)
        };
        set(2, 0, &(-6 * 256i16).to_le_bytes()).unwrap();
        set(2, 1, &(-96 * 256i16).to_le_bytes()).unwrap();
        set(1, 0, &[1]).unwrap();
        let volume = device.volume();
        assert!((volume.scalar - 0.501).abs() < 0.01);
        assert!(volume.channels[0] < 0.001 && volume.muted);
        assert_eq!(device.streams().volumes.lock().unwrap().len(), 3);
        let get = Setup { request_type: 0xa1, request: 0x81, value: 2 << 8, index: 0x0200, length: 2 };
        assert_eq!(device.control(get, &[]).unwrap(), (-6 * 256i16).to_le_bytes().to_vec());
        assert!(set(3, 0, &[0, 0]).is_err(), "no such control");
    }

    #[test]
    fn pcm_is_forwarded_packed() {
        let device = speaker();
        let timing = IsoTiming { start_ns: 0, packet_ns: FRAME_NS };
        device.iso_out(SPEAKER_EP, timing, &[&[1, 2], &[3, 4, 5, 6]]);
        assert_eq!(*device.streams().pcm.lock().unwrap(), vec![1, 2, 3, 4, 5, 6]);
    }
}
