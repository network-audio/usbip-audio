use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

/// USB/IP status of a stalled (refused) transfer: `-EPIPE`.
pub const EPIPE: i32 = -32;

/// A control transfer's setup packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Setup {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl Setup {
    pub fn parse(bytes: [u8; 8]) -> Self {
        Self {
            request_type: bytes[0],
            request: bytes[1],
            value: u16::from_le_bytes([bytes[2], bytes[3]]),
            index: u16::from_le_bytes([bytes[4], bytes[5]]),
            length: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }
}

/// The bus speed the device reports, which sets the duration of an isochronous (micro)frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    /// 1 ms frames
    Full,
    /// 125 µs microframes
    High,
}

impl Speed {
    /// The USB/IP (Linux `usb_device_speed`) value.
    pub(crate) fn usbip_value(self) -> u32 {
        match self {
            Speed::Full => 2,
            Speed::High => 3,
        }
    }
}

/// An isochronous endpoint: how long each of its packets lasts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IsoEndpoint {
    pub packet_ns: u64,
}

/// When the packets of an isochronous transfer are due: packet `i` covers
/// `start_ns + i * packet_ns .. start_ns + (i + 1) * packet_ns` of the server's [`Clock`](crate::Clock).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IsoTiming {
    pub start_ns: u64,
    pub packet_ns: u64,
}

/// Wakes the [`Server`](crate::Server) when a device may have an interrupt transfer to complete: see
/// [`UsbDevice::interrupt_endpoint`]. Clones notify the same server.
#[derive(Clone, Default)]
pub struct Notifier(Arc<(Mutex<u64>, Condvar)>);

impl Notifier {
    /// [`UsbDevice::interrupt_in`] may now return data.
    pub fn notify(&self) {
        *self.0 .0.lock().unwrap() += 1;
        self.0 .1.notify_all();
    }

    /// How often it was notified: [`Notifier::wait`] returns when this changes.
    pub(crate) fn count(&self) -> u64 {
        *self.0 .0.lock().unwrap()
    }

    /// Waits at most `timeout` for a notification after the `seen`th.
    pub(crate) fn wait(&self, seen: u64, timeout: Duration) {
        let count = self.0 .0.lock().unwrap();
        let _ = self.0 .1.wait_timeout_while(count, timeout, |count| *count == seen).unwrap();
    }
}

/// A device the [`Server`](crate::Server) exports.
pub trait UsbDevice: Send + Sync {
    fn speed(&self) -> Speed;
    /// The standard device descriptor.
    fn device_descriptor(&self) -> Vec<u8>;
    /// The (only) configuration descriptor, with all interface, endpoint and class-specific descriptors.
    fn configuration_descriptor(&self) -> Vec<u8>;
    /// Handles a control transfer on endpoint 0, including standard requests: the data to return (IN), or
    /// an empty vector (OUT); or a status such as [`EPIPE`].
    fn control(&self, setup: Setup, data: &[u8]) -> Result<Vec<u8>, i32>;
    /// The isochronous endpoint with this address (number, with bit 7 set for IN), if there is one.
    fn iso_endpoint(&self, address: u8) -> Option<IsoEndpoint>;
    /// Data Windows sent to an OUT endpoint, one slice per packet. Called when the transfer arrives, which is
    /// ahead of the time it covers.
    fn iso_out(&self, address: u8, timing: IsoTiming, packets: &[&[u8]]);
    /// Fills the packets of an IN transfer, at the end of the time it covers: appends each packet's data
    /// (at most `capacity[i]` bytes) to `data`, returns each packet's length.
    fn iso_in(&self, address: u8, timing: IsoTiming, capacity: &[usize], data: &mut Vec<u8>) -> Vec<usize>;
    /// The interrupt IN endpoint with this address, if there is one: what notifies the server when
    /// [`UsbDevice::interrupt_in`] may have data for it. Its transfers wait until then, in the order Windows
    /// submitted them.
    fn interrupt_endpoint(&self, _address: u8) -> Option<Notifier> {
        None
    }
    /// The data (at most `capacity` bytes) of the interrupt IN transfer to complete now, if the device has any.
    fn interrupt_in(&self, _address: u8, _capacity: usize) -> Option<Vec<u8>> {
        None
    }
}
