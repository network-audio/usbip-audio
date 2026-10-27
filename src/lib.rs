//! Virtual USB audio devices served over [USB/IP] on the loopback interface.
//!
//! The signed [usbip-win2] client driver attaches them, and Windows binds its in-box USB audio class drivers
//! (`usbaudio.sys`, `usbaudio2.sys`): the devices appear as ordinary sound inputs and outputs, without a kernel
//! driver of our own.
//!
//! - [`Server`] serves any [`UsbDevice`] over USB/IP. Several applications can share the installed driver: each
//!   runs its own server on its own port (pass port 0 for a free one), attaching and detaching only its own
//!   devices.
//! - [`uac1::Uac1Speaker`] is a USB Audio Class 1.0 stereo speaker with volume and mute controls.
//! - [`uac2::Uac2Device`] is a USB Audio Class 2.0 device with up to one playback (Windows to the device) and
//!   one capture (device to Windows) streaming interface, of any channel count.
//!
//! Isochronous transfers complete at the pace of a [`Clock`], which therefore is the clock of the audio:
//! Windows renders and captures at its rate (e.g. a network's media clock), without resampling of its own.
//!
//! [USB/IP]: https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html
//! [usbip-win2]: https://github.com/vadimgrn/usbip-win2

mod clock;
mod device;
mod server;
pub mod uac1;
pub mod uac2;
pub mod usbip_exe;

pub use clock::{Clock, MonotonicClock};
pub use device::{IsoEndpoint, IsoTiming, Setup, Speed, UsbDevice, EPIPE};
pub use server::Server;
