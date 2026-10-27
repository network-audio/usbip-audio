# usbip-audio

Virtual USB audio devices for Windows, without a kernel driver of our own.

A Rust process emulates USB audio devices and serves them over [USB/IP](https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html) on `127.0.0.1`. The signed [usbip-win2](https://github.com/vadimgrn/usbip-win2) client driver attaches them, and Windows binds its in-box USB audio class drivers: the devices appear as ordinary sound inputs and outputs.

- `Server` serves any `UsbDevice`. Pass port 0 to listen on a free port.
- `uac2::Uac2Device` is a high-speed USB Audio Class 2.0 device (`usbaudio2.sys`) with a playback and/or a capture stream of up to 128 channels at 48 kHz (24-bit samples in 32-bit subslots).
- Windows' shared-mode audio engine opens at most 32 channels of a device (33 fail with `E_INVALIDARG`); exclusive-mode applications can use all of them (CI tests up to 64).
- Isochronous transfers complete at the pace of a `Clock`, which so is the clock of the audio: Windows plays and records at its rate (e.g. a network media clock), without resampling.

Used by [Inferno](https://github.com/network-audio/inferno) (Dante) and [Win AirPlay](https://github.com/network-audio/win-airplay).

## Sharing the driver

Several applications can use the installed usbip-win2 driver at the same time:

- each runs its own `Server`, on its own port (attach with `usbip --tcp-port <port> attach ...`);
- each detaches only the ports it attached (`usbip detach --port N`), never `--all`;
- each uses its own USB vendor/product id and per-device serial numbers, so Windows keeps their settings apart;
- installers pin the same usbip-win2 release, install it only when it's missing, and leave it installed on uninstall.

## Testing

`cargo test` checks descriptors and the protocol. The Windows CI job installs usbip-win2, attaches probe devices of 2 to 64 channels, and plays and records through each with WASAPI (`examples/uac2_probe.rs`).
