//! Attaching and detaching with the installed usbip-win2 client (`usbip.exe`).
//!
//! Applications sharing the driver must only detach the ports they attached: never `detach --all`.

use std::path::PathBuf;
use std::process::Command;

/// The usbip-win2 client: `USBIP_EXE` if set, else its default install locations.
pub fn find() -> Option<PathBuf> {
    if let Some(path) = std::env::var_os("USBIP_EXE") {
        return Some(path.into());
    }
    ["ProgramW6432", "ProgramFiles", "ProgramFiles(x86)"]
        .iter()
        .filter_map(std::env::var_os)
        .map(|base| PathBuf::from(base).join("USBip").join("usbip.exe"))
        .find(|path| path.is_file())
}

/// Attaches `busid` of the server on `127.0.0.1:port`; returns the usbip-win2 port it was attached to.
pub fn attach(usbip: &std::path::Path, port: u16, busid: &str) -> Result<u32, String> {
    let output = Command::new(usbip)
        .args(["--tcp-port", &port.to_string(), "attach", "--remote", "127.0.0.1", "--bus-id", busid])
        .args(["--once", "--terse"])
        .output()
        .map_err(|e| format!("could not run {}: {e}", usbip.display()))?;
    if !output.status.success() {
        return Err(format!(
            "usbip attach failed: {}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    let text = String::from_utf8_lossy(&output.stdout);
    text.trim().parse().map_err(|_| format!("usbip attach returned no port: {text:?}"))
}

/// Detaches a port this application attached.
pub fn detach(usbip: &std::path::Path, port: u32) -> Result<(), String> {
    let status = Command::new(usbip)
        .args(["detach", "--port", &port.to_string()])
        .status()
        .map_err(|e| format!("could not run {}: {e}", usbip.display()))?;
    status.success().then_some(()).ok_or_else(|| format!("usbip detach of port {port} failed: {status}"))
}
