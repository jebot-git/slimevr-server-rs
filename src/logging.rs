//! Shora-style file logging keeps Rust and OpenXR diagnostics off the TUI.
use std::path::{Path, PathBuf};
use std::os::unix::io::AsRawFd;

pub fn default_path() -> PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir).join("shora-rust.log")
}
pub fn init(path: Option<&Path>) -> anyhow::Result<()> {
    if let Some(path) = path {
        use std::os::unix::fs::OpenOptionsExt;
        let file = std::fs::OpenOptions::new().create(true).append(true).mode(0o600).open(path)?;
        // Redirect third-party native runtime diagnostics as well as tracing.
        if unsafe { libc::dup2(file.as_raw_fd(), 2) } < 0 { return Err(std::io::Error::last_os_error().into()); }
    }
    tracing_subscriber::fmt().with_ansi(path.is_none())
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()))
        .with_writer(std::io::stderr).init();
    Ok(())
}
