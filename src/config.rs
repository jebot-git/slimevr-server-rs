//! Server configuration: CLI flags, an optional TOML file, and the resolved
//! runtime values. CLI flags override file values, which override defaults.

use std::path::PathBuf;

use clap::Parser;
use serde::Deserialize;

/// Default tracker UDP protocol port.
pub const DEFAULT_TRACKER_PORT: u16 = 6969;

/// Default SolarXR WebSocket port (the port WiVRn connects to).
pub const DEFAULT_SOLARXR_PORT: u16 = 21110;

/// Default liveness ping interval.
pub const DEFAULT_PING_INTERVAL_SECS: u64 = 2;

/// Default user height for autobone bone lengths.
pub const DEFAULT_HEIGHT_M: f32 = 1.80;

/// Command-line interface.
#[derive(Parser, Debug)]
#[command(name = "slimevr-server-rs", version, about = "A from-scratch Rust SlimeVR FBT server")]
pub struct Cli {
    /// Path to a TOML config file.
    #[arg(long, value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Tracker UDP protocol port ("Hey OVR =D 5").
    #[arg(long)]
    pub tracker_port: Option<u16>,

    /// SolarXR WebSocket port (WiVRn connects here).
    #[arg(long)]
    pub solarxr_port: Option<u16>,

    /// Liveness ping interval in seconds.
    #[arg(long)]
    pub ping_interval_secs: Option<u64>,

    /// User height in meters (drives autobone bone lengths).
    #[arg(long)]
    pub height_m: Option<f32>,
}

/// Values read from a TOML config file. Everything is optional so a partial file
/// is accepted and the rest falls back to defaults.
#[derive(Deserialize, Default, Debug, Clone)]
#[serde(default, deny_unknown_fields)]
pub struct FileConfig {
    pub tracker_port: Option<u16>,
    pub solarxr_port: Option<u16>,
    pub ping_interval_secs: Option<u64>,
    pub height_m: Option<f32>,
}

/// Resolved runtime configuration.
#[derive(Debug, Clone)]
pub struct Config {
    pub tracker_port: u16,
    pub solarxr_port: u16,
    pub ping_interval_secs: u64,
    pub height_m: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tracker_port: DEFAULT_TRACKER_PORT,
            solarxr_port: DEFAULT_SOLARXR_PORT,
            ping_interval_secs: DEFAULT_PING_INTERVAL_SECS,
            height_m: DEFAULT_HEIGHT_M,
        }
    }
}

impl Config {
    /// Resolve configuration: defaults → TOML file (if any) → CLI flags.
    pub fn load(cli: &Cli) -> anyhow::Result<Self> {
        let mut cfg = Config::default();

        if let Some(path) = &cli.config {
            let text = std::fs::read_to_string(path)
                .map_err(|e| anyhow::anyhow!("failed to read config file {path:?}: {e}"))?;
            let file: FileConfig = toml::from_str(&text)
                .map_err(|e| anyhow::anyhow!("failed to parse config file {path:?}: {e}"))?;
            cfg.apply_file(file);
        }

        if let Some(v) = cli.tracker_port {
            cfg.tracker_port = v;
        }
        if let Some(v) = cli.solarxr_port {
            cfg.solarxr_port = v;
        }
        if let Some(v) = cli.ping_interval_secs {
            cfg.ping_interval_secs = v;
        }
        if let Some(v) = cli.height_m {
            cfg.height_m = v;
        }

        Ok(cfg)
    }

    fn apply_file(&mut self, f: FileConfig) {
        if let Some(v) = f.tracker_port {
            self.tracker_port = v;
        }
        if let Some(v) = f.solarxr_port {
            self.solarxr_port = v;
        }
        if let Some(v) = f.ping_interval_secs {
            self.ping_interval_secs = v;
        }
        if let Some(v) = f.height_m {
            self.height_m = v;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_constants() {
        let cfg = Config::default();
        assert_eq!(cfg.tracker_port, DEFAULT_TRACKER_PORT);
        assert_eq!(cfg.solarxr_port, DEFAULT_SOLARXR_PORT);
        assert_eq!(cfg.ping_interval_secs, DEFAULT_PING_INTERVAL_SECS);
        assert_eq!(cfg.height_m, DEFAULT_HEIGHT_M);
    }

    #[test]
    fn cli_flags_override_defaults() {
        let cli = Cli {
            config: None,
            tracker_port: Some(7000),
            solarxr_port: Some(22000),
            ping_interval_secs: Some(5),
            height_m: Some(1.65),
        };
        let cfg = Config::load(&cli).unwrap();
        assert_eq!(cfg.tracker_port, 7000);
        assert_eq!(cfg.solarxr_port, 22000);
        assert_eq!(cfg.ping_interval_secs, 5);
        assert_eq!(cfg.height_m, 1.65);
    }

    #[test]
    fn partial_file_config_falls_back_to_defaults() {
        let text = "height_m = 1.65\nsolarxr_port = 22000\n";
        let file: FileConfig = toml::from_str(text).unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file);
        assert_eq!(cfg.height_m, 1.65);
        assert_eq!(cfg.solarxr_port, 22000);
        assert_eq!(cfg.tracker_port, DEFAULT_TRACKER_PORT);
        assert_eq!(cfg.ping_interval_secs, DEFAULT_PING_INTERVAL_SECS);
    }
}
