//! Server configuration: CLI flags, an optional TOML file, and the resolved
//! runtime values. CLI flags override file values, which override defaults.

use std::collections::HashMap;
use std::path::PathBuf;

use clap::Parser;
use serde::Deserialize;

/// Default tracker UDP protocol port.
pub const DEFAULT_TRACKER_PORT: u16 = 6969;

/// Default SolarXR IPC socket path — the Unix domain socket WiVRn/SteamVR
/// connect to for the skeleton data feed.
pub fn default_solarxr_socket() -> String {
    std::env::var("XDG_RUNTIME_DIR")
        .map(|d| format!("{d}/SlimeVRRpc"))
        .unwrap_or_else(|_| "/run/user/1000/SlimeVRRpc".to_string())
}

/// Default SteamVR feeder socket path — where WiVRn sends the HMD pose.
pub fn default_feeder_socket() -> String {
    std::env::var("XDG_RUNTIME_DIR")
        .map(|d| format!("{d}/SlimeVRInput"))
        .unwrap_or_else(|_| "/run/user/1000/SlimeVRInput".to_string())
}

/// Default liveness ping interval.
pub const DEFAULT_PING_INTERVAL_SECS: u64 = 2;

/// Default tracker timeout: a tracker is dropped after this long with no packets
/// (must be longer than the ping interval; matches shora's 5 s tracker timeout).
pub const DEFAULT_TRACKER_TIMEOUT_SECS: u64 = 5;

/// Default user height for autobone bone lengths.
pub const DEFAULT_HEIGHT_M: f32 = 1.80;

/// Default tracker rotation smoothing (0..1 blend toward the latest sample).
pub const DEFAULT_SMOOTHING: f32 = 0.3;

/// Parse a MAC address like `AA:BB:CC:DD:EE:FF` or `AABBCCDDEEFF` (hex, with
/// optional `:`/`-` separators) into its 6 bytes.
pub fn parse_mac(s: &str) -> anyhow::Result<[u8; 6]> {
    let cleaned: String = s.chars().filter(|c| *c != ':' && *c != '-').collect();
    anyhow::ensure!(
        cleaned.len() == 12,
        "MAC must be exactly 12 hex characters, got {s:?}"
    );
    let mut out = [0u8; 6];
    for i in 0..6 {
        let byte = &cleaned[i * 2..i * 2 + 2];
        out[i] = u8::from_str_radix(byte, 16)
            .map_err(|e| anyhow::anyhow!("invalid MAC byte {byte:?}: {e}"))?;
    }
    Ok(out)
}

/// Parse a `MAC=POSITION` assignment string (e.g. `AA:BB:CC:DD:EE:FF=9`).
pub fn parse_assignment(s: &str) -> anyhow::Result<([u8; 6], u8)> {
    let (mac, pos) = s
        .split_once('=')
        .ok_or_else(|| anyhow::anyhow!("assignment must be `MAC=POSITION`, got {s:?}"))?;
    let mac = parse_mac(mac.trim())?;
    let pos: u8 = pos
        .trim()
        .parse()
        .map_err(|e| anyhow::anyhow!("invalid tracker position {pos:?}: {e}"))?;
    Ok((mac, pos))
}

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

    /// SolarXR IPC socket path (Unix domain socket WiVRn connects to).
    #[arg(long)]
    pub solarxr_socket: Option<String>,

    /// SteamVR feeder socket path (where WiVRn sends the HMD pose).
    #[arg(long)]
    pub feeder_socket: Option<String>,

    /// Liveness ping interval in seconds.
    #[arg(long)]
    pub ping_interval_secs: Option<u64>,

    /// Drop a tracker after this many seconds without packets.
    #[arg(long)]
    pub tracker_timeout_secs: Option<u64>,

    /// User height in meters (drives autobone bone lengths).
    #[arg(long)]
    pub height_m: Option<f32>,

    /// Tracker rotation smoothing (0..1 blend toward the latest sample).
    #[arg(long)]
    pub smoothing: Option<f32>,

    /// Override a tracker's body-part assignment: `MAC=POSITION` (repeatable).
    #[arg(long, value_name = "MAC=POSITION")]
    pub assign: Vec<String>,
}

/// One manual tracker assignment from a TOML file.
#[derive(Deserialize, Default, Debug, Clone)]
#[serde(default, deny_unknown_fields)]
pub struct AssignmentFile {
    pub mac: String,
    pub position: u8,
}

/// Values read from a TOML config file. Everything is optional so a partial file
/// is accepted and the rest falls back to defaults.
#[derive(Deserialize, Default, Debug, Clone)]
#[serde(default, deny_unknown_fields)]
pub struct FileConfig {
    pub tracker_port: Option<u16>,
    pub solarxr_socket: Option<String>,
    pub feeder_socket: Option<String>,
    pub ping_interval_secs: Option<u64>,
    pub tracker_timeout_secs: Option<u64>,
    pub height_m: Option<f32>,
    pub smoothing: Option<f32>,
    #[serde(default)]
    pub tracker_assignments: Vec<AssignmentFile>,
}

/// Resolved runtime configuration.
#[derive(Debug, Clone)]
pub struct Config {
    pub tracker_port: u16,
    pub solarxr_socket: String,
    pub feeder_socket: String,
    pub ping_interval_secs: u64,
    pub tracker_timeout_secs: u64,
    pub height_m: f32,
    pub smoothing: f32,
    /// Manual tracker body-part overrides, keyed by MAC.
    pub tracker_assignments: HashMap<[u8; 6], u8>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tracker_port: DEFAULT_TRACKER_PORT,
            solarxr_socket: default_solarxr_socket(),
            feeder_socket: default_feeder_socket(),
            ping_interval_secs: DEFAULT_PING_INTERVAL_SECS,
            tracker_timeout_secs: DEFAULT_TRACKER_TIMEOUT_SECS,
            height_m: DEFAULT_HEIGHT_M,
            smoothing: DEFAULT_SMOOTHING,
            tracker_assignments: HashMap::new(),
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
            cfg.apply_file(file)?;
        }

        if let Some(v) = cli.tracker_port {
            cfg.tracker_port = v;
        }
        if let Some(v) = &cli.solarxr_socket {
            cfg.solarxr_socket = v.clone();
        }
        if let Some(v) = &cli.feeder_socket {
            cfg.feeder_socket = v.clone();
        }
        if let Some(v) = cli.ping_interval_secs {
            cfg.ping_interval_secs = v;
        }
        if let Some(v) = cli.tracker_timeout_secs {
            cfg.tracker_timeout_secs = v;
        }
        if let Some(v) = cli.height_m {
            cfg.height_m = v;
        }
        if let Some(v) = cli.smoothing {
            cfg.smoothing = v;
        }
        for s in &cli.assign {
            let (mac, position) = parse_assignment(s)?;
            cfg.tracker_assignments.insert(mac, position);
        }

        Ok(cfg)
    }

    fn apply_file(&mut self, f: FileConfig) -> anyhow::Result<()> {
        if let Some(v) = f.tracker_port {
            self.tracker_port = v;
        }
        if let Some(v) = f.solarxr_socket {
            self.solarxr_socket = v;
        }
        if let Some(v) = f.feeder_socket {
            self.feeder_socket = v;
        }
        if let Some(v) = f.ping_interval_secs {
            self.ping_interval_secs = v;
        }
        if let Some(v) = f.tracker_timeout_secs {
            self.tracker_timeout_secs = v;
        }
        if let Some(v) = f.height_m {
            self.height_m = v;
        }
        if let Some(v) = f.smoothing {
            self.smoothing = v;
        }
        for a in f.tracker_assignments {
            let mac = parse_mac(&a.mac)
                .map_err(|e| anyhow::anyhow!("bad tracker_assignments mac {:?}: {e}", a.mac))?;
            self.tracker_assignments.insert(mac, a.position);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_constants() {
        let cfg = Config::default();
        assert_eq!(cfg.tracker_port, DEFAULT_TRACKER_PORT);
        assert_eq!(cfg.solarxr_socket, default_solarxr_socket());
        assert_eq!(cfg.ping_interval_secs, DEFAULT_PING_INTERVAL_SECS);
        assert_eq!(cfg.tracker_timeout_secs, DEFAULT_TRACKER_TIMEOUT_SECS);
        assert_eq!(cfg.height_m, DEFAULT_HEIGHT_M);
    }

    #[test]
    fn cli_flags_override_defaults() {
        let cli = Cli {
            config: None,
            tracker_port: Some(7000),
            solarxr_socket: Some("/tmp/test.sock".into()),
            feeder_socket: Some("/tmp/feeder.sock".into()),
            ping_interval_secs: Some(5),
            tracker_timeout_secs: Some(9),
            height_m: Some(1.65),
            smoothing: Some(0.5),
            assign: vec!["aa:bb:cc:dd:ee:ff=9".into()],
        };
        let cfg = Config::load(&cli).unwrap();
        assert_eq!(cfg.tracker_port, 7000);
        assert_eq!(cfg.solarxr_socket, "/tmp/test.sock");
        assert_eq!(cfg.feeder_socket, "/tmp/feeder.sock");
        assert_eq!(cfg.ping_interval_secs, 5);
        assert_eq!(cfg.tracker_timeout_secs, 9);
        assert_eq!(cfg.height_m, 1.65);
        assert_eq!(cfg.smoothing, 0.5);
        assert_eq!(cfg.tracker_assignments[&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]], 9);
    }

    #[test]
    fn partial_file_config_falls_back_to_defaults() {
        let text = "height_m = 1.65\nsolarxr_socket = \"/tmp/test.sock\"\n";
        let file: FileConfig = toml::from_str(text).unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file).unwrap();
        assert_eq!(cfg.height_m, 1.65);
        assert_eq!(cfg.solarxr_socket, "/tmp/test.sock");
        assert_eq!(cfg.tracker_port, DEFAULT_TRACKER_PORT);
        assert_eq!(cfg.ping_interval_secs, DEFAULT_PING_INTERVAL_SECS);
        assert_eq!(cfg.tracker_timeout_secs, DEFAULT_TRACKER_TIMEOUT_SECS);
    }

    #[test]
    fn file_assignments_parse() {
        let text = "[[tracker_assignments]]\nmac = \"AA:BB:CC:DD:EE:FF\"\nposition = 9\n";
        let file: FileConfig = toml::from_str(text).unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file).unwrap();
        assert_eq!(cfg.tracker_assignments[&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]], 9);
    }

    #[test]
    fn parse_mac_formats() {
        assert_eq!(
            parse_mac("AA:BB:CC:DD:EE:FF").unwrap(),
            [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]
        );
        assert_eq!(
            parse_mac("aabbccddeeff").unwrap(),
            [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]
        );
        assert_eq!(
            parse_mac("aa-bb-cc-dd-ee-ff").unwrap(),
            [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]
        );
        assert!(parse_mac("aabbcc").is_err());
        assert!(parse_mac("zz:bb:cc:dd:ee:ff").is_err());
    }

    #[test]
    fn parse_assignment_round_trips() {
        assert_eq!(
            parse_assignment("AA:BB:CC:DD:EE:FF=10").unwrap(),
            ([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 10)
        );
        assert!(parse_assignment("AA:BB:CC:DD:EE:FF").is_err());
        assert!(parse_assignment("AA:BB:CC:DD:EE:FF=notanumber").is_err());
    }
}
