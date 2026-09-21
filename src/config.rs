//! Server configuration: CLI flags, an optional TOML file, and the resolved
//! runtime values. CLI flags override file values, which override defaults.

use std::collections::HashMap;
use std::path::PathBuf;

use clap::Parser;
use serde::Deserialize;

use crate::tracker::TrackerId;
use crate::haritorax::config::{HaritoraXConfig, Model};
use crate::ui::UiMode;
use crate::face::config::{FaceConfig, Source, OutputMode};

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

/// Default prediction time (seconds to extrapolate rotations forward).
pub const DEFAULT_PREDICTION: f32 = 0.0;

/// Parse a MAC address like `AA:BB:CC:DD:EE:FF` or `AABBCCDDEEFF` (hex, with
/// optional `:`/`-` separators) into its 6 bytes.
pub fn parse_mac(s: &str) -> anyhow::Result<[u8; 6]> {
    let cleaned: String = s.chars().filter(|c| *c != ':' && *c != '-').collect();
    anyhow::ensure!(
        cleaned.len() == 12 && cleaned.is_ascii(),
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

/// Parse `MAC[/SENSOR]=POSITION`; a bare MAC targets the primary sensor (0).
pub fn parse_assignment(s: &str) -> anyhow::Result<(TrackerId, u8)> {
    let (mac, pos) = s
        .split_once('=')
        .ok_or_else(|| anyhow::anyhow!("assignment must be `MAC[/SENSOR]=POSITION`, got {s:?}"))?;
    let (mac, sensor_id) = match mac.trim().split_once('/') {
        Some((mac, sensor)) => (mac, sensor.trim().parse::<u8>()
            .map_err(|e| anyhow::anyhow!("invalid sensor id {sensor:?}: {e}"))?),
        None => (mac, 0),
    };
    let id = TrackerId::new(parse_mac(mac.trim())?, sensor_id);
    let pos: u8 = pos
        .trim()
        .parse()
        .map_err(|e| anyhow::anyhow!("invalid tracker position {pos:?}: {e}"))?;
    Ok((id, pos))
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

    /// Prediction time in seconds (extrapolate rotations forward for latency).
    #[arg(long)]
    pub prediction: Option<f32>,

    /// Override a tracker's body-part assignment: `MAC[/SENSOR]=POSITION` (repeatable).
    #[arg(long, value_name = "MAC[/SENSOR]=POSITION")]
    pub assign: Vec<String>,

    /// Enable oscavmgr face/eye tracking with this source.
    #[arg(long, value_enum)]
    pub face_source: Option<Source>,

    /// Disable face tracking, overriding the config file and --face-source.
    #[arg(long)]
    pub no_face: bool,

    /// OSC destination for face output (default 127.0.0.1:9000).
    #[arg(long)]
    pub face_destination: Option<std::net::SocketAddr>,
    /// Face OSC mapping: auto, fixed UniFT, VRChat JSON, or UniFT plus JSON.
    #[arg(long, value_enum)]
    pub face_output: Option<OutputMode>,
    /// VRChat avatar OSC JSON translation sheet (uses input addresses/types).
    #[arg(long, value_name = "PATH")]
    pub face_translation_sheet: Option<PathBuf>,
    /// Enable native HaritoraX serial input (all discovered GX6/GX2 ports).
    #[arg(long)]
    pub haritorax: bool,
    /// Serial device path, repeatable; also enables HaritoraX input.
    #[arg(long)]
    pub serial_port: Vec<String>,
    #[arg(long, value_enum)]
    pub haritorax_model: Option<Model>,
    /// Terminal UI or headless operation.
    #[arg(long, value_enum)]
    pub ui: Option<UiMode>,
    /// Optional owner-only Unix control socket for the Qt frontend.
    #[arg(long)]
    pub control_socket: Option<PathBuf>,
    /// Write logs here (TUI otherwise uses shora-rust.log in the runtime dir).
    #[arg(long)]
    pub log_file: Option<PathBuf>,
}

/// One manual tracker assignment from a TOML file.
#[derive(Deserialize, Default, Debug, Clone)]
#[serde(default, deny_unknown_fields)]
pub struct AssignmentFile {
    pub mac: String,
    /// Sensor on the device; omitted values target sensor 0.
    pub sensor_id: u8,
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
    pub prediction: Option<f32>,
    pub drift_correction: Option<bool>,
    pub drift_amount: Option<f32>,
    #[serde(default)]
    pub tracker_assignments: Vec<AssignmentFile>,
    pub face: FaceConfig,
    pub haritorax: HaritoraXConfig,
    pub ui: UiMode,
    pub control_socket: Option<PathBuf>,
    pub log_file: Option<PathBuf>,
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
    pub prediction: f32,
    pub drift_correction: bool,
    pub drift_amount: f32,
    /// Manual tracker body-part overrides, keyed by MAC and sensor id.
    pub tracker_assignments: HashMap<TrackerId, u8>,
    pub face: FaceConfig,
    pub haritorax: HaritoraXConfig,
    pub ui: UiMode,
    pub control_socket: Option<PathBuf>,
    pub log_file: Option<PathBuf>,
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
            prediction: DEFAULT_PREDICTION,
            drift_correction: false,
            drift_amount: 0.5,
            tracker_assignments: HashMap::new(),
            face: FaceConfig::default(),
            haritorax: HaritoraXConfig::default(),
            ui: UiMode::Headless,
            control_socket: None,
            log_file: None,
        }
    }
}

impl Config {
    pub fn tracking_settings(&self) -> crate::settings::TrackingSettings {
        crate::settings::TrackingSettings { height_m: self.height_m, smoothing: self.smoothing,
            prediction: self.prediction, drift_correction: self.drift_correction, drift_amount: self.drift_amount }
    }
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
        if let Some(v) = cli.prediction {
            cfg.prediction = v;
        }
        for s in &cli.assign {
            let (mac, position) = parse_assignment(s)?;
            cfg.tracker_assignments.insert(mac, position);
        }

        if let Some(source) = cli.face_source {
            cfg.face.source = source;
            cfg.face.enabled = true;
        }
        if let Some(destination) = cli.face_destination {
            cfg.face.destination = destination;
        }
        if let Some(output) = cli.face_output {
            cfg.face.output = output;
            if output == OutputMode::Unift {
                cfg.face.avatar = None;
                cfg.face.translation_sheet = None;
            }
        }
        if let Some(path) = &cli.face_translation_sheet {
            cfg.face.translation_sheet = Some(path.clone());
            cfg.face.avatar = None;
        }
        if cli.no_face {
            cfg.face.enabled = false;
        }
        if cli.haritorax || !cli.serial_port.is_empty() { cfg.haritorax.enabled = true; }
        if !cli.serial_port.is_empty() { cfg.haritorax.ports = cli.serial_port.clone(); }
        if let Some(model) = cli.haritorax_model { cfg.haritorax.model = model; }
        if let Some(ui) = cli.ui { cfg.ui = ui; }
        if cli.control_socket.is_some() { cfg.control_socket = cli.control_socket.clone(); }
        if cli.log_file.is_some() { cfg.log_file = cli.log_file.clone(); }
        cfg.haritorax.validate()?;
        cfg.tracking_settings().validate()?;
        anyhow::ensure!(cfg.ping_interval_secs > 0 && cfg.tracker_timeout_secs > 0, "tracker timing values must be positive");
        if let Some(path) = &cfg.control_socket {
            anyhow::ensure!(path != &PathBuf::from(&cfg.solarxr_socket) && path != &PathBuf::from(&cfg.feeder_socket), "control socket must differ from tracking sockets");
        }
        cfg.face.validate()?;
        if cfg.face.enabled {
            anyhow::ensure!(cfg.tracker_port != cfg.face.osc_port &&
                (cfg.face.source != Source::Babble || cfg.tracker_port != cfg.face.babble_port),
                "face input ports must differ from tracker_port");
        }
        Ok(cfg)
    }

    fn apply_file(&mut self, f: FileConfig) -> anyhow::Result<()> {
        if let Some(v) = f.drift_correction { self.drift_correction = v; }
        if let Some(v) = f.drift_amount { self.drift_amount = v; }
        self.face = f.face;
        self.haritorax = f.haritorax;
        self.ui = f.ui;
        self.control_socket = f.control_socket;
        self.log_file = f.log_file;
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
        if let Some(v) = f.prediction {
            self.prediction = v;
        }
        for a in f.tracker_assignments {
            let mac = parse_mac(&a.mac)
                .map_err(|e| anyhow::anyhow!("bad tracker_assignments mac {:?}: {e}", a.mac))?;
            self.tracker_assignments.insert(TrackerId::new(mac, a.sensor_id), a.position);
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
            face_source: None,
            no_face: false,
            face_destination: None,
            face_output: None,
            face_translation_sheet: None,
            haritorax: false,
            serial_port: vec![],
            haritorax_model: None,
            ui: None,
            control_socket: None,
            log_file: None,
            tracker_port: Some(7000),
            solarxr_socket: Some("/tmp/test.sock".into()),
            feeder_socket: Some("/tmp/feeder.sock".into()),
            ping_interval_secs: Some(5),
            tracker_timeout_secs: Some(9),
            height_m: Some(1.65),
            smoothing: Some(0.5),
            prediction: Some(0.02),
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
        assert_eq!(cfg.prediction, 0.02);
        assert_eq!(cfg.tracker_assignments[&TrackerId::new([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 0)], 9);
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
        assert_eq!(cfg.tracker_assignments[&TrackerId::new([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 0)], 9);
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
            (TrackerId::new([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 0), 10)
        );
        assert!(parse_assignment("AA:BB:CC:DD:EE:FF").is_err());
        assert!(parse_assignment("AA:BB:CC:DD:EE:FF=notanumber").is_err());
    }

    #[test]
    fn sensor_assignments_preserve_primary_and_validate_ids() {
        let mac = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff];
        let cli = Cli::try_parse_from([
            "server", "--assign", "AA:BB:CC:DD:EE:FF=9",
            "--assign", "AA:BB:CC:DD:EE:FF/1=10",
        ]).unwrap();
        let cfg = Config::load(&cli).unwrap();
        assert_eq!(cfg.tracker_assignments[&TrackerId::new(mac, 0)], 9);
        assert_eq!(cfg.tracker_assignments[&TrackerId::new(mac, 1)], 10);
        for invalid in ["AA:BB:CC:DD:EE:FF/=9", "AA:BB:CC:DD:EE:FF/256=9", "AA:BB:CC:DD:EE:FF/-1=9"] {
            assert!(parse_assignment(invalid).is_err());
        }
        // Twelve bytes of non-ASCII input must return an error, not panic at a
        // UTF-8 character boundary while slicing MAC bytes.
        assert!(parse_mac("aéaaaaaaaaa").is_err());
    }

    #[test]
    fn file_assigns_extension_on_same_mac() {
        let file: FileConfig = toml::from_str(r#"
            [[tracker_assignments]]
            mac = "AA:BB:CC:DD:EE:FF"
            position = 9
            [[tracker_assignments]]
            mac = "AA:BB:CC:DD:EE:FF"
            sensor_id = 1
            position = 10
        "#).unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file).unwrap();
        assert_eq!(cfg.tracker_assignments[&TrackerId::new([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 0)], 9);
        assert_eq!(cfg.tracker_assignments[&TrackerId::new([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], 1)], 10);
    }
    #[test]
    fn face_cli_enables_source_and_no_face_wins() {
        let cli = Cli::try_parse_from(["server", "--face-source", "babble",
            "--face-destination", "127.0.0.1:9999"]).unwrap();
        let cfg = Config::load(&cli).unwrap();
        assert!(cfg.face.enabled);
        assert_eq!(cfg.face.source, Source::Babble);
        assert_eq!(cfg.face.destination.port(), 9999);
        let cli = Cli::try_parse_from(["server", "--face-source", "openxr", "--no-face"]).unwrap();
        assert!(!Config::load(&cli).unwrap().face.enabled);
        let cli = Cli::try_parse_from(["server", "--face-source", "babble", "--tracker-port", "9400"]).unwrap();
        assert!(Config::load(&cli).is_err());
    }

    #[test]
    fn nested_face_file_settings_are_applied() {
        let file: FileConfig = toml::from_str("[face]\nenabled = true\nsource = 'babble'\nbabble_port = 9410\n").unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file).unwrap();
        assert!(cfg.face.enabled);
        assert_eq!(cfg.face.babble_port, 9410);
        assert_eq!(cfg.face.osc_port, 9002);
    }

    #[test]
    fn face_output_cli_selects_unift_and_vrchat_json() {
        let cli = Cli::try_parse_from(["server", "--face-source", "babble", "--face-output", "unift"]).unwrap();
        assert_eq!(Config::load(&cli).unwrap().face.output, OutputMode::Unift);
        let cli = Cli::try_parse_from(["server", "--face-source", "babble", "--face-output", "both",
            "--face-translation-sheet", "avatar.json"]).unwrap();
        let cfg = Config::load(&cli).unwrap();
        assert_eq!(cfg.face.output, OutputMode::Both);
        assert_eq!(cfg.face.translation_sheet, Some(PathBuf::from("avatar.json")));
        let cli = Cli::try_parse_from(["server", "--face-source", "babble", "--face-output", "json"]).unwrap();
        assert!(Config::load(&cli).is_err());
        let cli = Cli::try_parse_from(["server", "--no-face", "--face-output", "json"]).unwrap();
        assert!(!Config::load(&cli).unwrap().face.enabled);
    }

    #[test]
    fn serial_cli_and_interface_configuration() {
        let cli = Cli::try_parse_from(["server", "--serial-port", "/dev/example0",
            "--serial-port", "/dev/example1", "--haritorax-model", "wireless",
            "--ui", "tui", "--control-socket", "/tmp/shora-control-test",
            "--log-file", "/tmp/shora-log-test"]).unwrap();
        let cfg = Config::load(&cli).unwrap();
        assert!(cfg.haritorax.enabled);
        assert_eq!(cfg.haritorax.ports, ["/dev/example0", "/dev/example1"]);
        assert_eq!(cfg.haritorax.model, Model::Wireless);
        assert_eq!(cfg.ui, UiMode::Tui);
        assert_eq!(cfg.control_socket, Some(PathBuf::from("/tmp/shora-control-test")));
        let cli = Cli::try_parse_from(["server", "--control-socket", "/tmp/same",
            "--feeder-socket", "/tmp/same"]).unwrap();
        assert!(Config::load(&cli).is_err());
        let cli = Cli::try_parse_from(["server", "--serial-port", ""]).unwrap();
        assert!(Config::load(&cli).is_err());
    }

    #[test]
    fn example_config_and_partial_serial_settings() {
        let file: FileConfig = toml::from_str(include_str!("../config.example.toml")).unwrap();
        let mut cfg = Config::default();
        cfg.apply_file(file).unwrap();
        assert!(!cfg.haritorax.enabled);
        assert_eq!(cfg.haritorax.baud_rate, 500000);
        let file: FileConfig = toml::from_str("ui = 'tui'\n[haritorax]\nenabled = true\n").unwrap();
        cfg.apply_file(file).unwrap();
        assert!(cfg.haritorax.enabled);
        assert!(cfg.haritorax.ports.is_empty());
        assert_eq!(cfg.haritorax.model, Model::X2);
        assert_eq!(cfg.ui, UiMode::Tui);
    }

}
