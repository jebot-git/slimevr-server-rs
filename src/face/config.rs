//! Opt-in oscavmgr face/eye relay settings.

use std::net::{Ipv4Addr, SocketAddr};
use std::path::PathBuf;

use clap::ValueEnum;
use serde::Deserialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum Source {
    Babble,
    Openxr,
}

/// Automatic OSCQuery, fixed Unified Expressions, avatar JSON, or both outputs.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum OutputMode {
    #[default]
    Auto,
    Unift,
    Json,
    Both,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FaceConfig {
    pub enabled: bool,
    pub source: Source,
    /// Destination is independent of whether input sockets are exposed to LAN.
    pub destination: SocketAddr,
    /// Avatar feedback / optional VSync input from VRChat.
    pub osc_port: u16,
    /// Project Babble and EyeTrackVR share this input port.
    pub babble_port: u16,
    pub expose: bool,
    /// Explicit OSCQuery /avatar tree; takes precedence over discovery.
    pub avatar: Option<PathBuf>,
    /// Advertise OSCQuery and discover VRChat's avatar parameter mapping.
    pub osc_query: bool,
    pub output: OutputMode,
    /// VRChat OSC avatar config: parameters[].name / input.address / input.type.
    pub translation_sheet: Option<PathBuf>,
}

impl Default for FaceConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            source: Source::Babble,
            destination: SocketAddr::from((Ipv4Addr::LOCALHOST, 9000)),
            osc_port: 9002,
            babble_port: 9400,
            expose: false,
            avatar: None,
            osc_query: true,
            output: OutputMode::Auto,
            translation_sheet: None,
        }
    }
}

impl FaceConfig {
    pub fn listen_addr(&self, port: u16) -> SocketAddr {
        SocketAddr::from((if self.expose {
            Ipv4Addr::UNSPECIFIED
        } else {
            Ipv4Addr::LOCALHOST
        }, port))
    }

    pub fn validate(&self) -> anyhow::Result<()> {
        if !self.enabled { return Ok(()); }
        anyhow::ensure!(self.avatar.is_none() || self.translation_sheet.is_none(),
            "choose either face.avatar (OSCQuery tree) or face.translation_sheet (VRChat avatar JSON)");
        if matches!(self.output, OutputMode::Json | OutputMode::Both) {
            anyhow::ensure!(self.translation_sheet.is_some(),
                "face.output 'json' and 'both' require face.translation_sheet");
        }
        if self.output == OutputMode::Unift {
            anyhow::ensure!(self.avatar.is_none() && self.translation_sheet.is_none(),
                "face.output 'unift' cannot use an avatar mapping; use 'both' for UniFT plus JSON");
        }
        #[cfg(not(feature = "face-xr"))]
        anyhow::ensure!(self.source != Source::Openxr,
            "face source 'openxr' requires rebuilding with --features face-xr");
        anyhow::ensure!(self.destination.port() != 0 && !self.destination.ip().is_unspecified(),
            "face.destination must specify a destination IP and nonzero port");
        anyhow::ensure!(self.osc_port != 0, "face.osc_port must be nonzero");
        if self.source == Source::Babble {
            anyhow::ensure!(self.babble_port != 0, "face.babble_port must be nonzero");
            anyhow::ensure!(self.babble_port != self.osc_port,
                "face.babble_port and face.osc_port must be different");
        }
        if self.destination.ip().is_loopback() {
            anyhow::ensure!(self.destination.port() != self.osc_port &&
                (self.source != Source::Babble || self.destination.port() != self.babble_port),
                "face.destination must not point back to a face input port");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_config_is_opt_in_and_partial_files_keep_defaults() {
        let cfg: FaceConfig = toml::from_str("enabled = true\nsource = 'babble'\n").unwrap();
        assert!(!FaceConfig::default().enabled);
        assert_eq!(cfg.destination, "127.0.0.1:9000".parse().unwrap());
        cfg.validate().unwrap();
        assert!(toml::from_str::<FaceConfig>("source = 'typo'").is_err());
        assert!(toml::from_str::<FaceConfig>("osc_prt = 9000").is_err());
    }

    #[test]
    fn conflicting_ports_and_unspecified_destination_are_rejected() {
        let mut cfg = FaceConfig { enabled: true, ..FaceConfig::default() };
        cfg.babble_port = cfg.osc_port;
        assert!(cfg.validate().is_err());
        cfg.babble_port = 9400;
        cfg.destination = "0.0.0.0:9000".parse().unwrap();
        assert!(cfg.validate().is_err());
        cfg.destination = "127.0.0.1:9400".parse().unwrap();
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn output_modes_require_unambiguous_mapping_settings() {
        let mut cfg: FaceConfig = toml::from_str("enabled = true\noutput = 'json'\n").unwrap();
        assert!(cfg.validate().is_err());
        cfg.translation_sheet = Some("avatar.json".into());
        cfg.validate().unwrap();
        cfg.output = OutputMode::Both;
        cfg.validate().unwrap();
        cfg.avatar = Some("tree.json".into());
        assert!(cfg.validate().is_err());
        cfg.avatar = None;
        cfg.output = OutputMode::Unift;
        assert!(cfg.validate().is_err());
        cfg.translation_sheet = None;
        cfg.validate().unwrap();
        assert!(toml::from_str::<FaceConfig>("output = 'typo'").is_err());
    }

    #[cfg(not(feature = "face-xr"))]
    #[test]
    fn unsupported_openxr_fails_explicitly_unless_disabled() {
        let mut cfg = FaceConfig { enabled: true, source: Source::Openxr, ..FaceConfig::default() };
        assert!(cfg.validate().unwrap_err().to_string().contains("--features face-xr"));
        cfg.enabled = false;
        cfg.validate().unwrap();
    }
}
