use clap::ValueEnum;
use serde::Deserialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, ValueEnum)]
#[serde(rename_all = "snake_case")]
pub enum Model {
    #[value(name = "x2")]
    X2,
    Wireless,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct HaritoraXConfig {
    pub enabled: bool,
    /// Empty selects every discovered GX6/GX2 serial interface.
    pub ports: Vec<String>,
    pub model: Model,
    pub baud_rate: u32,
}

impl Default for HaritoraXConfig {
    fn default() -> Self {
        Self { enabled: false, ports: Vec::new(), model: Model::X2, baud_rate: 500_000 }
    }
}

impl HaritoraXConfig {
    pub fn validate(&self) -> anyhow::Result<()> {
        anyhow::ensure!((1..=4_000_000).contains(&self.baud_rate), "invalid HaritoraX baud_rate");
        anyhow::ensure!(self.ports.iter().all(|port| !port.trim().is_empty()), "serial port paths cannot be empty");
        Ok(())
    }
}
