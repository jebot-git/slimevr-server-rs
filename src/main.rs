//! Native SlimeVR + HaritoraX + oscavmgr server with Shora frontends.
mod app;
mod autobone;
mod calibration;
mod config;
mod control;
mod face;
mod feeder;
mod haritorax;
mod logging;
mod reset;
mod skeleton;
mod settings;
mod smoothing;
mod solarxr;
mod status;
mod tracker;
mod ui;

use clap::Parser;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = config::Cli::parse();
    let mut config = config::Config::load(&cli)?;
    if config.ui == ui::UiMode::Tui && config.log_file.is_none() {
        config.log_file = Some(logging::default_path());
    }
    logging::init(config.log_file.as_deref())?;
    tracing::info!(?config, "slimevr-server-rs starting");
    app::run(config).await
}
