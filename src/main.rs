//! slimevr-server-rs — a from-scratch Rust rewrite of the SlimeVR server.
//!
//! The goal is to replace the Java/Kotlin SlimeVR server with a native Rust
//! binary, using the [SlimeVR-Rust] workspace as boilerplate for the tracker
//! protocol ([`firmware_protocol`]), the skeleton model ([`skeletal_model`]), and
//! the IMU filter ([`vqf`]).
//!
//! [SlimeVR-Rust]: https://github.com/SlimeVR/SlimeVR-Rust

mod calibration;
mod config;
mod reset;
mod skeleton;
mod solarxr;
mod tracker;

use std::net::SocketAddr;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use clap::Parser;
use firmware_protocol::deku::DekuContainerWrite as _;
use firmware_protocol::{CbPacket, Packet};
use tokio::net::UdpSocket;

use crate::calibration::Calibration;
use crate::config::{Cli, Config};
use crate::solarxr::Pose;
use crate::tracker::TrackerRegistry;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Parse args first so `--help`/`--version` exit cleanly before logging starts.
    let cli = Cli::parse();

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let config = Config::load(&cli)?;
    tracing::info!(?config, "slimevr-server-rs starting");

    let registry = Arc::new(RwLock::new(TrackerRegistry::default()));
    let pose: Arc<RwLock<Pose>> = Arc::new(RwLock::new(Pose::default()));
    let calib: Arc<RwLock<Calibration>> = Arc::new(RwLock::new(Calibration::new()));
    let assignments = Arc::new(config.tracker_assignments.clone());

    // 1. Tracker UDP protocol server ("Hey OVR =D 5").
    let _tracker_task = tokio::spawn(tracker::udp::run(
        SocketAddr::from(([0, 0, 0, 0], config.tracker_port)),
        registry.clone(),
        calib.clone(),
        assignments,
    ));

    // 2. SolarXR WebSocket server (WiVRn connects here).
    let _solarxr_task = tokio::spawn(solarxr::run(
        SocketAddr::from(([0, 0, 0, 0], config.solarxr_port)),
        pose.clone(),
        registry.clone(),
        calib.clone(),
    ));

    // 3. Tracker liveness pings.
    let _ping_task = tokio::spawn(ping_trackers(
        registry.clone(),
        config.ping_interval_secs,
        config.tracker_timeout_secs,
    ));

    // 4. Pose estimation loop: tracker rotations → skeleton pose.
    let mut tick = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tick.tick().await;

        let trackers: Vec<_> = registry.read().unwrap().iter().cloned().collect();
        if !trackers.is_empty() {
            tracing::trace!("trackers: {}", trackers.len());
        }
        let new_pose = {
            let calib = calib.read().unwrap();
            skeleton::solve_pose(trackers.into_iter(), &calib, config.height_m)
        };
        *pose.write().unwrap() = new_pose;
    }
}

/// Periodically send a `Ping` to every known tracker and evict any that have
/// stopped responding.
async fn ping_trackers(
    registry: Arc<RwLock<TrackerRegistry>>,
    interval_secs: u64,
    timeout_secs: u64,
) {
    let Ok(socket) = UdpSocket::bind("0.0.0.0:0").await else {
        tracing::error!("failed to bind ping socket");
        return;
    };

    let mut tick = tokio::time::interval(Duration::from_secs(interval_secs));
    loop {
        tick.tick().await;

        let trackers: Vec<_> = registry.read().unwrap().iter().cloned().collect();
        for t in trackers {
            // A real server would use a unique challenge per ping; a constant
            // placeholder is fine for the first milestone.
            let pkt = Packet::new(0, CbPacket::Ping { challenge: [0u8; 4] });
            if let Ok(bytes) = pkt.to_bytes() {
                let _ = socket.send_to(&bytes, t.addr).await;
            }
        }

        // Drop trackers that stopped answering.
        let removed = registry
            .write()
            .unwrap()
            .remove_stale(Duration::from_secs(timeout_secs));
        if removed > 0 {
            tracing::info!(removed, timeout_secs, "evicted timed-out trackers");
        }
    }
}
