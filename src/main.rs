//! slimevr-server-rs — a from-scratch Rust rewrite of the SlimeVR server.
//!
//! The goal is to replace the Java/Kotlin SlimeVR server with a native Rust
//! binary, using the [SlimeVR-Rust] workspace as boilerplate for the tracker
//! protocol ([`firmware_protocol`]), the skeleton model ([`skeletal_model`]), and
//! the IMU filter ([`vqf`]).
//!
//! [SlimeVR-Rust]: https://github.com/SlimeVR/SlimeVR-Rust

mod autobone;
mod calibration;
mod config;
mod feeder;
mod reset;
mod skeleton;
mod solarxr;
mod tracker;

use std::net::SocketAddr;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use clap::Parser;
use skeletal_model::BoneMap;

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
    let hmd: Arc<RwLock<Option<feeder::HmdPose>>> = Arc::new(RwLock::new(None));
    // Current bone lengths (start height-autoboned; replaced by autobone::optimize).
    let lengths: Arc<RwLock<BoneMap<f32>>> = Arc::new(RwLock::new(skeleton::bone_lengths_from_height(
        config.height_m,
    )));
    let autobone: Arc<RwLock<autobone::AutoboneState>> =
        Arc::new(RwLock::new(autobone::AutoboneState::default()));

    // 1. Tracker UDP protocol server ("Hey OVR =D 5"; also handles pings/eviction).
    let _tracker_task = tokio::spawn(tracker::udp::run(
        SocketAddr::from(([0, 0, 0, 0], config.tracker_port)),
        registry.clone(),
        calib.clone(),
        assignments,
        config.ping_interval_secs,
        config.tracker_timeout_secs,
    ));

    // 2. SolarXR IPC server (WiVRn connects to this Unix domain socket).
    let _solarxr_task = tokio::spawn(solarxr::run(
        config.solarxr_socket.clone(),
        pose.clone(),
        registry.clone(),
        calib.clone(),
        hmd.clone(),
        lengths.clone(),
        autobone.clone(),
        config.height_m,
    ));

    // 3. SteamVR feeder bridge (WiVRn sends the HMD pose here).
    let _feeder_task = tokio::spawn(feeder::run(config.feeder_socket.clone(), hmd.clone()));

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
            let lengths = lengths.read().unwrap();
            skeleton::solve_pose_with_lengths(trackers.clone().into_iter(), &calib, *lengths)
        };
        *pose.write().unwrap() = new_pose;

        // Autobone recording (frames are appended while `recording` is set).
        let mut ab = autobone.write().unwrap();
        if ab.recording {
            ab.frames.push(autobone::Frame { trackers });
        }
    }
}
