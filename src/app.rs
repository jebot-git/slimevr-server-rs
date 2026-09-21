//! Unified native runtime and shared Shora UI command handling.
use std::net::SocketAddr;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use firmware_protocol::ActionType;
use tokio::sync::mpsc;
use crate::{autobone, calibration::Calibration, config::Config, control::{Command, Envelope, ControlServer},
    face, feeder, haritorax::{bridge::Bridge, serial::Acquisition}, reset, skeleton, smoothing,
    solarxr::{self, Pose}, status::{StatusHandle, StatusSnapshot}, tracker::{self, TrackerRegistry}, ui::{UiMode, TuiHandle}};

pub async fn run(config: Config) -> anyhow::Result<()> {
    let started = Instant::now();
    let registry = Arc::new(RwLock::new(TrackerRegistry::default()));
    let pose = Arc::new(RwLock::new(Pose::default()));
    let calib = Arc::new(RwLock::new(Calibration::new()));
    let mut settings = config.tracking_settings();
    calib.write().unwrap().configure_drift(settings.drift_correction, settings.drift_amount);
    let target_height = Arc::new(RwLock::new(settings.height_m));
    let assignments = Arc::new(config.tracker_assignments.clone());
    let hmd = Arc::new(RwLock::new(None));
    let lengths = Arc::new(RwLock::new(skeleton::bone_lengths_from_height(config.height_m)));
    let autobone = Arc::new(RwLock::new(autobone::AutoboneState::default()));
    let status: StatusHandle = Arc::new(RwLock::new(StatusSnapshot {
        version: env!("CARGO_PKG_VERSION").into(), pid: std::process::id(), tracker_port: config.tracker_port,
        solarxr_socket: config.solarxr_socket.clone(), feeder_socket: config.feeder_socket.clone(),
        control_socket: config.control_socket.as_ref().map(|p| p.display().to_string()),
        log_file: config.log_file.as_ref().map(|p| p.display().to_string()),
        face_source: if config.face.enabled { format!("{:?}", config.face.source).to_lowercase() } else { "disabled".into() },
        face_output: if config.face.translation_sheet.is_some() && config.face.output == face::config::OutputMode::Auto {
            "json".into()
        } else { format!("{:?}", config.face.output).to_lowercase() },
        haritorax_enabled: config.haritorax.enabled,
        settings,
        ..Default::default()
    }));
    let (commands, mut command_rx) = mpsc::channel::<Envelope>(32);
    // Claim the control endpoint before opening hardware or starting services.
    let control = if let Some(path) = &config.control_socket { Some(ControlServer::bind(path.clone()).await?) } else { None };
    let mut face_relay = if config.face.enabled { Some(face::FaceRelay::start(config.face.clone())?) } else { None };
    let mut acquisition = Acquisition::start(config.haritorax.clone())?;
    let mut bridge = Bridge::default();
    let mut services = tokio::task::JoinSet::new();
    if let Some(control) = control { services.spawn(control.run(status.clone(), commands.clone())); }
    services.spawn(tracker::udp::run(SocketAddr::from(([0,0,0,0],config.tracker_port)),
        registry.clone(), calib.clone(), hmd.clone(), assignments.clone(), config.ping_interval_secs, config.tracker_timeout_secs));
    services.spawn(solarxr::run(config.solarxr_socket.clone(), pose.clone(), registry.clone(),
        calib.clone(), hmd.clone(), lengths.clone(), autobone.clone(), target_height.clone()));
    services.spawn(feeder::run(config.feeder_socket.clone(), hmd.clone()));
    let mut tui = if config.ui == UiMode::Tui { Some(TuiHandle::start(status.clone(), commands.clone())?) } else { None };
    let mut filter = smoothing::RotationFilter::new(config.smoothing, config.prediction);
    let mut tick = tokio::time::interval(Duration::from_millis(33));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let shutdown = tokio::signal::ctrl_c(); tokio::pin!(shutdown);
    let mut paused = false;
    loop {
        let mut pending = None;
        let frame_due = tokio::select! {
            result = &mut shutdown => { result?; break; }
            result = services.join_next() => {
                match result {
                    Some(Ok(Err(error))) => return Err(error.context("server service failed")),
                    Some(Err(error)) => return Err(error.into()),
                    _ => anyhow::bail!("server service stopped unexpectedly"),
                }
            }
            Some(event) = acquisition.events.recv() => {
                bridge.handle(event, &mut registry.write().unwrap(), &mut calib.write().unwrap(), &assignments);
                false
            }
            Some(command) = command_rx.recv() => { pending = Some(command); false }
            _ = tick.tick() => true,
        };
        if let Some(relay) = face_relay.as_mut() { relay.check_health()?; }
        if let Some(tui) = tui.as_mut() { tui.check_health()?; }
        if frame_due {
            for command in bridge.take_actions() { let _ = commands.try_send(command.into()); }
        }
        let mut stop = false;
        let mut reply = None;
        if let Some(envelope) = pending {
            let result = match envelope.command {
                Command::Status => Ok(()),
                Command::Shutdown => { stop = true; Ok(()) }
                Command::ShutdownTrackers => acquisition.shutdown_trackers().await,
                Command::PauseTracking => { paused = !paused; Ok(()) }
                Command::SetSettings { settings: next } => {
                    match next.validate() {
                        Err(error) => Err(error),
                        Ok(()) => {
                            // Validate the whole update before changing any live state.
                            if next.smoothing != settings.smoothing || next.prediction != settings.prediction {
                                filter = smoothing::RotationFilter::new(next.smoothing, next.prediction);
                            }
                            if next.height_m != settings.height_m {
                                *lengths.write().unwrap() = skeleton::bone_lengths_from_height(next.height_m);
                                *target_height.write().unwrap() = next.height_m;
                            }
                            calib.write().unwrap().configure_drift(next.drift_correction, next.drift_amount);
                            settings = next;
                            Ok(())
                        }
                    }
                }
                Command::ClearDrift => { calib.write().unwrap().clear_drift(); Ok(()) }
                Command::Restart => {
                    let result = acquisition.restart();
                    bridge.clear(&mut registry.write().unwrap());
                    result
                }
                command => {
                    let action = match command { Command::YawReset => ActionType::ResetYaw,
                        Command::FullReset => ActionType::Reset, Command::MountingReset => ActionType::ResetMounting, _ => unreachable!() };
                    let reg = registry.read().unwrap();
                    if reg.iter().all(|t| t.rotation.is_none()) { Err(anyhow::anyhow!("no tracker samples available for calibration")) }
                    else { reset::handle_user_action(&reg, &mut calib.write().unwrap(), hmd.read().unwrap().as_ref(), &action); Ok(()) }
                }
            };
            status.write().unwrap().last_action = match &result {
                Ok(()) => match envelope.command { Command::SetSettings { .. } => "Tracking settings applied".into(),
                    Command::ShutdownTrackers => "Tracker shutdown commands sent".into(),
                    other => format!("{other:?}") },
                Err(e) => format!("{e:#}")
            };
            reply = envelope.reply.map(|tx| (tx, result.map_err(|e| format!("{e:#}"))));
        }
        if frame_due && !paused {
            let mut trackers: Vec<_> = registry.read().unwrap().iter().cloned().collect();
            let now = Instant::now();
            for t in &mut trackers { if let Some(rot) = t.rotation { t.rotation = Some(filter.apply(t.id(), rot, now)); } }
            let new_pose = skeleton::solve_pose_with_lengths(trackers.clone().into_iter(),
                &calib.read().unwrap(), *lengths.read().unwrap(), hmd.read().unwrap().as_ref());
            *pose.write().unwrap() = new_pose;
            let mut ab = autobone.write().unwrap();
            if ab.recording { ab.frames.push(autobone::Frame { trackers }); }
        }
        if frame_due || reply.is_some() {
            let mut snapshot = status.write().unwrap();
            snapshot.uptime_secs = started.elapsed().as_secs(); snapshot.paused = paused;
            snapshot.refresh_trackers(&registry.read().unwrap(), &bridge);
            snapshot.ports = acquisition.ports();
            snapshot.settings = settings;
            let reg = registry.read().unwrap();
            snapshot.refresh_pose(&pose.read().unwrap(), &reg);
            let calibration = calib.read().unwrap();
            snapshot.calibrated_trackers = reg.iter().filter(|t| calibration.is_calibrated(t.id())).count();
            snapshot.drift_samples = reg.iter().filter(|t| calibration.has_drift_sample(t.id())).count();
            snapshot.hmd_pose_received = hmd.read().unwrap().is_some();
            snapshot.face_active = face_relay.as_ref().is_some_and(|relay| relay.active());
        }
        if let Some((tx, result)) = reply { let _ = tx.send(result); }
        if stop {
            // Allow the control reply to flush before cancellation closes clients.
            tokio::time::sleep(Duration::from_millis(30)).await;
            break;
        }
    }
    drop(tui);
    services.abort_all();
    while services.join_next().await.is_some() {}
    tracing::info!("native tracking hub stopped");
    Ok(())
}
