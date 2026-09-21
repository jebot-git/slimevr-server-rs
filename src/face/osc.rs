//! Managed face/eye → OSC relay using oscavmgr's expression model and mapping.

use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::Context;
use rosc::{OscBundle, OscPacket};

use super::bundle::{AvatarBundle, PARAM_PREFIX};
use super::config::{FaceConfig, Source};
use super::output::FaceOutput;
use super::unified::{FaceParams, UnifiedTrackingData};
use super::FaceSource;

const FRAME_INTERVAL: Duration = Duration::from_millis(11);
type SourceFactory = Box<dyn FnOnce() -> Box<dyn FaceSource> + Send>;

/// Owns the worker and its sockets. Drop requests stop and joins the worker.
pub struct FaceRelay {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<anyhow::Result<()>>>,
    active: Arc<AtomicBool>,
}

impl FaceRelay {
    pub fn start(cfg: FaceConfig) -> anyhow::Result<Self> {
        cfg.validate()?;
        let output = FaceOutput::load(&cfg)?;
        let listener = UdpSocket::bind(cfg.listen_addr(cfg.osc_port))
            .context("bind face OSC feedback listener")?;
        listener.set_nonblocking(true)?;
        let upstream = UdpSocket::bind(if cfg.destination.is_ipv4() {
            "0.0.0.0:0"
        } else {
            "[::]:0"
        }).context("bind face OSC output socket")?;
        upstream.set_write_timeout(Some(Duration::from_millis(100)))?;

        // Bind Babble before spawning so startup failures reach the caller.
        // OpenXR must construct and drop its handles on the worker thread.
        let factory: SourceFactory = match cfg.source {
            Source::Babble => {
                let source = super::babble::BabbleSource::bind(cfg.listen_addr(cfg.babble_port))?;
                Box::new(move || Box::new(source))
            }
            Source::Openxr => {
                #[cfg(feature = "face-xr")]
                { Box::new(|| Box::new(super::openxr::OpenXrSource::new())) }
                #[cfg(not(feature = "face-xr"))]
                { anyhow::bail!("openxr requires --features face-xr") }
            }
        };
        let advert = if cfg.osc_query {
            match super::oscquery::OscQueryAdvert::new(cfg.osc_port) {
                Ok(advert) => Some(advert),
                Err(error) => {
                    tracing::warn!(%error, "OSCQuery advertisement unavailable; direct OSC remains available");
                    None
                }
            }
        } else { None };
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let active = Arc::new(AtomicBool::new(false));
        let worker_active = active.clone();
        let worker = std::thread::Builder::new().name("face-relay".into()).spawn(move || {
            let _advert = advert;
            let mut source = factory();
            tracing::info!(source = source.name(), output = ?cfg.output, translation_sheet = ?cfg.translation_sheet, destination = %cfg.destination,
                feedback = %listener.local_addr()?, "oscavmgr face relay started");
            run_loop(cfg, listener, upstream, source.as_mut(), output, &worker_stop, &worker_active)
        }).context("spawn face relay")?;
        Ok(Self { stop, worker: Some(worker), active })
    }

    pub fn active(&self) -> bool { self.active.load(Ordering::Relaxed) }

    pub fn check_health(&mut self) -> anyhow::Result<()> {
        if self.worker.as_ref().is_some_and(|worker| worker.is_finished()) {
            match self.worker.take().unwrap().join() {
                Ok(Err(error)) => return Err(error).context("face relay stopped"),
                Ok(Ok(())) => anyhow::bail!("face relay stopped unexpectedly"),
                Err(_) => anyhow::bail!("face relay worker panicked"),
            }
        }
        Ok(())
    }
}

impl Drop for FaceRelay {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            if worker.join().is_err() {
                tracing::error!("face relay worker panicked during shutdown");
            }
        }
    }
}

fn run_loop(
    cfg: FaceConfig,
    listener: UdpSocket,
    upstream: UdpSocket,
    source: &mut dyn FaceSource,
    mut output: FaceOutput,
    stop: &AtomicBool,
    active_flag: &AtomicBool,
) -> anyhow::Result<()> {
    let mut browser = if FaceOutput::discovers_avatar(&cfg) {
        super::oscjson::OscJsonBrowser::new()
    } else { None };
    let mut data = UnifiedTrackingData::default();
    let mut cache = FaceParams::new();
    let mut last_frame = Instant::now();
    let mut last_vsync: Option<Instant> = None;
    let mut last_active = false;
    let mut buf = [0; 65535];
    while !stop.load(Ordering::Relaxed) {
        let started = Instant::now();
        if let Some(browser) = browser.as_mut() {
            if let Some(node) = browser.poll_avatar() {
                output = FaceOutput::from_oscquery(&node);
                output.invalidate(&mut data);
                tracing::debug!("face avatar mapping refreshed");
            }
        }
        let mut vsync = false;
        for _ in 0..128 {
            match listener.recv_from(&mut buf) {
                Ok((len, _)) => {
                    if let Ok((_, packet)) = rosc::decoder::decode_udp(&buf[..len]) {
                        if handle_feedback(&packet, &mut cache, &mut vsync, &mut browser) {
                            output.invalidate(&mut data);
                        }
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e).context("receive face OSC feedback"),
            }
        }
        if vsync { last_vsync = Some(Instant::now()); }
        // Continue at ~90 Hz without VSync, or after its 500 ms watchdog expires.
        if vsync || last_vsync.is_none_or(|at| at.elapsed() > Duration::from_millis(500)) {
            let active = source.receive(&mut data)?;
            active_flag.store(active, Ordering::Relaxed);
            if last_active && !active {
                data.shapes.fill(0.0);
                data.eyes = [None, None];
            }
            if active || last_active {
                data.calc_combined(last_frame.elapsed().as_secs_f32(), &cache);
                let mut bundle = OscBundle::new_bundle();
                output.apply(&mut data, &mut bundle, active);
                send_bundle(&upstream, cfg.destination, bundle);
            }
            if active != last_active {
                tracing::info!(active, "face/eye tracking state changed");
            }
            last_active = active;
            last_frame = Instant::now();
        }
        std::thread::park_timeout(FRAME_INTERVAL.saturating_sub(started.elapsed()));
    }
    if last_active {
        data.shapes.fill(0.0);
        data.eyes = [None, None];
        data.calc_combined(0.0, &cache);
        let mut bundle = OscBundle::new_bundle();
        output.apply(&mut data, &mut bundle, false);
        send_bundle(&upstream, cfg.destination, bundle);
    }
    Ok(())
}

fn handle_feedback(
    packet: &OscPacket,
    cache: &mut FaceParams,
    vsync: &mut bool,
    browser: &mut Option<super::oscjson::OscJsonBrowser>,
) -> bool {
    match packet {
        OscPacket::Bundle(bundle) => {
            let mut changed = false;
            for packet in &bundle.content { changed |= handle_feedback(packet, cache, vsync, browser); }
            return changed;
        }
        OscPacket::Message(message) => {
            if message.addr == "/avatar/change" {
                cache.clear();
                if let Some(browser) = browser { browser.refresh(); }
                return true;
            } else if let Some(name) = message.addr.strip_prefix(PARAM_PREFIX) {
                if name == "VSync" {
                    *vsync = true;
                } else if let Some(value) = message.args.first() {
                    if cache.len() < 512 || cache.contains_key(name) {
                        cache.insert(name.into(), value.clone());
                    }
                }
            }
        }
    }
    false
}

fn send_bundle(socket: &UdpSocket, destination: SocketAddr, bundle: OscBundle) {
    // Keep oscavmgr's <=30 messages per bundle. An unconnected UDP socket lets
    // VRChat start/restart independently without terminating this relay on ICMP.
    for chunk in bundle.content.chunks(30) {
        let packet = OscPacket::Bundle(OscBundle {
            timetag: bundle.timetag,
            content: chunk.to_vec(),
        });
        if let Ok(bytes) = rosc::encoder::encode(&packet) {
            if let Err(error) = socket.send_to(&bytes, destination) {
                tracing::debug!(%error, "face OSC send failed");
            }
        }
    }
}

#[cfg(test)]
mod tests;
