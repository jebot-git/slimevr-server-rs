//! SlimeVR tracker UDP protocol server ("Hey OVR =D 5" handshake family).
//!
//! This is the tracker→server half of the SlimeVR firmware protocol (v13). It
//! reuses the [`firmware_protocol`] crate's packet types for the well-formed
//! packets, and reads the raw payload for `SENSOR_INFO` (whose upstream struct
//! omits the v13 `tracker_position`/mag fields).

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::RwLock;
use std::time::Duration;

use firmware_protocol::deku::DekuContainerRead as _;
use firmware_protocol::deku::DekuContainerWrite as _;
use firmware_protocol::{CbPacket, Packet, SbPacket};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tokio::net::UdpSocket;

use super::TrackerRegistry;
use crate::calibration::Calibration;
use crate::reset;

/// Server-bound packet type tags (see `firmware_protocol::packet_type` in shora).
const TAG_SENSOR_INFO: i32 = 15;

/// Sequence number of the server's `"Hey OVR =D 5"` handshake response.
/// `u64::from_be_bytes(*b" OVR =D ")`.
const HANDSHAKE_RESPONSE_SEQ: u64 = 2_328_174_443_102_028_832;

/// Run the tracker UDP server forever, updating `registry` and reacting to user
/// actions (resets) through `calib`. `assignments` overrides a tracker's
/// self-reported body-part position (manual assignment).
pub async fn run(
    bind: SocketAddr,
    registry: Arc<RwLock<TrackerRegistry>>,
    calib: Arc<RwLock<Calibration>>,
    assignments: Arc<HashMap<[u8; 6], u8>>,
    ping_interval_secs: u64,
    tracker_timeout_secs: u64,
) -> anyhow::Result<()> {
    let socket = Arc::new(UdpSocket::bind(bind).await?);
    tracing::info!("tracker UDP server listening on {bind}");

    // Liveness pings must come from this same socket so trackers see the correct
    // source address, answer with a PONG, and keep `last_seen` fresh.
    tokio::spawn(ping_loop(
        socket.clone(),
        registry.clone(),
        ping_interval_secs,
        tracker_timeout_secs,
    ));

    let mut buf = [0u8; 2048];
    loop {
        let Ok((len, src)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        let data = &buf[..len];

        if let Some(tag) = read_tag(data) {
            if tag == TAG_SENSOR_INFO {
                handle_sensor_info_raw(data, src, &registry, &calib, &assignments);
                continue;
            }
            // Log non-rotation packet tags once each for debugging.
            if tag != 17 && tag != 4 {
                tracing::debug!(?src, tag, len, "tracker packet");
            }
        }

        // Reuse the firmware_protocol types for everything else.
        let Ok(((_, _), packet)) = Packet::<SbPacket>::from_bytes((data, 0)) else {
            tracing::debug!(?src, len, "undecodable tracker packet");
            continue;
        };

        let (_, sb) = packet.split();
        match sb {
            SbPacket::Handshake { mac_address, .. } => {
                {
                    let mut reg = registry.write().unwrap();
                    reg.register_handshake(src, mac_address);
                }
                send_handshake_response(&socket, src).await;
                tracing::info!(?src, mac = ?mac_address, "tracker handshake");
            }
            SbPacket::Ping { .. } => {
                registry.write().unwrap().mark_seen(src, 0);
            }
            SbPacket::RotationData {
                sensor_id, quat, ..
            } => {
                let q = UnitQuaternion::from_quaternion(Quaternion::new(
                    quat.w, quat.i, quat.j, quat.k,
                ));
                registry
                    .write()
                    .unwrap()
                    .update_rotation(src, sensor_id, q);
            }
            SbPacket::Acceleration {
                vector,
                sensor_id,
            } => {
                let a = Vector3::new(vector.0, vector.1, vector.2);
                registry.write().unwrap().update_accel(src, sensor_id, a);
            }
            SbPacket::Heartbeat => {
                registry.write().unwrap().mark_seen(src, 0);
            }
            SbPacket::UserAction { action } => {
                tracing::info!(?src, ?action, "tracker user action");
                let reg = registry.read().unwrap();
                let mut cal = calib.write().unwrap();
                reset::handle_user_action(&reg, &mut cal, &action);
            }
            _ => {}
        }
    }
}

/// Read the packet type tag (i32 BE) without a full parse.
fn read_tag(buf: &[u8]) -> Option<i32> {
    if buf.len() < 4 {
        return None;
    }
    Some(i32::from_be_bytes(buf[0..4].try_into().unwrap()))
}

/// Parse the `SENSOR_INFO` payload directly, because the upstream
/// `firmware_protocol::SbPacket::SensorInfo` struct only covers the first three
/// fields (sensor_id, status, type) and drops `sensor_config`, the
/// rest-calibration flag, and `tracker_position`.
fn handle_sensor_info_raw(
    buf: &[u8],
    src: SocketAddr,
    registry: &Arc<RwLock<TrackerRegistry>>,
    calib: &Arc<RwLock<Calibration>>,
    assignments: &Arc<HashMap<[u8; 6], u8>>,
) {
    // payload starts after the 12-byte header.
    if buf.len() < 19 {
        return;
    }
    let sensor_id = buf[12];
    // buf[13] = status, buf[14] = type, buf[15..17] = mag config,
    // buf[17] = hasCompletedRestCalibration, buf[18] = tracker_position.
    let reported_position = buf[18];

    // Manual assignment (config) overrides the tracker's self-reported position.
    let mac = registry.read().unwrap().get(src, 0).map(|t| t.mac);
    let position = mac
        .and_then(|m| assignments.get(&m).copied())
        .unwrap_or(reported_position);

    let mac = registry
        .write()
        .unwrap()
        .update_sensor_info(src, sensor_id, position);
    // Frame alignment: set the mounting orientation from the tracker's body part.
    if let Some(mac) = mac {
        calib.write().unwrap().set_mounting(mac, position);
        tracing::info!(?src, ?mac, position, "SENSOR_INFO");
    }
}

/// Respond to a handshake with the `"Hey OVR =D 5"` magic.
async fn send_handshake_response(socket: &UdpSocket, src: SocketAddr) {
    let resp = Packet::new(
        HANDSHAKE_RESPONSE_SEQ,
        CbPacket::HandshakeResponse { version: b'5' },
    );
    if let Ok(bytes) = resp.to_bytes() {
        let _ = socket.send_to(&bytes, src).await;
    }
}

/// Periodically ping every tracker (from the shared socket) and evict trackers
/// that stop answering.
async fn ping_loop(
    socket: Arc<UdpSocket>,
    registry: Arc<RwLock<TrackerRegistry>>,
    interval_secs: u64,
    timeout_secs: u64,
) {
    let mut tick = tokio::time::interval(Duration::from_secs(interval_secs));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

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

        let removed = registry
            .write()
            .unwrap()
            .remove_stale(Duration::from_secs(timeout_secs));
        if removed > 0 {
            tracing::info!(removed, timeout_secs, "evicted timed-out trackers");
        }
    }
}
