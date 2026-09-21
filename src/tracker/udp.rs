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

use super::{SensorStatus, TrackerId, TrackerRegistry};
use crate::calibration::Calibration;
use crate::feeder::HmdPose;
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
    hmd: Arc<RwLock<Option<HmdPose>>>,
    assignments: Arc<HashMap<TrackerId, u8>>,
    ping_interval_secs: u64,
    tracker_timeout_secs: u64,
) -> anyhow::Result<()> {
    let socket = UdpSocket::bind(bind).await?;
    tracing::info!("tracker UDP server listening on {}", socket.local_addr()?);
    serve(socket, registry, calib, hmd, assignments, ping_interval_secs, tracker_timeout_secs).await
}

async fn serve(
    socket: UdpSocket,
    registry: Arc<RwLock<TrackerRegistry>>,
    calib: Arc<RwLock<Calibration>>,
    hmd: Arc<RwLock<Option<HmdPose>>>,
    assignments: Arc<HashMap<TrackerId, u8>>,
    ping_interval_secs: u64,
    tracker_timeout_secs: u64,
) -> anyhow::Result<()> {
    anyhow::ensure!(ping_interval_secs > 0, "ping interval must be positive");
    let mut tick = tokio::time::interval(Duration::from_secs(ping_interval_secs));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    let mut buf = [0u8; 2048];
    loop {
        let (len, src) = tokio::select! {
            packet = socket.recv_from(&mut buf) => packet?,
            _ = tick.tick() => {
                // Share the receiving socket so ping echoes return to this server.
                // Keep maintenance in this task so cancellation also stops pings.
                ping_devices(&socket, &registry, tracker_timeout_secs).await;
                continue;
            }
        };
        let data = &buf[..len];

        if let Some(tag) = read_tag(data) {
            if tag == TAG_SENSOR_INFO {
                if let Some(ack) = handle_sensor_info_raw(data, src, &registry, &calib, &assignments) {
                    socket.send_to(&ack, src).await?;
                }
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
                registry.write().unwrap().mark_seen(src);
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
                    .update_rotation(src, sensor_id, super::sensor_rotation_to_tracking(q));
            }
            SbPacket::Acceleration {
                vector,
                sensor_id,
            } => {
                let a = Vector3::new(vector.0, vector.1, vector.2);
                registry.write().unwrap().update_accel(src, sensor_id, a);
            }
            SbPacket::Heartbeat => {
                registry.write().unwrap().mark_seen(src);
            }
            SbPacket::UserAction { action } => {
                tracing::info!(?src, ?action, "tracker user action");
                let reg = registry.read().unwrap();
                let mut cal = calib.write().unwrap();
                let hmd_pose = hmd.read().unwrap();
                reset::handle_user_action(&reg, &mut cal, hmd_pose.as_ref(), &action);
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
    assignments: &Arc<HashMap<TrackerId, u8>>,
) -> Option<[u8; 6]> {
    // Only id + status are mandatory; older firmware omits the later fields.
    if buf.len() < 14 {
        return None;
    }
    let sensor_id = buf[12];
    let status = SensorStatus::from_wire(buf[13])?;
    let mut reg = registry.write().unwrap();
    let mac = reg.device_mac(src)?;
    let id = TrackerId::new(mac, sensor_id);
    let position = assignments.get(&id).copied().or_else(|| buf.get(18).copied());
    reg.update_sensor_info(src, sensor_id, position, status);
    let position = reg.get(src, sensor_id).unwrap().position;
    drop(reg);
    calib.write().unwrap().set_mounting(id, position);
    tracing::debug!(?src, ?id, position, ?status, "SENSOR_INFO");
    // Unlike ordinary packets, SENSOR_INFO acknowledgements have no sequence.
    // SlimeVR's UDPProtocolParser.writeSensorInfoResponse writes tag + id + status.
    Some([0, 0, 0, TAG_SENSOR_INFO as u8, sensor_id, buf[13]])
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

/// Ping once per device and evict expired sensors.
async fn ping_devices(
    socket: &UdpSocket,
    registry: &Arc<RwLock<TrackerRegistry>>,
    timeout_secs: u64,
) {
    let addresses: Vec<_> = registry.read().unwrap().device_addresses().collect();
    let pkt = Packet::new(0, CbPacket::Ping { challenge: [0u8; 4] });
    if let Ok(bytes) = pkt.to_bytes() {
        for addr in addresses {
            let _ = socket.send_to(&bytes, addr).await;
        }
    }
    let removed = registry.write().unwrap().remove_stale(Duration::from_secs(timeout_secs));
    if removed > 0 {
        tracing::info!(removed, timeout_secs, "evicted timed-out trackers");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firmware_protocol::{BoardType, ImuType, McuType, SensorDataType, SlimeQuaternion, SlimeString};

    fn sensor_info(sensor_id: u8, status: u8, position: Option<u8>) -> Vec<u8> {
        let mut bytes = vec![0; 12];
        bytes[..4].copy_from_slice(&TAG_SENSOR_INFO.to_be_bytes());
        bytes.extend_from_slice(&[sensor_id, status]);
        if let Some(position) = position {
            bytes.extend_from_slice(&[0, 0, 0, 0, position]);
        }
        bytes
    }

    #[test]
    fn sensor_info_handles_legacy_packets_assignments_and_invalid_input() {
        let reg = Arc::new(RwLock::new(TrackerRegistry::default()));
        let calib = Arc::new(RwLock::new(Calibration::new()));
        let src = "127.0.0.1:12345".parse().unwrap();
        let primary = TrackerId::new([1; 6], 0);
        let extension = TrackerId::new([1; 6], 1);
        let assignments = Arc::new(HashMap::from([(primary, 9), (extension, 10)]));
        let packet = sensor_info(1, 1, Some(4));
        assert!(handle_sensor_info_raw(&packet, src, &reg, &calib, &assignments).is_none());
        reg.write().unwrap().register_handshake(src, [1; 6]);
        for length in 0..14 {
            assert!(handle_sensor_info_raw(&packet[..length], src, &reg, &calib, &assignments).is_none());
        }
        assert!(handle_sensor_info_raw(&sensor_info(1, 255, None), src, &reg, &calib, &assignments).is_none());
        assert_eq!(reg.read().unwrap().iter().count(), 1);
        assert_eq!(handle_sensor_info_raw(&packet, src, &reg, &calib, &assignments), Some([0, 0, 0, 15, 1, 1]));
        handle_sensor_info_raw(&sensor_info(0, 1, None), src, &reg, &calib, &assignments);
        assert_eq!(reg.read().unwrap().get(src, 0).unwrap().position, 9);
        assert_eq!(reg.read().unwrap().get(src, 1).unwrap().position, 10);
        // Short legacy refresh must not erase an already assigned position.
        handle_sensor_info_raw(&sensor_info(1, 1, None), src, &reg, &calib, &Arc::new(HashMap::new()));
        assert_eq!(reg.read().unwrap().get(src, 1).unwrap().position, 10);
    }

    async fn receive(socket: &UdpSocket, expected: &[u8]) {
        tokio::time::timeout(Duration::from_secs(2), async {
            let mut buf = [0; 256];
            loop {
                let (len, _) = socket.recv_from(&mut buf).await.unwrap();
                if &buf[..len] == expected {
                    break;
                }
            }
        }).await.expect("server did not acknowledge packet");
    }

    #[tokio::test]
    async fn udp_extension_rotations_reach_distinct_skeleton_bones() {
        let server = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let addr = server.local_addr().unwrap();
        let client = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let src = client.local_addr().unwrap();
        let reg = Arc::new(RwLock::new(TrackerRegistry::default()));
        let calib = Arc::new(RwLock::new(Calibration::new()));
        let task = tokio::spawn(serve(server, reg.clone(), calib.clone(),
            Arc::new(RwLock::new(None)), Arc::new(HashMap::new()), 2, 5));
        let handshake = Packet::new(0, SbPacket::Handshake {
            board: BoardType::Haritora,
            imu: ImuType::Unknown(0),
            mcu: McuType::Haritora,
            imu_info: (0, 0, 0),
            build: 13,
            firmware: SlimeString::from("multi-sensor-test"),
            mac_address: [1; 6],
        }).to_bytes().unwrap();
        client.send_to(&handshake, addr).await.unwrap();
        receive(&client, b"\x03Hey OVR =D 5").await;
        // Deliberately announce the extension first, then repeat both announcements.
        for _ in 0..2 {
            for (id, position) in [(1, 10), (0, 9)] {
                client.send_to(&sensor_info(id, 1, Some(position)), addr).await.unwrap();
                receive(&client, &[0, 0, 0, 15, id, 1]).await;
            }
        }
        let rotations = [UnitQuaternion::from_euler_angles(0.2, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(-0.4, 0.0, 0.0)];
        for (sensor_id, q) in rotations.iter().enumerate() {
            // Encode desired VR-space poses in the protocol's IMU world frame.
            let wire = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_2) * q;
            let packet = Packet::new(1, SbPacket::RotationData {
                sensor_id: sensor_id as u8,
                data_type: SensorDataType::Normal,
                quat: SlimeQuaternion { i: wire.i, j: wire.j, k: wire.k, w: wire.w },
                calibration_info: 0,
            }).to_bytes().unwrap();
            client.send_to(&packet, addr).await.unwrap();
        }
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if reg.read().unwrap().iter().filter(|t| t.rotation.is_some()).count() == 2 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        }).await.expect("both sensors should receive independent rotations");
        {
            let registry = reg.read().unwrap();
            let calibration = calib.read().unwrap();
            let pose = crate::skeleton::solve_pose(registry.iter().cloned(), &calibration, 1.8, None);
            for (sensor_id, position) in [(0, 9), (1, 10)] {
                let t = registry.get(src, sensor_id).unwrap();
                let expected = calibration.adjust(t.id(), rotations[sensor_id as usize]);
                let bone = crate::skeleton::bone_kind_for_position(position).unwrap();
                let part = crate::skeleton::body_part_for_bone(bone).unwrap();
                assert!(pose[&part].rotation.angle_to(&expected) < 1e-4);
            }
        }
        // Disconnecting an extension clears only its own pose; the primary survives.
        client.send_to(&sensor_info(1, 0, None), addr).await.unwrap();
        receive(&client, &[0, 0, 0, 15, 1, 0]).await;
        assert!(reg.read().unwrap().get(src, 1).unwrap().rotation.is_none());
        assert!(reg.read().unwrap().get(src, 0).unwrap().rotation.is_some());
        task.abort();
        assert!(task.await.unwrap_err().is_cancelled());
    }
}
