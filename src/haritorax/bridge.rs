//! Feed interpreted samples directly into the native server, retaining SlimeTora IDs.
use std::collections::HashMap;
use std::time::{Duration, Instant};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use crate::{calibration::Calibration, tracker::{TrackerId, TrackerRegistry}, control::Command};
use super::{identity::mac_from_name, tracker_position_id, HaritoraXEvent, serial::InputEvent};

#[derive(Default)]
pub struct TrackerMetadata {
    pub name: String,
    pub port: String,
    pub battery_percent: Option<f32>,
    pub voltage_v: Option<f32>,
}
#[derive(Default)]
pub struct Bridge {
    pub trackers: HashMap<TrackerId, TrackerMetadata>,
    clicks: HashMap<String, (u32, Instant)>,
}
impl Bridge {
    pub fn handle(&mut self, input: InputEvent, registry: &mut TrackerRegistry,
        calibration: &mut Calibration, assignments: &HashMap<TrackerId, u8>) {
        let name = match &input.event {
            HaritoraXEvent::TrackerConnected { name } | HaritoraXEvent::TrackerDisconnected { name }
            | HaritoraXEvent::Imu { name, .. } | HaritoraXEvent::Battery { name, .. }
            | HaritoraXEvent::Button { name, .. } => name.clone(),
        };
        let id = TrackerId::new(mac_from_name(&name), 0);
        let position = assignments.get(&id).copied().unwrap_or_else(|| tracker_position_id(&name));
        let metadata = self.trackers.entry(id).or_default();
        metadata.name = name.clone(); metadata.port = input.port;
        match input.event {
            HaritoraXEvent::TrackerConnected { .. } => {
                registry.ensure_local(id, position);
                calibration.set_mounting(id, position);
            }
            HaritoraXEvent::TrackerDisconnected { .. } => { registry.remove_local(id); }
            HaritoraXEvent::Imu { sample, .. } => {
                let [x,y,z,w] = sample.rotation;
                if !sample.rotation.iter().all(|x| x.is_finite()) { return; }
                let raw = Quaternion::new(w,x,y,z);
                if raw.norm_squared() < 1e-12 { return; }
                let rotation = crate::tracker::sensor_rotation_to_tracking(UnitQuaternion::from_quaternion(raw));
                let tracker = registry.ensure_local(id, position);
                tracker.rotation = Some(rotation);
                tracker.accel = Some(Vector3::from(sample.acceleration));
                tracker.last_seen = Instant::now();
                calibration.set_mounting(id, position);
            }
            HaritoraXEvent::Battery { voltage_v, percentage, .. } => {
                if percentage.is_finite() { metadata.battery_percent = Some((percentage * 100.0).clamp(0.0, 100.0)); }
                if voltage_v.is_finite() { metadata.voltage_v = Some(voltage_v); }
            }
            HaritoraXEvent::Button { button, .. } if button == "main" => {
                let entry = self.clicks.entry(name).or_insert((0, Instant::now()));
                entry.0 += 1; entry.1 = Instant::now();
            }
            _ => {}
        }
    }
    pub fn take_actions(&mut self) -> Vec<Command> {
        let mut actions = Vec::new();
        self.clicks.retain(|_, (count, at)| {
            if at.elapsed() < Duration::from_millis(500) { return true; }
            actions.push(match count { 1 => Command::YawReset, 2 => Command::FullReset,
                3 => Command::MountingReset, _ => Command::PauseTracking });
            false
        });
        actions
    }
    pub fn clear(&mut self, registry: &mut TrackerRegistry) {
        for id in self.trackers.keys() { registry.remove_local(*id); }
        self.trackers.clear(); self.clicks.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::skeleton;
    use super::super::gx6::Decoder;
    use base64::Engine as _;

    #[test]
    fn haritorax_calibration_preserves_forward_bends_knee_lifts_and_turns() {
        use firmware_protocol::ActionType;
        use super::super::decode_imu;
        let yaw = |angle| UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
        let pitch = |angle| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        for heading in [-1.2, 0.0, 0.9] {
            for (name, position, ski_pitch) in [("chest", 4, -0.5), ("hip", 6, -0.4),
                ("leftKnee", 7, 0.6), ("rightKnee", 8, 0.6),
                ("leftAnkle", 9, -0.4), ("rightAnkle", 10, -0.4)] {
                let mut bridge = Bridge::default();
                let mut registry = TrackerRegistry::default();
                let mut calibration = Calibration::new();
                let id = TrackerId::new(mac_from_name(name), 0);
                // Arbitrary fixed sensor mounting, including tilt and facing.
                let mounting = UnitQuaternion::from_euler_angles(0.35, -0.8, 0.2);
                let mut feed = |body: UnitQuaternion<f32>, registry: &mut TrackerRegistry, calibration: &mut Calibration| {
                    // Wire orientation uses the IMU world axes. Encode a real
                    // Haritora frame; the decoder must retain that convention.
                    let wire = pitch(std::f32::consts::FRAC_PI_2) * body * mounting;
                    let mut bytes = [0u8; 14];
                    for (i, component) in [wire.i, wire.j, -wire.k, -wire.w].iter().enumerate() {
                        bytes[i*2..i*2+2].copy_from_slice(&((component * 18000.0).round() as i16).to_le_bytes());
                    }
                    let sample = decode_imu(&bytes, true).unwrap();
                    bridge.handle(InputEvent { port: "/dev/test".into(),
                        event: HaritoraXEvent::Imu { name: name.into(), sample } }, registry, calibration, &HashMap::new());
                };
                let reference = yaw(heading);
                let hmd = [0.0, 1.7, 0.0, reference.i, reference.j, reference.k, reference.w];
                feed(reference, &mut registry, &mut calibration);
                crate::reset::handle_user_action(&registry, &mut calibration, Some(&hmd), &ActionType::Reset);
                feed(reference * pitch(ski_pitch), &mut registry, &mut calibration);
                crate::reset::handle_user_action(&registry, &mut calibration, Some(&hmd), &ActionType::ResetMounting);
                for (turn, bend) in [(0.0, 0.0), (0.0, -0.7), (0.0, 0.8), (0.7, 0.0), (-0.4, 0.5)] {
                    let expected = yaw(heading + turn) * pitch(bend);
                    feed(expected, &mut registry, &mut calibration);
                    let pose = skeleton::solve_pose(registry.iter().cloned(), &calibration, 1.8, Some(&hmd));
                    let part = skeleton::body_part_for_bone(skeleton::bone_kind_for_position(position).unwrap()).unwrap();
                    let actual = pose[&part].rotation;
                    assert!(actual.angle_to(&expected) < 0.002,
                        "{name}: heading={heading} turn={turn} bend={bend}, error={} rad", actual.angle_to(&expected));
                    assert!(calibration.is_calibrated(id));
                }
            }
        }
    }

    #[test]
    fn interpreted_samples_drive_distinct_bones_with_assignment_overrides() {
        let mut decoder = Decoder::default();
        let mut bridge = Bridge::default();
        let mut registry = TrackerRegistry::default();
        let mut calibration = Calibration::new();
        let assignments = HashMap::from([(TrackerId::new(mac_from_name("leftAnkle"), 0), 11)]);
        let mut bytes = [0; 30];
        bytes[6..8].copy_from_slice(&(-18000_i16).to_le_bytes());
        bytes[16..18].copy_from_slice(&9000_i16.to_le_bytes());
        bytes[22..24].copy_from_slice(&(-15588_i16).to_le_bytes());
        let frame = base64::engine::general_purpose::STANDARD.encode(bytes);
        for line in ["r0:0000300000".into(), format!("x0:{frame}"),
            "r1:0000900000".into(), format!("x1:{frame}")] {
            for event in decoder.line(&line, true) {
                bridge.handle(InputEvent { port: "/dev/test".into(), event }, &mut registry, &mut calibration, &assignments);
            }
        }
        assert_eq!(registry.iter().count(), 3);
        let pose = skeleton::solve_pose(registry.iter().cloned(), &calibration, 1.8, None);
        for tracker in registry.iter() {
            let expected = calibration.adjust(tracker.id(), tracker.rotation.unwrap());
            let bone = skeleton::bone_kind_for_position(tracker.position).unwrap();
            let part = skeleton::body_part_for_bone(bone).unwrap();
            assert!(pose[&part].rotation.angle_to(&expected) < 1e-4);
        }
        assert_eq!(registry.iter().find(|t| t.mac == mac_from_name("leftAnkle")).unwrap().position, 11);
        for event in decoder.disconnect() {
            bridge.handle(InputEvent { port: "/dev/test".into(), event }, &mut registry, &mut calibration, &assignments);
        }
        assert_eq!(registry.iter().count(), 0);
    }
}
