//! Shared snapshot used by the Shora TUI and Qt6 control client.
use std::sync::{Arc, RwLock};
use serde::Serialize;
use crate::{haritorax::{bridge::Bridge, serial::PortStatus}, tracker::TrackerRegistry};

pub type StatusHandle = Arc<RwLock<StatusSnapshot>>;
#[derive(Debug, Clone, Serialize)]
pub struct TrackerInfo {
    pub name: String,
    pub mac: String,
    pub sensor_id: u8,
    pub source: String,
    pub position: u8,
    pub active: bool,
    pub age_ms: u64,
    pub battery_percent: Option<f32>,
}
#[derive(Debug, Clone, Serialize)]
pub struct BoneInfo {
    pub body_part: u8,
    pub head: [f32; 3],
    pub tail: [f32; 3],
    /// Quaternion in x, y, z, w order, matching SolarXR.
    pub rotation: [f32; 4],
    pub length: f32,
    pub tracked: bool,
}
#[derive(Debug, Clone, Default, Serialize)]
pub struct StatusSnapshot {
    pub version: String,
    pub pid: u32,
    pub uptime_secs: u64,
    pub paused: bool,
    pub trackers: Vec<TrackerInfo>,
    pub haritorax_enabled: bool,
    pub ports: Vec<PortStatus>,
    pub face_source: String,
    pub face_output: String,
    pub face_active: bool,
    pub hmd_pose_received: bool,
    pub bone_count: usize,
    pub bones: Vec<BoneInfo>,
    pub settings: crate::settings::TrackingSettings,
    pub calibrated_trackers: usize,
    pub drift_samples: usize,
    pub tracker_port: u16,
    pub solarxr_socket: String,
    pub feeder_socket: String,
    pub control_socket: Option<String>,
    pub log_file: Option<String>,
    pub last_action: String,
}
impl StatusSnapshot {
    pub fn refresh_pose(&mut self, pose: &crate::solarxr::Pose, registry: &TrackerRegistry) {
        let tracked: std::collections::HashSet<_> = registry.iter().filter(|t| t.rotation.is_some())
            .filter_map(|t| crate::skeleton::bone_kind_for_position(t.position))
            .filter_map(crate::skeleton::body_part_for_bone).collect();
        self.bones = pose.iter().map(|(&body_part, bone)| {
            let offset = bone.rotation * nalgebra::Vector3::new(0.0, -bone.length, 0.0);
            BoneInfo { body_part, head: bone.head_pos,
                tail: [bone.head_pos[0] + offset.x, bone.head_pos[1] + offset.y, bone.head_pos[2] + offset.z],
                rotation: [bone.rotation.i, bone.rotation.j, bone.rotation.k, bone.rotation.w],
                length: bone.length, tracked: tracked.contains(&body_part) }
        }).collect();
        self.bones.sort_by_key(|b| b.body_part);
        self.bone_count = self.bones.len();
    }
    pub fn refresh_trackers(&mut self, registry: &TrackerRegistry, bridge: &Bridge) {
        self.trackers = registry.iter().map(|t| {
            let meta = bridge.trackers.get(&t.id());
            let mac = t.mac.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(":");
            TrackerInfo {
                name: meta.map(|m| m.name.clone()).unwrap_or_else(|| format!("{mac}/{}", t.sensor_id)),
                mac, sensor_id: t.sensor_id,
                source: meta.map(|m| format!("HaritoraX {}", m.port)).unwrap_or_else(|| format!("UDP {}", t.addr)),
                position: t.position, active: t.rotation.is_some(),
                age_ms: t.last_seen.elapsed().as_millis() as u64,
                battery_percent: meta.and_then(|m| m.battery_percent),
            }
        }).collect();
        self.trackers.sort_by(|a,b| a.name.cmp(&b.name).then(a.sensor_id.cmp(&b.sensor_id)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn preview_endpoints_use_solved_rotation_and_length() {
        let mut pose = crate::solarxr::Pose::new();
        pose.insert(3, crate::skeleton::BonePose {
            head_pos: [1.0, 2.0, 3.0], length: 0.4,
            rotation: nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), std::f32::consts::FRAC_PI_2),
        });
        let mut registry = TrackerRegistry::default();
        let tracker = registry.ensure_local(crate::tracker::TrackerId::new([1; 6], 0), 4);
        tracker.rotation = Some(nalgebra::UnitQuaternion::identity());
        let mut snapshot = StatusSnapshot::default();
        snapshot.refresh_pose(&pose, &registry);
        assert_eq!(snapshot.bone_count, 1);
        let bone = &snapshot.bones[0];
        assert!(bone.tracked);
        assert!((bone.tail[0] - 1.4).abs() < 1e-5);
        assert!((bone.tail[1] - 2.0).abs() < 1e-5);
        assert_eq!(bone.tail[2], 3.0);
    }
}
