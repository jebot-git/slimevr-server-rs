//! Connected devices and independent per-sensor tracking state.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::{Duration, Instant};

use nalgebra::{UnitQuaternion, Vector3};

pub mod udp;

/// Convert a SlimeVR wire quaternion from IMU world axes to VR world axes.
/// Matches TrackersUDPServer.AXES_OFFSET: a -90 degree X rotation applied
/// on the left (world frame only, not a quaternion basis conjugation).
/// HaritoraX's interpreter emits this same wire frame; the OpenXR/HMD feeder
/// already uses VR axes and must not receive this conversion.
pub fn sensor_rotation_to_tracking(rotation: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2) * rotation
}

/// Stable identity of a sensor, including extensions on the same device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TrackerId {
    pub mac: [u8; 6],
    pub sensor_id: u8,
}

impl TrackerId {
    pub const fn new(mac: [u8; 6], sensor_id: u8) -> Self {
        Self { mac, sensor_id }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorStatus {
    Disconnected,
    Ok,
    Error,
}

impl SensorStatus {
    pub fn from_wire(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disconnected),
            1 => Some(Self::Ok),
            2 => Some(Self::Error),
            _ => None,
        }
    }
}

/// A single sensor on a connected SlimeVR device.
#[derive(Debug, Clone)]
pub struct Tracker {
    pub sensor_id: u8,
    pub addr: SocketAddr,
    pub mac: [u8; 6],
    /// SlimeVR `TrackerPosition` id (0 = unassigned).
    pub position: u8,
    pub status: SensorStatus,
    pub rotation: Option<UnitQuaternion<f32>>,
    pub accel: Option<Vector3<f32>>,
    pub last_seen: Instant,
}

impl Tracker {
    pub fn id(&self) -> TrackerId {
        TrackerId::new(self.mac, self.sensor_id)
    }

    fn new(addr: SocketAddr, id: TrackerId) -> Self {
        Self {
            sensor_id: id.sensor_id,
            addr,
            mac: id.mac,
            position: 0,
            status: SensorStatus::Ok,
            rotation: None,
            accel: None,
            last_seen: Instant::now(),
        }
    }
}

/// Device addresses route packets; MAC + sensor id owns tracking state.
#[derive(Debug, Default)]
pub struct TrackerRegistry {
    by_id: HashMap<TrackerId, Tracker>,
    by_socket: HashMap<SocketAddr, [u8; 6]>,
}

impl TrackerRegistry {
    /// Local sensors bypass UDP routing and never receive network pings.
    pub fn ensure_local(&mut self, id: TrackerId, position: u8) -> &mut Tracker {
        let tracker = self.by_id.entry(id).or_insert_with(||
            Tracker::new(SocketAddr::from(([127,0,0,1], 0)), id));
        tracker.position = position;
        tracker
    }

    pub fn remove_local(&mut self, id: TrackerId) {
        if self.by_id.get(&id).is_some_and(|t| t.addr.port() == 0) {
            self.by_id.remove(&id);
        }
    }

    /// Refresh a device and move all its sensors if its UDP address changed.
    pub fn register_handshake(&mut self, addr: SocketAddr, mac: [u8; 6]) {
        if let Some(previous) = self.by_socket.get(&addr).copied() {
            if previous != mac {
                self.by_id.retain(|id, _| id.mac != previous);
            }
        }
        self.by_socket.retain(|_, m| *m != mac);
        self.by_socket.insert(addr, mac);
        // Preserve the legacy primary sensor for rotation-only clients.
        let primary = TrackerId::new(mac, 0);
        self.by_id.entry(primary).or_insert_with(|| Tracker::new(addr, primary));
        for t in self.by_id.values_mut().filter(|t| t.mac == mac) {
            t.addr = addr;
            t.last_seen = Instant::now();
        }
    }

    pub fn device_mac(&self, addr: SocketAddr) -> Option<[u8; 6]> {
        self.by_socket.get(&addr).copied()
    }

    pub fn get(&self, addr: SocketAddr, sensor_id: u8) -> Option<&Tracker> {
        self.by_id.get(&TrackerId::new(self.device_mac(addr)?, sensor_id))
    }

    fn get_mut(&mut self, addr: SocketAddr, sensor_id: u8) -> Option<&mut Tracker> {
        let id = TrackerId::new(self.device_mac(addr)?, sensor_id);
        self.by_id.get_mut(&id)
    }

    /// Register or refresh one sensor without replacing its siblings. Older
    /// SENSOR_INFO packets omit position, so preserve any existing assignment.
    pub fn update_sensor_info(
        &mut self,
        addr: SocketAddr,
        sensor_id: u8,
        position: Option<u8>,
        status: SensorStatus,
    ) -> Option<TrackerId> {
        let id = TrackerId::new(self.device_mac(addr)?, sensor_id);
        let t = self.by_id.entry(id).or_insert_with(|| Tracker::new(addr, id));
        if let Some(position) = position {
            t.position = position;
        }
        t.status = status;
        if status != SensorStatus::Ok {
            // Do not feed stale poses into FK or resets while a sensor is offline.
            t.rotation = None;
            t.accel = None;
        }
        t.last_seen = Instant::now();
        Some(id)
    }

    pub fn update_rotation(&mut self, addr: SocketAddr, sensor_id: u8, q: UnitQuaternion<f32>) {
        if let Some(t) = self.get_mut(addr, sensor_id) {
            if t.status == SensorStatus::Ok {
                t.rotation = Some(q);
            }
            t.last_seen = Instant::now();
        }
    }

    pub fn update_accel(&mut self, addr: SocketAddr, sensor_id: u8, a: Vector3<f32>) {
        if let Some(t) = self.get_mut(addr, sensor_id) {
            if t.status == SensorStatus::Ok {
                t.accel = Some(a);
            }
            t.last_seen = Instant::now();
        }
    }

    /// Heartbeats and ping echoes describe the whole device, not sensor zero.
    pub fn mark_seen(&mut self, addr: SocketAddr) {
        if let Some(mac) = self.device_mac(addr) {
            let now = Instant::now();
            for t in self.by_id.values_mut().filter(|t| t.mac == mac) {
                t.last_seen = now;
            }
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Tracker> {
        self.by_id.values()
    }

    /// Ping each device only once, regardless of its number of sensors.
    pub fn device_addresses(&self) -> impl Iterator<Item = SocketAddr> + '_ {
        self.by_socket.keys().copied()
    }

    pub fn remove_stale(&mut self, timeout: Duration) -> usize {
        let before = self.by_id.len();
        self.by_id.retain(|_, t| t.last_seen.elapsed() <= timeout);
        self.by_socket.retain(|_, mac| self.by_id.keys().any(|id| id.mac == *mac));
        before - self.by_id.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(n: u8) -> SocketAddr {
        format!("127.0.0.1:{n}").parse().unwrap()
    }

    #[test]
    fn sensors_keep_independent_samples_and_assignments() {
        let mut reg = TrackerRegistry::default();
        reg.register_handshake(addr(1), [1; 6]);
        // Extension may announce itself first and repeat its announcement.
        for _ in 0..2 {
            reg.update_sensor_info(addr(1), 1, Some(10), SensorStatus::Ok);
            reg.update_sensor_info(addr(1), 0, Some(9), SensorStatus::Ok);
        }
        let left = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let right = UnitQuaternion::from_euler_angles(-0.1, -0.2, -0.3);
        reg.update_rotation(addr(1), 0, left);
        reg.update_rotation(addr(1), 1, right);
        reg.update_accel(addr(1), 1, Vector3::x());
        assert_eq!(reg.iter().count(), 2);
        assert_eq!(reg.get(addr(1), 0).unwrap().rotation, Some(left));
        assert_eq!(reg.get(addr(1), 1).unwrap().rotation, Some(right));
        assert_eq!(reg.get(addr(1), 0).unwrap().position, 9);
        assert_eq!(reg.get(addr(1), 1).unwrap().position, 10);
        assert_eq!(reg.get(addr(1), 0).unwrap().accel, None);
        assert_eq!(reg.get(addr(1), 1).unwrap().accel, Some(Vector3::x()));
        assert_eq!(reg.device_addresses().count(), 1);
    }

    #[test]
    fn reconnect_moves_all_sensors_and_invalidates_old_address() {
        let mut reg = TrackerRegistry::default();
        reg.register_handshake(addr(1), [1; 6]);
        reg.update_sensor_info(addr(1), 7, Some(9), SensorStatus::Ok);
        reg.register_handshake(addr(2), [1; 6]);
        assert!(reg.get(addr(1), 0).is_none());
        assert!(reg.get(addr(1), 7).is_none());
        assert!(reg.update_sensor_info(addr(1), 2, Some(4), SensorStatus::Ok).is_none());
        assert_eq!(reg.get(addr(2), 7).unwrap().position, 9);
        assert_eq!(reg.get(addr(2), 7).unwrap().addr, addr(2));
        // Another MAC taking over the same socket must not inherit old sensors.
        reg.register_handshake(addr(2), [2; 6]);
        assert_eq!(reg.iter().count(), 1);
        assert!(reg.get(addr(2), 7).is_none());
    }

    #[test]
    fn status_clears_samples_until_sensor_recovers() {
        let mut reg = TrackerRegistry::default();
        reg.register_handshake(addr(1), [1; 6]);
        for status in [SensorStatus::Disconnected, SensorStatus::Error] {
            reg.update_sensor_info(addr(1), 1, Some(9), SensorStatus::Ok);
            reg.update_rotation(addr(1), 1, UnitQuaternion::identity());
            reg.update_sensor_info(addr(1), 1, None, status);
            reg.update_rotation(addr(1), 1, UnitQuaternion::identity());
            assert_eq!(reg.get(addr(1), 1).unwrap().rotation, None);
            assert_eq!(reg.get(addr(1), 1).unwrap().position, 9);
        }
        reg.update_sensor_info(addr(1), 1, None, SensorStatus::Ok);
        reg.update_rotation(addr(1), 1, UnitQuaternion::identity());
        assert!(reg.get(addr(1), 1).unwrap().rotation.is_some());
    }

    #[test]
    fn heartbeat_refreshes_extensions_and_eviction_cleans_routes() {
        let mut reg = TrackerRegistry::default();
        reg.register_handshake(addr(1), [1; 6]);
        reg.update_sensor_info(addr(1), 1, Some(9), SensorStatus::Ok);
        reg.register_handshake(addr(2), [2; 6]);
        for t in reg.by_id.values_mut() {
            t.last_seen = Instant::now() - Duration::from_secs(10);
        }
        reg.mark_seen(addr(1));
        assert_eq!(reg.remove_stale(Duration::from_secs(5)), 1);
        assert_eq!(reg.iter().count(), 2);
        assert!(reg.device_mac(addr(2)).is_none());
        // Removing only an expired extension must preserve the device route.
        reg.get_mut(addr(1), 1).unwrap().last_seen = Instant::now() - Duration::from_secs(10);
        assert_eq!(reg.remove_stale(Duration::from_secs(5)), 1);
        assert_eq!(reg.device_mac(addr(1)), Some([1; 6]));
    }
}
