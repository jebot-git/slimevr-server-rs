//! Connected-tracker registry and per-tracker state.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Instant;

use nalgebra::{UnitQuaternion, Vector3};

pub mod udp;

/// A single connected SlimeVR tracker.
#[derive(Debug, Clone)]
pub struct Tracker {
    /// Sensor id reported by the tracker (0 for the primary sensor).
    pub sensor_id: u8,
    /// UDP source address of the tracker.
    pub addr: SocketAddr,
    /// MAC address from the handshake (stable device identity).
    pub mac: [u8; 6],
    /// SlimeVR `TrackerPosition` id (0 = unassigned), from SENSOR_INFO.
    pub position: u8,
    /// Latest fused orientation (ROTATION_DATA).
    pub rotation: Option<UnitQuaternion<f32>>,
    /// Latest acceleration (ACCELERATION).
    pub accel: Option<Vector3<f32>>,
    /// When the last packet from this tracker arrived.
    pub last_seen: Instant,
}

/// A registry of connected trackers.
///
/// The MAC is the stable identity across handshakes; the UDP address and sensor id
/// are used to route the high-rate rotation/accel packets back to a tracker.
#[derive(Debug, Default)]
pub struct TrackerRegistry {
    by_mac: HashMap<[u8; 6], Tracker>,
    by_socket: HashMap<(SocketAddr, u8), [u8; 6]>,
}

impl TrackerRegistry {
    /// Register (or refresh) a tracker from its handshake. Returns its MAC.
    pub fn register_handshake(&mut self, addr: SocketAddr, mac: [u8; 6]) {
        self.by_mac.entry(mac).or_insert_with(|| Tracker {
            sensor_id: 0,
            addr,
            mac,
            position: 0,
            rotation: None,
            accel: None,
            last_seen: Instant::now(),
        });
        if let Some(t) = self.by_mac.get_mut(&mac) {
            t.addr = addr;
            t.last_seen = Instant::now();
        }
        self.by_socket.insert((addr, 0), mac);
    }

    /// Resolve a tracker by (addr, sensor_id).
    #[allow(dead_code)] // read accessor for future tracker-management code
    pub fn get(&self, addr: SocketAddr, sensor_id: u8) -> Option<&Tracker> {
        self.by_socket
            .get(&(addr, sensor_id))
            .and_then(|mac| self.by_mac.get(mac))
    }

    fn get_mut(&mut self, addr: SocketAddr, sensor_id: u8) -> Option<&mut Tracker> {
        let mac = *self.by_socket.get(&(addr, sensor_id))?;
        self.by_mac.get_mut(&mac)
    }

    /// Update sensor id + position from a SENSOR_INFO packet. Returns the tracker's MAC.
    pub fn update_sensor_info(
        &mut self,
        addr: SocketAddr,
        sensor_id: u8,
        position: u8,
    ) -> Option<[u8; 6]> {
        // Re-key the socket map from the default sensor id 0 to the reported id.
        let mac = self.by_socket.remove(&(addr, 0))?;
        self.by_socket.insert((addr, sensor_id), mac);
        if let Some(t) = self.by_mac.get_mut(&mac) {
            t.sensor_id = sensor_id;
            t.position = position;
            t.last_seen = Instant::now();
        }
        Some(mac)
    }

    /// Update a tracker's rotation.
    pub fn update_rotation(&mut self, addr: SocketAddr, sensor_id: u8, q: UnitQuaternion<f32>) {
        if let Some(t) = self.get_mut(addr, sensor_id) {
            t.rotation = Some(q);
            t.last_seen = Instant::now();
        }
    }

    /// Update a tracker's acceleration.
    pub fn update_accel(&mut self, addr: SocketAddr, sensor_id: u8, a: Vector3<f32>) {
        if let Some(t) = self.get_mut(addr, sensor_id) {
            t.accel = Some(a);
            t.last_seen = Instant::now();
        }
    }

    /// Mark a tracker as seen (heartbeat / ping echo).
    pub fn mark_seen(&mut self, addr: SocketAddr, sensor_id: u8) {
        if let Some(t) = self.get_mut(addr, sensor_id) {
            t.last_seen = Instant::now();
        }
    }

    /// All connected trackers.
    pub fn iter(&self) -> impl Iterator<Item = &Tracker> {
        self.by_mac.values()
    }
}
