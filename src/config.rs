//! Server configuration (ports, etc.). Kept minimal for the first milestone.

/// Port the tracker UDP protocol server listens on.
pub const TRACKER_PORT: u16 = 6969;

/// Port the SolarXR WebSocket server listens on (the port WiVRn connects to).
pub const SOLARXR_PORT: u16 = 21110;

/// How often the server sends a ping to each tracker (liveness check).
pub const PING_INTERVAL_SECS: u64 = 10;
