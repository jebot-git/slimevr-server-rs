//! HaritoraX tracker acquisition, ported from SlimeTora +
//! `haritorax-interpreter`.
//!
//! This module reads HaritoraX trackers through the GX6/GX2 communication dongle
//! (serial/COM) using the serial protocol, decodes their IMU frames, and
//! emits normalized events that the app forwards into the SlimeVR server as
//! emulated trackers.
//!
//! The supported targets are the GX6 dongle and the HaritoraX 2 / HaritoraX
//! Wireless trackers (which share the HaritoraX 2 protocol). HaritoraX Wired 1.x
//! (BTSPP classic Bluetooth) is out of scope.

pub mod gx6;
pub mod config;
pub mod identity;
pub mod bridge;
pub mod serial;


/// `0.01 / 180` — rotation quaternion int16 scalar (the missing π is intentional;
/// a uniform scale on all four quaternion components is normalized away).
pub const ROTATION_SCALAR: f32 = 0.01 / 180.0;
/// `1 / 256` — gravity vector int16 scalar.
pub const GRAVITY_SCALAR: f32 = 1.0 / 256.0;
pub const GRAVITY_CONSTANT: f32 = 9.81;

/// A decoded IMU sample: rotation quaternion `(x, y, z, w)` and gravity-corrected
/// acceleration vector `(x, y, z)`.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub rotation: [f32; 4],
    pub acceleration: [f32; 3],
}

/// A normalized event emitted by a HaritoraX device.
#[derive(Debug, Clone)]
pub enum HaritoraXEvent {
    TrackerConnected { name: String },
    TrackerDisconnected { name: String },
    Imu { name: String, sample: ImuSample },
    Battery { name: String, voltage_v: f32, percentage: f32 },
    /// A tracker button was pressed (`button` is "main" or "sub").
    Button { name: String, button: String },
}


/// The static tracker-id → body-part map for the COM path (§3.7 of the port spec).
///
/// Ids `1`–`8` are the standard HaritoraX 2 / Wireless set; `9`/`a`/`b`/`c`/`d`
/// are the extended ids from SlimeTora's tracker-assignment table (the pre-Shiftall
/// "Haritora" DIY set), kept for full COM parity.
pub fn body_part_from_tracker_id(id: char) -> Option<&'static str> {
    Some(match id {
        '1' => "chest",
        '2' => "leftKnee",
        '3' => "leftAnkle",
        '4' => "rightKnee",
        '5' => "rightAnkle",
        '6' => "hip",
        '7' => "leftElbow",
        '8' => "rightElbow",
        '9' => "leftWrist",
        'a' => "rightWrist",
        'b' => "head",
        'c' => "leftFoot",
        'd' => "rightFoot",
        _ => return None,
    })
}

/// Map an ankle name to its thigh/extension name for the HaritoraX 2 legs.
pub fn thigh_name_for_ankle(name: &str) -> Option<&'static str> {
    match name {
        "leftAnkle" => Some("leftKnee"),
        "rightAnkle" => Some("rightKnee"),
        _ => None,
    }
}

/// Map a HaritoraX body-part name to the SlimeVR `TrackerPosition` id, which is
/// sent in the SENSOR_INFO packet so the server auto-assigns the body part for
/// newly-seen trackers (no GUI assignment required).
///
/// SlimeVR `TrackerPosition` ids:
/// CHEST=4, HIP=6, LEFT_UPPER_LEG=7, RIGHT_UPPER_LEG=8, LEFT_LOWER_LEG=9,
/// RIGHT_LOWER_LEG=10, LEFT_FOOT=11, RIGHT_FOOT=12, LEFT_UPPER_ARM=15,
/// RIGHT_UPPER_ARM=16, HEAD=1, LEFT_HAND=17, RIGHT_HAND=18. `0` = unassigned.
///
/// The HaritoraX 2 "ankle" tracker sits on the shin (lower leg), so it maps to
/// LEFT/RIGHT_LOWER_LEG — the foot is only tracked with the expansion set.
pub fn tracker_position_id(name: &str) -> u8 {
    match name {
        "chest" => 4,
        "hip" => 6,
        "leftKnee" => 7,
        "rightKnee" => 8,
        "leftAnkle" => 9,
        "rightAnkle" => 10,
        "leftElbow" => 15,
        "rightElbow" => 16,
        // Extended ids — best-effort SlimeVR positions. Wrist trackers sit at the
        // hand; the "foot" names are for the expansion set (real foot trackers).
        "head" => 1,
        "leftWrist" => 17,
        "rightWrist" => 18,
        "leftFoot" => 11,
        "rightFoot" => 12,
        _ => 0,
    }
}

/// Hamilton quaternion product `a ⊗ b`.
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Decode a 14-byte IMU buffer into an [`ImuSample`], ported from
/// `haritorax-interpreter`'s `decodeIMUPacket`.
///
/// `is_x2` selects the gravity adjustment (1.0 for HaritoraX 2, 1.2 for Wireless).
pub fn decode_imu(buf: &[u8], is_x2: bool) -> Option<ImuSample> {
    if buf.len() < 14 {
        return None;
    }

    let read_i16 = |off: usize| i16::from_le_bytes([buf[off], buf[off + 1]]) as f32;

    let mut rot = [
        read_i16(0) * ROTATION_SCALAR,
        read_i16(2) * ROTATION_SCALAR,
        read_i16(4) * -ROTATION_SCALAR,
        read_i16(6) * -ROTATION_SCALAR,
    ];
    let norm = rot.iter().map(|v| v * v).sum::<f32>().sqrt();
    if !norm.is_finite() || norm < 1e-6 { return None; }
    for value in &mut rot { *value /= norm; }
    let grav = [
        read_i16(8) * GRAVITY_SCALAR,
        read_i16(10) * GRAVITY_SCALAR,
        read_i16(12) * GRAVITY_SCALAR,
    ];

    // Hamilton product gravity prediction.
    let rc = [rot[3], rot[0], rot[1], rot[2]];
    let r = [rc[0], -rc[1], -rc[2], -rc[3]];
    let p = [0.0, 0.0, 0.0, GRAVITY_CONSTANT];
    let hrp = quat_mul(r, p);
    let h_final = quat_mul(hrp, rc);

    let adj = if is_x2 { 1.0 } else { 1.2 };
    let acceleration = [
        grav[0] - h_final[1] * -adj,
        grav[1] - h_final[2] * -adj,
        grav[2] - h_final[3] * adj,
    ];

    Some(ImuSample {
        rotation: rot,
        acceleration,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_zero_buffer_is_rejected() {
        assert!(decode_imu(&[0; 14], true).is_none());
        assert!(decode_imu(&[0; 13], true).is_none());
    }

    #[test]
    fn body_part_map() {
        assert_eq!(body_part_from_tracker_id('1'), Some("chest"));
        assert_eq!(body_part_from_tracker_id('8'), Some("rightElbow"));
        assert_eq!(body_part_from_tracker_id('9'), Some("leftWrist"));
        assert_eq!(body_part_from_tracker_id('a'), Some("rightWrist"));
        assert_eq!(body_part_from_tracker_id('b'), Some("head"));
        assert_eq!(body_part_from_tracker_id('c'), Some("leftFoot"));
        assert_eq!(body_part_from_tracker_id('d'), Some("rightFoot"));
        assert_eq!(body_part_from_tracker_id('0'), None);
    }

    #[test]
    fn thigh_mapping() {
        assert_eq!(thigh_name_for_ankle("leftAnkle"), Some("leftKnee"));
        assert_eq!(thigh_name_for_ankle("chest"), None);
    }

    #[test]
    fn tracker_position_mapping() {
        assert_eq!(tracker_position_id("chest"), 4);
        assert_eq!(tracker_position_id("hip"), 6);
        assert_eq!(tracker_position_id("leftKnee"), 7);
        assert_eq!(tracker_position_id("rightKnee"), 8);
        assert_eq!(tracker_position_id("leftAnkle"), 9);
        assert_eq!(tracker_position_id("rightAnkle"), 10);
        assert_eq!(tracker_position_id("leftElbow"), 15);
        assert_eq!(tracker_position_id("rightElbow"), 16);
        assert_eq!(tracker_position_id("head"), 1);
        assert_eq!(tracker_position_id("leftWrist"), 17);
        assert_eq!(tracker_position_id("rightWrist"), 18);
        assert_eq!(tracker_position_id("leftFoot"), 11);
        assert_eq!(tracker_position_id("rightFoot"), 12);
        assert_eq!(tracker_position_id("unknown"), 0);
    }
}
