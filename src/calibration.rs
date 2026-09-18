//! Tracker → skeleton calibration: mounting offsets and the global heading.
//!
//! A tracker's raw rotation is in its own gravity-aligned sensor frame. Before it
//! can drive a bone it must be transformed into the bone's global frame:
//!
//! ```text
//! bone_rot = heading * raw_rot * mounting_offset
//! ```
//!
//! * `mounting_offset` maps the tracker's sensor frame onto the bone (constant once
//!   the tracker is strapped on). It is (re)computed by a **mounting reset**, which
//!   also implicitly absorbs any sensor↔global frame difference, since both the raw
//!   rotation and the bone's calibration rotation share gravity "up".
//! * `heading` is a yaw-only correction computed by a **full reset** from a reference
//!   tracker (head/chest), aligning the skeleton's forward with that reference.

use std::collections::HashMap;

use nalgebra::{UnitQuaternion, Vector3};

/// A MAC address identifying a tracker.
pub type Mac = [u8; 6];

/// Per-tracker mounting offsets + a global heading correction.
#[derive(Default)]
pub struct Calibration {
    /// `tracker frame → bone frame` offset, keyed by tracker MAC.
    mounting: HashMap<Mac, UnitQuaternion<f32>>,
    /// Global yaw correction from the last full reset. Identity = uncalibrated.
    heading: UnitQuaternion<f32>,
}

impl Calibration {
    pub fn new() -> Self {
        Self::default()
    }

    /// Full reset: align the skeleton's forward with `reference` (the head/chest
    /// tracker's current rotation). Only the yaw component is used.
    pub fn full_reset(&mut self, reference: UnitQuaternion<f32>) {
        self.heading = inverse_yaw(&reference);
    }

    /// Yaw reset: identical to a full reset for heading purposes (recenter).
    pub fn yaw_reset(&mut self, reference: UnitQuaternion<f32>) {
        self.full_reset(reference);
    }

    /// Mounting reset for one tracker: compute its `tracker → bone` offset so that,
    /// in the calibration pose, `raw * offset` equals the bone's expected orientation
    /// `bone_calib`.
    pub fn mounting_reset(
        &mut self,
        mac: Mac,
        raw: UnitQuaternion<f32>,
        bone_calib: UnitQuaternion<f32>,
    ) {
        let offset = raw.inverse() * bone_calib;
        self.mounting.insert(mac, offset);
    }

    /// Adjust a tracker's raw rotation into its bone's global rotation.
    pub fn adjust(&self, mac: Mac, raw: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        let offset = self
            .mounting
            .get(&mac)
            .copied()
            .unwrap_or_else(UnitQuaternion::identity);
        self.heading * raw * offset
    }
}

/// The inverse of a rotation's yaw component (rotation about the global `+Y` axis).
fn inverse_yaw(q: &UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    let (_, _, yaw) = q.euler_angles();
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mounting_reset_makes_bone_match_calib() {
        let mut c = Calibration::new();
        let mac = [1u8; 6];
        // Tracker mounted at some arbitrary offset.
        let raw = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.3);
        let bone_calib = UnitQuaternion::identity();

        c.mounting_reset(mac, raw, bone_calib);
        let adjusted = c.adjust(mac, raw);

        // Heading is identity (no full reset), so adjusted == bone_calib.
        let angle = adjusted.angle_to(&bone_calib);
        assert!(angle.abs() < 1e-5, "angle = {angle}");
    }

    #[test]
    fn full_reset_cancels_reference_yaw() {
        let mut c = Calibration::new();
        let mac = [2u8; 6];
        let raw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.9);

        c.full_reset(raw);
        let adjusted = c.adjust(mac, raw); // no mounting offset yet

        let (_, _, yaw) = adjusted.euler_angles();
        assert!(yaw.abs() < 1e-5, "yaw = {yaw}");
    }

    #[test]
    fn inverse_yaw_leaves_only_non_yaw() {
        let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.3);
        let (_, _, yaw) = (inverse_yaw(&q) * q).euler_angles();
        assert!(yaw.abs() < 1e-5, "yaw = {yaw}");
    }
}
