//! Tracker → skeleton calibration: standing (full) reset, mounting reset, yaw
//! reset, and yaw-drift compensation. Ported from the Java server's
//! `TrackerResetsHandler` (simplified: no arm/T-pose modes, no HMD special-casing).
//!
//! Each tracker carries its own set of corrective rotations. The adjustment chain,
//! matching the Java `adjustToReference` + `adjustToDrift`, is:
//!
//! ```text
//! adjusted = drift * (yaw_fix * mount_rot_fix⁻¹ * (attachment_fix → gyro_fix →
//!             mounting_orientation → raw) * mount_rot_fix)
//! ```
//!
//! Concretely, in order: `rot = raw * mounting_orientation`, `rot = gyro_fix * rot`,
//! `rot *= attachment_fix`, `rot = mount_rot_fix⁻¹ * rot * mount_rot_fix`,
//! `rot = yaw_fix * rot`, then drift is applied on top.

use std::collections::HashMap;
use std::time::Instant;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// A MAC address identifying a tracker.
pub type Mac = [u8; 6];

/// Seconds over which a yaw reset eases its correction in.
const YAW_SMOOTH_SECS: f32 = 1.0;

/// Yaw-drift compensation: records the yaw drift between resets and applies a
/// gradually-ramping correction (a simplified form of the Java `calculateDrift`).
#[derive(Debug, Default)]
pub struct DriftCompensator {
    /// Whether drift compensation is active.
    pub enabled: bool,
    /// Correction amount (0..1, the Java `driftCompensationConfig.amount`).
    pub amount: f32,
    /// Latest recorded yaw-drift quaternion.
    drift_quat: UnitQuaternion<f32>,
    /// Duration (s) of the interval the latest drift was measured over.
    drift_duration: f32,
    /// When the last reset happened.
    since: Option<Instant>,
}

impl DriftCompensator {
    fn new() -> Self {
        Self {
            enabled: false,
            amount: 0.5,
            drift_quat: UnitQuaternion::identity(),
            drift_duration: 0.0,
            since: None,
        }
    }

    /// Record the yaw drift between the pre-reset (`before`) and post-reset (`after`)
    /// reference-adjusted rotations.
    fn record(&mut self, before: &UnitQuaternion<f32>, after: &UnitQuaternion<f32>) {
        if !self.enabled {
            self.since = Some(Instant::now());
            return;
        }
        let drift = yaw_quat_of(after) * yaw_quat_of(before).inverse();
        let dt = self.since.map(|s| s.elapsed().as_secs_f32()).unwrap_or(0.0);
        self.drift_quat = drift;
        self.drift_duration = dt;
        self.since = Some(Instant::now());
    }

    /// Apply the ramping drift correction to a rotation.
    fn apply(&self, rot: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        if !self.enabled || self.drift_duration <= 0.0 {
            return rot;
        }
        let elapsed = self.since.map(|s| s.elapsed().as_secs_f32()).unwrap_or(0.0);
        let ratio = (elapsed / self.drift_duration).clamp(0.0, 1.0);
        quat_pow(&self.drift_quat, self.amount * ratio) * rot
    }
}

/// Per-tracker calibration state.
#[derive(Debug)]
pub struct TrackerCalibration {
    /// Fixed mounting orientation (tracker frame → bone frame). The SlimeVR
    /// `HalfHorizontal`/`defaultMounting` conventions are a TODO; identity for now.
    pub mounting_orientation: UnitQuaternion<f32>,
    gyro_fix: UnitQuaternion<f32>,
    attachment_fix: UnitQuaternion<f32>,
    mount_rot_fix: UnitQuaternion<f32>,
    yaw_fix: UnitQuaternion<f32>,
    /// The `yaw_fix` value at the moment the last yaw reset started (for easing).
    yaw_fix_start: UnitQuaternion<f32>,
    /// When the last yaw reset started (for easing the correction in).
    yaw_reset_since: Option<Instant>,
    drift: DriftCompensator,
}

impl Default for TrackerCalibration {
    fn default() -> Self {
        Self {
            mounting_orientation: default_mounting(0),
            gyro_fix: UnitQuaternion::identity(),
            attachment_fix: UnitQuaternion::identity(),
            mount_rot_fix: UnitQuaternion::identity(),
            yaw_fix: UnitQuaternion::identity(),
            yaw_fix_start: UnitQuaternion::identity(),
            yaw_reset_since: None,
            drift: DriftCompensator::new(),
        }
    }
}

impl TrackerCalibration {
    /// Full reset (standing): zero this tracker's yaw/pitch/roll against `reference`.
    pub fn full_reset(&mut self, raw: UnitQuaternion<f32>, reference: UnitQuaternion<f32>) {
        let before = self.adjust_reference(raw);
        let mounting_adjusted = raw * self.mounting_orientation;

        self.gyro_fix = inverse_yaw(&mounting_adjusted);
        self.attachment_fix = (self.gyro_fix * mounting_adjusted).inverse();
        self.yaw_fix = self.fix_yaw(mounting_adjusted, reference);
        // Full reset snaps immediately (no yaw easing).
        self.yaw_reset_since = None;

        let after = self.adjust_reference(raw);
        self.drift.record(&before, &after);
    }

    /// Yaw reset: align only the yaw to `reference`, easing the correction in
    /// over [`YAW_SMOOTH_SECS`] instead of snapping.
    pub fn yaw_reset(&mut self, raw: UnitQuaternion<f32>, reference: UnitQuaternion<f32>) {
        let before = self.adjust_reference(raw);
        let target = self.fix_yaw(raw * self.mounting_orientation, reference);
        self.yaw_fix_start = self.yaw_fix;
        self.yaw_fix = target;
        self.yaw_reset_since = Some(Instant::now());
        let after = self.adjust_reference(raw);
        self.drift.record(&before, &after);
    }

    /// Mounting reset (skip pose): compute the yaw-only axis alignment.
    ///
    /// `position` is the tracker's `TrackerPosition` id. Non-thigh trackers face
    /// "back" during a mounting reset, so their yaw angle is flipped by 180°
    /// (matching the Java `resetMounting`), while thigh trackers keep it as-is.
    pub fn mounting_reset(
        &mut self,
        raw: UnitQuaternion<f32>,
        reference: UnitQuaternion<f32>,
        position: u8,
    ) {
        let before = self.adjust_reference(raw);

        let mut rot = raw * self.mounting_orientation;
        rot = self.gyro_fix * rot;
        rot = rot * self.attachment_fix;
        rot = self.yaw_fix * rot;
        rot = reference_yaw(&reference).inverse() * rot;

        let up = rot * Vector3::y();
        let mut yaw_angle = up.x.atan2(up.z);
        // LEFT_UPPER_LEG (7) / RIGHT_UPPER_LEG (8) are thighs; everything else
        // (chest, hip, lower legs, …) points backward in the skip pose.
        if !matches!(position, 7 | 8) {
            yaw_angle -= std::f32::consts::PI;
        }
        self.mount_rot_fix = yaw_quat(yaw_angle);

        let after = self.adjust_reference(raw);
        self.drift.record(&before, &after);
    }

    /// The corrected bone rotation: reference fixes then drift.
    pub fn adjust(&self, raw: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        self.drift.apply(self.adjust_reference(raw))
    }

    /// The reference-adjustment chain (without drift).
    fn adjust_reference(&self, raw: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        let mut rot = raw * self.mounting_orientation;
        rot = self.gyro_fix * rot;
        rot = rot * self.attachment_fix;
        rot = self.mount_rot_fix.inverse() * (rot * self.mount_rot_fix);
        rot = self.effective_yaw_fix() * rot;
        rot
    }

    /// The yaw fix, eased from `yaw_fix_start` to `yaw_fix` over the smooth window.
    fn effective_yaw_fix(&self) -> UnitQuaternion<f32> {
        match self.yaw_reset_since {
            Some(since) => {
                let t = (since.elapsed().as_secs_f32() / YAW_SMOOTH_SECS).clamp(0.0, 1.0);
                self.yaw_fix_start.slerp(&self.yaw_fix, t)
            }
            None => self.yaw_fix,
        }
    }

    /// Port of the Java `fixYaw`.
    fn fix_yaw(&self, sensor: UnitQuaternion<f32>, reference: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        let mut rot = self.gyro_fix * sensor;
        rot = rot * self.attachment_fix;
        rot = self.mount_rot_fix.inverse() * (rot * self.mount_rot_fix);
        let yaw = yaw_quat_of(&rot);
        yaw.inverse() * reference_yaw(&reference)
    }
}

/// The set of per-tracker calibrations, keyed by MAC.
#[derive(Default)]
pub struct Calibration {
    trackers: HashMap<Mac, TrackerCalibration>,
}

impl Calibration {
    pub fn new() -> Self {
        Self::default()
    }

    /// The corrected bone rotation for a tracker.
    pub fn adjust(&self, mac: Mac, raw: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        self.trackers
            .get(&mac)
            .map(|t| t.adjust(raw))
            .unwrap_or(raw)
    }

    /// Get (or create) a tracker's calibration state.
    pub fn tracker_mut(&mut self, mac: Mac) -> &mut TrackerCalibration {
        self.trackers.entry(mac).or_default()
    }

    /// Set a tracker's mounting orientation from its body part (frame alignment).
    pub fn set_mounting(&mut self, mac: Mac, position: u8) {
        self.trackers.entry(mac).or_default().mounting_orientation =
            default_mounting(position);
    }
}

// ---- Frame alignment (SlimeVR mounting orientations) ----
//
// The SlimeVR tracker's sensor frame differs from the bone frame; these constants
// (vendored from ktmath's `Quaternion.SLIMEVR`) map the two. ktmath's constructor is
// `Quaternion(w, x, y, z)` — scalar first — and each constant is a rotation about the
// Y axis (yaw), matching the physical mounting orientation.

/// Build a unit quaternion from ktmath's `(w, x, y, z)` component order.
fn q(w: f32, x: f32, y: f32, z: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

/// The SlimeVR `defaultMounting()` orientation for a `TrackerPosition` id.
pub fn default_mounting(position: u8) -> UnitQuaternion<f32> {
    match position {
        // LEFT_LOWER_ARM, LEFT_HAND, left fingers → LEFT (90° yaw)
        13 | 17 | 21..=35 => q(0.707, 0.0, 0.707, 0.0),
        // RIGHT_LOWER_ARM, RIGHT_HAND, right fingers → RIGHT (-90° yaw)
        14 | 18 | 36..=50 => q(0.707, 0.0, -0.707, 0.0),
        // LEFT_UPPER_ARM, LEFT_LOWER_LEG → FRONT_LEFT (135° yaw)
        15 | 9 => q(0.383, 0.0, 0.924, 0.0),
        // RIGHT_UPPER_ARM, RIGHT_LOWER_LEG → FRONT_RIGHT (-135° yaw)
        16 | 10 => q(0.383, 0.0, -0.924, 0.0),
        // everything else (chest, hip, thighs, feet, neck, …) → FRONT (180° yaw)
        _ => q(0.0, 0.0, 1.0, 0.0),
    }
}

// ---- Yaw helpers ----
//
// "Yaw" here is the heading: the direction the forward vector (-Z) points in the
// horizontal XZ plane. `+X` right, `+Y` up, `-Z` forward.

fn yaw_of(q: &UnitQuaternion<f32>) -> f32 {
    let f = q * Vector3::new(0.0, 0.0, -1.0);
    (-f.x).atan2(-f.z)
}

fn yaw_quat(a: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), a)
}

fn yaw_quat_of(q: &UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    yaw_quat(yaw_of(q))
}

fn inverse_yaw(q: &UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    yaw_quat(-yaw_of(q))
}

fn reference_yaw(q: &UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    yaw_quat_of(q)
}

/// Fractional power of a unit quaternion (same axis, angle scaled by `t`).
fn quat_pow(q: &UnitQuaternion<f32>, t: f32) -> UnitQuaternion<f32> {
    match q.axis_angle() {
        Some((axis, angle)) => UnitQuaternion::from_axis_angle(&axis, angle * t),
        None => UnitQuaternion::identity(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_close(a: &UnitQuaternion<f32>, b: &UnitQuaternion<f32>, eps: f32) -> bool {
        (yaw_of(a) - yaw_of(b)).abs() < eps
    }

    #[test]
    fn full_reset_aligns_to_reference_yaw() {
        let raw = yaw_quat(1.2) * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.3);
        let reference = yaw_quat(0.1);

        let mut c = TrackerCalibration::default();
        c.full_reset(raw, reference);

        // After full reset, the tracker's yaw matches the reference's yaw.
        let adjusted = c.adjust(raw);
        assert!(yaw_close(&adjusted, &reference, 1e-4), "adjusted yaw {}", yaw_of(&adjusted));
    }

    #[test]
    fn full_reset_removes_pitch_and_roll() {
        // A tracker mounted with pitch + roll, but standing upright: full reset
        // should zero those out (relative to gravity "up").
        let raw = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.25);
        let reference = UnitQuaternion::identity();

        let mut c = TrackerCalibration::default();
        c.full_reset(raw, reference);

        let adjusted = c.adjust(raw);
        // The rotated up vector should be ~straight up (pitch/roll removed).
        let up = adjusted * Vector3::y();
        assert!((up - Vector3::y()).norm() < 1e-3, "up = {up:?}");
    }

    #[test]
    fn yaw_reset_only_changes_yaw() {
        let raw = yaw_quat(2.0);
        let reference = yaw_quat(-0.5);

        let mut c = TrackerCalibration::default();
        c.yaw_reset(raw, reference);
        // Fast-forward past the smooth window so the correction is fully applied.
        c.yaw_reset_since = Some(Instant::now() - std::time::Duration::from_secs(2));

        let adjusted = c.adjust(raw);
        assert!(yaw_close(&adjusted, &reference, 1e-4));
    }

    #[test]
    fn yaw_reset_eases_in_over_time() {
        let raw = yaw_quat(2.0);
        let reference = yaw_quat(-0.5);

        let mut c = TrackerCalibration::default();
        c.yaw_reset(raw, reference);

        // Immediately after the reset the correction is not fully applied.
        let t0 = c.adjust(raw);
        assert!(
            !yaw_close(&t0, &reference, 1e-3),
            "correction should ease in, got yaw {}",
            yaw_of(&t0)
        );

        // After the smooth window it is fully applied.
        c.yaw_reset_since = Some(Instant::now() - std::time::Duration::from_secs(2));
        let t1 = c.adjust(raw);
        assert!(yaw_close(&t1, &reference, 1e-4), "yaw {}", yaw_of(&t1));
    }

    #[test]
    fn drift_ramps_in() {
        let mut d = DriftCompensator::new();
        d.enabled = true;
        d.amount = 1.0;

        // Drift of 0.2 rad over a 2 s interval.
        let before = UnitQuaternion::identity();
        let after = yaw_quat(0.2);
        // Simulate a 2 s interval since the previous reset.
        d.since = Some(Instant::now() - std::time::Duration::from_secs(2));
        d.record(&before, &after);

        // Immediately after reset: no correction yet.
        let rot0 = d.apply(yaw_quat(0.0));
        assert!(yaw_of(&rot0).abs() < 1e-4);

        // At full ratio (>= 2 s later) the full drift is applied.
        let d2 = DriftCompensator {
            enabled: true,
            amount: 1.0,
            drift_quat: d.drift_quat,
            drift_duration: d.drift_duration,
            since: Some(Instant::now() - std::time::Duration::from_secs(3)),
        };
        let rot_full = d2.apply(yaw_quat(0.0));
        assert!((yaw_of(&rot_full) - 0.2).abs() < 1e-3, "yaw {}", yaw_of(&rot_full));
    }

    fn assert_mounting(p: u8, w: f32, x: f32, y: f32, z: f32) {
        let m = default_mounting(p);
        assert!((m.w - w).abs() < 1e-3, "position {p}: w = {}", m.w);
        assert!((m.i - x).abs() < 1e-3, "position {p}: i = {}", m.i);
        assert!((m.j - y).abs() < 1e-3, "position {p}: j = {}", m.j);
        assert!((m.k - z).abs() < 1e-3, "position {p}: k = {}", m.k);
    }

    #[test]
    fn default_mounting_matches_slimevr() {
        // chest/hip/thighs/feet → FRONT = (w=0, x=0, y=1, z=0) = 180° yaw
        for p in [0u8, 4, 6, 7, 8, 11, 12] {
            assert_mounting(p, 0.0, 0.0, 1.0, 0.0);
        }
        // left upper arm / left lower leg → FRONT_LEFT = (0.383, 0, 0.924, 0)
        assert_mounting(15, 0.383, 0.0, 0.924, 0.0);
        assert_mounting(9, 0.383, 0.0, 0.924, 0.0);
        // right upper arm / right lower leg → FRONT_RIGHT = (0.383, 0, -0.924, 0)
        assert_mounting(16, 0.383, 0.0, -0.924, 0.0);
        assert_mounting(10, 0.383, 0.0, -0.924, 0.0);
        // left forearm/hand → LEFT = (0.707, 0, 0.707, 0)
        assert_mounting(13, 0.707, 0.0, 0.707, 0.0);
        assert_mounting(17, 0.707, 0.0, 0.707, 0.0);
        // right forearm/hand → RIGHT = (0.707, 0, -0.707, 0)
        assert_mounting(14, 0.707, 0.0, -0.707, 0.0);
        assert_mounting(18, 0.707, 0.0, -0.707, 0.0);
    }
}
