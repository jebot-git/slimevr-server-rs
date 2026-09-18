//! Tracker rotation smoothing (a slerp exponential moving average).
//!
//! The HaritoraX GX6 fuses on-board, but the fused rotations still have jitter;
//! smoothing reduces high-frequency noise before the skeleton solve. `alpha` is
//! the blend toward the latest sample per tick (`0.0` = no smoothing, `1.0` = the
//! rotation never moves from its initial value).

use std::collections::HashMap;

use nalgebra::UnitQuaternion;

/// Per-tracker slerp smoothing filter.
#[derive(Default)]
pub struct Smoother {
    alpha: f32,
    prev: HashMap<[u8; 6], UnitQuaternion<f32>>,
}

impl Smoother {
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            prev: HashMap::new(),
        }
    }

    /// Return the smoothed rotation for `mac`, given its latest `raw` rotation.
    pub fn apply(&mut self, mac: [u8; 6], raw: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        if self.alpha <= 0.0 {
            return raw;
        }
        let smoothed = match self.prev.get(&mac) {
            Some(&prev) => prev.slerp(&raw, self.alpha),
            None => raw,
        };
        self.prev.insert(mac, smoothed);
        smoothed
    }

    /// Forget a tracker's history (e.g. on reset/disconnect).
    #[allow(dead_code)]
    pub fn reset(&mut self, mac: [u8; 6]) {
        self.prev.remove(&mac);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn rot_y(angle: f32) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle)
    }

    #[test]
    fn no_smoothing_passes_through() {
        let mut s = Smoother::new(0.0);
        let q = rot_y(0.5);
        assert_eq!(s.apply([1; 6], q), q);
    }

    #[test]
    fn smoothing_reduces_step() {
        // First sample seeds the filter; a jump is then only partially applied.
        let mut s = Smoother::new(0.5);
        let a = rot_y(0.0);
        let b = rot_y(1.0);
        assert_eq!(s.apply([1; 6], a), a);
        let out = s.apply([1; 6], b);
        // The output should sit between `a` and `b`.
        let to_a = out.angle_to(&a);
        let to_b = out.angle_to(&b);
        assert!(to_a > 0.1 && to_a < 0.9, "to_a = {to_a}");
        assert!(to_b > 0.1 && to_b < 0.9, "to_b = {to_b}");
    }

    #[test]
    fn smoothing_converges() {
        let mut s = Smoother::new(0.5);
        let target = rot_y(1.0);
        let mut out = s.apply([1; 6], target);
        for _ in 0..50 {
            out = s.apply([1; 6], target);
        }
        assert!(out.angle_to(&target) < 0.01);
    }
}
