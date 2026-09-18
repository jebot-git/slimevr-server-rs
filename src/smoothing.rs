//! Tracker rotation filtering: prediction (latency compensation) + smoothing.
//!
//! The HaritoraX GX6 fuses on-board, but the fused rotations still have jitter
//! and the pipeline adds a little latency. This module extrapolates each rotation
//! forward using a finite-difference angular velocity (`Predictor`), then applies
//! a slerp exponential moving average (`Smoother`). `RotationFilter` chains them.

use std::collections::HashMap;
use std::time::Instant;

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
}

/// Per-tracker prediction: extrapolate the rotation forward by `prediction_time`
/// using the finite-difference angular velocity between consecutive samples.
#[derive(Default)]
pub struct Predictor {
    prediction_time: f32,
    prev_rot: HashMap<[u8; 6], UnitQuaternion<f32>>,
    prev_time: HashMap<[u8; 6], Instant>,
}

impl Predictor {
    pub fn new(prediction_time: f32) -> Self {
        Self {
            prediction_time: prediction_time.max(0.0),
            prev_rot: HashMap::new(),
            prev_time: HashMap::new(),
        }
    }

    /// Return the predicted rotation for `mac` given its latest `raw` rotation.
    pub fn apply(
        &mut self,
        mac: [u8; 6],
        raw: UnitQuaternion<f32>,
        now: Instant,
    ) -> UnitQuaternion<f32> {
        let out = match (self.prev_rot.get(&mac), self.prev_time.get(&mac)) {
            (Some(&prev_rot), Some(&prev_time)) => {
                let dt = now.duration_since(prev_time).as_secs_f32();
                // Guard against bogus timestamps / gaps.
                if dt > 0.0 && dt < 0.5 && self.prediction_time > 0.0 {
                    // Local angular velocity = rotation vector between samples / dt.
                    let delta = prev_rot.conjugate() * raw;
                    let omega = delta.scaled_axis() / dt;
                    raw * UnitQuaternion::from_scaled_axis(omega * self.prediction_time)
                } else {
                    raw
                }
            }
            _ => raw,
        };
        self.prev_rot.insert(mac, raw);
        self.prev_time.insert(mac, now);
        out
    }
}

/// Prediction followed by smoothing, keyed per tracker MAC.
pub struct RotationFilter {
    predictor: Predictor,
    smoother: Smoother,
}

impl RotationFilter {
    pub fn new(alpha: f32, prediction_time: f32) -> Self {
        Self {
            predictor: Predictor::new(prediction_time),
            smoother: Smoother::new(alpha),
        }
    }

    pub fn apply(
        &mut self,
        mac: [u8; 6],
        raw: UnitQuaternion<f32>,
        now: Instant,
    ) -> UnitQuaternion<f32> {
        let predicted = self.predictor.apply(mac, raw, now);
        self.smoother.apply(mac, predicted)
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
        let mut s = Smoother::new(0.5);
        let a = rot_y(0.0);
        let b = rot_y(1.0);
        assert_eq!(s.apply([1; 6], a), a);
        let out = s.apply([1; 6], b);
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

    #[test]
    fn predictor_advances_constant_velocity() {
        // A constant angular velocity of 1 rad/s about Y, predicted 0.1 s forward.
        let mut p = Predictor::new(0.1);
        let t0 = Instant::now();
        let r0 = rot_y(0.0);
        assert_eq!(p.apply([1; 6], r0, t0), r0);

        // 0.1 s later, the raw is 0.1 rad; predicted should be ~0.2 rad.
        let t1 = t0 + std::time::Duration::from_millis(100);
        let r1 = rot_y(0.1);
        let out = p.apply([1; 6], r1, t1);
        // The predicted angle should be ahead of r1.
        let a1 = r1.angle_to(&r0); // 0.1
        let a_out = out.angle_to(&r0);
        assert!(a_out > a1, "a1={a1} a_out={a_out}");
        assert!(a_out < a1 + 0.2, "a1={a1} a_out={a_out}");
    }

    #[test]
    fn predictor_zero_prediction_passes_through() {
        let mut p = Predictor::new(0.0);
        let t0 = Instant::now();
        let r0 = rot_y(0.0);
        p.apply([1; 6], r0, t0);
        let t1 = t0 + std::time::Duration::from_millis(100);
        let r1 = rot_y(0.5);
        assert_eq!(p.apply([1; 6], r1, t1), r1);
    }
}
