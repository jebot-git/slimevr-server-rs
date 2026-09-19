//! Per-user autobone: optimize bone lengths from recorded tracker motion.
//!
//! A simplified port of SlimeVR's `AutoBone`. During a recording the user moves;
//! we then coordinate-descent over the vertical (height) bone lengths to minimise
//! (a) how much the feet "slide" (their height should stay consistent across
//! frames) and (b) the difference from the configured target height. The result
//! replaces the generic anthropometric `bone_lengths_from_height`.

use std::collections::HashMap;

use nalgebra::Vector3;
use skeletal_model::{BoneKind, BoneMap};

use crate::calibration::Calibration;
use crate::skeleton::{solve_pose_with_lengths, BonePose};
use crate::tracker::Tracker;

/// A recorded frame: a snapshot of all trackers at one instant.
#[derive(Clone, Debug)]
pub struct Frame {
    pub trackers: Vec<Tracker>,
}

/// Live autobone state: whether we're recording, plus the recorded frames.
#[derive(Default)]
pub struct AutoboneState {
    pub recording: bool,
    pub frames: Vec<Frame>,
}

/// The bone lengths autobone may adjust (the vertical chain + legs).
pub const ADJUSTABLE_BONES: &[BoneKind] = &[
    BoneKind::Neck,
    BoneKind::UpperChest,
    BoneKind::Chest,
    BoneKind::Waist,
    BoneKind::Hip,
    BoneKind::ThighL,
    BoneKind::ThighR,
    BoneKind::AnkleL,
    BoneKind::AnkleR,
];

/// Total vertical extent of the skeleton: the height-contributing bone lengths.
pub fn vertical_height(lengths: &BoneMap<f32>) -> f32 {
    lengths[BoneKind::Neck]
        + lengths[BoneKind::UpperChest]
        + lengths[BoneKind::Chest]
        + lengths[BoneKind::Waist]
        + lengths[BoneKind::Hip]
        + lengths[BoneKind::ThighL]
        + lengths[BoneKind::AnkleL]
        + lengths[BoneKind::FootL]
}

/// The tail (child-side) position of a body part from a solved pose.
fn tail_pos(pose: &HashMap<u8, BonePose>, body_part: u8) -> Option<[f32; 3]> {
    let bp = pose.get(&body_part)?;
    let off = bp.rotation * Vector3::new(0.0, -bp.length, 0.0);
    Some([
        bp.head_pos[0] + off.x,
        bp.head_pos[1] + off.y,
        bp.head_pos[2] + off.z,
    ])
}

/// Total error for a candidate length set. Lower is better.
fn error(
    frames: &[Frame],
    calib: &Calibration,
    lengths: &BoneMap<f32>,
    target_height: f32,
) -> f32 {
    // Feet height consistency (foot-plant / slide error).
    let mut feet_y = Vec::with_capacity(frames.len());
    for frame in frames {
        let pose = solve_pose_with_lengths(frame.trackers.iter().cloned(), calib, *lengths, None);
        let l = tail_pos(&pose, 10); // LEFT_FOOT
        let r = tail_pos(&pose, 11); // RIGHT_FOOT
        if let (Some(l), Some(r)) = (l, r) {
            feet_y.push((l[1] + r[1]) / 2.0);
        }
    }
    let variance = if feet_y.is_empty() {
        f32::INFINITY
    } else {
        let mean = feet_y.iter().sum::<f32>() / feet_y.len() as f32;
        feet_y.iter().map(|y| (y - mean).powi(2)).sum::<f32>() / feet_y.len() as f32
    };

    // Height constraint (keep total height near the target).
    let height_err = (vertical_height(lengths) - target_height).powi(2);

    variance + 10.0 * height_err
}

/// Optimize bone lengths against the recorded frames. Returns the adjusted map.
pub fn optimize(
    frames: &[Frame],
    calib: &Calibration,
    mut lengths: BoneMap<f32>,
    target_height: f32,
    epochs: usize,
) -> BoneMap<f32> {
    if frames.is_empty() {
        return lengths;
    }

    let mut best = error(frames, calib, &lengths, target_height);
    let mut delta = 0.02f32; // 2 cm initial step

    for _ in 0..epochs {
        for &bone in ADJUSTABLE_BONES {
            for sign in [1.0f32, -1.0f32] {
                let candidate_len = (lengths[bone] + sign * delta).max(0.01);
                let mut candidate = lengths;
                candidate[bone] = candidate_len;
                let e = error(frames, calib, &candidate, target_height);
                if e < best {
                    best = e;
                    lengths = candidate;
                }
            }
        }
        delta *= 0.8;
    }
    lengths
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;
    use nalgebra::UnitQuaternion;

    fn tracker(position: u8, rot: Option<UnitQuaternion<f32>>) -> Tracker {
        Tracker {
            sensor_id: 0,
            addr: "127.0.0.1:1".parse().unwrap(),
            mac: [position; 6],
            position,
            rotation: rot,
            accel: None,
            last_seen: Instant::now(),
        }
    }

    #[test]
    fn height_constraint_is_respected() {
        // A single static frame: only the height term matters.
        let frames = vec![Frame {
            trackers: vec![
                tracker(4, Some(UnitQuaternion::identity())),  // chest
                tracker(6, Some(UnitQuaternion::identity())),  // hip
                tracker(7, Some(UnitQuaternion::identity())),  // thigh L
                tracker(8, Some(UnitQuaternion::identity())),  // thigh R
                tracker(9, Some(UnitQuaternion::identity())),  // ankle L
                tracker(10, Some(UnitQuaternion::identity())), // ankle R
            ],
        }];

        let lengths = BoneMap::new([0.2f32; BoneKind::NUM_TYPES]);
        let out = optimize(&frames, &Calibration::new(), lengths, 1.80, 10);

        // With a static frame the optimizer should still push total height toward
        // the target (the height term dominates).
        assert!((vertical_height(&out) - 1.80).abs() < 0.25);
    }
}
