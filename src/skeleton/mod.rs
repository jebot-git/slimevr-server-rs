//! Skeleton pose estimation: tracker rotations → per-bone pose.
//!
//! **First-milestone implementation**: a passthrough mapping — each tracker's
//! fused rotation is copied straight to its corresponding bone. The real
//! forward-kinematics solver (turning the partial tracker set into a complete,
//! anatomically-constrained skeleton) is the main remaining work; see
//! [`ROADMAP.md`]. The [`skeletal_model`] crate is wired in below and its graph is
//! built as the target representation, but its upstream `do_fk` solver is still
//! unimplemented (`todo!()`), which is exactly what this project exists to finish.

use std::collections::HashMap;

use nalgebra::UnitQuaternion;

use crate::tracker::Tracker;

/// Map a SlimeVR `TrackerPosition` id (from the tracker's SENSOR_INFO packet) to
/// the SolarXR `BodyPart` id we emit for that tracker.
///
/// Values taken from SlimeVR's `TrackerPosition.kt` (its `id` and `bodyPart`
/// columns).
pub fn body_part_for_position(position: u8) -> Option<u8> {
    Some(match position {
        4 => 3, // CHEST
        6 => 5, // HIP
        7 => 6, // LEFT_UPPER_LEG
        8 => 7, // RIGHT_UPPER_LEG
        11 => 10, // LEFT_FOOT
        12 => 11, // RIGHT_FOOT
        15 => 16, // LEFT_UPPER_ARM
        16 => 17, // RIGHT_UPPER_ARM
        _ => return None,
    })
}

/// Compute the skeleton pose as a `BodyPart id → rotation` map.
///
/// Placeholder: each tracker with a known position contributes its raw rotation
/// to its body part. No forward kinematics or filtering is applied yet.
pub fn estimate_pose(trackers: impl Iterator<Item = Tracker>) -> HashMap<u8, UnitQuaternion<f32>> {
    let mut pose = HashMap::new();
    for t in trackers {
        let Some(body_part) = body_part_for_position(t.position) else {
            continue;
        };
        if let Some(q) = t.rotation {
            pose.insert(body_part, q);
        }
    }
    pose
}

/// Build the [`skeletal_model::Skeleton`] graph (zero bone lengths).
///
/// This proves the SlimeVR-Rust dependency is wired in and gives us the target
/// data structure the solver will operate on. Bone lengths + tracker attachment +
/// the FK solver are TODO.
pub fn build_skeletal_model() -> skeletal_model::Skeleton {
    let lengths = skeletal_model::BoneMap::new([0.0; skeletal_model::BoneKind::num_types()]);
    let config = skeletal_model::skeleton::SkeletonConfig::new(lengths);
    skeletal_model::Skeleton::new(&config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_standard_fbt_positions() {
        assert_eq!(body_part_for_position(4), Some(3)); // chest
        assert_eq!(body_part_for_position(6), Some(5)); // hip
        assert_eq!(body_part_for_position(7), Some(6)); // left upper leg
        assert_eq!(body_part_for_position(8), Some(7)); // right upper leg
        assert_eq!(body_part_for_position(11), Some(10)); // left foot
        assert_eq!(body_part_for_position(12), Some(11)); // right foot
        assert_eq!(body_part_for_position(15), Some(16)); // left upper arm
        assert_eq!(body_part_for_position(16), Some(17)); // right upper arm
    }

    #[test]
    fn unassigned_is_none() {
        assert_eq!(body_part_for_position(0), None);
        assert_eq!(body_part_for_position(99), None);
    }

    #[test]
    fn skeletal_model_builds() {
        // Just ensure the boilerplate graph constructs without panicking.
        let _ = build_skeletal_model();
    }
}
