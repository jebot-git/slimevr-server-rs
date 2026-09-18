//! Skeleton pose estimation: tracker rotations → solved skeleton pose.
//!
//! Uses the (now-implemented) forward-kinematics solver in the vendored
//! `skeletal_model` crate: tracker rotations pin their bone's rotation, the solver
//! propagates them through the bone tree, and every bone's global rotation is
//! emitted as a SolarXR `BodyPart`.

use std::collections::HashMap;

use nalgebra::{Quaternion, UnitQuaternion};
use skeletal_model::skeleton::SkeletonConfig;
use skeletal_model::{BoneKind, BoneMap, Skeleton};

use crate::tracker::Tracker;

/// Map a SlimeVR `TrackerPosition` id (from SENSOR_INFO) to the bone it drives.
pub fn bone_kind_for_position(position: u8) -> Option<BoneKind> {
    Some(match position {
        4 => BoneKind::Chest,
        6 => BoneKind::Hip,
        7 => BoneKind::ThighL,
        8 => BoneKind::ThighR,
        11 => BoneKind::FootL,
        12 => BoneKind::FootR,
        15 => BoneKind::UpperArmL,
        16 => BoneKind::UpperArmR,
        _ => return None,
    })
}

/// Map a bone to the SolarXR `BodyPart` id we emit for it.
pub fn body_part_for_bone(bone: BoneKind) -> Option<u8> {
    Some(match bone {
        BoneKind::Neck => 2,
        BoneKind::Chest => 3,
        BoneKind::Waist => 4,
        BoneKind::Hip => 5,
        BoneKind::ThighL => 6,
        BoneKind::ThighR => 7,
        BoneKind::AnkleL => 8,
        BoneKind::AnkleR => 9,
        BoneKind::FootL => 10,
        BoneKind::FootR => 11,
        BoneKind::UpperArmL => 16,
        BoneKind::UpperArmR => 17,
        BoneKind::ForearmL => 14,
        BoneKind::ForearmR => 15,
        BoneKind::WristL => 18,
        BoneKind::WristR => 19,
    })
}

/// Approximate adult bone lengths (meters). Placeholder until real proportions /
/// autobone land.
fn default_bone_lengths() -> BoneMap<f32> {
    let mut m = BoneMap::new([0.0f32; BoneKind::NUM_TYPES]);
    m[BoneKind::Neck] = 0.10;
    m[BoneKind::Chest] = 0.22;
    m[BoneKind::Waist] = 0.10;
    m[BoneKind::Hip] = 0.10;
    m[BoneKind::ThighL] = 0.45;
    m[BoneKind::ThighR] = 0.45;
    m[BoneKind::AnkleL] = 0.42;
    m[BoneKind::AnkleR] = 0.42;
    m[BoneKind::FootL] = 0.07;
    m[BoneKind::FootR] = 0.07;
    m[BoneKind::UpperArmL] = 0.30;
    m[BoneKind::UpperArmR] = 0.30;
    m[BoneKind::ForearmL] = 0.26;
    m[BoneKind::ForearmR] = 0.26;
    m[BoneKind::WristL] = 0.15;
    m[BoneKind::WristR] = 0.15;
    m
}

/// Build a [`Skeleton`] with default bone lengths.
fn build_skeleton() -> Skeleton {
    Skeleton::new(&SkeletonConfig::new(default_bone_lengths()))
}

/// Solve the skeleton from the tracker set and return `BodyPart id → rotation`.
///
/// Tracker rotations are fed directly as the bone's global rotation. The FK solver
/// then fills in every untracked bone. The returned quaternion is in the
/// `skeletal_model` global frame (see its `conventions` module).
pub fn solve_pose(trackers: impl Iterator<Item = Tracker>) -> HashMap<u8, UnitQuaternion<f32>> {
    let mut skeleton = build_skeleton();

    for t in trackers {
        if let (Some(bone), Some(rot)) = (bone_kind_for_position(t.position), t.rotation) {
            // nalgebra 0.32 UnitQuaternion -> [w, i, j, k] for the vendored crate.
            skeleton.attach_input_tracker(bone, [rot.w, rot.i, rot.j, rot.k]);
        }
    }

    if skeleton.solve().is_err() {
        return HashMap::new();
    }

    let mut pose = HashMap::new();
    for bone in BoneKind::iter() {
        if let Some(body_part) = body_part_for_bone(bone) {
            let [w, i, j, k] = skeleton.bone_output_rot(bone);
            pose.insert(
                body_part,
                UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k)),
            );
        }
    }
    pose
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_standard_fbt_positions() {
        assert_eq!(bone_kind_for_position(4), Some(BoneKind::Chest));
        assert_eq!(bone_kind_for_position(6), Some(BoneKind::Hip));
        assert_eq!(bone_kind_for_position(7), Some(BoneKind::ThighL));
        assert_eq!(bone_kind_for_position(8), Some(BoneKind::ThighR));
        assert_eq!(bone_kind_for_position(11), Some(BoneKind::FootL));
        assert_eq!(bone_kind_for_position(12), Some(BoneKind::FootR));
        assert_eq!(bone_kind_for_position(15), Some(BoneKind::UpperArmL));
        assert_eq!(bone_kind_for_position(16), Some(BoneKind::UpperArmR));
        assert_eq!(bone_kind_for_position(0), None);
    }

    #[test]
    fn bone_to_body_part() {
        assert_eq!(body_part_for_bone(BoneKind::Chest), Some(3));
        assert_eq!(body_part_for_bone(BoneKind::Hip), Some(5));
        assert_eq!(body_part_for_bone(BoneKind::FootL), Some(10));
        assert_eq!(body_part_for_bone(BoneKind::UpperArmL), Some(16));
    }

    #[test]
    fn solves_full_skeleton_from_one_tracker() {
        // A single chest tracker at identity should still solve every bone
        // (untracked bones inherit their parent's rotation).
        let t = Tracker {
            sensor_id: 0,
            addr: "127.0.0.1:1".parse().unwrap(),
            mac: [0; 6],
            position: 4, // chest
            rotation: Some(UnitQuaternion::identity()),
            accel: None,
            last_seen: std::time::Instant::now(),
        };
        let pose = solve_pose(std::iter::once(t));
        // All 16 bones map to a SolarXR body part.
        assert_eq!(pose.len(), 16);
        assert!(pose.contains_key(&3)); // chest
        assert!(pose.contains_key(&10)); // left foot (filled by FK)
    }
}
