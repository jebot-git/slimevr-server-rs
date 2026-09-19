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

use crate::calibration::Calibration;
use crate::feeder::HmdPose;
use crate::tracker::Tracker;

/// Map a SlimeVR `TrackerPosition` id (from SENSOR_INFO) to the bone it drives.
pub fn bone_kind_for_position(position: u8) -> Option<BoneKind> {
    Some(match position {
        4 => BoneKind::Chest,
        6 => BoneKind::Hip,
        7 => BoneKind::ThighL,
        8 => BoneKind::ThighR,
        // The HaritoraX 2 "ankle" tracker sits on the shin (lower leg).
        9 => BoneKind::AnkleL,
        10 => BoneKind::AnkleR,
        // Real foot trackers (expansion set).
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
        BoneKind::UpperChest => 22,
        BoneKind::Chest => 3,
        BoneKind::Waist => 4,
        BoneKind::Hip => 5,
        BoneKind::HipL => 23,
        BoneKind::HipR => 24,
        BoneKind::ThighL => 6,
        BoneKind::ThighR => 7,
        BoneKind::AnkleL => 8,
        BoneKind::AnkleR => 9,
        BoneKind::FootL => 10,
        BoneKind::FootR => 11,
        BoneKind::ShoulderL => 20,
        BoneKind::ShoulderR => 21,
        BoneKind::UpperArmL => 16,
        BoneKind::UpperArmR => 17,
        BoneKind::ForearmL => 14,
        BoneKind::ForearmR => 15,
        BoneKind::WristL => 18,
        BoneKind::WristR => 19,
    })
}

/// Compute bone lengths from a user's height using standard anthropometric
/// proportions (fractions of stature; Drillis & Contini-style ratios). The head
/// (≈13%) is the skeleton root and is not a bone here.
pub fn bone_lengths_from_height(h: f32) -> BoneMap<f32> {
    let mut m = BoneMap::new([0.0f32; BoneKind::NUM_TYPES]);
    m[BoneKind::Neck] = 0.052 * h;
    m[BoneKind::UpperChest] = 0.06 * h;
    m[BoneKind::Chest] = 0.09 * h;
    m[BoneKind::Waist] = 0.07 * h;
    m[BoneKind::Hip] = 0.05 * h;
    m[BoneKind::HipL] = 0.03 * h;
    m[BoneKind::HipR] = 0.03 * h;
    m[BoneKind::ThighL] = 0.245 * h;
    m[BoneKind::ThighR] = 0.245 * h;
    m[BoneKind::AnkleL] = 0.246 * h;
    m[BoneKind::AnkleR] = 0.246 * h;
    m[BoneKind::FootL] = 0.039 * h;
    m[BoneKind::FootR] = 0.039 * h;
    m[BoneKind::ShoulderL] = 0.06 * h;
    m[BoneKind::ShoulderR] = 0.06 * h;
    m[BoneKind::UpperArmL] = 0.186 * h;
    m[BoneKind::UpperArmR] = 0.186 * h;
    m[BoneKind::ForearmL] = 0.146 * h;
    m[BoneKind::ForearmR] = 0.146 * h;
    m[BoneKind::WristL] = 0.108 * h;
    m[BoneKind::WristR] = 0.108 * h;
    m
}

/// A solved bone's pose: global rotation, head-joint position, and length.
#[derive(Debug, Clone, Copy)]
pub struct BonePose {
    pub rotation: UnitQuaternion<f32>,
    /// Head (parent-side) joint position in the skeleton's global frame.
    pub head_pos: [f32; 3],
    pub length: f32,
}

/// Solve the skeleton from the tracker set and return `BodyPart id → BonePose`.
///
/// Each tracker's raw rotation is first adjusted by its mounting offset and the
/// global heading (see [`Calibration`]), then fed as its bone's global rotation.
/// The HMD anchors the head (position + rotation), matching the Java server's
/// head tracking, so the solved pose is in the tracking frame. `height_m` drives
/// the autobone bone lengths.
pub fn solve_pose(
    trackers: impl Iterator<Item = Tracker>,
    calib: &Calibration,
    height_m: f32,
    hmd: Option<&HmdPose>,
) -> HashMap<u8, BonePose> {
    solve_pose_with_lengths(trackers, calib, bone_lengths_from_height(height_m), hmd)
}

/// Like [`solve_pose`], but with explicit bone lengths (used by autobone).
pub fn solve_pose_with_lengths(
    trackers: impl Iterator<Item = Tracker>,
    calib: &Calibration,
    lengths: BoneMap<f32>,
    hmd: Option<&HmdPose>,
) -> HashMap<u8, BonePose> {
    let mut skeleton = Skeleton::new(&SkeletonConfig::new(lengths));

    // Anchor the head at the HMD pose (tracking frame). The body trackers then
    // FK from it, overriding the head's pitch/roll so the body stays upright.
    if let Some(h) = hmd {
        let hmd_rot =
            UnitQuaternion::from_quaternion(Quaternion::new(h[6], h[3], h[4], h[5]));
        skeleton.attach_input_tracker(
            BoneKind::Neck,
            [hmd_rot.w, hmd_rot.i, hmd_rot.j, hmd_rot.k],
        );
        skeleton.set_root_position(skeletal_model::Point::new(h[0], h[1], h[2]));
    }

    for t in trackers {
        if let (Some(bone), Some(raw)) = (bone_kind_for_position(t.position), t.rotation) {
            let adjusted = calib.adjust(t.mac, raw);
            skeleton.attach_input_tracker(bone, [adjusted.w, adjusted.i, adjusted.j, adjusted.k]);
        }
    }

    if skeleton.solve().is_err() {
        return HashMap::new();
    }

    let mut pose = HashMap::new();
    for bone in BoneKind::iter() {
        if let Some(body_part) = body_part_for_bone(bone) {
            let [w, i, j, k] = skeleton.bone_output_rot(bone);
            let head = skeleton.bone_head_pos(bone);
            pose.insert(
                body_part,
                BonePose {
                    rotation: UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k)),
                    head_pos: [head.x, head.y, head.z],
                    length: skeleton.bone_length(bone),
                },
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
        assert_eq!(bone_kind_for_position(9), Some(BoneKind::AnkleL));
        assert_eq!(bone_kind_for_position(10), Some(BoneKind::AnkleR));
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
        let pose = solve_pose(std::iter::once(t), &Calibration::new(), 1.80, None);
        // All 21 bones map to a SolarXR body part.
        assert_eq!(pose.len(), 21);
        assert!(pose.contains_key(&3)); // chest
        assert!(pose.contains_key(&10)); // left foot (filled by FK)
        assert!(pose.contains_key(&22)); // upper chest
        assert!(pose.contains_key(&23)); // left hip
    }
}
