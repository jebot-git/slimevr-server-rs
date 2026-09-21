//! User-action → calibration orchestration.
//!
//! The chest tracker's button sends a [`ActionType`] user action; this module turns
//! it into the corresponding per-tracker calibration update.

use firmware_protocol::ActionType;
use nalgebra::{Quaternion, UnitQuaternion};

use crate::calibration::Calibration;
use crate::feeder::HmdPose;
use crate::tracker::TrackerRegistry;

/// Handle a tracker user action (full / yaw / mounting reset).
pub fn handle_user_action(
    registry: &TrackerRegistry,
    calib: &mut Calibration,
    hmd: Option<&HmdPose>,
    action: &ActionType,
) {
    match action {
        ActionType::Reset | ActionType::ResetYaw | ActionType::ResetMounting => {
            let reference = reference_rotation(registry, hmd).unwrap_or_else(UnitQuaternion::identity);
            let mut count = 0;
            for t in registry.iter() {
                let Some(raw) = t.rotation else { continue };
                match action {
                    ActionType::Reset => calib.tracker_mut(t.id()).full_reset(raw, reference),
                    ActionType::ResetYaw => calib.tracker_mut(t.id()).yaw_reset(raw, reference),
                    ActionType::ResetMounting => {
                        calib.tracker_mut(t.id()).mounting_reset(raw, reference, t.position)
                    }
                    _ => unreachable!(),
                }
                count += 1;
            }
            tracing::info!(?action, "calibration applied to {count} trackers");
        }
        _ => {}
    }
}

/// Pick the reference rotation for a reset. The synthetic trackers are emitted
/// HMD-anchored (`hmd_rot * local_rot`), so the local frame must be HMD-relative:
/// the HMD (feeder) pose is the primary yaw reference, matching the Java server.
/// Fall back to head → chest → first tracker when no HMD pose is available.
fn reference_rotation(
    registry: &TrackerRegistry,
    hmd: Option<&HmdPose>,
) -> Option<UnitQuaternion<f32>> {
    if let Some(h) = hmd {
        // HmdPose = [x, y, z, qx, qy, qz, qw] → nalgebra scalar-first (w, i, j, k).
        return Some(UnitQuaternion::from_quaternion(Quaternion::new(
            h[6], h[3], h[4], h[5],
        )));
    }
    for position in [1u8, 4u8] {
        // TrackerPosition HEAD, CHEST
        if let Some(t) = registry.iter().find(|t| t.position == position) {
            if let Some(r) = t.rotation {
                return Some(r);
            }
        }
    }
    registry.iter().find_map(|t| t.rotation)
}
