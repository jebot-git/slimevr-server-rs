//! User-action → calibration orchestration.
//!
//! The chest tracker's button sends a [`ActionType`] user action; this module turns
//! it into the corresponding calibration update.

use firmware_protocol::ActionType;
use nalgebra::UnitQuaternion;

use crate::calibration::Calibration;
use crate::skeleton::bone_calibration_rotation;
use crate::tracker::TrackerRegistry;

/// Handle a tracker user action (full / yaw / mounting reset).
pub fn handle_user_action(
    registry: &TrackerRegistry,
    calib: &mut Calibration,
    action: &ActionType,
) {
    match action {
        ActionType::Reset | ActionType::ResetYaw => {
            if let Some(reference) = reference_rotation(registry) {
                calib.full_reset(reference);
                tracing::info!("full/yaw reset: heading updated");
            } else {
                tracing::warn!("full reset: no reference tracker to align to");
            }
        }
        ActionType::ResetMounting => {
            let mut count = 0;
            for t in registry.iter() {
                if let (Some(raw), Some(calib_rot)) =
                    (t.rotation, bone_calibration_rotation(t.position))
                {
                    calib.mounting_reset(t.mac, raw, calib_rot);
                    count += 1;
                }
            }
            tracing::info!("mounting reset: {count} trackers calibrated");
        }
        _ => {}
    }
}

/// Pick the reference tracker for a full reset: head → chest → first tracker.
fn reference_rotation(registry: &TrackerRegistry) -> Option<UnitQuaternion<f32>> {
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
