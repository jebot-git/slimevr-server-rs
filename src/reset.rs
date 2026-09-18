//! User-action → calibration orchestration.
//!
//! The chest tracker's button sends a [`ActionType`] user action; this module turns
//! it into the corresponding per-tracker calibration update.

use firmware_protocol::ActionType;
use nalgebra::UnitQuaternion;

use crate::calibration::Calibration;
use crate::tracker::TrackerRegistry;

/// Handle a tracker user action (full / yaw / mounting reset).
pub fn handle_user_action(
    registry: &TrackerRegistry,
    calib: &mut Calibration,
    action: &ActionType,
) {
    match action {
        ActionType::Reset | ActionType::ResetYaw | ActionType::ResetMounting => {
            let reference = reference_rotation(registry).unwrap_or_else(UnitQuaternion::identity);
            let mut count = 0;
            for t in registry.iter() {
                let Some(raw) = t.rotation else { continue };
                match action {
                    ActionType::Reset => calib.tracker_mut(t.mac).full_reset(raw, reference),
                    ActionType::ResetYaw => calib.tracker_mut(t.mac).yaw_reset(raw, reference),
                    ActionType::ResetMounting => {
                        calib.tracker_mut(t.mac).mounting_reset(raw, reference)
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

/// Pick the reference tracker for a full/yaw reset: head → chest → first tracker.
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
