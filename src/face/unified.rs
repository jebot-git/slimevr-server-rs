//! Unified face/eye expression model, ported from oscavmgr's
//! `src/core/ext_tracking/unified.rs`.
//!
//! The model is a flat `[f32; NUM_SHAPES]` buffer: 89 "unified" (biometric/source)
//! shapes at indices 0..=88, followed by 56 derived "combined" shapes at
//! 89..=144. `calc_combined` derives the combined shapes from the unified ones and
//! `apply_to_bundle` fans each shape out to the OSC addresses mapped by an
//! OSC-JSON avatar description (`MysteryParam`).

use glam::Vec3;
use rosc::{OscBundle, OscType};
use std::collections::HashMap;
use std::sync::Arc;
use strum::{EnumCount, EnumString, IntoStaticStr};

use super::bundle::AvatarBundle;
use super::mystery::MysteryParam;

/// Cached incoming `/avatar/parameters/*` values.
pub type FaceParams = HashMap<Arc<str>, OscType>;

pub const NUM_SHAPES: usize = UnifiedExpressions::COUNT + CombinedExpression::COUNT;
#[cfg(feature = "face-xr")]
pub type UnifiedShapes = [f32; NUM_SHAPES];

#[cfg(feature = "face-xr")]
pub trait UnifiedShapeAccessors {
    fn setu(&mut self, exp: UnifiedExpressions, value: f32);
}

#[cfg(feature = "face-xr")]
impl UnifiedShapeAccessors for UnifiedShapes {
    fn setu(&mut self, exp: UnifiedExpressions, value: f32) {
        self[exp as usize] = value;
    }
}

#[derive(Debug, Clone)]
pub struct UnifiedTrackingData {
    pub eyes: [Option<Vec3>; 2],
    pub shapes: [f32; NUM_SHAPES],
    pub eye_active: bool,
    eye_tracking: Option<bool>,
    expression_tracking: Option<bool>,
    lip_tracking: Option<bool>,
}

impl Default for UnifiedTrackingData {
    fn default() -> Self {
        Self {
            eyes: [None, None],
            shapes: [0.0; NUM_SHAPES],
            eye_active: false,
            eye_tracking: None,
            expression_tracking: None,
            lip_tracking: None,
        }
    }
}

impl UnifiedTrackingData {
    #[inline(always)]
    pub fn getu(&self, exp: UnifiedExpressions) -> f32 {
        self.shapes[exp as usize]
    }
    #[inline(always)]
    pub fn getc(&self, exp: CombinedExpression) -> f32 {
        self.shapes[exp as usize]
    }
    #[inline(always)]
    pub fn setu(&mut self, exp: UnifiedExpressions, value: f32) {
        self.shapes[exp as usize] = value;
    }
    #[inline(always)]
    pub fn setc(&mut self, exp: CombinedExpression, value: f32) {
        self.shapes[exp as usize] = value;
    }

    pub fn calc_combined(&mut self, delta_t: f32, params: &FaceParams) {
        let left_eye_openness =
            (1. - self.getu(UnifiedExpressions::EyeClosedLeft) * 1.5).clamp(0., 1.);
        self.setc(
            CombinedExpression::EyeLidLeft,
            left_eye_openness * 0.75
                + self.getu(UnifiedExpressions::EyeWideLeft) * left_eye_openness * 0.25,
        );

        let right_eye_openness =
            (1. - self.getu(UnifiedExpressions::EyeClosedRight) * 1.5).clamp(0., 1.);
        self.setc(
            CombinedExpression::EyeLidRight,
            right_eye_openness * 0.75
                + self.getu(UnifiedExpressions::EyeWideRight) * right_eye_openness * 0.25,
        );

        self.setc(
            CombinedExpression::EyeLid,
            (self.getc(CombinedExpression::EyeLidLeft)
                + self.getc(CombinedExpression::EyeLidRight))
                * 0.5,
        );

        self.setc(
            CombinedExpression::EyeX,
            (self.getu(UnifiedExpressions::EyeLeftX) + self.getu(UnifiedExpressions::EyeRightX))
                * 0.5,
        );

        let brow_down_left = self.getu(UnifiedExpressions::BrowLowererLeft) * 0.75
            + self.getu(UnifiedExpressions::BrowPinchLeft) * 0.25;
        let brow_down_right = self.getu(UnifiedExpressions::BrowLowererRight) * 0.75
            + self.getu(UnifiedExpressions::BrowPinchRight) * 0.25;

        self.setc(CombinedExpression::BrowDownLeft, brow_down_left);
        self.setc(CombinedExpression::BrowDownRight, brow_down_right);

        let brow_outer_up = (self.getu(UnifiedExpressions::BrowOuterUpLeft)
            + self.getu(UnifiedExpressions::BrowOuterUpRight))
            * 0.5;
        self.setc(CombinedExpression::BrowOuterUp, brow_outer_up);

        let brow_inner_up = (self.getu(UnifiedExpressions::BrowInnerUpLeft)
            + self.getu(UnifiedExpressions::BrowInnerUpRight))
            * 0.5;
        self.setc(CombinedExpression::BrowInnerUp, brow_inner_up);

        self.setc(
            CombinedExpression::BrowUp,
            (brow_outer_up + brow_inner_up) * 0.5,
        );

        let brow_exp_left = (self.getu(UnifiedExpressions::BrowInnerUpLeft) * 0.5
            + self.getu(UnifiedExpressions::BrowOuterUpLeft) * 0.5)
            - brow_down_left;
        let brow_exp_right = (self.getu(UnifiedExpressions::BrowInnerUpRight) * 0.5
            + self.getu(UnifiedExpressions::BrowOuterUpRight) * 0.5)
            - brow_down_right;

        self.setc(CombinedExpression::BrowExpressionLeft, brow_exp_left);
        self.setc(CombinedExpression::BrowExpressionRight, brow_exp_right);
        self.setc(
            CombinedExpression::BrowExpression,
            (brow_exp_left + brow_exp_right) * 0.5,
        );

        let ape_faceness = self.getu(UnifiedExpressions::MouthClosed) * 0.75;

        let mouth_smile_left = self.getu(UnifiedExpressions::MouthCornerPullLeft) * 0.75
            + self.getu(UnifiedExpressions::MouthCornerSlantLeft) * 0.25;
        let mouth_smile_right = self.getu(UnifiedExpressions::MouthCornerPullRight) * 0.75
            + self.getu(UnifiedExpressions::MouthCornerSlantRight) * 0.25;

        let mouth_sad_left = self.getu(UnifiedExpressions::MouthFrownLeft) * 0.75
            + self.getu(UnifiedExpressions::MouthStretchLeft) * 0.25;
        let mouth_sad_right = self.getu(UnifiedExpressions::MouthFrownRight) * 0.75
            + self.getu(UnifiedExpressions::MouthStretchRight) * 0.25;

        self.setu(
            UnifiedExpressions::EyeSquintLeft,
            (self.getu(UnifiedExpressions::EyeSquintLeft) + mouth_smile_left * 0.6).min(1.0)
                * left_eye_openness,
        );
        self.setu(
            UnifiedExpressions::EyeSquintRight,
            (self.getu(UnifiedExpressions::EyeSquintRight) + mouth_smile_right * 0.6).min(1.0)
                * right_eye_openness,
        );

        self.setc(
            CombinedExpression::EyeSquint,
            (self.getu(UnifiedExpressions::EyeSquintLeft)
                + self.getu(UnifiedExpressions::EyeSquintRight))
                * 0.5,
        );

        self.setc(
            CombinedExpression::MouthSmileLeft,
            mouth_smile_left * ape_faceness,
        );
        self.setc(
            CombinedExpression::MouthSmileRight,
            mouth_smile_right * ape_faceness,
        );
        self.setc(
            CombinedExpression::MouthSadLeft,
            mouth_sad_left * ape_faceness,
        );
        self.setc(
            CombinedExpression::MouthSadRight,
            mouth_sad_right * ape_faceness,
        );

        self.setc(
            CombinedExpression::MouthUpperX,
            self.getu(UnifiedExpressions::MouthUpperRight)
                - self.getu(UnifiedExpressions::MouthUpperLeft),
        );
        self.setc(
            CombinedExpression::MouthLowerX,
            self.getu(UnifiedExpressions::MouthLowerRight)
                - self.getu(UnifiedExpressions::MouthLowerLeft),
        );

        self.setc(
            CombinedExpression::SmileSadLeft,
            (mouth_smile_left - mouth_sad_left) * ape_faceness,
        );
        self.setc(
            CombinedExpression::SmileSadRight,
            (mouth_smile_right - mouth_sad_right) * ape_faceness,
        );
        self.setc(
            CombinedExpression::SmileSad,
            (mouth_smile_left - mouth_sad_left + mouth_smile_right - mouth_sad_right)
                * 0.5
                * ape_faceness,
        );
        self.setc(
            CombinedExpression::SmileFrownLeft,
            mouth_smile_left - self.getu(UnifiedExpressions::MouthFrownLeft) + ape_faceness,
        );
        self.setc(
            CombinedExpression::SmileFrownRight,
            mouth_smile_right - self.getu(UnifiedExpressions::MouthFrownRight) + ape_faceness,
        );
        self.setc(
            CombinedExpression::SmileFrown,
            (mouth_smile_left - self.getu(UnifiedExpressions::MouthFrownLeft) + mouth_smile_right
                - self.getu(UnifiedExpressions::MouthFrownRight))
                * 0.5
                + ape_faceness,
        );
        self.setc(
            CombinedExpression::CheekPuffSuckLeft,
            self.getu(UnifiedExpressions::CheekPuffLeft)
                - self.getu(UnifiedExpressions::CheekSuckLeft),
        );
        self.setc(
            CombinedExpression::CheekPuffSuckRight,
            self.getu(UnifiedExpressions::CheekPuffRight)
                - self.getu(UnifiedExpressions::CheekSuckRight),
        );
        self.setc(
            CombinedExpression::CheekPuffSuck,
            (self.getu(UnifiedExpressions::CheekPuffLeft)
                + self.getu(UnifiedExpressions::CheekPuffRight)
                - self.getu(UnifiedExpressions::CheekSuckLeft)
                - self.getu(UnifiedExpressions::CheekSuckRight))
                * 0.5,
        );

        self.setc(
            CombinedExpression::CheekSquint,
            (self.getu(UnifiedExpressions::CheekSquintLeft)
                + self.getu(UnifiedExpressions::CheekSquintRight))
                * 0.5,
        );

        self.setc(
            CombinedExpression::LipSuckUpper,
            (self.getu(UnifiedExpressions::LipSuckUpperLeft)
                + self.getu(UnifiedExpressions::LipSuckUpperRight))
                * 0.5,
        );
        self.setc(
            CombinedExpression::LipSuckLower,
            (self.getu(UnifiedExpressions::LipSuckLowerLeft)
                + self.getu(UnifiedExpressions::LipSuckLowerRight))
                * 0.5,
        );
        self.setc(
            CombinedExpression::LipSuck,
            (self.getu(UnifiedExpressions::LipSuckLowerLeft)
                + self.getu(UnifiedExpressions::LipSuckLowerRight)
                + self.getu(UnifiedExpressions::LipSuckUpperLeft)
                + self.getu(UnifiedExpressions::LipSuckUpperRight))
                * 0.25,
        );
        self.setc(
            CombinedExpression::MouthStretchTightenLeft,
            self.getu(UnifiedExpressions::MouthStretchLeft)
                - self.getu(UnifiedExpressions::MouthTightenerLeft),
        );
        self.setc(
            CombinedExpression::MouthStretchTightenRight,
            self.getu(UnifiedExpressions::MouthStretchRight)
                - self.getu(UnifiedExpressions::MouthTightenerRight),
        );

        self.setc(
            CombinedExpression::MouthStretch,
            (self.getu(UnifiedExpressions::MouthStretchLeft)
                + self.getu(UnifiedExpressions::MouthStretchRight))
                * 0.5,
        );
        self.setc(
            CombinedExpression::MouthTightener,
            (self.getu(UnifiedExpressions::MouthTightenerLeft)
                + self.getu(UnifiedExpressions::MouthTightenerRight))
                * 0.5,
        );
        self.setc(
            CombinedExpression::MouthDimple,
            (self.getu(UnifiedExpressions::MouthDimpleLeft)
                + self.getu(UnifiedExpressions::MouthDimpleRight))
                * 0.5,
        );

        let mouth_upper_up = (self.getu(UnifiedExpressions::MouthUpperUpLeft)
            + self.getu(UnifiedExpressions::MouthUpperUpRight))
            * 0.5;
        let mouth_lower_down = (self.getu(UnifiedExpressions::MouthLowerDownLeft)
            + self.getu(UnifiedExpressions::MouthLowerDownRight))
            * 0.5;

        self.setc(CombinedExpression::MouthUpperUp, mouth_upper_up);
        self.setc(CombinedExpression::MouthLowerDown, mouth_lower_down);
        self.setc(
            CombinedExpression::MouthOpen,
            (mouth_upper_up + mouth_lower_down) * 0.5,
        );
        self.setc(
            CombinedExpression::MouthX,
            (self.getu(UnifiedExpressions::MouthUpperRight)
                + self.getu(UnifiedExpressions::MouthLowerRight)
                - self.getu(UnifiedExpressions::MouthUpperLeft)
                - self.getu(UnifiedExpressions::MouthLowerLeft))
                * 0.5,
        );
        self.setc(
            CombinedExpression::JawX,
            self.getu(UnifiedExpressions::JawRight) - self.getu(UnifiedExpressions::JawLeft),
        );
        self.setc(
            CombinedExpression::JawZ,
            self.getu(UnifiedExpressions::JawForward) - self.getu(UnifiedExpressions::JawBackward),
        );
        let lip_pucker_left = (self.getu(UnifiedExpressions::LipPuckerLowerLeft)
            + self.getu(UnifiedExpressions::LipPuckerUpperLeft))
            * 0.5;
        let lip_pucker_right = (self.getu(UnifiedExpressions::LipPuckerLowerRight)
            + self.getu(UnifiedExpressions::LipPuckerUpperRight))
            * 0.5;
        self.setc(
            CombinedExpression::LipPucker,
            (lip_pucker_left + lip_pucker_right) * 0.5,
        );
        let lip_funnel_upper = (self.getu(UnifiedExpressions::LipFunnelUpperLeft)
            + self.getu(UnifiedExpressions::LipFunnelUpperRight))
            * 0.5;
        let lip_funnel_lower = (self.getu(UnifiedExpressions::LipFunnelLowerLeft)
            + self.getu(UnifiedExpressions::LipFunnelLowerRight))
            * 0.5;

        self.setc(CombinedExpression::LipFunnelUpper, lip_funnel_upper);
        self.setc(CombinedExpression::LipFunnelLower, lip_funnel_lower);
        self.setc(
            CombinedExpression::LipFunnel,
            (lip_funnel_upper + lip_funnel_lower) * 0.5,
        );

        self.setc(
            CombinedExpression::MouthPress,
            (self.getu(UnifiedExpressions::MouthPressLeft)
                + self.getu(UnifiedExpressions::MouthPressRight))
                * 0.5,
        );
        self.setc(
            CombinedExpression::NoseSneer,
            (self.getu(UnifiedExpressions::NoseSneerLeft)
                + self.getu(UnifiedExpressions::NoseSneerRight))
                * 0.5,
        );

        self.setc(
            CombinedExpression::EarLeft,
            (self.getu(UnifiedExpressions::BrowInnerUpLeft)
                + self.getu(UnifiedExpressions::EyeWideLeft)
                - self.getu(UnifiedExpressions::EyeSquintLeft)
                - self.getu(UnifiedExpressions::BrowPinchLeft))
            .clamp(-1.0, 1.0),
        );
        self.setc(
            CombinedExpression::EarRight,
            (self.getu(UnifiedExpressions::BrowInnerUpLeft)
                + self.getu(UnifiedExpressions::EyeWideRight)
                - self.getu(UnifiedExpressions::EyeSquintRight)
                - self.getu(UnifiedExpressions::BrowPinchRight))
            .clamp(-1.0, 1.0),
        );

        self.setc(
            CombinedExpression::TongueX,
            self.getu(UnifiedExpressions::TongueRight) - self.getu(UnifiedExpressions::TongueLeft),
        );
        self.setc(
            CombinedExpression::TongueY,
            self.getu(UnifiedExpressions::TongueUp) - self.getu(UnifiedExpressions::TongueDown),
        );

        let allow_blush = !matches!(params.get("AllowBlush"), Some(OscType::Bool(false)));

        let blush_face = matches!(params.get("BlushFace"), Some(OscType::Float(f)) if *f > 0.1);
        let blush_nade = matches!(params.get("BlushNade"), Some(OscType::Float(f)) if *f > 0.1);
        let blush_eye = self.eyes[0].map(|e| e.x).unwrap_or(0.0) > 0.3;

        let rate = if allow_blush && (blush_face || blush_nade || blush_eye) {
            0.10
        } else {
            -0.05
        };

        let old_blush = self.getc(CombinedExpression::Blush);
        let new_blush = (old_blush + rate * delta_t).clamp(0.0, 1.0);
        self.setc(CombinedExpression::Blush, new_blush);
    }

    pub fn apply_to_bundle(
        &mut self,
        params: &mut [Option<MysteryParam>; NUM_SHAPES],
        bundle: &mut OscBundle,
        active: bool,
    ) {
        let eye_active = active && self.eye_active;
        if self.eye_tracking != Some(eye_active) {
            bundle.send_parameter("EyeTrackingActive", OscType::Bool(eye_active));
            self.eye_tracking = Some(eye_active);
        }
        if self.expression_tracking != Some(active) {
            bundle.send_parameter("ExpressionTrackingActive", OscType::Bool(active));
            self.expression_tracking = Some(active);
        }
        if self.lip_tracking != Some(active) {
            bundle.send_parameter("LipTrackingActive", OscType::Bool(active));
            self.lip_tracking = Some(active);
        }

        for (idx, shape) in self.shapes.iter().enumerate() {
            if let Some(param) = &mut params[idx] {
                param.send(*shape, bundle);
            }
        }

        self.apply_eye_tracking(bundle, params[CombinedExpression::EyeLidLeft as usize].is_some());
    }

    pub fn invalidate_output(&mut self) {
        self.eye_tracking = None;
        self.expression_tracking = None;
        self.lip_tracking = None;
    }

    pub fn apply_eye_tracking(&self, bundle: &mut OscBundle, has_eyelid: bool) {
        if let Some(left_euler) = self.eyes[0] {
            if !has_eyelid {
                bundle.send_tracking(
                    "/tracking/eye/EyesClosedAmount",
                    vec![OscType::Float(self.getu(UnifiedExpressions::EyeClosedLeft))],
                );
            }
            let right_euler = self.eyes[1].unwrap_or(left_euler);

            bundle.send_tracking(
                "/tracking/eye/LeftRightPitchYaw",
                vec![
                    OscType::Float(-left_euler.x.to_degrees()),
                    OscType::Float(-left_euler.y.to_degrees()),
                    OscType::Float(-right_euler.x.to_degrees()),
                    OscType::Float(-right_euler.y.to_degrees()),
                ],
            );
        }
    }
}

#[allow(unused)]
#[repr(usize)]
#[derive(Debug, Clone, Copy, strum::EnumIter, EnumCount, EnumString, IntoStaticStr)]
pub enum UnifiedExpressions {
    EyeLeftX,
    EyeRightX,
    EyeY,
    EyeClosedRight,
    EyeClosedLeft,
    EyeSquintRight,
    EyeSquintLeft,
    EyeWideRight,
    EyeWideLeft,
    BrowPinchRight,
    BrowPinchLeft,
    BrowLowererRight,
    BrowLowererLeft,
    BrowInnerUpRight,
    BrowInnerUpLeft,
    BrowOuterUpRight,
    BrowOuterUpLeft,
    NasalDilationRight,
    NasalDilationLeft,
    NasalConstrictRight,
    NasalConstrictLeft,
    CheekSquintRight,
    CheekSquintLeft,
    CheekPuffRight,
    CheekPuffLeft,
    CheekSuckRight,
    CheekSuckLeft,
    JawOpen,
    JawRight,
    JawLeft,
    JawForward,
    JawBackward,
    JawClench,
    JawMandibleRaise,
    MouthClosed,
    LipSuckUpperRight,
    LipSuckUpperLeft,
    LipSuckLowerRight,
    LipSuckLowerLeft,
    LipSuckCornerRight,
    LipSuckCornerLeft,
    LipFunnelUpperRight,
    LipFunnelUpperLeft,
    LipFunnelLowerRight,
    LipFunnelLowerLeft,
    LipPuckerUpperRight,
    LipPuckerUpperLeft,
    LipPuckerLowerRight,
    LipPuckerLowerLeft,
    MouthUpperUpRight,
    MouthUpperUpLeft,
    MouthUpperDeepenRight,
    MouthUpperDeepenLeft,
    NoseSneerRight,
    NoseSneerLeft,
    MouthLowerDownRight,
    MouthLowerDownLeft,
    MouthUpperRight,
    MouthUpperLeft,
    MouthLowerRight,
    MouthLowerLeft,
    MouthCornerPullRight,
    MouthCornerPullLeft,
    MouthCornerSlantRight,
    MouthCornerSlantLeft,
    MouthFrownRight,
    MouthFrownLeft,
    MouthStretchRight,
    MouthStretchLeft,
    MouthDimpleRight,
    MouthDimpleLeft,
    MouthRaiserUpper,
    MouthRaiserLower,
    MouthPressRight,
    MouthPressLeft,
    MouthTightenerRight,
    MouthTightenerLeft,
    TongueOut,
    TongueUp,
    TongueDown,
    TongueRight,
    TongueLeft,
    TongueRoll,
    TongueBendDown,
    TongueCurlUp,
    TongueSquish,
    TongueFlat,
    TongueTwistRight,
    TongueTwistLeft,
}

#[allow(unused)]
#[repr(usize)]
#[derive(Debug, Clone, Copy, strum::EnumIter, EnumCount, EnumString, IntoStaticStr)]
pub enum CombinedExpression {
    EyeLidLeft = UnifiedExpressions::COUNT,
    EyeLidRight,
    EyeLid,
    EyeSquint,
    EyeX,
    JawX,
    JawZ,
    BrowDownLeft,
    BrowDownRight,
    BrowOuterUp,
    BrowInnerUp,
    BrowUp,
    BrowExpressionLeft,
    BrowExpressionRight,
    BrowExpression,
    MouthX,
    MouthUpperX,
    MouthLowerX,
    MouthUpperUp,
    MouthLowerDown,
    MouthOpen,
    MouthSmileLeft,
    MouthSmileRight,
    MouthSadLeft,
    MouthSadRight,
    MouthStretchTightenLeft,
    MouthStretchTightenRight,
    MouthStretch,
    MouthTightener,
    MouthDimple,
    MouthPress,
    SmileFrownLeft,
    SmileFrownRight,
    SmileFrown,
    SmileSadLeft,
    SmileSadRight,
    SmileSad,
    LipSuckUpper,
    LipSuckLower,
    LipSuck,
    LipFunnelUpper,
    LipFunnelLower,
    LipFunnel,
    LipPuckerUpper,
    LipPuckerLower,
    LipPucker,
    NoseSneer,
    CheekPuffSuckLeft,
    CheekPuffSuckRight,
    CheekPuffSuck,
    CheekSquint,
    TongueX,
    TongueY,
    EarLeft,
    EarRight,
    Blush,
}
