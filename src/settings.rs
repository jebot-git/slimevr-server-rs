//! Validated body-tracking settings shared by startup config and live controls.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrackingSettings {
    pub height_m: f32,
    /// Slerp blend toward the newest sample: 0 disables, smaller nonzero = smoother.
    pub smoothing: f32,
    /// Prediction horizon in seconds.
    pub prediction: f32,
    pub drift_correction: bool,
    pub drift_amount: f32,
}
impl Default for TrackingSettings {
    fn default() -> Self {
        Self { height_m: 1.8, smoothing: 0.3, prediction: 0.0,
            drift_correction: false, drift_amount: 0.5 }
    }
}
impl TrackingSettings {
    pub fn validate(&self) -> anyhow::Result<()> {
        anyhow::ensure!(self.height_m.is_finite() && (0.5..=3.0).contains(&self.height_m),
            "height_m must be between 0.5 and 3.0 metres");
        anyhow::ensure!(self.smoothing.is_finite() && (0.0..=1.0).contains(&self.smoothing),
            "smoothing must be between 0 and 1 (0 disables smoothing)");
        anyhow::ensure!(self.prediction.is_finite() && (0.0..=0.2).contains(&self.prediction),
            "prediction must be between 0 and 0.2 seconds");
        anyhow::ensure!(self.drift_amount.is_finite() && (0.0..=1.0).contains(&self.drift_amount),
            "drift_amount must be between 0 and 1");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_invalid_settings_before_application() {
        for settings in [
            TrackingSettings { smoothing: f32::NAN, ..Default::default() },
            TrackingSettings { smoothing: -0.1, ..Default::default() },
            TrackingSettings { prediction: 0.21, ..Default::default() },
            TrackingSettings { height_m: 0.0, ..Default::default() },
            TrackingSettings { drift_amount: 1.1, ..Default::default() },
        ] { assert!(settings.validate().is_err()); }
        TrackingSettings::default().validate().unwrap();
    }
}
