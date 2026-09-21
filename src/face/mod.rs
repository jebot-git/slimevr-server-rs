//! Embedded oscavmgr face/eye pipeline, adapted from Shora.
//! See THIRD_PARTY.md for provenance and licenses.

pub mod config;
mod babble;
mod bundle;
mod mystery;
mod osc;
mod oscjson;
mod oscquery;
mod output;
mod unified;

#[cfg(feature = "face-xr")]
mod face2_fb;
#[cfg(feature = "face-xr")]
mod openxr;

pub use osc::FaceRelay;
use unified::UnifiedTrackingData;

/// Construct and use sources on the relay thread; OpenXR handles are !Send.
trait FaceSource {
    fn receive(&mut self, data: &mut UnifiedTrackingData) -> anyhow::Result<bool>;
    fn name(&self) -> &'static str;
}
