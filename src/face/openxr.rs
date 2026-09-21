//! OpenXR headless face/eye source, ported from oscavmgr's
//! `src/core/ext_tracking/openxr.rs`.
//!
//! Opens a headless OpenXR session on the WiVRn/Monado runtime and reads the
//! **Meta/Facebook** tracking path: `XR_FB_face_tracking2` (face) and
//! `XR_EXT_eye_gaze_interaction` (eye gaze, as exposed by Meta Quest Pro via
//! WiVRn/Monado).

use std::time::{Duration, Instant};

use glam::{EulerRot, Quat, Vec3};
use mint::Quaternion;
use openxr as xr;
use strum::EnumCount;

use super::unified::{UnifiedExpressions, UnifiedTrackingData};
use super::{face2_fb, FaceSource};

pub struct OpenXrSource {
    state: Option<XrState>,
    last_attempt: Instant,
}

impl OpenXrSource {
    pub fn new() -> Self {
        Self {
            state: None,
            last_attempt: Instant::now() - Duration::from_secs(20),
        }
    }
}

impl Default for OpenXrSource {
    fn default() -> Self {
        Self::new()
    }
}

impl FaceSource for OpenXrSource {
    fn name(&self) -> &'static str {
        "openxr"
    }

    fn receive(&mut self, data: &mut UnifiedTrackingData) -> anyhow::Result<bool> {
        data.eye_active = false;
        let Some(state) = self.state.as_mut() else {
            if self.last_attempt + Duration::from_secs(15) < Instant::now() {
                self.state = XrState::new()
                    .map_err(|e| tracing::error!("XR: {e:#}"))
                    .ok();
                self.last_attempt = Instant::now();
            }
            return Ok(false);
        };

        match state.receive(data) {
            Ok(valid) => Ok(valid),
            Err(e) => {
                tracing::error!("XR: {e:#}");
                self.state = None;
                Ok(false)
            }
        }
    }
}

struct XrState {
    // Declaration order = drop order (Rust drops fields top-to-bottom). OpenXR
    // requires destroying session resources before the session, and the session
    // before the instance, so `face_tracker_fb` must come first and `instance`
    // last — otherwise destroying the face tracker after the session/instance is
    // gone segfaults in the runtime (use-after-free).
    face_tracker_fb: Option<MyFaceTrackerFB>,
    events: xr::EventDataBuffer,
    actions: xr::ActionSet,
    eye_space: xr::Space,
    view_space: xr::Space,
    session: xr::Session<xr::Headless>,
    session_running: bool,
    eyes_closed_frames: u32,
    system: xr::SystemId,
    instance: xr::Instance,
}

impl XrState {
    fn new() -> anyhow::Result<Self> {
        let (instance, system) = xr_init()?;

        let actions = instance.create_action_set("slimevr_face", "SlimeVR Face", 0)?;
        let eye_action = actions.create_action("eye_gaze", "Eye Gaze", &[])?;

        let (session, _frame_waiter, _frame_stream) =
            unsafe { instance.create_session(system, &xr::headless::SessionCreateInfo {})? };

        instance.suggest_interaction_profile_bindings(
            instance.string_to_path("/interaction_profiles/ext/eye_gaze_interaction")?,
            &[xr::Binding::new(
                &eye_action,
                instance.string_to_path("/user/eyes_ext/input/gaze_ext/pose")?,
            )],
        )?;

        session.attach_action_sets(&[&actions])?;

        let view_space =
            session.create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)?;
        let eye_space =
            eye_action.create_space(session.clone(), xr::Path::NULL, xr::Posef::IDENTITY)?;

        let mut me = Self {
            instance,
            system,
            session,
            view_space,
            eye_space,
            actions,
            events: xr::EventDataBuffer::new(),
            session_running: false,
            face_tracker_fb: None,
            eyes_closed_frames: 0,
        };

        me.face_tracker_fb = MyFaceTrackerFB::new(&me)
            .map_err(|e| tracing::info!("FB_face_tracking2: {e}"))
            .ok();

        Ok(me)
    }

    fn load_properties<T>(&self, next: *mut T) -> xr::Result<()> {
        unsafe {
            let mut p = xr::sys::SystemProperties {
                ty: xr::sys::SystemProperties::TYPE,
                next: next as _,
                ..std::mem::zeroed()
            };
            let res = (self.instance.fp().get_system_properties)(
                self.instance.as_raw(),
                self.system,
                &mut p,
            );
            if res != xr::sys::Result::SUCCESS {
                return Err(res);
            }
            Ok(())
        }
    }

    fn receive(&mut self, data: &mut UnifiedTrackingData) -> anyhow::Result<bool> {
        while let Some(event) = self.instance.poll_event(&mut self.events)? {
            use xr::Event::*;
            match event {
                SessionStateChanged(e) => match e.state() {
                    xr::SessionState::READY => {
                        self.session
                            .begin(xr::ViewConfigurationType::PRIMARY_STEREO)?;
                        self.session_running = true;
                        tracing::info!("XrSession started.");
                    }
                    xr::SessionState::STOPPING => {
                        self.session.end()?;
                        self.session_running = false;
                        tracing::warn!("XrSession stopped.");
                    }
                    xr::SessionState::EXITING | xr::SessionState::LOSS_PENDING => {
                        anyhow::bail!("XR session exiting");
                    }
                    _ => {}
                },
                InstanceLossPending(_) => anyhow::bail!("XR instance loss pending"),
                EventsLost(e) => {
                    tracing::warn!("lost {} events", e.lost_event_count());
                }
                _ => {}
            }
        }

        if !self.session_running {
            return Ok(false);
        }

        let next_frame = xr::Time::from_nanos(
            self.instance.now()?.as_nanos() + (0.03334f32 * 1_000_000_000f32) as i64,
        );

        self.session.sync_actions(&[(&self.actions).into()])?;

        let mut valid = false;

        // Eye gaze: derive eye-closed + gaze euler angles.
        let eye_loc = self.eye_space.locate(&self.view_space, next_frame)?;
        if eye_loc.location_flags.contains(
            xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::ORIENTATION_TRACKED,
        ) {
            valid = true;
            let now_q = to_quat(eye_loc.pose.orientation);
            let (y, x, z) = now_q.to_euler(EulerRot::YXZ);

            let mut eye_closed = ((x.to_degrees() + 5.0) / -55.0).max(0.0);

            if let Some(last) = data.eyes[0] {
                let last_q = Quat::from_euler(EulerRot::YXZ, last.y, last.x, last.z);
                if last_q.angle_between(now_q).to_degrees() > 10.0 {
                    self.eyes_closed_frames = 5;
                }
            }

            if self.eyes_closed_frames > 0 {
                self.eyes_closed_frames -= 1;
                eye_closed = 1.0;
            }

            data.setu(UnifiedExpressions::EyeClosedLeft, eye_closed);
            data.setu(UnifiedExpressions::EyeClosedRight, eye_closed);

            data.eyes[0] = Some(Vec3::new(x, y, z));
            data.eyes[1] = data.eyes[0];
            data.eye_active = true;
        }

        if let Some(face_tracker) = self.face_tracker_fb.as_ref() {
            let mut weights = [0f32; 70];
            let mut confidences = [0f32; 2];
            if face_tracker.get_face_expression_weights(next_frame, &mut weights, &mut confidences)? {
                valid = true;
                if let Some(shapes) = face2_fb::face2_fb_to_unified(&weights) {
                    data.eye_active = true;
                    data.shapes[..UnifiedExpressions::COUNT]
                        .copy_from_slice(&shapes[..UnifiedExpressions::COUNT]);
                }
            }
        }

        Ok(valid)
    }
}

fn xr_init() -> anyhow::Result<(xr::Instance, xr::SystemId)> {
    let entry = xr::Entry::linked();
    let available_extensions = entry
        .enumerate_extensions()
        .map_err(|_| anyhow::anyhow!("Failed to enumerate OpenXR extensions."))?;

    anyhow::ensure!(
        available_extensions.mnd_headless,
        "Missing MND_headless extension."
    );

    let mut enabled_extensions = xr::ExtensionSet::default();
    enabled_extensions.mnd_headless = true;
    enabled_extensions.khr_convert_timespec_time = true;

    if available_extensions.ext_eye_gaze_interaction {
        enabled_extensions.ext_eye_gaze_interaction = true;
    } else {
        tracing::warn!("Missing EXT_eye_gaze_interaction extension. Is Monado/WiVRn up to date?");
    }
    if available_extensions.fb_face_tracking2 {
        enabled_extensions.fb_face_tracking2 = true;
    }
    if available_extensions.htc_facial_tracking {
        enabled_extensions.htc_facial_tracking = true;
    }

    let instance = entry
        .create_instance(
            &xr::ApplicationInfo {
                api_version: xr::Version::new(1, 0, 0),
                application_name: "slimevr-server-rs",
                application_version: 0,
                engine_name: "slimevr-server-rs",
                engine_version: 0,
            },
            &enabled_extensions,
            &[],
        )
        .map_err(|_| anyhow::anyhow!("Failed to create OpenXR instance."))?;

    let instance_props = instance
        .properties()
        .map_err(|_| anyhow::anyhow!("Failed to query OpenXR instance properties."))?;
    tracing::info!(
        "Using OpenXR runtime: {} {}",
        instance_props.runtime_name,
        instance_props.runtime_version
    );

    let system = instance
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .map_err(|_| anyhow::anyhow!("Failed to access OpenXR HMD system."))?;

    Ok((instance, system))
}

fn to_quat(p: xr::Quaternionf) -> Quat {
    let q: Quaternion<f32> = p.into();
    q.into()
}

struct MyFaceTrackerFB {
    api: xr::raw::FaceTracking2FB,
    tracker: xr::sys::FaceTracker2FB,
}

impl MyFaceTrackerFB {
    fn new(xr_state: &XrState) -> anyhow::Result<Self> {
        if xr_state.instance.exts().fb_face_tracking2.is_none() {
            anyhow::bail!("Extension not supported.");
        }

        let mut props = xr::sys::SystemFaceTrackingProperties2FB {
            ty: xr::StructureType::SYSTEM_FACE_TRACKING_PROPERTIES2_FB,
            next: std::ptr::null_mut(),
            supports_visual_face_tracking: xr::sys::Bool32::from_raw(0),
            supports_audio_face_tracking: xr::sys::Bool32::from_raw(0),
        };
        xr_state.load_properties(&mut props)?;
        if props.supports_visual_face_tracking.into_raw() == 0 {
            anyhow::bail!("Unable to provide visual data.");
        }

        let api = unsafe {
            xr::raw::FaceTracking2FB::load(
                xr_state.session.instance().entry(),
                xr_state.session.instance().as_raw(),
            )?
        };

        let mut data_source = xr::sys::FaceTrackingDataSource2FB::VISUAL;
        let info = xr::sys::FaceTrackerCreateInfo2FB {
            ty: xr::StructureType::FACE_TRACKER_CREATE_INFO2_FB,
            next: std::ptr::null(),
            face_expression_set: xr::FaceExpressionSet2FB::DEFAULT,
            requested_data_source_count: 1,
            requested_data_sources: &mut data_source,
        };

        let mut tracker = xr::sys::FaceTracker2FB::default();
        let res =
            unsafe { (api.create_face_tracker2)(xr_state.session.as_raw(), &info, &mut tracker) };
        if res.into_raw() != 0 {
            anyhow::bail!("Could not initialize: {:?}", res);
        }

        tracing::info!("Using FB_face_tracking2 for face.");
        Ok(Self { api, tracker })
    }

    fn get_face_expression_weights(
        &self,
        time: xr::Time,
        weights: &mut [f32],
        confidences: &mut [f32],
    ) -> anyhow::Result<bool> {
        let mut expressions = xr::sys::FaceExpressionWeights2FB {
            ty: xr::StructureType::FACE_EXPRESSION_WEIGHTS2_FB,
            next: std::ptr::null_mut(),
            weight_count: weights.len() as _,
            weights: weights.as_mut_ptr(),
            confidence_count: confidences.len() as _,
            confidences: confidences.as_mut_ptr(),
            is_eye_following_blendshapes_valid: xr::sys::Bool32::from_raw(0),
            is_valid: xr::sys::Bool32::from_raw(0),
            data_source: xr::sys::FaceTrackingDataSource2FB::VISUAL,
            time,
        };

        let info = xr::sys::FaceExpressionInfo2FB {
            ty: xr::StructureType::FACE_EXPRESSION_INFO2_FB,
            next: std::ptr::null(),
            time,
        };

        let res = unsafe {
            (self.api.get_face_expression_weights2)(self.tracker, &info, &mut expressions)
        };
        if res.into_raw() != 0 {
            anyhow::bail!("Failed to get expression weights");
        }
        Ok(expressions.is_valid.into_raw() != 0)
    }
}

impl Drop for MyFaceTrackerFB {
    fn drop(&mut self) {
        unsafe {
            (self.api.destroy_face_tracker2)(self.tracker);
        }
    }
}
