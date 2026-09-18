# Roadmap

The Java SlimeVR server is roughly 12k lines of battle-tested Kotlin. This document
tracks what the Rust rewrite has and what remains. Items are ordered roughly by the
dependency chain: nothing above an item can be finished without the items below it.

## Done

- [x] Project scaffold with SlimeVR-Rust git deps (`firmware_protocol`, `skeletal_model`, `vqf`).
- [x] Tracker UDP protocol server (`tracker/udp.rs`): handshake → `"Hey OVR =D 5"`,
      ping, rotation, acceleration, sensor-info (raw `tracker_position` parse).
- [x] `TrackerRegistry` keyed by MAC.
- [x] `skeletal_model::Skeleton` graph construction (zero bone lengths).
- [x] Passthrough pose estimation: `TrackerPosition` → SolarXR `BodyPart` + rotation.
- [x] SolarXR WebSocket server scaffold: accept, verify incoming `MessageBundle`,
      stream a `DataFeedUpdate` with one `TrackerData` per tracked bone.
- [x] **Forward-kinematics solver** (vendored `skeletal_model`): BFS `do_fk` with
      rotation inheritance (`input_rot_g` → else `parent_rot * calib_rot_l`) and
      position propagation (`parent_pos + rot * -Y * length`), plus 3DoF root
      anchoring at the origin.
- [x] **Tracker attachment** (`Skeleton::attach_input_tracker`) + bone output
      accessors (`bone_output_rot` / `bone_output_pos`), wired into `skeleton::solve_pose`.
- [x] **Calibration / offsets** (`src/calibration.rs`): per-tracker mounting offset
      (`offset = raw⁻¹ * bone_calib`) and a global full-reset heading correction
      (`heading = inverse_yaw(reference)`), applied as `heading * raw * offset`.
      Triggered from tracker user actions (`Reset` / `ResetYaw` / `ResetMounting`)
      via `src/reset.rs`.

## Core fusion (the hard part)

- [ ] **Calibration refinements.** The mounting offset currently assumes the standing
      pose (`BoneKind::calibration_rotation`) and no per-arm skip-pose/T-pose modes,
      and full reset only isolates yaw. Mounting reset in the skip pose and arm-mode
      handling remain.
- [ ] **Frame alignment.** The mounting offset already absorbs the tracker→bone frame
      difference (both share gravity "up"), but the SlimeVR sensor↔skeletal_model
      sign conventions have not been explicitly verified end-to-end.
- [ ] **Bone lengths / proportions.** Real user proportions (autobone / height).
      Approximate adult defaults are in place.
- [ ] **Drift compensation.** Use `vqf` (or a port of the Java complementary filter)
      for mag/accel yaw-drift correction on each tracker's rotation.
- [ ] **Smoothing / prediction.** Port the Java server's filtering and pose smoothing.

## Protocol completeness

- [x] **SolarXR handshake.** Reply to `StartDataFeed` (begin streaming) and to
      `SubscriptionRequest`/`TopicHandleRequest` with a `TopicMapping`.
- [x] **Bone feed (`bone_mask`).** Emit `DataFeedUpdate.bones` (`Bone` with
      `rotation_g` + `head_position_g` + `body_part` + `bone_length`) instead of
      synthetic trackers.
- [ ] **RPC surface.** Reset/calibration, tracker assignment, status, settings,
      serial, autobone, etc. — the `rpc/` and remaining `pub_sub/` message families.
- [ ] **Tracker management.** SENSOR_INFO sensor-id/status handling, multi-sensor
      trackers, disconnect/timeout cleanup.

## Ops & ergonomics

- [ ] Config file + CLI (ports, bone lengths, tracker assignments).
- [ ] Tracker body-part assignment (auto + manual), matching the Java server's setup.
- [ ] Skeleton/pose logging or a lightweight debug UI.
- [ ] Parity testing against the Java server (feed identical UDP, diff SolarXR output).
- [ ] Remove the vendored `solarxr_protocol` in favour of the upstream git dep once
      the version is pinned.
