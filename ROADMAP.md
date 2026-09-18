# Roadmap

The Java SlimeVR server is roughly 12k lines of battle-tested Kotlin. This document
tracks what the Rust rewrite has and what remains. Items are ordered roughly by the
dependency chain: nothing above an item can be finished without the items below it.

## Done (first milestone)

- [x] Project scaffold with SlimeVR-Rust git deps (`firmware_protocol`, `skeletal_model`, `vqf`).
- [x] Tracker UDP protocol server (`tracker/udp.rs`): handshake → `"Hey OVR =D 5"`,
      ping, rotation, acceleration, sensor-info (raw `tracker_position` parse).
- [x] `TrackerRegistry` keyed by MAC.
- [x] `skeletal_model::Skeleton` graph construction (zero bone lengths).
- [x] Passthrough pose estimation: `TrackerPosition` → SolarXR `BodyPart` + rotation.
- [x] SolarXR WebSocket server scaffold: accept, verify incoming `MessageBundle`,
      stream a `DataFeedUpdate` with one `TrackerData` per tracked bone.

## Core fusion (the hard part)

- [ ] **Forward-kinematics solver.** `skeletal_model`'s `Skeleton::solve`/`do_fk` is
      `todo!()` upstream. Implement the FK (pop node → solve edge → solve node) so a
      partial tracker set yields a complete, constrained skeleton. This is *the*
      central task.
- [ ] **Bone lengths / proportions.** Feed real bone lengths (defaults + autobone /
      user height) into `SkeletonConfig`.
- [ ] **Tracker attachment.** Port `Skeleton::attach_input_tracker` (currently
      commented out upstream): map a tracker's body position onto its bone and store
      the calibrated local offset.
- [ ] **Calibration.** Mounting reset (skip pose) → compute tracker→bone offsets;
      full reset (standing) → compute global yaw/heading.
- [ ] **Drift compensation.** Use `vqf` (or a port of the Java complementary filter)
      for mag/accel yaw-drift correction on each tracker's rotation.
- [ ] **Smoothing / prediction.** Port the Java server's filtering and pose smoothing.

## Protocol completeness

- [ ] **SolarXR handshake.** WiVRn sends a `StartDataFeed` RPC + expects a topic
      mapping and an RPC reply before it consumes updates. Reference: Java
      `websocketapi/WebSocketVRBridge.kt` + `protocol/`.
- [ ] **Bone feed (`bone_mask`).** WiVRn subscribes with `bone_mask: true`; emit
      `DataFeedUpdate.bones` (`Bone` + `BodyPart`) instead of synthetic trackers.
- [ ] **RPC surface.** Reset/calibration, tracker assignment, status, settings,
      serial, autobone, etc. — the `rpc/` and `pub_sub/` message families.
- [ ] **Tracker management.** SENSOR_INFO sensor-id/status handling, multi-sensor
      trackers, disconnect/timeout cleanup.

## Ops & ergonomics

- [ ] Config file + CLI (ports, bone lengths, tracker assignments).
- [ ] Tracker body-part assignment (auto + manual), matching the Java server's setup.
- [ ] Skeleton/pose logging or a lightweight debug UI.
- [ ] Parity testing against the Java server (feed identical UDP, diff SolarXR output).
- [ ] Remove the vendored `solarxr_protocol` in favour of the upstream git dep once
      the version is pinned.
