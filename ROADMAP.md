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
- [x] **SolarXR IPC server** (`src/solarxr/`) over a **Unix domain socket**
      (`/run/user/1000/SlimeVRRpc`), speaking the `MessageBundle` FlatBuffers
      protocol framed as a 4-byte little-endian length prefix — the same transport
      WiVRn/SteamVR use (a WebSocket-on-21110 was the original wrong guess).
- [x] **Forward-kinematics solver** (vendored `skeletal_model`): BFS `do_fk` with
      rotation inheritance (`input_rot_g` → else `parent_rot * calib_rot_l`) and
      position propagation (`parent_pos + rot * -Y * length`), plus 3DoF root
      anchoring. Fixed a sibling-collapse bug where all children of a bone shared
      one tail node/edge (`HipL`/`HipR`, `Chest`/`ShoulderL`/`ShoulderR`).
- [x] **Tracker attachment** (`Skeleton::attach_input_tracker`) + bone output
      accessors (`bone_output_rot` / `bone_output_pos`), wired into `skeleton::solve_pose`.
- [x] **Calibration / offsets** (`src/calibration.rs`): per-tracker Java-faithful
      adjustment chain (`mounting_orientation`, `gyro_fix`, `attachment_fix`,
      `mount_rot_fix`, `yaw_fix`) + drift, triggered from tracker user actions
      (`Reset` / `ResetYaw` / `ResetMounting`) via `src/reset.rs`.

## Core fusion (the hard part)

- [x] **Standing (full) + mounting + yaw resets** — ported the Java
      `TrackerResetsHandler` chain per tracker (`mounting_orientation`, `gyro_fix`,
      `attachment_fix`, `mount_rot_fix`, `yaw_fix`).
- [x] **Drift compensation** — yaw drift recorded between resets and ramped back in
      (simplified: latest-drift only, no multi-reset weighted average yet).
- [ ] **Calibration refinements.** Per-arm skip-pose/T-pose modes, HMD
      special-casing, and yaw-reset smoothing are not ported.
- [x] **Frame alignment.** Ported the ktmath `Quaternion.SLIMEVR` mounting
      orientations (`FRONT`/`LEFT`/`RIGHT`/`FRONT_LEFT`/`FRONT_RIGHT`) via
      `default_mounting(position)`, applied as `mounting_orientation` on SENSOR_INFO.
- [~] **Bone lengths / proportions.** Height-based autobone is in
      (`bone_lengths_from_height`, anthropometric ratios at `DEFAULT_HEIGHT_M`),
      plus a per-user autobone (`src/autobone.rs`) that records frames and
      coordinate-descents the vertical/leg lengths to minimise foot-slide + height
      error, triggered via `AutoBoneProcessRequest` RPC. This is a simplified port
      of SlimeVR's `AutoBone` — the full multi-objective error set (slide/offset/
      proportion/position), recording save/load, and status callbacks are still TODO.
- [ ] **Smoothing / prediction.** Port the Java server's filtering and pose smoothing.

## Protocol completeness

- [x] **SolarXR handshake.** Replies to `StartDataFeed` (begin streaming),
      `PollDataFeed` (one-shot `DataFeedUpdate`), `SubscriptionRequest` /
      `TopicHandleRequest` (`TopicMapping`), and `SettingsRequest`
      (`SettingsResponse`) — enough for WiVRn's `create_xdevs` to succeed.
- [x] **Bone feed (`bone_mask`).** Emit `DataFeedUpdate.bones` (`Bone` with
      `rotation_g` + `head_position_g` + `body_part` + `bone_length`).
- [x] **Synthetic trackers (emulated Vive trackers).** Emit `DataFeedUpdate
      .synthetic_trackers` — the 11 computed 6-DoF trackers (head/chest/hip/2
      knees/2 feet/2 elbows/2 hands) WiVRn maps to emulated Vive trackers. Their
      `tracker_id` must omit `device_id` (WiVRn filters trackers with a device id).
- [x] **SteamVR feeder bridge** (`src/feeder.rs`) — the `/run/user/1000/SlimeVRInput`
      Unix socket where WiVRn sends the HMD pose (protobuf `ProtobufMessage`,
      `TrackerAdded` + `Position`), used to anchor the synthetic trackers.
- [~] **RPC surface.** Reset RPC is handled over SolarXR (`ResetRequest` →
      full/yaw/mounting reset + `ResetResponse`). Assignment, status, settings,
      serial, and autobone RPCs are still TODO.
- [~] **Tracker management.** SENSOR_INFO sensor-id/position handling, manual
      per-MAC assignment, and disconnect/timeout cleanup (configurable
      `tracker_timeout_secs`) are in; pings go out on the tracker socket so
      trackers answer with `Pong` and stay alive. Multi-sensor trackers and
      SENSOR_INFO status handling are still TODO.

## Ops & ergonomics

- [~] Config file + CLI. TOML config file + `--tracker-port`/`--solarxr-socket`/
      `--feeder-socket`/`--ping-interval-secs`/`--tracker-timeout-secs`/`--height-m`/
      `--assign MAC=POSITION` flags (precedence: defaults → file → CLI).
- [x] Tracker body-part assignment (auto + manual). Auto reads SENSOR_INFO
      `tracker_position`; manual overrides it per-MAC via config/`--assign`,
      matching the Java server's `vrconfig.yml` assignment.
- [ ] Skeleton/pose logging or a lightweight debug UI.
- [~] Parity testing against the Java server — `examples/solarxr_dump` +
      `examples/tracker_emulate` built. **End-to-end live test succeeded**: the Rust
      server swapped in for Java, 6 live HaritoraX trackers connect, and WiVRn
      connects to both the SolarXR socket (`Enumerated 11 SolarXR synthetic
      trackers`) and the feeder socket (`feeder HMD tracker added`). Fixed along the
      way: ktmath `(w,x,y,z)` frame-alignment, a 10s ping interval exceeding
      shora's 5s tracker timeout, skeleton structure matching Java
      (`UpperChest`/`ShoulderL/R`/`HipL/R`), the HaritoraX 2 ankle→shin mapping, the
      FK sibling-collapse bug, and the WebSocket→Unix-socket transport. `HEAD` and
      finger bones are intentionally omitted (HMD-derived / WiVRn's hand tracking).
      Also fixed in shora: an `XrState` drop-order segfault on WiVRn restart.
- [ ] Remove the vendored `solarxr_protocol` in favour of the upstream git dep once
      the version is pinned.
