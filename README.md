# slimevr-server-rs

A from-scratch **Rust rewrite of the [SlimeVR](https://github.com/SlimeVR/SlimeVR-Server) full-body-tracking server**, using the [SlimeVR-Rust](https://github.com/SlimeVR/SlimeVR-Rust) workspace as boilerplate.

This is a **separate development** from [shora](https://github.com/jebot-git/shora). Shora currently *launches* the Java/Kotlin SlimeVR server as a headless subprocess; the goal of this project is to eventually replace that subprocess with a native Rust binary.

> **Status: early but progressing.** The tracker input path and a first
> forward-kinematics solver are working; the WiVRn-facing SolarXR output is
> scaffolded. See [`ROADMAP.md`](ROADMAP.md).

## What SlimeVR-Rust provides (and what we build on top)

| SlimeVR-Rust crate | What it gives us | Used here? |
|---|---|---|
| `firmware_protocol` | The "Hey OVR =D 5" tracker UDP protocol (`Packet`, `SbPacket`, `CbPacket`) | ✅ tracker input |
| `skeletal_model` | The human-skeleton graph (`Skeleton`, `BoneKind`) + FK solver | ✅ vendored; solver completed here |
| `vqf` | The VQF IMU orientation filter | 🔜 future drift/filter work |
| `solarxr` | A SolarXR **client** (not the server we need) | ❌ (we write the server) |

`skeletal_model` is vendored under `vendor/skeletal_model` because its upstream FK
solver (`Skeleton::solve` / `do_fk`) is still `todo!()`. We completed it in-place
(BFS forward kinematics with per-edge rotation inheritance and per-node position
propagation) and added `attach_input_tracker` + output accessors.

The SolarXR *server* half (what WiVRn connects to) is implemented here from scratch on top of the vendored `solarxr_protocol` FlatBuffers bindings.

## Architecture

```
SlimeVR trackers ── UDP (v13, "Hey OVR =D 5", :6969) ──► tracker/udp.rs ──► TrackerRegistry
                                                                                │
                                                                                ▼
                                                        skeleton::estimate_pose  (passthrough → FK later)
                                                                                │
                                                                                ▼
WiVRn ◄── WebSocket SolarXR (:21110) ── solarxr/mod.rs ◄── Pose (BodyPart → rotation)
```

| Module | Role |
|---|---|
| `tracker/udp.rs` | Tracker UDP protocol server: handshake response, ping/pong, rotation/accel/sensor-info |
| `tracker/mod.rs` | `TrackerRegistry` — connected trackers keyed by MAC |
| `calibration.rs` | Per-tracker mounting offsets + full-reset heading (`heading * raw * offset`) |
| `reset.rs` | Tracker user actions (`Reset`/`ResetYaw`/`ResetMounting`) → calibration updates |
| `skeleton/mod.rs` | Pose estimation: adjusted tracker rotations → `skeletal_model` FK solve → bone pose |
| `solarxr/mod.rs` | SolarXR WebSocket server + `DataFeedUpdate` encoder |
| `main.rs` | Orchestration: spawn servers, run the pose loop |

## Building & running

```bash
cargo build            # or `cargo build --release`
cargo run              # listens on UDP 6969 (trackers) + TCP 21110 (SolarXR)
```

Stop any other SlimeVR server first — this binds the same `6969`/`21110` ports and is intended to *replace* it, not run alongside it.

## License

MIT OR Apache-2.0. Vendored `vendor/solarxr-protocol` is MIT/Apache-2.0 (SlimeVR SolarXR-Protocol).
