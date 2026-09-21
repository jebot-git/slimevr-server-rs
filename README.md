# slimevr-server-rs

An experimental Rust rewrite of the [SlimeVR server](https://github.com/SlimeVR/SlimeVR-Server),
using the SlimeVR-Rust firmware protocol and a completed, vendored skeletal model.
It reuses the adjacent Shora application's HaritoraX interpreter, tracker IDs,
terminal UI, and face pipeline in one native server. An optional Qt6 frontend
can launch this server or attach to it. The original Shora application remains
a separate project.

The prototype receives tracker rotations over UDP, applies calibration and
filtering, solves the skeleton, and serves bones and synthetic trackers to WiVRn
through SolarXR IPC. It also accepts the HMD pose through the SteamVR feeder
socket. It is **not yet a feature-complete replacement**; see [ROADMAP.md](ROADMAP.md).

## Data flow

```text
SlimeVR trackers → UDP :6969 → per-sensor registry → filtering / calibration → FK
HaritoraX → GX6/GX2 serial ────────────┘                                      │
                                                                            ↓
WiVRn ← SolarXR Unix socket (SlimeVRRpc) ← bones + 11 synthetic trackers
  └──→ feeder Unix socket (SlimeVRInput) → HMD pose for skeleton anchoring
```

SolarXR uses length-prefixed FlatBuffers over a Unix socket. This implementation
does not expose the Java server's WebSocket endpoint on TCP 21110.

## Build and run

RPM packages for the server, Qt6 frontend, and automatic WiVRn integration can be
built with `python3 packaging/rpm/build-rpm.py`. See the
[RPM build and installation guide](packaging/rpm/README.md) for dependencies,
offline source rebuilds, and migration from checkout-based user services.

A Rust toolchain and a Unix platform are required (tested on Linux). Serial-port
discovery needs libudev development files and pkg-config on Linux, for example
`libudev-dev pkg-config` on Debian/Ubuntu. The first build fetches the
pinned SlimeVR-Rust dependencies; subsequent builds can use `--offline` if cached.

```bash
cargo build --release
cargo run --release -- --help
cargo run --release -- --config config.example.toml
```

Stop another server using the same endpoints before starting this one. Defaults:

- Tracker input: UDP `0.0.0.0:6969`.
- SolarXR: `$XDG_RUNTIME_DIR/SlimeVRRpc`.
- HMD feeder: `$XDG_RUNTIME_DIR/SlimeVRInput`.

When `XDG_RUNTIME_DIR` is unset, socket paths default to `/run/user/1000/`.
Override them with `--solarxr-socket PATH` and `--feeder-socket PATH` as needed.
Configuration precedence is defaults → TOML file → CLI.

## Keep WiVRn attached across server restarts

The optional **`shora-proxy`** process owns the two public sockets and reconnects
to the tracking backend when it restarts. WiVRn keeps its original connections:

```text
WiVRn ← SlimeVRRpc   ← shora-proxy ← SlimeVRRpc.backend   ← Rust server
WiVRn → SlimeVRInput → shora-proxy → SlimeVRInput.backend → Rust server
Qt    ↔ SlimeVRControl                                  ↔ Rust server
```

Build both binaries and start the proxy in its own terminal or user service:

```bash
cargo build --release --features face-xr --bins
target/release/shora-proxy
```

In another terminal, start the backend with private socket paths. Keep your
usual config/face/serial options; this example matches HaritoraX 2 with WiVRn face
tracking:

```bash
target/release/slimevr-server-rs --haritorax --haritorax-model x2 \
  --face-source openxr \
  --solarxr-socket "$XDG_RUNTIME_DIR/SlimeVRRpc.backend" \
  --feeder-socket "$XDG_RUNTIME_DIR/SlimeVRInput.backend" \
  --control-socket "$XDG_RUNTIME_DIR/SlimeVRControl"
# Optional Qt window, in another terminal:
python3 frontend/shora_qt.py --attach
```

For initial adoption, stop the backend that owns the public sockets, start the
proxy and the backend configured as above, then reconnect WiVRn's runtime once.
An established socket cannot be transferred to this proxy on the fly. After
adoption, restart **only the backend**, leaving the proxy and WiVRn running.
Start the backend before connecting the headset: WiVRn's initial device discovery
has a timeout even though the proxy can accept clients before a backend exists.
Use Qt's attach mode for a service-managed server. If Qt launches the server,
give it a TOML config with the two `.backend` socket paths instead of the public
defaults.

The [proxy user unit](examples/systemd/shora-proxy.service),
[backend user unit](examples/systemd/shora-server.service), and
[WiVRn drop-in](examples/systemd/wivrn.service.d/shora.conf) integrate this with
`wivrn.service`. WiVRn pulls in both services and waits for their socket readiness
checks. The proxy uses `BindsTo=wivrn.service` and `PartOf=wivrn.service` to follow
WiVRn's lifetime and explicit restarts. The backend remains independent: restarting
it leaves WiVRn and the proxy alive; stopping WiVRn leaves the backend available
to Qt and preserves its calibration. A proxy failure also stops WiVRn through its
`Requires=` dependency, because losing the public sockets requires a new runtime
connection.

The units assume this checkout is at `~/shora/slimevr-server-rs` and release
binaries are built with `face-xr`. Adjust paths if necessary. The backend reads
`~/.config/shora-rust/server.toml`; create a profile from `config.example.toml`
and set the desired tuning, HaritoraX, and face options before first startup.
Socket paths are set by the unit's command line. Install the units with:

```bash
install -Dm644 examples/systemd/shora-proxy.service \
  "$HOME/.config/systemd/user/shora-proxy.service"
install -Dm644 examples/systemd/shora-server.service \
  "$HOME/.config/systemd/user/shora-server.service"
install -Dm644 examples/systemd/wivrn.service.d/shora.conf \
  "$HOME/.config/systemd/user/wivrn.service.d/shora.conf"
systemctl --user daemon-reload
systemctl --user enable wivrn.service
# Stop the previous backend using the public sockets before this first switch.
systemctl --user restart wivrn.service
```

The proxy/backend units do not need separate enablement: WiVRn starts them on
demand, including on login when WiVRn is enabled. For subsequent backend updates:

```bash
systemctl --user restart shora-server.service
journalctl --user -u shora-proxy.service -u shora-server.service -f
```

Do not bind the proxy to `shora-server.service`; it must outlive backend restarts.
Proxy restart/failure itself still breaks WiVRn's connections. `--help` lists
socket overrides; `--runtime-dir` changes all default paths. Logs report client
connections, backend loss, and successful reattachment.

The proxy restores WiVRn's SolarXR setup/subscription messages and feeder device
announcements, then resumes live traffic. During an outage it coalesces feeder
poses and only replays the latest pose if it is less than one second old. It
does not manufacture body poses or queue reset/calibration actions. This means a
brief tracking gap during restart; how that gap appears depends on the client.
Protocol replay targets this native backend and WiVRn's setup sequence. Only
entire setup-only SolarXR bundles are cached; mixed action/setup bundles are not
replayed. This is not a general session-recovery proxy for the Java server.

The proxy does **not** persist mounting/standing calibration, tuning changes, or
learned drift. Load tuning from TOML and perform standing/mounting resets after
restarting the backend. OpenXR face tracking uses its existing retry path and
does not pass through these sockets.

## Native HaritoraX / SlimeTora integration

The server embeds Shora's Rust port of the
[HaritoraX interpreter](https://github.com/JovannMC/haritorax-interpreter).
It feeds interpreted samples directly into the same calibration and FK pipeline
as UDP trackers, retaining [SlimeTora](https://github.com/OCSYT/SlimeTora)'s stable
MAC generation and body assignments. No Java, Node, or separate SlimeTora process
is required for this path.

```bash
# Auto-detect all GX6/GX2 USB serial interfaces; HaritoraX 2 is the default.
cargo run --release -- --haritorax --ui tui

# Explicit ports also enable acquisition. Repeat for each dongle interface.
cargo run --release -- --serial-port /dev/ttyACM0 --serial-port /dev/ttyACM1 \
  --haritorax-model wireless --ui tui
```

This supports HaritoraX **2 and Wireless through GX6/GX2 serial dongles** at
500000 baud. HaritoraX 2 leg frames produce independent shin and thigh samples;
the "ankle" tracker maps to the lower leg, with feet supplied by expansion
trackers. It also interprets battery reports and main-button click sequences:
one click = yaw reset, two = full reset, three = mounting reset, four or more =
toggle body-pose pause, after a 500 ms grouping interval.

Wrist extensions drive the hand bones using SlimeVR's
[hand assignment IDs](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/TrackerPosition.kt).
Head position/orientation remains anchored by the HMD feeder.

HaritoraX interpreter output and SlimeVR UDP quaternions share the IMU wire
coordinate frame. Both input paths apply the Java server's `AXES_OFFSET`
(-90° around X, multiplied on the left) before calibration, following
[TrackersUDPServer](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/TrackersUDPServer.kt).
HMD feeder poses already use VR coordinates and are not converted.

Serial ports reconnect after failure; **Reconnect serial** / TUI `r` reopens
configured ports or rescans USB devices when `ports = []`. The UI shows failures
such as missing devices or insufficient serial access. Use the normal serial
device permissions for your distribution. Stop Shora/SlimeTora before opening the
same dongles here. Running both paths for the same trackers produces duplicate
identities. Manual `--assign MAC=POSITION` overrides apply to serial trackers too.
Startup identity requests are repeated after the dongle wakes; a channel sending
IMU data without an identity report is queried again on subsequent heartbeats.
HaritoraX 2 knee extensions are decoded from the parent's dual-sensor frame and
do not require a separate button press or reset to register.

BLE acquisition, legacy wired models, pairing/channel configuration, firmware
updates, and tracker power-off are not implemented. This is an integrated subset,
not full SlimeTora parity. Reused code and notices are in [THIRD_PARTY.md](THIRD_PARTY.md).

## Shora terminal and Qt6 interfaces

`--ui tui` shows tracker IDs/assignments, sample ages, battery levels, serial port
errors, face-source activity, and HMD status. It uses the existing Shora ratatui
interface expanded for the native services. Keys: `y` yaw reset, `f` full reset,
`m` mounting reset, `p` pause/resume body pose, `r` reconnect/rescan, `q` quit.
Pause freezes body output while input collection continues; face OSC continues.
Logs go to `$XDG_RUNTIME_DIR/shora-rust.log` (or the system temporary directory),
with `--log-file PATH` available to override it. Headless mode remains the default.

The Qt6 frontend uses native **PySide6 Widgets** and Qt local sockets:

```bash
cargo build --release
python3 -m venv .venv
.venv/bin/pip install -r frontend/requirements.txt
.venv/bin/python frontend/shora_qt.py
```

Choose a config file, serial model/ports and face source, then **Start server**.
The window shows live trackers and logs and exposes the same calibration/pause/
reconnect controls. Closing a window that launched the server stops its child;
closing an attached window leaves the external server running.

The **Skeleton & calibration** tab renders the server's solved bone endpoints,
including tracked versus inferred bones. Drag to orbit, scroll to zoom, or choose
Front/Side/Orbit. The view uses the same solved pose as SolarXR, follows the HMD
when available, and marks a relative/reference pose when no HMD or tracker samples
are available. Pause freezes the preview and outgoing body pose together.

Calibration offers full standing, yaw, and mounting resets with a cancellable
0–10 second countdown. The immediate reset buttons remain available above the
tabs. The panel reports the reference source and number of calibrated trackers;
disconnecting or closing the window cancels any pending countdown.
For mounting calibration, first perform a full reset standing upright, then bend
the knees and lean the torso forward in a ski pose with parallel feet. Mounting
reset identifies the bend axes; its tilted pose is not used to learn yaw drift.

**Tracking settings** apply live to the running server:

| Setting | Meaning |
| --- | --- |
| Body height | 50–300 cm; rebuilds height-based bone proportions, replacing any current optimized lengths. |
| Smoothing blend | 0 disables smoothing; smaller nonzero values smooth more, while 1 follows the latest sample. |
| Prediction | 0–200 ms of angular-velocity extrapolation; 0 disables prediction. |
| Drift correction | Enable/disable yaw-drift learning and correction for current and future trackers. |
| Drift amount | 0–100% strength of the learned yaw correction. |

**Apply settings** validates the entire update before changing server state.
Editing fields keeps them stable while live status arrives; **Reload** restores
the current server values. Changing filter settings clears the previous filter
history. Drift learning needs successive resets facing the same reference
direction, separated by time for drift to accumulate. **Clear learned drift**
clears only drift history, preserving mounting and reset calibration. Toggling
drift correction also starts fresh learning; changing its amount preserves it.
The current compensator uses the latest measured interval rather than SlimeVR's
full multi-sample drift estimator.

Changes are session-local. **Export tuning TOML…** saves the active server values
as a config profile, or values to copy into an existing config; it excludes
device/OSC setup and runtime calibration offsets. Prediction is stored in seconds
and height in metres. The matching TOML fields are `height_m`, `smoothing`,
`prediction`, `drift_correction`, and `drift_amount`.

To run the TUI and attach Qt to the same server:

```bash
target/release/slimevr-server-rs --haritorax --ui tui \
  --control-socket "${XDG_RUNTIME_DIR:-/run/user/1000}/SlimeVRControl"
# In another terminal:
.venv/bin/python frontend/shora_qt.py --attach
```

Qt's `--server PATH`, `--config PATH`, and `--socket PATH` customize startup.
OpenXR face selection needs a binary built with `--features face-xr`. The frontend
does not automatically build the Rust server or install the OpenXR runtime.

The optional control endpoint is an owner-only (`0600`) Unix socket. Send one
JSON object per line, e.g. `{"command":"status"}`. Commands are `status`,
`yaw_reset`, `full_reset`, `mounting_reset`, `pause_tracking`, `restart` (serial
reconnect/rescan), and `shutdown`. Replies contain `ok`, `error`, and `status`.
Calibration without tracker samples returns an error. This API is separate from
the existing SolarXR and feeder sockets; it is not a SlimeVR RPC replacement.
Status also includes `bones` (body part, head/tail positions, quaternion, length,
tracked flag), `settings`, `calibrated_trackers`, and `drift_samples`.
`clear_drift` clears learned drift. Send a complete validated settings object to
change tuning atomically, for example:

```json
{"command":"set_settings","settings":{"height_m":1.8,"smoothing":0.3,"prediction":0.025,"drift_correction":true,"drift_amount":0.5}}
```

## Multi-sensor devices

Each sensor is identified by **device MAC + sensor ID**. Primary and extension
sensors have independent body assignments, rotations, acceleration, calibration,
smoothing, and prediction. A device reconnect updates the address for all its
sensors; heartbeats refresh all sensors and pings are sent once per device.

Assignments use `MAC[/SENSOR]=POSITION`, where omitting `/SENSOR` selects sensor 0.
For example, assign a primary sensor to the left lower leg and its extension to
the right lower leg:

```bash
cargo run --release -- \
  --assign AA:BB:CC:DD:EE:FF=9 \
  --assign AA:BB:CC:DD:EE:FF/1=10
```

In TOML, use a separate `[[tracker_assignments]]` entry for each sensor with an
optional `sensor_id` (default `0`); see [config.example.toml](config.example.toml).
Manual assignments override the position reported in `SENSOR_INFO`.

The server accepts legacy sensor-info packets without a position, acknowledges
valid announcements, and clears a sensor's samples when it reports disconnected
or error status. It accepts that sensor's samples again after an OK announcement.
The parser follows the upstream [sensor-info fields and status values](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPPacket.kt)
and [six-byte acknowledgement format](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPProtocolParser.kt).

## Embedded oscavmgr face/eye tracking

The server includes the face/eye pipeline adapted from Shora's
[oscavmgr](https://github.com/galister/oscavmgr) port. It runs in the same process
as body tracking and is disabled by default. No separate oscavmgr or Shora process
is needed for this pipeline.

For Project Babble and/or EyeTrackVR:

```bash
cargo run --release -- --face-source babble
# With customized ports/settings from the example file:
cargo run --release -- --config config.example.toml --face-source babble
```

Point Babble and EyeTrackVR output at **127.0.0.1:9400**. The server accepts both
OSC messages and nested bundles and emits mapped face, eyelid, and gaze parameters
to **127.0.0.1:9000** by default. VRChat feedback / optional VSync arrives on
**9002**. Enable OSC in the receiver and use an avatar with compatible face
parameters. Source data expires after one second without recognized input;
the relay clears the stale pose and sends tracking-active flags as false.

For Meta face/eye tracking through a WiVRn/Monado headless OpenXR runtime:

```bash
cargo run --release --features face-xr -- --face-source openxr
```

This optional build needs the system OpenXR loader library and a runtime exposing
`XR_MND_headless`, `XR_FB_face_tracking2`, and `XR_EXT_eye_gaze_interaction`.
The source retries initialization when the runtime is unavailable. Selecting
`openxr` in a build without `face-xr` reports a configuration error.

Settings live in the `[face]` section of [config.example.toml](config.example.toml).
`--face-source` enables the selected source; `--no-face` overrides file settings
and disables it. `--face-destination IP:PORT` overrides the OSC destination.
`expose = true` allows LAN input and leaves the destination unchanged.
Stop any other relay occupying the chosen face input ports first.

### UniFT and VRChat JSON output

Face output can use **UniFT (VRCFT Unified Expressions)**, a **VRChat avatar OSC
JSON translation sheet**, or both at the same time. These are OSC output modes;
the JSON file supplies routing information, not a JSON stream sent over UDP.

```bash
# Fixed UniFT output, independent of OSCQuery discovery:
cargo run --release -- --face-source babble --face-output unift

# Translate to the input addresses/types in a VRChat avatar OSC JSON file:
cargo run --release -- --face-source babble --face-output json \
  --face-translation-sheet /path/to/avtr_example.json

# Send both canonical UniFT channels and the JSON-mapped channels:
cargo run --release -- --face-source babble --face-output both \
  --face-translation-sheet /path/to/avtr_example.json
```

The same options work with `--face-source openxr` in a `face-xr` build. Qt exposes
**Face output** and a JSON file picker; the TUI and Qt status show the selected
output mode. TOML equivalents are `face.output` and `face.translation_sheet`.
Relative mapping-file paths resolve from the server's working directory.

| Mode | Behavior |
| --- | --- |
| `auto` (default) | Use an explicit translation sheet or OSCQuery tree, otherwise discover the avatar and keep oscavmgr's small `FT/v2/*` fallback until discovery. |
| `unift` | Emit every unified and combined expression supported by this port at `/avatar/parameters/FT/v2/<Name>`, plus tracking-active flags. |
| `json` | Emit recognized face parameters only at their configured VRChat input addresses and types. Requires a translation sheet. |
| `both` | Emit UniFT and JSON routes together. JSON takes precedence if an address overlaps. Requires a translation sheet. |

For JSON, the source expression comes from `parameters[].name`, including nested
names such as `FT/v2/JawOpen`. The target comes from `input.address`, used exactly
as written, and `input.type` (`Float`, `Bool`, `Int`). An `output` member describes
feedback **from VRChat** and is ignored. Output-only entries and unrelated avatar
controls are skipped. Multiple parameters can receive the same expression.
This follows [VRChat's avatar OSC config format](https://docs.vrchat.com/docs/osc-avatar-parameters).

```json
{
  "id": "avtr_example",
  "name": "My avatar",
  "parameters": [{
    "name": "FT/v2/JawOpen",
    "input": { "address": "/avatar/parameters/CustomMouthOpen", "type": "Float" }
  }]
}
```

A fuller [example translation sheet](examples/face/avatar-osc.json) includes
eyelids, remapped activity flags, and signed binary jaw channels. `Negative` and
power-of-two suffixes (`1` through `64`) encode sign/magnitude in separate
channels; their bit resolution is computed separately for each parameter prefix.
Plain Bool routes use a `> 0.5` threshold; Int routes truncate the expression
value (they do not automatically scale it to 0–255). Invalid types, addresses,
duplicate targets, or sheets with no supported face inputs fail startup with an
error. Unrecognized expression names are skipped; legacy SRanipal aliases are
not implemented.

UniFT covers this port's supported expression set, rather than every extension
in newer VRCFT releases. Source hardware determines which shapes have data.
`EyeLeftY` and `EyeRightY` currently share the interpreter's `EyeY` value.
The parameter convention is documented by
[VRCFaceTracking](https://docs.vrcft.io/docs/tutorial-avatars/tutorial-avatars-extras/parameters).
Both output paths resend their values on `/avatar/change` and clear mapped
tracking-active flags and face values when input expires or the relay shuts down.

OSCQuery advertises **SlimeVR-Rust-Face** on localhost. In `auto` mode it discovers
VRChat's IPv4 OSCQuery service to load an avatar mapping. A static `face.avatar`
path still accepts the OSCQuery **/avatar tree** containing `CONTENTS.parameters`;
use `face.translation_sheet` for VRChat's on-disk avatar JSON instead. These two
file settings are mutually exclusive. Explicit files and `unift`/`json`/`both`
modes bypass discovery; `osc_query = false` also disables advertisement.

The relay supports float, sign, and bit-encoded shape channels and the VSync
watchdog. It does not include oscavmgr's autopilot, Gogo Loco, external storage,
ALVR source, or non-Meta OpenXR face providers. Source attribution and the upstream
MIT notice are in [THIRD_PARTY.md](THIRD_PARTY.md).

Ctrl+C stops the body services and joins the face worker, releasing its sockets.
Port conflicts and worker failures are reported instead of leaving a silent,
partially running server.

## Validation and remaining work

```bash
cargo test --workspace
cargo test --workspace --features face-xr # requires the OpenXR loader
cargo build
python3 tests/hub_smoke.py                # simulated serial + control + real TUI
python3 tests/hub_smoke.py --qt           # additionally needs PySide6
```

Tests cover the FK solver, calibration, filtering, configuration, IPC message
encoding, sensor isolation and reconnects. A loopback UDP test checks the
handshake, repeated sensor announcements, acknowledgements, independent rotations
reaching two skeleton bones, and extension disconnects. It requires permission
to bind local UDP sockets. Face-relay loopback tests cover Babble/EyeTrackVR
bundles, UniFT/JSON/combined OSC output, avatar changes, stale data, port conflicts,
and shutdown; mapping and config
have regression tests as well. The hardware-free hub smoke test uses a pseudo
terminal to check dual leg decoding, battery/button reports, calibration,
reconnect, disconnect, shutdown, TUI keys/terminal restoration and optionally the
Qt launch/attach lifecycle. It binds temporary local sockets and an ephemeral UDP
port. OpenXR builds are checked, but physical dongles, headset sampling,
VRChat discovery, and real avatar behavior still require hardware/runtime testing.

`cargo test --test proxy_restart` starts the real backend and proxy in a temporary
directory, kills/restarts the backend three times, and checks that the same two
simulated WiVRn connections recover current HMD-anchored bones without another
client handshake. Proxy unit tests cover frame fragmentation, replay bounds,
pose expiry, and exclusion of reset actions. These tests do not restart the
installed WiVRn service or access physical trackers.

Advanced leg correction, arm calibration modes, complete RPC/settings support,
calibration persistence, and full parity with the Java server remain unfinished.
Drift compensation and autobone are simplified implementations. Shora is not
automatically switched to this binary.

## License

MIT OR Apache-2.0. Vendored SolarXR protocol bindings retain their upstream license.
