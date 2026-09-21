# RPM packaging

The build produces a native package, a Qt frontend package, a WiVRn integration
package, a source RPM, and SHA256 checksums. Initial target: Nobara/Fedora 44,
x86_64. Packages built on this host require its library versions; rebuild the
source RPM for other distribution releases. Packages are not RPM-signed.

```bash
sudo dnf install rpm-build rust cargo gcc systemd-devel openxr-devel \
  python3-pyside6 desktop-file-utils systemd-rpm-macros
python3 packaging/rpm/build-rpm.py
```

The script uses Cargo's existing cache by default. Add `--online` to fetch missing
locked dependencies. `--jobs N` controls build parallelism and `--output PATH`
selects the artifact directory (default `dist/`). The source archive contains the
project and vendored dependencies, including pinned Git sources. It excludes
Git internals and ignored files. RPM compilation and tests run with Cargo's
`--frozen` mode and a fresh Cargo home, without network access. The spec runs the
Rust tests, real backend/proxy restart tests, and simulated serial/TUI/Qt smoke
tests. It needs permission to bind local sockets and create pseudo terminals.

```bash
sudo dnf install ./dist/slimevr-server-rs-0.1.1-2.fc44.x86_64.rpm \
  ./dist/slimevr-server-rs-qt-0.1.1-2.fc44.noarch.rpm \
  ./dist/slimevr-server-rs-wivrn-0.1.1-2.fc44.noarch.rpm
```

The main package installs `slimevr-server-rs`, `shora-proxy`, and
`shora-server.service`. The Qt package adds `shora-qt` and a desktop entry that
attaches to the server. Run `shora-qt` without `--attach` to launch a standalone
server instead. The WiVRn package adds `shora-proxy.service` and the
`wivrn.service.d/shora.conf` dependency drop-in. Omit that subpackage when you do
not want WiVRn integration. Installation does not restart an active VR session.

On first backend service startup, a private `~/.config/shora-rust/server.toml`
is created with HaritoraX 2 and WiVRn OpenXR face tracking enabled. Existing
profiles are preserved. Edit body height and device/face settings there; the
service's working directory is the profile directory, so relative avatar JSON
paths resolve there. Use the Qt tuning export to save later session changes.

For a first package deployment, stop any older tracking process holding the
public sockets, then:

```bash
systemctl --user daemon-reload
systemctl --user enable wivrn.service
systemctl --user restart wivrn.service
shora-qt --attach
```

WiVRn pulls in the backend and proxy automatically. Later,
`systemctl --user restart shora-server.service` leaves WiVRn and the proxy alive.
Calibration is session-local and must be repeated after a backend restart.

If migrating from the checkout-based units, user files in
`~/.config/systemd/user/` take precedence over the packaged files in
`/usr/lib/systemd/user/`. After installing/upgrading the server and WiVRn
packages, run as your desktop user, **without sudo**:

```bash
shora-use-packaged-services
```

This backs up the local `shora-server.service`, `shora-proxy.service`, and
`wivrn.service.d/shora.conf` under a private `shora-package-backup-*` directory
in your systemd user configuration, removes those overrides, and reloads the
manager. Active backend/proxy/WiVRn services are stopped and started again;
this interrupts WiVRn and resets calibration. Inactive services remain stopped.
Existing `~/.config/shora-rust/server.toml` and other custom drop-ins are
preserved. Unit masks are refused. If activation fails, the old unit files are
restored. Review any preserved custom drop-ins if they override executable
paths; the command prints the effective unit paths and commands after migration.
`XDG_CONFIG_HOME` is respected. Package installation itself never edits user
files or restarts active sessions.

The 0.1.1-2 package revision includes this migration command and corrects the
example service files to use the installed package paths. Existing 0.1.1-1 users
can upgrade normally and then run the command once.

To rebuild from the source RPM on a compatible host with the build dependencies:

```bash
rpmbuild --rebuild slimevr-server-rs-0.1.1-2.fc44.src.rpm
sha256sum -c dist/SHA256SUMS
```
