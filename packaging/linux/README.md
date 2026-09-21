# DEB and AppImage packaging

These builds target **x86_64, Ubuntu 24.04 or newer (glibc 2.39+)**. Other
distributions with compatible libraries may work but are not installation-tested.
The backend includes OpenXR face tracking. The Qt frontend bundles Python 3.12,
PySide6 Essentials 6.8.3 and their shared libraries; no Python setup is required.

Build with Podman and Cargo installed on the host:

```bash
python3 packaging/linux/build-linux.py --output dist/v0.1.1
```

The first container build needs internet access for Ubuntu packages, Rust 1.88,
Python wheels and the checksum-pinned AppImage tooling. Cargo sources come from
the host's locked cache; use `--online` if dependencies are missing. Compilation,
Rust tests, simulated serial/TUI/Qt tests and package assembly then run without
network access. Build products are cached under `target/linux-packaging/`.
`--jobs N` limits Cargo parallelism. `SHA256SUMS` covers all packages in the
selected output directory; use a fresh directory for each release.

## DEB

```bash
sudo apt install ./slimevr-server-rs_0.1.1-1_amd64.deb \
  ./slimevr-server-rs-qt_0.1.1-1_amd64.deb
systemctl --user daemon-reload
systemctl --user start shora-server.service
shora-qt --attach
```

For automatic WiVRn startup/proxy integration, optionally install
`slimevr-server-rs-wivrn_0.1.1-1_amd64.deb`. It depends on a native `wivrn`
package and installs the same user units as the RPM integration package.
Follow the service setup and migration instructions in
[`../rpm/README.md`](../rpm/README.md). Installing packages does not start or
restart services. Existing profiles are preserved. Run `shora-qt` without
`--attach` to launch a standalone backend from the window instead.

## AppImage

```bash
chmod +x Shora-0.1.1-x86_64.AppImage
./Shora-0.1.1-x86_64.AppImage
# Attach to an existing server instead:
./Shora-0.1.1-x86_64.AppImage --attach
# Access the bundled native programs:
./Shora-0.1.1-x86_64.AppImage --server --help
./Shora-0.1.1-x86_64.AppImage --proxy --help
```

On systems without working FUSE, add `--appimage-extract-and-run` as the first
argument. The AppImage does not install systemd units or change serial-device
permissions. A working desktop/OpenGL driver and a host OpenXR runtime are still
needed for those features. Select HaritoraX and the desired face source before
starting the server from the window.

Third-party license texts and inventories are included inside the Qt bundle's
`licenses/` directory. The bundled Qt libraries are dynamically linked and can
be replaced in an extracted AppDir (`--appimage-extract`). Upstream source:
[Qt/PySide 6.8.3](https://download.qt.io/official_releases/QtForPython/pyside6/PySide6-6.8.3-src/),
[Qt 6.8.3](https://download.qt.io/archive/qt/6.8/6.8.3/single/),
[CPython 3.12](https://www.python.org/downloads/source/).
AppImage format/tooling follows the
[official packaging guide](https://docs.appimage.org/packaging-guide/manual.html).

Packages are unsigned; validate downloads with `sha256sum -c SHA256SUMS`.
