#!/usr/bin/env python3
"""Container-side build, validation and package assembly (see build-linux.py)."""
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import tomllib

ROOT = Path("/source")
BUILD = Path("/build")
OUT = Path("/output")
VERSION = tomllib.loads((ROOT / "Cargo.toml").read_text())["package"]["version"]
JOBS = os.environ.get("BUILD_JOBS", "8")
os.environ["CARGO_TARGET_DIR"] = str(BUILD / "target")
os.environ["QT_QPA_PLATFORM"] = "offscreen"


def run(*args, **kwargs):
    return subprocess.run(args, check=True, cwd=ROOT, **kwargs)


def install(source, destination, executable=False):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    destination.chmod(0o755 if executable else 0o644)


def fresh(path):
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)
    return path


def license_bundle(destination):
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("LICENSE-MIT", "LICENSE-APACHE", "THIRD_PARTY.md"):
        install(ROOT / name, destination / name)
    shutil.copytree(ROOT / "licenses", destination / "project", dirs_exist_ok=True)
    inventory = []
    for crate in sorted((ROOT / "cargo-vendor").iterdir()):
        package = tomllib.loads((crate / "Cargo.toml").read_text())["package"]
        inventory.append({key: package.get(key) for key in ("name", "version", "license", "repository")})
        for path in crate.rglob("*"):
            if path.is_file() and path.name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "COPYRIGHT", "NOTICE")):
                install(path, destination / "rust" / crate.name / path.relative_to(crate))
    (destination / "rust-inventory.json").write_text(json.dumps(inventory, indent=2) + "\n")
    for dist in importlib.metadata.distributions():
        for path in dist.files or []:
            if any(part.lower().startswith(("license", "copying", "notice")) for part in path.parts):
                original = Path(dist.locate_file(path))
                if original.is_file():
                    install(original, destination / "python" / dist.metadata["Name"] / str(path))
    install("/usr/share/doc/python3.12/copyright", destination / "python/copyright")
    # Shared libraries are dynamically linked and remain replaceable in the
    # extracted AppDir / installed Qt bundle. Retain distro copyright notices.
    for path in Path("/usr/share/doc").glob("*/copyright"):
        install(path, destination / "system" / path.parent.name / "copyright")


def deb(root, name, description, depends):
    control = root / "DEBIAN"
    control.mkdir()
    size = sum(p.stat().st_size for p in root.rglob("*") if p.is_file()) // 1024
    (control / "control").write_text(f"""Package: {name}
Version: {VERSION}-1
Architecture: amd64
Maintainer: jebot-git <326713999+jebot-git@users.noreply.github.com>
Section: utils
Priority: optional
Installed-Size: {size}
Depends: {depends}
Homepage: https://github.com/jebot-git/slimevr-server-rs
Description: {description}
 Native SlimeVR-compatible full-body, HaritoraX and face tracking for WiVRn.
""")
    artifact = OUT / f"{name}_{VERSION}-1_amd64.deb"
    run("dpkg-deb", "--root-owner-group", "--build", str(root), str(artifact))
    run("dpkg-deb", "--info", str(artifact))


def main():
    run("cargo", "build", "--frozen", "--release", "--features", "face-xr", "--bins", "-j", JOBS)
    run("cargo", "test", "--frozen", "--release", "--workspace", "--features", "face-xr", "-j", JOBS)
    binary = BUILD / "target/release/slimevr-server-rs"
    proxy = BUILD / "target/release/shora-proxy"
    run("python3", "tests/hub_smoke.py", "--server", str(binary), "--qt")
    run("pyinstaller", "--noconfirm", "--clean", "--name", "shora-qt", "--onedir",
        "--distpath", str(BUILD / "frozen"), "--workpath", str(BUILD / "pyinstaller"),
        "--specpath", str(BUILD), "--paths", str(ROOT / "frontend"),
        "--add-binary", f"{binary}:bin", "--add-binary", f"{proxy}:bin",
        str(ROOT / "frontend/shora_qt.py"))
    frozen = BUILD / "frozen/shora-qt"
    # Qt's XCB plugin loads this library at runtime rather than through DT_NEEDED.
    # Include it explicitly so an otherwise minimal desktop can launch the GUI.
    for name in ("libxcb-cursor.so.0", "libxcb-util.so.1", "libxcb-image.so.0", "libxcb-render-util.so.0"):
        install(Path("/usr/lib/x86_64-linux-gnu") / name, frozen / "_internal" / name)
    license_bundle(frozen / "licenses")
    run(str(frozen / "shora-qt"), "--help")

    server = fresh(BUILD / "deb-server")
    for path in (binary, proxy):
        install(path, server / "usr/bin" / path.name, True)
        run("strip", "--strip-unneeded", str(server / "usr/bin" / path.name))
    shared = server / "usr/share/slimevr-server-rs"
    for name in ("config.example.toml", "README.md", "THIRD_PARTY.md"):
        install(ROOT / name, shared / name)
    install(ROOT / "packaging/rpm/service-default.toml", shared / "service-default.toml")
    install(ROOT / "packaging/rpm/init-profile", server / "usr/libexec/slimevr-server-rs/init-profile", True)
    install(ROOT / "examples/systemd/wait-sockets.py", server / "usr/libexec/slimevr-server-rs/wait-sockets.py", True)
    install(ROOT / "packaging/rpm/shora-server.service", server / "usr/lib/systemd/user/shora-server.service")
    shutil.copytree(frozen / "licenses", server / "usr/share/doc/slimevr-server-rs/licenses")
    deb(server, "slimevr-server-rs", "Shora native tracking server and socket proxy",
        "libc6 (>= 2.39), libgcc-s1, libstdc++6, libudev1, libopenxr-loader1, python3")

    qt = fresh(BUILD / "deb-qt")
    shutil.copytree(frozen, qt / "usr/lib/shora-qt", symlinks=True)
    launcher = qt / "usr/bin/shora-qt"
    launcher.parent.mkdir(parents=True)
    launcher.write_text('#!/bin/sh\nexport SLIMEVR_SERVER_BIN=/usr/bin/slimevr-server-rs\nexec /usr/lib/shora-qt/shora-qt "$@"\n')
    launcher.chmod(0o755)
    desktop = (ROOT / "packaging/rpm/shora-qt.desktop").read_text().replace(
        "Icon=preferences-desktop-peripherals", "Icon=shora")
    entry = qt / "usr/share/applications/shora.desktop"
    entry.parent.mkdir(parents=True)
    entry.write_text(desktop)
    run("desktop-file-validate", str(entry))
    install(ROOT / "packaging/linux/shora.svg", qt / "usr/share/icons/hicolor/scalable/apps/shora.svg")
    deb(qt, "slimevr-server-rs-qt", "Shora Qt6 skeleton preview and calibration frontend",
        f"slimevr-server-rs (= {VERSION}-1), libc6 (>= 2.39), libgl1, libegl1, libfontconfig1, libdbus-1-3, libxkbcommon0, libx11-6")

    wivrn = fresh(BUILD / "deb-wivrn")
    install(ROOT / "packaging/rpm/shora-proxy.service", wivrn / "usr/lib/systemd/user/shora-proxy.service")
    install(ROOT / "examples/systemd/wivrn.service.d/shora.conf", wivrn / "usr/lib/systemd/user/wivrn.service.d/shora.conf")
    deb(wivrn, "slimevr-server-rs-wivrn", "Shora WiVRn user-service integration",
        f"slimevr-server-rs (= {VERSION}-1), wivrn")

    appdir = fresh(BUILD / "Shora.AppDir")
    shutil.copytree(frozen, appdir / "usr/lib/shora-qt", symlinks=True)
    install(ROOT / "packaging/linux/AppRun", appdir / "AppRun", True)
    install(ROOT / "packaging/linux/shora.svg", appdir / "shora.svg")
    (appdir / ".DirIcon").symlink_to("shora.svg")
    (appdir / "shora.desktop").write_text(desktop.replace("Exec=shora-qt --attach", "Exec=AppRun"))
    run("desktop-file-validate", str(appdir / "shora.desktop"))
    image = OUT / f"Shora-{VERSION}-x86_64.AppImage"
    run("/opt/appimagetool/AppRun", "--no-appstream", "--runtime-file", "/opt/runtime-x86_64",
        str(appdir), str(image), env={**os.environ, "ARCH": "x86_64"})
    image.chmod(0o755)
    run(str(image), "--appimage-extract-and-run", "--server", "--version")
    run(str(image), "--appimage-extract-and-run", "--proxy", "--help")
    run(str(image), "--appimage-extract-and-run", "--help")
    # Exercise the bundled GUI's real startup after Qt imports and plugin loading.
    with (BUILD / "appimage-gui.log").open("w+") as log:
        process = subprocess.Popen([str(image), "--appimage-extract-and-run"], stdout=log, stderr=log)
        try:
            time.sleep(3)
            if process.poll() is not None:
                log.seek(0)
                raise RuntimeError(f"AppImage GUI exited at startup: {log.read()}")
        finally:
            process.terminate()
            process.wait(timeout=10)
    print("PASS packaged AppImage server, proxy, Qt CLI and offscreen GUI startup", flush=True)


if __name__ == "__main__":
    main()
