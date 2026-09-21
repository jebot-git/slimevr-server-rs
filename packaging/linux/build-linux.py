#!/usr/bin/env python3
"""Build Ubuntu 24.04 amd64 DEBs and an x86_64 AppImage in rootless Podman."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
IMAGE = "localhost/shora-packaging:ubuntu24.04"


def run(*args, **kwargs):
    return subprocess.run(args, check=True, cwd=ROOT, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--jobs", type=int, default=min(os.cpu_count() or 2, 8))
    parser.add_argument("--online", action="store_true", help="fetch uncached Cargo dependencies")
    options = parser.parse_args()
    if options.jobs < 1:
        parser.error("--jobs must be positive")
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    work = ROOT / "target/linux-packaging"
    source = work / "source"
    if source.exists():
        shutil.rmtree(source)
    source.mkdir(parents=True)
    files = run("git", "ls-files", "-z", "--cached", "--others", "--exclude-standard", capture_output=True).stdout.decode().split("\0")
    for relative in files:
        if relative and (ROOT / relative).is_file():
            dest = source / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, dest)
    config = run("cargo", "vendor", "--locked", *([] if options.online else ["--offline"]),
                 "--versioned-dirs", str(source / "cargo-vendor"), capture_output=True, text=True).stdout
    (source / ".cargo").mkdir(exist_ok=True)
    (source / ".cargo/config.toml").write_text(config.replace(str(source / "cargo-vendor"), "cargo-vendor"))
    run("podman", "build", "-t", IMAGE, "-f", str(ROOT / "packaging/linux/Containerfile"), str(ROOT / "packaging/linux"))
    build = work / "build"
    build.mkdir(exist_ok=True)
    run("podman", "run", "--rm", "--network=none", "--security-opt=label=disable",
        "-v", f"{source}:/source:ro", "-v", f"{build}:/build", "-v", f"{output}:/output",
        "-e", f"BUILD_JOBS={options.jobs}", IMAGE, "python3", "packaging/linux/package.py")
    artifacts = sorted(p for p in output.iterdir() if p.suffix in (".rpm", ".deb", ".AppImage"))
    (output / "SHA256SUMS").write_text("".join(
        f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}\n" for p in artifacts))


if __name__ == "__main__":
    main()
