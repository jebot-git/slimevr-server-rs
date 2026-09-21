#!/usr/bin/python3
"""Build binary/source RPMs from this checkout and locked, vendored dependencies."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import tomllib


ROOT = Path(__file__).resolve().parents[2]


def run(*args, cwd=ROOT, **kwargs):
    return subprocess.run(args, cwd=cwd, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--online", action="store_true", help="allow Cargo to fetch uncached dependencies")
    parser.add_argument("--jobs", type=int, default=min(os.cpu_count() or 2, 8))
    options = parser.parse_args()
    if options.jobs < 1:
        parser.error("--jobs must be positive")
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    name = manifest["package"]["name"]
    version = manifest["package"]["version"]
    cargo_flags = ["--locked"] + ([] if options.online else ["--offline"])
    # Keep Git credentials, local config, build products and personal settings out
    # of the archive. Include untracked source so first-time packaging also works.
    files = run("git", "ls-files", "-z", "--cached", "--others", "--exclude-standard",
                capture_output=True).stdout.decode().split("\0")
    with tempfile.TemporaryDirectory(prefix="shora-rpmbuild-") as temporary:
        top = Path(temporary)
        for subdir in ["BUILD", "BUILDROOT", "SOURCES", "SPECS", "RPMS", "SRPMS"]:
            (top / subdir).mkdir()
        source = top / f"{name}-{version}"
        source.mkdir()
        for relative in files:
            if not relative:
                continue
            src = ROOT / relative
            if not src.is_file():
                continue
            dest = source / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        config = run("cargo", "vendor", *cargo_flags, "--versioned-dirs", str(source / "cargo-vendor"),
                     stdout=subprocess.PIPE, text=True).stdout
        config = config.replace(str(source / "cargo-vendor"), "cargo-vendor")
        (source / ".cargo").mkdir(exist_ok=True)
        (source / ".cargo/config.toml").write_text(config)
        licenses = source / "packaging/rpm/dependency-licenses"
        licenses.mkdir(parents=True)
        inventory = []
        license_texts = {}
        for crate in sorted((source / "cargo-vendor").iterdir()):
            package = tomllib.loads((crate / "Cargo.toml").read_text())["package"]
            inventory.append({key: package.get(key) for key in ["name", "version", "license", "repository"]})
            # Preserve original relative paths for all license/notice texts.
            for path in crate.rglob("*"):
                if path.is_file() and path.name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "COPYRIGHT", "NOTICE")):
                    dest = licenses / crate.name / path.relative_to(crate)
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    digest = hashlib.sha256(path.read_bytes()).hexdigest()
                    if digest in license_texts:
                        dest.symlink_to(os.path.relpath(license_texts[digest], dest.parent))
                    else:
                        shutil.copy2(path, dest)
                        license_texts[digest] = dest
        (licenses / "inventory.json").write_text(json.dumps(inventory, indent=2) + "\n")
        archive = top / "SOURCES" / f"{name}-{version}.tar.gz"
        print(f"Creating {archive.name} with vendored sources", flush=True)
        with tarfile.open(archive, "w:gz") as bundle:
            bundle.add(source, arcname=source.name)
        spec = top / "SPECS" / f"{name}.spec"
        shutil.copy2(source / "packaging/rpm" / spec.name, spec)
        run("rpmbuild", "-ba", "--define", f"_topdir {top}", "--define",
            f"_smp_build_ncpus {options.jobs}", str(spec))
        artifacts = []
        for directory in [top / "RPMS", top / "SRPMS"]:
            for artifact in sorted(directory.rglob("*.rpm")):
                destination = output / artifact.name
                shutil.copy2(artifact, destination)
                artifacts.append(destination)
        checksums = []
        for artifact in artifacts:
            with artifact.open("rb") as stream:
                digest = hashlib.file_digest(stream, "sha256").hexdigest()
            checksums.append(f"{digest}  {artifact.name}\n")
            print(artifact, flush=True)
        (output / "SHA256SUMS").write_text("".join(checksums))


if __name__ == "__main__":
    main()
