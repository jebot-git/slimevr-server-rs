#!/usr/bin/env python3
"""Regression coverage for switching existing checkout units to package units."""
from pathlib import Path
import runpy
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = runpy.run_path(str(ROOT / "packaging/systemd/shora-use-packaged-services"))


class PackageServices(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.local = Path(self.temp.name) / "local"
        self.vendor = Path(self.temp.name) / "vendor"
        for base in (self.local, self.vendor):
            for name in HELPER["OVERRIDES"]:
                path = base / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f"{base.name}: {name}\n")
        self.calls = []
        self.state = "active"
        self.fail_start = False

    def ctl(self, *args):
        self.calls.append(args)
        if args[0] == "show" and "ActiveState" in args:
            return self.state
        if args[0] == "start" and self.fail_start:
            self.fail_start = False
            raise subprocess.CalledProcessError(1, args)
        return ""

    def migrate(self):
        HELPER["migrate"](self.local, self.vendor, self.ctl)

    def test_migrate_and_repeat(self):
        profile = self.local / "profile.toml"
        profile.write_text("height_m = 1.78\n")
        custom = self.local / "shora-server.service.d/custom.conf"
        custom.parent.mkdir()
        custom.write_text("[Service]\nEnvironment=CUSTOM=1\n")
        self.migrate()
        backups = list(self.local.glob("shora-package-backup-*"))
        self.assertEqual(len(backups), 1)
        for name in HELPER["OVERRIDES"]:
            self.assertFalse((self.local / name).exists())
            self.assertEqual((backups[0] / name).read_text(), f"local: {name}\n")
        self.assertEqual(profile.read_text(), "height_m = 1.78\n")
        self.assertTrue(custom.is_file())
        self.assertIn(("stop", *HELPER["UNITS"]), self.calls)
        self.assertIn(("start", *HELPER["UNITS"]), self.calls)
        self.calls.clear()
        self.migrate()
        self.assertEqual(self.calls, [("daemon-reload",)])

    def test_inactive_services_remain_stopped(self):
        self.state = "inactive"
        self.migrate()
        self.assertFalse(any(call[0] in ("stop", "start") for call in self.calls))

    def test_missing_package_leaves_local_units_untouched(self):
        (self.vendor / "shora-proxy.service").unlink()
        with self.assertRaisesRegex(RuntimeError, "install the server and WiVRn packages"):
            self.migrate()
        self.assertEqual(self.calls, [])
        self.assertTrue((self.local / "shora-proxy.service").is_file())

    def test_mask_is_preserved(self):
        path = self.local / "shora-proxy.service"
        path.unlink()
        path.symlink_to("/dev/null")
        with self.assertRaisesRegex(RuntimeError, "masked"):
            self.migrate()
        self.assertEqual(self.calls, [])
        self.assertTrue(path.is_symlink())

    def test_symlink_is_backed_up_without_changing_target(self):
        target = Path(self.temp.name) / "checkout.service"
        target.write_text("checkout unit\n")
        path = self.local / "shora-proxy.service"
        path.unlink()
        path.symlink_to(target)
        self.migrate()
        backup = next(self.local.glob("shora-package-backup-*")) / path.name
        self.assertTrue(backup.is_symlink())
        self.assertEqual(target.read_text(), "checkout unit\n")

    def test_failed_start_restores_local_units(self):
        self.fail_start = True
        with self.assertRaises(subprocess.CalledProcessError):
            self.migrate()
        for name in HELPER["OVERRIDES"]:
            self.assertEqual((self.local / name).read_text(), f"local: {name}\n")
        self.assertEqual(sum(call[0] == "start" for call in self.calls), 2)

    def test_examples_match_installed_units(self):
        for name in ("shora-server.service", "shora-proxy.service"):
            packaged = (ROOT / "packaging/rpm" / name).read_text()
            self.assertEqual((ROOT / "examples/systemd" / name).read_text(), packaged)
            self.assertNotIn("target/release", packaged)
            self.assertNotIn("%h/shora", packaged)


if __name__ == "__main__":
    unittest.main()
