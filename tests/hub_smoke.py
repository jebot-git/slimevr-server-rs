#!/usr/bin/env python3
"""Hardware-free Linux smoke tests. Build first; pass --qt to test PySide6 too."""
import argparse
import base64
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import stat
import struct
import subprocess
import tempfile
import termios
import time

ROOT = Path(__file__).resolve().parents[1]


def wait_for(check, timeout=8, pump=lambda: None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        pump()
        result = check()
        if result:
            return result
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for condition")


def request(path, command):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(3)
        client.connect(str(path))
        client.sendall(json.dumps(command if isinstance(command, dict) else {"command": command}).encode() + b"\n")
        with client.makefile("rb") as stream:
            return json.loads(stream.readline())


def status(path):
    try:
        return request(path, "status")["status"]
    except (OSError, ValueError):
        return None


def serial_frame(master):
    data = bytearray(30)
    struct.pack_into("<h", data, 6, -18000)
    struct.pack_into("<h", data, 16, 9000)
    struct.pack_into("<h", data, 22, -15588)
    os.write(master, b"r0:0000300000\nX0:" + base64.b64encode(data) + b"\n")
    os.write(master, b'v0:{"battery voltage":3900,"battery remaining":75}\n')


def config_file(directory, slave):
    path = directory / "config.toml"
    path.write_text(f'''tracker_port = 0
solarxr_socket = "{directory}/rpc"
feeder_socket = "{directory}/input"
control_socket = "{directory}/control"
[haritorax]
enabled = true
ports = ["{slave}"]
model = "x2"
''')
    return path


def check_trackers(snapshot):
    trackers = {t["name"]: t for t in snapshot.get("trackers", [])}
    if len(trackers) != 2 or not all(t["active"] for t in trackers.values()):
        return False
    assert trackers["leftAnkle"]["position"] == 9
    assert trackers["leftKnee"]["position"] == 7
    assert trackers["leftAnkle"]["mac"] == "7E:E2:E4:79:6A:A8"
    assert trackers["leftAnkle"]["battery_percent"] == 75
    assert snapshot["bone_count"] > 0
    return True


def backend_test(binary):
    with tempfile.TemporaryDirectory(prefix="shora-smoke-") as tmp:
        directory = Path(tmp)
        master, slave = pty.openpty()
        config = config_file(directory, os.ttyname(slave))
        control = directory / "control"
        with (directory / "server.log").open("w+") as log:
            process = subprocess.Popen([str(binary), "--config", str(config)], stdout=log, stderr=log)
            try:
                wait_for(lambda: (s := status(control)) and s["ports"] and s["ports"][0]["connected"])
                assert stat.S_IMODE(control.stat().st_mode) == 0o600
                assert not request(control, "yaw_reset")["ok"]
                assert not request(control, "unknown_command")["ok"]
                # Lose the initial identity response, then stream a leg frame
                # before r0 arrives. The server must request identity again;
                # a physical parent-tracker button/reset must not be required.
                while select.select([master], [], [], 0)[0]:
                    os.read(master, 4096)
                serial_frame_bytes = bytearray(30)
                struct.pack_into("<h", serial_frame_bytes, 6, -18000)
                struct.pack_into("<h", serial_frame_bytes, 22, -18000)
                os.write(master, b"X0:" + base64.b64encode(serial_frame_bytes) + b"\n")
                serial_commands = bytearray()
                def identity_requested_again():
                    if select.select([master], [], [], 0)[0]:
                        serial_commands.extend(os.read(master, 4096))
                    return b"r0:" in serial_commands
                wait_for(identity_requested_again)
                serial_frame(master)
                wait_for(lambda: check_trackers(status(control) or {}))
                print("PASS late serial identity recovery discovers knee without parent reset")
                before = status(control)
                assert len(before["bones"]) == before["bone_count"] == 21
                for bone in before["bones"]:
                    assert abs(math.dist(bone["head"], bone["tail"]) - bone["length"]) < 1e-5
                # Exercise the real feeder framing/protobuf, not only the solver.
                with socket.socket(socket.AF_UNIX) as feeder:
                    feeder.connect(str(directory / "input"))
                    def send_feeder(body):
                        feeder.sendall(struct.pack("<I", len(body) + 4) + body)
                    # TrackerAdded: tracker id 0 (default), HMD role 19.
                    send_feeder(b"\x1a\x02\x20\x13")
                    anchored = []
                    anchors = [(1.25, 1.65, -2.5), (-0.75, 1.2, 0.4)]
                    for anchor in anchors:
                        # Position fields 2..8, quaternion x/y/z/w.
                        values = (*anchor, 0.0, 0.0, 0.0, 1.0)
                        body = b"".join(bytes([(field << 3) | 5]) + struct.pack("<f", value)
                                        for field, value in enumerate(values, 2))
                        send_feeder(b"\x0a" + bytes([len(body)]) + body)
                        def at_anchor():
                            snapshot = status(control)
                            if snapshot and snapshot["hmd_pose_received"]:
                                neck = next(b for b in snapshot["bones"] if b["body_part"] == 2)
                                if math.dist(neck["head"], anchor) < 1e-5:
                                    return snapshot
                        anchored.append(wait_for(at_anchor))
                    delta = [b - a for a, b in zip(*anchors)]
                    for first, second in zip(anchored[0]["bones"], anchored[1]["bones"]):
                        for endpoint in ("head", "tail"):
                            expected = [v + d for v, d in zip(first[endpoint], delta)]
                            assert math.dist(second[endpoint], expected) < 1e-5
                print("PASS moving HMD feeder anchors every skeleton joint")
                settings = dict(before["settings"], height_m=2.0, smoothing=0.6, prediction=0.04,
                                drift_correction=True, drift_amount=0.75)
                assert request(control, {"command": "set_settings", "settings": settings})["ok"]
                first_length = before["bones"][0]["length"]
                wait_for(lambda: (s := status(control)) and abs(s["bones"][0]["length"] / first_length - 2.0 / 1.8) < 1e-4)
                invalid = dict(settings, height_m=1.5, smoothing=2.0)
                rejected = request(control, {"command": "set_settings", "settings": invalid})
                assert not rejected["ok"] and rejected["status"]["settings"]["height_m"] == 2.0
                # Binary garbage, oversized lines and invalid IMU must not stop acquisition.
                os.write(master, b"\xff\n" + b"z" * 9000 + b"\nX0:bad!\nX0:AAAA\n")
                serial_frame(master)
                wait_for(lambda: check_trackers(status(control) or {}))
                for command in ["yaw_reset", "full_reset", "mounting_reset"]:
                    assert request(control, command)["ok"]
                calibrated = status(control)
                assert calibrated["calibrated_trackers"] == 2
                assert calibrated["drift_samples"] == 2
                assert request(control, "clear_drift")["status"]["drift_samples"] == 0
                print("PASS authoritative skeleton geometry, live tuning, atomic validation, drift controls")
                assert request(control, "pause_tracking")["status"]["paused"]
                assert not request(control, "pause_tracking")["status"]["paused"]
                os.write(master, b"r0:0000301000\n")
                wait_for(lambda: (status(control) or {}).get("last_action") == "YawReset")
                assert request(control, "restart")["ok"]
                wait_for(lambda: (s := status(control)) and s["ports"][0]["connected"])
                serial_frame(master)
                wait_for(lambda: check_trackers(status(control) or {}))
                os.close(master)
                master = None
                wait_for(lambda: (s := status(control)) and not s["ports"][0]["connected"] and not s["trackers"])
                assert request(control, "shutdown")["ok"]
                assert process.wait(timeout=4) == 0
                assert not control.exists()
                print("PASS serial decode → registry → skeleton, battery, buttons, control, reconnect, disconnect, shutdown")
            except Exception:
                log.seek(0)
                print(log.read())
                raise
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                if master is not None:
                    os.close(master)
                os.close(slave)
        # Refuse to overwrite regular files at the control endpoint.
        control.write_text("keep me")
        result = subprocess.run([str(binary), "--config", str(config)], capture_output=True, timeout=4)
        assert result.returncode != 0 and control.read_text() == "keep me"
        print("PASS existing non-socket control path preserved")
        control.unlink()
        with socket.socket(socket.AF_UNIX) as stale:
            stale.bind(str(control))
        process = subprocess.Popen([str(binary), "--config", str(config)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            wait_for(lambda: (s := status(control)) and s["ports"] and s["ports"][0]["error"])
            duplicate = subprocess.run([str(binary), "--config", str(config)], capture_output=True, timeout=4)
            assert duplicate.returncode != 0
            assert request(control, "status")["status"]["pid"] == process.pid
            assert request(control, "shutdown")["ok"]
            assert process.wait(timeout=4) == 0
            print("PASS stale socket recovery, missing-device diagnostics, active server protection")
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


def tui_test(binary):
    with tempfile.TemporaryDirectory(prefix="shora-tui-") as tmp:
        directory = Path(tmp)
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 32, 120, 0, 0))
        before = termios.tcgetattr(slave)
        control = directory / "control"
        args = [str(binary), "--ui", "tui", "--tracker-port", "0", "--control-socket", str(control),
                "--solarxr-socket", str(directory / "rpc"), "--feeder-socket", str(directory / "input"),
                "--log-file", str(directory / "tui.log")]
        process = subprocess.Popen(args, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        output = bytearray()
        def drain():
            while select.select([master], [], [], 0)[0]:
                output.extend(os.read(master, 65536))
        try:
            wait_for(lambda: b"Shora" in output and status(control), pump=drain)
            os.write(master, b"p")
            wait_for(lambda: (status(control) or {}).get("paused"), pump=drain)
            os.write(master, b"q")
            wait_for(lambda: process.poll() is not None, pump=drain)
            assert process.returncode == 0
            assert termios.tcgetattr(slave) == before
            assert b"\x1b[?1049l" in output
            assert not control.exists()
            print("PASS real TUI render, pause/quit keys, terminal restoration")
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)


def qt_test(binary):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import sys
    sys.path.insert(0, str(ROOT / "frontend"))
    from shora_qt import HubWindow, STYLE
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import QProcess
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    with tempfile.TemporaryDirectory(prefix="shora-qt-") as tmp:
        directory = Path(tmp)
        master, slave = pty.openpty()
        config = config_file(directory, os.ttyname(slave))
        control = directory / "control"
        window = HubWindow(server=str(binary), config=str(config), socket_path=str(control))
        face_sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        face_sink.bind(("127.0.0.1", 0))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as babble, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as feedback:
            babble.bind(("127.0.0.1", 0))
            feedback.bind(("127.0.0.1", 0))
            babble_port = babble.getsockname()[1]
            with config.open("a") as file:
                file.write(f'\n[face]\nosc_query = false\nbabble_port = {babble_port}\nosc_port = {feedback.getsockname()[1]}\ndestination = "127.0.0.1:{face_sink.getsockname()[1]}"\n')
        window.face.setCurrentIndex(2)  # Babble source, selected through Qt.
        window.face_output.setCurrentIndex(4)  # UniFT + VRChat JSON, overrides config.
        window.face_sheet.setText(str(ROOT / "examples/face/avatar-osc.json"))
        window.show()
        wait = lambda check: wait_for(check, pump=app.processEvents)
        try:
            window.start_button.click()
            wait(lambda: window.last_snapshot and window.last_snapshot["ports"] and window.last_snapshot["ports"][0]["connected"])
            assert window.last_snapshot["face_output"] == "both"
            address = b"/jawOpen\0"
            face_sink.sendto(address + b"\0" * (-len(address) % 4) + b",f\0\0" + struct.pack(">f", 0.65), ("127.0.0.1", babble_port))
            wait(lambda: window.last_snapshot and window.last_snapshot["face_active"])
            output = bytearray()
            def both_outputs():
                while select.select([face_sink], [], [], 0)[0]:
                    output.extend(face_sink.recv(65535))
                return b"/avatar/parameters/FT/v2/JawOpen\0" in output and b"/avatar/parameters/CustomMouthOpen\0" in output
            wait(both_outputs)
            assert b"/feedback/mouth\0" not in output
            print("PASS Qt selection → native UniFT + VRChat JSON OSC output")
            serial_frame(master)
            wait(lambda: window.last_snapshot and check_trackers(window.last_snapshot))
            assert window.trackers.rowCount() == 2
            panel = window.tracking
            wait(lambda: len(panel.preview.projected_bones) == 21)
            previous_projection = [(part, a.x(), a.y(), b.x(), b.y()) for part, a, b in panel.preview.projected_bones]
            panel.preview.set_view("Side")
            app.processEvents()
            assert previous_projection != [(part, a.x(), a.y(), b.x(), b.y()) for part, a, b in panel.preview.projected_bones]
            panel.preview.set_view("Orbit")
            panel.height.setValue(185)
            panel.smoothing.setValue(0.15)
            panel.prediction.setValue(35)
            panel.drift_enabled.setChecked(True)
            panel.drift_amount.setValue(70)
            panel.apply_button.click()
            wait(lambda: not panel.applying and abs(window.last_snapshot["settings"]["prediction"] - 0.035) < 1e-5)
            assert window.last_snapshot["settings"]["drift_correction"]
            assert not panel.dirty
            # Incoming snapshots must not erase edits the user has not applied yet.
            panel.smoothing.setValue(0.25)
            panel.set_snapshot(window.last_snapshot)
            assert panel.smoothing.value() == 0.25 and panel.dirty
            panel.reload_button.click()
            assert panel.smoothing.value() == 0.15 and not panel.dirty
            profile = directory / "tracking.toml"
            panel.export_profile(profile)
            import tomllib
            saved = tomllib.loads(profile.read_text())
            assert saved["drift_correction"] and abs(saved["prediction"] - 0.035) < 1e-5
            # Cancelled countdown sends no reset. Zero-delay calibration still waits for server ACK.
            action = window.last_snapshot["last_action"]
            panel.delay.setValue(3)
            panel.calibrate_button.click()
            assert panel.deadline is not None
            panel.cancel_button.click()
            assert panel.deadline is None and request(control, "status")["status"]["last_action"] == action
            panel.delay.setValue(0)
            panel.calibrate_button.click()
            wait(lambda: window.last_snapshot["last_action"] == "FullReset")
            assert window.last_snapshot["calibrated_trackers"] == 2
            panel.clear_drift.click()
            wait(lambda: window.last_snapshot["last_action"] == "ClearDrift")
            assert window.last_snapshot["drift_samples"] == 0
            print("PASS Qt skeleton orbit, tuning/ACK, retained edits, profile export, calibration/cancel, drift clear")
            window.command_buttons["pause_tracking"].click()
            wait(lambda: window.last_snapshot and window.last_snapshot["paused"])
            window.command_buttons["full_reset"].click()
            wait(lambda: window.last_snapshot and window.last_snapshot["last_action"] == "FullReset")
            window.grab().save("/tmp/shora-qt-live.png")
            # A second, attached window must not stop a server it did not launch.
            attached = HubWindow(socket_path=str(control), attach=True)
            attached.show()
            wait(lambda: attached.last_snapshot)
            attached.close()
            app.processEvents()
            assert window.process.state() == QProcess.ProcessState.Running
            assert request(control, "status")["ok"]
            window.stop_button.click()
            wait(lambda: window.process.state() == QProcess.ProcessState.NotRunning)
            assert window.process.exitCode() == 0
            assert not control.exists()
            assert not panel.preview.bones and not panel.apply_button.isEnabled()
            print("PASS Qt launch, live tracker table, pause/reset, attach-only close, graceful stop")
            broken = directory / "broken-server"
            broken.write_text("#!/nonexistent-shora-test-interpreter\n")
            broken.chmod(0o700)
            window.binary.setText(str(broken))
            window.start_button.click()
            wait(lambda: window.start_button.isEnabled())
            assert not window.reconnect
            assert not window.stop_button.isEnabled()
            print("PASS Qt failed-launch recovery")
        finally:
            if window.process.state() != QProcess.ProcessState.NotRunning:
                window.process.kill()
                window.process.waitForFinished(3000)
            window.close()
            os.close(master)
            os.close(slave)
            face_sink.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", type=Path, default=ROOT / "target/debug/slimevr-server-rs")
    parser.add_argument("--qt", action="store_true")
    args = parser.parse_args()
    backend_test(args.server.resolve())
    tui_test(args.server.resolve())
    if args.qt:
        qt_test(args.server.resolve())
