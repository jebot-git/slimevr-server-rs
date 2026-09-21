#!/usr/bin/env python3
"""Native Qt6 frontend for slimevr-server-rs (PySide6).

The Rust process owns tracking. This frontend launches it or attaches to its
owner-only JSON control socket; closing an attached view leaves the server alone.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import shutil
import sys

from PySide6.QtCore import QObject, QProcess, QTimer, Signal, Qt
from PySide6.QtNetwork import QLocalSocket
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMainWindow, QPlainTextEdit, QPushButton, QSplitter, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget, QTabWidget,
)
from tracking_panel import TrackingPanel

ROOT = Path(__file__).resolve().parents[1]


def default_binary():
    override = os.environ.get("SLIMEVR_SERVER_BIN")
    if override:
        return override
    candidates = [ROOT / "target" / profile / "slimevr-server-rs" for profile in ("release", "debug")]
    existing = [path for path in candidates if path.exists()]
    if existing:
        return str(max(existing, key=lambda path: path.stat().st_mtime))
    return shutil.which("slimevr-server-rs") or str(candidates[0])


def default_socket():
    return str(Path(os.environ.get("XDG_RUNTIME_DIR", "/run/user/1000")) / "SlimeVRControl")


class ControlClient(QObject):
    snapshot = Signal(dict)
    failure = Signal(str)
    connection_changed = Signal(bool)
    command_finished = Signal(str, bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.socket = QLocalSocket(self)
        self.socket.connected.connect(self._connected)
        self.socket.disconnected.connect(self._disconnected)
        self.socket.readyRead.connect(self._read)
        self.socket.errorOccurred.connect(self._error)
        self.queue = deque()
        self.pending = None
        self.buffer = bytearray()
        self.deadline = QTimer(self)
        self.deadline.setSingleShot(True)
        self.deadline.setInterval(4000)
        self.deadline.timeout.connect(self._timeout)

    @property
    def connected(self):
        return self.socket.state() == QLocalSocket.LocalSocketState.ConnectedState

    def connect_to(self, path):
        if self.socket.state() != QLocalSocket.LocalSocketState.UnconnectedState:
            return
        self.socket.connectToServer(path)

    def disconnect(self):
        self.deadline.stop()
        self.socket.abort()
        self._disconnected()

    def _connected(self):
        self.connection_changed.emit(True)
        self.send("status")

    def _disconnected(self):
        self.deadline.stop()
        self.pending = None
        self.queue.clear()
        self.buffer.clear()
        self.connection_changed.emit(False)

    def _error(self, _error):
        self.failure.emit(self.socket.errorString())

    def _timeout(self):
        self.failure.emit("The server did not answer. Reconnecting…")
        self.disconnect()

    def send(self, command):
        if not self.connected:
            return False
        if command == "status" and (self.pending or self.queue):
            return False
        if len(self.queue) >= 16:
            return False
        self.queue.append(dict(command) if isinstance(command, dict) else {"command": command})
        self._next()
        return True

    def _next(self):
        if self.pending is not None or not self.queue:
            return
        self.pending = self.queue.popleft()
        payload = json.dumps(self.pending).encode() + b"\n"
        self.socket.write(payload)
        self.deadline.start()

    def _read(self):
        self.buffer.extend(bytes(self.socket.readAll()))
        if len(self.buffer) > 1024 * 1024:
            self.failure.emit("Invalid oversized status response")
            self.disconnect()
            return
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            try:
                response = json.loads(line)
                snapshot = response["status"]
                if not isinstance(snapshot, dict):
                    raise ValueError("status must be an object")
            except (ValueError, KeyError, TypeError) as error:
                self.failure.emit(f"Invalid status response: {error}")
                self.disconnect()
                return
            self.deadline.stop()
            command = self.pending["command"] if self.pending else "status"
            self.pending = None
            self.snapshot.emit(snapshot)
            self.command_finished.emit(command, bool(response.get("ok")), response.get("error") or "")
            if not response.get("ok"):
                self.failure.emit(response.get("error") or "Command failed")
            self._next()


class HubWindow(QMainWindow):
    def __init__(self, server=None, config=None, socket_path=None, attach=False):
        super().__init__()
        self.setWindowTitle("Shora · Native tracking hub")
        self.resize(1120, 850)
        self.client = ControlClient(self)
        self.client.snapshot.connect(self.update_snapshot)
        self.client.failure.connect(self.show_error)
        self.client.connection_changed.connect(self.connection_changed)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_logs)
        self.process.finished.connect(self.process_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.reconnect = False
        self.closing = False
        self.stopping = False
        self.last_snapshot = None
        self.setup_ui(server or default_binary(), config or "", socket_path or default_socket())
        self.tracking.command_requested.connect(self.send_control)
        self.client.command_finished.connect(self.tracking.command_finished)
        self.poll = QTimer(self)
        self.poll.setInterval(50)
        self.poll.timeout.connect(self.refresh)
        self.poll.start()
        if attach:
            QTimer.singleShot(0, self.attach_server)

    def setup_ui(self, server, config, socket_path):
        container = QWidget()
        self.setCentralWidget(container)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(14)
        header = QHBoxLayout()
        title = QLabel("SHORA")
        title.setObjectName("brand")
        header.addWidget(title)
        subtitle = QLabel("Native body, face & eye tracking")
        subtitle.setObjectName("muted")
        header.addWidget(subtitle)
        header.addStretch()
        self.connection_label = QLabel("OFFLINE")
        self.connection_label.setObjectName("badge")
        header.addWidget(self.connection_label)
        layout.addLayout(header)

        self.settings = QGroupBox("Connect your tracking setup")
        form = QGridLayout(self.settings)
        self.binary = QLineEdit(server)
        self.config_path = QLineEdit(config)
        self.config_path.setPlaceholderText("Optional TOML config")
        self.socket_path = QLineEdit(socket_path)
        self.serial = QLineEdit()
        self.serial.setPlaceholderText("Auto-detect GX6/GX2, or device paths separated by ;")
        self.haritorax = QCheckBox("Enable HaritoraX")
        self.haritorax.setToolTip("When unchecked, HaritoraX follows the config file.")
        self.model = QComboBox()
        self.model.addItems(["Use config model", "HaritoraX 2", "HaritoraX Wireless"])
        self.face = QComboBox()
        self.face.addItems(["Use config face source", "Face disabled", "Babble / EyeTrackVR", "OpenXR (face-xr build)"])
        self.face_output = QComboBox()
        self.face_output.addItems(["Use config output", "Auto / OSCQuery", "UniFT", "VRChat JSON", "UniFT + JSON"])
        self.face_sheet = QLineEdit()
        self.face_sheet.setPlaceholderText("VRChat avatar OSC JSON file (optional)")
        self.face_sheet.setToolTip("Uses parameters[].name and input.address/type. Select JSON or UniFT + JSON, or leave output on Auto.")
        for row, (label, widget, browse) in enumerate([
            ("Server binary", self.binary, True), ("Configuration", self.config_path, True),
            ("Control socket", self.socket_path, False), ("Serial devices", self.serial, False),
        ]):
            form.addWidget(QLabel(label), row, 0)
            form.addWidget(widget, row, 1, 1, 3)
            if browse:
                button = QPushButton("Browse…")
                button.clicked.connect(lambda _, target=widget: self.browse(target))
                form.addWidget(button, row, 4)
        form.addWidget(self.haritorax, 4, 0)
        form.addWidget(self.model, 4, 1)
        form.addWidget(self.face, 4, 2, 1, 2)
        form.addWidget(QLabel("Face output"), 5, 0)
        form.addWidget(self.face_output, 5, 1)
        form.addWidget(self.face_sheet, 5, 2, 1, 2)
        sheet_browse = QPushButton("Browse…")
        sheet_browse.clicked.connect(lambda: self.browse(self.face_sheet))
        form.addWidget(sheet_browse, 5, 4)
        self.face_output.currentIndexChanged.connect(lambda index: (
            self.face_sheet.setEnabled(index != 2), sheet_browse.setEnabled(index != 2)))
        self.start_button = QPushButton("Start server")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_server)
        self.attach_button = QPushButton("Attach")
        self.attach_button.clicked.connect(self.attach_server)
        self.stop_button = QPushButton("Stop server")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_server)
        self.detach_button = QPushButton("Disconnect view")
        self.detach_button.clicked.connect(self.detach_server)
        actions = QHBoxLayout()
        for button in [self.start_button, self.attach_button, self.stop_button, self.detach_button]:
            actions.addWidget(button)
        layout.addLayout(actions)

        cards = QHBoxLayout()
        self.cards = {}
        for name, heading in [("body", "BODY TRACKING"), ("face", "FACE & EYES"), ("input", "SERIAL INPUT")]:
            card = QFrame()
            card.setObjectName("card")
            card_layout = QVBoxLayout(card)
            caption = QLabel(heading)
            caption.setObjectName("muted")
            value = QLabel("Waiting for server")
            value.setWordWrap(True)
            card_layout.addWidget(caption)
            card_layout.addWidget(value)
            cards.addWidget(card)
            self.cards[name] = value
        layout.addLayout(cards)
        controls = QHBoxLayout()
        self.command_buttons = {}
        for label, command in [("Yaw reset", "yaw_reset"), ("Full reset", "full_reset"),
                               ("Mounting reset", "mounting_reset"), ("Pause tracking", "pause_tracking"),
                               ("Reconnect serial", "restart")]:
            button = QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda _, cmd=command: self.client.send(cmd))
            controls.addWidget(button)
            self.command_buttons[command] = button
        layout.addLayout(controls)

        self.tabs = QTabWidget()
        self.tracking = TrackingPanel()
        self.tabs.addTab(self.tracking, "Skeleton && calibration")
        devices = QWidget()
        devices_layout = QVBoxLayout(devices)
        devices_layout.addWidget(self.settings)
        self.trackers = QTableWidget(0, 6)
        self.trackers.setHorizontalHeaderLabels(["Tracker", "Body ID", "State", "Battery", "Last seen", "Source"])
        self.trackers.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.trackers.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.trackers.setAlternatingRowColors(True)
        self.trackers.verticalHeader().hide()
        self.trackers.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.trackers.horizontalHeader().setSectionResizeMode(5, QHeaderView.ResizeMode.Stretch)
        devices_layout.addWidget(self.trackers, 1)
        self.tabs.addTab(devices, "Devices && OSC")
        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.document().setMaximumBlockCount(500)
        self.logs.setPlaceholderText("Server output appears here when launched from this window.")
        self.tabs.addTab(self.logs, "Server log")
        layout.addWidget(self.tabs, 1)
        self.message = QLabel("Start the Rust server, or attach to one already running.")
        self.message.setWordWrap(True)
        self.message.setObjectName("muted")
        layout.addWidget(self.message)

    def browse(self, target):
        path, _ = QFileDialog.getOpenFileName(self, "Choose file", target.text())
        if path:
            target.setText(path)

    def send_control(self, request):
        if not self.last_snapshot or not self.client.send(request):
            self.tracking.command_finished(request["command"], False, "Server unavailable or command queue is full")

    def start_server(self):
        if self.process.state() != QProcess.ProcessState.NotRunning or self.client.connected:
            self.show_error("Disconnect the attached view before starting another server.")
            return
        binary = Path(self.binary.text()).expanduser()
        if not binary.is_file() or not os.access(binary, os.X_OK):
            self.show_error("Server binary is missing. Run cargo build --release or choose an executable.")
            return
        # Do not launch another backend on a control endpoint that is already active.
        probe = QLocalSocket()
        probe.connectToServer(self.socket_path.text())
        if probe.waitForConnected(150):
            probe.abort()
            self.show_error("A server already owns this socket. Use Attach.")
            return
        args = ["--ui", "headless", "--control-socket", self.socket_path.text()]
        if self.config_path.text():
            args += ["--config", str(Path(self.config_path.text()).expanduser())]
        if self.haritorax.isChecked():
            args.append("--haritorax")
        for path in self.serial.text().split(";"):
            if path.strip():
                args += ["--serial-port", path.strip()]
        if self.model.currentIndex():
            args += ["--haritorax-model", "x2" if self.model.currentIndex() == 1 else "wireless"]
        if self.face.currentIndex() == 1:
            args.append("--no-face")
        elif self.face.currentIndex() in (2, 3):
            args += ["--face-source", "babble" if self.face.currentIndex() == 2 else "openxr"]
        if self.face_output.currentIndex():
            args += ["--face-output", ["auto", "unift", "json", "both"][self.face_output.currentIndex() - 1]]
        if self.face_sheet.text().strip() and self.face_output.currentIndex() != 2:
            args += ["--face-translation-sheet", str(Path(self.face_sheet.text().strip()).expanduser())]
        self.logs.clear()
        self.reconnect = True
        self.stopping = False
        self.process.setProgram(str(binary.resolve()))
        self.process.setArguments(args)
        self.process.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.message.setText("Starting tracking services…")

    def attach_server(self):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            return
        self.reconnect = True
        self.client.disconnect()
        self.client.connect_to(self.socket_path.text())

    def detach_server(self):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.show_error("This window owns the server. Stop it before disconnecting.")
            return
        self.reconnect = False
        self.client.disconnect()

    def refresh(self):
        if self.client.connected:
            self.client.send("status")
        elif self.reconnect and not self.stopping:
            self.client.connect_to(self.socket_path.text())

    def connection_changed(self, connected):
        self.connection_label.setText("CONNECTED" if connected else "OFFLINE")
        for button in self.command_buttons.values():
            # Wait for a status reply, including the owned-process identity check.
            button.setEnabled(False)
        if not connected:
            self.last_snapshot = None
            self.tracking.set_snapshot(None)
            self.trackers.setRowCount(0)
            for card in self.cards.values():
                card.setText("Waiting for server")
        self.start_button.setEnabled(not connected and self.process.state() == QProcess.ProcessState.NotRunning)

    def update_snapshot(self, snapshot):
        if self.process.state() != QProcess.ProcessState.NotRunning and snapshot.get("pid") != self.process.processId():
            self.reconnect = False
            self.client.disconnect()
            self.show_error("Control socket belongs to a different process; refusing to control it.")
            return
        self.last_snapshot = snapshot
        self.tracking.set_snapshot(snapshot)
        for button in self.command_buttons.values():
            button.setEnabled(True)
        trackers = snapshot.get("trackers", [])
        for command in ("yaw_reset", "full_reset", "mounting_reset"):
            self.command_buttons[command].setEnabled(any(t.get("active") for t in trackers))
        paused = snapshot.get("paused", False)
        self.cards["body"].setText(f"{'Paused' if paused else 'Tracking'} · {len(trackers)} trackers · {snapshot.get('bone_count', 0)} bones\nHMD {'received' if snapshot.get('hmd_pose_received') else 'waiting'}")
        self.cards["face"].setText(f"{snapshot.get('face_source', 'disabled')} → {snapshot.get('face_output', 'auto')} · {'active' if snapshot.get('face_active') else 'waiting / off'}")
        ports = snapshot.get("ports", [])
        serial_status = "Disabled" if not snapshot.get("haritorax_enabled") else "No dongles found · reconnect to rescan"
        if ports:
            serial_status = "\n".join(f"{p['path']}: {'connected' if p['connected'] else p.get('error') or 'waiting'}" for p in ports)
        self.cards["input"].setText(serial_status)
        self.command_buttons["pause_tracking"].setText("Resume tracking" if paused else "Pause tracking")
        self.command_buttons["restart"].setEnabled(bool(snapshot.get("haritorax_enabled")))
        self.trackers.setRowCount(len(trackers))
        for row, tracker in enumerate(trackers):
            battery = tracker.get("battery_percent")
            values = [tracker["name"], str(tracker["position"]), "Active" if tracker["active"] else "Waiting",
                      "—" if battery is None else f"{battery:.0f}%", f"{tracker['age_ms']} ms", tracker["source"]]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setToolTip(f"{tracker['mac']} / sensor {tracker['sensor_id']}")
                self.trackers.setItem(row, col, item)
        if snapshot.get("last_action"):
            self.message.setText(snapshot["last_action"])
        elif snapshot.get("log_file"):
            self.message.setText(f"Logs: {snapshot['log_file']}")
        else:
            self.message.setText("Connected · calibration commands apply to the native tracking server.")

    def show_error(self, message):
        self.message.setText(message)

    def read_logs(self):
        text = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
        # Strip terminal color escapes; this is a text log pane.
        import re
        self.logs.appendPlainText(re.sub(r"\x1b\[[0-9;]*m", "", text).rstrip())

    def stop_server(self):
        if self.process.state() == QProcess.ProcessState.NotRunning:
            return
        self.stopping = True
        self.reconnect = False
        verified = self.last_snapshot and self.last_snapshot.get("pid") == self.process.processId()
        if not verified or not self.client.send("shutdown"):
            self.process.terminate()
        QTimer.singleShot(3000, self.terminate_owned)
        QTimer.singleShot(5000, self.kill_owned)

    def terminate_owned(self):
        if self.stopping and self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.terminate()

    def kill_owned(self):
        if self.stopping and self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()

    def process_finished(self, code, _status):
        self.read_logs()
        self.reconnect = False
        self.stopping = False
        self.client.disconnect()
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        self.message.setText("Server stopped" if code == 0 else f"Server exited with code {code}. See logs.")
        if self.closing:
            self.close()

    def process_error(self, error):
        message = self.process.errorString()
        if error == QProcess.ProcessError.FailedToStart:
            self.process_finished(-1, QProcess.ExitStatus.CrashExit)
        self.show_error(message)

    def closeEvent(self, event):
        self.tracking.cancel_calibration()
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.closing = True
            self.stop_server()
            event.ignore()
        else:
            self.reconnect = False
            self.client.disconnect()
            event.accept()


STYLE = """
QWidget { background: #141d27; color: #e2edf2; font-family: 'DejaVu Sans'; font-size: 12px; }
QLabel#brand { font-size: 28px; font-weight: 700; letter-spacing: 5px; color: #7de1c3; }
QLabel#muted { color: #9cabb9; }
QLabel#badge { border: 1px solid #3e7568; border-radius: 10px; padding: 7px 14px; color: #7de1c3; }
QGroupBox { border: 1px solid #344351; border-radius: 9px; margin-top: 12px; padding-top: 14px; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; color: #b4c8d5; }
QLineEdit, QComboBox, QPlainTextEdit { background: #0e1620; border: 1px solid #344351; border-radius: 5px; padding: 7px; selection-background-color: #316d61; }
QPushButton { background: #263744; border: 1px solid #405664; border-radius: 5px; padding: 8px 12px; }
QPushButton:hover { border-color: #7de1c3; background: #314a55; }
QPushButton:disabled { color: #71818d; background: #1b2833; border-color: #293945; }
QPushButton#primary { color: #0d251d; background: #7de1c3; font-weight: 600; }
QPushButton#primary:disabled { color: #71818d; background: #1b2833; border-color: #293945; }
QFrame#card { background: #1d2b37; border: 1px solid #344351; border-radius: 9px; }
QFrame#card QLabel { background: transparent; }
QTableWidget { border: 1px solid #344351; border-radius: 6px; background: #101923; alternate-background-color: #1b2935; gridline-color: #273744; }
QHeaderView::section { background: #253543; padding: 8px; border: none; color: #a7c4d0; }
QSplitter::handle { background: #344351; height: 4px; }
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default=default_binary())
    parser.add_argument("--config")
    parser.add_argument("--socket", default=default_socket())
    parser.add_argument("--attach", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    window = HubWindow(args.server, args.config, args.socket, args.attach)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
