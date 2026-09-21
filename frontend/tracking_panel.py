"""Live skeleton, guided calibration, and authoritative server tuning controls."""
import math
import time
from PySide6.QtCore import QIODevice, QSaveFile, QTimer, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox, QLabel,
    QPushButton, QComboBox, QDoubleSpinBox, QSpinBox, QCheckBox,
    QScrollArea, QFileDialog,
)
from skeleton_preview import SkeletonPreview

DEFAULTS = {"height_m": 1.8, "smoothing": 0.3, "prediction": 0.0,
            "drift_correction": False, "drift_amount": 0.5}
CALIBRATIONS = [
    ("Full standing reset", "full_reset", "Stand upright and face forward. Keep still until the countdown finishes."),
    ("Yaw alignment", "yaw_reset", "Face forward in your reference direction. Only yaw alignment changes."),
    ("Mounting reset", "mounting_reset", "First do a full standing reset. Then bend your knees and lean your torso forward in a ski pose, keeping feet parallel and facing forward. Hold until the countdown finishes."),
]


class TrackingPanel(QWidget):
    command_requested = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.snapshot = None
        self.dirty = False
        self.loading = False
        self.applying = False
        self.deadline = None
        self.calibration_command = None
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.calibration_tick)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 12, 0, 0)
        left = QVBoxLayout()
        views = QHBoxLayout()
        self.preview = SkeletonPreview()
        for name in ["Front", "Side", "Orbit"]:
            button = QPushButton(name)
            button.clicked.connect(lambda _, view=name: self.preview.set_view(view))
            views.addWidget(button)
        grid = QCheckBox("Floor grid")
        grid.setChecked(True)
        grid.toggled.connect(self.set_grid)
        views.addWidget(grid)
        left.addLayout(views)
        left.addWidget(self.preview, 1)
        layout.addLayout(left, 3)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(340)
        scroll.setMaximumWidth(440)
        content = QWidget()
        right = QVBoxLayout(content)
        right.setContentsMargins(8, 0, 8, 8)
        self.calibration_group = QGroupBox("Calibration")
        calibration = QVBoxLayout(self.calibration_group)
        self.calibration_type = QComboBox()
        self.calibration_type.addItems([entry[0] for entry in CALIBRATIONS])
        calibration.addWidget(self.calibration_type)
        self.instructions = QLabel(CALIBRATIONS[0][2])
        self.instructions.setWordWrap(True)
        calibration.addWidget(self.instructions)
        self.calibration_type.currentIndexChanged.connect(lambda index: self.instructions.setText(CALIBRATIONS[index][2]))
        row = QHBoxLayout()
        row.addWidget(QLabel("Countdown"))
        self.delay = QSpinBox()
        self.delay.setRange(0, 10)
        self.delay.setValue(3)
        self.delay.setSuffix(" s")
        row.addWidget(self.delay)
        calibration.addLayout(row)
        row = QHBoxLayout()
        self.calibrate_button = QPushButton("Start calibration")
        self.calibrate_button.clicked.connect(self.start_calibration)
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel_calibration)
        self.cancel_button.setEnabled(False)
        row.addWidget(self.calibrate_button)
        row.addWidget(self.cancel_button)
        calibration.addLayout(row)
        self.calibration_status = QLabel("Waiting for tracker samples")
        self.calibration_status.setWordWrap(True)
        calibration.addWidget(self.calibration_status)
        right.addWidget(self.calibration_group)

        self.tuning_group = QGroupBox("Tracking settings")
        tuning = QVBoxLayout(self.tuning_group)
        form = QFormLayout()
        self.height = self.spin(50, 300, 180, 1, " cm")
        self.smoothing = self.spin(0, 1, 0.3, 3)
        self.smoothing.setSingleStep(0.05)
        self.smoothing.setSpecialValueText("Off")
        self.prediction = self.spin(0, 200, 0, 1, " ms")
        self.prediction.setSingleStep(5)
        self.drift_enabled = QCheckBox("Enable drift correction")
        self.drift_amount = self.spin(0, 100, 50, 1, " %")
        form.addRow("Body height", self.height)
        form.addRow("Smoothing blend", self.smoothing)
        form.addRow("Prediction", self.prediction)
        form.addRow(self.drift_enabled)
        form.addRow("Drift amount", self.drift_amount)
        tuning.addLayout(form)
        help_text = QLabel("Smoothing: lower nonzero values smooth more; 1 follows the newest sample. Prediction offsets latency. Height rebuilds body proportions.")
        help_text.setWordWrap(True)
        help_text.setObjectName("muted")
        tuning.addWidget(help_text)
        self.drift_status = QLabel("Drift learns yaw change between resets. Enable it, reset facing forward, then repeat when drift appears.")
        self.drift_status.setWordWrap(True)
        tuning.addWidget(self.drift_status)
        self.clear_drift = QPushButton("Clear learned drift")
        self.clear_drift.clicked.connect(lambda: self.command_requested.emit({"command": "clear_drift"}))
        tuning.addWidget(self.clear_drift)
        row = QHBoxLayout()
        self.apply_button = QPushButton("Apply settings")
        self.apply_button.setObjectName("primary")
        self.apply_button.clicked.connect(self.apply_settings)
        self.reload_button = QPushButton("Reload")
        self.reload_button.clicked.connect(self.reload_settings)
        row.addWidget(self.apply_button)
        row.addWidget(self.reload_button)
        tuning.addLayout(row)
        self.export_button = QPushButton("Export tuning TOML…")
        self.export_button.setToolTip("Save active settings as a startup profile, or copy them into your existing server config.")
        self.export_button.clicked.connect(self.choose_export)
        tuning.addWidget(self.export_button)
        self.notice = QLabel("Changes apply to the running server. Export to keep settings for future sessions.")
        self.notice.setWordWrap(True)
        self.notice.setObjectName("muted")
        tuning.addWidget(self.notice)
        right.addWidget(self.tuning_group)
        right.addStretch()
        scroll.setWidget(content)
        layout.addWidget(scroll, 2)
        for spin in [self.height, self.smoothing, self.prediction, self.drift_amount]:
            spin.valueChanged.connect(self.edited)
        self.drift_enabled.toggled.connect(self.edited)
        self.set_snapshot(None)

    @staticmethod
    def spin(low, high, value, decimals, suffix=""):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def set_grid(self, visible):
        self.preview.grid = visible
        self.preview.update()

    def load_fields(self, settings):
        self.loading = True
        self.height.setValue(settings["height_m"] * 100)
        self.smoothing.setValue(settings["smoothing"])
        self.prediction.setValue(settings["prediction"] * 1000)
        self.drift_enabled.setChecked(settings["drift_correction"])
        self.drift_amount.setValue(settings["drift_amount"] * 100)
        self.drift_amount.setEnabled(settings["drift_correction"])
        self.loading = False

    def edited(self, *_):
        if self.loading:
            return
        self.dirty = True
        self.drift_amount.setEnabled(self.drift_enabled.isChecked())
        self.apply_button.setEnabled(bool(self.snapshot) and not self.applying)
        self.notice.setText("Unapplied changes · Apply settings to update the running server.")

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        self.preview.set_snapshot(snapshot)
        supported = bool(snapshot and isinstance(snapshot.get("settings"), dict))
        self.tuning_group.setEnabled(supported and not self.applying)
        active = bool(snapshot and any(t.get("active") for t in snapshot.get("trackers", [])))
        self.calibrate_button.setEnabled(active and self.deadline is None)
        self.calibration_type.setEnabled(self.deadline is None)
        self.delay.setEnabled(self.deadline is None)
        if not active and self.deadline is not None:
            self.cancel_calibration()
        if not snapshot:
            self.dirty = False
            self.applying = False
            self.calibration_status.setText("Waiting for tracker samples")
            return
        if supported and not self.dirty and not self.applying:
            self.load_fields(snapshot["settings"])
        self.apply_button.setEnabled(supported and self.dirty and not self.applying)
        reference = "HMD reference" if snapshot.get("hmd_pose_received") else "Tracker reference · HMD unavailable"
        if self.deadline is None:
            self.calibration_status.setText(f"{reference}\n{snapshot.get('calibrated_trackers', 0)} trackers calibrated · {snapshot.get('drift_samples', 0)} with learned drift")

    def apply_settings(self):
        if not self.snapshot or self.applying:
            return
        settings = {"height_m": self.height.value() / 100, "smoothing": self.smoothing.value(),
                    "prediction": self.prediction.value() / 1000,
                    "drift_correction": self.drift_enabled.isChecked(), "drift_amount": self.drift_amount.value() / 100}
        self.applying = True
        self.tuning_group.setEnabled(False)
        self.notice.setText("Applying settings…")
        self.command_requested.emit({"command": "set_settings", "settings": settings})

    def command_finished(self, command, ok, error):
        if command == "set_settings":
            self.applying = False
            if ok:
                self.dirty = False
            self.set_snapshot(self.snapshot)
            self.notice.setText("Settings applied · export a tuning profile to keep them." if ok else error)
        elif command in ("full_reset", "yaw_reset", "mounting_reset", "clear_drift"):
            result = "Drift history cleared" if command == "clear_drift" else "Calibration applied"
            self.calibration_status.setText(result if ok else error)

    def reload_settings(self):
        self.dirty = False
        self.set_snapshot(self.snapshot)
        self.notice.setText("Loaded active server settings.")

    def start_calibration(self):
        if not self.snapshot or not any(t.get("active") for t in self.snapshot.get("trackers", [])):
            return
        self.calibration_command = CALIBRATIONS[self.calibration_type.currentIndex()][1]
        self.deadline = time.monotonic() + self.delay.value()
        self.cancel_button.setEnabled(True)
        self.calibrate_button.setEnabled(False)
        self.calibration_type.setEnabled(False)
        self.delay.setEnabled(False)
        self.timer.start()
        self.calibration_tick()

    def calibration_tick(self):
        if self.deadline is None:
            return
        remaining = max(0, math.ceil(self.deadline - time.monotonic()))
        if remaining:
            self.calibration_status.setText(f"Hold your pose · calibrating in {remaining}…")
        else:
            command = self.calibration_command
            self.cancel_calibration()
            self.calibration_status.setText("Applying calibration…")
            self.command_requested.emit({"command": command})

    def cancel_calibration(self):
        self.timer.stop()
        self.deadline = None
        self.calibration_command = None
        self.cancel_button.setEnabled(False)
        self.calibration_type.setEnabled(True)
        self.delay.setEnabled(True)
        self.calibrate_button.setEnabled(bool(self.snapshot and any(t.get("active") for t in self.snapshot.get("trackers", []))))
        self.calibration_status.setText("Calibration countdown cancelled")

    def choose_export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export active tracking settings", "shora-tracking.toml", "TOML (*.toml)")
        if path:
            try:
                self.export_profile(path)
                self.notice.setText(f"Saved {path}. Use it as a server config or copy its values into your current config.")
            except OSError as error:
                self.notice.setText(str(error))

    def export_profile(self, path):
        if not self.snapshot:
            raise OSError("Connect to the server before exporting settings")
        settings = self.snapshot["settings"]
        text = "# Shora body-tracking settings. Runtime calibration offsets are not included.\n"
        for key in DEFAULTS:
            value = settings[key]
            text += f"{key} = {str(value).lower() if isinstance(value, bool) else value}\n"
        file = QSaveFile(str(path))
        if not file.open(QIODevice.OpenModeFlag.WriteOnly):
            raise OSError(file.errorString())
        payload = text.encode()
        if file.write(payload) != len(payload) or not file.commit():
            raise OSError(file.errorString())
