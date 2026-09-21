"""Native Qt skeleton view: projects authoritative server bone endpoints."""
import math
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QLinearGradient
from PySide6.QtWidgets import QWidget


class SkeletonPreview(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 340)
        self.setMouseTracking(True)
        self.setToolTip("Drag to orbit · scroll to zoom · double-click to reset view")
        self.bones = []
        self.snapshot = None
        self.yaw = -25.0
        self.pitch = 10.0
        self.zoom = 1.0
        self.grid = True
        self.drag_start = None
        self.projected_bones = []

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        # Ignore malformed/nonfinite points rather than passing them into Qt paint.
        self.bones = [bone for bone in (snapshot or {}).get("bones", [])
                      if all(len(bone.get(key, [])) == 3 and
                             all(isinstance(v, (int, float)) and math.isfinite(v) for v in bone[key])
                             for key in ("head", "tail"))]
        self.update()

    def set_view(self, view):
        self.yaw, self.pitch = {"Front": (0.0, 0.0), "Side": (90.0, 0.0), "Orbit": (-25.0, 10.0)}[view]
        self.zoom = 1.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self.drag_start is not None:
            delta = event.position() - self.drag_start
            self.yaw = (self.yaw + delta.x() * 0.5) % 360
            self.pitch = max(-75, min(75, self.pitch + delta.y() * 0.35))
            self.drag_start = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        self.drag_start = None
        self.unsetCursor()

    def mouseDoubleClickEvent(self, event):
        self.set_view("Orbit")

    def wheelEvent(self, event):
        self.zoom = max(0.35, min(3.0, self.zoom * math.exp(event.angleDelta().y() / 1000)))
        self.update()
        event.accept()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        background = QLinearGradient(0, 0, 0, self.height())
        background.setColorAt(0, QColor("#101e2b"))
        background.setColorAt(1, QColor("#152d36"))
        painter.fillRect(self.rect(), background)
        self.projected_bones = []
        if not self.bones:
            painter.setPen(QColor("#9cabb9"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                             "Connect to the server to preview the skeleton")
            return
        snapshot = self.snapshot
        points = [point for bone in self.bones for point in (bone["head"], bone["tail"])]
        neck = next((bone["head"] for bone in self.bones if bone["body_part"] == 2), points[0])
        height = snapshot.get("settings", {}).get("height_m", 1.8)
        # Follow horizontal HMD movement; keep scale stable while the user moves.
        center = (neck[0], neck[1] - height * 0.45, neck[2])
        scale = min(self.width() / (height * 1.5), (self.height() - 100) / (height * 1.2)) * self.zoom
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)

        def project(point):
            x, y, z = (point[i] - center[i] for i in range(3))
            x, z = cy * x + sy * z, -sy * x + cy * z
            y, depth = cp * y - sp * z, sp * y + cp * z
            return QPointF(self.width() * 0.5 + x * scale, self.height() * 0.51 - y * scale), depth

        # HMD position uses world floor y=0. Without it, use the pose's relative floor.
        floor = 0.0 if snapshot.get("hmd_pose_received") else min(p[1] for p in points)
        if self.grid:
            painter.setPen(QPen(QColor("#28464e"), 1))
            for index in range(-6, 7):
                d = index * 0.25
                for a, b in [((neck[0] + d, floor, neck[2] - 1.5), (neck[0] + d, floor, neck[2] + 1.5)),
                             ((neck[0] - 1.5, floor, neck[2] + d), (neck[0] + 1.5, floor, neck[2] + d))]:
                    painter.drawLine(project(a)[0], project(b)[0])
        # Back-to-front painter ordering provides consistent orbit views without a GPU dependency.
        ordered = sorted(self.bones, key=lambda b: project(b["head"])[1], reverse=True)
        for bone in ordered:
            start, _ = project(bone["head"])
            end, _ = project(bone["tail"])
            tracked = bone.get("tracked", False)
            color = QColor("#7de1c3" if tracked else "#7893a5")
            if not any(t.get("active") for t in snapshot.get("trackers", [])):
                color = QColor("#526a7a")
            pen = QPen(color, 6 if tracked else 4)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            painter.drawLine(start, end)
            painter.setPen(QPen(QColor("#b9d6df"), 1))
            painter.setBrush(QColor("#203e49"))
            painter.drawEllipse(start, 4.5, 4.5)
            self.projected_bones.append((bone["body_part"], start, end))
        head, _ = project(neck)
        radius = max(8, height * 0.046 * scale)
        painter.setBrush(QColor("#274d59"))
        painter.setPen(QPen(QColor("#c3e4e9"), 2))
        painter.drawEllipse(head, radius, radius)
        painter.setPen(QColor("#e2edf2"))
        mode = "PAUSED" if snapshot.get("paused") else "LIVE SKELETON"
        painter.drawText(QPointF(20, 28), mode)
        painter.setPen(QColor("#9cabb9"))
        source = "HMD anchored" if snapshot.get("hmd_pose_received") else "Relative pose · waiting for HMD"
        if not any(t.get("active") for t in snapshot.get("trackers", [])):
            source = "Reference pose · waiting for tracker samples"
        painter.drawText(QPointF(20, 48), source)
        painter.drawText(QRectF(20, self.height() - 45, self.width() - 40, 35), Qt.AlignmentFlag.AlignLeft,
                         "Mint: tracked bone   ·   Slate: inferred bone\nDrag to orbit · scroll to zoom")
