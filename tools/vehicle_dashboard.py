#!/usr/bin/env python3
"""Integrated serial dashboard for MSPM0 vehicle status, IMU and wheel speed."""

from __future__ import annotations

import argparse
import copy
import glob
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import serial
from PyQt6 import QtCore, QtGui, QtWidgets


MAX_POINTS = 600
PLOT_WINDOW_S = 20.0
READ_CHUNK = 512


PAIR_RE = re.compile(r"(?P<name>\w+)=\((?P<a>-?\d+(?:\.\d+)?),(?P<b>-?\d+(?:\.\d+)?)\)")
INT_PAIR_RE = re.compile(r"(?P<name>\w+)=\((?P<a>-?\d+),(?P<b>-?\d+)\)")
FIELD_RE = re.compile(r"(?P<name>[A-Za-z_][\w/]*)=(?P<value>0x[0-9A-Fa-f]+|-?\d+(?:\.\d+)?|[A-Za-z0-9_./-]+)")
IMU_FIELD_RE = re.compile(r"(?P<name>\w+)=(?P<value>-?\d+(?:\.\d+)?)")
DIST_RE = re.compile(r"dist=(?P<now>-?\d+(?:\.\d+)?)/(?P<target>-?\d+(?:\.\d+)?)")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_float(text: str | None, default: float = 0.0) -> float:
    if text is None:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def parse_int(text: str | None, default: int = 0) -> int:
    if text is None:
        return default
    try:
        if text.lower().startswith("0x"):
            return int(text, 0)
        return int(float(text))
    except ValueError:
        return default


@dataclass
class VehicleState:
    connected: bool = False
    status: str = "disconnected"
    port: str = ""
    baud: int = 115200
    last_line_time: float = 0.0
    last_update_time: float = 0.0

    mode: str = "--"
    main_state: str = "--"
    stop: int = 0
    tick: int = 0
    irq_per_s: int = 0
    distance_m: float = 0.0
    target_distance_m: float = 0.0

    cmd_left_speed: float = 0.0
    cmd_right_speed: float = 0.0
    target_left_speed: float = 0.0
    target_right_speed: float = 0.0
    measured_left_speed: float = 0.0
    measured_right_speed: float = 0.0
    applied_left_duty: float = 0.0
    applied_right_duty: float = 0.0
    count_left: int = 0
    count_right: int = 0

    twist_v: float = 0.0
    twist_w: float = 0.0

    line: str = "-------"
    line_bits: int = 0
    line_detected: int = 0
    line_position: int = 0

    imu_ready: int = 0
    imu_stable: int = 0
    yaw: float = 0.0
    hold_heading: float = 0.0
    heading_error: float = 0.0
    yaw_dmp: float = 0.0
    yaw_rel: float = 0.0
    gz: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    imu_up_ms: int = 0

    raw_line: str = ""
    history_t: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    left_meas_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    right_meas_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    left_target_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    right_target_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    yaw_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))
    gz_hist: deque[float] = field(default_factory=lambda: deque(maxlen=MAX_POINTS))

    def update_history(self) -> None:
        now = time.time()
        self.history_t.append(now)
        self.left_meas_hist.append(self.measured_left_speed)
        self.right_meas_hist.append(self.measured_right_speed)
        self.left_target_hist.append(self.target_left_speed)
        self.right_target_hist.append(self.target_right_speed)
        self.yaw_hist.append(self.yaw)
        self.gz_hist.append(self.gz)
        self.last_update_time = now

    def stale_sec(self) -> float:
        if self.last_line_time <= 0.0:
            return math.inf
        return time.time() - self.last_line_time


def update_from_status_line(state: VehicleState, line: str) -> bool:
    if "mode=" not in line:
        return False

    state.raw_line = line
    state.last_line_time = time.time()

    fields = {m.group("name"): m.group("value") for m in FIELD_RE.finditer(line)}
    if "mode" in fields:
        state.mode = fields["mode"]
    if "state" in fields:
        state.main_state = fields["state"]
    state.stop = parse_int(fields.get("stop"), state.stop)
    state.tick = parse_int(fields.get("tick"), state.tick)
    state.irq_per_s = parse_int(fields.get("irq/s"), state.irq_per_s)
    state.line_detected = parse_int(fields.get("det"), state.line_detected)
    state.line_position = parse_int(fields.get("pos"), state.line_position)
    state.imu_up_ms = parse_int(fields.get("up"), state.imu_up_ms)
    state.yaw = parse_float(fields.get("yaw"), state.yaw)
    state.hold_heading = parse_float(fields.get("hold"), state.hold_heading)
    state.heading_error = parse_float(fields.get("err"), state.heading_error)
    state.gz = parse_float(fields.get("gz"), state.gz)

    dist_match = DIST_RE.search(line)
    if dist_match:
        state.distance_m = parse_float(dist_match.group("now"), state.distance_m)
        state.target_distance_m = parse_float(dist_match.group("target"), state.target_distance_m)

    if "line" in fields:
        state.line = fields["line"]
    if "bits" in fields:
        state.line_bits = parse_int(fields["bits"], state.line_bits)

    for match in PAIR_RE.finditer(line):
        name = match.group("name")
        left = parse_float(match.group("a"))
        right = parse_float(match.group("b"))
        if name == "spd":
            state.cmd_left_speed = left
            state.cmd_right_speed = right
            if state.mode == "WHEEL":
                state.target_left_speed = left
                state.target_right_speed = right
        elif name == "target":
            state.target_left_speed = left
            state.target_right_speed = right
        elif name == "meas":
            state.measured_left_speed = left
            state.measured_right_speed = right
        elif name == "duty":
            state.applied_left_duty = left
            state.applied_right_duty = right
            if state.mode == "WHEEL":
                state.target_left_speed = left
                state.target_right_speed = right
        elif name == "vw":
            state.twist_v = left
            state.twist_w = right
        elif name == "imu":
            state.imu_ready = int(left)
            state.imu_stable = int(right)

    count_match = re.search(r"count=\((-?\d+),(-?\d+)\)", line)
    if count_match:
        state.count_left = parse_int(count_match.group(1), state.count_left)
        state.count_right = parse_int(count_match.group(2), state.count_right)

    state.update_history()
    return True


def update_from_imu_line(state: VehicleState, line: str) -> bool:
    if not line.startswith("imu:"):
        return False

    fields = {m.group("name"): m.group("value") for m in IMU_FIELD_RE.finditer(line)}
    state.raw_line = line
    state.last_line_time = time.time()
    state.yaw = parse_float(fields.get("yaw"), state.yaw)
    state.yaw_dmp = parse_float(fields.get("yaw_dmp"), state.yaw_dmp)
    state.yaw_rel = parse_float(fields.get("yaw_rel"), state.yaw_rel)
    state.gz = parse_float(fields.get("gz"), state.gz)
    state.gx = parse_float(fields.get("gx"), state.gx)
    state.gy = parse_float(fields.get("gy"), state.gy)
    state.ax = parse_float(fields.get("ax"), state.ax)
    state.ay = parse_float(fields.get("ay"), state.ay)
    state.az = parse_float(fields.get("az"), state.az)
    state.pitch = parse_float(fields.get("pitch"), state.pitch)
    state.roll = parse_float(fields.get("roll"), state.roll)
    state.imu_ready = parse_int(fields.get("rdy"), state.imu_ready)
    state.imu_stable = parse_int(fields.get("stb"), state.imu_stable)
    state.imu_up_ms = parse_int(fields.get("up"), state.imu_up_ms)
    state.update_history()
    return True


class SerialWorker(QtCore.QThread):
    line_received = QtCore.pyqtSignal(str)
    state_changed = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int, auto_imu: bool, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.auto_imu = auto_imu
        self._stop_requested = False
        self._ser: serial.Serial | None = None
        self._state = VehicleState(port=port, baud=baud)
        self._pending_commands: deque[str] = deque()
        self._lock = QtCore.QMutex()
        self._last_imu_cmd = 0.0

    def run(self) -> None:
        buf = b""
        try:
            self._ser = serial.Serial()
            self._ser.port = self.port
            self._ser.baudrate = self.baud
            self._ser.timeout = 0.1
            self._ser.dsrdtr = False
            self._ser.rtscts = False
            self._ser.xonxoff = False
            self._ser.dtr = False
            self._ser.rts = False
            self._ser.open()
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._state.connected = True
            self._state.status = "connected"
            self.status_changed.emit("connected")
        except Exception as exc:
            self._state.connected = False
            self._state.status = f"open failed: {exc}"
            self.status_changed.emit(self._state.status)
            self._emit_state()
            return

        if self.auto_imu:
            self.send_command("imu,50")

        while not self._stop_requested:
            self._flush_commands()
            try:
                chunk = self._ser.read(READ_CHUNK) if self._ser is not None else b""
            except Exception as exc:
                self._state.connected = False
                self._state.status = f"read failed: {exc}"
                self.status_changed.emit(self._state.status)
                break

            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    self.line_received.emit(line)
                    if update_from_status_line(self._state, line) or update_from_imu_line(self._state, line):
                        self._state.status = "streaming"
                        self._emit_state()

            if self.auto_imu and (time.time() - self._state.last_line_time) > 2.0:
                now = time.time()
                if (now - self._last_imu_cmd) > 1.5:
                    self.send_command("imu,50")
                    self._last_imu_cmd = now

        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._state.connected = False
        self._state.status = "disconnected"
        self.status_changed.emit("disconnected")
        self._emit_state()

    def stop(self) -> None:
        self._stop_requested = True
        self.wait(1200)

    def send_command(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        locker = QtCore.QMutexLocker(self._lock)
        self._pending_commands.append(command)
        del locker

    def _flush_commands(self) -> None:
        commands: list[str] = []
        locker = QtCore.QMutexLocker(self._lock)
        while self._pending_commands:
            commands.append(self._pending_commands.popleft())
        del locker

        for command in commands:
            try:
                if self._ser is not None:
                    self._ser.write((command + "\n").encode("utf-8"))
                    self._ser.flush()
            except Exception as exc:
                self._state.status = f"write failed: {exc}"
                self.status_changed.emit(self._state.status)

    def _emit_state(self) -> None:
        self.state_changed.emit(copy.deepcopy(self._state))


class PlotWidget(QtWidgets.QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.series: list[tuple[str, deque[float], QtGui.QColor]] = []
        self.t: deque[float] | None = None
        self.setMinimumHeight(170)

    def set_data(self, t: deque[float], series: list[tuple[str, deque[float], str]]) -> None:
        self.t = t
        self.series = [(name, values, QtGui.QColor(color)) for name, values, color in series]
        self.update()

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(10, 10, -10, -10)
        painter.fillRect(rect, QtGui.QColor("#121416"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#33383d"), 1))
        painter.drawRoundedRect(rect, 6, 6)

        title_rect = QtCore.QRect(rect.left() + 10, rect.top() + 6, rect.width() - 20, 24)
        painter.setPen(QtGui.QColor("#d7dde8"))
        painter.drawText(title_rect, QtCore.Qt.AlignmentFlag.AlignLeft, self.title)

        plot = rect.adjusted(42, 36, -12, -24)
        if plot.width() <= 10 or plot.height() <= 10:
            return

        painter.setPen(QtGui.QPen(QtGui.QColor("#2d3237"), 1))
        for i in range(5):
            y = plot.top() + i * plot.height() / 4.0
            painter.drawLine(plot.left(), int(y), plot.right(), int(y))
        for i in range(5):
            x = plot.left() + i * plot.width() / 4.0
            painter.drawLine(int(x), plot.top(), int(x), plot.bottom())

        if not self.t or len(self.t) < 2 or not self.series:
            painter.setPen(QtGui.QColor("#7f8a99"))
            painter.drawText(plot, QtCore.Qt.AlignmentFlag.AlignCenter, "waiting for data")
            return

        t_values = list(self.t)
        t_max = t_values[-1]
        t_min = max(t_values[0], t_max - PLOT_WINDOW_S)
        indexes = [i for i, tv in enumerate(t_values) if tv >= t_min]
        if len(indexes) < 2:
            return

        all_values: list[float] = []
        for _, values, _ in self.series:
            seq = list(values)
            all_values.extend(seq[i] for i in indexes if i < len(seq))
        if not all_values:
            return

        v_min = min(all_values)
        v_max = max(all_values)
        if abs(v_max - v_min) < 1e-6:
            v_min -= 1.0
            v_max += 1.0
        margin = (v_max - v_min) * 0.15
        v_min -= margin
        v_max += margin

        painter.setPen(QtGui.QColor("#8390a3"))
        painter.drawText(QtCore.QRect(rect.left() + 8, plot.top() - 4, 34, 18),
                         QtCore.Qt.AlignmentFlag.AlignRight, f"{v_max:.2f}")
        painter.drawText(QtCore.QRect(rect.left() + 8, plot.bottom() - 12, 34, 18),
                         QtCore.Qt.AlignmentFlag.AlignRight, f"{v_min:.2f}")

        for name, values, color in self.series:
            seq = list(values)
            points = []
            for i in indexes:
                if i >= len(seq):
                    continue
                x = plot.left() + (t_values[i] - t_min) / max(0.001, t_max - t_min) * plot.width()
                y = plot.bottom() - (seq[i] - v_min) / (v_max - v_min) * plot.height()
                points.append(QtCore.QPointF(x, y))
            if len(points) >= 2:
                painter.setPen(QtGui.QPen(color, 2))
                painter.drawPolyline(QtGui.QPolygonF(points))

        legend_x = plot.left() + 6
        for name, _, color in self.series:
            painter.setPen(QtGui.QPen(color, 3))
            painter.drawLine(legend_x, rect.bottom() - 10, legend_x + 18, rect.bottom() - 10)
            painter.setPen(QtGui.QColor("#c8d0dc"))
            painter.drawText(legend_x + 24, rect.bottom() - 17, 90, 16,
                             QtCore.Qt.AlignmentFlag.AlignLeft, name)
            legend_x += 110


class CompassWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.setMinimumSize(160, 160)

    def set_yaw(self, yaw: float) -> None:
        self.yaw = yaw
        self.update()

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(10, 10, -10, -10)
        size = min(rect.width(), rect.height())
        center = QtCore.QPointF(rect.center())
        radius = size * 0.42
        painter.setPen(QtGui.QPen(QtGui.QColor("#344054"), 2))
        painter.setBrush(QtGui.QColor("#121416"))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QtGui.QColor("#9aa6b8"))
        for label, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = math.radians(deg - 90)
            x = center.x() + math.cos(rad) * radius * 0.78
            y = center.y() + math.sin(rad) * radius * 0.78
            painter.drawText(QtCore.QRectF(x - 10, y - 10, 20, 20),
                             QtCore.Qt.AlignmentFlag.AlignCenter, label)
        rad = math.radians(self.yaw - 90.0)
        tip = QtCore.QPointF(center.x() + math.cos(rad) * radius * 0.72,
                             center.y() + math.sin(rad) * radius * 0.72)
        painter.setPen(QtGui.QPen(QtGui.QColor("#f97316"), 4))
        painter.drawLine(center, tip)
        painter.setPen(QtGui.QColor("#e5e7eb"))
        painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignBottom | QtCore.Qt.AlignmentFlag.AlignHCenter,
                         f"{self.yaw:.1f} deg")


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("cardTitle")
        self.value = QtWidgets.QLabel("--")
        self.value.setObjectName("cardValue")
        self.sub = QtWidgets.QLabel("")
        self.sub.setObjectName("cardSub")
        layout.addWidget(title_label)
        layout.addWidget(self.value)
        layout.addWidget(self.sub)

    def set_values(self, value: str, sub: str = "") -> None:
        self.value.setText(value)
        self.sub.setText(sub)


class LineSensorWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.line = "-------"
        self.detected = 0
        self.position = 0
        self.setMinimumHeight(70)

    def set_state(self, line: str, detected: int, position: int) -> None:
        self.line = line if line else "-------"
        self.detected = detected
        self.position = position
        self.update()

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(6, 8, -6, -8)
        count = 7
        gap = 8
        w = (rect.width() - gap * (count - 1)) / count
        for i in range(count):
            x = rect.left() + i * (w + gap)
            active = i < len(self.line) and self.line[i] == "1"
            color = QtGui.QColor("#22c55e" if active else "#2b3035")
            painter.setBrush(color)
            painter.setPen(QtGui.QPen(QtGui.QColor("#475569"), 1))
            painter.drawRoundedRect(QtCore.QRectF(x, rect.top(), w, 28), 5, 5)
            painter.setPen(QtGui.QColor("#cbd5e1"))
            painter.drawText(QtCore.QRectF(x, rect.top() + 34, w, 18),
                             QtCore.Qt.AlignmentFlag.AlignCenter, str(i))
        painter.setPen(QtGui.QColor("#cbd5e1"))
        painter.drawText(rect.adjusted(0, 52, 0, 0),
                         QtCore.Qt.AlignmentFlag.AlignCenter,
                         f"det={self.detected}  pos={self.position}")


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, port: str, baud: int, auto_imu: bool):
        super().__init__()
        self.setWindowTitle("MSPM0 Vehicle Dashboard")
        self.resize(1320, 860)
        self.worker: SerialWorker | None = None
        self.state = VehicleState(port=port, baud=baud)

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QHBoxLayout(root)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(12)

        self._build_left_panel(outer, port, baud, auto_imu)
        self._build_main_panel(outer)
        self._apply_style()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_stale)
        self.timer.start(250)

    def _build_left_panel(self, outer, port: str, baud: int, auto_imu: bool) -> None:
        panel = QtWidgets.QFrame()
        panel.setObjectName("sidePanel")
        panel.setFixedWidth(310)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title = QtWidgets.QLabel("Vehicle Dashboard")
        title.setObjectName("appTitle")
        layout.addWidget(title)

        self.port_box = QtWidgets.QComboBox()
        self.port_box.setEditable(True)
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        for item in ports or [port]:
            self.port_box.addItem(item)
        self.port_box.setCurrentText(port)
        self.baud_box = QtWidgets.QComboBox()
        self.baud_box.addItems(["115200", "9600"])
        self.baud_box.setCurrentText(str(baud))
        self.auto_imu_check = QtWidgets.QCheckBox("Request IMU-only stream on connect")
        self.auto_imu_check.setChecked(auto_imu)

        form = QtWidgets.QFormLayout()
        form.addRow("Port", self.port_box)
        form.addRow("Baud", self.baud_box)
        layout.addLayout(form)
        layout.addWidget(self.auto_imu_check)

        row = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disconnect_btn)
        layout.addLayout(row)

        self.link_label = QtWidgets.QLabel("disconnected")
        self.link_label.setObjectName("statusLabel")
        layout.addWidget(self.link_label)

        cmd_group = QtWidgets.QGroupBox("Commands")
        cmd_layout = QtWidgets.QVBoxLayout(cmd_group)
        buttons = QtWidgets.QGridLayout()
        for idx, (label, cmd) in enumerate([
            ("Stop", "stop"),
            ("Show PID", "showpid"),
            ("UART Stat", "uartstat"),
            ("IMU 50ms", "imu,50"),
            ("IMU toggle", "imu"),
            ("Line Raw", "lineraw"),
        ]):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _, c=cmd: self.send_command(c))
            buttons.addWidget(button, idx // 2, idx % 2)
        cmd_layout.addLayout(buttons)

        speed_form = QtWidgets.QFormLayout()
        self.left_speed = QtWidgets.QDoubleSpinBox()
        self.right_speed = QtWidgets.QDoubleSpinBox()
        for spin in (self.left_speed, self.right_speed):
            spin.setDecimals(3)
            spin.setRange(-1.2, 1.2)
            spin.setSingleStep(0.05)
        speed_form.addRow("Left m/s", self.left_speed)
        speed_form.addRow("Right m/s", self.right_speed)
        cmd_layout.addLayout(speed_form)
        speed_btn = QtWidgets.QPushButton("Set Wheel Speed")
        speed_btn.clicked.connect(self.send_speed_command)
        cmd_layout.addWidget(speed_btn)

        twist_form = QtWidgets.QFormLayout()
        self.twist_v = QtWidgets.QDoubleSpinBox()
        self.twist_w = QtWidgets.QDoubleSpinBox()
        self.twist_v.setRange(-1.0, 1.0)
        self.twist_w.setRange(-8.0, 8.0)
        self.twist_v.setDecimals(3)
        self.twist_w.setDecimals(3)
        self.twist_v.setSingleStep(0.05)
        self.twist_w.setSingleStep(0.2)
        twist_form.addRow("v m/s", self.twist_v)
        twist_form.addRow("w rad/s", self.twist_w)
        cmd_layout.addLayout(twist_form)
        twist_btn = QtWidgets.QPushButton("Set Twist")
        twist_btn.clicked.connect(self.send_twist_command)
        cmd_layout.addWidget(twist_btn)

        self.raw_cmd = QtWidgets.QLineEdit()
        self.raw_cmd.setPlaceholderText("raw command")
        self.raw_cmd.returnPressed.connect(self.send_raw_command)
        cmd_layout.addWidget(self.raw_cmd)
        layout.addWidget(cmd_group)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(600)
        layout.addWidget(self.log, 1)
        outer.addWidget(panel)

    def _build_main_panel(self, outer) -> None:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        cards = QtWidgets.QGridLayout()
        self.mode_card = MetricCard("Mode")
        self.speed_card = MetricCard("Speed L/R")
        self.duty_card = MetricCard("Duty L/R")
        self.imu_card = MetricCard("IMU")
        self.line_card = MetricCard("Line")
        self.tick_card = MetricCard("Tick / IRQ")
        for i, card in enumerate([self.mode_card, self.speed_card, self.duty_card,
                                  self.imu_card, self.line_card, self.tick_card]):
            cards.addWidget(card, i // 3, i % 3)
        layout.addLayout(cards)

        mid = QtWidgets.QHBoxLayout()
        imu_box = QtWidgets.QFrame()
        imu_box.setObjectName("card")
        imu_layout = QtWidgets.QVBoxLayout(imu_box)
        imu_title = QtWidgets.QLabel("Heading / Line Sensors")
        imu_title.setObjectName("cardTitle")
        self.compass = CompassWidget()
        self.line_widget = LineSensorWidget()
        imu_layout.addWidget(imu_title)
        imu_layout.addWidget(self.compass)
        imu_layout.addWidget(self.line_widget)
        mid.addWidget(imu_box, 1)

        plots = QtWidgets.QVBoxLayout()
        self.speed_plot = PlotWidget("Wheel Speed")
        self.imu_plot = PlotWidget("Yaw / Gyro Z")
        plots.addWidget(self.speed_plot)
        plots.addWidget(self.imu_plot)
        mid.addLayout(plots, 3)
        layout.addLayout(mid, 1)

        self.detail = QtWidgets.QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setMaximumHeight(135)
        layout.addWidget(self.detail)
        outer.addWidget(panel, 1)

    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QWidget { background: #111111; color: #e2e4e7; font-size: 13px; }
            QFrame#sidePanel, QFrame#card { background: #191b1d; border: 1px solid #33383d; border-radius: 8px; }
            QLabel#appTitle { font-size: 22px; font-weight: 700; color: #f8fafc; }
            QLabel#cardTitle { color: #aeb7c2; font-weight: 600; }
            QLabel#cardValue { font-size: 24px; font-weight: 700; color: #f8fafc; }
            QLabel#cardSub, QLabel#statusLabel { color: #a1a8b0; }
            QPushButton { background: #252a30; border: 1px solid #3a4148; border-radius: 6px; padding: 7px 9px; }
            QPushButton:hover { background: #303740; }
            QPushButton:pressed { background: #0f766e; }
            QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit {
                background: #141619; border: 1px solid #3a4148; border-radius: 6px; padding: 5px;
            }
            QGroupBox { border: 1px solid #33383d; border-radius: 8px; margin-top: 12px; padding-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #aeb7c2; }
        """)

    def connect_serial(self) -> None:
        self.disconnect_serial()
        port = self.port_box.currentText().strip()
        baud = int(self.baud_box.currentText())
        self.worker = SerialWorker(port, baud, self.auto_imu_check.isChecked())
        self.worker.line_received.connect(self.on_line)
        self.worker.state_changed.connect(self.on_state)
        self.worker.status_changed.connect(self.link_label.setText)
        self.worker.start()

    def disconnect_serial(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None

    def send_command(self, command: str) -> None:
        if self.worker is not None:
            self.worker.send_command(command)
            self.log.appendPlainText(f"> {command}")

    def send_speed_command(self) -> None:
        self.send_command(f"spd,{self.left_speed.value():.3f},{self.right_speed.value():.3f}")

    def send_twist_command(self) -> None:
        self.send_command(f"twist,{self.twist_v.value():.3f},{self.twist_w.value():.3f}")

    def send_raw_command(self) -> None:
        text = self.raw_cmd.text().strip()
        if text:
            self.send_command(text)
            self.raw_cmd.clear()

    @QtCore.pyqtSlot(str)
    def on_line(self, line: str) -> None:
        self.log.appendPlainText(line)

    @QtCore.pyqtSlot(object)
    def on_state(self, state: VehicleState) -> None:
        self.state = state
        self.update_view()

    def _refresh_stale(self) -> None:
        if self.state.connected:
            stale = self.state.stale_sec()
            if stale < math.inf:
                self.link_label.setText(f"{self.state.status}  stale={stale:.1f}s")

    def update_view(self) -> None:
        s = self.state
        self.mode_card.set_values(s.mode, f"state={s.main_state}  stop={s.stop}  port={s.port}")
        self.speed_card.set_values(
            f"{s.measured_left_speed:.3f} / {s.measured_right_speed:.3f}",
            f"target {s.target_left_speed:.3f} / {s.target_right_speed:.3f} m/s",
        )
        self.duty_card.set_values(
            f"{s.applied_left_duty:.3f} / {s.applied_right_duty:.3f}",
            f"count {s.count_left} / {s.count_right}",
        )
        imu_state = "ready" if s.imu_ready else "not ready"
        stable = "stable" if s.imu_stable else "warming"
        self.imu_card.set_values(f"yaw {s.yaw:.1f} deg", f"{imu_state}, {stable}, gz={s.gz:.2f}")
        self.line_card.set_values(s.line, f"bits=0x{s.line_bits:02X} det={s.line_detected} pos={s.line_position}")
        self.tick_card.set_values(str(s.tick), f"irq/s={s.irq_per_s} up={s.imu_up_ms}ms")
        self.compass.set_yaw(s.yaw)
        self.line_widget.set_state(s.line, s.line_detected, s.line_position)
        self.speed_plot.set_data(s.history_t, [
            ("L meas", s.left_meas_hist, "#38bdf8"),
            ("R meas", s.right_meas_hist, "#f97316"),
            ("L target", s.left_target_hist, "#22c55e"),
            ("R target", s.right_target_hist, "#eab308"),
        ])
        self.imu_plot.set_data(s.history_t, [
            ("yaw", s.yaw_hist, "#a78bfa"),
            ("gz", s.gz_hist, "#f43f5e"),
        ])
        self.detail.setPlainText(
            "\n".join([
                f"mode={s.mode} tick={s.tick} irq/s={s.irq_per_s}",
                f"distance={s.distance_m:.3f}/{s.target_distance_m:.3f} "
                f"hold={s.hold_heading:.1f} err={s.heading_error:.1f}",
                f"speed target=({s.target_left_speed:.3f},{s.target_right_speed:.3f}) "
                f"meas=({s.measured_left_speed:.3f},{s.measured_right_speed:.3f}) "
                f"duty=({s.applied_left_duty:.3f},{s.applied_right_duty:.3f})",
                f"imu ready={s.imu_ready} stable={s.imu_stable} yaw={s.yaw:.2f} "
                f"gz={s.gz:.2f} gx={s.gx:.2f} gy={s.gy:.2f} pitch={s.pitch:.1f} roll={s.roll:.1f}",
                f"line={s.line} bits=0x{s.line_bits:02X} det={s.line_detected} pos={s.line_position}",
                f"last: {s.raw_line}",
            ])
        )

    def closeEvent(self, event) -> None:
        self.disconnect_serial()
        event.accept()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyACM1")
    parser.add_argument("baud", nargs="?", type=int, default=115200)
    parser.add_argument("--auto-imu", action="store_true",
                        help="send imu,50 after connect; this switches firmware to IMU-only output")
    parser.add_argument("--no-auto-imu", action="store_false", dest="auto_imu",
                        help=argparse.SUPPRESS)
    parser.set_defaults(auto_imu=False)
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = Dashboard(args.port, args.baud, auto_imu=args.auto_imu)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
