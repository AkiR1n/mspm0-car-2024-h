#!/usr/bin/env python3
"""Browser dashboard for the MSPM0 vehicle.

Run on the machine connected to the board, then open the shown URL locally or
through VS Code port forwarding.
"""

from __future__ import annotations

import argparse
import copy
import glob
import json
import mimetypes
import queue
import re
import threading
import time
from dataclasses import asdict, dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import serial


ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "web_dashboard_static"
READ_TIMEOUT_S = 0.2
MAX_LINE_BYTES = 4096

PAIR_RE = re.compile(r"(?P<name>\w+)=\((?P<a>-?\d+(?:\.\d+)?),(?P<b>-?\d+(?:\.\d+)?)\)")
FIELD_RE = re.compile(r"(?P<name>[A-Za-z_][\w/]*)=(?P<value>0x[0-9A-Fa-f]+|-?\d+(?:\.\d+)?|[A-Za-z0-9_./@-]+)")
IMU_FIELD_RE = re.compile(r"(?P<name>\w+)=(?P<value>-?\d+(?:\.\d+)?)")
DIST_RE = re.compile(r"dist=(?P<now>-?\d+(?:\.\d+)?)/(?P<target>-?\d+(?:\.\d+)?)")
HDG_RE = re.compile(
    r"hdg=\((?P<geo>-?\d+(?:\.\d+)?),(?P<hold>-?\d+(?:\.\d+)?),(?P<err>-?\d+(?:\.\d+)?)\)"
)
PID_RE = re.compile(
    r"pid\[(?P<label>left|right)\]\s+kp=(?P<kp>-?\d+(?:\.\d+)?)\s+"
    r"ki=(?P<ki>-?\d+(?:\.\d+)?)\s+kd=(?P<kd>-?\d+(?:\.\d+)?)\s+"
    r"ff=(?P<ff>-?\d+(?:\.\d+)?)\s+mode=(?P<mode>\w+)\s+"
    r"alpha=(?P<alpha>-?\d+(?:\.\d+)?)\s+"
    r"out=\((?P<out_min>-?\d+(?:\.\d+)?),(?P<out_max>-?\d+(?:\.\d+)?)\)\s+"
    r"i=\((?P<i_min>-?\d+(?:\.\d+)?),(?P<i_max>-?\d+(?:\.\d+)?)\)"
)


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

    mode: str = "--"
    main_state: str = "--"
    challenge: str = "--"
    challenge_phase: str = "--"
    challenge_action: str = "--"
    challenge_event: str = "--"
    lap_index: int = 0
    lap_total: int = 0
    checkpoint_count: int = 0
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
    gz_raw: float = 0.0
    gz_bias: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    imu_bias_committed: int = 0
    imu_sign: float = 0.0
    imu_sens: float = 0.0
    imu_up_ms: int = 0

    auto_enabled: int = 0
    auto_phase: str = "-"
    last_ack: str = ""

    pid: dict = field(default_factory=lambda: {"left": {}, "right": {}})
    line_logic: str = "--"
    line_raw: str = "-------"
    line_norm: str = "-------"
    line_raw_bits: int = 0
    line_norm_bits: int = 0
    line_channels: list = field(default_factory=lambda: [{"pin": "--", "raw": 0, "norm": 0} for _ in range(7)])
    line_aux: dict = field(default_factory=dict)
    uart_stats: dict = field(default_factory=dict)
    imu_uart_stats: dict = field(default_factory=dict)

    raw_line: str = ""


def update_from_status_line(state: VehicleState, line: str) -> bool:
    if not line.startswith("mode="):
        return False

    fields = {m.group("name"): m.group("value") for m in FIELD_RE.finditer(line)}
    state.raw_line = line
    state.last_line_time = time.time()
    state.mode = fields.get("mode", state.mode)
    state.main_state = fields.get("state", state.main_state)
    if "q" in fields:
        parts = fields["q"].split("/", 1)
        state.challenge = parts[1] if len(parts) == 2 and parts[1] != "NONE" else parts[0]
    state.challenge_phase = fields.get("ph", state.challenge_phase)
    state.challenge_action = fields.get("act", state.challenge_action)
    state.challenge_event = fields.get("evt", state.challenge_event)
    if "lap" in fields:
        parts = fields["lap"].split("/", 1)
        state.lap_index = parse_int(parts[0], state.lap_index)
        if len(parts) == 2:
            state.lap_total = parse_int(parts[1], state.lap_total)
    state.checkpoint_count = parse_int(fields.get("cp"), state.checkpoint_count)
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
    hdg_match = HDG_RE.search(line)
    if hdg_match:
        state.hold_heading = parse_float(hdg_match.group("hold"), state.hold_heading)
        state.heading_error = parse_float(hdg_match.group("err"), state.heading_error)
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
    state.gz_raw = parse_float(fields.get("gz_raw"), state.gz_raw)
    state.gz_bias = parse_float(fields.get("gz_bias"), state.gz_bias)
    state.gx = parse_float(fields.get("gx"), state.gx)
    state.gy = parse_float(fields.get("gy"), state.gy)
    state.ax = parse_float(fields.get("ax"), state.ax)
    state.ay = parse_float(fields.get("ay"), state.ay)
    state.az = parse_float(fields.get("az"), state.az)
    state.pitch = parse_float(fields.get("pitch"), state.pitch)
    state.roll = parse_float(fields.get("roll"), state.roll)
    state.imu_ready = parse_int(fields.get("rdy"), state.imu_ready)
    state.imu_stable = parse_int(fields.get("stb"), state.imu_stable)
    state.imu_bias_committed = parse_int(fields.get("bias"), state.imu_bias_committed)
    state.imu_sign = parse_float(fields.get("sign"), state.imu_sign)
    state.imu_sens = parse_float(fields.get("sens"), state.imu_sens)
    state.imu_up_ms = parse_int(fields.get("up"), state.imu_up_ms)
    ok_match = re.search(r"ok=\(a(?P<angle>\d+),g(?P<gyro>\d+)\)", line)
    err_match = re.search(r"err=\(sum(?P<sum>\d+),sync(?P<sync>\d+),of(?P<overflow>\d+),hw(?P<hw>\d+)\)", line)
    if "rx" in fields:
        state.imu_uart_stats["rx"] = parse_int(fields.get("rx"), state.imu_uart_stats.get("rx", 0))
    if ok_match:
        state.imu_uart_stats["angle"] = parse_int(ok_match.group("angle"))
        state.imu_uart_stats["gyro"] = parse_int(ok_match.group("gyro"))
    if err_match:
        state.imu_uart_stats["sumerr"] = parse_int(err_match.group("sum"))
        state.imu_uart_stats["sync"] = parse_int(err_match.group("sync"))
        state.imu_uart_stats["overflow"] = parse_int(err_match.group("overflow"))
        state.imu_uart_stats["hw_overrun"] = parse_int(err_match.group("hw"))
    return True


def update_from_pid_line(state: VehicleState, line: str) -> bool:
    match = PID_RE.search(line)
    if match is None:
        return False
    label = match.group("label")
    state.raw_line = line
    state.last_line_time = time.time()
    state.pid[label] = {
        "kp": parse_float(match.group("kp")),
        "ki": parse_float(match.group("ki")),
        "kd": parse_float(match.group("kd")),
        "ff": parse_float(match.group("ff")),
        "mode": match.group("mode"),
        "alpha": parse_float(match.group("alpha")),
        "out_min": parse_float(match.group("out_min")),
        "out_max": parse_float(match.group("out_max")),
        "i_min": parse_float(match.group("i_min")),
        "i_max": parse_float(match.group("i_max")),
    }
    return True


def update_from_auto_line(state: VehicleState, line: str) -> bool:
    if not line.startswith("auto,"):
        return False

    parts = [part.strip() for part in line.split(",")]
    state.raw_line = line
    state.last_line_time = time.time()

    if len(parts) >= 3 and parts[1] == "ack":
        state.last_ack = line
        if parts[2] == "on":
            state.auto_enabled = 1
        elif parts[2] == "off":
            state.auto_enabled = 0
        elif parts[2] == "phase" and len(parts) >= 4:
            state.auto_phase = parts[3]
        elif parts[2] == "stop":
            state.auto_enabled = 0
        return True

    if len(parts) >= 13 and parts[1] == "pid":
        label = parts[2]
        if label in ("left", "right"):
            state.pid[label] = {
                "kp": parse_float(parts[3]),
                "ki": parse_float(parts[4]),
                "kd": parse_float(parts[5]),
                "ff": parse_float(parts[6]),
                "mode": parts[7],
                "alpha": parse_float(parts[8]),
                "out_min": parse_float(parts[9]),
                "out_max": parse_float(parts[10]),
                "i_min": parse_float(parts[11]),
                "i_max": parse_float(parts[12]),
            }
            return True

    if len(parts) >= 18 and parts[1] == "sample":
        state.auto_phase = parts[3]
        state.mode = parts[4]
        state.cmd_left_speed = parse_float(parts[7], state.cmd_left_speed)
        state.cmd_right_speed = parse_float(parts[8], state.cmd_right_speed)
        state.target_left_speed = parse_float(parts[9], state.target_left_speed)
        state.target_right_speed = parse_float(parts[10], state.target_right_speed)
        state.measured_left_speed = parse_float(parts[11], state.measured_left_speed)
        state.measured_right_speed = parse_float(parts[12], state.measured_right_speed)
        state.applied_left_duty = parse_float(parts[13], state.applied_left_duty)
        state.applied_right_duty = parse_float(parts[14], state.applied_right_duty)
        state.count_left = parse_int(parts[15], state.count_left)
        state.count_right = parse_int(parts[16], state.count_right)
        state.irq_per_s = parse_int(parts[17], state.irq_per_s)
        if len(parts) >= 19:
            state.tick = parse_int(parts[18], state.tick)
        return True

    state.last_ack = line
    return True


def update_from_line_tool_line(state: VehicleState, line: str) -> bool:
    if line.startswith("linecfg "):
        state.raw_line = line
        state.last_line_time = time.time()
        for index, pin in re.findall(r"(\d):([A-Za-z0-9_]+)", line):
            idx = parse_int(index, -1)
            if 0 <= idx < len(state.line_channels):
                state.line_channels[idx]["pin"] = pin
        return True

    if line.startswith("line_logic="):
        state.raw_line = line
        state.last_line_time = time.time()
        state.line_logic = line.split("=", 1)[1].strip()
        return True

    if line.startswith("lineraw "):
        fields = {m.group("name"): m.group("value") for m in FIELD_RE.finditer(line)}
        state.raw_line = line
        state.last_line_time = time.time()
        state.line_logic = fields.get("logic", state.line_logic)
        state.line_raw = fields.get("raw", state.line_raw)
        state.line_norm = fields.get("norm", state.line_norm)
        state.line_norm_bits = parse_int(fields.get("bits"), state.line_norm_bits)
        state.line_bits = state.line_norm_bits
        state.line_detected = parse_int(fields.get("det"), state.line_detected)
        state.line_position = parse_int(fields.get("pos"), state.line_position)
        state.line = state.line_norm
        return True

    channel_match = re.match(r"linech\[(?P<idx>\d+)\]\s+pin=(?P<pin>\S+)\s+raw=(?P<raw>\d+)\s+norm=(?P<norm>\d+)", line)
    if channel_match:
        idx = parse_int(channel_match.group("idx"), -1)
        if 0 <= idx < len(state.line_channels):
            state.raw_line = line
            state.last_line_time = time.time()
            state.line_channels[idx] = {
                "pin": channel_match.group("pin"),
                "raw": parse_int(channel_match.group("raw")),
                "norm": parse_int(channel_match.group("norm")),
            }
            return True

    if line.startswith("lineaux "):
        fields = {m.group("name"): parse_int(m.group("value")) for m in FIELD_RE.finditer(line)}
        state.raw_line = line
        state.last_line_time = time.time()
        state.line_aux = fields
        return True

    return False


def update_from_stats_line(state: VehicleState, line: str) -> bool:
    if line.startswith(("uart0 ", "uart1 ")):
        name, rest = line.split(" ", 1)
        state.raw_line = line
        state.last_line_time = time.time()
        state.uart_stats[name] = {
            m.group("name"): parse_int(m.group("value"))
            for m in FIELD_RE.finditer(rest)
        }
        return True

    if line.startswith("imu_uart "):
        state.raw_line = line
        state.last_line_time = time.time()
        state.imu_uart_stats.update({
            m.group("name"): parse_int(m.group("value"))
            for m in FIELD_RE.finditer(line)
        })
        return True

    return False


class DashboardHub:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._clients: set[queue.Queue[dict]] = set()
        self._ser: serial.Serial | None = None
        self._reader: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self.state = VehicleState()

    def snapshot(self) -> dict:
        with self._lock:
            return asdict(copy.deepcopy(self.state))

    def ports(self) -> list[str]:
        return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

    def add_client(self) -> queue.Queue[dict]:
        q: queue.Queue[dict] = queue.Queue(maxsize=256)
        with self._lock:
            self._clients.add(q)
            q.put({"kind": "state", "state": self.snapshot()})
        return q

    def remove_client(self, q: queue.Queue[dict]) -> None:
        with self._lock:
            self._clients.discard(q)

    def _broadcast(self, payload: dict) -> None:
        dead: list[queue.Queue[dict]] = []
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                dead.append(q)
        for q in dead:
            self.remove_client(q)

    def _set_status(self, status: str) -> None:
        with self._lock:
            self.state.status = status
            self.state.connected = self._ser is not None and self._ser.is_open
            snapshot = asdict(copy.deepcopy(self.state))
        self._broadcast({"kind": "state", "state": snapshot})

    def connect(self, port: str, baud: int, auto_imu: bool = False) -> None:
        self.disconnect()
        ser = serial.Serial(port, baud, timeout=READ_TIMEOUT_S)
        with self._lock:
            self._ser = ser
            self.state.port = port
            self.state.baud = baud
            self.state.connected = True
            self.state.status = "connected"
        self._stop_reader.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._broadcast({"kind": "state", "state": self.snapshot()})
        if auto_imu:
            time.sleep(0.2)
            self.send_command("imu,50")

    def disconnect(self) -> None:
        self._stop_reader.set()
        with self._lock:
            ser = self._ser
            self._ser = None
            self.state.connected = False
            self.state.status = "disconnected"
        if ser is not None:
            try:
                ser.close()
            except serial.SerialException:
                pass
        self._broadcast({"kind": "state", "state": self.snapshot()})

    def send_command(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        with self._lock:
            ser = self._ser
        if ser is None or not ser.is_open:
            raise RuntimeError("serial port is not connected")
        ser.write((command + "\n").encode("utf-8"))
        ser.flush()
        self._broadcast({"kind": "tx", "line": f"> {command}", "state": self.snapshot()})

    def _read_serial_line(self, ser: serial.Serial) -> bytes:
        line = bytearray()
        while not self._stop_reader.is_set():
            chunk = ser.read(1)
            if not chunk:
                return bytes(line)
            if chunk in (b"\n", b"\r"):
                if line:
                    return bytes(line)
                continue
            line.extend(chunk)
            if len(line) >= MAX_LINE_BYTES:
                return bytes(line)
        return b""

    def _read_loop(self) -> None:
        while not self._stop_reader.is_set():
            with self._lock:
                ser = self._ser
            if ser is None:
                break
            try:
                raw = self._read_serial_line(ser)
            except (OSError, TypeError, serial.SerialException) as exc:
                if not self._stop_reader.is_set():
                    self._set_status(f"serial error: {exc}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            with self._lock:
                parsed = (
                    update_from_status_line(self.state, line)
                    or update_from_imu_line(self.state, line)
                    or update_from_pid_line(self.state, line)
                    or update_from_auto_line(self.state, line)
                    or update_from_line_tool_line(self.state, line)
                    or update_from_stats_line(self.state, line)
                )
                snapshot = asdict(copy.deepcopy(self.state))
            self._broadcast({"kind": "line", "line": line, "parsed": parsed, "state": snapshot})


HUB = DashboardHub()


class Handler(BaseHTTPRequestHandler):
    server_version = "MSPM0WebDashboard/1.0"

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/ports":
            self._send_json({"ports": HUB.ports(), "state": HUB.snapshot()})
            return
        if path == "/api/state":
            self._send_json({"state": HUB.snapshot()})
            return
        if path == "/events":
            self._events()
            return
        self._static(path)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            body = self._read_json()
            if path == "/api/connect":
                port = str(body.get("port") or "/dev/ttyACM1")
                baud = int(body.get("baud") or 115200)
                auto_imu = bool(body.get("auto_imu", False))
                HUB.connect(port, baud, auto_imu)
                self._send_json({"ok": True, "state": HUB.snapshot()})
                return
            if path == "/api/disconnect":
                HUB.disconnect()
                self._send_json({"ok": True, "state": HUB.snapshot()})
                return
            if path == "/api/command":
                HUB.send_command(str(body.get("command") or ""))
                self._send_json({"ok": True, "state": HUB.snapshot()})
                return
        except Exception as exc:  # noqa: BLE001 - return API error details
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def _events(self) -> None:
        q = HUB.add_client()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                try:
                    payload = q.get(timeout=15.0)
                except queue.Empty:
                    payload = {"kind": "ping", "time": time.time()}
                data = json.dumps(payload, ensure_ascii=False)
                self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            HUB.remove_client(q)

    def _static(self, path: str) -> None:
        if path in ("", "/"):
            path = "/index.html"
        target = (STATIC_ROOT / path.lstrip("/")).resolve()
        if not str(target).startswith(str(STATIC_ROOT.resolve())) or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--serial-port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--connect", action="store_true", help="open serial immediately")
    parser.add_argument("--auto-imu", action="store_true", help="send imu,50 after opening serial")
    args = parser.parse_args()

    if args.connect:
        HUB.connect(args.serial_port, args.baud, args.auto_imu)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Web dashboard: http://{args.host}:{args.port}")
    print(f"Default serial: {args.serial_port} @ {args.baud}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        HUB.disconnect()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
