#!/usr/bin/env python3
"""Automate wheel speed-loop characterization and first-pass PI/FF tuning."""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import glob
import json
import math
import re
import pathlib
import statistics
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from typing import Iterable

try:
    import serial
except ImportError as exc:  # pragma: no cover
    print(
        "Missing dependency: pyserial\n"
        "Install with: python -m pip install pyserial\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    sys.exit(2)


RAW_LOG_PREFIX = "speed_tune_raw_"
CSV_PREFIX = "speed_tune_data_"
SUMMARY_PREFIX = "speed_tune_summary_"
AUTO_SAMPLE_PREFIX = "auto,sample,"
AUTO_PID_PREFIX = "auto,pid,"
DEFAULT_DUTY_STEPS = [10, 15, 20, 25, 30, 40, 50, 60]

AUTO_SAMPLE_RE = re.compile(
    r"auto,sample,"
    r"(?P<board_ms>\d+),"
    r"(?P<phase_name>[^,\r\n]+),"
    r"(?P<board_mode>[^,\r\n]+),"
    r"(?P<cmd_left_duty>-?\d+(?:\.\d+)?),"
    r"(?P<cmd_right_duty>-?\d+(?:\.\d+)?),"
    r"(?P<cmd_left_speed>-?\d+(?:\.\d+)?),"
    r"(?P<cmd_right_speed>-?\d+(?:\.\d+)?),"
    r"(?P<target_left_speed>-?\d+(?:\.\d+)?),"
    r"(?P<target_right_speed>-?\d+(?:\.\d+)?),"
    r"(?P<measured_left_speed>-?\d+(?:\.\d+)?),"
    r"(?P<measured_right_speed>-?\d+(?:\.\d+)?),"
    r"(?P<applied_left_duty>-?\d+(?:\.\d+)?),"
    r"(?P<applied_right_duty>-?\d+(?:\.\d+)?),"
    r"(?P<left_count>-?\d+),"
    r"(?P<right_count>-?\d+),"
    r"(?P<irq_per_s>\d+),"
    r"(?P<sample_tick>\d+)"
)

AUTO_PID_RE = re.compile(
    r"auto,pid,"
    r"(?P<wheel>[^,\r\n]+),"
    r"(?P<kp>-?\d+(?:\.\d+)?),"
    r"(?P<ki>-?\d+(?:\.\d+)?),"
    r"(?P<kd>-?\d+(?:\.\d+)?),"
    r"(?P<ff>-?\d+(?:\.\d+)?),"
    r"(?P<mode>[^,\r\n]+),"
    r"(?P<alpha>-?\d+(?:\.\d+)?),"
    r"(?P<out_min>-?\d+(?:\.\d+)?),"
    r"(?P<out_max>-?\d+(?:\.\d+)?),"
    r"(?P<integral_min>-?\d+(?:\.\d+)?),"
    r"(?P<integral_max>-?\d+(?:\.\d+)?)"
)

HUMAN_WHEEL_RE = re.compile(
    r"mode=WHEEL "
    r"duty=\((?P<cmd_left_duty>-?\d+(?:\.\d+)?),(?P<cmd_right_duty>-?\d+(?:\.\d+)?)\) "
    r"meas=\((?P<measured_left_speed>-?\d+(?:\.\d+)?),(?P<measured_right_speed>-?\d+(?:\.\d+)?)\) "
    r"count=\((?P<left_count>-?\d+),(?P<right_count>-?\d+)\) "
    r"irq/s=(?P<irq_per_s>\d+) "
    r"tick=(?P<sample_tick>\d+)"
)

HUMAN_WSPD_RE = re.compile(
    r"mode=WSPD "
    r"spd=\((?P<cmd_left_speed>-?\d+(?:\.\d+)?),(?P<cmd_right_speed>-?\d+(?:\.\d+)?)\) "
    r"target=\((?P<target_left_speed>-?\d+(?:\.\d+)?),(?P<target_right_speed>-?\d+(?:\.\d+)?)\) "
    r"meas=\((?P<measured_left_speed>-?\d+(?:\.\d+)?),(?P<measured_right_speed>-?\d+(?:\.\d+)?)\) "
    r"duty=\((?P<applied_left_duty>-?\d+(?:\.\d+)?),(?P<applied_right_duty>-?\d+(?:\.\d+)?)\) "
    r"count=\((?P<left_count>-?\d+),(?P<right_count>-?\d+)\) "
    r"irq/s=(?P<irq_per_s>\d+) "
    r"tick=(?P<sample_tick>\d+)"
)


@dataclasses.dataclass(frozen=True)
class Phase:
    name: str
    kind: str
    wheel: str
    left_cmd: float
    right_cmd: float
    duration_s: float


@dataclasses.dataclass
class SampleRow:
    host_time_iso: str
    phase_name: str
    phase_kind: str
    wheel_under_test: str
    phase_elapsed_s: float
    board_ms: int
    board_mode: str
    cmd_left_duty: float
    cmd_right_duty: float
    cmd_left_speed: float
    cmd_right_speed: float
    target_left_speed: float
    target_right_speed: float
    measured_left_speed: float
    measured_right_speed: float
    applied_left_duty: float
    applied_right_duty: float
    left_count: int
    right_count: int
    irq_per_s: int
    sample_tick: int


@dataclasses.dataclass
class WheelFit:
    wheel: str
    sign: float
    min_moving_duty: float
    max_speed_mps: float
    slope_speed_per_duty: float
    ff_gain: float
    tau_s: float
    kp: float
    ki: float
    kd: float
    target_low_mps: float
    target_mid_mps: float
    target_high_mps: float


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        default=pathlib.Path(__file__).resolve().parents[1],
        type=pathlib.Path,
        help="Repository root",
    )
    parser.add_argument("--port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--flash",
        action="store_true",
        help="Flash firmware with J-Link before opening serial",
    )
    parser.add_argument(
        "--flash-command-file",
        default=".vscode/jlink-flash.jlink",
        help="J-Link command file relative to repo root",
    )
    parser.add_argument(
        "--reset-before-start",
        action="store_true",
        help="Reset target via J-Link before starting serial handshake",
    )
    parser.add_argument(
        "--output-dir",
        default="logs/speed_tuning",
        help="Directory for raw log, CSV and summary files",
    )
    parser.add_argument(
        "--duty-steps",
        default=",".join(str(step) for step in DEFAULT_DUTY_STEPS),
        help="Comma-separated open-loop duty percent steps",
    )
    parser.add_argument(
        "--duty-phase-s",
        type=float,
        default=2.5,
        help="Seconds for each non-zero open-loop phase",
    )
    parser.add_argument(
        "--idle-phase-s",
        type=float,
        default=1.0,
        help="Seconds for each idle phase",
    )
    parser.add_argument(
        "--verify-phase-s",
        type=float,
        default=3.0,
        help="Seconds for each closed-loop verify phase",
    )
    parser.add_argument(
        "--moving-threshold",
        type=float,
        default=0.03,
        help="Minimum |speed| in m/s considered as moving",
    )
    parser.add_argument(
        "--boot-settle-s",
        type=float,
        default=1.5,
        help="Seconds to wait after serial opens",
    )
    parser.add_argument(
        "--command-gap-s",
        type=float,
        default=0.08,
        help="Pause after each command",
    )
    parser.add_argument(
        "--sample-period-s",
        type=float,
        default=0.05,
        help="Interval between host-polled auto,sample requests during each phase",
    )
    parser.add_argument(
        "--ready-timeout-s",
        type=float,
        default=4.0,
        help="Seconds to wait for board ready banner/status before sending test commands",
    )
    parser.add_argument(
        "--verify-high-fraction",
        type=float,
        default=0.90,
        help="Fraction of measured max speed used for high-speed closed-loop verification",
    )
    parser.add_argument(
        "--verify-high-cap-mps",
        type=float,
        default=1.50,
        help="Upper cap for the generated high-speed verify target in m/s",
    )
    return parser


def detect_port() -> str:
    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("No /dev/ttyACM* or /dev/ttyUSB* serial port found")
    return candidates[0]


def flash_firmware(repo_root: pathlib.Path, command_file: str) -> None:
    cmd = [
        "JLinkExe",
        "-CommandFile",
        str((repo_root / command_file).resolve()),
    ]
    print(f"[flash] {' '.join(cmd)}")
    subprocess.run(cmd, cwd=repo_root, check=True)


def reset_target(repo_root: pathlib.Path) -> None:
    script = "\n".join(
        [
            "si SWD",
            "speed 4000",
            "device MSPM0G3507",
            "r",
            "g",
            "exit",
            "",
        ]
    )

    with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False) as handle:
        handle.write(script)
        command_file = pathlib.Path(handle.name)

    try:
        cmd = ["JLinkExe", "-CommandFile", str(command_file)]
        print(f"[reset] {' '.join(cmd)}")
        subprocess.run(cmd, cwd=repo_root, check=True)
    finally:
        command_file.unlink(missing_ok=True)


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.1
    ser.dsrdtr = False
    ser.rtscts = False
    ser.xonxoff = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def write_command(ser: serial.Serial, text: str) -> None:
    payload = (text.rstrip("\r\n") + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()


def safe_readline(
    ser: serial.Serial,
    port: str,
    baud: int,
    reconnect_deadline: float,
) -> tuple[serial.Serial, bytes]:
    while True:
        try:
            return ser, ser.readline()
        except serial.SerialException as exc:
            last_error = exc
            try:
                ser.close()
            except Exception:
                pass

            while time.monotonic() < reconnect_deadline:
                try:
                    ser = open_serial(port, baud)
                    print(f"[serial] reconnected to {port}")
                    break
                except serial.SerialException:
                    time.sleep(0.2)
            else:
                raise serial.SerialException(
                    f"failed to reconnect to {port} before timeout: {last_error}"
                ) from last_error


def parse_auto_samples(text: str) -> list[dict[str, str]]:
    return [match.groupdict() for match in AUTO_SAMPLE_RE.finditer(text)]


def parse_human_statuses(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []

    for match in HUMAN_WHEEL_RE.finditer(text):
        row = match.groupdict()
        row["board_mode"] = "WHEEL"
        row["cmd_left_speed"] = "0.0"
        row["cmd_right_speed"] = "0.0"
        row["target_left_speed"] = row["cmd_left_duty"]
        row["target_right_speed"] = row["cmd_right_duty"]
        row["applied_left_duty"] = row["cmd_left_duty"]
        row["applied_right_duty"] = row["cmd_right_duty"]
        rows.append(row)

    for match in HUMAN_WSPD_RE.finditer(text):
        row = match.groupdict()
        row["board_mode"] = "WSPD"
        row["cmd_left_duty"] = row["applied_left_duty"]
        row["cmd_right_duty"] = row["applied_right_duty"]
        rows.append(row)

    return rows


def parse_auto_pid(line: str) -> dict[str, str] | None:
    match = AUTO_PID_RE.search(line)
    if match is None:
        return None
    return match.groupdict()


def read_serial_text(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    duration_s: float,
) -> tuple[serial.Serial, str]:
    deadline = time.monotonic() + duration_s
    chunks: list[str] = []

    while time.monotonic() < deadline:
        ser, raw = safe_readline(ser, port, baud, deadline)
        if not raw:
            continue

        decoded = raw.decode("utf-8", errors="replace")
        raw_handle.write(decoded)
        raw_handle.flush()
        sys.stdout.write(decoded)
        sys.stdout.flush()
        chunks.append(decoded)

    return ser, "".join(chunks)


def wait_for_board_ready(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    timeout_s: float,
) -> serial.Serial:
    deadline = time.monotonic() + timeout_s
    markers = ("wheel test ready", "mode=WHEEL", "mode=STOP", "pid[left]")

    while time.monotonic() < deadline:
        ser, text = read_serial_text(ser, port, baud, raw_handle, 0.25)
        if any(marker in text for marker in markers):
            return ser

    raise RuntimeError("Board did not reach ready state before timeout")


def send_command_until_ack(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    command: str,
    ack_markers: tuple[str, ...],
    timeout_s: float = 2.0,
    resend_interval_s: float = 0.35,
) -> serial.Serial:
    deadline = time.monotonic() + timeout_s
    next_send = 0.0
    seen_text = ""

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            write_command(ser, command)
            next_send = now + resend_interval_s

        ser, text = read_serial_text(ser, port, baud, raw_handle, 0.15)
        seen_text += text
        if any(marker in seen_text for marker in ack_markers):
            return ser

    raise RuntimeError(f"Timeout waiting for ack to command '{command}'")


def try_send_command_until_ack(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    command: str,
    ack_markers: tuple[str, ...],
    timeout_s: float = 2.0,
    resend_interval_s: float = 0.35,
) -> tuple[serial.Serial, bool]:
    try:
        ser = send_command_until_ack(
            ser,
            port,
            baud,
            raw_handle,
            command,
            ack_markers,
            timeout_s=timeout_s,
            resend_interval_s=resend_interval_s,
        )
        return ser, True
    except RuntimeError as exc:
        print(f"[warn] {exc}")
        return ser, False


def drain_serial(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    duration_s: float,
    phase: Phase | None = None,
) -> tuple[serial.Serial, list[SampleRow]]:
    deadline = time.monotonic() + duration_s
    phase_start = time.monotonic()
    rows: list[SampleRow] = []

    while time.monotonic() < deadline:
        ser, raw = safe_readline(ser, port, baud, deadline)
        if not raw:
            continue

        decoded = raw.decode("utf-8", errors="replace")
        raw_handle.write(decoded)
        raw_handle.flush()
        sys.stdout.write(decoded)
        sys.stdout.flush()

        if phase is None:
            continue

        parsed_rows = parse_auto_samples(decoded)
        if not parsed_rows:
            parsed_rows = parse_human_statuses(decoded)

        for parsed in parsed_rows:
            sample_tick = int(float(parsed["sample_tick"]))
            board_ms = int(float(parsed.get("board_ms", sample_tick * 10)))
            rows.append(
                SampleRow(
                    host_time_iso=dt.datetime.now().isoformat(timespec="milliseconds"),
                    phase_name=phase.name,
                    phase_kind=phase.kind,
                    wheel_under_test=phase.wheel,
                    phase_elapsed_s=time.monotonic() - phase_start,
                    board_ms=board_ms,
                    board_mode=parsed["board_mode"],
                    cmd_left_duty=float(parsed["cmd_left_duty"]),
                    cmd_right_duty=float(parsed["cmd_right_duty"]),
                    cmd_left_speed=float(parsed["cmd_left_speed"]),
                    cmd_right_speed=float(parsed["cmd_right_speed"]),
                    target_left_speed=float(parsed["target_left_speed"]),
                    target_right_speed=float(parsed["target_right_speed"]),
                    measured_left_speed=float(parsed["measured_left_speed"]),
                    measured_right_speed=float(parsed["measured_right_speed"]),
                    applied_left_duty=float(parsed["applied_left_duty"]),
                    applied_right_duty=float(parsed["applied_right_duty"]),
                    left_count=int(float(parsed["left_count"])),
                    right_count=int(float(parsed["right_count"])),
                    irq_per_s=int(float(parsed["irq_per_s"])),
                    sample_tick=sample_tick,
                )
            )

    return ser, rows


def collect_phase_samples(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    phase: Phase,
    duration_s: float,
    sample_period_s: float,
) -> tuple[serial.Serial, list[SampleRow]]:
    deadline = time.monotonic() + duration_s
    phase_start = time.monotonic()
    next_sample = phase_start
    rows: list[SampleRow] = []

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_sample:
            write_command(ser, "auto,sample")
            next_sample = now + sample_period_s

        read_window_s = min(0.10, max(0.02, next_sample - time.monotonic()))
        ser, text = read_serial_text(ser, port, baud, raw_handle, read_window_s)

        parsed_rows = parse_auto_samples(text)
        if not parsed_rows:
            parsed_rows = parse_human_statuses(text)

        for parsed in parsed_rows:
            sample_tick = int(float(parsed["sample_tick"]))
            board_ms = int(float(parsed.get("board_ms", sample_tick * 10)))
            rows.append(
                SampleRow(
                    host_time_iso=dt.datetime.now().isoformat(timespec="milliseconds"),
                    phase_name=phase.name,
                    phase_kind=phase.kind,
                    wheel_under_test=phase.wheel,
                    phase_elapsed_s=time.monotonic() - phase_start,
                    board_ms=board_ms,
                    board_mode=parsed["board_mode"],
                    cmd_left_duty=float(parsed["cmd_left_duty"]),
                    cmd_right_duty=float(parsed["cmd_right_duty"]),
                    cmd_left_speed=float(parsed["cmd_left_speed"]),
                    cmd_right_speed=float(parsed["cmd_right_speed"]),
                    target_left_speed=float(parsed["target_left_speed"]),
                    target_right_speed=float(parsed["target_right_speed"]),
                    measured_left_speed=float(parsed["measured_left_speed"]),
                    measured_right_speed=float(parsed["measured_right_speed"]),
                    applied_left_duty=float(parsed["applied_left_duty"]),
                    applied_right_duty=float(parsed["applied_right_duty"]),
                    left_count=int(float(parsed["left_count"])),
                    right_count=int(float(parsed["right_count"])),
                    irq_per_s=int(float(parsed["irq_per_s"])),
                    sample_tick=sample_tick,
                )
            )

    ser, _ = read_serial_text(ser, port, baud, raw_handle, 0.10)
    return ser, rows


def make_open_loop_phases(duty_steps: Iterable[float], duty_phase_s: float, idle_phase_s: float) -> list[Phase]:
    phases = [Phase("idle_start", "idle", "none", 0.0, 0.0, idle_phase_s)]
    for step in duty_steps:
        phases.append(Phase(f"left_{int(step):02d}", "open_loop", "left", float(step), 0.0, duty_phase_s))
        phases.append(Phase(f"idle_l_{int(step):02d}", "idle", "none", 0.0, 0.0, idle_phase_s))
    for step in duty_steps:
        phases.append(Phase(f"right_{int(step):02d}", "open_loop", "right", 0.0, float(step), duty_phase_s))
        phases.append(Phase(f"idle_r_{int(step):02d}", "idle", "none", 0.0, 0.0, idle_phase_s))
    phases.append(Phase("idle_end", "idle", "none", 0.0, 0.0, idle_phase_s))
    return phases


def tail_rows(rows: list[SampleRow]) -> list[SampleRow]:
    if not rows:
        return []
    start = max(0, len(rows) // 2)
    return rows[start:]


def select_phase_rows(rows: list[SampleRow], phase_name: str) -> list[SampleRow]:
    return [row for row in rows if row.phase_name == phase_name]


def summarize_open_loop_rows(
    rows: list[SampleRow],
    wheel: str,
    moving_threshold: float,
    verify_high_fraction: float,
    verify_high_cap_mps: float,
) -> WheelFit:
    wheel_rows = [
        row for row in rows
        if row.phase_kind == "open_loop" and row.wheel_under_test == wheel
    ]
    if not wheel_rows:
        raise RuntimeError(f"No open-loop rows collected for {wheel}")

    grouped: dict[str, list[SampleRow]] = defaultdict(list)
    for row in wheel_rows:
        grouped[row.phase_name].append(row)

    steady_points: list[tuple[float, float, str]] = []
    best_tau_rows: list[SampleRow] = []
    best_tau_speed = -1.0
    sign_votes: list[float] = []

    for phase_name, phase_rows in grouped.items():
        tail = tail_rows(phase_rows)
        if not tail:
            continue

        if wheel == "left":
            cmd = statistics.fmean(abs(row.cmd_left_duty) for row in tail)
            speed = statistics.fmean(abs(row.measured_left_speed) for row in tail)
            signed_speed = statistics.fmean(row.measured_left_speed for row in tail)
        else:
            cmd = statistics.fmean(abs(row.cmd_right_duty) for row in tail)
            speed = statistics.fmean(abs(row.measured_right_speed) for row in tail)
            signed_speed = statistics.fmean(row.measured_right_speed for row in tail)

        if cmd <= 1e-6:
            continue

        steady_points.append((cmd, speed, phase_name))
        if speed > best_tau_speed:
            best_tau_speed = speed
            best_tau_rows = phase_rows
        if abs(signed_speed) >= moving_threshold:
            sign_votes.append(math.copysign(1.0, signed_speed))

    moving_points = [point for point in steady_points if point[1] >= moving_threshold]
    if not moving_points:
        raise RuntimeError(f"{wheel}: no moving points found above {moving_threshold:.3f} m/s")

    slope_num = sum(cmd * speed for cmd, speed, _ in moving_points)
    slope_den = sum(cmd * cmd for cmd, _, _ in moving_points)
    slope_speed_per_duty = slope_num / slope_den
    ff_gain = 1.0 / slope_speed_per_duty
    min_moving_duty = min(cmd for cmd, _, _ in moving_points)
    max_speed_mps = max(speed for _, speed, _ in moving_points)

    if sign_votes:
        sign = 1.0 if sum(sign_votes) >= 0.0 else -1.0
    else:
        sign = 1.0

    tau_s = estimate_tau(best_tau_rows, wheel)
    tau_s = max(0.08, tau_s)
    lam = max(4.0 * tau_s, 0.35)
    kp = clamp(ff_gain * tau_s / lam, 0.05, 3.0)
    ki = clamp(kp / tau_s, 0.10, 12.0)
    kd = 0.0

    target_low_mps = max(0.08, min(0.20, max_speed_mps * 0.35))
    target_mid_mps = max(target_low_mps + 0.03, min(0.45, max_speed_mps * 0.60))
    high_fraction = clamp(verify_high_fraction, 0.65, 0.98)
    target_high_mps = clamp(max_speed_mps * high_fraction,
                            target_mid_mps + 0.05,
                            verify_high_cap_mps)

    return WheelFit(
        wheel=wheel,
        sign=sign,
        min_moving_duty=min_moving_duty,
        max_speed_mps=max_speed_mps,
        slope_speed_per_duty=slope_speed_per_duty,
        ff_gain=ff_gain,
        tau_s=tau_s,
        kp=kp,
        ki=ki,
        kd=kd,
        target_low_mps=target_low_mps,
        target_mid_mps=target_mid_mps,
        target_high_mps=target_high_mps,
    )


def estimate_tau(rows: list[SampleRow], wheel: str) -> float:
    if not rows:
        return 0.20

    tail = tail_rows(rows)
    if wheel == "left":
        steady = statistics.fmean(abs(row.measured_left_speed) for row in tail)
    else:
        steady = statistics.fmean(abs(row.measured_right_speed) for row in tail)

    if steady <= 1e-6:
        return 0.20

    threshold = steady * 0.632
    for row in rows:
        speed = abs(row.measured_left_speed) if wheel == "left" else abs(row.measured_right_speed)
        if speed >= threshold:
            return max(0.05, row.phase_elapsed_s)

    return max(0.20, rows[-1].phase_elapsed_s * 0.5)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return min(max(value, min_value), max_value)


def make_verify_phases(left_fit: WheelFit, right_fit: WheelFit, verify_phase_s: float) -> list[Phase]:
    return [
        Phase("verify_idle_0", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_left_low", "verify_speed", "left", left_fit.sign * left_fit.target_low_mps, 0.0, verify_phase_s),
        Phase("verify_idle_1", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_left_mid", "verify_speed", "left", left_fit.sign * left_fit.target_mid_mps, 0.0, verify_phase_s),
        Phase("verify_idle_2", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_left_high", "verify_speed", "left", left_fit.sign * left_fit.target_high_mps, 0.0, verify_phase_s),
        Phase("verify_idle_3", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_right_low", "verify_speed", "right", 0.0, right_fit.sign * right_fit.target_low_mps, verify_phase_s),
        Phase("verify_idle_4", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_right_mid", "verify_speed", "right", 0.0, right_fit.sign * right_fit.target_mid_mps, verify_phase_s),
        Phase("verify_idle_5", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase("verify_right_high", "verify_speed", "right", 0.0, right_fit.sign * right_fit.target_high_mps, verify_phase_s),
        Phase("verify_idle_6", "verify_idle", "none", 0.0, 0.0, 1.0),
        Phase(
            "verify_both_high",
            "verify_speed",
            "both",
            left_fit.sign * left_fit.target_high_mps,
            right_fit.sign * right_fit.target_high_mps,
            verify_phase_s,
        ),
        Phase("verify_idle_end", "verify_idle", "none", 0.0, 0.0, 1.0),
    ]


def write_csv(rows: list[SampleRow], csv_path: pathlib.Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "host_time_iso",
                "phase_name",
                "phase_kind",
                "wheel_under_test",
                "phase_elapsed_s",
                "board_ms",
                "board_mode",
                "cmd_left_duty",
                "cmd_right_duty",
                "cmd_left_speed",
                "cmd_right_speed",
                "target_left_speed",
                "target_right_speed",
                "measured_left_speed",
                "measured_right_speed",
                "applied_left_duty",
                "applied_right_duty",
                "left_count",
                "right_count",
                "irq_per_s",
                "sample_tick",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(dataclasses.asdict(row))


def summarize_verify(rows: list[SampleRow], phase_name: str, wheel: str) -> dict[str, float | str]:
    phase_rows = select_phase_rows(rows, phase_name)
    tail = tail_rows(phase_rows)
    if not tail:
        return {"phase": phase_name, "wheel": wheel, "samples": 0}

    if wheel == "left":
        target = statistics.fmean(row.target_left_speed for row in tail)
        measured = statistics.fmean(row.measured_left_speed for row in tail)
        duty = statistics.fmean(row.applied_left_duty for row in tail)
    elif wheel == "right":
        target = statistics.fmean(row.target_right_speed for row in tail)
        measured = statistics.fmean(row.measured_right_speed for row in tail)
        duty = statistics.fmean(row.applied_right_duty for row in tail)
    else:
        target = statistics.fmean((row.target_left_speed + row.target_right_speed) * 0.5 for row in tail)
        measured = statistics.fmean((row.measured_left_speed + row.measured_right_speed) * 0.5 for row in tail)
        duty = statistics.fmean((row.applied_left_duty + row.applied_right_duty) * 0.5 for row in tail)

    error = target - measured
    return {
        "phase": phase_name,
        "wheel": wheel,
        "samples": len(phase_rows),
        "target": round(target, 6),
        "measured": round(measured, 6),
        "error": round(error, 6),
        "applied_duty": round(duty, 6),
    }


def run_phase_sequence(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    phases: Iterable[Phase],
    command_gap_s: float,
    speed_mode: bool,
    sample_period_s: float,
) -> tuple[serial.Serial, list[SampleRow]]:
    rows: list[SampleRow] = []

    for phase in phases:
        ser = send_command_until_ack(
            ser,
            port,
            baud,
            raw_handle,
            f"auto,phase,{phase.name}",
            (f"auto,ack,phase,{phase.name}",),
        )
        time.sleep(command_gap_s)

        if speed_mode:
            ser = send_command_until_ack(
                ser,
                port,
                baud,
                raw_handle,
                f"auto,spd,{phase.left_cmd:.6f},{phase.right_cmd:.6f}",
                ("auto,ack,spd,",),
            )
        else:
            ser = send_command_until_ack(
                ser,
                port,
                baud,
                raw_handle,
                f"auto,duty,{phase.left_cmd:.3f},{phase.right_cmd:.3f}",
                ("auto,ack,duty,",),
            )
        time.sleep(command_gap_s)
        ser, phase_rows = collect_phase_samples(
            ser,
            port,
            baud,
            raw_handle,
            phase,
            phase.duration_s,
            sample_period_s,
        )
        rows.extend(phase_rows)

    ser = send_command_until_ack(
        ser,
        port,
        baud,
        raw_handle,
        "auto,stop",
        ("auto,ack,stop",),
    )
    time.sleep(command_gap_s)
    ser, tail_rows = drain_serial(ser, port, baud, raw_handle, 0.5, phase=None)
    rows.extend(tail_rows)
    return ser, rows


def request_pid_dump(ser: serial.Serial, port: str, baud: int, raw_handle) -> serial.Serial:
    ser, _ = try_send_command_until_ack(
        ser,
        port,
        baud,
        raw_handle,
        "auto,pid",
        ("auto,pid,left,", "auto,pid,right,"),
        timeout_s=2.5,
    )
    return ser


def apply_wheel_pid(
    ser: serial.Serial,
    port: str,
    baud: int,
    raw_handle,
    fit: WheelFit,
) -> serial.Serial:
    ser, ok = try_send_command_until_ack(
        ser,
        port,
        baud,
        raw_handle,
        f"auto,pid,{fit.wheel},{fit.kp:.6f},{fit.ki:.6f},{fit.kd:.6f},{fit.ff_gain:.6f}",
        (f"auto,ack,pid,{fit.wheel}", f"auto,pid,{fit.wheel},"),
        timeout_s=2.5,
    )
    if not ok:
        raise RuntimeError(f"Failed to apply PID config for {fit.wheel}")
    return ser


def save_summary(
    summary_path: pathlib.Path,
    left_fit: WheelFit,
    right_fit: WheelFit,
    verify_rows: list[SampleRow],
) -> None:
    verify_summary = [
        summarize_verify(verify_rows, "verify_left_low", "left"),
        summarize_verify(verify_rows, "verify_left_mid", "left"),
        summarize_verify(verify_rows, "verify_left_high", "left"),
        summarize_verify(verify_rows, "verify_right_low", "right"),
        summarize_verify(verify_rows, "verify_right_mid", "right"),
        summarize_verify(verify_rows, "verify_right_high", "right"),
        summarize_verify(verify_rows, "verify_both_high", "both"),
    ]

    payload = {
        "timestamp": dt.datetime.now().isoformat(timespec="seconds"),
        "left": dataclasses.asdict(left_fit),
        "right": dataclasses.asdict(right_fit),
        "verify": verify_summary,
    }
    summary_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_steps(text: str) -> list[float]:
    steps = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        steps.append(float(part))
    if not steps:
        raise ValueError("No duty steps specified")
    return steps


def main() -> int:
    args = build_arg_parser().parse_args()
    repo_root = args.repo_root.resolve()
    output_dir = (repo_root / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = output_dir / f"{RAW_LOG_PREFIX}{stamp}.log"
    csv_path = output_dir / f"{CSV_PREFIX}{stamp}.csv"
    summary_path = output_dir / f"{SUMMARY_PREFIX}{stamp}.json"

    duty_steps = parse_steps(args.duty_steps)
    port = args.port or detect_port()

    if args.flash:
        flash_firmware(repo_root, args.flash_command_file)
    elif args.reset_before_start:
        reset_target(repo_root)

    print(f"[serial] opening {port} @ {args.baud}")
    ser = open_serial(port, args.baud)

    try:
        with raw_path.open("w", encoding="utf-8") as raw_handle:
            print(f"[boot] settling for {args.boot_settle_s:.1f}s")
            ser, _ = drain_serial(
                ser,
                port,
                args.baud,
                raw_handle,
                args.boot_settle_s,
                phase=None,
            )

            ser = wait_for_board_ready(ser, port, args.baud, raw_handle, args.ready_timeout_s)

            ser = send_command_until_ack(
                ser,
                port,
                args.baud,
                raw_handle,
                "auto,off",
                ("auto,ack,off",),
            )
            time.sleep(args.command_gap_s)

            ser = send_command_until_ack(
                ser,
                port,
                args.baud,
                raw_handle,
                "stop",
                ("cmd=stop", "mode=STOP"),
            )
            time.sleep(args.command_gap_s)

            ser = request_pid_dump(ser, port, args.baud, raw_handle)

            open_loop_phases = make_open_loop_phases(
                duty_steps,
                args.duty_phase_s,
                args.idle_phase_s,
            )
            ser, open_loop_rows = run_phase_sequence(
                ser,
                port,
                args.baud,
                raw_handle,
                open_loop_phases,
                args.command_gap_s,
                speed_mode=False,
                sample_period_s=args.sample_period_s,
            )

            left_fit = summarize_open_loop_rows(
                open_loop_rows,
                "left",
                args.moving_threshold,
                args.verify_high_fraction,
                args.verify_high_cap_mps,
            )
            right_fit = summarize_open_loop_rows(
                open_loop_rows,
                "right",
                args.moving_threshold,
                args.verify_high_fraction,
                args.verify_high_cap_mps,
            )

            print("[fit] left ", dataclasses.asdict(left_fit))
            print("[fit] right", dataclasses.asdict(right_fit))

            ser = apply_wheel_pid(ser, port, args.baud, raw_handle, left_fit)
            time.sleep(args.command_gap_s)
            ser = apply_wheel_pid(ser, port, args.baud, raw_handle, right_fit)
            time.sleep(args.command_gap_s)
            ser = request_pid_dump(ser, port, args.baud, raw_handle)

            verify_phases = make_verify_phases(left_fit, right_fit, args.verify_phase_s)
            ser, verify_rows = run_phase_sequence(
                ser,
                port,
                args.baud,
                raw_handle,
                verify_phases,
                args.command_gap_s,
                speed_mode=True,
                sample_period_s=args.sample_period_s,
            )

            ser = send_command_until_ack(
                ser,
                port,
                args.baud,
                raw_handle,
                "auto,off",
                ("auto,ack,off",),
            )
            time.sleep(args.command_gap_s)

        all_rows = open_loop_rows + verify_rows
        write_csv(all_rows, csv_path)
        save_summary(summary_path, left_fit, right_fit, verify_rows)

        print(f"[done] raw log : {raw_path}")
        print(f"[done] csv     : {csv_path}")
        print(f"[done] summary : {summary_path}")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
