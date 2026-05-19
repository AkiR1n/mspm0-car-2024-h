#!/usr/bin/env python3
"""Collect serial telemetry to diagnose IMU-only straight steering.

The script sends a controlled command such as ``straight,dist,speed`` or
``q2``/``run``, records the board status stream, and computes simple sign and
response checks:

- heading error -> commanded w
- commanded w -> left/right wheel target difference
- wheel target difference -> measured wheel speed difference
- commanded w -> IMU gyro/yaw-rate response

It intentionally does not infer lateral drift from IMU data. If yaw error is
small but the vehicle still drifts sideways, that is a position/traction issue
that yaw-only control cannot observe.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import json
import math
import pathlib
import re
import statistics
import sys
import time
from typing import Any

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


PAIR_RE = re.compile(r"(?P<name>\w+)=\((?P<values>[^)]*)\)")
FIELD_RE = re.compile(r"(?P<name>[A-Za-z_][\w/]*?)=(?P<value>[^\s]+)")
DIST_RE = re.compile(r"dist=(?P<now>-?\d+(?:\.\d+)?)/(?P<target>-?\d+(?:\.\d+)?)")

CSV_COLUMNS = [
    "host_time_s",
    "elapsed_s",
    "mode",
    "selected_challenge",
    "active_challenge",
    "phase",
    "action",
    "state",
    "lap_index",
    "lap_total",
    "checkpoint_count",
    "event",
    "event_ms",
    "distance_m",
    "target_distance_m",
    "geometry_heading_deg",
    "hold_heading_deg",
    "heading_error_deg",
    "v_mps",
    "w_radps",
    "target_left_speed",
    "target_right_speed",
    "measured_left_speed",
    "measured_right_speed",
    "applied_left_duty",
    "applied_right_duty",
    "left_count",
    "right_count",
    "target_speed_diff",
    "measured_speed_diff",
    "predicted_w_from_targets",
    "yaw_rate_degps",
    "line",
    "line_bits",
    "line_detected",
    "line_position",
    "imu_ready",
    "imu_stable",
    "yaw_deg",
    "gyro_z",
    "imu_up_ms",
    "irq_per_s",
    "tick",
    "raw_line",
]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--case",
        choices=("straight", "q2", "listen", "raw"),
        default="straight",
        help="Test case to run",
    )
    parser.add_argument("--distance", type=float, default=1.0, help="Straight distance in m")
    parser.add_argument("--speed", type=float, default=0.25, help="Straight speed in m/s")
    parser.add_argument("--timeout", type=float, help="Collection timeout in seconds")
    parser.add_argument("--pre-idle", type=float, default=1.0, help="Idle capture before command")
    parser.add_argument("--post-stop", type=float, default=1.0, help="Capture after STOP appears")
    parser.add_argument("--wheel-base", type=float, default=0.14, help="Wheel base in m")
    parser.add_argument(
        "--raw-command",
        action="append",
        default=[],
        help="Raw command for --case raw; can be specified multiple times",
    )
    parser.add_argument(
        "--output-dir",
        default="logs/imu_straight",
        type=pathlib.Path,
        help="Directory for raw, CSV, JSON summary and text report",
    )
    parser.add_argument("--note", default="", help="Optional operator note stored in summary")
    parser.add_argument("--no-stop", action="store_true", help="Do not send stop at exit")
    return parser


def choose_default_port() -> str | None:
    for preferred in ("/dev/ttyACM1", "/dev/ttyACM0"):
        if pathlib.Path(preferred).exists():
            return preferred
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(value: Any, default: int = 0) -> int:
    try:
        if isinstance(value, str) and value.lower().startswith("0x"):
            return int(value, 0)
        return int(float(value))
    except (TypeError, ValueError):
        return default


def wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def mean_abs(values: list[float]) -> float:
    return mean([abs(v) for v in values])


def max_abs(values: list[float]) -> float:
    return max((abs(v) for v in values), default=0.0)


def corr(xs: list[float], ys: list[float]) -> float | None:
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return None
    x_vals = [p[0] for p in pairs]
    y_vals = [p[1] for p in pairs]
    x_mean = mean(x_vals)
    y_mean = mean(y_vals)
    x_var = sum((x - x_mean) ** 2 for x in x_vals)
    y_var = sum((y - y_mean) ** 2 for y in y_vals)
    if x_var <= 1e-12 or y_var <= 1e-12:
        return None
    cov = sum((x - x_mean) * (y - y_mean) for x, y in pairs)
    return cov / math.sqrt(x_var * y_var)


def parse_pair_values(text: str) -> list[float]:
    return [to_float(part.strip()) for part in text.split(",")]


def parse_status_line(line: str, host_time_s: float, elapsed_s: float) -> dict[str, Any] | None:
    if "mode=" not in line:
        return None

    fields = {m.group("name"): m.group("value") for m in FIELD_RE.finditer(line)}
    row: dict[str, Any] = {name: "" for name in CSV_COLUMNS}
    row["host_time_s"] = host_time_s
    row["elapsed_s"] = elapsed_s
    row["raw_line"] = line

    row["mode"] = fields.get("mode", "")
    row["phase"] = fields.get("ph", fields.get("phase", ""))
    row["action"] = fields.get("act", "")
    row["state"] = fields.get("state", "")
    row["checkpoint_count"] = to_int(fields.get("cp"), 0)
    row["line"] = fields.get("line", "")
    row["line_bits"] = to_int(fields.get("bits"), 0)
    row["line_detected"] = to_int(fields.get("det"), 0)
    row["line_position"] = to_int(fields.get("pos"), 0)
    row["yaw_deg"] = to_float(fields.get("yaw"), 0.0)
    row["gyro_z"] = to_float(fields.get("gz"), 0.0)
    row["imu_up_ms"] = to_int(fields.get("up"), 0)
    row["irq_per_s"] = to_int(fields.get("irq/s"), 0)
    row["tick"] = to_int(fields.get("tick"), 0)

    if "q" in fields:
        selected, _, active = fields["q"].partition("/")
        row["selected_challenge"] = selected
        row["active_challenge"] = active
    if "lap" in fields:
        lap_index, _, lap_total = fields["lap"].partition("/")
        row["lap_index"] = to_int(lap_index, 0)
        row["lap_total"] = to_int(lap_total, 0)
    if "evt" in fields:
        event, _, event_ms = fields["evt"].partition("@")
        row["event"] = event
        row["event_ms"] = to_int(event_ms, 0)

    dist_match = DIST_RE.search(line)
    if dist_match:
        row["distance_m"] = to_float(dist_match.group("now"), 0.0)
        row["target_distance_m"] = to_float(dist_match.group("target"), 0.0)
    elif "dist" in fields:
        row["distance_m"] = to_float(fields["dist"], 0.0)

    for match in PAIR_RE.finditer(line):
        name = match.group("name")
        values = parse_pair_values(match.group("values"))
        if name == "hdg" and len(values) >= 3:
            row["geometry_heading_deg"] = values[0]
            row["hold_heading_deg"] = values[1]
            row["heading_error_deg"] = values[2]
        elif name == "vw" and len(values) >= 2:
            row["v_mps"] = values[0]
            row["w_radps"] = values[1]
        elif name == "target" and len(values) >= 2:
            row["target_left_speed"] = values[0]
            row["target_right_speed"] = values[1]
        elif name == "meas" and len(values) >= 2:
            row["measured_left_speed"] = values[0]
            row["measured_right_speed"] = values[1]
        elif name == "duty" and len(values) >= 2:
            row["applied_left_duty"] = values[0]
            row["applied_right_duty"] = values[1]
        elif name == "count" and len(values) >= 2:
            row["left_count"] = int(values[0])
            row["right_count"] = int(values[1])
        elif name == "imu" and len(values) >= 2:
            row["imu_ready"] = int(values[0])
            row["imu_stable"] = int(values[1])

    return row


def enrich_rows(rows: list[dict[str, Any]], wheel_base_m: float) -> None:
    prev: dict[str, Any] | None = None
    for row in rows:
        target_left = to_float(row.get("target_left_speed"))
        target_right = to_float(row.get("target_right_speed"))
        meas_left = to_float(row.get("measured_left_speed"))
        meas_right = to_float(row.get("measured_right_speed"))
        target_diff = target_right - target_left
        measured_diff = meas_right - meas_left

        row["target_speed_diff"] = target_diff
        row["measured_speed_diff"] = measured_diff
        row["predicted_w_from_targets"] = (
            target_diff / wheel_base_m if wheel_base_m > 1e-6 else 0.0
        )

        row["yaw_rate_degps"] = 0.0
        if prev is not None:
            dt_s = to_float(row["elapsed_s"]) - to_float(prev["elapsed_s"])
            if dt_s > 1e-4:
                row["yaw_rate_degps"] = wrap_deg(to_float(row["yaw_deg"]) - to_float(prev["yaw_deg"])) / dt_s
        prev = row


def read_serial_line(ser: serial.Serial, deadline_s: float) -> bytes | None:
    buf = bytearray()
    while time.monotonic() < deadline_s:
        chunk = ser.read(1)
        if not chunk:
            if buf:
                return bytes(buf)
            continue
        if chunk in (b"\n", b"\r"):
            if buf:
                return bytes(buf)
            continue
        buf.extend(chunk)
        if len(buf) >= 4096:
            return bytes(buf)
    if buf:
        return bytes(buf)
    return None


def write_command(ser: serial.Serial, command: str) -> None:
    print(f"[tx] {command}")
    ser.write((command.strip() + "\n").encode("utf-8"))
    ser.flush()


def command_sequence(args: argparse.Namespace) -> list[str]:
    if args.case == "straight":
        return [f"straight,{args.distance:.3f},{args.speed:.3f}"]
    if args.case == "q2":
        return ["q2", "run"]
    if args.case == "raw":
        return args.raw_command
    return []


def default_timeout(args: argparse.Namespace) -> float:
    if args.timeout is not None:
        return args.timeout
    if args.case == "q2":
        return 35.0
    if args.case == "straight":
        return max(6.0, args.distance / max(args.speed, 0.05) + 4.0)
    return 10.0


def collect(args: argparse.Namespace) -> tuple[list[str], list[dict[str, Any]]]:
    port = args.port or choose_default_port()
    if not port:
        raise RuntimeError("No serial port found. Pass --port /dev/ttyACM1.")

    timeout_s = default_timeout(args)
    raw_lines: list[str] = []
    rows: list[dict[str, Any]] = []
    commands = command_sequence(args)
    start_s = time.monotonic()
    deadline_s = start_s + args.pre_idle + timeout_s
    command_sent = False
    saw_active = False
    stop_seen_at: float | None = None

    print(f"[open] {port} @ {args.baud}")
    with serial.Serial(port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while time.monotonic() < deadline_s:
            now_s = time.monotonic()
            elapsed_s = now_s - start_s
            if (not command_sent) and elapsed_s >= args.pre_idle:
                for cmd in commands:
                    write_command(ser, cmd)
                    time.sleep(0.12)
                command_sent = True

            raw = read_serial_line(ser, min(deadline_s, time.monotonic() + 0.2))
            if raw is None:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            raw_lines.append(line)
            row = parse_status_line(line, time.time(), time.monotonic() - start_s)
            if row is not None:
                rows.append(row)
                if row["mode"] not in ("", "STOP"):
                    saw_active = True
                if saw_active and row["mode"] == "STOP" and stop_seen_at is None:
                    stop_seen_at = time.monotonic()

            if stop_seen_at is not None and (time.monotonic() - stop_seen_at) >= args.post_stop:
                break

        if not args.no_stop and args.case != "listen":
            write_command(ser, "stop")
            time.sleep(0.1)

    enrich_rows(rows, args.wheel_base)
    return raw_lines, rows


def numeric(rows: list[dict[str, Any]], key: str) -> list[float]:
    return [to_float(row.get(key)) for row in rows if row.get(key) != ""]


def active_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        mode = str(row.get("mode", ""))
        if mode in ("MAIN", "STRAIGHT") or str(row.get("phase", "")).startswith("GAP"):
            result.append(row)
    return result or rows


def make_summary(args: argparse.Namespace, rows: list[dict[str, Any]]) -> dict[str, Any]:
    focus = active_rows(rows)
    err = numeric(focus, "heading_error_deg")
    w = numeric(focus, "w_radps")
    gz = numeric(focus, "gyro_z")
    yaw_rate = numeric(focus, "yaw_rate_degps")
    target_diff = numeric(focus, "target_speed_diff")
    meas_diff = numeric(focus, "measured_speed_diff")
    pred_w = numeric(focus, "predicted_w_from_targets")

    modes = sorted({str(row.get("mode", "")) for row in rows if row.get("mode")})
    phases = sorted({str(row.get("phase", "")) for row in rows if row.get("phase")})
    states = sorted({str(row.get("state", "")) for row in rows if row.get("state")})
    stable_rows = [
        row for row in focus
        if to_int(row.get("imu_ready")) != 0 and to_int(row.get("imu_stable")) != 0
    ]

    summary: dict[str, Any] = {
        "case": args.case,
        "note": args.note,
        "sample_count": len(rows),
        "focus_sample_count": len(focus),
        "imu_stable_sample_count": len(stable_rows),
        "modes": modes,
        "phases": phases,
        "states": states,
        "duration_s": (to_float(rows[-1]["elapsed_s"]) - to_float(rows[0]["elapsed_s"])) if rows else 0.0,
        "mean_abs_heading_error_deg": mean_abs(err),
        "max_abs_heading_error_deg": max_abs(err),
        "mean_abs_w_radps": mean_abs(w),
        "max_abs_w_radps": max_abs(w),
        "mean_abs_gyro_z_dps": mean_abs(gz),
        "max_abs_gyro_z_dps": max_abs(gz),
        "corr_heading_error_to_w": corr(err, w),
        "corr_w_to_target_speed_diff": corr(w, target_diff),
        "corr_w_to_predicted_w_from_targets": corr(w, pred_w),
        "corr_target_diff_to_measured_diff": corr(target_diff, meas_diff),
        "corr_w_to_gyro_z": corr(w, gz),
        "corr_w_to_yaw_rate": corr(w, yaw_rate),
        "final": rows[-1] if rows else {},
    }

    diagnostics: list[str] = []
    if not rows:
        diagnostics.append("No parseable status rows were collected.")
    if len(stable_rows) < max(5, len(focus) // 2):
        diagnostics.append("IMU was not ready/stable for most focus samples; yaw control may be bypassed.")
    if summary["max_abs_heading_error_deg"] > 4.0 and summary["mean_abs_w_radps"] < 0.05:
        diagnostics.append("Heading error exists but commanded w is very small; yaw PID gain may be too weak or disabled.")

    err_w = summary["corr_heading_error_to_w"]
    if err_w is not None and err_w < 0.2:
        diagnostics.append("heading_error and commanded w are weakly/negatively correlated; yaw PID sign or publishing path is suspect.")

    w_target = summary["corr_w_to_target_speed_diff"]
    if w_target is not None and w_target < 0.7:
        diagnostics.append("commanded w does not map cleanly to wheel target speed difference; v/w to wheel mapping or left/right mapping is suspect.")

    target_meas = summary["corr_target_diff_to_measured_diff"]
    if target_meas is not None and target_meas < 0.4:
        diagnostics.append("wheel measured speed difference does not follow target difference; speed loop, motor direction, or wheel response is suspect.")

    w_gz = summary["corr_w_to_gyro_z"]
    if w_gz is not None and w_gz < -0.3:
        diagnostics.append("positive commanded w tends to produce negative gyro_z; IMU yaw/gyro sign may oppose chassis turn sign.")

    if summary["mean_abs_heading_error_deg"] < 2.0 and args.note:
        diagnostics.append("If physical drift is visible while heading error stays small, yaw-only control cannot observe lateral translation; use line/position cue or fix mechanics.")
    elif summary["mean_abs_heading_error_deg"] < 2.0:
        diagnostics.append("Heading error is small. If the car still drifts sideways, the issue is likely lateral slip/mechanics rather than yaw control.")

    summary["diagnostics"] = diagnostics
    return summary


def write_outputs(args: argparse.Namespace, raw_lines: list[str], rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir: pathlib.Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / f"imu_straight_raw_{timestamp}.log"
    csv_path = output_dir / f"imu_straight_data_{timestamp}.csv"
    json_path = output_dir / f"imu_straight_summary_{timestamp}.json"
    report_path = output_dir / f"imu_straight_report_{timestamp}.txt"

    raw_path.write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    report_lines = [
        f"case: {summary['case']}",
        f"samples: {summary['sample_count']} focus={summary['focus_sample_count']} imu_stable={summary['imu_stable_sample_count']}",
        f"modes: {', '.join(summary['modes'])}",
        f"phases: {', '.join(summary['phases'])}",
        f"states: {', '.join(summary['states'])}",
        f"mean_abs_heading_error_deg: {summary['mean_abs_heading_error_deg']:.3f}",
        f"max_abs_heading_error_deg: {summary['max_abs_heading_error_deg']:.3f}",
        f"mean_abs_w_radps: {summary['mean_abs_w_radps']:.3f}",
        f"max_abs_w_radps: {summary['max_abs_w_radps']:.3f}",
        f"corr_heading_error_to_w: {summary['corr_heading_error_to_w']}",
        f"corr_w_to_target_speed_diff: {summary['corr_w_to_target_speed_diff']}",
        f"corr_target_diff_to_measured_diff: {summary['corr_target_diff_to_measured_diff']}",
        f"corr_w_to_gyro_z: {summary['corr_w_to_gyro_z']}",
        "",
        "diagnostics:",
    ]
    report_lines.extend(f"- {item}" for item in summary["diagnostics"])
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

    print(f"[raw]     {raw_path}")
    print(f"[csv]     {csv_path}")
    print(f"[summary] {json_path}")
    print(f"[report]  {report_path}")
    print()
    print("\n".join(report_lines))


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.case == "raw" and not args.raw_command:
        parser.error("--case raw requires at least one --raw-command")

    try:
        raw_lines, rows = collect(args)
        summary = make_summary(args, rows)
        write_outputs(args, raw_lines, rows, summary)
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
