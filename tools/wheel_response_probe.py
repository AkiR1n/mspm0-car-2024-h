#!/usr/bin/env python3
"""Short, non-tuning wheel and twist response probe over UART."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import pathlib
import re
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any

try:
    import serial
except ImportError as exc:  # pragma: no cover
    print(f"Missing dependency: pyserial ({exc})", file=sys.stderr)
    raise SystemExit(2)


PAIR_RE = re.compile(r"(?P<name>\w+)=\((?P<values>[^)]*)\)")
FIELD_RE = re.compile(r"(?P<name>[A-Za-z_][\w/]*?)=(?P<value>[^\s]+)")

CSV_COLUMNS = [
    "host_time_s",
    "elapsed_s",
    "phase",
    "command",
    "mode",
    "v_cmd",
    "w_cmd",
    "target_left",
    "target_right",
    "measured_left",
    "measured_right",
    "applied_left_duty",
    "applied_right_duty",
    "left_count",
    "right_count",
    "yaw_deg",
    "gyro_z",
    "line_bits",
    "line_detected",
    "line_position",
    "raw_line",
]


@dataclass(frozen=True)
class Phase:
    name: str
    command: str
    duration_s: float


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output-dir", type=pathlib.Path, default=pathlib.Path("logs/wheel_probe"))
    parser.add_argument("--boot-settle-s", type=float, default=0.6)
    parser.add_argument("--sample-timeout-s", type=float, default=0.25)
    parser.add_argument("--no-stop", action="store_true")
    parser.add_argument(
        "--phase",
        action="append",
        help="Custom phase as name:command:duration_s, e.g. left:spd,0.12,0:1.5",
    )
    return parser


def default_phases() -> list[Phase]:
    return [
        Phase("idle_start", "stop", 0.8),
        Phase("left_spd_012", "spd,0.120,0.000", 1.8),
        Phase("idle_after_left", "stop", 0.6),
        Phase("right_spd_012", "spd,0.000,0.120", 1.8),
        Phase("idle_after_right", "stop", 0.6),
        Phase("both_spd_018", "spd,0.180,0.180", 1.8),
        Phase("idle_after_both", "stop", 0.6),
        Phase("twist_pos_w", "twist,0.120,0.500", 1.4),
        Phase("idle_after_twist_pos", "stop", 0.6),
        Phase("twist_neg_w", "twist,0.120,-0.500", 1.4),
        Phase("idle_end", "stop", 0.8),
    ]


def parse_phase(text: str) -> Phase:
    parts = text.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("phase must be name:command:duration_s")
    name, command, duration = parts
    try:
        duration_s = float(duration)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid phase duration: {duration}") from exc
    if duration_s <= 0.0:
        raise argparse.ArgumentTypeError("phase duration must be > 0")
    return Phase(name.strip(), command.strip(), duration_s)


def to_float(text: str | None, default: float = 0.0) -> float:
    if text is None or text == "":
        return default
    try:
        return float(text)
    except ValueError:
        return default


def to_int(text: str | None, default: int = 0) -> int:
    if text is None or text == "":
        return default
    try:
        return int(text, 0)
    except ValueError:
        try:
            return int(float(text))
        except ValueError:
            return default


def parse_pair(text: str) -> list[float]:
    return [to_float(part.strip()) for part in text.split(",")]


def write_command(ser: serial.Serial, command: str) -> None:
    ser.write((command.strip() + "\n").encode("utf-8"))
    ser.flush()


def parse_line(line: str, phase: Phase, start_s: float) -> dict[str, Any] | None:
    if "mode=" not in line:
        return None

    fields = {match.group("name"): match.group("value") for match in FIELD_RE.finditer(line)}
    row: dict[str, Any] = {name: "" for name in CSV_COLUMNS}
    row["host_time_s"] = time.time()
    row["elapsed_s"] = time.monotonic() - start_s
    row["phase"] = phase.name
    row["command"] = phase.command
    row["mode"] = fields.get("mode", "")
    row["yaw_deg"] = to_float(fields.get("yaw"), 0.0)
    row["gyro_z"] = to_float(fields.get("gz"), 0.0)
    row["line_bits"] = to_int(fields.get("bits"), 0)
    row["line_detected"] = to_int(fields.get("det"), 0)
    row["line_position"] = to_int(fields.get("pos"), 0)
    row["raw_line"] = line

    for match in PAIR_RE.finditer(line):
        name = match.group("name")
        values = parse_pair(match.group("values"))
        if name == "spd" and len(values) >= 2:
            row["target_left"] = values[0]
            row["target_right"] = values[1]
        elif name == "vw" and len(values) >= 2:
            row["v_cmd"] = values[0]
            row["w_cmd"] = values[1]
        elif name == "target" and len(values) >= 2:
            row["target_left"] = values[0]
            row["target_right"] = values[1]
        elif name == "meas" and len(values) >= 2:
            row["measured_left"] = values[0]
            row["measured_right"] = values[1]
        elif name == "duty" and len(values) >= 2:
            row["applied_left_duty"] = values[0]
            row["applied_right_duty"] = values[1]
        elif name == "count" and len(values) >= 2:
            row["left_count"] = int(values[0])
            row["right_count"] = int(values[1])

    return row


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def tail(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not rows:
        return []
    return rows[max(0, len(rows) // 2):]


def summarize(rows: list[dict[str, Any]], phases: list[Phase]) -> dict[str, Any]:
    result: dict[str, Any] = {"phases": []}
    for phase in phases:
        phase_rows = tail([row for row in rows if row["phase"] == phase.name])
        if not phase_rows:
            result["phases"].append({"phase": phase.name, "samples": 0})
            continue
        first = phase_rows[0]
        last = phase_rows[-1]
        yaw_delta = to_float(last.get("yaw_deg")) - to_float(first.get("yaw_deg"))
        item = {
            "phase": phase.name,
            "command": phase.command,
            "samples": len(phase_rows),
            "mode": phase_rows[-1].get("mode", ""),
            "target_left_mean": mean([to_float(row.get("target_left")) for row in phase_rows]),
            "target_right_mean": mean([to_float(row.get("target_right")) for row in phase_rows]),
            "measured_left_mean": mean([to_float(row.get("measured_left")) for row in phase_rows]),
            "measured_right_mean": mean([to_float(row.get("measured_right")) for row in phase_rows]),
            "duty_left_mean": mean([to_float(row.get("applied_left_duty")) for row in phase_rows]),
            "duty_right_mean": mean([to_float(row.get("applied_right_duty")) for row in phase_rows]),
            "gyro_z_mean": mean([to_float(row.get("gyro_z")) for row in phase_rows]),
            "yaw_delta_deg_tail": yaw_delta,
        }
        item["target_diff_mean"] = item["target_right_mean"] - item["target_left_mean"]
        item["measured_diff_mean"] = item["measured_right_mean"] - item["measured_left_mean"]
        result["phases"].append(item)
    return result


def main() -> int:
    args = build_arg_parser().parse_args()
    phases = [parse_phase(text) for text in args.phase] if args.phase else default_phases()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = args.output_dir / f"wheel_probe_raw_{stamp}.log"
    csv_path = args.output_dir / f"wheel_probe_data_{stamp}.csv"
    summary_path = args.output_dir / f"wheel_probe_summary_{stamp}.json"

    rows: list[dict[str, Any]] = []
    start_s = time.monotonic()
    print(f"[open] {args.port} @ {args.baud}")
    print(f"[raw] {raw_path}")

    with serial.Serial(args.port, args.baud, timeout=args.sample_timeout_s) as ser, \
            raw_path.open("w", encoding="utf-8") as raw:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(args.boot_settle_s)
        write_command(ser, "stop")

        for phase in phases:
            print(f"[phase] {phase.name}: {phase.command} ({phase.duration_s:.2f}s)")
            write_command(ser, phase.command)
            deadline = time.monotonic() + phase.duration_s
            while time.monotonic() < deadline:
                data = ser.readline()
                if not data:
                    continue
                line = data.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                raw.write(line + "\n")
                raw.flush()
                parsed = parse_line(line, phase, start_s)
                if parsed is not None:
                    rows.append(parsed)

        if not args.no_stop:
            write_command(ser, "stop")
            time.sleep(0.1)

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows, phases)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
                            encoding="utf-8")

    print(f"[csv] {csv_path}")
    print(f"[summary] {summary_path}")
    for item in summary["phases"]:
        print(json.dumps(item, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
