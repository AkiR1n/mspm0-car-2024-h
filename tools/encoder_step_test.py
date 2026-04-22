#!/usr/bin/env python3
"""Automate open-loop wheel/encoder data collection over UART.

The firmware is expected to:
1. Accept wheel test commands as "left,right\\n"
2. Print CSV samples in the form:
   csv,<ms>,<left_cmd>,<right_cmd>,<left_duty>,<right_duty>,<left_mps>,<right_mps>,<left_count>,<right_count>,<irq_per_s>,<sample_tick>
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import pathlib
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List

try:
    import serial
except ImportError as exc:  # pragma: no cover - runtime dependency check
    print(
        "Missing dependency: pyserial\n"
        "Install with: python -m pip install pyserial\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    sys.exit(2)


CSV_PREFIX = "csv,"
CSV_HEADER = [
    "host_time_iso",
    "phase_index",
    "phase_name",
    "phase_elapsed_s",
    "board_ms",
    "left_cmd",
    "right_cmd",
    "left_duty",
    "right_duty",
    "left_mps",
    "right_mps",
    "left_count",
    "right_count",
    "irq_per_s",
    "sample_tick",
]


@dataclass(frozen=True)
class Phase:
    name: str
    left: float
    right: float
    duration_s: float


def parse_phase(spec: str) -> Phase:
    name = ""
    payload = spec.strip()

    if ":" in payload:
        name, payload = payload.split(":", 1)
        name = name.strip()

    parts = [part.strip() for part in payload.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Invalid phase '{spec}', expected left,right,duration or name:left,right,duration"
        )

    left_text, right_text, duration_text = parts

    try:
        left = float(left_text)
        right = float(right_text)
        duration_s = float(duration_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid numeric value in phase '{spec}': {exc}") from exc

    if duration_s <= 0.0:
        raise argparse.ArgumentTypeError(f"Phase duration must be > 0: '{spec}'")

    if not name:
        name = f"L{left:g}_R{right:g}_{duration_s:g}s"

    return Phase(name=name, left=left, right=right, duration_s=duration_s)


def default_phases() -> List[Phase]:
    return [
        Phase("idle_start", 0.0, 0.0, 2.0),
        Phase("left_05", 5.0, 0.0, 4.0),
        Phase("idle_1", 0.0, 0.0, 2.0),
        Phase("left_10", 10.0, 0.0, 4.0),
        Phase("idle_2", 0.0, 0.0, 2.0),
        Phase("left_20", 20.0, 0.0, 4.0),
        Phase("idle_3", 0.0, 0.0, 2.0),
        Phase("right_05", 0.0, 5.0, 4.0),
        Phase("idle_4", 0.0, 0.0, 2.0),
        Phase("right_10", 0.0, 10.0, 4.0),
        Phase("idle_5", 0.0, 0.0, 2.0),
        Phase("right_20", 0.0, 20.0, 4.0),
        Phase("idle_6", 0.0, 0.0, 2.0),
        Phase("both_10", 10.0, 10.0, 4.0),
        Phase("idle_7", 0.0, 0.0, 2.0),
        Phase("both_20", 20.0, 20.0, 4.0),
        Phase("idle_end", 0.0, 0.0, 2.0),
    ]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--output-dir",
        default="logs/wheel_pid",
        help="Directory for raw log and parsed csv output",
    )
    parser.add_argument(
        "--phase",
        action="append",
        type=parse_phase,
        help="Custom phase: left,right,duration or name:left,right,duration. Can be repeated.",
    )
    parser.add_argument(
        "--settle-before-start",
        type=float,
        default=1.0,
        help="Seconds to wait after opening serial before first command",
    )
    parser.add_argument(
        "--command-interval",
        type=float,
        default=0.05,
        help="Seconds to wait after sending each command",
    )
    parser.add_argument(
        "--no-default-phases",
        action="store_true",
        help="Disable built-in phase list and require --phase",
    )
    return parser


def send_command(ser: serial.Serial, left: float, right: float) -> None:
    payload = f"{left:g},{right:g}\n".encode("utf-8")
    ser.write(payload)
    ser.flush()


def reopen_serial(port: str, baud: int, timeout_s: float = 0.1) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=timeout_s)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


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
                    ser = reopen_serial(port, baud)
                    print(f"[serial] reconnected to {port}")
                    break
                except serial.SerialException:
                    time.sleep(0.2)
            else:
                raise serial.SerialException(
                    f"failed to reconnect to {port} before timeout: {last_error}"
                ) from last_error


def drain_boot_messages(
    ser: serial.Serial,
    port: str,
    baud: int,
    duration_s: float,
    raw_handle,
) -> serial.Serial:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        ser, line = safe_readline(ser, port, baud, deadline)
        if not line:
            continue
        decoded = line.decode("utf-8", errors="replace")
        raw_handle.write(decoded)
        raw_handle.flush()
        sys.stdout.write(decoded)
        sys.stdout.flush()
    return ser


def parse_csv_line(line: str) -> List[str] | None:
    if not line.startswith(CSV_PREFIX):
        return None
    fields = [field.strip() for field in line[len(CSV_PREFIX):].strip().split(",")]
    if len(fields) != 11:
        return None
    return fields


def iter_phases(args) -> Iterable[Phase]:
    if args.no_default_phases:
        return args.phase or []
    return (args.phase or []) if args.phase else default_phases()


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    phases = list(iter_phases(args))
    if not phases:
        parser.error("No phases specified. Use --phase or omit --no-default-phases.")

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = output_dir / f"wheel_step_raw_{stamp}.log"
    csv_path = output_dir / f"wheel_step_data_{stamp}.csv"

    print(f"Opening {args.port} @ {args.baud}")
    print(f"Raw log: {raw_path}")
    print(f"CSV log: {csv_path}")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, \
            raw_path.open("w", encoding="utf-8") as raw_handle, \
            csv_path.open("w", encoding="utf-8", newline="") as csv_handle:
        writer = csv.writer(csv_handle)
        writer.writerow(CSV_HEADER)

        print(f"Settling for {args.settle_before_start:.2f}s")
        ser = drain_boot_messages(ser, args.port, args.baud, args.settle_before_start, raw_handle)

        for phase_index, phase in enumerate(phases):
            print(
                f"[{phase_index:02d}] phase={phase.name} cmd=({phase.left:g},{phase.right:g}) "
                f"duration={phase.duration_s:.2f}s"
            )
            send_command(ser, phase.left, phase.right)
            time.sleep(args.command_interval)

            phase_start = time.monotonic()
            phase_end = phase_start + phase.duration_s

            while time.monotonic() < phase_end:
                ser, line = safe_readline(ser, args.port, args.baud, phase_end)
                if not line:
                    continue

                decoded = line.decode("utf-8", errors="replace")
                raw_handle.write(decoded)
                raw_handle.flush()
                sys.stdout.write(decoded)
                sys.stdout.flush()

                fields = parse_csv_line(decoded)
                if fields is None:
                    continue

                phase_elapsed = time.monotonic() - phase_start
                writer.writerow(
                    [
                        dt.datetime.now().isoformat(timespec="milliseconds"),
                        phase_index,
                        phase.name,
                        f"{phase_elapsed:.3f}",
                        *fields,
                    ]
                )
                csv_handle.flush()

        print("Sending final stop command")
        send_command(ser, 0.0, 0.0)
        time.sleep(args.command_interval)
        ser = drain_boot_messages(ser, args.port, args.baud, 1.0, raw_handle)

    print("Done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
