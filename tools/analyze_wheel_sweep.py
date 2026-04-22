#!/usr/bin/env python3
"""Summarize wheel sweep CSV logs for open-loop motor/encoder characterization."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
from collections import defaultdict


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="Path to wheel_step_data_*.csv")
    parser.add_argument(
        "--moving-threshold",
        type=float,
        default=0.02,
        help="Minimum |speed_mps| to consider the wheel as moving",
    )
    return parser.parse_args()


def safe_float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def safe_int(row: dict[str, str], key: str) -> int:
    return int(float(row[key]))


def load_rows(csv_path: pathlib.Path) -> list[dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def eval_ppr_expr(text: str) -> int | None:
    match = re.search(r"\.pulses_per_revolution\s*=\s*([^,]+),", text)
    if not match:
        return None
    expr = match.group(1).replace("u", "").replace("U", "").strip()
    if not re.fullmatch(r"[0-9* +()-/]+", expr):
        return None
    return int(eval(expr, {"__builtins__": {}}, {}))


def parse_encoder_params(repo_root: pathlib.Path) -> dict[str, float | int]:
    text = (repo_root / "Sources" / "chassis_system.c").read_text(encoding="utf-8")

    ppr = eval_ppr_expr(text)
    radius_match = re.search(r"\.wheel_radius_m\s*=\s*([0-9.]+)f", text)
    sample_match = re.search(r"\.sample_period_s\s*=\s*([0-9.]+)f", text)

    if ppr is None or radius_match is None or sample_match is None:
        raise RuntimeError("Failed to parse encoder params from Sources/chassis_system.c")

    radius_m = float(radius_match.group(1))
    sample_period_s = float(sample_match.group(1))
    circumference_m = 2.0 * math.pi * radius_m

    return {
        "pulses_per_revolution": ppr,
        "wheel_radius_m": radius_m,
        "wheel_circumference_m": circumference_m,
        "sample_period_s": sample_period_s,
        "counts_per_meter": ppr / circumference_m,
        "meters_per_count": circumference_m / ppr,
    }


def summarize_phases(rows: list[dict[str, str]]) -> list[dict[str, float | int | str]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[row["phase_name"]].append(row)

    summaries: list[dict[str, float | int | str]] = []
    for phase_name, phase_rows in grouped.items():
        tail = phase_rows[max(0, len(phase_rows) // 2):]

        left_cmd = sum(safe_float(row, "left_cmd") for row in tail) / len(tail)
        right_cmd = sum(safe_float(row, "right_cmd") for row in tail) / len(tail)
        left_mps = sum(safe_float(row, "left_mps") for row in tail) / len(tail)
        right_mps = sum(safe_float(row, "right_mps") for row in tail) / len(tail)
        irq_per_s = sum(safe_float(row, "irq_per_s") for row in tail) / len(tail)

        summaries.append(
            {
                "phase_name": phase_name,
                "phase_index": safe_int(phase_rows[0], "phase_index"),
                "samples": len(phase_rows),
                "left_cmd": left_cmd,
                "right_cmd": right_cmd,
                "left_mps": left_mps,
                "right_mps": right_mps,
                "left_abs_mps": abs(left_mps),
                "right_abs_mps": abs(right_mps),
                "irq_per_s": irq_per_s,
            }
        )

    summaries.sort(key=lambda item: int(item["phase_index"]))
    return summaries


def select_wheel_summaries(
    summaries: list[dict[str, float | int | str]],
    wheel: str,
) -> list[dict[str, float | int | str]]:
    selected = []
    for item in summaries:
        left_cmd = float(item["left_cmd"])
        right_cmd = float(item["right_cmd"])
        if wheel == "left" and abs(left_cmd) > 1e-6 and abs(right_cmd) < 1e-6:
            selected.append(item)
        elif wheel == "right" and abs(right_cmd) > 1e-6 and abs(left_cmd) < 1e-6:
            selected.append(item)
    return selected


def print_wheel_summary(items: list[dict[str, float | int | str]], wheel: str, threshold: float) -> None:
    speed_key = f"{wheel}_abs_mps"
    cmd_key = f"{wheel}_cmd"

    if not items:
        print(f"{wheel}: no matching phases")
        return

    moving_items = [item for item in items if float(item[speed_key]) >= threshold]
    min_item = moving_items[0] if moving_items else None
    max_item = max(items, key=lambda item: float(item[speed_key]))

    print(f"{wheel}:")
    if min_item is None:
        print(f"  min moving: not reached above threshold {threshold:.3f} m/s")
    else:
        print(
            "  min moving: "
            f"cmd={float(min_item[cmd_key]):.3f}, "
            f"speed={float(min_item[speed_key]):.3f} m/s, "
            f"phase={min_item['phase_name']}"
        )
    print(
        "  max speed: "
        f"cmd={float(max_item[cmd_key]):.3f}, "
        f"speed={float(max_item[speed_key]):.3f} m/s, "
        f"phase={max_item['phase_name']}"
    )


def main() -> int:
    args = parse_args()
    csv_path = pathlib.Path(args.csv_path).resolve()
    repo_root = csv_path.parent.parent.parent

    rows = load_rows(csv_path)
    summaries = summarize_phases(rows)
    params = parse_encoder_params(repo_root)

    print(f"csv={csv_path}")
    print(f"samples={len(rows)}")
    print("encoder:")
    print(f"  pulses_per_revolution={params['pulses_per_revolution']}")
    print(f"  wheel_radius_m={params['wheel_radius_m']:.6f}")
    print(f"  wheel_circumference_m={params['wheel_circumference_m']:.6f}")
    print(f"  sample_period_s={params['sample_period_s']:.6f}")
    print(f"  counts_per_meter={params['counts_per_meter']:.3f}")
    print(f"  meters_per_count={params['meters_per_count']:.8f}")

    print_wheel_summary(select_wheel_summaries(summaries, "left"), "left", args.moving_threshold)
    print_wheel_summary(select_wheel_summaries(summaries, "right"), "right", args.moving_threshold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
