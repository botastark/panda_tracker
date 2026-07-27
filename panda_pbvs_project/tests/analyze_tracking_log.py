#!/usr/bin/env python3
"""Analyze a CSV produced by TrackingPrecisionLogger."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Report PBVS position error over time."
    )
    parser.add_argument(
        "log",
        type=Path,
        help="Tracking precision CSV file.",
    )
    parser.add_argument(
        "--final-window",
        type=float,
        default=2.0,
        help="Final steady-state window in seconds.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.0,
        help="Exclude this many seconds after TRACKING begins.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.final_window <= 0.0:
        raise ValueError("--final-window must be positive.")

    if args.warmup < 0.0:
        raise ValueError("--warmup cannot be negative.")

    rows: list[dict[str, float]] = []

    with args.log.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["controller_state"] != "TRACKING":
                continue

            rows.append(
                {
                    "time": float(row["time_s"]),
                    "error": float(row["position_error_m"]),
                    "ex": float(row["error_x_m"]),
                    "ey": float(row["error_y_m"]),
                    "ez": float(row["error_z_m"]),
                }
            )

    if not rows:
        raise RuntimeError("No TRACKING samples found.")

    start_time = rows[0]["time"]

    for row in rows:
        row["relative_time"] = row["time"] - start_time

    rows = [
        row
        for row in rows
        if row["relative_time"] >= args.warmup
    ]

    if not rows:
        raise RuntimeError("No samples remain after warm-up exclusion.")

    bins: defaultdict[int, list[float]] = defaultdict(list)

    for row in rows:
        second = int(row["relative_time"])
        bins[second].append(row["error"])

    print("Mean position error by elapsed second:")

    for second in sorted(bins):
        values = np.asarray(bins[second])

        print(
            f"{second:2d}-{second + 1:2d} s: "
            f"mean={np.mean(values) * 1000:7.2f} mm, "
            f"max={np.max(values) * 1000:7.2f} mm"
        )

    errors = np.asarray([row["error"] for row in rows])
    times = np.asarray([row["relative_time"] for row in rows])

    final_start = max(
        args.warmup,
        times[-1] - args.final_window,
    )
    final_rows = [
        row
        for row in rows
        if row["relative_time"] >= final_start
    ]

    final_errors = np.asarray(
        [row["error"] for row in final_rows]
    )
    final_components = np.asarray(
        [
            [row["ex"], row["ey"], row["ez"]]
            for row in final_rows
        ]
    )

    print()
    print("Overall:")
    print(f"  minimum: {np.min(errors) * 1000:.2f} mm")
    print(f"  mean:    {np.mean(errors) * 1000:.2f} mm")
    print(
        "  RMS:     "
        f"{np.sqrt(np.mean(errors**2)) * 1000:.2f} mm"
    )
    print(f"  maximum: {np.max(errors) * 1000:.2f} mm")
    print(f"  final:   {errors[-1] * 1000:.2f} mm")

    print()
    print(f"Final {args.final_window:g} seconds:")
    print(f"  mean: {np.mean(final_errors) * 1000:.2f} mm")
    print(
        "  RMS:  "
        f"{np.sqrt(np.mean(final_errors**2)) * 1000:.2f} mm"
    )
    print(f"  max:  {np.max(final_errors) * 1000:.2f} mm")
    print(
        "  mean error components: "
        f"x={np.mean(final_components[:, 0]) * 1000:.2f} mm, "
        f"y={np.mean(final_components[:, 1]) * 1000:.2f} mm, "
        f"z={np.mean(final_components[:, 2]) * 1000:.2f} mm"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
