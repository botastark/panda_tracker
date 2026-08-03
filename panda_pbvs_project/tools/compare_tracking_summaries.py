#!/usr/bin/env python3
"""Compare two TrackingPrecisionLogger summary JSON files."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


STAT_NAMES = ("mean", "rms", "p95", "maximum")


def load_summary(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object.")

    for key in (
        "tracking_sample_count",
        "tracking_position_error_m",
        "tracking_orientation_error_deg",
    ):
        if key not in data:
            raise ValueError(f"{path}: missing {key!r}.")

    return data


def improvement_percent(baseline: float, candidate: float) -> float:
    if baseline == 0.0:
        return math.nan if candidate != 0.0 else 0.0
    return 100.0 * (baseline - candidate) / baseline


def print_group(
    title: str,
    baseline: dict[str, Any],
    candidate: dict[str, Any],
    *,
    scale: float,
    unit: str,
) -> None:
    print(f"\n{title}")
    print(
        f"{'metric':<10} {'baseline':>14} {'candidate':>14} "
        f"{'improvement':>13}"
    )

    for name in STAT_NAMES:
        b = float(baseline[name]) * scale
        c = float(candidate[name]) * scale
        improvement = improvement_percent(b, c)
        improvement_text = (
            "n/a" if math.isnan(improvement) else f"{improvement:+.2f}%"
        )
        print(
            f"{name:<10} {b:>11.4f} {unit:<2} "
            f"{c:>11.4f} {unit:<2} {improvement_text:>13}"
        )


def regression_percent(baseline: float, candidate: float) -> float:
    return -improvement_percent(baseline, candidate)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare baseline and candidate PBVS tracking summaries. "
            "Positive improvement means lower candidate error."
        )
    )
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument(
        "--minimum-sample-ratio",
        type=float,
        default=0.90,
        help=(
            "Candidate tracking samples must be at least this fraction "
            "of the baseline. Default: 0.90."
        ),
    )
    parser.add_argument(
        "--maximum-rms-regression-percent",
        type=float,
        default=2.0,
        help=(
            "Maximum permitted position or orientation RMS regression. "
            "Default: 2.0."
        ),
    )
    args = parser.parse_args()

    if not 0.0 < args.minimum_sample_ratio <= 1.0:
        parser.error("--minimum-sample-ratio must be in (0, 1].")
    if args.maximum_rms_regression_percent < 0.0:
        parser.error("--maximum-rms-regression-percent must be non-negative.")

    baseline = load_summary(args.baseline)
    candidate = load_summary(args.candidate)

    baseline_samples = int(baseline["tracking_sample_count"])
    candidate_samples = int(candidate["tracking_sample_count"])

    if baseline_samples <= 0:
        raise ValueError("Baseline has no TRACKING samples.")
    if candidate_samples <= 0:
        raise ValueError("Candidate has no TRACKING samples.")

    print(f"Baseline : {args.baseline}")
    print(f"Candidate: {args.candidate}")
    print(
        "\nTracking samples: "
        f"{baseline_samples} -> {candidate_samples} "
        f"({candidate_samples / baseline_samples:.3f}x)"
    )

    position_baseline = baseline["tracking_position_error_m"]
    position_candidate = candidate["tracking_position_error_m"]
    orientation_baseline = baseline["tracking_orientation_error_deg"]
    orientation_candidate = candidate["tracking_orientation_error_deg"]

    print_group(
        "Position error",
        position_baseline,
        position_candidate,
        scale=1000.0,
        unit="mm",
    )
    print_group(
        "Orientation error",
        orientation_baseline,
        orientation_candidate,
        scale=1.0,
        unit="deg",
    )

    failures: list[str] = []

    sample_ratio = candidate_samples / baseline_samples
    if sample_ratio < args.minimum_sample_ratio:
        failures.append(
            "candidate tracking sample ratio "
            f"{sample_ratio:.3f} is below "
            f"{args.minimum_sample_ratio:.3f}"
        )

    for label, b_group, c_group in (
        ("position", position_baseline, position_candidate),
        ("orientation", orientation_baseline, orientation_candidate),
    ):
        regression = regression_percent(
            float(b_group["rms"]),
            float(c_group["rms"]),
        )
        if regression > args.maximum_rms_regression_percent:
            failures.append(
                f"{label} RMS regressed by {regression:.2f}% "
                f"(limit {args.maximum_rms_regression_percent:.2f}%)"
            )

    if failures:
        print("\nVERDICT: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("\nVERDICT: PASS")
    print(
        "The candidate meets the sample-count and RMS-regression gates. "
        "Review p95, maximum, HOLD reasons, and repeatability before keeping it."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
