"""CSV logging and summary statistics for direct T_TS tracking."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import TextIO

import numpy as np

from common.geometry import invert_transform, so3_log
from common.safety import finite_transform


class TrackingPrecisionLogger:
    def __init__(self, path: Path) -> None:
        self.path = path.expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)

        self._file: TextIO = self.path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self._writer = csv.writer(self._file)
        self._start_time = time.monotonic()
        self._closed = False
        self._rows = 0

        self._tracking_position_errors: list[float] = []
        self._tracking_orientation_errors_deg: list[float] = []

        self._writer.writerow(
            [
                "time_s",
                "controller_state",
                "reason",
                "robot_state_age_s",
                "command_sent",
                "measured_x_T_S_m",
                "measured_y_T_S_m",
                "measured_z_T_S_m",
                "desired_x_T_S_m",
                "desired_y_T_S_m",
                "desired_z_T_S_m",
                "error_x_m",
                "error_y_m",
                "error_z_m",
                "position_error_m",
                "orientation_error_deg",
            ]
        )

    def log(
        self,
        *,
        T_TS: np.ndarray,
        T_TS_des: np.ndarray,
        controller_state: str,
        reason: str,
        robot_state_age: float,
        command_sent: bool,
    ) -> None:
        if self._closed:
            raise RuntimeError("Tracking logger is already closed.")

        if not finite_transform(T_TS):
            return

        if not finite_transform(T_TS_des):
            raise ValueError("T_TS_des is not a valid transform.")

        error_transform = invert_transform(T_TS) @ T_TS_des
        position_error = error_transform[:3, 3]
        position_error_norm = float(np.linalg.norm(position_error))

        orientation_error_deg = math.degrees(
            float(
                np.linalg.norm(
                    so3_log(error_transform[:3, :3])
                )
            )
        )

        elapsed = time.monotonic() - self._start_time

        self._writer.writerow(
            [
                f"{elapsed:.9f}",
                controller_state,
                reason,
                f"{robot_state_age:.9f}",
                int(command_sent),
                f"{T_TS[0, 3]:.9f}",
                f"{T_TS[1, 3]:.9f}",
                f"{T_TS[2, 3]:.9f}",
                f"{T_TS_des[0, 3]:.9f}",
                f"{T_TS_des[1, 3]:.9f}",
                f"{T_TS_des[2, 3]:.9f}",
                f"{position_error[0]:.9f}",
                f"{position_error[1]:.9f}",
                f"{position_error[2]:.9f}",
                f"{position_error_norm:.9f}",
                f"{orientation_error_deg:.9f}",
            ]
        )

        self._rows += 1

        if controller_state == "TRACKING":
            self._tracking_position_errors.append(
                position_error_norm
            )
            self._tracking_orientation_errors_deg.append(
                orientation_error_deg
            )

        if self._rows % 100 == 0:
            self._file.flush()

    @staticmethod
    def _statistics(values: list[float]) -> dict[str, float]:
        if not values:
            return {}

        array = np.asarray(values, dtype=float)

        return {
            "mean": float(np.mean(array)),
            "rms": float(np.sqrt(np.mean(array**2))),
            "p95": float(np.percentile(array, 95.0)),
            "maximum": float(np.max(array)),
        }

    def close(self) -> dict[str, object]:
        if self._closed:
            return {}

        self._closed = True
        duration = time.monotonic() - self._start_time

        self._file.flush()
        self._file.close()

        summary: dict[str, object] = {
            "log_file": str(self.path),
            "duration_s": duration,
            "valid_sample_count": self._rows,
            "tracking_sample_count": len(
                self._tracking_position_errors
            ),
            "tracking_position_error_m": self._statistics(
                self._tracking_position_errors
            ),
            "tracking_orientation_error_deg": self._statistics(
                self._tracking_orientation_errors_deg
            ),
        }

        summary_path = self.path.with_suffix(
            self.path.suffix + ".summary.json"
        )
        summary_path.write_text(
            json.dumps(summary, indent=2) + "\n",
            encoding="utf-8",
        )

        print("Tracking precision summary:")
        print(json.dumps(summary, indent=2))

        return summary

    def __enter__(self) -> "TrackingPrecisionLogger":
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()