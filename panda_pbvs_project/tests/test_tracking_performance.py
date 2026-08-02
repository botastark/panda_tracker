from __future__ import annotations

from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from control.pbvs_controller import (  # noqa: E402
    ControllerState,
    PBVSController,
    TaskPoseMeasurement,
)


def controller_config(
    *,
    feedforward_enabled: bool,
) -> SimpleNamespace:
    return SimpleNamespace(
        T_ES=np.eye(4),
        T_TS_des=np.eye(4),
        control_orientation=False,

        tracker_timeout=1.0,
        panda_state_timeout=1.0,

        max_tracker_position_jump=1.0,
        max_tracker_angle_jump=np.deg2rad(180.0),

        max_enable_position_error=1.0,
        max_enable_orientation_error=np.deg2rad(180.0),

        consecutive_valid_required=1,

        kp_position=1.0,
        kp_orientation=0.0,

        max_linear_speed=0.10,
        max_angular_speed=np.deg2rad(30.0),
        max_command_lead=0.05,

        target_feedforward_enabled=feedforward_enabled,
        target_velocity_filter_alpha=1.0,
        max_target_linear_speed=0.05,
        max_target_angular_speed=np.deg2rad(30.0),

        workspace_min=np.array([-2.0, -2.0, -2.0]),
        workspace_max=np.array([2.0, 2.0, 2.0]),
    )


def measurement_for_goal(
    *,
    current_x: float,
    goal_x: float,
    sequence_id: int,
    timestamp: float,
) -> TaskPoseMeasurement:
    """
    With identity T_ES and T_TS_des:

        T_goal = T_BE @ inverse(T_TS)

    Therefore T_TS.x = current_x - goal_x.
    """

    T_TS = np.eye(4)
    T_TS[0, 3] = current_x - goal_x

    return TaskPoseMeasurement(
        T_TS=T_TS,
        timestamp=timestamp,
        valid=True,
        sequence_id=sequence_id,
    )


class TrackingPerformanceTests(unittest.TestCase):
    DT = 0.01
    DURATION = 4.0
    TARGET_SPEED = 0.02

    def run_tracking_case(
        self,
        *,
        feedforward_enabled: bool,
    ) -> np.ndarray:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=feedforward_enabled,
            )
        )

        T_BE = np.eye(4)
        errors: list[float] = []

        base_time = time.monotonic()
        number_of_steps = int(self.DURATION / self.DT)

        for index in range(number_of_steps):
            elapsed = index * self.DT
            simulated_time = base_time + elapsed
            goal_x = self.TARGET_SPEED * elapsed

            measurement = measurement_for_goal(
                current_x=float(T_BE[0, 3]),
                goal_x=goal_x,
                sequence_id=index + 1,
                timestamp=simulated_time,
            )

            with patch(
                "control.pbvs_controller.time.monotonic",
                return_value=simulated_time,
            ):
                command, diagnostics = controller.step(
                    T_BE=T_BE,
                    robot_state_age=0.0,
                    task_pose=measurement,
                    dt=self.DT,
                )

            self.assertEqual(
                diagnostics.state,
                ControllerState.TRACKING,
            )
            self.assertIsNotNone(command)

            # Ideal inner loop: assume the robot reaches the command exactly.
            T_BE = command.copy()

            errors.append(
                goal_x - float(T_BE[0, 3])
            )

        return np.asarray(errors)

    def test_feedforward_reduces_constant_velocity_lag(
        self,
    ) -> None:
        feedback_errors = self.run_tracking_case(
            feedforward_enabled=False,
        )
        feedforward_errors = self.run_tracking_case(
            feedforward_enabled=True,
        )

        warmup_steps = int(1.0 / self.DT)

        feedback_window = feedback_errors[warmup_steps:]
        feedforward_window = feedforward_errors[warmup_steps:]

        feedback_rms = float(
            np.sqrt(np.mean(np.square(feedback_window)))
        )
        feedforward_rms = float(
            np.sqrt(np.mean(np.square(feedforward_window)))
        )

        feedback_final = float(feedback_errors[-1])
        feedforward_final = float(feedforward_errors[-1])

        rms_ratio = (
            feedforward_rms / feedback_rms
            if feedback_rms > 0.0
            else float("inf")
        )

        print(
            "\n\nTracking performance:",
            f"\n  Proportional-only final lag: "
            f"{1000.0 * feedback_final:.3f} mm",
            f"\n  Feedforward final error:     "
            f"{1000.0 * feedforward_final:.3f} mm",
            f"\n  Proportional-only RMS:       "
            f"{1000.0 * feedback_rms:.3f} mm",
            f"\n  Feedforward RMS:             "
            f"{1000.0 * feedforward_rms:.3f} mm",
            f"\n  RMS ratio:                   "
            f"{100.0 * rms_ratio:.2f}%",
        )

        self.assertGreater(
            feedback_final,
            0.015,
            msg=(
                "Feedback-only tracking did not exhibit the "
                "expected constant-velocity lag."
            ),
        )

        self.assertLess(
            abs(feedforward_final),
            0.001,
            msg=(
                "Feedforward final error did not fall below 1 mm."
            ),
        )

        self.assertLess(
            rms_ratio,
            0.20,
            msg=(
                "Feedforward RMS must be below 20% of the "
                "feedback-only RMS."
            ),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
