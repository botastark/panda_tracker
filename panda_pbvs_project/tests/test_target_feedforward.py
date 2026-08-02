from __future__ import annotations

import contextlib
import io
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace

import numpy as np


# Allow both:
#
#   python -m unittest discover -s tests -p 'test_target_feedforward.py' -v
#
# and:
#
#   python tests/test_target_feedforward.py
#
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
    max_target_linear_speed: float = 0.20,
    target_velocity_filter_alpha: float = 1.0,
) -> SimpleNamespace:
    """Return the minimal configuration required by PBVSController."""

    return SimpleNamespace(
        T_ES=np.eye(4),
        T_TS_des=np.eye(4),
        control_orientation=False,
        tracker_timeout=1.0,
        panda_state_timeout=1.0,
        max_tracker_position_jump=0.50,
        max_tracker_angle_jump=np.deg2rad(90.0),
        max_enable_position_error=0.50,
        max_enable_orientation_error=np.deg2rad(90.0),
        consecutive_valid_required=1,

        # Zero proportional gain isolates target-motion feedforward.
        kp_position=0.0,
        kp_orientation=0.0,

        max_linear_speed=0.20,
        max_angular_speed=np.deg2rad(30.0),
        max_command_lead=0.05,

        target_feedforward_enabled=feedforward_enabled,
        target_velocity_filter_alpha=target_velocity_filter_alpha,
        max_target_linear_speed=max_target_linear_speed,
        max_target_angular_speed=np.deg2rad(30.0),

        workspace_min=np.array([-1.0, -1.0, -1.0]),
        workspace_max=np.array([1.0, 1.0, 1.0]),
    )


def task_measurement(
    *,
    sequence_id: int,
    timestamp: float,
    goal_x: float,
) -> TaskPoseMeasurement:
    """
    Construct a task measurement producing the requested EE goal.

    With identity T_ES, identity T_TS_des, and identity T_BE:

        T_goal = inverse(T_TS)

    Therefore T_TS must translate by -goal_x to produce a +goal_x goal.
    """

    T_TS = np.eye(4)
    T_TS[0, 3] = -goal_x

    return TaskPoseMeasurement(
        T_TS=T_TS,
        timestamp=timestamp,
        valid=True,
        sequence_id=sequence_id,
    )


class TargetFeedforwardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.T_BE = np.eye(4)

    def step(
        self,
        controller: PBVSController,
        measurement: TaskPoseMeasurement | None,
        *,
        dt: float = 0.01,
    ):
        return controller.step(
            T_BE=self.T_BE,
            robot_state_age=0.0,
            task_pose=measurement,
            dt=dt,
        )

    def seed_velocity_estimator(
        self,
        controller: PBVSController,
    ) -> tuple[np.ndarray, object]:
        """
        Supply two unique measurements representing motion at 0.1 m/s.

        The goal moves from 0.00 m to 0.01 m over 0.10 seconds.
        """

        now = time.monotonic()

        self.step(
            controller,
            task_measurement(
                sequence_id=1,
                timestamp=now - 0.10,
                goal_x=0.00,
            ),
        )

        return self.step(
            controller,
            task_measurement(
                sequence_id=2,
                timestamp=now,
                goal_x=0.01,
            ),
        )

    def test_feedforward_moves_with_zero_proportional_gain(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=True,
            )
        )

        command, diagnostics = self.seed_velocity_estimator(
            controller
        )

        self.assertEqual(
            diagnostics.state,
            ControllerState.TRACKING,
        )
        self.assertIsNotNone(command)

        # 0.01 m / 0.10 s = 0.10 m/s.
        self.assertAlmostEqual(
            diagnostics.target_linear_speed,
            0.10,
            places=6,
        )
        self.assertAlmostEqual(
            diagnostics.commanded_linear_speed,
            0.10,
            places=6,
        )

        # 0.10 m/s integrated for 0.01 s = 0.001 m.
        self.assertAlmostEqual(
            command[0, 3],
            0.001,
            places=6,
        )

    def test_feedforward_disabled_preserves_previous_behavior(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=False,
            )
        )

        command, diagnostics = self.seed_velocity_estimator(
            controller
        )

        self.assertEqual(
            diagnostics.state,
            ControllerState.TRACKING,
        )
        self.assertIsNotNone(command)

        # The estimator may still run for diagnostics.
        self.assertAlmostEqual(
            diagnostics.target_linear_speed,
            0.10,
            places=6,
        )

        # But disabled feedforward and zero Kp produce no motion.
        self.assertAlmostEqual(
            diagnostics.commanded_linear_speed,
            0.0,
            places=6,
        )
        self.assertAlmostEqual(
            command[0, 3],
            0.0,
            places=6,
        )

    def test_target_velocity_is_clamped_before_use(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=True,
                max_target_linear_speed=0.05,
            )
        )

        command, diagnostics = self.seed_velocity_estimator(
            controller
        )

        self.assertIsNotNone(command)

        # Raw estimate is 0.10 m/s but estimator limit is 0.05 m/s.
        self.assertAlmostEqual(
            diagnostics.target_linear_speed,
            0.05,
            places=6,
        )
        self.assertAlmostEqual(
            diagnostics.commanded_linear_speed,
            0.05,
            places=6,
        )

        # 0.05 m/s integrated for 0.01 s = 0.0005 m.
        self.assertAlmostEqual(
            command[0, 3],
            0.0005,
            places=6,
        )

    def test_duplicate_measurement_does_not_change_velocity_estimate(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=True,
            )
        )

        now = time.monotonic()

        first = task_measurement(
            sequence_id=10,
            timestamp=now - 0.10,
            goal_x=0.00,
        )
        second = task_measurement(
            sequence_id=11,
            timestamp=now,
            goal_x=0.01,
        )

        self.step(controller, first)
        self.step(controller, second)

        estimate_before = (
            controller.target_linear_velocity.copy()
        )

        # Reuse the same sequence ID and timestamp.
        self.step(controller, second)

        np.testing.assert_allclose(
            controller.target_linear_velocity,
            estimate_before,
            atol=1e-12,
        )

    def test_missing_measurement_resets_velocity_estimator(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=True,
            )
        )

        self.seed_velocity_estimator(controller)

        self.assertGreater(
            np.linalg.norm(
                controller.target_linear_velocity
            ),
            0.0,
        )

        _, diagnostics = self.step(
            controller,
            None,
        )

        self.assertEqual(
            diagnostics.state,
            ControllerState.WAIT_FOR_TASK_POSE,
        )
        self.assertIsNone(controller.last_goal_pose)
        self.assertIsNone(controller.last_goal_timestamp)

        np.testing.assert_allclose(
            controller.target_linear_velocity,
            np.zeros(3),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            controller.target_angular_velocity,
            np.zeros(3),
            atol=1e-12,
        )

    def test_control_step_does_not_print(
        self,
    ) -> None:
        controller = PBVSController(
            controller_config(
                feedforward_enabled=True,
            )
        )

        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            self.seed_velocity_estimator(controller)

        self.assertEqual(output.getvalue(), "")


if __name__ == "__main__":
    unittest.main(verbosity=2)