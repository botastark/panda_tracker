from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

import numpy as np

from control.pbvs_controller import (
    ControllerState,
    PBVSController,
    TaskPoseMeasurement,
)


def controller_config(
    consecutive_valid_required: int = 3,
) -> SimpleNamespace:
    return SimpleNamespace(
        T_ES=np.eye(4),
        T_TS_des=np.eye(4),
        control_orientation=False,
        tracker_timeout=1.0,
        panda_state_timeout=1.0,
        max_tracker_position_jump=0.10,
        max_tracker_angle_jump=np.deg2rad(20.0),
        max_enable_position_error=0.50,
        max_enable_orientation_error=np.deg2rad(90.0),
        consecutive_valid_required=consecutive_valid_required,
        kp_position=1.0,
        kp_orientation=1.0,
        max_linear_speed=0.20,
        max_angular_speed=np.deg2rad(30.0),
        max_command_lead=0.05,
        workspace_min=np.array([-1.0, -1.0, -1.0]),
        workspace_max=np.array([1.0, 1.0, 1.0]),
    )


def measurement(sequence_id: int) -> TaskPoseMeasurement:
    return TaskPoseMeasurement(
        T_TS=np.eye(4),
        timestamp=time.monotonic(),
        valid=True,
        sequence_id=sequence_id,
    )


class MeasurementSequenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.T_BE = np.eye(4)

    def step(
        self,
        controller: PBVSController,
        task_pose: TaskPoseMeasurement,
    ):
        return controller.step(
            T_BE=self.T_BE,
            robot_state_age=0.0,
            task_pose=task_pose,
            dt=0.01,
        )

    def test_repeated_packet_does_not_advance_valid_count(self) -> None:
        controller = PBVSController(
            controller_config(consecutive_valid_required=3)
        )

        packet = measurement(sequence_id=10)

        _, first = self.step(controller, packet)
        _, second = self.step(controller, packet)
        _, third = self.step(controller, packet)

        self.assertEqual(first.state, ControllerState.READY)
        self.assertEqual(second.state, ControllerState.READY)
        self.assertEqual(third.state, ControllerState.READY)

        self.assertEqual(controller.valid_count, 1)
        self.assertEqual(
            controller.last_processed_task_sequence,
            10,
        )

    def test_distinct_packets_advance_valid_count(self) -> None:
        controller = PBVSController(
            controller_config(consecutive_valid_required=3)
        )

        self.step(controller, measurement(sequence_id=10))
        self.step(controller, measurement(sequence_id=11))
        command, diagnostics = self.step(
            controller,
            measurement(sequence_id=12),
        )

        self.assertEqual(controller.valid_count, 3)
        self.assertEqual(
            diagnostics.state,
            ControllerState.TRACKING,
        )
        self.assertIsNotNone(command)

    def test_duplicate_packet_remains_usable_after_tracking(self) -> None:
        controller = PBVSController(
            controller_config(consecutive_valid_required=2)
        )

        self.step(controller, measurement(sequence_id=20))

        packet = measurement(sequence_id=21)

        first_command, first_diagnostics = self.step(
            controller,
            packet,
        )
        second_command, second_diagnostics = self.step(
            controller,
            packet,
        )

        self.assertEqual(
            first_diagnostics.state,
            ControllerState.TRACKING,
        )
        self.assertEqual(
            second_diagnostics.state,
            ControllerState.TRACKING,
        )

        self.assertEqual(controller.valid_count, 2)
        self.assertIsNotNone(first_command)
        self.assertIsNotNone(second_command)

    def test_missing_pose_resets_measurement_sequence(self) -> None:
        controller = PBVSController(
            controller_config(consecutive_valid_required=2)
        )

        self.step(controller, measurement(sequence_id=30))

        _, diagnostics = controller.step(
            T_BE=self.T_BE,
            robot_state_age=0.0,
            task_pose=None,
            dt=0.01,
        )

        self.assertEqual(
            diagnostics.state,
            ControllerState.WAIT_FOR_TASK_POSE,
        )
        self.assertEqual(controller.valid_count, 0)
        self.assertIsNone(
            controller.last_processed_task_sequence
        )


if __name__ == "__main__":
    unittest.main()