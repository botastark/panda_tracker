from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from control.pbvs_controller import ControllerState  # noqa: E402
from run_control import tracking_command_sent  # noqa: E402


class RunControlLoggingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.command = np.eye(4)

    def test_tracking_command_is_marked_sent(self) -> None:
        self.assertTrue(
            tracking_command_sent(
                command=self.command,
                dry_run=False,
                controller_state=ControllerState.TRACKING,
            )
        )

    def test_ready_hold_and_fault_are_not_tracking_commands(
        self,
    ) -> None:
        for state in (
            ControllerState.READY,
            ControllerState.HOLD,
            ControllerState.FAULT,
            ControllerState.WAIT_FOR_ROBOT,
            ControllerState.WAIT_FOR_TASK_POSE,
        ):
            with self.subTest(state=state):
                self.assertFalse(
                    tracking_command_sent(
                        command=self.command,
                        dry_run=False,
                        controller_state=state,
                    )
                )

    def test_dry_run_is_not_marked_sent(self) -> None:
        self.assertFalse(
            tracking_command_sent(
                command=self.command,
                dry_run=True,
                controller_state=ControllerState.TRACKING,
            )
        )

    def test_missing_command_is_not_marked_sent(self) -> None:
        self.assertFalse(
            tracking_command_sent(
                command=None,
                dry_run=False,
                controller_state=ControllerState.TRACKING,
            )
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)