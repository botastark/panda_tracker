from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from common.geometry import invert_transform  # noqa: E402
from perception.task_pose_adapter import (  # noqa: E402
    task_pose_from_camera_target,
)


class TaskPoseAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.T_EC = np.eye(4)
        self.T_EC[:3, 3] = [0.05, 0.0, 0.0965]

        self.T_ES = np.eye(4)
        self.T_ES[:3, 3] = [0.0, 0.0, 0.28375]

    def test_identity_camera_target_relation(
        self,
    ) -> None:
        T_CT = np.eye(4)

        result = task_pose_from_camera_target(
            T_CT=T_CT,
            T_EC=self.T_EC,
            T_ES=self.T_ES,
        )

        expected = (
            invert_transform(self.T_EC)
            @ self.T_ES
        )

        np.testing.assert_allclose(
            result,
            expected,
            atol=1e-12,
        )

    def test_camera_pose_that_matches_desired_task_pose(
        self,
    ) -> None:
        T_TS_des = np.eye(4)
        T_TS_des[:3, 3] = [0.0, 0.0, 0.05]

        # From:
        #
        #   T_TS_des = inverse(T_EC @ T_CT) @ T_ES
        #
        # solve for T_CT:
        #
        #   T_CT = inverse(T_EC) @ T_ES @ inverse(T_TS_des)
        #
        T_CT = (
            invert_transform(self.T_EC)
            @ self.T_ES
            @ invert_transform(T_TS_des)
        )

        result = task_pose_from_camera_target(
            T_CT=T_CT,
            T_EC=self.T_EC,
            T_ES=self.T_ES,
        )

        np.testing.assert_allclose(
            result,
            T_TS_des,
            atol=1e-12,
        )

    def test_target_translation_changes_task_pose(
        self,
    ) -> None:
        T_CT_1 = np.eye(4)

        T_CT_2 = np.eye(4)
        T_CT_2[0, 3] = 0.02

        result_1 = task_pose_from_camera_target(
            T_CT=T_CT_1,
            T_EC=self.T_EC,
            T_ES=self.T_ES,
        )

        result_2 = task_pose_from_camera_target(
            T_CT=T_CT_2,
            T_EC=self.T_EC,
            T_ES=self.T_ES,
        )

        self.assertFalse(
            np.allclose(
                result_1,
                result_2,
            )
        )

        self.assertAlmostEqual(
            result_2[0, 3] - result_1[0, 3],
            -0.02,
            places=12,
        )

    def test_rejects_wrong_shape(self) -> None:
        with self.assertRaises(ValueError):
            task_pose_from_camera_target(
                T_CT=np.eye(3),
                T_EC=self.T_EC,
                T_ES=self.T_ES,
            )

    def test_rejects_nonfinite_transform(
        self,
    ) -> None:
        T_CT = np.eye(4)
        T_CT[0, 3] = np.nan

        with self.assertRaises(ValueError):
            task_pose_from_camera_target(
                T_CT=T_CT,
                T_EC=self.T_EC,
                T_ES=self.T_ES,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)