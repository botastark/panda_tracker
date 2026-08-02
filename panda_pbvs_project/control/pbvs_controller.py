from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time

import numpy as np

from common.config import PBVSConfig
from common.geometry import (
    clamp_norm,
    invert_transform,
    make_transform,
    so3_exp,
    so3_log,
)
from common.safety import clamp_workspace, finite_transform


class ControllerState(Enum):
    WAIT_FOR_ROBOT = auto()
    WAIT_FOR_TASK_POSE = auto()
    READY = auto()
    TRACKING = auto()
    HOLD = auto()
    FAULT = auto()


@dataclass
class TaskPoseMeasurement:
    T_TS: np.ndarray
    timestamp: float
    valid: bool = True
    # Monotonically increasing identifier assigned when a UDP packet is
    # received. Control-loop iterations that reuse the same latest packet
    # therefore retain the same sequence_id.
    sequence_id: int = 0

@dataclass
class PBVSDiagnostics:
    state: ControllerState
    position_error: float = 0.0
    orientation_error: float = 0.0
    reason: str = ""

    target_linear_speed: float = 0.0
    target_angular_speed: float = 0.0
    commanded_linear_speed: float = 0.0
    commanded_angular_speed: float = 0.0
    command_lead: float = 0.0

class PBVSController:
    def __init__(self, config: PBVSConfig) -> None:
        self.config = config
        self.state = ControllerState.WAIT_FOR_ROBOT
        self.last_task_pose: TaskPoseMeasurement | None = None
        self.last_processed_task_sequence: int | None = None
        self.valid_count = 0

        # Persistent equilibrium pose sent to explorer.
        self.command_pose: np.ndarray | None = None

        # Maximum distance between measured EE and commanded equilibrium.
        # Units: metres.
        self.max_command_lead = self.config.max_command_lead

        self.target_feedforward_enabled = bool(
            getattr(config, "target_feedforward_enabled", False)
        )
        self.target_velocity_filter_alpha = float(
            getattr(config, "target_velocity_filter_alpha", 0.25)
        )
        self.max_target_linear_speed = float(
            getattr(
                config,
                "max_target_linear_speed",
                self.config.max_linear_speed,
            )
        )
        self.max_target_angular_speed = float(
            getattr(
                config,
                "max_target_angular_speed",
                self.config.max_angular_speed,
            )
        )

        self.last_goal_pose: np.ndarray | None = None
        self.last_goal_timestamp: float | None = None
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)

    def _task_pose_valid(
        self,
        measurement: TaskPoseMeasurement,
        now: float,
    ) -> bool:
        if (
            not measurement.valid
            or not finite_transform(measurement.T_TS)
        ):
            return False

        if (
            now - measurement.timestamp
            > self.config.tracker_timeout
        ):
            return False

        if self.last_task_pose is not None:
            delta = (
                invert_transform(self.last_task_pose.T_TS)
                @ measurement.T_TS
            )

            if (
                np.linalg.norm(delta[:3, 3])
                > self.config.max_tracker_position_jump
            ):
                return False

            if (
                self.config.control_orientation
                and np.linalg.norm(
                    so3_log(delta[:3, :3])
                )
                > self.config.max_tracker_angle_jump
            ):
                return False

        return True

    def _goal_pose(
        self,
        T_BE: np.ndarray,
        T_TS: np.ndarray,
    ) -> np.ndarray:
        T_SE = invert_transform(self.config.T_ES)

        delta_T_S = (
            invert_transform(T_TS)
            @ self.config.T_TS_des
        )

        delta_T_E = (
            self.config.T_ES
            @ delta_T_S
            @ T_SE
        )

        return T_BE @ delta_T_E

    def _reset_target_velocity_estimator(self) -> None:
        self.last_goal_pose = None
        self.last_goal_timestamp = None
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)

    def _update_target_velocity_estimate(
        self,
        goal_pose: np.ndarray,
        measurement_timestamp: float,
    ) -> None:
        if (
            self.last_goal_pose is None
            or self.last_goal_timestamp is None
        ):
            self.last_goal_pose = goal_pose.copy()
            self.last_goal_timestamp = measurement_timestamp
            self.target_linear_velocity = np.zeros(3)
            self.target_angular_velocity = np.zeros(3)
            return

        sample_dt = measurement_timestamp - self.last_goal_timestamp

        if sample_dt <= 1e-6 or sample_dt > self.config.tracker_timeout:
            self.last_goal_pose = goal_pose.copy()
            self.last_goal_timestamp = measurement_timestamp
            self.target_linear_velocity = np.zeros(3)
            self.target_angular_velocity = np.zeros(3)
            return

        raw_linear_velocity = (
            goal_pose[:3, 3] - self.last_goal_pose[:3, 3]
        ) / sample_dt

        raw_angular_velocity = so3_log(
            self.last_goal_pose[:3, :3].T
            @ goal_pose[:3, :3]
        ) / sample_dt

        raw_linear_velocity = clamp_norm(
            raw_linear_velocity,
            self.max_target_linear_speed,
        )
        raw_angular_velocity = clamp_norm(
            raw_angular_velocity,
            self.max_target_angular_speed,
        )

        alpha = self.target_velocity_filter_alpha

        self.target_linear_velocity = (
            (1.0 - alpha) * self.target_linear_velocity
            + alpha * raw_linear_velocity
        )
        self.target_angular_velocity = (
            (1.0 - alpha) * self.target_angular_velocity
            + alpha * raw_angular_velocity
        )

        self.last_goal_pose = goal_pose.copy()
        self.last_goal_timestamp = measurement_timestamp

    def step(
        self,
        T_BE: np.ndarray | None,
        robot_state_age: float,
        task_pose: TaskPoseMeasurement | None,
        dt: float,
    ) -> tuple[np.ndarray | None, PBVSDiagnostics]:
        if (
            T_BE is None
            or robot_state_age > self.config.panda_state_timeout
        ):
            self.command_pose = None
            self.state = ControllerState.WAIT_FOR_ROBOT
            self.valid_count = 0
            self.last_processed_task_sequence = None
            self._reset_target_velocity_estimator()

            return None, PBVSDiagnostics(
                self.state,
                reason="robot_state_missing_or_stale",
            )

        if task_pose is None:
            self.command_pose = None
            self.last_task_pose = None
            self.last_processed_task_sequence = None
            self.state = ControllerState.WAIT_FOR_TASK_POSE
            self.valid_count = 0
            self._reset_target_velocity_estimator()

            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                reason="task_pose_missing",
            )

        now = time.monotonic()

        if now - task_pose.timestamp > self.config.tracker_timeout:
            self.command_pose = None
            self.last_task_pose = None
            self.last_processed_task_sequence = None
            self.state = ControllerState.HOLD
            self.valid_count = 0
            self._reset_target_velocity_estimator()

            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                reason="task_pose_stale",
            )

        if not self._task_pose_valid(task_pose, now):
            self.command_pose = None
            self.last_processed_task_sequence = None
            self.state = ControllerState.HOLD
            self.valid_count = 0

            if (
                task_pose.valid
                and finite_transform(task_pose.T_TS)
            ):
                # Do not update last_task_pose here.
                # A rejected jump must not become the new baseline.
                pass
            self._reset_target_velocity_estimator()

            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                reason="task_pose_invalid_or_jump",
            )

        T_goal = self._goal_pose(
            T_BE,
            task_pose.T_TS,
        )

        p_error = (
            T_goal[:3, 3]
            - T_BE[:3, 3]
        )

        if self.config.control_orientation:
            r_error = so3_log(
                T_BE[:3, :3].T
                @ T_goal[:3, :3]
            )
        else:
            T_goal[:3, :3] = T_BE[:3, :3]
            r_error = np.zeros(3)

        p_norm = float(np.linalg.norm(p_error))
        r_norm = float(np.linalg.norm(r_error))

        position_error_too_large = (
            p_norm
            > self.config.max_enable_position_error
        )

        orientation_error_too_large = (
            self.config.control_orientation
            and r_norm
            > self.config.max_enable_orientation_error
        )

        if (
            position_error_too_large
            or orientation_error_too_large
        ):
            self.command_pose = None
            self.state = ControllerState.HOLD
            self.valid_count = 0
            self._reset_target_velocity_estimator()

            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                position_error=p_norm,
                orientation_error=r_norm,
                reason="error_exceeds_enable_threshold",
            )
        is_new_measurement = (
            self.last_processed_task_sequence
            != task_pose.sequence_id
        )

        if is_new_measurement:
            self.last_task_pose = task_pose
            self.last_processed_task_sequence = task_pose.sequence_id
            self.valid_count += 1
            self._update_target_velocity_estimate(
                T_goal,
                task_pose.timestamp,
            )

        if (
            self.valid_count
            < self.config.consecutive_valid_required
        ):
            self.command_pose = None
            self.state = ControllerState.READY

            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                position_error=p_norm,
                orientation_error=r_norm,
                reason="waiting_for_consecutive_valid_measurements",
            )
        target_linear_velocity = (
            self.target_linear_velocity
            if self.target_feedforward_enabled
            else np.zeros(3)
        )

        target_angular_velocity = (
            self.target_angular_velocity
            if (
                self.target_feedforward_enabled
                and self.config.control_orientation
            )
            else np.zeros(3)
        )

        linear_velocity = clamp_norm(
            target_linear_velocity
            + self.config.kp_position * p_error,
            self.config.max_linear_speed,
        )

        if self.config.control_orientation:
            angular_velocity = clamp_norm(
                target_angular_velocity
                + self.config.kp_orientation * r_error,
                self.config.max_angular_speed,
            )
        else:
            angular_velocity = np.zeros(3)

        if self.command_pose is None:
            self.command_pose = T_BE.copy()

        command_position = (
            self.command_pose[:3, 3]
            + linear_velocity * dt
        )

        command_lead = (
            command_position
            - T_BE[:3, 3]
        )
        lead_norm = float(np.linalg.norm(command_lead))

        if lead_norm > self.max_command_lead:
            command_lead *= (
                self.max_command_lead
                / lead_norm
            )

        command_position = (
            T_BE[:3, 3]
            + command_lead
        )

        if self.config.control_orientation:
            command_rotation = (
                self.command_pose[:3, :3]
                @ so3_exp(
                    angular_velocity * dt
                )
            )
        else:
            command_rotation = T_BE[:3, :3].copy()

        command = make_transform(
            command_rotation,
            command_position,
        )

        command = clamp_workspace(
            command,
            self.config.workspace_min,
            self.config.workspace_max,
        )

        self.command_pose = command.copy()
        self.state = ControllerState.TRACKING

        return command, PBVSDiagnostics(
            self.state,
            position_error=p_norm,
            orientation_error=r_norm,
            target_linear_speed=float(
                np.linalg.norm(self.target_linear_velocity)
            ),
            target_angular_speed=float(
                np.linalg.norm(self.target_angular_velocity)
            ),
            commanded_linear_speed=float(
                np.linalg.norm(linear_velocity)
            ),
            commanded_angular_speed=float(
                np.linalg.norm(angular_velocity)
            ),
            command_lead=float(
                np.linalg.norm(command_lead)
            ),
        )
