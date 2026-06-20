from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import time
import numpy as np

from common.config import PBVSConfig
from common.geometry import clamp_norm, invert_transform, make_transform, so3_exp, so3_log
from common.safety import clamp_workspace, finite_transform


class ControllerState(Enum):
    WAIT_FOR_ROBOT = auto()
    WAIT_FOR_TRACKER = auto()
    READY = auto()
    TRACKING = auto()
    HOLD = auto()
    FAULT = auto()


@dataclass
class TrackerMeasurement:
    T_TC: np.ndarray
    timestamp: float
    valid: bool = True


@dataclass
class PBVSDiagnostics:
    state: ControllerState
    position_error: float = 0.0
    orientation_error: float = 0.0
    reason: str = ""


class PBVSController:
    def __init__(self, config: PBVSConfig) -> None:
        self.config = config
        self.state = ControllerState.WAIT_FOR_ROBOT
        self.last_tracker: TrackerMeasurement | None = None
        self.valid_count = 0

    def _tracker_valid(
        self,
        measurement: TrackerMeasurement,
        now: float,
    ) -> bool:
        if not measurement.valid or not finite_transform(measurement.T_TC):
            return False
        if now - measurement.timestamp > self.config.tracker_timeout:
            return False

        if self.last_tracker is not None:
            delta = invert_transform(self.last_tracker.T_TC) @ measurement.T_TC
            if np.linalg.norm(delta[:3, 3]) > self.config.max_tracker_position_jump:
                return False
            if np.linalg.norm(so3_log(delta[:3, :3])) > self.config.max_tracker_angle_jump:
                return False

        return True

    def _goal_pose(
        self,
        T_BE: np.ndarray,
        T_TC: np.ndarray,
    ) -> np.ndarray:
        T_CE = invert_transform(self.config.T_EC)
        delta_T_C = invert_transform(T_TC) @ self.config.T_TC_des
        delta_T_E = self.config.T_EC @ delta_T_C @ T_CE
        return T_BE @ delta_T_E

    def step(
        self,
        T_BE: np.ndarray | None,
        robot_state_age: float,
        tracker: TrackerMeasurement | None,
        dt: float,
    ) -> tuple[np.ndarray | None, PBVSDiagnostics]:
        if T_BE is None or robot_state_age > self.config.panda_state_timeout:
            self.state = ControllerState.WAIT_FOR_ROBOT
            return None, PBVSDiagnostics(self.state, reason="robot_state_missing_or_stale")

        if tracker is None:
            self.state = ControllerState.WAIT_FOR_TRACKER
            self.valid_count = 0
            return T_BE.copy(), PBVSDiagnostics(self.state, reason="tracker_missing")

        now = time.monotonic()
        if not self._tracker_valid(tracker, now):
            self.state = ControllerState.HOLD
            self.valid_count = 0
            if tracker.valid and finite_transform(tracker.T_TC):
                self.last_tracker = tracker
            return T_BE.copy(), PBVSDiagnostics(self.state, reason="tracker_invalid_stale_or_jump")

        T_goal = self._goal_pose(T_BE, tracker.T_TC)
        p_error = T_goal[:3, 3] - T_BE[:3, 3]
        r_error = so3_log(T_BE[:3, :3].T @ T_goal[:3, :3])

        p_norm = float(np.linalg.norm(p_error))
        r_norm = float(np.linalg.norm(r_error))

        if (
            p_norm > self.config.max_enable_position_error
            or r_norm > self.config.max_enable_orientation_error
        ):
            self.state = ControllerState.HOLD
            self.valid_count = 0
            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                position_error=p_norm,
                orientation_error=r_norm,
                reason="error_exceeds_enable_threshold",
            )

        self.last_tracker = tracker
        self.valid_count += 1

        if self.valid_count < self.config.consecutive_valid_required:
            self.state = ControllerState.READY
            return T_BE.copy(), PBVSDiagnostics(
                self.state,
                position_error=p_norm,
                orientation_error=r_norm,
                reason="waiting_for_consecutive_valid_measurements",
            )

        linear_velocity = clamp_norm(
            self.config.kp_position * p_error,
            self.config.max_linear_speed,
        )
        angular_velocity = clamp_norm(
            self.config.kp_orientation * r_error,
            self.config.max_angular_speed,
        )

        command = make_transform(
            T_BE[:3, :3] @ so3_exp(angular_velocity * dt),
            T_BE[:3, 3] + linear_velocity * dt,
        )
        command = clamp_workspace(
            command,
            self.config.workspace_min,
            self.config.workspace_max,
        )

        self.state = ControllerState.TRACKING
        return command, PBVSDiagnostics(
            self.state,
            position_error=p_norm,
            orientation_error=r_norm,
        )
