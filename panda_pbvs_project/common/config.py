from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import json
import math
import numpy as np
from common.safety import finite_transform

def _matrix(raw: dict[str, Any], name: str) -> np.ndarray:
    value = np.asarray(raw[name], dtype=float)
    if value.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4.")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} contains invalid values.")
    return value

def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 rigid-body homogeneous transformation matrix."""
    T = np.asarray(T, dtype=float)

    if T.shape != (4, 4):
        raise ValueError("Transform must be 4x4.")
    if not np.all(np.isfinite(T)):
        raise ValueError("Transform contains invalid values.")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError("Transform has an invalid homogeneous bottom row.")

    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4, dtype=float)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -(R.T @ t)

    return T_inv

@dataclass(frozen=True)
class PBVSConfig:
    control_rate_hz: float
    control_orientation: bool
    kp_position: float
    kp_orientation: float
    max_linear_speed: float
    max_angular_speed: float
    panda_state_timeout: float
    tracker_timeout: float
    max_tracker_position_jump: float
    max_tracker_angle_jump: float
    max_enable_position_error: float
    max_enable_orientation_error: float
    consecutive_valid_required: int
    workspace_min: np.ndarray
    workspace_max: np.ndarray
    T_EC: np.ndarray
    T_CS: np.ndarray
    T_TS_des: np.ndarray
    tool_visualization: dict[str, Any]

    @property
    def T_ES(self) -> np.ndarray:
        """Pose of stick-tip frame S expressed in robot EE frame E."""
        return self.T_EC @ self.T_CS

    @property
    def T_TC_des(self) -> np.ndarray:
        """Desired camera pose C expressed in target frame T."""
        return self.T_TS_des @ invert_transform(self.T_CS)


def load_pbvs_config(path: Path) -> PBVSConfig:
    raw = json.loads(path.read_text())
    workspace = raw.get("workspace", {})
    workspace_min = np.asarray(
        workspace.get("min", [-1.0, -1.0, -1.0]),
        dtype=float,
    )
    workspace_max = np.asarray(
        workspace.get("max", [1.0, 1.0, 1.0]),
        dtype=float,
    )

    config = PBVSConfig(
        control_rate_hz=float(raw["control_rate_hz"]),
        control_orientation=bool(raw.get("control_orientation", True)),
        kp_position=float(raw["kp_position"]),
        kp_orientation=float(raw["kp_orientation"]),
        max_linear_speed=float(raw["max_linear_speed"]),
        max_angular_speed=math.radians(
            float(raw["max_angular_speed_deg"])
        ),
        panda_state_timeout=float(raw["panda_state_timeout"]),
        tracker_timeout=float(raw["tracker_timeout"]),
        max_tracker_position_jump=float(
            raw["max_tracker_position_jump"]
        ),
        max_tracker_angle_jump=math.radians(
            float(raw["max_tracker_angle_jump_deg"])
        ),
        max_enable_position_error=float(
            raw["max_enable_position_error"]
        ),
        max_enable_orientation_error=math.radians(
            float(raw["max_enable_orientation_error_deg"])
        ),
        consecutive_valid_required=int(
            raw["consecutive_valid_required"]
        ),
        workspace_min=workspace_min,
        workspace_max=workspace_max,
        T_EC=_matrix(raw, "T_EC"),
        T_CS=_matrix(raw, "T_CS"),
        T_TS_des=_matrix(raw, "T_TS_des"),
        tool_visualization=dict(
            raw.get("tool_visualization", {})
        ),
    )

    if not finite_transform(config.T_ES):
        raise ValueError(
            "Derived T_ES = T_EC @ T_CS is not a valid "
            "homogeneous transform."
        )

    return config
