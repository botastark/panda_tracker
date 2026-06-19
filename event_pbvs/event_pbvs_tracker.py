#!/usr/bin/env python3
"""
event_pbvs_tracker.py

Outer-loop pose-based visual servoing (PBVS) controller for a Franka Panda.

Inputs
------
1. Panda EE state from explorer_safe on UDP port 6200:
       <6f = x, y, z, roll, pitch, yaw
   Position is in metres; angles are radians.
   Euler convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

2. Event tracker pose T_TC:
       pose of camera frame C expressed in triangle frame T.

Output
------
Absolute Panda EE pose command sent to explorer_safe on UDP port 2600:
       <6f = x, y, z, roll, pitch, yaw

The tracker adapter is intentionally isolated in TrackerPoseSource.
Replace DummyTrackerPoseSource with your actual event-pose receiver.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Protocol, Tuple

import numpy as np


PAYLOAD_FORMAT = "<6f"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)


# ---------------------------------------------------------------------------
# Geometry utilities
# ---------------------------------------------------------------------------

def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [[0.0, -z, y],
         [z, 0.0, -x],
         [-y, x, 0.0]],
        dtype=float,
    )


def project_to_so3(rotation: np.ndarray) -> np.ndarray:
    """Return the nearest proper rotation matrix."""
    u, _, vt = np.linalg.svd(np.asarray(rotation, dtype=float).reshape(3, 3))
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def so3_exp(rotation_vector: np.ndarray) -> np.ndarray:
    phi = np.asarray(rotation_vector, dtype=float).reshape(3)
    theta = np.linalg.norm(phi)
    if theta < 1e-10:
        return np.eye(3) + skew(phi)

    axis = phi / theta
    axis_hat = skew(axis)
    return (
        np.eye(3)
        + math.sin(theta) * axis_hat
        + (1.0 - math.cos(theta)) * (axis_hat @ axis_hat)
    )


def so3_log(rotation: np.ndarray) -> np.ndarray:
    rotation = project_to_so3(rotation)
    cos_theta = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    theta = math.acos(float(cos_theta))

    if theta < 1e-8:
        return 0.5 * np.array(
            [rotation[2, 1] - rotation[1, 2],
             rotation[0, 2] - rotation[2, 0],
             rotation[1, 0] - rotation[0, 1]]
        )

    if math.pi - theta < 1e-5:
        # More stable extraction near pi.
        diagonal = np.maximum((np.diag(rotation) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        if rotation[2, 1] - rotation[1, 2] < 0.0:
            axis[0] *= -1.0
        if rotation[0, 2] - rotation[2, 0] < 0.0:
            axis[1] *= -1.0
        if rotation[1, 0] - rotation[0, 1] < 0.0:
            axis[2] *= -1.0
        norm = np.linalg.norm(axis)
        if norm < 1e-8:
            raise ValueError("Cannot extract a stable rotation axis near pi.")
        return theta * axis / norm

    factor = theta / (2.0 * math.sin(theta))
    return factor * np.array(
        [rotation[2, 1] - rotation[1, 2],
         rotation[0, 2] - rotation[2, 0],
         rotation[1, 0] - rotation[0, 1]]
    )


def rpy_zyx_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def rotation_to_rpy_zyx(rotation: np.ndarray) -> Tuple[float, float, float]:
    rotation = project_to_so3(rotation)
    pitch = math.atan2(
        -rotation[2, 0],
        math.hypot(rotation[0, 0], rotation[1, 0]),
    )

    if abs(abs(pitch) - math.pi / 2.0) < 1e-6:
        # Gimbal-lock fallback. Keep yaw at zero and absorb into roll.
        yaw = 0.0
        roll = math.atan2(-rotation[0, 1], rotation[1, 1])
    else:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])

    return roll, pitch, yaw


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = project_to_so3(rotation)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def pose6_to_transform(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float).reshape(6)
    return make_transform(
        rpy_zyx_to_rotation(pose[3], pose[4], pose[5]),
        pose[:3],
    )


def transform_to_pose6(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    roll, pitch, yaw = rotation_to_rpy_zyx(transform[:3, :3])
    return np.array(
        [transform[0, 3], transform[1, 3], transform[2, 3],
         roll, pitch, yaw],
        dtype=float,
    )


def clamp_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm <= maximum_norm or norm < 1e-12:
        return vector
    return vector * (maximum_norm / norm)


def finite_transform(transform: np.ndarray) -> bool:
    transform = np.asarray(transform)
    return (
        transform.shape == (4, 4)
        and np.all(np.isfinite(transform))
        and abs(transform[3, 3] - 1.0) < 1e-6
    )


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ControllerConfig:
    panda_ip: str
    panda_command_port: int
    panda_state_bind_ip: str
    panda_state_port: int

    control_rate_hz: float
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

    dry_run: bool

    T_EC: np.ndarray
    T_CS: np.ndarray
    T_TS_des: np.ndarray

    @property
    def T_TC_des(self) -> np.ndarray:
        return self.T_TS_des @ invert_transform(self.T_CS)


def load_config(path: Path) -> ControllerConfig:
    raw = json.loads(path.read_text())

    def matrix(name: str) -> np.ndarray:
        value = np.asarray(raw[name], dtype=float)
        if value.shape != (4, 4):
            raise ValueError(f"{name} must be a 4x4 matrix.")
        if not finite_transform(value):
            raise ValueError(f"{name} is not a valid finite homogeneous transform.")
        value = value.copy()
        value[:3, :3] = project_to_so3(value[:3, :3])
        return value

    return ControllerConfig(
        panda_ip=raw["panda_ip"],
        panda_command_port=int(raw["panda_command_port"]),
        panda_state_bind_ip=raw.get("panda_state_bind_ip", "0.0.0.0"),
        panda_state_port=int(raw["panda_state_port"]),
        control_rate_hz=float(raw["control_rate_hz"]),
        kp_position=float(raw["kp_position"]),
        kp_orientation=float(raw["kp_orientation"]),
        max_linear_speed=float(raw["max_linear_speed"]),
        max_angular_speed=math.radians(float(raw["max_angular_speed_deg"])),
        panda_state_timeout=float(raw["panda_state_timeout"]),
        tracker_timeout=float(raw["tracker_timeout"]),
        max_tracker_position_jump=float(raw["max_tracker_position_jump"]),
        max_tracker_angle_jump=math.radians(float(raw["max_tracker_angle_jump_deg"])),
        max_enable_position_error=float(raw["max_enable_position_error"]),
        max_enable_orientation_error=math.radians(
            float(raw["max_enable_orientation_error_deg"])
        ),
        consecutive_valid_required=int(raw["consecutive_valid_required"]),
        dry_run=bool(raw.get("dry_run", True)),
        T_EC=matrix("T_EC"),
        T_CS=matrix("T_CS"),
        T_TS_des=matrix("T_TS_des"),
    )


# ---------------------------------------------------------------------------
# Thread-safe measurements
# ---------------------------------------------------------------------------

@dataclass
class PandaState:
    pose6: np.ndarray
    arrival_time: float


@dataclass
class TrackerMeasurement:
    T_TC: np.ndarray
    measurement_time: float
    valid: bool


class LatestValue:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = None

    def set(self, value) -> None:
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


class PandaStateReceiver(threading.Thread):
    def __init__(self, bind_ip: str, port: int, output: LatestValue) -> None:
        super().__init__(daemon=True)
        self._bind_ip = bind_ip
        self._port = port
        self._output = output
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind_ip, self._port))
        sock.settimeout(0.1)

        try:
            while not self._stop_event.is_set():
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    continue

                # Drain queued packets and retain the newest state.
                sock.setblocking(False)
                latest = data
                try:
                    while True:
                        latest, _ = sock.recvfrom(1024)
                except BlockingIOError:
                    pass
                finally:
                    sock.setblocking(True)
                    sock.settimeout(0.1)

                if len(latest) != PAYLOAD_SIZE:
                    continue

                pose6 = np.asarray(struct.unpack(PAYLOAD_FORMAT, latest), dtype=float)
                if np.all(np.isfinite(pose6)):
                    self._output.set(
                        PandaState(pose6=pose6, arrival_time=time.monotonic())
                    )
        finally:
            sock.close()


class TrackerPoseSource(Protocol):
    def get_latest(self) -> Optional[TrackerMeasurement]:
        """Return the latest tracker measurement, or None if unavailable."""


class DummyTrackerPoseSource:
    """
    Placeholder tracker source.

    This deliberately returns None so the robot remains in HOLD.
    Replace this class with the actual event tracker adapter.
    """

    def get_latest(self) -> Optional[TrackerMeasurement]:
        return None


class UdpMatrixTrackerPoseSource(threading.Thread):
    """
    Optional reference adapter.

    Expects 16 little-endian float64 values containing T_TC in row-major order.
    Packet format: <16d

    This is only a transport example. Adapt it to the event tracker's real API.
    """

    FORMAT = "<16d"
    SIZE = struct.calcsize(FORMAT)

    def __init__(self, bind_ip: str, port: int) -> None:
        super().__init__(daemon=True)
        self._bind_ip = bind_ip
        self._port = port
        self._latest = LatestValue()
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def get_latest(self) -> Optional[TrackerMeasurement]:
        return self._latest.get()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind_ip, self._port))
        sock.settimeout(0.1)

        try:
            while not self._stop_event.is_set():
                try:
                    data, _ = sock.recvfrom(2048)
                except socket.timeout:
                    continue

                if len(data) != self.SIZE:
                    continue

                matrix = np.asarray(
                    struct.unpack(self.FORMAT, data), dtype=float
                ).reshape(4, 4)

                valid = finite_transform(matrix)
                if valid:
                    matrix = matrix.copy()
                    matrix[:3, :3] = project_to_so3(matrix[:3, :3])

                self._latest.set(
                    TrackerMeasurement(
                        T_TC=matrix,
                        measurement_time=time.monotonic(),
                        valid=valid,
                    )
                )
        finally:
            sock.close()


# ---------------------------------------------------------------------------
# PBVS controller
# ---------------------------------------------------------------------------

class ControlState(Enum):
    WAIT_FOR_PANDA = auto()
    WAIT_FOR_TRACKER = auto()
    READY = auto()
    TRACKING = auto()
    HOLD = auto()


class PBVSController:
    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.state = ControlState.WAIT_FOR_PANDA
        self._last_tracker: Optional[TrackerMeasurement] = None
        self._consecutive_valid = 0

    def tracker_measurement_is_valid(
        self,
        measurement: TrackerMeasurement,
        now: float,
    ) -> bool:
        if not measurement.valid or not finite_transform(measurement.T_TC):
            return False
        if now - measurement.measurement_time > self.config.tracker_timeout:
            return False

        if self._last_tracker is not None:
            delta = invert_transform(self._last_tracker.T_TC) @ measurement.T_TC
            translation_jump = np.linalg.norm(delta[:3, 3])
            angle_jump = np.linalg.norm(so3_log(delta[:3, :3]))

            if translation_jump > self.config.max_tracker_position_jump:
                return False
            if angle_jump > self.config.max_tracker_angle_jump:
                return False

        return True

    def compute_goal(
        self,
        T_BE_state: np.ndarray,
        T_TC_meas: np.ndarray,
    ) -> np.ndarray:
        T_CE = invert_transform(self.config.T_EC)
        delta_T_C = invert_transform(T_TC_meas) @ self.config.T_TC_des
        delta_T_E = self.config.T_EC @ delta_T_C @ T_CE
        return T_BE_state @ delta_T_E

    def compute_bounded_command(
        self,
        T_BE_state: np.ndarray,
        T_BE_goal: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, float, float]:
        position = T_BE_state[:3, 3]
        goal_position = T_BE_goal[:3, 3]
        position_error = goal_position - position

        rotation = T_BE_state[:3, :3]
        goal_rotation = T_BE_goal[:3, :3]
        body_rotation_error = so3_log(rotation.T @ goal_rotation)

        linear_velocity = clamp_norm(
            self.config.kp_position * position_error,
            self.config.max_linear_speed,
        )
        angular_velocity = clamp_norm(
            self.config.kp_orientation * body_rotation_error,
            self.config.max_angular_speed,
        )

        command_position = position + linear_velocity * dt
        command_rotation = rotation @ so3_exp(angular_velocity * dt)
        command = make_transform(command_rotation, command_position)

        return (
            command,
            float(np.linalg.norm(position_error)),
            float(np.linalg.norm(body_rotation_error)),
        )

    def step(
        self,
        panda_state: Optional[PandaState],
        tracker_measurement: Optional[TrackerMeasurement],
        now: float,
        dt: float,
    ) -> Tuple[Optional[np.ndarray], dict]:
        diagnostics = {"state": self.state.name}

        if panda_state is None or now - panda_state.arrival_time > self.config.panda_state_timeout:
            self.state = ControlState.WAIT_FOR_PANDA
            diagnostics["state"] = self.state.name
            diagnostics["reason"] = "missing_or_stale_panda_state"
            return None, diagnostics

        T_BE_state = pose6_to_transform(panda_state.pose6)

        if tracker_measurement is None:
            self.state = ControlState.WAIT_FOR_TRACKER
            self._consecutive_valid = 0
            diagnostics["state"] = self.state.name
            diagnostics["reason"] = "no_tracker_pose"
            return T_BE_state, diagnostics

        if not self.tracker_measurement_is_valid(tracker_measurement, now):
            self.state = ControlState.HOLD
            self._consecutive_valid = 0
            diagnostics["state"] = self.state.name
            diagnostics["reason"] = "invalid_stale_or_jumping_tracker_pose"
            return T_BE_state, diagnostics

        T_BE_goal = self.compute_goal(T_BE_state, tracker_measurement.T_TC)
        position_error = np.linalg.norm(T_BE_goal[:3, 3] - T_BE_state[:3, 3])
        orientation_error = np.linalg.norm(
            so3_log(T_BE_state[:3, :3].T @ T_BE_goal[:3, :3])
        )

        diagnostics["position_error_m"] = float(position_error)
        diagnostics["orientation_error_deg"] = math.degrees(float(orientation_error))

        if (
            position_error > self.config.max_enable_position_error
            or orientation_error > self.config.max_enable_orientation_error
        ):
            self.state = ControlState.HOLD
            self._consecutive_valid = 0
            diagnostics["state"] = self.state.name
            diagnostics["reason"] = "initial_or_current_error_exceeds_enable_threshold"
            return T_BE_state, diagnostics

        self._consecutive_valid += 1
        self._last_tracker = tracker_measurement

        if self._consecutive_valid < self.config.consecutive_valid_required:
            self.state = ControlState.READY
            diagnostics["state"] = self.state.name
            diagnostics["reason"] = "collecting_consecutive_valid_measurements"
            return T_BE_state, diagnostics

        self.state = ControlState.TRACKING
        command, position_error, orientation_error = self.compute_bounded_command(
            T_BE_state, T_BE_goal, dt
        )
        diagnostics["state"] = self.state.name
        diagnostics["position_error_m"] = position_error
        diagnostics["orientation_error_deg"] = math.degrees(orientation_error)
        return command, diagnostics


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def send_pose(sock: socket.socket, destination: Tuple[str, int], transform: np.ndarray) -> None:
    pose6 = transform_to_pose6(transform)
    if not np.all(np.isfinite(pose6)):
        raise ValueError("Refusing to send a non-finite Panda command.")
    sock.sendto(struct.pack(PAYLOAD_FORMAT, *pose6.astype(np.float32)), destination)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Event-camera PBVS outer loop")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("pbvs_config.json"),
        help="Path to JSON controller/calibration configuration.",
    )
    parser.add_argument(
        "--tracker-udp-port",
        type=int,
        default=None,
        help="Use the reference <16d UDP tracker adapter on this local port.",
    )
    parser.add_argument(
        "--tracker-bind-ip",
        default="0.0.0.0",
        help="Bind address for the optional tracker UDP adapter.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)

    panda_latest = LatestValue()
    panda_receiver = PandaStateReceiver(
        config.panda_state_bind_ip,
        config.panda_state_port,
        panda_latest,
    )
    panda_receiver.start()

    tracker_source: TrackerPoseSource
    tracker_thread = None
    if args.tracker_udp_port is None:
        tracker_source = DummyTrackerPoseSource()
        print("Tracker adapter is in dummy HOLD mode.")
    else:
        tracker_thread = UdpMatrixTrackerPoseSource(
            args.tracker_bind_ip,
            args.tracker_udp_port,
        )
        tracker_thread.start()
        tracker_source = tracker_thread
        print(
            f"Receiving T_TC as <16d row-major matrices on "
            f"{args.tracker_bind_ip}:{args.tracker_udp_port}"
        )

    controller = PBVSController(config)
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (config.panda_ip, config.panda_command_port)

    period = 1.0 / config.control_rate_hz
    previous_time = time.monotonic()
    next_tick = previous_time
    last_print = 0.0

    print(
        f"PBVS loop started at {config.control_rate_hz:.1f} Hz; "
        f"dry_run={config.dry_run}"
    )
    print("Press Ctrl-C to stop. explorer_safe watchdog will hold the robot.")

    try:
        while True:
            now = time.monotonic()
            dt = now - previous_time
            previous_time = now
            if not math.isfinite(dt) or dt <= 0.0 or dt > 0.1:
                dt = period

            command, diagnostics = controller.step(
                panda_latest.get(),
                tracker_source.get_latest(),
                now,
                dt,
            )

            if command is not None and not config.dry_run:
                send_pose(command_socket, destination, command)

            if now - last_print >= 0.5:
                suffix = " [DRY RUN]" if config.dry_run else ""
                print(json.dumps(diagnostics, sort_keys=True) + suffix)
                last_print = now

            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopping PBVS publisher.")
    finally:
        panda_receiver.stop()
        if tracker_thread is not None:
            tracker_thread.stop()
        command_socket.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
