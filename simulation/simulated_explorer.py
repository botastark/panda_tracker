#!/usr/bin/env python3
"""
Kinematic Panda simulator that mirrors explorer_safe's UDP interface.

Receives:
    port 2600, <6f: x, y, z, roll, pitch, yaw in Panda base frame

Sends:
    port 6200, <6f: simulated EE state
    port 6500, <16d: synthetic tracker pose T_TC

The simulated EE follows the commanded Cartesian pose with damped least-squares
inverse kinematics. This is a commissioning simulator, not a torque-accurate
digital twin.

Keyboard controls for the triangle:
    W/S: triangle +/- x_B
    A/D: triangle +/- y_B
    R/F: triangle +/- z_B
    I/K: triangle roll +/-
    J/L: triangle pitch +/-
    U/O: triangle yaw +/-
    0: reset triangle
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)
TRACKER_FORMAT = "<16d"


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_log(rotation: np.ndarray) -> np.ndarray:
    cos_theta = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(float(cos_theta))
    if theta < 1e-8:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])
    return theta / (2.0 * math.sin(theta)) * np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rpy_from_rotation(rotation: np.ndarray) -> np.ndarray:
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def make_transform(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def quat_wxyz_from_rotation(rotation: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    return quaternion


def load_transform_config(path: Path) -> np.ndarray:
    raw = json.loads(path.read_text())
    transform = np.asarray(raw["T_EC"], dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("T_EC in the PBVS config must be 4x4.")
    return transform


def create_scene_xml(panda_xml: Path) -> Path:
    """
    Inject a floor, light, mocap triangle, camera/tool placeholders, and an EE
    marker into the Panda model's existing worldbody.

    The generated XML is saved beside panda.xml so relative asset paths remain
    valid.
    """
    source = panda_xml.read_text()
    marker = "</worldbody>"
    if marker not in source:
        raise ValueError(f"{panda_xml} has no </worldbody> tag.")

    additions = r"""
      <light name="pbvs_light" pos="0 0 2" dir="0 0 -1"/>
      <geom name="pbvs_floor" type="plane" size="2 2 .1" rgba=".85 .85 .85 1"/>

      <body name="pbvs_triangle" mocap="true" pos=".5 0 .4">
        <geom type="box" size=".12 .09 .006" rgba=".15 .65 .20 .45"
              contype="0" conaffinity="0"/>
        <geom type="sphere" size=".012" pos="0 .055 .01" rgba="1 .1 .1 1"
              contype="0" conaffinity="0"/>
        <geom type="sphere" size=".012" pos="-.05 -.035 .01" rgba="1 .1 .1 1"
              contype="0" conaffinity="0"/>
        <geom type="sphere" size=".012" pos=".05 -.035 .01" rgba="1 .1 .1 1"
              contype="0" conaffinity="0"/>
        <geom type="capsule" fromto="0 0 0 .08 0 0" size=".004" rgba="1 0 0 1"
              contype="0" conaffinity="0"/>
        <geom type="capsule" fromto="0 0 0 0 .08 0" size=".004" rgba="0 1 0 1"
              contype="0" conaffinity="0"/>
        <geom type="capsule" fromto="0 0 0 0 0 .08" size=".004" rgba="0 0 1 1"
              contype="0" conaffinity="0"/>
      </body>
    """
    generated = source.replace(marker, additions + "\n" + marker, 1)
    output = panda_xml.parent / "_pbvs_generated_panda.xml"
    output.write_text(generated)
    return output


class LatestPoseCommand:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[np.ndarray] = None
        self._time = 0.0

    def set(self, pose: np.ndarray) -> None:
        with self._lock:
            self._pose = pose.copy()
            self._time = time.monotonic()

    def get(self) -> tuple[Optional[np.ndarray], float]:
        with self._lock:
            return (
                None if self._pose is None else self._pose.copy(),
                self._time,
            )


class CommandReceiver(threading.Thread):
    def __init__(self, bind_ip: str, port: int, latest: LatestPoseCommand) -> None:
        super().__init__(daemon=True)
        self.bind_ip = bind_ip
        self.port = port
        self.latest = latest

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.bind_ip, self.port))
        while True:
            data, _ = sock.recvfrom(1024)
            if len(data) != POSE_SIZE:
                continue
            pose = np.asarray(struct.unpack(POSE_FORMAT, data), dtype=float)
            if np.all(np.isfinite(pose)):
                self.latest.set(pose)


@dataclass
class ArmJoint:
    joint_id: int
    qpos_address: int
    dof_address: int
    minimum: float
    maximum: float


def discover_arm_joints(model: mujoco.MjModel) -> list[ArmJoint]:
    joints: list[ArmJoint] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
        if name.endswith(tuple(str(i) for i in range(1, 8))) or "joint" in name.lower():
            qpos_address = int(model.jnt_qposadr[joint_id])
            dof_address = int(model.jnt_dofadr[joint_id])
            minimum, maximum = model.jnt_range[joint_id]
            joints.append(
                ArmJoint(
                    joint_id,
                    qpos_address,
                    dof_address,
                    float(minimum),
                    float(maximum),
                )
            )

    joints.sort(key=lambda joint: joint.dof_address)
    if len(joints) < 7:
        raise RuntimeError(
            f"Found only {len(joints)} candidate hinge joints. "
            "Run inspect_model.py and pass a compatible Panda model."
        )
    return joints[:7]


def body_transform(data: mujoco.MjData, body_id: int) -> np.ndarray:
    return make_transform(
        data.xmat[body_id].reshape(3, 3).copy(),
        data.xpos[body_id].copy(),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--panda-xml",
        type=Path,
        required=True,
        help="Path to mujoco_menagerie/franka_emika_panda/panda.xml.",
    )
    parser.add_argument(
        "--pbvs-config",
        type=Path,
        required=True,
        help="PBVS JSON config containing T_EC.",
    )
    parser.add_argument("--ee-body", default="hand")
    parser.add_argument("--command-bind-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=2600)
    parser.add_argument("--state-ip", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=6200)
    parser.add_argument("--tracker-ip", default="127.0.0.1")
    parser.add_argument("--tracker-port", type=int, default=6500)
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--max-joint-speed", type=float, default=0.4)
    parser.add_argument("--kp-position", type=float, default=3.0)
    parser.add_argument("--kp-orientation", type=float, default=2.0)
    parser.add_argument("--command-timeout", type=float, default=0.25)
    parser.add_argument("--triangle-step", type=float, default=0.01)
    parser.add_argument("--triangle-rotation-step-deg", type=float, default=2.0)
    args = parser.parse_args()

    panda_xml = args.panda_xml.expanduser().resolve()
    scene_xml = create_scene_xml(panda_xml)
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, args.ee_body)
    if ee_body_id < 0:
        raise ValueError(
            f"EE body {args.ee_body!r} not found. Run inspect_model.py."
        )
    triangle_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "pbvs_triangle"
    )
    joints = discover_arm_joints(model)
    T_EC = load_transform_config(args.pbvs_config)

    latest_command = LatestPoseCommand()
    CommandReceiver(args.command_bind_ip, args.command_port, latest_command).start()

    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tracker_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    triangle_initial_pos = np.array([0.5, 0.0, 0.4])
    triangle_pos = triangle_initial_pos.copy()
    triangle_rpy = np.array([0.0, math.pi, math.pi])
    triangle_initial_rpy = triangle_rpy.copy()

    def key_callback(keycode: int) -> None:
        key = chr(keycode).upper() if 0 <= keycode < 256 else ""
        dr = math.radians(args.triangle_rotation_step_deg)
        if key == "W":
            triangle_pos[0] += args.triangle_step
        elif key == "S":
            triangle_pos[0] -= args.triangle_step
        elif key == "A":
            triangle_pos[1] += args.triangle_step
        elif key == "D":
            triangle_pos[1] -= args.triangle_step
        elif key == "R":
            triangle_pos[2] += args.triangle_step
        elif key == "F":
            triangle_pos[2] -= args.triangle_step
        elif key == "I":
            triangle_rpy[0] += dr
        elif key == "K":
            triangle_rpy[0] -= dr
        elif key == "J":
            triangle_rpy[1] += dr
        elif key == "L":
            triangle_rpy[1] -= dr
        elif key == "U":
            triangle_rpy[2] += dr
        elif key == "O":
            triangle_rpy[2] -= dr
        elif key == "0":
            triangle_pos[:] = triangle_initial_pos
            triangle_rpy[:] = triangle_initial_rpy

    target_transform = body_transform(data, ee_body_id)
    previous_time = time.monotonic()
    last_state_send = 0.0
    last_tracker_send = 0.0
    last_status = 0.0

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    print("Simulation started.")
    print("Triangle keys: W/S A/D R/F, I/K J/L U/O, 0 reset.")
    print(f"EE body: {args.ee_body}; generated scene: {scene_xml}")

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        while viewer.is_running():
            now = time.monotonic()
            dt = min(max(now - previous_time, 1e-4), 0.02)
            previous_time = now

            # Update triangle mocap body.
            data.mocap_pos[0] = triangle_pos
            data.mocap_quat[0] = quat_wxyz_from_rotation(
                rotation_from_rpy(*triangle_rpy)
            )
            mujoco.mj_forward(model, data)

            command, command_time = latest_command.get()
            if command is not None and now - command_time <= args.command_timeout:
                target_transform = make_transform(
                    rotation_from_rpy(command[3], command[4], command[5]),
                    command[:3],
                )
            else:
                # Hold current EE pose after timeout.
                target_transform = body_transform(data, ee_body_id)

            T_BE = body_transform(data, ee_body_id)
            position_error = target_transform[:3, 3] - T_BE[:3, 3]
            orientation_error = so3_log(
                target_transform[:3, :3] @ T_BE[:3, :3].T
            )
            twist = np.concatenate([
                args.kp_position * position_error,
                args.kp_orientation * orientation_error,
            ])

            mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
            jacobian = np.vstack([jacp, jacr])
            arm_columns = [joint.dof_address for joint in joints]
            arm_jacobian = jacobian[:, arm_columns]

            regularized = (
                arm_jacobian @ arm_jacobian.T
                + (args.damping ** 2) * np.eye(6)
            )
            qdot = arm_jacobian.T @ np.linalg.solve(regularized, twist)
            qdot = np.clip(
                qdot,
                -args.max_joint_speed,
                args.max_joint_speed,
            )

            for joint, velocity in zip(joints, qdot):
                data.qpos[joint.qpos_address] += float(velocity) * dt
                margin = 0.01
                data.qpos[joint.qpos_address] = np.clip(
                    data.qpos[joint.qpos_address],
                    joint.minimum + margin,
                    joint.maximum - margin,
                )

            mujoco.mj_forward(model, data)
            T_BE = body_transform(data, ee_body_id)

            # Simulated state packet.
            if now - last_state_send >= 0.002:
                rpy = rpy_from_rotation(T_BE[:3, :3])
                state = np.concatenate([T_BE[:3, 3], rpy])
                state_socket.sendto(
                    struct.pack(POSE_FORMAT, *state.astype(np.float32)),
                    (args.state_ip, args.state_port),
                )
                last_state_send = now

            # Synthetic event tracker measurement.
            if now - last_tracker_send >= 0.01:
                T_BC = T_BE @ T_EC
                T_BT = body_transform(data, triangle_body_id)
                T_TC = np.linalg.inv(T_BT) @ T_BC
                tracker_socket.sendto(
                    struct.pack(TRACKER_FORMAT, *T_TC.reshape(-1)),
                    (args.tracker_ip, args.tracker_port),
                )
                last_tracker_send = now

            if now - last_status >= 1.0:
                smallest_singular_value = np.linalg.svd(
                    arm_jacobian, compute_uv=False
                )[-1]
                print(
                    f"|e_p|={np.linalg.norm(position_error):.4f} m, "
                    f"|e_R|={math.degrees(np.linalg.norm(orientation_error)):.2f} deg, "
                    f"sigma_min={smallest_singular_value:.4f}"
                )
                last_status = now

            viewer.sync()
            time.sleep(max(model.opt.timestep, 0.001))

    state_socket.close()
    tracker_socket.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
