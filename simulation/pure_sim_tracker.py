#!/usr/bin/env python3
"""
Visual transform sandbox for T_TC without a Panda model.

A triangle frame is fixed at the world origin. A camera frame is shown as a
small blue housing. Move the camera with the keyboard and stream T_TC as <16d.

Controls:
    W/S: camera +/- x_T
    A/D: camera +/- y_T
    R/F: camera +/- z_T
    I/K: roll +/-
    J/L: pitch +/-
    U/O: yaw +/-
    0: reset
    Q or Esc: close

Red/green/blue capsules visualize +x/+y/+z axes.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time

import mujoco
import mujoco.viewer
import numpy as np


TRACKER_FORMAT = "<16d"


def quat_wxyz_from_rotation(rotation: np.ndarray) -> np.ndarray:
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
    return quat


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


XML = r"""
<mujoco model="PBVS transform sandbox">
  <option timestep="0.002" gravity="0 0 0"/>
  <visual>
    <global azimuth="135" elevation="-20"/>
  </visual>
  <worldbody>
    <light pos="0 0 2"/>
    <geom type="plane" size="2 2 0.1" rgba=".85 .85 .85 1"/>

    <body name="triangle" pos="0 0 0.3">
      <geom type="box" size=".12 .09 .006" rgba=".15 .65 .20 .45"/>
      <geom type="sphere" size=".012" pos="0 .055 .01" rgba="1 .1 .1 1"/>
      <geom type="sphere" size=".012" pos="-.05 -.035 .01" rgba="1 .1 .1 1"/>
      <geom type="sphere" size=".012" pos=".05 -.035 .01" rgba="1 .1 .1 1"/>
      <site name="triangle_frame" type="sphere" size=".008" rgba="1 1 1 1"/>
      <geom type="capsule" fromto="0 0 0 .08 0 0" size=".004" rgba="1 0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 .08 0" size=".004" rgba="0 1 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 0 .08" size=".004" rgba="0 0 1 1"/>
    </body>

    <body name="camera" mocap="true" pos="0 0 .70">
      <geom type="box" size=".035 .025 .02" rgba=".15 .3 .9 1"/>
      <site name="camera_frame" type="sphere" size=".008" rgba="1 1 1 1"/>
      <geom type="capsule" fromto="0 0 0 .08 0 0" size=".004" rgba="1 0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 .08 0" size=".004" rgba="0 1 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 0 .08" size=".004" rgba="0 0 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tracker-ip", default="127.0.0.1")
    parser.add_argument("--tracker-port", type=int, default=6500)
    parser.add_argument("--translation-step", type=float, default=0.01)
    parser.add_argument("--rotation-step-deg", type=float, default=2.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    initial_pos = np.array([0.0, 0.0, 0.70])
    camera_pos = initial_pos.copy()
    camera_rpy = np.zeros(3)

    def reset() -> None:
        camera_pos[:] = initial_pos
        camera_rpy[:] = 0.0

    def key_callback(keycode: int) -> None:
        nonlocal camera_pos, camera_rpy
        key = chr(keycode).upper() if 0 <= keycode < 256 else ""
        sign = 1.0
        if 0 <= keycode < 256 and chr(keycode).islower():
            sign = 1.0

        if key == "W":
            camera_pos[0] += args.translation_step
        elif key == "S":
            camera_pos[0] -= args.translation_step
        elif key == "A":
            camera_pos[1] += args.translation_step
        elif key == "D":
            camera_pos[1] -= args.translation_step
        elif key == "R":
            camera_pos[2] += args.translation_step
        elif key == "F":
            camera_pos[2] -= args.translation_step
        elif key == "I":
            camera_rpy[0] += math.radians(args.rotation_step_deg)
        elif key == "K":
            camera_rpy[0] -= math.radians(args.rotation_step_deg)
        elif key == "J":
            camera_rpy[1] += math.radians(args.rotation_step_deg)
        elif key == "L":
            camera_rpy[1] -= math.radians(args.rotation_step_deg)
        elif key == "U":
            camera_rpy[2] += math.radians(args.rotation_step_deg)
        elif key == "O":
            camera_rpy[2] -= math.radians(args.rotation_step_deg)
        elif key == "0":
            reset()

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            data.mocap_pos[0] = camera_pos
            data.mocap_quat[0] = quat_wxyz_from_rotation(rotation_from_rpy(*camera_rpy))
            mujoco.mj_forward(model, data)

            triangle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "triangle")
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera")

            T_BT = np.eye(4)
            T_BT[:3, :3] = data.xmat[triangle_id].reshape(3, 3)
            T_BT[:3, 3] = data.xpos[triangle_id]

            T_BC = np.eye(4)
            T_BC[:3, :3] = data.xmat[camera_id].reshape(3, 3)
            T_BC[:3, 3] = data.xpos[camera_id]

            T_TC = np.linalg.inv(T_BT) @ T_BC
            sock.sendto(
                struct.pack(TRACKER_FORMAT, *T_TC.reshape(-1)),
                (args.tracker_ip, args.tracker_port),
            )

            viewer.sync()
            time.sleep(model.opt.timestep)

    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
