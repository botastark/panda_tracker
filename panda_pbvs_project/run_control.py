#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from backends.mujoco_udp import MujocoUdpBackend
from backends.panda_udp import PandaUdpBackend
from common.config import load_pbvs_config
from common.geometry import invert_transform
from control.pbvs_controller import PBVSController
from perception.task_pose_udp import TaskPoseUdpSource


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PBVS controller for simulation or Panda"
    )
    parser.add_argument(
        "--backend",
        choices=("sim", "panda"),
        required=True,
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--tracker-bind-ip",
        default="0.0.0.0",
        help="Local IP for receiving T_TS UDP packets.",
    )
    parser.add_argument(
        "--tracker-port",
        type=int,
        default=6501,
        help="UDP port for receiving T_TS packets.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute commands but do not send them.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    raw = json.loads(args.config.read_text())
    config = load_pbvs_config(args.config)

    print("Configured T_ES:")
    print(
        np.array2string(
            config.T_ES,
            precision=6,
            suppress_small=True,
        )
    )

    print(
        "stick_tip_xyz_in_E =",
        np.array2string(
            config.T_ES[:3, 3],
            precision=6,
        ),
    )

    # Check consistency of the derived camera-to-stick transform.
    expected_T_ES = config.T_EC @ config.T_CS

    assert np.allclose(
        config.T_ES,
        expected_T_ES,
        atol=1e-9,
    ), "T_EC @ T_CS does not equal T_ES."

    identity = (
        config.T_ES
        @ invert_transform(config.T_ES)
    )

    assert np.allclose(
        identity,
        np.eye(4),
        atol=1e-9,
    ), "T_ES inverse consistency check failed."

    if args.backend == "sim":
        backend = MujocoUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(
                raw["panda_command_port"]
            ),
            state_bind_ip=raw[
                "panda_state_bind_ip"
            ],
            state_port=int(
                raw["panda_state_port"]
            ),
        )
    else:
        backend = PandaUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(
                raw["panda_command_port"]
            ),
            state_bind_ip=raw[
                "panda_state_bind_ip"
            ],
            state_port=int(
                raw["panda_state_port"]
            ),
        )

    task_pose_source = TaskPoseUdpSource(
        args.tracker_bind_ip,
        args.tracker_port,
    )

    controller = PBVSController(config)

    period = 1.0 / config.control_rate_hz
    previous = time.monotonic()

    last_print = 0.0
    last_command_print = 0.0
    last_debug_print = 0.0

    print(
        f"Backend: {args.backend}; "
        f"dry_run={args.dry_run}"
    )
    print(
        "Receiving T_TS on "
        f"{args.tracker_bind_ip}:"
        f"{args.tracker_port}"
    )
    print("Ctrl-C to stop.")

    try:
        while True:
            loop_start = time.monotonic()

            dt = loop_start - previous
            previous = loop_start

            # Protect the controller from unusually small or large dt.
            dt = max(
                min(dt, 0.1),
                1e-6,
            )

            T_BE, robot_state_age = (
                backend.get_current_pose()
            )

            # Read the latest task measurement exactly once.
            task_pose = task_pose_source.get_latest()

            command, diagnostics = controller.step(
                T_BE=T_BE,
                robot_state_age=robot_state_age,
                task_pose=task_pose,
                dt=dt,
            )

            if (
                loop_start - last_debug_print > 0.25
                and T_BE is not None
                and task_pose is not None
            ):
                T_TS = task_pose.T_TS

                # Physical stick-tip pose in Panda base.
                T_BS = T_BE @ config.T_ES

                # T_TS = inv(T_BT) @ T_BS
                # Therefore:
                # T_BT = T_BS @ inv(T_TS)
                T_BT = (
                    T_BS
                    @ invert_transform(T_TS)
                )

                T_goal = controller._goal_pose(
                    T_BE,
                    T_TS,
                )

                position_error = (
                    T_goal[:3, 3]
                    - T_BE[:3, 3]
                )

                print(
                    "\n--- PBVS TASK-POSE DEBUG ---"
                )

                print(
                    "current_EE_xyz =",
                    np.array2string(
                        T_BE[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "stick_tip_xyz  =",
                    np.array2string(
                        T_BS[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "triangle_xyz   =",
                    np.array2string(
                        T_BT[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "measured_TS_xyz=",
                    np.array2string(
                        T_TS[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "desired_TS_xyz =",
                    np.array2string(
                        config.T_TS_des[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "goal_EE_xyz    =",
                    np.array2string(
                        T_goal[:3, 3],
                        precision=6,
                    ),
                )

                print(
                    "position_error =",
                    np.array2string(
                        position_error,
                        precision=6,
                    ),
                )

                print(
                    "command_xyz    =",
                    (
                        "None"
                        if command is None
                        else np.array2string(
                            command[:3, 3],
                            precision=6,
                        )
                    ),
                )

                print(
                    "command_lead   =",
                    (
                        "None"
                        if command is None
                        else np.array2string(
                            (
                                command[:3, 3]
                                - T_BE[:3, 3]
                            ),
                            precision=6,
                        )
                    ),
                )

                last_debug_print = loop_start

            if (
                command is not None
                and loop_start - last_command_print
                > 0.25
            ):
                print(
                    "current_xyz=",
                    (
                        None
                        if T_BE is None
                        else np.array2string(
                            T_BE[:3, 3],
                            precision=6,
                        )
                    ),
                    "command_xyz=",
                    np.array2string(
                        command[:3, 3],
                        precision=6,
                    ),
                )

                last_command_print = loop_start

            if (
                command is not None
                and not args.dry_run
            ):
                backend.send_target_pose(command)

            if loop_start - last_print > 0.5:
                print(
                    f"state={diagnostics.state.name}, "
                    f"|e_p|="
                    f"{diagnostics.position_error:.4f} m, "
                    f"|e_R|="
                    f"{math.degrees(
                        diagnostics.orientation_error
                    ):.2f} deg, "
                    f"reason="
                    f"{diagnostics.reason or '-'}"
                )

                last_print = loop_start

            elapsed = time.monotonic() - loop_start
            time.sleep(
                max(
                    0.0,
                    period - elapsed,
                )
            )

    except KeyboardInterrupt:
        print("\nStopping.")

    finally:
        task_pose_source.close()
        backend.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
