#!/usr/bin/env python3
"""
Send placeholder T_TC tracker packets for format and PBVS dry-run testing.

Packet:
    <16d, row-major homogeneous transform T_TC

Controls:
    x/X  camera +/- x_T
    y/Y  camera +/- y_T
    z/Z  camera +/- z_T
    r/R  camera roll +/-
    p/P  camera pitch +/-
    w/W  camera yaw +/-
    0    reset
    q    quit

This utility does not use MuJoCo.
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import termios
import time
import tty

import numpy as np


FORMAT = "<16d"


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def read_key() -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    return sys.stdin.read(1) if ready else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6500)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--distance", type=float, default=0.40)
    parser.add_argument("--translation-step", type=float, default=0.01)
    parser.add_argument("--rotation-step-deg", type=float, default=2.0)
    args = parser.parse_args()

    translation = np.array([0.0, 0.0, args.distance], dtype=float)
    rpy = np.zeros(3)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate

    print(__doc__)
    with RawTerminal():
        try:
            while True:
                key = read_key()
                if key == "q":
                    break
                if key == "0":
                    translation[:] = [0.0, 0.0, args.distance]
                    rpy[:] = 0.0
                elif key in ("x", "X", "y", "Y", "z", "Z"):
                    axis = {"x": 0, "y": 1, "z": 2}[key.lower()]
                    translation[axis] += args.translation_step * (1.0 if key.islower() else -1.0)
                elif key in ("r", "R", "p", "P", "w", "W"):
                    axis = {"r": 0, "p": 1, "w": 2}[key.lower()]
                    rpy[axis] += math.radians(args.rotation_step_deg) * (
                        1.0 if key.islower() else -1.0
                    )

                transform = np.eye(4)
                transform[:3, :3] = rotation_from_rpy(*rpy)
                transform[:3, 3] = translation
                sock.sendto(
                    struct.pack(FORMAT, *transform.reshape(-1)),
                    (args.ip, args.port),
                )

                print(
                    f"\rT_TC p={translation.round(3)} m, "
                    f"rpy={np.degrees(rpy).round(1)} deg",
                    end="",
                    flush=True,
                )
                time.sleep(period)
        except KeyboardInterrupt:
            pass
        finally:
            print()
            sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
