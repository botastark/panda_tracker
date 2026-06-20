#!/usr/bin/env python3
"""Example publisher for a future streamed absolute EE target."""

import socket
import time
import numpy as np

from common.geometry import transform_to_pose6
from common.protocol import pack_pose6


def main() -> None:
    destination = ("127.0.0.1", 2600)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    T_BE = np.eye(4)
    T_BE[:3, 3] = [0.45, 0.0, 0.45]

    while True:
        sock.sendto(pack_pose6(transform_to_pose6(T_BE)), destination)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
