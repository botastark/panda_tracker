#!/usr/bin/env python3
"""
Duplicate Franka/Panda UDP pose-state packets to multiple local consumers.

Default routing:
    explorer -> 127.0.0.1:6200
                    |
                    +-> run_control   127.0.0.1:6201
                    +-> digital twin  127.0.0.1:6202

The expected packet format is little-endian <6f:
    x, y, z, roll, pitch, yaw
which is exactly 24 bytes.
"""

from __future__ import annotations

import argparse
import signal
import socket
import struct
import time
from typing import Sequence


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)


def parse_destination(value: str) -> tuple[str, int]:
    host, separator, port_text = value.rpartition(":")
    if not separator or not host:
        raise argparse.ArgumentTypeError(
            f"Invalid destination {value!r}; expected HOST:PORT."
        )

    try:
        port = int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid port in destination {value!r}."
        ) from exc

    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError(
            f"Port must be between 1 and 65535: {value!r}."
        )

    return host, port


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fan out Panda <6f UDP state packets to multiple consumers."
    )
    parser.add_argument(
        "--bind-ip",
        default="127.0.0.1",
        help="Address receiving explorer state packets.",
    )
    parser.add_argument(
        "--bind-port",
        type=int,
        default=6200,
        help="Port receiving explorer state packets.",
    )
    parser.add_argument(
        "--destination",
        action="append",
        type=parse_destination,
        dest="destinations",
        help=(
            "Forward destination as HOST:PORT. May be repeated. "
            "Defaults to 127.0.0.1:6201 and 127.0.0.1:6202."
        ),
    )
    parser.add_argument(
        "--accept-any-size",
        action="store_true",
        help="Forward packets even when their size is not the expected 24 bytes.",
    )
    parser.add_argument(
        "--status-period",
        type=float,
        default=1.0,
        help="Seconds between packet-rate status messages; 0 disables them.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    destinations: Sequence[tuple[str, int]] = (
        args.destinations
        if args.destinations
        else [("127.0.0.1", 6201), ("127.0.0.1", 6202)]
    )

    if not 1 <= args.bind_port <= 65535:
        raise ValueError("--bind-port must be between 1 and 65535.")

    running = True

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive_socket.bind((args.bind_ip, args.bind_port))
    receive_socket.settimeout(0.2)

    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    destinations_text = ", ".join(
        f"{host}:{port}" for host, port in destinations
    )
    print(
        f"Listening for Panda state on {args.bind_ip}:{args.bind_port}\n"
        f"Forwarding to: {destinations_text}\n"
        f"Expected packet: {POSE_SIZE} bytes ({POSE_FORMAT})"
    )

    accepted_count = 0
    rejected_count = 0
    last_status_time = time.monotonic()
    last_status_count = 0

    try:
        while running:
            try:
                packet, source = receive_socket.recvfrom(2048)
            except socket.timeout:
                continue

            if len(packet) != POSE_SIZE and not args.accept_any_size:
                rejected_count += 1
                if rejected_count <= 5 or rejected_count % 100 == 0:
                    print(
                        f"Rejected {len(packet)}-byte packet from "
                        f"{source[0]}:{source[1]}; expected {POSE_SIZE} bytes."
                    )
                continue

            for destination in destinations:
                send_socket.sendto(packet, destination)

            accepted_count += 1

            if args.status_period > 0.0:
                now = time.monotonic()
                elapsed = now - last_status_time
                if elapsed >= args.status_period:
                    packets_since_status = accepted_count - last_status_count
                    rate = packets_since_status / elapsed
                    print(
                        f"Forwarded {accepted_count} packets "
                        f"({rate:.1f} packets/s), rejected={rejected_count}"
                    )
                    last_status_time = now
                    last_status_count = accepted_count
    finally:
        receive_socket.close()
        send_socket.close()

    print("UDP pose fan-out stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
