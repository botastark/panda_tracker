#!/usr/bin/env python3
"""Send synthetic PTP2 tracker poses using only the Python standard library."""

import argparse
import math
import signal
import socket
import struct
import sys
import time


TASK_POSE_STRUCT = struct.Struct("<4sBBHQf16d")
TASK_POSE_MAGIC = b"PTP2"
TASK_POSE_VERSION = 2
UINT64_MAX = (1 << 64) - 1

stop_requested = False


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def finite_number(value, option, parser):
    if not math.isfinite(value):
        parser.error(f"{option} must be finite")


def ipv4_address(value, option, parser):
    try:
        socket.inet_pton(socket.AF_INET, value)
    except OSError:
        parser.error(f"{option} must be an IPv4 address: {value}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send synthetic PTP2 tracker poses over UDP.",
        epilog=(
            "TEST DATA ONLY. Use only with the read-only pbvs_observer."
        ),
    )
    parser.add_argument(
        "--confirm-read-only-receiver",
        action="store_true",
        help="required safety acknowledgement",
    )
    parser.add_argument(
        "--destination-ip",
        default="127.0.0.1",
        help="receiver IPv4 address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--destination-port",
        type=int,
        default=6501,
        help="receiver UDP port (default: 6501)",
    )
    parser.add_argument(
        "--source-bind-ip",
        default="",
        help="optional local IPv4 interface address",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="packet rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="seconds; 0 means until interrupted (default: 10)",
    )
    parser.add_argument(
        "--pattern",
        choices=("static", "sine-x", "sine-y", "sine-z", "circle-xy"),
        default="static",
        help="synthetic translation pattern (default: static)",
    )
    parser.add_argument("--base-x", type=float, default=0.0)
    parser.add_argument("--base-y", type=float, default=0.0)
    parser.add_argument("--base-z", type=float, default=0.05)
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.01,
        help="pattern amplitude in meters (default: 0.01)",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.2,
        help="pattern frequency in Hz (default: 0.2)",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=1.0,
        help="tracker confidence in [0,1] (default: 1)",
    )
    parser.add_argument(
        "--invalid",
        action="store_true",
        help="publish packets with valid=false",
    )

    args = parser.parse_args()

    if not args.confirm_read_only_receiver:
        parser.error(
            "refusing to send test poses without "
            "--confirm-read-only-receiver"
        )

    ipv4_address(args.destination_ip, "--destination-ip", parser)
    if args.source_bind_ip:
        ipv4_address(args.source_bind_ip, "--source-bind-ip", parser)

    if not 1 <= args.destination_port <= 65535:
        parser.error("--destination-port must be in [1, 65535]")

    for option in (
        "rate",
        "duration",
        "base_x",
        "base_y",
        "base_z",
        "amplitude",
        "frequency",
        "confidence",
    ):
        finite_number(getattr(args, option), f"--{option.replace('_', '-')}", parser)

    if not 0.0 < args.rate <= 1000.0:
        parser.error("--rate must be in (0, 1000]")
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.amplitude < 0.0:
        parser.error("--amplitude must be non-negative")
    if args.frequency < 0.0:
        parser.error("--frequency must be non-negative")
    if not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be in [0, 1]")

    return args


def transform_for(args, elapsed):
    x = args.base_x
    y = args.base_y
    z = args.base_z
    phase = 2.0 * math.pi * args.frequency * elapsed

    if args.pattern == "sine-x":
        x += args.amplitude * math.sin(phase)
    elif args.pattern == "sine-y":
        y += args.amplitude * math.sin(phase)
    elif args.pattern == "sine-z":
        z += args.amplitude * math.sin(phase)
    elif args.pattern == "circle-xy":
        x += args.amplitude * math.cos(phase)
        y += args.amplitude * math.sin(phase)

    return (
        1.0, 0.0, 0.0, x,
        0.0, 1.0, 0.0, y,
        0.0, 0.0, 1.0, z,
        0.0, 0.0, 0.0, 1.0,
    )


def run(args):
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    destination = (args.destination_ip, args.destination_port)
    period = 1.0 / args.rate
    start = time.monotonic()
    next_send = start
    next_console = start
    sequence = 1

    print("SYNTHETIC TRACKER TEST DATA")
    print("Confirmed receiver: read-only pbvs_observer")
    print(f"Destination: {args.destination_ip}:{args.destination_port}")
    print(
        f"Pattern: {args.pattern} rate_hz={args.rate:g} "
        f"duration_s={args.duration:g}"
    )

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        if args.source_bind_ip:
            udp_socket.bind((args.source_bind_ip, 0))

        while not stop_requested:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            if sequence > UINT64_MAX:
                raise RuntimeError("PTP2 sequence exhausted uint64")

            transform = transform_for(args, elapsed)
            packet = TASK_POSE_STRUCT.pack(
                TASK_POSE_MAGIC,
                TASK_POSE_VERSION,
                0 if args.invalid else 1,
                0,
                sequence,
                args.confidence,
                *transform,
            )
            sent = udp_socket.sendto(packet, destination)
            if sent != TASK_POSE_STRUCT.size:
                raise RuntimeError(
                    f"short UDP send: {sent}/{TASK_POSE_STRUCT.size} bytes"
                )

            if now >= next_console:
                print(
                    f"seq={sequence} p_TS_m=[{transform[3]:.4f} "
                    f"{transform[7]:.4f} {transform[11]:.4f}] "
                    f"confidence={args.confidence:.4f} "
                    f"valid={0 if args.invalid else 1}",
                    flush=True,
                )
                next_console = now + 1.0

            sequence += 1
            next_send += period
            delay = next_send - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -period:
                next_send = time.monotonic()

    print(f"Sent {sequence - 1} PTP2 packets.")


def main():
    if TASK_POSE_STRUCT.size != 148:
        raise RuntimeError("internal PTP2 packet size is not 148 bytes")

    args = parse_args()
    try:
        run(args)
    except (OSError, RuntimeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
