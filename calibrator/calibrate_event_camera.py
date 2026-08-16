#!/usr/bin/env python3
"""Calibrate an event camera from reconstructed blinking-checkerboard images.

The event stream is received through AEStream UDP. The checkerboard is shown
by pattern_chessboard.html on a separate display. This program estimates
intrinsics only; it does not connect to or move the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import aestream
import cv2
import numpy as np


@dataclass
class AcceptedView:
    index: int
    timestamp_utc: str
    corners: np.ndarray
    signature: np.ndarray
    image_path: str
    overlay_path: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate an event camera using the blinking checkerboard "
            "displayed by pattern_chessboard.html."
        )
    )
    parser.add_argument("--port", type=int, default=6460)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument(
        "--columns",
        type=int,
        default=9,
        help="inner corners across the board; OpenCV first dimension",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=6,
        help="inner corners down the board; OpenCV second dimension",
    )
    parser.add_argument(
        "--square-size-m",
        type=float,
        required=True,
        help="physically measured checker square side length in metres",
    )
    parser.add_argument("--accepted-views", type=int, default=50)
    parser.add_argument(
        "--minimum-view-distance",
        type=float,
        default=0.06,
        help="minimum normalized signature distance from prior views",
    )
    parser.add_argument(
        "--minimum-seconds-between-views",
        type=float,
        default=0.75,
    )
    parser.add_argument(
        "--validation-fraction",
        type=float,
        default=0.2,
    )
    parser.add_argument("--median-error-limit-px", type=float, default=0.5)
    parser.add_argument("--p95-error-limit-px", type=float, default=1.0)
    parser.add_argument("--camera-serial", required=True)
    parser.add_argument(
        "--camera-settings",
        type=Path,
        help="settings/bias JSON used by the event reconstruction source",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="disable the live OpenCV window",
    )
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.image_width <= 0 or args.image_height <= 0:
        parser.error("image dimensions must be positive")
    if args.columns < 3 or args.rows < 3:
        parser.error("checkerboard dimensions must both be at least 3")
    if not math.isfinite(args.square_size_m) or args.square_size_m <= 0:
        parser.error("--square-size-m must be positive")
    if args.accepted_views < 15:
        parser.error("--accepted-views must be at least 15")
    if not 0.0 < args.minimum_view_distance < 1.0:
        parser.error("--minimum-view-distance must be in (0, 1)")
    if args.minimum_seconds_between_views < 0:
        parser.error("--minimum-seconds-between-views cannot be negative")
    if not 0.1 <= args.validation_fraction <= 0.4:
        parser.error("--validation-fraction must be in [0.1, 0.4]")
    if args.median_error_limit_px <= 0 or args.p95_error_limit_px <= 0:
        parser.error("reprojection limits must be positive")
    if args.camera_settings and not args.camera_settings.is_file():
        parser.error("--camera-settings does not name a readable file")
    return args


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def json_write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def prepare_output_directory(path: Path) -> tuple[Path, Path]:
    if path.exists() and any(path.iterdir()):
        raise RuntimeError(
            f"Output directory is not empty: {path}. "
            "Use a new directory for each calibration session."
        )
    images = path / "reconstructed"
    overlays = path / "overlays"
    images.mkdir(parents=True, exist_ok=True)
    overlays.mkdir(parents=True, exist_ok=True)
    return images, overlays


def stream_array(reading: Any, width: int, height: int) -> np.ndarray:
    if hasattr(reading, "detach"):
        reading = reading.detach()
    if hasattr(reading, "cpu"):
        reading = reading.cpu()
    if hasattr(reading, "numpy"):
        reading = reading.numpy()
    image = np.asarray(reading, dtype=np.float32).squeeze()

    if image.shape == (width, height):
        image = image.T
    elif image.shape != (height, width):
        raise RuntimeError(
            "Unexpected AEStream image shape "
            f"{image.shape}; expected {(height, width)} or {(width, height)}"
        )
    return image


def reconstruct_activity_image(reading: Any, width: int, height: int) -> np.ndarray:
    activity = np.abs(stream_array(reading, width, height))
    nonzero = activity[activity > 0]
    if nonzero.size == 0:
        return np.zeros((height, width), dtype=np.uint8)

    high = float(np.percentile(nonzero, 99.5))
    if not math.isfinite(high) or high <= 0:
        return np.zeros((height, width), dtype=np.uint8)
    return np.clip(activity * (255.0 / high), 0.0, 255.0).astype(np.uint8)


def view_signature(corners: np.ndarray, width: int, height: int) -> np.ndarray:
    points = corners.reshape(-1, 2).astype(np.float64)
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    covariance = centered.T @ centered / max(1, len(points) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = np.maximum(eigenvalues[order], 1e-12)
    major = eigenvectors[:, order[0]]
    angle = math.atan2(float(major[1]), float(major[0]))
    hull_area = float(cv2.contourArea(cv2.convexHull(points.astype(np.float32))))
    area_scale = math.sqrt(max(hull_area, 0.0) / float(width * height))
    anisotropy = 0.5 * math.log(float(eigenvalues[0] / eigenvalues[1]))
    return np.array(
        [
            centroid[0] / width,
            centroid[1] / height,
            area_scale,
            0.25 * anisotropy,
            0.20 * math.cos(2.0 * angle),
            0.20 * math.sin(2.0 * angle),
        ],
        dtype=np.float64,
    )


def is_diverse(
    signature: np.ndarray,
    accepted: list[AcceptedView],
    minimum_distance: float,
) -> tuple[bool, float]:
    if not accepted:
        return True, float("inf")
    nearest = min(
        float(np.linalg.norm(signature - view.signature))
        for view in accepted
    )
    return nearest >= minimum_distance, nearest


def object_points(columns: int, rows: int, square_size_m: float) -> np.ndarray:
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid.astype(np.float32) * square_size_m
    return points


def draw_status(
    image: np.ndarray,
    corners: np.ndarray | None,
    found: bool,
    message: str,
    pattern: tuple[int, int],
) -> np.ndarray:
    overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if found and corners is not None:
        cv2.drawChessboardCorners(overlay, pattern, corners, True)
    cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(
        overlay,
        message,
        (8, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return overlay


def collect_views(
    args: argparse.Namespace,
    image_directory: Path,
    overlay_directory: Path,
) -> list[AcceptedView]:
    pattern = (args.columns, args.rows)
    flags = getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(
        cv2, "CALIB_CB_ACCURACY", 0
    )
    accepted: list[AcceptedView] = []
    last_accepted_s = -float("inf")
    window_name = "Event-camera checkerboard calibration"

    if not args.no_display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print(
        f"Waiting for {args.accepted_views} diverse views of "
        f"{args.columns}x{args.rows} inner corners on UDP port {args.port}."
    )
    print("Move the camera/board between accepted views; press q to stop.")

    with aestream.UDPInput(
        (args.image_width, args.image_height),
        device="cpu",
        port=args.port,
    ) as stream:
        while len(accepted) < args.accepted_views:
            gray = reconstruct_activity_image(
                stream.read(), args.image_width, args.image_height
            )
            found, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
            now_s = time.monotonic()
            message = f"accepted {len(accepted)}/{args.accepted_views}"

            if found:
                signature = view_signature(
                    corners, args.image_width, args.image_height
                )
                diverse, nearest = is_diverse(
                    signature, accepted, args.minimum_view_distance
                )
                cooled_down = (
                    now_s - last_accepted_s
                    >= args.minimum_seconds_between_views
                )
                if diverse and cooled_down:
                    index = len(accepted)
                    image_name = f"view_{index:03d}.png"
                    overlay_name = f"view_{index:03d}_corners.png"
                    overlay = draw_status(
                        gray,
                        corners,
                        True,
                        f"ACCEPTED {index + 1}/{args.accepted_views}",
                        pattern,
                    )
                    if not cv2.imwrite(
                        str(image_directory / image_name), gray
                    ):
                        raise RuntimeError("Failed to save reconstructed image")
                    if not cv2.imwrite(
                        str(overlay_directory / overlay_name), overlay
                    ):
                        raise RuntimeError("Failed to save overlay image")
                    accepted.append(
                        AcceptedView(
                            index=index,
                            timestamp_utc=utc_now(),
                            corners=corners.astype(np.float32),
                            signature=signature,
                            image_path=f"reconstructed/{image_name}",
                            overlay_path=f"overlays/{overlay_name}",
                        )
                    )
                    last_accepted_s = now_s
                    print(
                        f"accepted view {index + 1}/{args.accepted_views}; "
                        f"nearest signature distance={nearest:.4f}"
                    )
                    message = (
                        f"ACCEPTED {index + 1}/{args.accepted_views}; "
                        "move to a new view"
                    )
                elif not diverse:
                    message += f"; duplicate/near view ({nearest:.3f})"
                else:
                    message += "; waiting for cooldown"
            else:
                corners = None
                message += "; checkerboard not found"

            if not args.no_display:
                overlay = draw_status(gray, corners, found, message, pattern)
                cv2.imshow(window_name, overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    if not args.no_display:
        cv2.destroyAllWindows()
    return accepted


def point_errors(
    object_point_set: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    projected, _ = cv2.projectPoints(
        object_point_set, rvec, tvec, camera_matrix, distortion
    )
    return np.linalg.norm(
        projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1
    )


def calibrate_and_validate(
    args: argparse.Namespace,
    views: list[AcceptedView],
    output_directory: Path,
) -> dict[str, Any]:
    if len(views) < args.accepted_views:
        raise RuntimeError(
            f"Only {len(views)} of {args.accepted_views} required views were "
            "collected; calibration was not run."
        )

    point_template = object_points(
        args.columns, args.rows, args.square_size_m
    )
    validation_count = max(
        3, int(round(len(views) * args.validation_fraction))
    )
    validation_indices = set(
        int(value)
        for value in np.linspace(
            0, len(views) - 1, validation_count, dtype=int
        )
    )
    training = [view for view in views if view.index not in validation_indices]
    validation = [view for view in views if view.index in validation_indices]

    training_objects = [point_template.copy() for _ in training]
    training_images = [view.corners for view in training]
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        training_objects,
        training_images,
        (args.image_width, args.image_height),
        None,
        None,
    )

    training_errors: list[float] = []
    training_view_errors: list[dict[str, Any]] = []
    for view, rvec, tvec in zip(training, rvecs, tvecs):
        errors = point_errors(
            point_template,
            view.corners,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        training_errors.extend(float(value) for value in errors)
        training_view_errors.append(
            {
                "view": view.index,
                "mean_px": float(np.mean(errors)),
                "max_px": float(np.max(errors)),
            }
        )

    validation_errors: list[float] = []
    validation_view_errors: list[dict[str, Any]] = []
    for view in validation:
        solved, rvec, tvec = cv2.solvePnP(
            point_template,
            view.corners,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved:
            validation_view_errors.append(
                {"view": view.index, "solve_pnp": "failed"}
            )
            continue
        errors = point_errors(
            point_template,
            view.corners,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        validation_errors.extend(float(value) for value in errors)
        validation_view_errors.append(
            {
                "view": view.index,
                "mean_px": float(np.mean(errors)),
                "max_px": float(np.max(errors)),
            }
        )

    if not validation_errors:
        raise RuntimeError("No held-out validation pose could be solved")

    validation_median = float(np.median(validation_errors))
    validation_p95 = float(np.percentile(validation_errors, 95.0))
    accepted = (
        validation_median <= args.median_error_limit_px
        and validation_p95 <= args.p95_error_limit_px
    )

    np.savez_compressed(
        output_directory / "detections.npz",
        object_points=point_template,
        image_points=np.stack([view.corners for view in views]),
        training_indices=np.array([view.index for view in training]),
        validation_indices=np.array([view.index for view in validation]),
    )

    return {
        "schema": "panda_tracker_event_camera_intrinsics_v1",
        "created_utc": utc_now(),
        "status": "accepted" if accepted else "rejected",
        "camera_serial": args.camera_serial,
        "camera_model": "opencv_pinhole",
        "image_size": [args.image_width, args.image_height],
        "pattern": {
            "inner_columns": args.columns,
            "inner_rows": args.rows,
            "square_width_m": args.square_size_m,
            "square_height_m": args.square_size_m,
        },
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "opencv_training_rms_px": float(rms),
        "training": {
            "view_count": len(training),
            "median_error_px": float(np.median(training_errors)),
            "p95_error_px": float(np.percentile(training_errors, 95.0)),
            "per_view": training_view_errors,
        },
        "validation": {
            "view_count": len(validation),
            "median_error_px": validation_median,
            "p95_error_px": validation_p95,
            "median_limit_px": args.median_error_limit_px,
            "p95_limit_px": args.p95_error_limit_px,
            "per_view": validation_view_errors,
        },
        "view_collection": {
            "accepted_count": len(views),
            "minimum_signature_distance": args.minimum_view_distance,
            "minimum_seconds_between_views": (
                args.minimum_seconds_between_views
            ),
            "views": [
                {
                    "index": view.index,
                    "timestamp_utc": view.timestamp_utc,
                    "signature": view.signature.tolist(),
                    "image": view.image_path,
                    "overlay": view.overlay_path,
                }
                for view in views
            ],
        },
        "notes": [
            "Object-point units are metres.",
            "Do not use this result if camera, lens, focus, resolution, biases, "
            "or reconstruction settings change.",
            "A result marked rejected must not be copied into PBVS config.",
        ],
    }


def main() -> int:
    args = parse_args()
    try:
        images, overlays = prepare_output_directory(args.output_dir)
        manifest: dict[str, Any] = {
            "schema": "panda_tracker_event_camera_calibration_session_v1",
            "started_utc": utc_now(),
            "camera_serial": args.camera_serial,
            "event_input": {
                "transport": "aestream_udp",
                "port": args.port,
                "image_width": args.image_width,
                "image_height": args.image_height,
                "activity_normalization_percentile": 99.5,
            },
            "pattern": {
                "html": "calibrator/pattern_chessboard.html",
                "url_query": (
                    f"rows={args.rows}&cols={args.columns}&hz=12"
                ),
                "inner_columns": args.columns,
                "inner_rows": args.rows,
                "measured_square_size_m": args.square_size_m,
            },
            "camera_settings_path": (
                str(args.camera_settings) if args.camera_settings else None
            ),
            "command": sys.argv,
        }
        if args.camera_settings:
            manifest["camera_settings"] = json.loads(
                args.camera_settings.read_text()
            )
        json_write(args.output_dir / "manifest.json", manifest)

        views = collect_views(args, images, overlays)
        result = calibrate_and_validate(args, views, args.output_dir)
        json_write(args.output_dir / "intrinsics.json", result)

        print("camera matrix:")
        print(np.asarray(result["camera_matrix"]))
        print("distortion coefficients:")
        print(np.asarray(result["distortion_coefficients"]))
        validation = result["validation"]
        print(
            "held-out reprojection: "
            f"median={validation['median_error_px']:.4f}px, "
            f"p95={validation['p95_error_px']:.4f}px"
        )
        print(f"Result: {result['status'].upper()}")
        print(f"Saved calibration session to {args.output_dir}")
        return 0 if result["status"] == "accepted" else 2
    except KeyboardInterrupt:
        print("Interrupted; no calibration result was accepted.", file=sys.stderr)
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
