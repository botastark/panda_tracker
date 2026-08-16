# Event-camera intrinsic calibration

This directory starts from the blinking checkerboard in `fe_panda` and makes
the target/detector convention explicit:

```text
HTML:   9 columns x 6 rows of inner corners
OpenCV: patternSize = (9, 6) = (columns, rows)
```

The display laptop needs only a browser. The Panda/event-camera computer runs
the Python collector and receives reconstructed event activity through
AEStream UDP. This tool estimates camera intrinsics only and never connects to
the Panda robot.

## 1. Display laptop

Copy `pattern_chessboard.html` to the laptop. It may be opened directly, or
served locally:

```bash
cd calibrator
python3 -m http.server 8000
```

Open:

```text
http://127.0.0.1:8000/pattern_chessboard.html?rows=6&cols=9&hz=12
```

Then:

1. Set browser zoom to 100% and enter full-screen mode.
2. Keep the tab visible; background tabs may throttle animation.
3. Leave the pattern running. Pausing it removes the continuing event source.
4. Measure one displayed checker square horizontally and vertically with a
   ruler or caliper. They must agree. Record the result in metres.
5. Do not resize the window or change display scaling after measurement.

The status panel hides after five seconds so it cannot cover the target. Press
`h` to show or hide it again.

The optional `square_px` query parameter limits the displayed square size, for
example `&square_px=80`. It controls pixels, not physical size; physical
measurement is still required.

## 2. Event-camera computer

Use the same environment that provides `aestream`, plus NumPy and OpenCV:

```bash
python3 -c "import aestream, cv2, numpy; print(cv2.__version__)"
```

Start the existing event-camera-to-AEStream UDP pipeline for a 1280x720 image
on port 6460. Then run, replacing the serial, measured square size, and output
directory:

```bash
python3 calibrator/calibrate_event_camera.py \
  --port 6460 \
  --image-width 1280 \
  --image-height 720 \
  --columns 9 \
  --rows 6 \
  --square-size-m 0.0250 \
  --accepted-views 50 \
  --camera-serial CAMERA_SERIAL \
  --camera-settings path/to/camera_settings.json \
  --output-dir calibration/CAMERA_SERIAL/2026-08-17-intrinsics
```

Omit `--camera-settings` only if the source has no settings file, and record
the biases/reconstruction configuration manually in the session manifest
before accepting the result.

## 3. Collect diverse views

Move the display laptop or manually guide the camera between accepted views.
For intrinsics, the target does not need to remain fixed in the robot base.
Cover:

- the image centre and all four corners;
- near, middle, and far distances;
- positive and negative tilt about both board axes;
- different apparent board sizes and in-plane angles.

The collector enforces a cooldown and rejects near-duplicate signatures, but
the operator remains responsible for genuine 3D viewpoint diversity. Press
`q` to stop; an incomplete data set does not produce an accepted calibration.

## 4. Outputs

Every accepted view is saved. A completed session contains:

```text
manifest.json
intrinsics.json
detections.npz
reconstructed/view_NNN.png
overlays/view_NNN_corners.png
```

Twenty percent of the views are withheld from intrinsic fitting. The result is
accepted only when held-out median reprojection error is at most 0.5 px and
the 95th percentile is at most 1.0 px, unless explicit project thresholds are
provided on the command line.

Do not use an `intrinsics.json` whose `status` is `rejected`. Repeat the entire
calibration after changing the lens, focus, aperture, resolution, event-camera
biases, reconstruction algorithm/window, or camera mounting.
