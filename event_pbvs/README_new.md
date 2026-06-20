# Event-camera PBVS controller

This folder contains the Python outer-loop controller for the Panda tracking demo.

## Files

- `event_pbvs_tracker.py`: PBVS state machine and UDP publisher.
- `pbvs_config.json`: controller parameters and rigid transforms.

## Transform convention

`T_AB` means the pose of frame `B` expressed in frame `A`.

The event tracker must provide:

\[
T_{TC}
\]

which is the camera pose in the triangle frame.

The controller uses:

\[
T_{BE}^{goal}
=
T_{BE}^{state}
T_{EC}
(T_{TC}^{meas})^{-1}
T_{TC}^{des}
T_{CE}.
\]

The desired camera pose is derived from the desired stick-tip pose:

\[
T_{TC}^{des}=T_{TS}^{des}T_{SC}.
\]

## Configuration that must be measured

Replace the placeholder matrices in `pbvs_config.json`:

- `T_EC`: camera pose in the Panda EE frame.
- `T_CS`: stick-tip pose in the camera frame.
- `T_TS_des`: desired stick-tip pose in the triangle frame.

Do not enable robot motion until these transforms and their axis directions have been checked.

## Panda UDP convention

Both directions use exactly 24 bytes:

```text
<6f = x, y, z, roll, pitch, yaw
```

- position: metres
- angles: radians
- rotation convention:

\[
R=R_z(yaw)R_y(pitch)R_x(roll).
\]

## Safe first run

Keep this in the config:

```json
"dry_run": true
```

Start `explorer_safe`, then run:

```bash
python3 event_pbvs_tracker.py --config pbvs_config.json
```

With the dummy tracker adapter, the controller remains in `WAIT_FOR_TRACKER` and sends no robot commands.

## Reference tracker UDP adapter

For integration testing, the script can receive `T_TC` as 16 little-endian float64 values in row-major order:

```text
<16d
```

Run:

```bash
python3 event_pbvs_tracker.py \
  --config pbvs_config.json \
  --tracker-udp-port 6500
```

This transport format is only an example. Replace `UdpMatrixTrackerPoseSource` with the actual event tracker API or packet format.

## Commissioning order

1. Keep `dry_run=true`.
2. Verify Panda state reception.
3. Feed a static `T_TC`.
4. Check printed position and orientation errors.
5. Move the board slowly and verify the predicted correction direction.
6. Verify that the desired pose produces nearly zero error.
7. Set very conservative transforms and thresholds.
8. Change `dry_run` to `false` only for a supervised lab test.
9. Keep the emergency stop ready.

## Hold behavior

The Python controller returns to HOLD when:

- Panda state is stale,
- tracker pose is missing or stale,
- tracker reports invalid data,
- a tracker jump exceeds the configured threshold,
- the pose error exceeds the enable threshold.

When Python stops, the C++ `explorer_safe` watchdog is expected to hold the Panda.
