# Safe 6-DoF Panda UDP baseline

## Files

- `explorer_safe.cpp`: patched libfranka Cartesian impedance controller.
- `randinator_safe.py`: bounded translation demo using the corrected 6-DoF protocol.

## UDP protocol

Exactly 24 bytes:

```text
<6f = x, y, z, roll, pitch, yaw
```

- position: metres
- orientation: radians
- rotation convention: `R = Rz(yaw) Ry(pitch) Rx(roll)`
- little-endian IEEE-754 float32 on the Python side

## Important behavior

1. `explorer_safe` ignores commands until the operator presses ENTER and the current robot pose has been installed as the equilibrium pose.
2. Incoming position is clamped to the configured workspace.
3. The equilibrium pose is slew-limited to 0.03 m/s and 20 deg/s.
4. If no new command is received for about 250 ms, the controller freezes the equilibrium at the measured pose.
5. State packets contain radians, not degrees.
6. `randinator_safe.py` waits for a valid state packet and preserves the initial robot orientation before sending commands.

## First commissioning

1. Compile `explorer_safe.cpp` as a separate executable; keep the original binary available.
2. Run explorer in moving mode without a Python sender. After ENTER, confirm that the arm holds its current pose.
3. Start `randinator_safe.py --mode small` with the robot speed setting low and an emergency stop available.
4. Confirm that printed target and state orientations are initially equal.
5. Stop Python with Ctrl-C and verify that the C++ watchdog holds the current pose.
6. Only after these checks, use the same protocol in the PBVS publisher.

The numeric collision thresholds inherited from `setDefaultBehavior()` still require approval for the actual holder, stick, payload, and lab setup.
