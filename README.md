source /home/bota/repos/panda_pbvs_sim/.venv/bin/activate
bota@panda:~/Desktop$ source /home/bota/panda_tracker/.venv/bin/activate

conda deactivate
cd /home/bota/repos/panda_pbvs_sim


sudo ss -lunp | grep -E ':(2600|6200|6500)\b'
kill -CONT PID
kill PID


terminal1:
python simulation/simulated_explorer.py \
  --panda-xml mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config event_pbvs/pbvs_config_sim.json \
  --ee-body hand \
  --command-bind-ip 127.0.0.1 \
  --command-port 2600 \
  --state-ip 127.0.0.1 \
  --state-port 6200 \
  --tracker-ip 127.0.0.1 \
  --tracker-port 6500
  --triangle-step 0.05 \
  --triangle-rotation-step-deg 10 \
  --max-joint-speed 2.0
  --kp-position 10.0 \
  --kp-orientation 8.0 \
  --command-timeout 0.5


python simulation/simulated_explorer_holder_camera.py \
  --panda-xml mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config event_pbvs/pbvs_config_sim.json \
  --ee-body hand \
  --command-bind-ip 127.0.0.1 \
  --command-port 2600 \
  --state-ip 127.0.0.1 \
  --state-port 6200 \
  --tracker-ip 127.0.0.1 \
  --tracker-port 6500 \
  --triangle-step 0.05 \
  --triangle-rotation-step-deg 15 \
  --max-joint-speed 2.0 \
  --kp-position 10.0 \
  --kp-orientation 8.0 \
  --command-timeout 0.5


Terminal 2:
bash 
'''
python event_pbvs/event_pbvs_tracker.py \
--config event_pbvs/pbvs_config_sim.json \
--tracker-bind-ip 127.0.0.1 \
--tracker-udp-port 6500


W/S  triangle ±x
A/D  triangle ±y
R/F  triangle ±z

I/K  roll
J/L  pitch
U/O  yaw

## Running the Physical Panda, Digital Twin, and PBVS Controller

This setup runs the physical Panda, the MuJoCo digital twin, and the PBVS controller on the same computer.

### UDP port layout

```text
Physical explorer
    |
    v
0.0.0.0:6200  udp_pose_fanout.py
    |---> 127.0.0.1:6201  run_control.py robot state
    `---> 127.0.0.1:6202  MuJoCo digital-twin state

MuJoCo digital twin
    `---> 127.0.0.1:6501  run_control.py tracker pose

run_control.py
    `---> Panda command port 2600
```

Only the UDP fan-out process should bind to port `6200`.

### 1. Configure the physical Panda backend

In `configs/pbvs_robot.json`, use:

```json
"panda_state_bind_ip": "127.0.0.1",
"panda_state_port": 6201
```

Keep the Panda command settings unchanged.

### 2. Start the UDP state fan-out

From `panda_pbvs_project`:

```bash
python3 udp_pose_fanout.py \
  --bind-ip 0.0.0.0 \
  --bind-port 6200 \
  --destination 127.0.0.1:6201 \
  --destination 127.0.0.1:6202
```

Expected output:

```text
Listening for Panda state on 0.0.0.0:6200
Forwarding to: 127.0.0.1:6201, 127.0.0.1:6202
Expected packet: 24 bytes (<6f)
```

Once the physical explorer starts, the relay should report a nonzero forwarding rate.

### 3. Start the physical Panda explorer

```bash
cd /opt/libfranka/fe_panda

../build/fe_panda/explorer 1 0
```

Leave this process running.

### 4. Start the MuJoCo digital twin

From the repository root:

```bash
python simulation/simulated_explorer_holder_camera.py \
  --panda-xml mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config panda_pbvs_project/configs/pbvs_robot.json \
  --ee-body hand \
  --real-state-bind-ip 127.0.0.1 \
  --real-state-port 6202 \
  --tracker-ip 127.0.0.1 \
  --tracker-port 6501 \
  --max-joint-speed 2.0 \
  --kp-position 10.0 \
  --kp-orientation 8.0
```

The simulator first waits for the physical Panda state, synchronizes its end-effector pose, initializes the triangle, and then starts publishing `T_TC`.

Expected transition:

```text
mode=mirror-waiting-for-state
```

followed by:

```text
mode=mirror-synchronizing
```
and finally a synchronization message indicating that tracker streaming is enabled.

### 5. Start the PBVS controller

From `panda_pbvs_project`:

```bash
python3 run_control.py \
  --backend panda \
  --config configs/pbvs_robot.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501
```

The tracker port must match the simulator's `--tracker-port`.

### 6. Check port ownership

Before starting, or when debugging, run:

```bash
sudo ss -lunp | grep -E ':(2600|6200|6201|6202|6501)\b'
```

Expected listeners:

```text
6200  udp_pose_fanout.py
6201  run_control.py
6202  simulated_explorer_holder_camera.py
6501  run_control.py
```

### Troubleshooting

If the simulator reports:

```text
sync_error: |e_p|=inf m, |e_R|=inf deg
```

then no valid physical-state packet has reached port `6202`.

Check the physical stream:

```bash
sudo tcpdump -ni any 'udp dst port 6200'
```

Check the forwarded streams:

```bash
sudo tcpdump -ni lo 'udp dst port 6201 or udp dst port 6202'
```

If `run_control.py` raises:

```text
OSError: [Errno 98] Address already in use
```

another process is already bound to the configured robot-state or tracker port. Stop the old process or correct the port assignments above.