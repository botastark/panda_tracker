source /home/bota/repos/panda_pbvs_sim/.venv/bin/activate
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