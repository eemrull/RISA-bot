# RISA-bot Commands & CLI Reference

Comprehensive quick-copy reference for commands, bash aliases, launch scripts, topic monitors, and runtime parameter tuning.

---

## 1. Robot Bash Aliases

Configured on the robot in `~/.bash_aliases` (generated via `bash tools/install_bashalias.sh`):

### Workspace & Build Shortcuts

| Alias | Command | Purpose |
|---|---|---|
| `cb` | `cd ~/risabotcar_ws && colcon build --symlink-install` | Standard build after code changes |
| `cbp <pkg>` | `cd ~/risabotcar_ws && colcon build --symlink-install --packages-select <pkg>` | Fastest build for a single package (e.g. `cbp risabot_automode`) |
| `cbc` | `cd ~/risabotcar_ws && rm -rf build/risabot_automode install/risabot_automode build/control_servo install/control_servo && colcon build --symlink-install && source install/setup.bash` | Clean rebuild of the Python packages |
| `cbd` | `rm -rf build/ install/ log/ && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/humble/setup.bash && colcon build --symlink-install` | Full workspace purge & recompile |
| `sos` | `source ~/risabotcar_ws/install/setup.bash` | Re-source workspace overlay |
| `s` | `source ~/.bashrc` | Reload bash profile |

### Node & Launch Shortcuts

| Alias | What it Launches |
|---|---|
| `risabot` | `ros2 launch risabot_automode competition.launch.py` |
| `astra` | Orbbec Astra Mini camera driver |
| `ydlidar` | YDLiDAR Tmini Plus driver on `/dev/serial/by-id/` |
| `servoc` | Servo controller hardware bridge |
| `obstav` | LiDAR obstacle avoidance node |
| `autod` | Auto driver state machine node |
| `linefollow` | Line follower camera node |
| `dashboard` | Web dashboard standalone Python runner |
| `setup_wifi`| `sudo bash ~/risabotcar_ws/tools/setup_wifi.sh` |
| `fix_astra` | Restores `openni2_redist` binaries from backup |
| `kill_risa` | Closes all open xfce4-terminal windows |

### Multi-Tab Launchers

```bash
run_risabot    # Launches 6 terminal tabs in X11 GUI
run_trisabot   # Launches 6 tmux windows over SSH
tmux attach -t risabot        # Reattach to running tmux session
tmux kill-session -t risabot  # Terminate session
```

---

## 2. Launch Commands

```bash
# 1. Full Competition Bringup (Sensors + Perception + Brain + SLAM + Dashboard + go2rtc)
ros2 launch risabot_automode bringup.launch.py

# 2. Competition Bringup Without SLAM (Reduces CPU load)
ros2 launch risabot_automode bringup.launch.py slam:=false

# 3. Isolated Lane Follower Testing (Camera + Line Follower + Auto Driver)
ros2 launch risabot_automode lane_test.launch.py

# 4. Isolated SLAM Mapping (LiDAR + Odom TF + SLAM Toolbox)
ros2 launch risabot_slam slam_test.launch.py

# 5. Manual RC Driving (Joystick + Servo Controller Bridge)
ros2 launch control_servo robot_rc.launch.py
```

---

## 3. Testing & Verification Commands

### BPU AI Hardware Diagnostics

```bash
# 1. Verify BPU model load & basic inference on dummy tensor:
python3 tools/bpu_model/verify_bpu.py

# 2. Live camera feed detection verification (displays detection confidence):
python3 tools/bpu_model/verify_live.py
```

### Automated Rosbag Regression Validator

```bash
# Record telemetry bag during a run:
ros2 bag record /loop_stats /health_status /cmd_safety_status /cmd_vel_auto /odom

# Execute regression analysis:
ros2 run risabot_automode bag_regression_validator --ros-args \
  -p window_sec:=45.0 \
  -p output_file:=/tmp/risa_regression.json
```

### Companion Desktop App (`RisaBotApp`)

```powershell
# On Windows developer PC:
cd RisaBotApp
dotnet build RisaBotApp.csproj
.\bin\Debug\net10.0-windows\RisaBotApp.exe
```

---

## 4. Topic Monitoring & Diagnostics

```bash
# List all active topics
ros2 topic list

# Control & State Telemetry
ros2 topic echo /dashboard_state          # Current state machine state
ros2 topic echo /health_status            # Global system health JSON
ros2 topic echo /loop_stats               # Loop frequencies and jitter
ros2 topic echo /cmd_safety_status        # Safety controller clamp status
ros2 topic echo /odom                     # Odometry position & speed
ros2 topic echo /imu/pitch                # IMU pitch angle for hill mode

# Perception Topics
ros2 topic echo /lane_error               # Lane center offset [-1.0 to 1.0]
ros2 topic echo /traffic_light_state      # "red", "yellow", "green", "unknown"
ros2 topic echo /boom_gate_open           # True when gate is raised
ros2 topic echo /tunnel_detected          # True when corridor walls present
ros2 topic echo /obstruction_active       # True when obstacle dodge active
ros2 topic echo /parking_signboard_detected # True when parking sign detected
ros2 topic echo /hill_sign_detected       # True when hill sign detected

# Sensor Frequencies
ros2 topic hz /camera/color/image_raw     # Expected: ~30 Hz
ros2 topic hz /scan                       # Expected: ~10 Hz
ros2 topic hz /lane_error                 # Expected: ~31 Hz
ros2 topic hz /odom                       # Expected: 50 Hz
```

---

## 5. Manual CLI Robot Control

```bash
# Toggle Auto / Manual Mode
ros2 topic pub --once /auto_mode std_msgs/Bool "data: true"
ros2 topic pub --once /auto_mode std_msgs/Bool "data: false"

# Override State Machine State
ros2 topic pub --once /set_challenge std_msgs/String "data: LANE_FOLLOW"
ros2 topic pub --once /set_challenge std_msgs/String "data: TUNNEL"
ros2 topic pub --once /set_challenge std_msgs/String "data: OBSTRUCTION"
ros2 topic pub --once /set_challenge std_msgs/String "data: HILL"
ros2 topic pub --once /set_challenge std_msgs/String "data: PARALLEL_PARK"
ros2 topic pub --once /set_challenge std_msgs/String "data: PERPENDICULAR_PARK"

# Trigger Emergency Stop
ros2 topic pub --once /e_stop std_msgs/Bool "data: true"
ros2 topic pub --once /e_stop std_msgs/Bool "data: false"

# Trigger Parking Maneuver
ros2 topic pub --once /parking_command std_msgs/String "data: parallel"
ros2 topic pub --once /parking_command std_msgs/String "data: perpendicular"

# Manual Drive from CLI (Testing Only)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" --rate 10
```

---

## 6. Live Parameter Tuning (`ros2 param set`)

Parameters take effect immediately without recompilation:

```bash
# Auto Driver / Lane Follower
ros2 param set /auto_driver forward_speed 0.15
ros2 param set /auto_driver pid_kp 0.8
ros2 param set /auto_driver pid_kd 0.20
ros2 param set /auto_driver min_turn_speed 0.4
ros2 param set /auto_driver lane_steer_slew 3.0

# Line Follower Camera
ros2 param set /line_follower_camera white_threshold 100
ros2 param set /line_follower_camera crop_ratio_base 0.55
ros2 param set /line_follower_camera invert_binary true
ros2 param set /line_follower_camera kalman_process_noise 0.01
ros2 param set /line_follower_camera kalman_measurement_noise 0.1

# Boom Gate Detector
ros2 param set /boom_gate_detector cam_red_min_width 80
ros2 param set /boom_gate_detector min_detect_dist 0.15
ros2 param set /boom_gate_detector max_detect_dist 0.80
ros2 param set /boom_gate_detector hysteresis 5

# Tunnel Wall Follower
ros2 param set /tunnel_wall_follower forward_speed 0.12
ros2 param set /tunnel_wall_follower kp 5.0
ros2 param set /tunnel_wall_follower kd 0.5
ros2 param set /tunnel_wall_follower kp_heading 1.0

# Hill Climb & Descent
ros2 param set /auto_driver hill_pitch_threshold 8.0
ros2 param set /auto_driver hill_base_speed 0.18
ros2 param set /auto_driver hill_speed_per_degree 0.006
ros2 param set /auto_driver descent_pitch_threshold 5.0

# Servo Controller Hardware & Odometry
ros2 param set /servo_controller ticks_per_meter 6249.0
ros2 param set /servo_controller auto_right_steer_boost 1.3
ros2 param set /servo_controller odom_reverse_polarity true

# BPU Signage Detector Confidence
ros2 param set /signage_detector thresh_hill 0.06
ros2 param set /signage_detector thresh_parallelp 0.05
ros2 param set /signage_detector thresh_perpendp 0.05
ros2 param set /signage_detector thresh_tl_green 0.25
ros2 param set /signage_detector thresh_tl_red 0.25
```
