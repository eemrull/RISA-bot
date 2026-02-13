# Test Branch Guide (`test`)

The test branch is for **competition-ready code** with all 9 challenges integrated.

## How to Run

```bash
ssh risabot
cd ~/risabotcar_ws/src/risabot_automode && git checkout test && git pull
cd ~/risabotcar_ws && cb && sos
ros2 launch risabot_automode competition.launch.py
```

## What `competition.launch.py` Launches

Everything in one command:

| Node | Package | Purpose |
|---|---|---|
| Astra Mini camera | astra_camera | Color + depth camera |
| YDLiDAR Tmini Plus | ydlidar_ros2_driver | LiDAR |
| TF publisher | tf2_ros | base_link → laser_frame |
| obstacle_avoidance | obstacle_avoidance | LiDAR front obstacle detection |
| obstacle_avoidance_camera | obstacle_avoidance_camera | Camera obstacle detection |
| line_follower_camera | risabot_automode | Lane error calculation |
| **traffic_light_detector** | risabot_automode | Red/Yellow/Green detection |
| **boom_gate_detector** | risabot_automode | Barrier detection via LiDAR |
| **tunnel_wall_follower** | risabot_automode | PD wall following in tunnel |
| **obstruction_avoidance** | risabot_automode | Lateral dodge maneuver |
| **parking_controller** | risabot_automode | Parallel + perpendicular parking |
| auto_driver | risabot_automode | State machine brain (5s delayed start) |
| joy_node | joy | Joystick driver |
| servo_controller | control_servo | Mode toggle + manual driving |

## Controller (Test Branch)

| Input | Action |
|---|---|
| **Start button** (7) | Toggle AUTO / MANUAL mode |
| Left Stick | Manual driving (forward/back + turn) — manual mode only |
| Right Stick | Camera servo pan/tilt — always active |
| **RB** (5) | Cycle to next challenge state |
| **LB** (4) | Cycle to previous challenge state |

> Button numbers are for Xbox-style controllers. Verify with `jstest /dev/input/js0`.

## State Machine

The `auto_driver` uses a `ChallengeState` enum to manage the competition:

```text
LANE_FOLLOW → OBSTRUCTION → ROUNDABOUT → TUNNEL → BOOM_GATE_TUNNEL
→ BOOM_GATE_MAIN → HILL → BUMPER → TRAFFIC_LIGHT
→ PARALLEL_PARK → PERPENDICULAR_PARK → FINISHED
```

### How State Transitions Work

- States advance based on **distance traveled** and **sensor triggers**
- Each state selects the appropriate `/cmd_vel` source:
  - Lane following states → uses `/lane_error` with steering gain
  - Tunnel → uses `/tunnel_cmd_vel` from wall follower
  - Obstruction → uses `/obstruction_cmd_vel` from dodge module
  - Parking → uses `/parking_cmd_vel` from parking controller
  - Boom gate / traffic light → stops until condition clears

### Manual State Override

```bash
# Jump to a specific state for testing
ros2 topic pub --once /set_challenge std_msgs/String "data: TUNNEL"
ros2 topic pub --once /set_challenge std_msgs/String "data: TRAFFIC_LIGHT"
ros2 topic pub --once /set_challenge std_msgs/String "data: PARALLEL_PARK"
```

Or use **LB/RB** on the controller to cycle through states.

### Trigger Parking Manually

```bash
ros2 topic pub --once /parking_command std_msgs/String "data: parallel"
ros2 topic pub --once /parking_command std_msgs/String "data: perpendicular"
```

## Key Topics (Test Branch)

| Topic | Type | Published By | Purpose |
|---|---|---|---|
| `/scan` | LaserScan | ydlidar | LiDAR data |
| `/camera/color/image_raw` | Image | astra | Color camera |
| `/camera/depth/image_raw` | Image | astra | Depth camera |
| `/odom` | Odometry | auto_driver | Wheel odometry |
| `/cmd_vel` | Twist | auto_driver or servo_controller | Motor commands |
| `/auto_mode` | Bool | servo_controller | Auto/manual toggle |
| `/lane_error` | Float32 | line_follower_camera | Lane offset |
| `/obstacle_front` | Bool | obstacle_avoidance | Front obstacle |
| `/traffic_light_state` | String | traffic_light_detector | red/yellow/green/unknown |
| `/boom_gate_open` | Bool | boom_gate_detector | Gate open/closed |
| `/tunnel_detected` | Bool | tunnel_wall_follower | In tunnel? |
| `/tunnel_cmd_vel` | Twist | tunnel_wall_follower | Tunnel steering |
| `/obstruction_active` | Bool | obstruction_avoidance | Dodging? |
| `/obstruction_cmd_vel` | Twist | obstruction_avoidance | Dodge steering |
| `/parking_complete` | Bool | parking_controller | Parking done? |
| `/parking_cmd_vel` | Twist | parking_controller | Parking steering |
| `/parking_signboard_detected` | Bool | parking_controller | Triangle sign seen? |
| `/set_challenge` | String | servo_controller or CLI | State override |
| `/parking_command` | String | CLI | Trigger parking type |

## Differences from Main

| Feature | Main | Test |
|---|---|---|
| Launch command | `run_risabot` | `ros2 launch risabot_automode competition.launch.py` |
| Auto mode | Monolithic controller handles everything | State machine + modular nodes |
| Manual driving | Through `Rosmaster.set_motor()` directly | Through `/cmd_vel` topic |
| Challenges | Lane follow + basic dodge | All 9 competition challenges |
| Camera | Launched separately | Included in launch file |
| Serial conflict | ⚠️ Controller and auto_driver fight | ✅ Only auto_driver uses serial |
