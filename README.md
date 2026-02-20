# RISA-bot
ROS 2 Intelligent System Autonomy — Competition Robot

## Branches

| Branch | Purpose | Launch Command |
|---|---|---|
| `main` | Individual node testing & development | `run_risabot` |
| `test` | Competition mode — all 9 challenges + dashboard | `ros2 launch risabot_automode competition.launch.py` |

## Quick Start

```bash
# On your PC — push changes
git add . && git commit -m "message" && git push

# On the robot — pull, build, run
ssh risabot
cd ~/risabotcar_ws/src/RISA-bot
git checkout test && git pull   # or: main
cd ~/risabotcar_ws && cb && sos

# Run (pick one)
run_risabot                                          # main branch
ros2 launch risabot_automode competition.launch.py   # test branch
```

## Build Aliases

| Alias | When to Use |
|---|---|
| `cb` | Normal rebuild after code changes |
| `cbp <pkg>` | Rebuild only one package (fastest) |
| `cbc` | Clean rebuild — use when stuff is broken |
| `sos` | Re-source the workspace (after any build) |

## 📖 Guide

Detailed documentation is in the [Guide/](Guide/) folder:

| Guide | Description |
|---|---|
| [Challenge Breakdown](Guide/challenges_breakdown.md) | Deep dive into each challenge's code, with course layout |
| [Main Branch](Guide/main_branch.md) | How `main` works — individual nodes, original controller |
| [Test Branch](Guide/test_branch.md) | How `test` works — state machine, dashboard, all challenges |
| [Commands Reference](Guide/commands_reference.md) | All ROS topics, launch files, `ros2 param set` commands |
| [Tuning Guide](Guide/tuning_guide.md) | Step-by-step parameter tuning on physical course |
| [Architecture](Guide/architecture.md) | Node graphs, data flow, package structure |

## Features (Test Branch)

- **9 Competition Challenges** — Autonomous navigation through lane follow, obstruction, roundabout, tunnel, boom gate, hill, bumper, traffic light, parking
- **Joystick Safety Watchdog** — Robot auto-stops if controller disconnects or is turned off
- **Lap Tracking** — Automatic lap 1 → lap 2 transition with different challenge routes
- **Manual Override** — Start button toggles auto/manual, LB/RB cycles states

## 🖥️ Dashboard (Test Branch)

Real-time web dashboard at `http://<robot_ip>:8080` with **Catppuccin Macchiato** theme.

### Core Panels
- **State Machine** — Current challenge, lap, mode (AUTO/MANUAL), state timer, distance, and lap timer
- **Traffic Light** — Animated red/yellow/green visualizer
- **Manual Control** — Speed gauge with gear dots and D-Pad shifting
- **Lane Following** — Error bar, linear X, angular Z readout
- **Sensors** — LiDAR, Camera, Fused obstacle, Boom Gate, Tunnel, Obstruction, Parking status
- **Odometry** — Distance + speed
- **Controller** — Button map + **live visual analog joystick circles**
- **Competition Flow** — Visual timeline of all challenges with progress tracking
- **Event Log** — Timestamped state/mode changes

### Camera Debug Views
Click tabs to switch between raw and annotated perception feeds:
| Tab | Shows |
|---|---|
| `Raw` | Unprocessed camera feed |
| `Lane Lines` | Detected lane peaks, center point, error value |
| `Traffic Light` | Color circles with confidence values |
| `Obstacle` | ROI box with intensity values |

> Debug tabs auto-toggle `show_debug` on the perception nodes for zero-config use.

### Header Indicators
- **Session Uptime** — `HH:MM:SS` counter since page load
- **Network Latency** — Round-trip `ms` badge showing WiFi quality
- **Connection Status** — Green dot with live/disconnected state

### Parameter Tuning
Slide-out ⚙️ drawer on the left edge with Get/Set for all tunable ROS parameters across all nodes. Changes are instant but session-only.

## Troubleshooting

### astra_camera fails with `openni2_redist` missing

```bash
cp -r ~/backups/openni2_redist ~/risabotcar_ws/src/ros2_astra_camera/astra_camera/
colcon build --symlink-install --packages-select astra_camera
```

### AMENT_PREFIX_PATH warnings after cbc

```bash
unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/humble/setup.bash
```

### Third-party packages disappear

These are gitignored and don't change with branch switching. If lost:

```bash
# ros2_astra_camera — use ~/backups
# YDLidar-SDK — git clone https://github.com/YDLIDAR/YDLidar-SDK.git
# ydlidar_ros2_driver — git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
```

## Key Rules

1. Only `git checkout/pull` inside your package dir — never from workspace root
2. Third-party packages (astra, YDLiDAR) are gitignored and separate
