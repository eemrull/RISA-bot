# RISA-bot
ROS 2 Intelligent System Autonomy — Competition Robot

## Branches

| Branch | Purpose | Launch Command |
|---|---|---|
| `main` | Individual node testing & development | `run_risabot` |
| `test` | Competition mode — all 9 challenges | `ros2 launch risabot_automode competition.launch.py` |

## Quick Start

```bash
# On your PC — push changes
git add . && git commit -m "message" && git push

# On the robot — pull, build, run
ssh risabot
cd ~/risabotcar_ws/src/risabot_automode
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
| [Test Branch](Guide/test_branch.md) | How `test` works — state machine, all challenges |
| [Commands Reference](Guide/commands_reference.md) | All ROS topics, launch files, `ros2 param set` commands |
| [Tuning Guide](Guide/tuning_guide.md) | Step-by-step parameter tuning on physical course |
| [Architecture](Guide/architecture.md) | Node graphs, data flow, package structure |

## 🖥️ Dashboard

The `refactor-test` branch includes a real-time web dashboard at `http://<robot_ip>:8080`.

### Features
- **Hybrid Priority Engine** — Current top-level priority, lap, mode (AUTO/MANUAL), and **stop reason** display
- **Camera Feed** — Raw + debug overlays (Lane Lines, Traffic Light, Obstacle)
- **Traffic Light Visualizer** — Animated red/yellow/green indicator
- **Sensor Panel** — LiDAR, Camera, Fused obstacle, Boom Gate, Tunnel, Obstruction status
- **Odometry** — Distance + speed readout
- **Controller Visualizer** — Button map + live analog joystick visualization
- **Lane Following** — Error bar, linear X, angular Z
- **Behavior Priority** — Visual stack highlighting the highest-priority active maneuver
- **Parameter Tuning** — Slide-out drawer with Get/Set for all tunable ROS parameters (dashboard slides right to accommodate)
- **Session Uptime & Latency** — Live header indicators
- **Lap Timer** — Auto-resets on lap change
- **Event Log** — Timestamped state/mode changes and stop/resume events
- **Light Theme** — Clean, readable light UI

### Camera Debug Views
Click the camera tabs to switch between raw and annotated feeds:
- `Raw` — Unprocessed camera feed
- `Lane Lines` — Shows detected lane peaks and error
- `Traffic Light` — Shows detected color circles and confidence
- `Obstacle` — Shows ROI box and intensity values

> **Note:** Debug tabs auto-toggle `show_debug` on the perception nodes.

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
