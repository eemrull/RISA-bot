# RISA-bot
ROS 2 Intelligent System Autonomy — Competition Robot

## Branches

| Branch | Purpose | Launch Command |
|---|---|---|
| `main` | Individual node testing & development | `run_risabot` |
| `new-control-code` | Competition mode — all 9 challenges | `ros2 launch risabot_automode competition.launch.py` |

## Quick Start

```bash
# On your PC — push changes
git add . && git commit -m "message" && git push

# On the robot — pull, build, run
ssh risabot
cd ~/risabotcar_ws/src/risabot_automode
git checkout new-control-code && git pull   # or: main
cd ~/risabotcar_ws && cb && sos

# Run (pick one)
run_risabot                                          # main branch
ros2 launch risabot_automode competition.launch.py   # new-control-code branch
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
| [Main Branch](Guide/main_branch.md) | How `main` works — individual nodes, original controller |
| [Test Branch](Guide/test_branch.md) | How `new-control-code` works — state machine, all challenges |
| [Commands Reference](Guide/commands_reference.md) | All ROS topics, launch files, `ros2 param set` commands |
| [Tuning Guide](Guide/tuning_guide.md) | Step-by-step parameter tuning on physical course |
| [Architecture](Guide/architecture.md) | Node graphs, data flow, package structure |

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


