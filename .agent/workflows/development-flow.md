---
description: RISA-bot development and deployment workflow
---

# RISA-bot Development Flow

## Branch Strategy
- **`main`** — Individual node testing with `run_risabot` (astra, lidar, etc.)
- **`test`** — Competition mode with `risabot` alias (runs competition.py)

## Everyday Workflow

### 1. Develop on PC
```bash
# Edit code in VS Code on your PC (c:\Users\eemrull\RISA-bot\)
# Work on whichever branch you need
git checkout main   # or test
# Make your changes...
git add .
git commit -m "your message"
git push
```

### 2. Deploy to Robot
```bash
ssh risabot
cd ~/risabotcar_ws/src/risabot_automode   # or wherever RISA-bot is cloned
git checkout main   # match the branch you pushed to
git pull
```

### 3. Build
```bash
cd ~/risabotcar_ws
cb                  # normal rebuild (fast, ~30s for Python packages)
sos                 # source the workspace
```

### 4. Run
- **Main branch**: `run_risabot` (launches all nodes: astra, lidar, etc.)
- **Test branch**: `risabot` (launches competition.py)

### 5. Switch Branches (on robot)
```bash
cd ~/risabotcar_ws/src/risabot_automode
git checkout test   # or main
git pull
cd ~/risabotcar_ws
cb                  # rebuild
sos                 # re-source
```

## Build Commands Cheat Sheet

| Alias | When to Use |
|-------|-------------|
| `cb`  | Normal rebuild after code changes |
| `cbp <pkg>` | Rebuild only one package (fastest) |
| `cbc` | Clean rebuild — use when stuff is broken |
| `sos` | Re-source the workspace (after any build) |

## Troubleshooting

### astra_camera fails with `openni2_redist` missing
```bash
cp -r ~/backups/openni2_redist ~/risabotcar_ws/src/ros2_astra_camera/astra_camera/
colcon build --symlink-install --packages-select astra_camera
```

### Warnings about AMENT_PREFIX_PATH after cbc
Already fixed in your `cbc` alias. If they appear, run:
```bash
unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/humble/setup.bash
```

### Third-party packages disappear
These are gitignored and should never change with branch switching.
If somehow lost, restore from backups or re-clone:
```bash
# ros2_astra_camera — use your backup
# YDLidar-SDK — git clone https://github.com/YDLIDAR/YDLidar-SDK.git
# ydlidar_ros2_driver — git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
```

## Key Rules
1. **Only `git checkout/pull` inside your package dir** — never from workspace root
2. **Third-party packages are separate** — ros2_astra_camera, YDLidar-SDK, ydlidar_ros2_driver are gitignored
3. **Always `sos` after building** before launching nodes
4. **Use `cb` not `cbc`** unless builds are broken
5. **`openni2_redist` backup** lives at `~/backups/openni2_redist`
