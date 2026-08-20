# RISA-bot Hardware Deployment & Setup Guide

Step-by-step setup guide for deploying RISA-bot software stack from scratch onto a new Horizon Sunrise RDK X5 robot.

---

## Step 0 — Hardware & Network Connectivity

1. Power on the robot (12V battery to Rosmaster board; USB-C power to RDK X5).
2. Connect to the robot via SSH:
   ```bash
   ssh sunrise@<ROBOT_IP>    # Default password: sunrise
   ```

---

## Step 1 — Clone the Repository

```bash
cd ~
git clone https://github.com/eemrull/RISA-bot.git risabotcar_ws
cd risabotcar_ws
```

---

## Step 2 — Run the Automated Installation

```bash
cd ~/risabotcar_ws
bash tools/install.sh
```

**The installer automatically:**
- Configures ROS 2 and system `apt` dependencies (`joy`, `opencv`, build tools, `libuvc`, `magic_enum`).
- Installs `Rosmaster_Lib` via pip.
- Compiles `YDLidar-SDK` from source.
- Installs udev hardware binding rules for Rosmaster motor board, YDLiDAR, and Orbbec Astra camera.
- Performs the initial `colcon build --symlink-install`.
- Adds build aliases (`cb`, `cbp`, `cbc`, `cbd`, `sos`) and environment sourcing to `~/.bashrc`.
- Configures `COLCON_IGNORE` on C++ packages so subsequent Python builds take ~5 seconds.

---

## Step 3 — Apply Udev Rules & Verify USB Devices ⚠️ CRITICAL

Reboot the robot or reload udev rules, then replug USB cables:

```bash
sudo reboot
```

After rebooting, log in and verify symlinks:

```bash
ls -l /dev/myserial   # MUST point to ttyUSB0 (Rosmaster Motor Board, CH340 chip 1a86:7523)
ls -l /dev/ydlidar    # MUST point to ttyUSB1 (YDLiDAR, CP2102 chip 10c4:ea60)
```

If symlinks are swapped or missing:
```bash
sudo tee /etc/udev/rules.d/99-risabot.rules > /dev/null << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="myserial"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```
*Then physically unplug and replug both USB cables.*

---

## Step 4 — Deploy the Compiled BPU AI Model

The `signage_detector` node requires the compiled YOLOv5s INT8 model `risabot_signs_640x640_nv12.bin` at `/home/sunrise/`:

### Option A: Via SCP from Developer PC
```bash
scp tools/bpu_model/model_output/risabot_signs_640x640_nv12.bin sunrise@<ROBOT_IP>:/home/sunrise/
```

### Option B: Via Companion App or Web Portal
1. Open the [RisaBotApp](file:///c:/Users/eemrull/RISA-bot/RisaBotApp) Settings screen (or visit `http://<ROBOT_IP>:8000`).
2. Click **Upload Model** and select `risabot_signs_640x640_nv12.bin`.
3. The portal validates the BPU magic bytes, places the model at `/home/sunrise/`, and sets correct permissions.

### Verify Model Execution on Robot
```bash
python3 tools/bpu_model/verify_bpu.py
```
*Expected output: `SUCCESS: BPU model loaded successfully.`*

---

## Step 5 — Launch the Autonomous Stack

```bash
ros2 launch risabot_automode bringup.launch.py
```

Wait ~10 seconds for all delayed nodes to start up. Verify active nodes:

```bash
ros2 node list
```

**Expected Node Graph:**
- `/astra_camera_container` & `/camera/camera`
- `/ydlidar_ros2_driver_node`
- `/base_to_laser` (Static TF, yaw=$\pi$)
- `/line_follower_camera`
- `/signage_detector` (BPU)
- `/boom_gate_detector`
- `/tunnel_wall_follower`
- `/obstacle_avoidance_node` & `/obstacle_avoidance_camera`
- `/auto_driver` (Brain)
- `/cmd_safety_controller`
- `/servo_controller`
- `/joy_node`
- `/health_monitor`
- `/dashboard` (Port 8080)
- `/ros2go2rtc_bridge` (Port 1985)
- `/odom_tf_publisher`
- `/slam_toolbox`

---

## Step 6 — Controller Unlock & Manual Verification

1. Connect the gamepad controller via USB or wireless dongle (`/dev/input/js0`).
2. Press **Y** or **A** once.
   - Terminal logs: `Controller unlocked (Button press detected)`
3. Center both analog sticks to neutral.
   - Terminal logs: `Controller neutral detected, manual drive enabled`
4. Drive manually with Left Stick to verify motor and Ackermann steering direction.
5. Press **Start button** (Button 7) to engage Autonomous Mode.

---

## Step 7 — Verification Matrix

| Verification Item | Command | Expected Output |
|---|---|---|
| Camera RGB Stream | `ros2 topic hz /camera/color/image_raw` | ~30 Hz |
| LiDAR Scan | `ros2 topic hz /scan` | ~10 Hz |
| Lane Detection | `ros2 topic hz /lane_error` | ~31 Hz |
| Odometry | `ros2 topic echo /odom --once` | Valid pose & velocity |
| Web Dashboard | Open `http://<ROBOT_IP>:8080` | HUD, camera, and map rendered |
| Video Stream (go2rtc) | Open `http://<ROBOT_IP>:1984` | Live go2rtc dashboard |
| System Health | `ros2 topic echo /health_status --once` | `"ok": true` |

---

## Step 8 — Autostart Service Setup (Optional)

To configure the robot to automatically launch on boot:

```bash
cd ~/risabotcar_ws
sudo bash tools/setup_autostart.sh
```

To control the systemd service manually:
```bash
sudo systemctl start risabot
sudo systemctl stop risabot
sudo systemctl status risabot
```
