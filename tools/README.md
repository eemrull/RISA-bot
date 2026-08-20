# RISA-Bot Scripts & Developer Utilities

This directory contains automated deployment, environment setup, fleet management, and AI compilation tools for the RISA-bot platform.

---

## 📂 Tools Directory Overview

| Script / Directory | Description |
|---|---|
| [`install.sh`](install.sh) | Primary setup wizard for fresh robots (dependencies, SDK, udev, build, aliases) |
| [`install_deps.sh`](install_deps.sh) | System dependencies, ROS 2 packages, and library build script |
| [`install_bashalias.sh`](install_bashalias.sh) | Generates `~/.bash_aliases` and configures `~/.bashrc` shortcuts |
| [`setup_autostart.sh`](setup_autostart.sh) | Installs `risabot.service` and autostart desktop launcher |
| [`setup_mdns.sh`](setup_mdns.sh) | Configures Avahi mDNS so the robot resolves as `risabot.local` |
| [`setup_wifi.sh`](setup_wifi.sh) | Pre-configures WiFi network connection and priority |
| [`wifi_hotspot_setup.sh`](wifi_hotspot_setup.sh) | Creates a standalone WiFi Access Point on the robot with WPA2-AES |
| [`bulk_setup_robots.py`](bulk_setup_robots.py) | Paramiko-driven multi-robot automated configuration and provisioning script |
| [`deploy_to_robots_paramiko.py`](deploy_to_robots_paramiko.py) | Fleet model deployment and git pull tool over SSH |
| [`bpu_model/`](bpu_model/) | Horizon BPU YOLOv5s training, ONNX patching, Docker quantization toolchain |
| [`go2rtc/`](go2rtc/) | High-performance WebRTC / RTSP / MJPEG low-latency streaming server |
| [`wifi_provisioning/`](wifi_provisioning/) | FastAPI portal backend running on port 8000 |
| [`rosmaster_lib/`](rosmaster_lib/) | Yahboom Rosmaster motor expansion board Python driver library |

---

## 🛠️ Script Details & Usage

### 1. `install.sh`
The primary setup wizard for fresh Horizon RDK X5 robot installations.

```bash
cd ~/risabotcar_ws
bash tools/install.sh
```

**Actions Performed:**
- Verifies workspace structure (`~/risabotcar_ws/src/RISA-bot`)
- Installs udev hardware rules (`99-risabot.rules`, `56-orbbec-usb.rules`)
- Installs ROS 2 packages and builds `YDLidar-SDK` from source
- Executes `colcon build --symlink-install`
- Adds `COLCON_IGNORE` on C++ dependencies to speed up subsequent Python builds
- Generates `~/.bash_aliases`

---

### 2. `install_bashalias.sh`
Lightweight script to regenerate aliases and environment variables without compiling code.

```bash
bash tools/install_bashalias.sh
```

---

### 3. `wifi_hotspot_setup.sh` & `setup_wifi.sh`

```bash
# Connect robot to a local WiFi router or phone hotspot:
sudo bash tools/setup_wifi.sh "MY_WIFI_SSID" "MY_PASSWORD"

# Configure robot to broadcast its own standalone WiFi Access Point:
sudo bash tools/wifi_hotspot_setup.sh
```

---

### 4. `bulk_setup_robots.py` & `deploy_to_robots_paramiko.py`
Automates simultaneous deployment across multiple robots on the competition bench:

```bash
# Deploy latest code and BPU model to all robots:
python3 tools/deploy_to_robots_paramiko.py
```

---

### 5. `bpu_model/` AI Toolchain

Contains everything needed to train, patch, quantize, and verify YOLOv5s models for the Horizon RDK X5 BPU:

- `colab_training_script.py`: Google Colab GPU training script with Roboflow dataset download.
- `patch_onnx_resize.py`: Patches ONNX Resize operators to ensure compatibility with Horizon `hb_mapper`.
- `compile_model.bat`: Windows batch script running Dockerized Horizon BPU compiler (`hb_mapper`).
- `verify_bpu.py`: Standalone Python script testing BPU model load and execution on dummy tensors.
- `verify_live.py`: Real-time diagnostic script running camera frames through the BPU and printing detections.

---

### 6. `wifi_provisioning/` Portal (Port 8000)

FastAPI + uvicorn backend running at `http://<robot_ip>:8000`:
- `GET /api/status`: Robot connectivity and IP status.
- `GET /api/model/info`: Active BPU model metadata (name, size, SHA256).
- `POST /api/model/upload`: Stream-uploads new BPU `.bin` model with magic-byte validation.
- `POST /api/model/rollback`: Restores previous `.bin.bak` model.
- `POST /api/launch_start` & `POST /api/launch_stop`: Controls `risabot.service`.
