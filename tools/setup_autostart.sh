#!/bin/bash
# ============================================================
# RISA-Bot Autostart Setup (Tailscale & Wifi Provisioning)
# Configures the robot to auto-launch all ROS 2 nodes on boot.
#
# Usage (Run on the robot):
#   sudo bash tools/setup_autostart.sh
# ============================================================

set -e

# Resolve the repo root from this script's own location, so the go2rtc assets
# under tools/go2rtc/ can be installed no matter where the repo was cloned.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "🤖 RISA-Bot Autostart Setup"
echo "============================"

# ---- 1. Create the startup script ----
echo "📝 Creating startup script at /usr/local/bin/risabot-launch.sh..."

cat > /usr/local/bin/risabot-launch.sh << 'LAUNCH_EOF'
#!/bin/bash
# RISA-Bot startup script — called by systemd
set -e

export HOME=/home/sunrise
export USER=sunrise

# Source ROS2 environment (Horizon TROS / Humble / Iron / Jazzy)
if [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
if [ -f "/home/sunrise/risabotcar_ws/install/setup.bash" ]; then
    source "/home/sunrise/risabotcar_ws/install/setup.bash"
elif [ -f "/home/sunrise/RISA-bot-1/install/setup.bash" ]; then
    source "/home/sunrise/RISA-bot-1/install/setup.bash"
elif [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Wait for the LiDAR's USB-serial adapter to enumerate before launching.
# A fixed sleep is a race: at boot the CP2102 is often not ready in a few
# seconds, and the driver then reads garbage ("Check Sum X != Y", followed by
# "-1 Device Failed"). auto_reconnect usually recovers it, but not always.
LIDAR_DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
echo "[RISABOT] Waiting for LiDAR at $LIDAR_DEV ..."
for i in $(seq 1 30); do
    [ -e "$LIDAR_DEV" ] && break
    sleep 1
done
if [ -e "$LIDAR_DEV" ]; then
    echo "[RISABOT] LiDAR present after ${i}s"
else
    echo "[RISABOT] WARNING: LiDAR not found after 30s — launching anyway"
fi

# Let the adapter settle once enumerated, and give the motor board the same grace.
sleep 3

echo "[RISABOT] Starting full ROS 2 bringup..."
# bringup.launch.py starts sensors, perception, auto_driver, servo_controller, health_monitor, AND dashboard (port 8080)
exec ros2 launch risabot_automode bringup.launch.py
LAUNCH_EOF

chmod +x /usr/local/bin/risabot-launch.sh
echo "  ✅ Startup script created at /usr/local/bin/risabot-launch.sh"

# ---- 2. Install go2rtc (camera streaming server) ----
# ros2go2rtc_bridge serves MJPEG on :1985; go2rtc pulls from it and republishes
# on :1984, which is what the web dashboard's camera panel connects to.
echo "📝 Installing go2rtc..."

GO2RTC_BIN="/home/sunrise/go2rtc"
GO2RTC_URL="https://github.com/AlexxIT/go2rtc/releases/latest/download/go2rtc_linux_arm64"

if [ -x "$GO2RTC_BIN" ]; then
    echo "  ℹ️  go2rtc binary already present — skipping download"
else
    echo "  ⬇️  Downloading go2rtc (linux_arm64)..."
    if curl -fsSL "$GO2RTC_URL" -o "$GO2RTC_BIN"; then
        chmod +x "$GO2RTC_BIN"
        echo "  ✅ go2rtc downloaded"
    else
        rm -f "$GO2RTC_BIN"
        echo "  ⚠️  go2rtc download FAILED — camera feed will not work."
        echo "      Fetch it manually: curl -fsSL $GO2RTC_URL -o $GO2RTC_BIN && chmod +x $GO2RTC_BIN"
    fi
fi
chown sunrise:sunrise "$GO2RTC_BIN" 2>/dev/null || true

if [ -f "$REPO_DIR/tools/go2rtc/go2rtc.yaml" ]; then
    cp "$REPO_DIR/tools/go2rtc/go2rtc.yaml" /home/sunrise/go2rtc.yaml
    chown sunrise:sunrise /home/sunrise/go2rtc.yaml
    cp "$REPO_DIR/tools/go2rtc/go2rtc.service" /etc/systemd/system/go2rtc.service
    echo "  ✅ go2rtc config and service installed"
else
    echo "  ⚠️  $REPO_DIR/tools/go2rtc/ not found — skipping go2rtc config"
fi

# ---- 3. Create systemd service ----
echo "📝 Creating systemd service /etc/systemd/system/risabot.service..."

cat > /etc/systemd/system/risabot.service << 'SERVICE_EOF'
[Unit]
Description=RISA-Bot ROS2 Autostart
# Soft ordering only: if go2rtc fails the robot still drives, it just has no video.
After=network.target network-online.target go2rtc.service
Wants=network.target network-online.target go2rtc.service

[Service]
Type=simple
User=sunrise
Group=sunrise
Environment="HOME=/home/sunrise"
ExecStart=/usr/local/bin/risabot-launch.sh
ExecStop=/bin/bash -c "pkill -f 'ros2' || true"
Restart=always
RestartSec=5
TimeoutStartSec=60

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# ---- 4. Setup user .bashrc for SSH ros2 CLI commands ----
echo "📝 Updating /home/sunrise/.bashrc for ROS 2 CLI commands..."
BASHRC="/home/sunrise/.bashrc"
if ! grep -q "risabotcar_ws/install/setup.bash" "$BASHRC" 2>/dev/null && ! grep -q "RISA-bot-1/install/setup.bash" "$BASHRC" 2>/dev/null; then
    cat >> "$BASHRC" << 'BASHRC_EOF'

# --- RISA-Bot ROS 2 Environment ---
if [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/risabotcar_ws/install/setup.bash" ]; then
    source "$HOME/risabotcar_ws/install/setup.bash"
elif [ -f "$HOME/RISA-bot-1/install/setup.bash" ]; then
    source "$HOME/RISA-bot-1/install/setup.bash"
fi
BASHRC_EOF
fi

# Reload systemd and enable + start services
systemctl daemon-reload

if [ -f /etc/systemd/system/go2rtc.service ]; then
    systemctl enable go2rtc.service
    systemctl restart go2rtc.service
    echo "  ✅ Service 'go2rtc' enabled and started (camera streaming on :1984)"
fi

systemctl enable risabot.service
systemctl restart risabot.service

echo "  ✅ Service 'risabot' created, enabled, and started!"

echo ""
echo "============================================"
echo "✅ AUTOLAUNCH SETUP COMPLETE!"
echo "============================================"
echo "Commands:"
echo "  Check status:   sudo systemctl status risabot"
echo "  View live logs: sudo journalctl -u risabot -f"
echo "  Camera server:  sudo systemctl status go2rtc"
echo "  Dashboard:      http://<robot_ip>:8080"
echo "============================================"
