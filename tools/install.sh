#!/bin/bash
# ============================================================
# RISA-Bot Fresh Installation Script
# Automates the setup of a new robot from a fresh clone.
# All third-party dependencies are included in the repo.
#
# Usage (run on the robot after cloning):
#   cd ~/risabotcar_ws
#   bash tools/install.sh
# ============================================================

set -e # Exit on error

echo "============================================================"
echo "🤖 RISA-Bot Fresh Installation Script"
echo "============================================================"

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SRC_DIR="$WS_DIR/src"

echo "[1/6] Initial Checks..."
if [ ! -d "$SRC_DIR/risabot_automode" ] && [ ! -d "$SRC_DIR/control_servo" ]; then
    echo "❌ ERROR: Cannot find RISA-bot source packages."
    echo "Make sure you run this from the workspace root: bash tools/install.sh"
    exit 1
fi

if [ ! -d "$SRC_DIR/YDLidar-SDK" ]; then
    echo "❌ ERROR: YDLidar-SDK not found. Did the clone complete properly?"
    exit 1
fi
echo "✅ Workspace found at $WS_DIR"
sleep 1

echo ""
echo "[2/6] Setting up udev rules..."
sudo -v

# YDLidar Rules
echo "  -> Installing YDLidar udev rules..."
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar.rules'
sudo sh -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar-V2.rules'
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"067b\", ATTRS{idProduct}==\"2303\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ydlidar\"" > /etc/udev/rules.d/ydlidar-2303.rules'

# Astra Camera Rules
echo "  -> Installing Astra Camera udev rules..."
if [ -f "$SRC_DIR/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" ]; then
    sudo cp "$SRC_DIR/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" /etc/udev/rules.d/
    echo "✅ Copied 56-orbbec-usb.rules"
fi

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
echo "✅ Udev rules applied. (Note: group changes require logout/login to fully apply)"
sleep 1

echo ""
echo "[3/6] Sourcing ROS 2 Environment..."
if [ -f "/opt/tros/humble/setup.bash" ]; then
    echo "✅ Found Horizon TROS Humble"
    source /opt/tros/humble/setup.bash
elif [ -f "/opt/tros/setup.bash" ]; then
    echo "✅ Found Horizon TROS"
    source /opt/tros/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✅ Found standard ROS 2 Humble"
    source /opt/ros/humble/setup.bash
else
    echo "❌ ERROR: Neither /opt/ros/humble nor /opt/tros setups were found!"
    exit 1
fi
sleep 1

echo ""
echo "[4/6] Building YDLidar-SDK..."
if [ -d "$SRC_DIR/YDLidar-SDK" ]; then
    cd "$SRC_DIR/YDLidar-SDK"
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    echo "✅ YDLidar-SDK built and installed"
else
    echo "❌ ERROR: YDLidar-SDK folder missing!"
    exit 1
fi
sleep 1

echo ""
echo "[5/6] Building ROS 2 Workspace..."
cd "$WS_DIR"
echo "  -> Installing rosdep dependencies..."
sudo apt update
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || echo "⚠️  Some rosdep installations may have been skipped."

echo "  -> colcon build..."
colcon build --symlink-install
echo "✅ Workspace built successfully!"
sleep 1

echo ""
echo "[6/6] Sourcing workspace into ~/.bashrc..."
if ! grep -q "risabotcar_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/risabotcar_ws/install/setup.bash" >> ~/.bashrc
    echo "✅ Added workspace source to ~/.bashrc"
else
    echo "✅ Workspace already sourced in ~/.bashrc"
fi

echo ""
echo "============================================================"
echo "🎉 INSTALLATION COMPLETE! 🎉"
echo "============================================================"
echo "Next steps:"
echo "1. REBOOT to apply udev rules and group permissions."
echo "2. (Optional) Run 'sudo bash tools/setup_autostart.sh' for auto-launch on boot."
echo "============================================================"
