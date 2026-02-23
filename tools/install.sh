#!/bin/bash
# ============================================================
# RISA-Bot Fresh Installation Script
# This script automates the setup of a new robot, including
# udev rules, building 3rd-party dependencies, and colcon build.
#
# Usage:
#   cd ~/risabotcar_ws
#   bash tools/install.sh [path_to_backups_folder]
# ============================================================

set -e # Exit on error

echo "============================================================"
echo "🤖 RISA-Bot Fresh Installation Script"
echo "============================================================"

# Default backup path is ~/backups unless specified
BACKUP_DIR=${1:-"$HOME/backups"}
WS_DIR="$HOME/risabotcar_ws"
SRC_DIR="$WS_DIR/src"

echo "[1/7] Initial Checks..."
if [ ! -d "$BACKUP_DIR" ]; then
    echo "❌ ERROR: Backup directory not found at: $BACKUP_DIR"
    echo "Please copy the 4 modified dependency folders to this location first:"
    echo "  - ros2_astra_camera"
    echo "  - openni2_redist"
    echo "  - ydlidar_ros2_driver"
    echo "  - YDLidar-SDK"
    echo "Usage: bash tools/install.sh /path/to/backups"
    exit 1
fi

if [ ! -d "$SRC_DIR/RISA-bot" ] && [ ! -d "$SRC_DIR/risabot_automode" ]; then
    echo "❌ ERROR: Run this script from the workspace root (e.g. ~/risabotcar_ws)"
    exit 1
fi

echo "✅ Backup directory found at $BACKUP_DIR"
sleep 1

echo ""
echo "[2/7] Copying 3rd-party dependencies..."
# Copy the modified packages into the workspace src directory
cp -r "$BACKUP_DIR/ros2_astra_camera" "$SRC_DIR/" 2>/dev/null || echo "⚠️  ros2_astra_camera not found in backup (maybe already copied?)"
cp -r "$BACKUP_DIR/ydlidar_ros2_driver" "$SRC_DIR/" 2>/dev/null || echo "⚠️  ydlidar_ros2_driver not found in backup"
cp -r "$BACKUP_DIR/YDLidar-SDK" "$SRC_DIR/" 2>/dev/null || echo "⚠️  YDLidar-SDK not found in backup"

# Orbbec requires openni2_redist inside its folder
if [ -d "$BACKUP_DIR/openni2_redist" ]; then
    cp -r "$BACKUP_DIR/openni2_redist" "$SRC_DIR/ros2_astra_camera/astra_camera/"
    echo "✅ Copied openni2_redist"
fi
echo "✅ Dependencies copied"
sleep 1

echo ""
echo "[3/7] Setting up udev rules..."
# Request sudo once
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

sudo service udev reload
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
echo "✅ Udev rules applied. (Note: group changes require logout/login to fully apply)"
sleep 1

echo ""
echo "[4/7] Sourcing ROS 2 Environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✅ Found standard ROS 2 Humble"
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/tros/setup.bash" ]; then
    echo "✅ Found Horizon TROS"
    source /opt/tros/setup.bash
else
    echo "❌ ERROR: Neither /opt/ros/humble nor /opt/tros setups were found!"
    exit 1
fi
sleep 1

echo ""
echo "[5/7] Building YDLidar-SDK..."
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
echo "[6/7] Building ROS 2 Workspace..."
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
echo "[7/7] Sourcing workspace into ~/.bashrc..."
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
echo "1. Run 'bash tools/setup_autostart.sh' if you want the robot to launch on boot."
echo "2. REBOOT YOUR ROBOT to apply udev rules and group permissions!"
echo "============================================================"
