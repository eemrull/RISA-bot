# Module 5: Working with Sensors

## Learning Objectives

By the end of this module, you will:
- Understand how camera and LiDAR data flows in ROS 2
- View camera images using ROS tools
- Read and interpret LiDAR scan data
- Know which topics the RISA-bot sensors publish on

## RISA-bot Sensors

| Sensor | Model | Topic | Message Type |
|---|---|---|---|
| Camera | Astra Mini | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Depth | Astra Mini | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| LiDAR | YDLiDAR Tmini Plus | `/scan` | `sensor_msgs/LaserScan` |

## Hands-On: Camera

### 1. Start the camera

```bash
ros2 launch astra_camera astra_mini.launch.py
```

### 2. Check camera topics

```bash
ros2 topic list | grep camera
# /camera/color/camera_info
# /camera/color/image_raw
# /camera/depth/image_raw
```

### 3. Check frame rate

```bash
ros2 topic hz /camera/color/image_raw
# Should be ~30 Hz
```

### 4. View the image (if display available)

```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/color/image_raw from dropdown
```

### 5. Understanding the Image message

```bash
ros2 topic echo /camera/color/image_raw --no-arr
# Shows header (timestamp, frame_id), height, width, encoding
# --no-arr hides the actual pixel data (too large to print)
```

Key fields:
- `height`: 480 pixels
- `width`: 640 pixels
- `encoding`: `rgb8` (3 bytes per pixel: R, G, B)
- `data`: Raw pixel array (480 × 640 × 3 = 921,600 bytes)

## Hands-On: LiDAR

### 1. Start the LiDAR

```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node
```

### 2. Check the scan topic

```bash
ros2 topic hz /scan
# Should be ~8-12 Hz
```

### 3. Read a single scan

```bash
ros2 topic echo /scan --once
```

### 4. Understanding the LaserScan message

```text
header:
  stamp: {sec: ..., nanosec: ...}     ← Timestamp
angle_min: -3.14                       ← Start angle (radians, -180°)
angle_max: 3.14                        ← End angle (radians, +180°)
angle_increment: 0.0087                ← Angle between readings (~0.5°)
range_min: 0.05                        ← Minimum valid range (meters)
range_max: 12.0                        ← Maximum valid range (meters)
ranges: [0.45, 0.46, 0.48, ...]       ← Distance array (one per angle)
```

**Interpreting ranges:**
- `ranges[0]` = distance at `angle_min` (behind-left)
- `ranges[180]` = distance at roughly 0° (straight ahead)
- `ranges[360]` = distance at `angle_max` (behind-right)
- `inf` or values > `range_max` = nothing detected

### 5. Quick distance check

```python
# In Python, to get the distance directly ahead:
import math
front_index = len(ranges) // 2  # Middle of array = straight ahead
front_distance = ranges[front_index]
```

## Exercise

1. Start the camera and check that it's publishing at ~30 Hz
2. Start the LiDAR and read one scan with `--once`
3. Place your hand 30cm in front of the LiDAR — what value do you see in `ranges`?
4. Move your hand to the side — which index in `ranges` changes?
5. **Challenge:** How many range readings are in one scan? (hint: look at `len(ranges)`)

---

**Previous:** [Module 4 — Launch Files](04-launch-files.md)
**Next:** [Module 6 — Joystick Control](06-joystick-control.md)
