# RISA-Bot Odometry & Kinematics Guide

This guide explains how wheel odometry, encoder tick integration, Ackermann steering kinematics, and SLAM transforms are implemented on RISA-bot in ROS 2.

---

## 1. Hardware Odometry Implementation

Hardware odometry is computed in [`src/control_servo/control_servo/servo_controller.py`](../src/control_servo/control_servo/servo_controller.py) and published on `/odom` (`nav_msgs/msg/Odometry`) and `/odom/path_length` (`std_msgs/msg/Float32`).

### Key Implementation Features
- **Encoder Sampling**: The Yahboom Rosmaster board refreshes encoder counters every 40 ms. Position integrates raw tick deltas directly to avoid discretization noise.
- **Velocity Differencing**: Velocity is computed by differencing cumulative path length across a sliding window (`odom_vel_window_sec: 0.2s`).
- **Publish Rate**: Broadcasts at **50 Hz** to match the control loop.
- **Polarity Inversion**: Driving forward decrements the physical counter on this hardware, so `odom_reverse_polarity: true` inverts the delta to maintain positive forward distance.
- **Encoder Jump Rejection**: Delta jumps exceeding `encoder_jump_threshold` (800 ticks) are filtered to prevent communication packet corruption from corrupting the pose.

---

## 2. SLAM Transform Integration (`odom_tf_publisher`)

[`src/risabot_slam/risabot_slam/odom_tf_publisher.py`](../src/risabot_slam/risabot_slam/odom_tf_publisher.py) combines cumulative distance from `/odom/path_length` with fused heading from the onboard IMU (`/imu/rpy` yaw) to broadcast the `odom -> base_link` TF transform at **25 Hz**.

> **Why IMU Yaw?** Integrating yaw from commanded servo steering angle drifts rapidly due to tire slip and mechanical linkage backlash. Using the physical IMU gyro provides a stable heading reference for `slam_toolbox`.

---

## 3. Calibration Procedure

### Encoder Resolution Calibration (`ticks_per_meter`)

1. Mark a start and finish line on the floor separated by exactly **2.00 meters**.
2. Align the front wheel contact patch with the start line.
3. Drive forward in a straight line until the contact patch reaches the finish line.
4. Read the reported distance $D_{\text{reported}}$ on the dashboard or via `ros2 topic echo /odom/path_length --once`.
5. Compute and update `ticks_per_meter`:
   $$\text{new\_ticks\_per\_meter} = \text{old\_ticks\_per\_meter} \times \left(\frac{D_{\text{reported}}}{2.00}\right)$$

> **Calibrated Benchmark:** On this platform, `ticks_per_meter` is calibrated to **`6249.0`** (tested across six 2 m legs with forward/reverse balance).

---

## 4. Key Runtime Parameters

| Parameter | Node | Default | Description |
|---|---|---|---|
| `ticks_per_meter` | `servo_controller` | `6249.0` | Calibrated encoder tick resolution |
| `odom_reverse_polarity` | `servo_controller` | `true` | Inverts encoder count direction |
| `odom_publish_rate` | `servo_controller` | `50.0` | Odometry topic frequency (Hz) |
| `encoder_jump_threshold`| `servo_controller` | `800.0` | Spike filter delta threshold |
| `wheel_base` | `servo_controller` | `0.14` | Distance between front and rear axles (m) |
| `steering_max_deg` | `servo_controller` | `50.0` | Maximum Ackermann steering angle (deg) |
| `auto_right_steer_boost`| `servo_controller` | `1.30` | Asymmetric right-turn servo multiplier |
| `publish_rate` | `odom_tf_publisher` | `25.0` | TF broadcast rate for SLAM (Hz) |

---

## 5. Runtime Tuning Commands

```bash
# Update encoder resolution
ros2 param set /servo_controller ticks_per_meter 6249.0

# Adjust right-turn boost if understeering
ros2 param set /servo_controller auto_right_steer_boost 1.35

# Adjust velocity deadband to prevent standstill drift
ros2 param set /servo_controller odom_velocity_deadband 0.02
```
