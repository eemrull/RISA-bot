# Competition Hardware Validation Guide

Comprehensive test plan, acceptance criteria, failsafe verification, and automated regression testing for hardware validation.

---

## 1. Primary Launch Selection

- **`bringup.launch.py` (Full Competition Stack)**: Camera, LiDAR, TF, Perception, YOLOv5s BPU, Brain, Safety Controller, Joystick, Servo Controller, Web Dashboard, go2rtc, and SLAM.
- **`bringup.launch.py slam:=false`**: Full competition stack with SLAM scan matcher disabled for low CPU overhead.
- **`lane_test.launch.py` (Isolated Lane Test)**: Camera, Line Follower, Joystick, and Auto Driver.
- **`slam_test.launch.py` (Isolated SLAM Test)**: LiDAR, Odometry TF, and SLAM Toolbox.

---

## 2. Pre-Run Checklist

1. Rebuild and source:
   ```bash
   cb && sos
   ```
2. Launch full competition stack:
   ```bash
   ros2 launch risabot_automode bringup.launch.py
   ```
3. Confirm critical topics are active:
   ```bash
   ros2 topic list | grep -E "cmd_vel_auto_raw|cmd_vel_auto|loop_stats|health_status|odom|dashboard_state|map"
   ```
4. Verify system health:
   ```bash
   ros2 topic echo /health_status --once
   ```
   - *Expected:* `"ok": true`

---

## 3. Gamepad Ghost Input & Safety Unlock Verification

**Goal:** Verify that controller analog axis drift on startup cannot move the robot.

1. Keep sticks and triggers untouched at launch.
2. Press **A** or **Y** once.
   - *Expected:* Controller unlocks, but robot does not move.
3. Center sticks to neutral.
   - *Expected:* Terminal logs `Controller neutral detected, manual drive enabled`.
4. Monitor `/cmd_vel`:
   ```bash
   ros2 topic echo /cmd_vel
   ```
   - *Expected:* Strict zero velocity until manual joystick input is given.

---

## 4. Failsafe & Emergency Stop Tests

### A. E-Stop Verification
```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```
- *Expected:*
  - `/cmd_vel_auto` immediately drops to zero.
  - `auto_driver` state displays `EMERGENCY_STOP`.
  - Web Dashboard indicates active safety stop.

Clear E-stop:
```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
```

### B. Command Safety Dropout Test
1. Engage autonomous mode (`/auto_mode = true`).
2. Simulate process crash by terminating `cmd_safety_controller`.
3. *Expected:* Within `auto_cmd_timeout` (0.4 s), `servo_controller` halts motors.

---

## 5. Control Loop Jitter & Frequency Benchmarks

Monitor loop statistics:
```bash
ros2 topic echo /loop_stats
```

**Target Pass Criteria:**
- `auto_driver:auto_driver_cmd` average $\ge 45\text{ Hz}$
- `cmd_safety_controller:cmd_safety` average $\ge 45\text{ Hz}$
- `servo_controller:servo_encoder` average $\ge 18\text{ Hz}$
- `servo_controller:servo_hw` average $\ge 9\text{ Hz}$
- Overrun count remains bounded and does not steadily grow.

---

## 6. Automated Rosbag Regression Validator

Record a 45-second test lap:
```bash
ros2 bag record /loop_stats /health_status /cmd_safety_status /cmd_vel_auto /odom
```

Run automated analysis:
```bash
ros2 run risabot_automode bag_regression_validator --ros-args \
  -p window_sec:=45.0 \
  -p output_file:=/tmp/risa_regression.json
```

**Validation Checks Reported:**
- Loop frequency ratios & jitter
- Safety clamp violations & timeout counters
- Health monitor error states
- Maximum speed and acceleration limits

---

## 7. Competition Acceptance Checklist

- [ ] Controller unlock sequence required before motion is permitted
- [ ] Emergency stop halts robot within one 20 ms cycle
- [ ] Loss of heartbeat halts motors within 0.4 s
- [ ] Control loop maintains $>45\text{ Hz}$ with low jitter
- [ ] BPU AI reliably detects parking and hill signs at $>5\text{ Hz}$
- [ ] 2D SLAM maps track boundaries without reflection artifacts
- [ ] Web dashboard reflects live camera views and parameter updates
