# Physical Course Parameter Tuning Guide

Step-by-step parameter calibration guide for track testing on the physical competition course.

> **Golden Rule of Tuning:** Always tune in this exact sequence:
> 1. **Lane Following** (base stability)
> 2. **Odometry & Steering Calibration**
> 3. **Perception Thresholds** (Boom Gate, Traffic Light, BPU AI)
> 4. **Tunnel Navigation** (RANSAC PD)
> 5. **Hill Climb & Descent**
> 6. **Parking Maneuvers**

---

## 🛠️ Prerequisites & Setup

```bash
# Terminal 1: Launch full autonomous bringup
ros2 launch risabot_automode bringup.launch.py

# Terminal 2: Keep open for parameter adjustments
```

> **Safety:** Use the **Start button** on the controller to immediately switch to `MANUAL` if the robot deviates or oscillates.

---

## Step 1: Lane Follower Tuning

Place the robot on a **straight section** of the track. Engage autonomous mode and observe `/lane_error`:

```bash
ros2 topic echo /lane_error
```

Open the web dashboard at `http://<ROBOT_IP>:8080` and select the **Lane Lines** debug view:
- **Blue dots** = Left boundary / scanline hit
- **Pink dots** = Right boundary / scanline hit
- **Green dots** = Computed lane center
- **Red vertical line** = Image center reference

### Lane Following Symptoms & Adjustments

| Symptom | Root Cause | Parameter Adjustment |
|---|---|---|
| Weaves / oscillates on straight road | Proportional gain too high | `ros2 param set /auto_driver pid_kp 0.6` |
| Still oscillating after lowering $K_p$ | Damping too low | `ros2 param set /auto_driver pid_kd 0.25` |
| Slow to turn into curves | Proportional gain too low | `ros2 param set /auto_driver pid_kp 1.0` |
| Constant offset / drifts to one side | Steady-state bias | `ros2 param set /auto_driver pid_ki 0.02` |
| Cuts inside on sharp corners | Lookahead window too short | `ros2 param set /line_follower_camera crop_ratio_base 0.55` |
| False border detections on dark floor | Threshold too sensitive | `ros2 param set /line_follower_camera white_threshold 110` |
| Misses dark track markings | Threshold too high | `ros2 param set /line_follower_camera white_threshold 85` |
| Lane error is noisy / twitchy | Kalman measurement noise | `ros2 param set /line_follower_camera kalman_process_noise 0.005` |
| Sluggish steering response | Slew limit too restrictive | `ros2 param set /auto_driver lane_steer_slew 4.0` |

### Core Lane Following Parameters

| Parameter | Node | Default | Recommended Range | Description |
|---|---|---|---|---|
| `forward_speed` | `auto_driver` | `0.15` | 0.10 – 0.25 m/s | Base straight-line cruising speed |
| `pid_kp` | `auto_driver` | `0.80` | 0.50 – 1.40 | Proportional steering gain |
| `pid_ki` | `auto_driver` | `0.01` | 0.00 – 0.03 | Integral steering gain |
| `pid_kd` | `auto_driver` | `0.20` | 0.10 – 0.40 | Derivative steering damping |
| `speed_error_scale` | `auto_driver` | `1.50` | 1.00 – 2.50 | Cornering speed reduction factor |
| `min_turn_speed` | `auto_driver` | `0.40` | 0.30 – 0.60 | Min speed floor in sharp turns |
| `white_threshold` | `line_follower_camera` | `100` | 75 – 140 | Binary gray cutoff (inverted mode) |
| `crop_ratio_base` | `line_follower_camera` | `0.55` | 0.40 – 0.65 | Image crop lookahead ratio |
| `invert_binary` | `line_follower_camera` | `true` | `true` / `false` | True for dark lane tracking |

---

## Step 2: Odometry & Steering Asymmetry Calibration

### A. Ackermann Right-Steer Boost
Due to physical Ackermann steering linkage asymmetry, right turns require slightly more servo throw:

```bash
# If the robot understeers on right turns, increase boost:
ros2 param set /servo_controller auto_right_steer_boost 1.35
```

### B. Encoder Distance Calibration
1. Drive exactly **2.00 meters** straight.
2. Read `/odom/path_length` or `/odom` distance:
3. Update `ticks_per_meter`:
   $$\text{new\_ticks} = \text{old\_ticks} \times \left(\frac{D_{\text{reported}}}{2.00}\right)$$
```bash
ros2 param set /servo_controller ticks_per_meter 6249.0
```

---

## Step 3: Boom Gate Dual-Modality Tuning

Place the robot approaching Boom Gate 2:

```bash
ros2 topic echo /boom_gate_open
```

| Symptom | Parameter Adjustment |
|---|---|
| Gate is closed but reads `open: true` | `ros2 param set /boom_gate_detector cam_red_min_width 60` |
| Raised gate bar false-triggers closed | `ros2 param set /boom_gate_detector cam_roi_y_min 0.55` |
| LiDAR barrier detection too sensitive | `ros2 param set /boom_gate_detector distance_variance_max 0.04` |
| Detection state flickers | `ros2 param set /boom_gate_detector hysteresis 6` |

---

## Step 4: Tunnel Wall Following (RANSAC PD)

Drive into the tunnel corridor:

```bash
ros2 topic echo /tunnel_detected
ros2 topic echo /tunnel_cmd_vel
```

| Symptom | Parameter Adjustment |
|---|---|
| Oscillates between left and right walls | Lower $K_p$, raise $K_d$: `ros2 param set /tunnel_wall_follower kp 3.5` and `kd 0.6` |
| Drifts into one wall on approach | Increase heading gain: `ros2 param set /tunnel_wall_follower kp_heading 1.5` |
| Does not engage tunnel mode | `ros2 param set /tunnel_wall_follower min_wall_points 3` |
| Speed too high in dark corridor | `ros2 param set /tunnel_wall_follower forward_speed 0.10` |

---

## Step 5: Hill Climbing & Descent Tuning

Drive toward the ramp. Monitor `/imu/pitch`:

```bash
ros2 topic echo /imu/pitch
```

| Symptom | Parameter Adjustment |
|---|---|
| Stalls on ramp slope | Increase base speed: `ros2 param set /auto_driver hill_base_speed 0.22` |
| Fishtails off ramp edge | Reduce steering throw: `ros2 param set /auto_driver hill_steer_scale 0.3` |
| Triggers hill mode prematurely on flats | `ros2 param set /auto_driver hill_pitch_threshold 10.0` |
| Runaway speed on descent | `ros2 param set /auto_driver descent_base_speed 0.06` |

---

## Step 6: BPU AI Signage Confidence Thresholds

Adjust per-class confidence thresholds on `/signage_detector`:

```bash
# Make parking detection more sensitive:
ros2 param set /signage_detector thresh_parallelp 0.04
ros2 param set /signage_detector thresh_perpendp 0.04

# Make traffic light detection stricter:
ros2 param set /signage_detector thresh_tl_red 0.30
ros2 param set /signage_detector thresh_tl_green 0.30
```

---

## 💾 Persisting Tuned Parameters

Once satisfied with live parameter values, persist them permanently to disk:
1. Open the Web Dashboard at `http://<ROBOT_IP>:8080`.
2. Open the **Parameters Drawer**.
3. Click **"Save as Default"** to write all active values to `src/risabot_automode/config/params.yaml`.
