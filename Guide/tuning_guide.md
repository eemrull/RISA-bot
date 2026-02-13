# Tuning Guide — Physical Course

Step-by-step parameter adjustments for on when testing on the course.
**Rule: Tune in this order.** Lane following first — everything else depends on it.

---

## Prerequisites

```bash
# Terminal 1: Launch everything
ros2 launch risabot_automode competition.launch.py

# Terminal 2: Keep open for tuning commands
```

Use **Start button** on controller to toggle auto/manual. Switch to manual whenever the robot misbehaves.

---

## Step 1: Lane Following

Place robot on a **straight section**. Toggle to auto mode.

```bash
ros2 topic echo /lane_error      # watch live error values
```

| Symptom | Fix |
|---|---|
| Jitters on straight road | `ros2 param set /auto_driver steering_gain 0.3` |
| Still jittery | `ros2 param set /line_follower_camera smoothing_alpha 0.15` |
| Too slow to respond to curves | `ros2 param set /line_follower_camera smoothing_alpha 0.5` |
| Doesn't turn enough on curves | `ros2 param set /auto_driver steering_gain 0.7` |
| Not detecting white lines | `ros2 param set /line_follower_camera white_threshold 120` |
| Detects noise as lines | `ros2 param set /line_follower_camera white_threshold 180` |
| Reads too far ahead | `ros2 param set /line_follower_camera crop_ratio 0.3` |

**Tuning order:**
1. Set `steering_gain` = 0.3 → check no shaking on straight
2. Slowly increase until curves work (0.4–0.6 is typical)
3. If still noisy, lower `smoothing_alpha` to 0.2
4. Test curves → if too slow, raise `smoothing_alpha` to 0.4

### Parameter Ranges

| Parameter | Node | Default | Range |
|---|---|---|---|
| `steering_gain` | auto_driver | 0.5 | 0.2 – 1.0 |
| `smoothing_alpha` | line_follower_camera | 0.3 | 0.1 – 1.0 |
| `dead_zone` | line_follower_camera | 0.03 | 0.0 – 0.1 |
| `white_threshold` | line_follower_camera | 150 | 100 – 220 |
| `crop_ratio` | line_follower_camera | 0.4 | 0.2 – 0.6 |

---

## Step 2: Front Obstacle Detection

Place object **in front** at ~0.4m.

```bash
ros2 topic echo /obstacle_front
```

| Symptom | Fix |
|---|---|
| Stops too far away | `ros2 param set /obstacle_avoidance_node min_obstacle_distance 0.35` |
| Hits object before stopping | `ros2 param set /obstacle_avoidance_node min_obstacle_distance 0.55` |

---

## Step 3: Obstruction Avoidance (Lateral Dodge)

Place obstacle in the lane. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: OBSTRUCTION"
```

| Symptom | Fix |
|---|---|
| Doesn't dodge early enough | `ros2 param set /obstruction_avoidance detect_dist 0.65` |
| Doesn't steer far enough | `ros2 param set /obstruction_avoidance steer_angular 0.8` |
| Clips while passing | `ros2 param set /obstruction_avoidance pass_duration 2.5` |
| Overshoots returning to lane | `ros2 param set /obstruction_avoidance steer_back_duration 1.0` |

---

## Step 4: Traffic Light

Hold colored cards in front of camera. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: TRAFFIC_LIGHT"
ros2 topic echo /traffic_light_state
```

| Symptom | Fix |
|---|---|
| Not detecting any color | `ros2 param set /traffic_light_detector sat_min 50` then `val_min 50` |
| Confusing red/green | Narrow the H ranges for each color |
| False positives | `ros2 param set /traffic_light_detector min_pixel_count 100` |

> ⚠️ HSV thresholds are **very sensitive to lighting**. Always tune at the competition venue.

---

## Step 5: Boom Gate

Drive toward the boom gate. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: BOOM_GATE_2"
ros2 topic echo /boom_gate_open
```

| Symptom | Fix |
|---|---|
| Gate closed but reads OPEN | `ros2 param set /boom_gate_detector min_gate_points 3` |
| Gate open but reads CLOSED | `ros2 param set /boom_gate_detector distance_variance_max 0.08` |
| Detection range wrong | Adjust `min_detect_dist` and `max_detect_dist` |

---

## Step 6: Tunnel Wall Following

Drive into the tunnel. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: TUNNEL"
ros2 topic echo /tunnel_detected
```

| Symptom | Fix |
|---|---|
| Oscillates between walls | `ros2 param set /tunnel_wall_follower kp 0.8` |
| Still oscillating | `ros2 param set /tunnel_wall_follower kd 0.5` |
| Drifts to one side | Adjust `target_center_dist` ±0.05 |
| Not entering tunnel mode | `ros2 param set /tunnel_wall_follower min_wall_points 2` |
| Too fast in tunnel | `ros2 param set /tunnel_wall_follower forward_speed 0.10` |

---

## Step 7: Parking (tune last)

> ⚠️ Start with **very low speeds** (0.08) and short distances (0.15). Increase gradually.

```bash
# Parallel
ros2 topic pub --once /set_challenge std_msgs/String "data: PARALLEL_PARK"
ros2 topic pub --once /parking_command std_msgs/String "data: parallel"

# Perpendicular
ros2 topic pub --once /set_challenge std_msgs/String "data: PERPENDICULAR_PARK"
ros2 topic pub --once /parking_command std_msgs/String "data: perpendicular"
```

| Symptom | Fix |
|---|---|
| Doesn't pull far enough past slot | increase `parallel_forward_dist` |
| Doesn't enter slot fully | increase `parallel_reverse_dist` |
| Turns too sharply | decrease `parallel_steer_angle` |
| Doesn't turn enough | increase `parallel_steer_angle` |
| Too fast | decrease `drive_speed` and `reverse_speed` |

## Step 8: State Transition Distances

These control when the state machine auto-advances. Run a full lap and adjust:

```bash
ros2 param set /auto_driver dist_roundabout 2.0         # how far through roundabout
ros2 param set /auto_driver dist_boom_gate_1_pass 0.5    # after boom gate 1
ros2 param set /auto_driver dist_boom_gate_2_pass 0.5    # after boom gate 2
ros2 param set /auto_driver dist_hill 1.0                # over the hill
ros2 param set /auto_driver dist_bumper 0.8              # over bumpers
ros2 param set /auto_driver dist_traffic_light_pass 0.5  # after green light
ros2 param set /auto_driver dist_drive_to_perp 1.0       # parallel → perp parking
```

| Symptom | Fix |
|---|---|
| Transitions too early | Increase the relevant `dist_*` parameter |
| Stuck in a state too long | Decrease the relevant `dist_*` parameter |
| Doesn't move forward | `ros2 param set /auto_driver forward_speed 0.2` |

---

## Quick Cheat Sheet

```bash
# The 6 most common params you'll adjust:
ros2 param set /auto_driver forward_speed 0.15
ros2 param set /auto_driver steering_gain 0.5
ros2 param set /auto_driver dist_roundabout 1.5
ros2 param set /line_follower_camera smoothing_alpha 0.3
ros2 param set /tunnel_wall_follower kp 1.2
ros2 param set /parking_controller drive_speed 0.15
```
