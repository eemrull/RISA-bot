# Tuning Guide — Physical Course

Step-by-step parameter adjustments for when testing on the course.
**Rule: Tune in this order.** Lane following first — everything else depends on it.

---

## Prerequisites

```bash
# Terminal 1: Launch everything
ros2 launch risabot_automode competition.launch.py
```

Open the **web dashboard** at `http://<robot_ip>:8080` — you can tune most parameters directly from the browser without needing a second terminal.

Use **Start button** on controller to toggle auto/manual. Switch to manual whenever the robot misbehaves.

---

## Step 1: Lane Following

Place robot on a **straight section**. Toggle to auto mode.

```bash
ros2 topic echo /lane_error      # watch live error values
```

| Symptom | Fix |
|---|---|
| Jitters on straight road | Set `steering_gain` → 0.3 |
| Still jittery | Set `smoothing_alpha` → 0.15 |
| Too slow to respond to curves | Set `smoothing_alpha` → 0.5 |
| Doesn't turn enough on curves | Set `steering_gain` → 0.7 |
| Not detecting white lines | Set `white_threshold` → 120 |
| Detects noise as lines | Set `white_threshold` → 180 |
| Reads too far ahead | Set `crop_ratio` → 0.3 |

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
| Stops too far away | Set `min_obstacle_distance` → 0.35 |
| Hits object before stopping | Set `min_obstacle_distance` → 0.55 |

---

## Step 3: Obstruction Avoidance (Lateral Dodge)

Place obstacle in the lane. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: OBSTRUCTION"
```

| Symptom | Fix |
|---|---|
| Doesn't dodge early enough | Set `detect_dist` → 0.65 |
| Doesn't steer far enough | Set `steer_angular` → 0.8 |
| Clips while passing | Set `pass_duration` → 2.5 |
| Overshoots returning to lane | Set `steer_back_duration` → 1.0 |

---

## Step 4: Traffic Light

Hold colored cards in front of camera. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: TRAFFIC_LIGHT"
ros2 topic echo /traffic_light_state
```

| Symptom | Fix |
|---|---|
| Not detecting any color | Set `sat_min` → 50 then `val_min` → 50 |
| Confusing red/green | Narrow the H ranges for each color |
| False positives | Set `min_pixel_count` → 100 |

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
| Gate closed but reads OPEN | Set `min_gate_points` → 3 |
| Gate open but reads CLOSED | Set `distance_variance_max` → 0.08 |
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
| Oscillates between walls | Set `kp` → 0.8 |
| Still oscillating | Set `kd` → 0.5 |
| Drifts to one side | Adjust `target_center_dist` ±0.05 |
| Not entering tunnel mode | Set `min_wall_points` → 2 |
| Too fast in tunnel | Set `forward_speed` → 0.10 |

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

| Parameter | Default | Purpose |
|---|---|---|
| `dist_roundabout` | 1.5 | How far through roundabout |
| `dist_boom_gate_1_pass` | 0.5 | After boom gate 1 |
| `dist_boom_gate_2_pass` | 0.5 | After boom gate 2 |
| `dist_hill` | 1.0 | Over the hill |
| `dist_bumper` | 0.8 | Over bumpers |
| `dist_traffic_light_pass` | 0.5 | After green light |
| `dist_drive_to_perp` | 1.0 | Parallel → perp parking |

| Symptom | Fix |
|---|---|
| Transitions too early | Increase the relevant `dist_*` parameter |
| Stuck in a state too long | Decrease the relevant `dist_*` parameter |
| Doesn't move forward | Set `forward_speed` → 0.2 |

---

## Quick Cheat Sheet

The 6 most common params you'll adjust (all on the `auto_driver` or `line_follower_camera` nodes):

| Parameter | Node | Default |
|---|---|---|
| `forward_speed` | auto_driver | 0.15 |
| `steering_gain` | auto_driver | 0.5 |
| `dist_roundabout` | auto_driver | 1.5 |
| `smoothing_alpha` | line_follower_camera | 0.3 |
| `kp` | tunnel_wall_follower | 1.2 |
| `drive_speed` | parking_controller | 0.15 |

> 💡 **Tip:** Use the dashboard at `http://<robot_ip>:8080` for faster tuning — expand the relevant group and click Get/Set.
