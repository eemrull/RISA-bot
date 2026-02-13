# Challenge & Code Breakdown

This guide explains how each competition challenge is implemented in the code (`src/risabot_automode/risabot_automode`).

---

## 1. Challenge State Machine (`auto_driver.py`)

**File:** [`auto_driver.py`](../src/risabot_automode/risabot_automode/auto_driver.py)

The robot uses a **State Machine** to know which challenge it is currently facing. It does NOT try to detect everything at once. It only looks for the specific sensor data relevant to the current challenge.

### The Sequence
```mermaid
graph TD
    start[START] --> lane1[LANE_FOLLOW]
    lane1 -->|Dist > 1.0m && Obstacle| obs[OBSTRUCTION]
    obs -->|Dodge Complete| lane2[LANE_FOLLOW]
    lane2 -->|Dist > 3.0m| round[ROUNDABOUT]
    round -->|Exit Roundabout| tunnel[TUNNEL]
    tunnel -->|Tunnel Exit| gate[BOOM_GATE_TUNNEL]
    gate -->|Gate Open| loops[LANE_FOLLOW (Loops)]
    loops -->|Lap 2| light[TRAFFIC_LIGHT]
    light -->|Green| park[PARKING]
    park -->|Complete| finish[FINISHED]
```

**How it works:**
- `auto_driver` subscribes to **all** module topics.
- In `publish_cmd_vel()`, it checks `self.state`.
- **If State = TUNNEL**: It ignores the line follower and listens to `/tunnel_cmd_vel`.
- **If State = PARKING**: It ignores everything and listens to `/parking_cmd_vel`.
- **If State = LANE_FOLLOW**: It uses the camera line follower + standard obstacle stop.

---

## 2. Traffic Light

**File:** [`traffic_light_detector.py`](../src/risabot_automode/risabot_automode/traffic_light_detector.py)
**Topic:** `/traffic_light_state` (red, yellow, green, unknown)

### How it works
1. **Input:** Color camera image (`/camera/color/image_raw`).
2. **HSV Thresholding:** Filters the image for Red, Yellow, and Green pixels.
   - *Tunable:* `red_h_low`, `green_h_high`, `val_min`, etc.
3. **Contour Detection:** Finds circular blobs of the filtered color.
4. **Logic:**
   - If a Red/Yellow circle is found → Publishes `red` or `yellow`.
   - If a Green circle is found → Publishes `green`.
   - `auto_driver` stops if state is `red` or `yellow`. It goes if `green`.

---

## 3. Boom Gate

**File:** [`boom_gate_detector.py`](../src/risabot_automode/risabot_automode/boom_gate_detector.py)
**Topic:** `/boom_gate_open` (True/False)

### How it works
1. **Input:** LiDAR scans (`/scan`).
2. **Region of Interest (ROI):** Looks only at a specific slice in front of the robot (e.g., ±20 degrees, 0.1m to 0.8m).
3. **Cluster Detection:**
   - If it sees a **dense cluster** of points in that ROI (a horizontal bar), it assumes the gate is **CLOSED**.
   - If the ROI is clear, the gate is **OPEN**.
4. **Hysteresis:** Uses a counter (`consecutive_open_scans`) to prevent flickering. The gate must be seen as open for N consecutive frames before the robot moves.

---

## 4. Tunnel

**File:** [`tunnel_wall_follower.py`](../src/risabot_automode/risabot_automode/tunnel_wall_follower.py)
**Topics:** `/tunnel_detected` (Bool), `/tunnel_cmd_vel` (Twist)

### How it works
1. **Input:** LiDAR scans (`/scan`).
2. **Wall Detection:** splits scan into Left and Right sides.
3. **Error Calculation:** `error = left_dist - right_dist`.
   - If `error > 0`, we are closer to the right wall → turn Left.
   - If `error < 0`, we are closer to the left wall → turn Right.
4. **PD Control:** `ang_vel = (kp * error) + (kd * derivative)`.
   - *Tunable:* `kp`, `kd`, `target_center_dist`.
5. **Auto Driver Logic:** When `auto_driver` enters `TUNNEL` state, it completely ignores the camera (often dark/confusing in tunnels) and drives solely off this LiDAR wall follower.

---

## 5. Obstruction Avoidance (Lateral Dodge)

**File:** [`obstruction_avoidance.py`](../src/risabot_automode/risabot_automode/obstruction_avoidance.py)
**Topics:** `/obstruction_active` (Bool), `/obstruction_cmd_vel` (Twist)

### How it works
1. **Detection:** LiDAR sees an object in the lane (closer than `detect_dist`).
2. **Decision:** Checks which side has more space (Left vs Right).
3. **Maneuver (Timed State Machine):**
   - **Phase 1 (Steer Out):** Turn 35° out of the lane for `steer_out_duration`.
   - **Phase 2 (Pass):** Drive straight parallel to the object for `pass_duration`.
   - **Phase 3 (Return):** Turn -35° back into the lane for `steer_back_duration`.
4. **Completion:** Publishes `active=False`, protecting `auto_driver` until the maneuver is fully done.

---

## 6. Parking

**File:** [`parking_controller.py`](../src/risabot_automode/risabot_automode/parking_controller.py)
**Topics:** `/parking_cmd_vel`, `/parking_complete`, `/parking_signboard_detected`

### How it works
1. **Sign Detection:** Uses the camera to look for the Parking Signboard (using Blue/Red HSV thresholding or shape detection).
2. **Phase Machine:**
   - **Parallel:**
     1. `FORWARD`: Drive past the spot (distance via Odometry).
     2. `STEER_REVERSE`: Reverse while turning into the spot.
     3. `STRAIGHTEN`: Straighten wheels and reverse fully in.
     4. `WAIT`: Stop for 3 seconds.
     5. `EXIT`: Drive forwards and turn out.
   - **Perpendicular:**
     1. `TURN_IN`: 90° turn into the spot.
     2. `FORWARD`: Drive in until LiDAR sees the back wall.
     3. `WAIT`: Stop.
     4. `REVERSE_OUT`: Back out and turn to rejoin lane.
3. **Odometry:** Heavily relies on `/odom` to measure `parallel_forward_dist`, `parallel_reverse_dist`, etc.

---

## 7. Line Follower & General Driving (Lane Follow)

**File:** [`line_follower_camera.py`](../src/risabot_automode/risabot_automode/line_follower_camera.py)
**Topic:** `/lane_error` (Float32)

### How it works
1. **Image Processing:** Crops bottom 40% of image. Filters for **White** pixels.
2. **Histogram:** Sums pixels vertically to find "peaks" (where the white lines are).
3. **Midpoint:** Calculates the center between Left and Right lines.
4. **Error:** `error = image_center - lane_center`.
5. **Cleaning:**
   - **Dead Zone:** Ignores tint errors (<0.03) to drive straight on straights.
   - **Smoothing:** Uses Exponential Moving Average (EMA) to filter out camera noise.
