# RISA-Bot Odometry Guide

**Prepared for Dr. Zul (Roboteam)**

This guide explains how Odometry (Distance and Position tracking) is implemented on the RISA-Bot in ROS 2. We use two methods:

1. **Hardware Odometry (Active):** Reads physical encoder ticks from the Rosmaster motor board.
2. **Software/Fake Odometry (Fallback):** Integrates commanded velocity (`/cmd_vel`) over time.

---

## 1. Hardware Odometry (Recommended)

The primary odometry is calculated inside the `servo_controller.py` node. It runs a fast 20Hz loop that polls the hardware board (Rosmaster) for motor encoder ticks and calculates distance and yaw based on Ackermann steering kinematics.

- **File Location:** `src/control_servo/control_servo/servo_controller.py`
- **ROS 2 Topic:** Publishes heavily to `nav_msgs/msg/Odometry` on `/odom`

### Reference Code Snippet

From `_encoder_read_loop()` in `servo_controller.py`:

```python
# Constants configured for the robot
self.wheel_base = 0.14        # Distance between front and rear axles (meters)
self.ticks_per_meter = 1050   # Needs physical calibration!

# 1. Read hardware ticks from the Rosmaster driver
ticks = self.bot.get_motor_encoder()

# 2. Calculate the difference (delta) from the last read to get ticks moved
left_ticks = (d_ticks[0] + d_ticks[2]) / 2.0
right_ticks = (d_ticks[1] + d_ticks[3]) / 2.0
avg_ticks = (left_ticks + right_ticks) / 2.0

# 3. Convert ticks to meters
distance = avg_ticks / self.ticks_per_meter
linear_velocity = distance / dt

# 4. Calculate Ackermann steering yaw (using servo angle)
# Servo range: 40-140, Center: 90.
steering_angle_deg = (SERVO_CENTER - self.target_servo_val) * (50.0 / SERVO_RANGE)
steering_angle_rad = math.radians(steering_angle_deg)

if abs(steering_angle_rad) > 0.01:
    turning_radius = self.wheel_base / math.tan(steering_angle_rad)
    angular_velocity = linear_velocity / turning_radius
else:
    angular_velocity = 0.0

yaw_delta = angular_velocity * dt
self.odom_yaw += yaw_delta
self.odom_x += distance * math.cos(self.odom_yaw)
self.odom_y += distance * math.sin(self.odom_yaw)

# 5. Publish to ROS 2 topic /odom
```

---

## 2. Software (Fake) Odometry

Before hardware encoders were fully integrated, the robot relied on an "open loop" software odometry. This method assumes the robot moves exactly as commanded by the `/cmd_vel` or `/cmd_vel_auto` topics.

It is less accurate because it ignores wheel slip, battery drain, and physical terrain resistance, but it is useful for simulation or when hardware encoders fail.

- **File Location:** Found integrated inside nodes like `auto_driver.py` or tracking nodes.
- **Mechanism:** It listens to the `linear.x` (speed in m/s) being commanded to the robot and multiplies it by the time passed (`dt`).

### Reference Code Snippet

From `odom_callback()` in `auto_driver.py` (which tracks total distance driven based on commanded speeds):

```python
def odom_callback(self, msg):
    """Integrate distance from commanded odometry/velocity for state transitions."""
    now = time.time()

    # Calculate time delta (dt) since last callback
    dt = min(now - self.last_odom_time, 0.1)  # Cap dt to 0.1s to prevent huge jumps if delayed
    self.last_odom_time = now

    # Extract commanded linear velocity (m/s)
    # Note: In a pure fake odometry node, this would come from a Twist message on /cmd_vel
    v = msg.twist.twist.linear.x

    # Distance = Velocity * Time
    d = v * dt
    self.distance += d

    # Example logic: Tracking distance past a traffic light to know when a lap finishes
    if self.lap_1_complete:
        self.distance_past_light += d
```

### Summary of Differences

| Feature            | Hardware Odometry (`servo_controller.py`)        | Software Odometry (`auto_driver.py` logic)     |
| :----------------- | :----------------------------------------------- | :--------------------------------------------- |
| **Source**         | Real wheel encoders via Rosmaster I2C            | Forward prediction from `/cmd_vel`             |
| **Accuracy**       | High (tracks wheel slip if 4x4, true distance)   | Low (assumes perfect motor performance)        |
| **Implementation** | Calculates X, Y, Yaw using Ackermann math        | Simple `Distance = Velocity * dt` integration  |
| **Use Case**       | Rviz mapping, precise parking, dashboard display | Failsafe, simulation, simple distance triggers |
