# Module 9: Putting It Together

## Learning Objectives

By the end of this module, you will:
- Combine multiple nodes into a working robot system
- Create a launch file that starts everything
- Understand how the RISA-bot's full system works
- Run a simple autonomous demo

## The Big Picture

Over the previous modules, you've built individual pieces:

| Module | What You Built | Topic |
|---|---|---|
| 3 | Publisher + Subscriber | `/my_number` |
| 6 | Joystick driver | `/cmd_vel` |
| 7 | Color detector | `/detected_color` |
| 8 | Obstacle detector | `/my_obstacle` |

Now let's combine them into a **simple autonomous robot** that:
1. Follows a lane (using camera)
2. Stops when obstacle detected (using LiDAR)
3. Can be overridden by joystick (manual mode)

## System Architecture

```text
┌──────────────┐        ┌───────────────┐
│   Camera     │───────▶│ Color/Line    │──── /lane_error
└──────────────┘        │ Detector      │     (Float32)
                        └───────────────┘         │
                                                  ▼
┌──────────────┐        ┌───────────────┐   ┌──────────┐
│   LiDAR      │───────▶│ Obstacle      │──▶│  Brain   │──── /cmd_vel
└──────────────┘        │ Detector      │   │ (decides │     (Twist)
                        └───────────────┘   │  what to │
┌──────────────┐        ┌───────────────┐   │  do)     │
│  Joystick    │───────▶│ Joy Driver    │──▶│          │
└──────────────┘        └───────────────┘   └──────────┘
```

## Hands-On: Simple Brain Node

Create `my_first_pkg/my_first_pkg/simple_brain.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class SimpleBrain(Node):
    def __init__(self):
        super().__init__('simple_brain')

        # State
        self.auto_mode = False
        self.obstacle = False
        self.lane_error = 0.0

        # Parameters
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('steering_gain', 0.5)
        self.speed = self.get_parameter('speed').value
        self.gain = self.get_parameter('steering_gain').value

        # Subscribers
        self.create_subscription(Bool, '/my_obstacle', self.obstacle_cb, 10)
        self.create_subscription(Float32, '/lane_error', self.lane_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: decide and publish at 20 Hz
        self.create_timer(0.05, self.decide)

        self.get_logger().info('Simple brain started! Press Y to toggle auto mode.')

    def obstacle_cb(self, msg):
        self.obstacle = msg.data

    def lane_cb(self, msg):
        self.lane_error = msg.data

    def joy_cb(self, msg):
        # Y button (index 3) toggles auto mode
        if len(msg.buttons) > 3 and msg.buttons[3] == 1:
            self.auto_mode = not self.auto_mode
            mode = 'AUTO' if self.auto_mode else 'MANUAL'
            self.get_logger().info(f'Mode: {mode}')

        # In manual mode, joystick controls directly
        if not self.auto_mode:
            twist = Twist()
            twist.linear.x = msg.axes[1] * self.speed  # Left stick Y
            twist.angular.z = msg.axes[0] * 0.5         # Left stick X
            self.cmd_pub.publish(twist)

    def decide(self):
        if not self.auto_mode:
            return  # Manual mode — joy_cb handles it

        twist = Twist()

        if self.obstacle:
            # STOP — obstacle ahead
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('Obstacle! Stopping.', throttle_duration_sec=1.0)
        else:
            # Follow lane
            twist.linear.x = self.speed
            twist.angular.z = -self.gain * self.lane_error
            # Negative because: positive error means line is to the right,
            # so we turn right (negative angular.z)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Create a Launch File

`my_first_pkg/launch/autonomous.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joystick
        Node(package='joy', executable='joy_node', name='joy_node'),

        # Your obstacle detector
        Node(
            package='my_first_pkg',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[{'min_distance': 0.40}],
        ),

        # Your brain
        Node(
            package='my_first_pkg',
            executable='simple_brain',
            name='simple_brain',
            parameters=[{
                'speed': 0.15,
                'steering_gain': 0.5,
            }],
        ),
    ])
```

> **Note:** You also need the camera, LiDAR, and line follower running. Either add them to this launch file or start them separately.

## Run It

```bash
colcon build --packages-select my_first_pkg
source install/setup.bash

# Terminal 1: Sensors
ros2 launch astra_camera astra_mini.launch.py

# Terminal 2: LiDAR
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node

# Terminal 3: Line follower
ros2 run risabot_automode line_follower_camera

# Terminal 4: Your autonomous system!
ros2 launch my_first_pkg autonomous.launch.py
```

Press **Y** on the controller to toggle between manual and auto mode.

## Compare With RISA-bot

Your `simple_brain` is a simplified version of `auto_driver.py`. The real one adds:
- State machine for 9 different challenges
- Multiple velocity sources (tunnel, parking, obstruction)
- Distance-based state transitions
- PD control with smoothing

But the core pattern is the same: **subscribe to sensors → decide → publish /cmd_vel**.

## Exercise

1. Build and run the full system
2. Test manual mode with the joystick
3. Toggle to auto mode with Y button
4. Place an obstacle — does the robot stop?
5. **Challenge:** Add a state that makes the robot turn around instead of stopping
6. **Challenge:** Look at `auto_driver.py` — can you identify the same subscribe-decide-publish pattern?

## What's Next?

You've completed the workshop! Here's where to go from here:

| Want to... | Read |
|---|---|
| Understand the competition system | [Guide/Test/](../Guide/Test/) |
| Tune parameters on the course | [Guide/tuning_guide.md](../Guide/tuning_guide.md) |
| See all available commands | [Guide/commands_reference.md](../Guide/commands_reference.md) |
| Dive into challenge code | [Guide/challenges_breakdown.md](../Guide/challenges_breakdown.md) |

---

**Previous:** [Module 8 — Obstacle Detection](08-obstacle-detection.md)

🎉 **Congratulations!** You've completed all 9 modules!
