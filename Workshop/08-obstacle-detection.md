# Module 8: Obstacle Detection

## Learning Objectives

By the end of this module, you will:
- Process LiDAR scan data to detect obstacles
- Write a simple obstacle detection node
- Understand how detection distance and angle affect behavior
- See how the RISA-bot uses LiDAR for obstacle avoidance

## LiDAR Data Recap

The `/scan` topic gives us an array of distances:

```text
ranges: [0.45, 0.46, ..., inf, ..., 0.82]
         ↑                              ↑
    angle_min (-π)               angle_max (+π)
```

Each element is the distance to the nearest object at that angle. We need to check the **front** of the robot for obstacles.

## Hands-On: Simple Obstacle Detector

Create `my_first_pkg/my_first_pkg/obstacle_detector.py`:

```python
#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('min_distance', 0.40)   # meters
        self.declare_parameter('scan_angle', 30.0)      # degrees to check

        self.min_distance = self.get_parameter('min_distance').value
        self.scan_angle = self.get_parameter('scan_angle').value

        # Subscribe to LiDAR
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publish obstacle detection
        self.obstacle_pub = self.create_publisher(Bool, '/my_obstacle', 10)

        self.get_logger().info(
            f'Obstacle detector started! '
            f'Distance: {self.min_distance}m, Angle: ±{self.scan_angle}°'
        )

    def scan_callback(self, msg):
        # Calculate which indices correspond to the front arc
        angle_rad = math.radians(self.scan_angle)
        num_readings = len(msg.ranges)
        center = num_readings // 2

        # Indices for ±scan_angle degrees around center
        spread = int(angle_rad / msg.angle_increment)
        start = max(0, center - spread)
        end = min(num_readings, center + spread)

        # Check for obstacles in the front arc
        obstacle_found = False
        closest = float('inf')

        for i in range(start, end):
            dist = msg.ranges[i]
            if msg.range_min < dist < self.min_distance:
                obstacle_found = True
                closest = min(closest, dist)

        # Publish result
        result = Bool()
        result.data = obstacle_found
        self.obstacle_pub.publish(result)

        if obstacle_found:
            self.get_logger().warn(
                f'OBSTACLE at {closest:.2f}m!',
                throttle_duration_sec=0.5
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Register, build, run

```python
# In setup.py
'obstacle_detector = my_first_pkg.obstacle_detector:main',
```

```bash
colcon build --packages-select my_first_pkg
source install/setup.bash

# Terminal 1: LiDAR
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node

# Terminal 2: Your detector
ros2 run my_first_pkg obstacle_detector

# Terminal 3: Watch
ros2 topic echo /my_obstacle
```

Walk in front of the LiDAR — you should see `data: true` when you're closer than 0.4m!

## Making the Robot Stop

Now combine obstacle detection with motor control:

```python
# In your joy_driver or auto node:
def obstacle_callback(self, msg):
    self.obstacle_detected = msg.data

def publish_velocity(self):
    twist = Twist()
    if self.obstacle_detected:
        # STOP!
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
        # Normal driving
        twist.linear.x = self.desired_speed
        twist.angular.z = self.desired_turn
    self.cmd_pub.publish(twist)
```

## Tuning Parameters

```bash
# Change detection distance live
ros2 param set /obstacle_detector min_distance 0.30

# Change scan angle
ros2 param set /obstacle_detector scan_angle 45.0
```

| Parameter | Effect of Increasing | Effect of Decreasing |
|---|---|---|
| `min_distance` | Detects earlier (safer) | Detects later (closer pass) |
| `scan_angle` | Wider detection (safer) | Narrower (only straight ahead) |

## How RISA-bot Does It

The real `obstacle_avoidance` package checks a ±30° front arc and publishes `Bool` on `/obstacle_front`. The `auto_driver` subscribes and stops when `True`.

See: `src/obstacle_avoidance/obstacle_avoidance/obstacle_avoidance.py`

## Exercise

1. Write and run the obstacle detector
2. Walk in front of the LiDAR at different distances
3. Change `min_distance` to 0.20 and 0.60 — what's the difference?
4. **Challenge:** Print which side the obstacle is on (left, center, right)
5. **Challenge:** Instead of just `True/False`, publish the distance to the closest obstacle

---

**Previous:** [Module 7 — Computer Vision Basics](07-computer-vision-basics.md)
**Next:** [Module 9 — Putting It Together](09-putting-it-together.md)
