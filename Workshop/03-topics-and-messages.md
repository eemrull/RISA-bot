# Module 3: Topics & Messages

## Learning Objectives

By the end of this module, you will:
- Understand what topics and messages are
- Write a **publisher** node that sends data
- Write a **subscriber** node that receives data
- Use CLI tools to inspect and debug topics

## How Communication Works

```text
Publisher Node                              Subscriber Node
┌──────────────┐    /my_topic    ┌──────────────┐
│  Publishes   │ ─── Float32 ──▶│  Receives    │
│  data every  │                 │  data and    │
│  100ms       │                 │  acts on it  │
└──────────────┘                 └──────────────┘
```

- **Publisher**: sends data on a topic
- **Subscriber**: listens for data on a topic
- **Topic**: the named channel (like a radio frequency)
- **Message type**: the format of the data

### Message Types

| Type | Package | What It Holds | Used For |
|---|---|---|---|
| `Bool` | std_msgs | True / False | Obstacle detected? |
| `Float32` | std_msgs | A decimal number | Lane error |
| `String` | std_msgs | Text | Traffic light state |
| `Twist` | geometry_msgs | Linear + angular velocity | Motor commands |
| `LaserScan` | sensor_msgs | Array of distances | LiDAR data |
| `Joy` | sensor_msgs | Buttons + axes | Joystick input |

## Hands-On: Write a Publisher

Create `my_first_pkg/my_first_pkg/number_publisher.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher = self.create_publisher(Float32, '/my_number', 10)
        self.timer = self.create_timer(0.1, self.publish_number)  # 10 Hz
        self.count = 0.0
        self.get_logger().info('Number publisher started!')

    def publish_number(self):
        msg = Float32()
        msg.data = math.sin(self.count)  # Sine wave
        self.publisher.publish(msg)
        self.count += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hands-On: Write a Subscriber

Create `my_first_pkg/my_first_pkg/number_subscriber.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/my_number',
            self.number_callback,
            10
        )
        self.get_logger().info('Number subscriber started!')

    def number_callback(self, msg):
        value = msg.data
        if value > 0:
            self.get_logger().info(f'Positive: {value:.3f}')
        else:
            self.get_logger().info(f'Negative: {value:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Register Both Nodes

Add to `setup.py` entry_points:

```python
'console_scripts': [
    'hello_node = my_first_pkg.hello_node:main',
    'number_publisher = my_first_pkg.number_publisher:main',
    'number_subscriber = my_first_pkg.number_subscriber:main',
],
```

## Build and Test

```bash
cd ~/risabotcar_ws
colcon build --packages-select my_first_pkg
source install/setup.bash

# Terminal 1
ros2 run my_first_pkg number_publisher

# Terminal 2
ros2 run my_first_pkg number_subscriber

# Terminal 3 — watch the raw topic
ros2 topic echo /my_number
```

## CLI Topic Tools

These are essential for debugging:

```bash
# List all active topics
ros2 topic list

# See full info about a topic
ros2 topic info /my_number

# Watch live messages
ros2 topic echo /my_number

# Check publish rate
ros2 topic hz /my_number

# Publish a single message from CLI
ros2 topic pub --once /my_number std_msgs/Float32 "data: 0.42"

# Publish continuously at 1 Hz
ros2 topic pub /my_number std_msgs/Float32 "data: 0.42" --rate 1
```

## RISA-bot Example

The line follower and auto driver use exactly this pattern:

```text
line_follower_camera → publishes Float32 on /lane_error
auto_driver          → subscribes to /lane_error, steers based on value
```

Try it on the real robot:

```bash
# Start the camera and line follower
ros2 launch astra_camera astra_mini.launch.py
ros2 run risabot_automode line_follower_camera

# In another terminal, watch the lane error
ros2 topic echo /lane_error
```

Move a white line in front of the camera — you'll see the error value change!

## Exercise

1. Run your publisher and subscriber — verify they communicate
2. Use `ros2 topic echo` to see the sine wave values
3. **Challenge:** Modify the subscriber to only print when `abs(value) > 0.5`
4. **Challenge:** Create a second subscriber that counts how many messages it receives
5. **Challenge:** Use `ros2 topic pub` to send a fake value — does your subscriber react?

---

**Previous:** [Module 2 — Nodes & Packages](02-nodes-and-packages.md)
**Next:** [Module 4 — Launch Files](04-launch-files.md)
