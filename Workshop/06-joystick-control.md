# Module 6: Joystick Control

## Learning Objectives

By the end of this module, you will:

- Read joystick data from the `/joy` topic
- Map joystick axes to robot movement
- Understand the `Twist` message format for motor control
- Drive the RISA-bot with a joystick

## The Joy Message

When you connect a game controller and run the `joy_node`, it publishes:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

```text
axes: [0.0, 0.5, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0]
#      ↑    ↑    ↑    ↑     ↑
#     LS-X LS-Y RS-X RS-Y  LT   RT  D-X  D-Y
buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
#         A  B  X  Y  LB RB BK ST  LS RS GUIDE
```

> Axis values range from -1.0 to 1.0. Button values are 0 (released) or 1 (pressed).

### Find Your Controller's Mapping

```bash
# Install jstest if not present
sudo apt install joystick

# Test your controller
jstest /dev/input/js0
# Move sticks and press buttons to see which axis/button is which
```

## The Twist Message

To move the robot, we publish `geometry_msgs/Twist`:

```text
linear:
  x: 0.2    ← forward/backward speed (m/s)
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3   ← turning speed (rad/s), positive = left
```

Only `linear.x` (forward) and `angular.z` (turning) matter for our robot.

## Hands-On: Simple Joystick Driver

Create `my_first_pkg/my_first_pkg/joy_driver.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyDriver(Node):
    def __init__(self):
        super().__init__('joy_driver')

        # Parameters
        self.declare_parameter('max_speed', 0.2)       # m/s
        self.declare_parameter('max_turn', 0.5)         # rad/s
        self.declare_parameter('speed_axis', 1)         # Left stick Y
        self.declare_parameter('turn_axis', 0)          # Left stick X

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.speed_axis = self.get_parameter('speed_axis').value
        self.turn_axis = self.get_parameter('turn_axis').value

        # Subscriber: joystick
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher: motor commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Joy driver started!')
        self.get_logger().info(f'Max speed: {self.max_speed}, Max turn: {self.max_turn}')

    def joy_callback(self, msg):
        twist = Twist()

        # Map joystick to velocity
        twist.linear.x = msg.axes[self.speed_axis] * self.max_speed
        twist.angular.z = msg.axes[self.turn_axis] * self.max_turn

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'Speed: {twist.linear.x:.2f}, Turn: {twist.angular.z:.2f}',
            throttle_duration_sec=0.5  # Don't spam logs
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Register and build

Add to `setup.py` entry_points:

```python
'joy_driver = my_first_pkg.joy_driver:main',
```

```bash
cd ~/risabotcar_ws
cbp my_first_pkg
sos
```

### Run it

```bash
# Terminal 1: Joystick driver
ros2 run joy joy_node

# Terminal 2: Your driver
ros2 run my_first_pkg joy_driver

# Terminal 3: Watch the output
ros2 topic echo /cmd_vel
```

Move the left stick — you should see `/cmd_vel` values change!

> ⚠️ **Safety first:** Start with `max_speed: 0.1` until you're comfortable. The robot can be fast!

## How RISA-bot Does It

The real servo controller (`control_servo`) does the same thing but adds:

- Gear shifting (D-pad changes speed limits)
- Auto/manual toggle (Start button)
- Challenge state cycling (LB/RB)

See: `src/control_servo/servo_controller/servo_controller.py`

## Exercise

1. Run `jstest` to find your controller's axis mapping
2. Write and run the `joy_driver` node
3. Verify `/cmd_vel` changes when you move the stick
4. **Challenge:** Add a "dead zone" — ignore stick values smaller than ±0.1
5. **Challenge:** Add a "turbo button" — hold a button to double the max speed

---

**Previous:** [Module 5 — Working with Sensors](05-working-with-sensors.md)
**Next:** [Module 7 — Computer Vision Basics](07-computer-vision-basics.md)
