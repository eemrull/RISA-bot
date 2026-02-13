# Module 2: Nodes & Packages

## Learning Objectives

By the end of this module, you will:
- Create a ROS 2 Python package from scratch
- Write a simple node that runs and prints output
- Understand `setup.py`, `package.xml`, and entry points
- Build and run your own node

## What is a Package?

A **package** is a folder that contains one or more nodes plus the files ROS needs to find and run them.

```text
my_package/
├── my_package/          ← Python code goes here
│   ├── __init__.py
│   └── my_node.py       ← Your node
├── setup.py             ← Tells ROS how to install
├── package.xml          ← Package metadata
└── setup.cfg
```

### RISA-bot Example

The main package is `risabot_automode`:

```text
risabot_automode/
├── risabot_automode/
│   ├── auto_driver.py
│   ├── line_follower_camera.py
│   ├── traffic_light_detector.py
│   └── ... (more nodes)
├── launch/
│   └── competition.launch.py
├── setup.py
└── package.xml
```

## Hands-On: Create Your Own Package

### 1. Create the package

```bash
cd ~/risabotcar_ws/src
ros2 pkg create --build-type ament_python my_first_pkg
```

This creates:

```text
my_first_pkg/
├── my_first_pkg/
│   └── __init__.py
├── setup.py
├── package.xml
├── setup.cfg
├── resource/
│   └── my_first_pkg
└── test/
```

### 2. Write your first node

Create a new file: `my_first_pkg/my_first_pkg/hello_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Hello Node started!')

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Hello #{self.count} from RISA-bot!')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Register the node

Edit `my_first_pkg/setup.py` — find the `entry_points` section and add:

```python
entry_points={
    'console_scripts': [
        'hello_node = my_first_pkg.hello_node:main',
    ],
},
```

> This line says: "when someone runs `ros2 run my_first_pkg hello_node`, call the `main()` function in `hello_node.py`"

### 4. Build and run

```bash
cd ~/risabotcar_ws
colcon build --packages-select my_first_pkg
source install/setup.bash

ros2 run my_first_pkg hello_node
```

You should see:

```
[INFO] [hello_node]: Hello Node started!
[INFO] [hello_node]: Hello #1 from RISA-bot!
[INFO] [hello_node]: Hello #2 from RISA-bot!
...
```

## Understanding the Key Files

### `setup.py` — How to install

```python
entry_points={
    'console_scripts': [
        'node_name = package.module:main',
    #    ↑ command    ↑ file path    ↑ function
    ],
},
```

### `package.xml` — Metadata & dependencies

```xml
<depend>rclpy</depend>          <!-- Python ROS client -->
<depend>std_msgs</depend>       <!-- Standard message types -->
<depend>sensor_msgs</depend>    <!-- Sensor message types -->
```

Add a `<depend>` line for every ROS package your code imports.

## Exercise

1. Create `my_first_pkg` as described above
2. Write the `hello_node.py` and register it
3. Build and run — verify you see the log messages
4. **Challenge:** Modify `timer_callback` to print the current time instead of a count
5. **Challenge:** Change the timer from 1.0 seconds to 0.5 seconds — what happens?

## How RISA-bot Does It

Look at `risabot_automode/setup.py` to see how real nodes are registered:

```bash
cat ~/risabotcar_ws/src/risabot_automode/setup.py
```

You'll see entries like:

```python
'auto_driver = risabot_automode.auto_driver:main',
'line_follower_camera = risabot_automode.line_follower_camera:main',
```

Same pattern as your hello_node!

---

**Previous:** [Module 1 — Intro to ROS 2](01-intro-to-ros2.md)
**Next:** [Module 3 — Topics & Messages](03-topics-and-messages.md)
