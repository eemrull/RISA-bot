# Module 4: Launch Files

## Learning Objectives

By the end of this module, you will:
- Understand why launch files exist
- Write a Python launch file that starts multiple nodes
- Pass parameters to nodes via launch files
- Use existing RISA-bot launch files as reference

## Why Launch Files?

Without a launch file, you'd need to open a terminal for **every** node:

```bash
# Terminal 1
ros2 run my_pkg node_a
# Terminal 2
ros2 run my_pkg node_b
# Terminal 3
ros2 run my_pkg node_c
# ... this gets old fast!
```

A launch file starts **everything in one command**:

```bash
ros2 launch my_pkg everything.launch.py
# Done! All nodes running.
```

## Hands-On: Write a Launch File

### 1. Create the launch directory

```bash
mkdir -p ~/risabotcar_ws/src/my_first_pkg/launch
```

### 2. Write the launch file

Create `my_first_pkg/launch/demo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the publisher
        Node(
            package='my_first_pkg',
            executable='number_publisher',
            name='my_publisher',
        ),
        # Start the subscriber
        Node(
            package='my_first_pkg',
            executable='number_subscriber',
            name='my_subscriber',
        ),
    ])
```

### 3. Register the launch directory

In `my_first_pkg/setup.py`, add to the `data_files` list:

```python
import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/my_first_pkg']),
    ('share/my_first_pkg', ['package.xml']),
    # Add this line:
    (os.path.join('share', 'my_first_pkg', 'launch'), glob('launch/*.py')),
],
```

### 4. Build and launch

```bash
cd ~/risabotcar_ws
colcon build --packages-select my_first_pkg
source install/setup.bash

ros2 launch my_first_pkg demo.launch.py
```

Both nodes start together! Press Ctrl+C to stop them all.

## Adding Parameters

Parameters let you tune values **without editing code**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='number_publisher',
            name='my_publisher',
            parameters=[{
                'publish_rate': 5.0,    # Hz
                'amplitude': 2.0,       # Sine wave amplitude
            }],
        ),
    ])
```

To read these in your node:

```python
class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('amplitude', 1.0)

        rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value

        self.timer = self.create_timer(1.0 / rate, self.publish_number)
```

Then change them live:

```bash
ros2 param set /my_publisher amplitude 5.0
ros2 param get /my_publisher publish_rate
```

## RISA-bot Example: `competition.launch.py`

Look at how the real competition launch file works:

```bash
cat ~/risabotcar_ws/src/risabot_automode/launch/competition.launch.py
```

It launches **14 nodes** in one command — camera, LiDAR, all challenge modules, and the controller.

Key patterns you'll see:

```python
# Including another launch file
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        '/path/to/astra_mini.launch.py'
    ])
)

# Delayed start (wait for sensors)
TimerAction(
    period=5.0,
    actions=[Node(package='risabot_automode', executable='auto_driver')]
)
```

## Exercise

1. Create `demo.launch.py` and verify both nodes start together
2. Add a parameter to your publisher and read it in the node
3. Change the parameter live with `ros2 param set`
4. **Challenge:** Add a third node (your `hello_node`) to the launch file
5. **Challenge:** Look at `competition.launch.py` — how many nodes does it launch?

---

**Previous:** [Module 3 — Topics & Messages](03-topics-and-messages.md)
**Next:** [Module 5 — Working with Sensors](05-working-with-sensors.md)
