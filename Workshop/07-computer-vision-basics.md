# Module 7: Computer Vision Basics

## Learning Objectives

By the end of this module, you will:
- Convert ROS Image messages to OpenCV images
- Filter colors using HSV color space
- Detect lines and calculate lane error
- Understand how the RISA-bot follows lanes

## ROS + OpenCV Bridge

ROS images use the `sensor_msgs/Image` format. OpenCV uses NumPy arrays. We use `cv_bridge` to convert between them:

```python
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    # ROS Image → OpenCV (NumPy array)
    cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    # Now you can use any OpenCV function
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
```

## Understanding HSV Color Space

**RGB** (Red, Green, Blue) is hard to filter — the same color looks different under different lighting.

**HSV** (Hue, Saturation, Value) separates **color** from **brightness**, making it much easier to filter:

| Component | What It Controls | Range |
|---|---|---|
| **H** (Hue) | The color itself | 0–179 |
| **S** (Saturation) | How vivid the color is | 0–255 |
| **V** (Value) | How bright it is | 0–255 |

Common color ranges:

| Color | H Low | H High | S Min | V Min |
|---|---|---|---|---|
| Red (low) | 0 | 10 | 100 | 100 |
| Red (high) | 170 | 179 | 100 | 100 |
| Green | 40 | 80 | 50 | 50 |
| Yellow | 20 | 35 | 100 | 100 |
| White | 0 | 179 | 0 | 150+ |

## Hands-On: Color Filter Node

Create `my_first_pkg/my_first_pkg/color_detector.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()

        # Subscribe to camera
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )

        # Publish detected color
        self.color_pub = self.create_publisher(String, '/detected_color', 10)
        self.get_logger().info('Color detector started!')

    def image_callback(self, msg):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges
        colors = {
            'red_low': ((0, 100, 100), (10, 255, 255)),
            'red_high': ((170, 100, 100), (179, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255)),
        }

        # Check each color
        for name, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            pixel_count = cv2.countNonZero(mask)

            if pixel_count > 500:  # Minimum pixels to count as "detected"
                color_name = 'red' if 'red' in name else name
                self.get_logger().info(f'Detected: {color_name} ({pixel_count} pixels)')
                msg_out = String()
                msg_out.data = color_name
                self.color_pub.publish(msg_out)
                return

        # Nothing detected
        msg_out = String()
        msg_out.data = 'none'
        self.color_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## How Lane Following Works

The RISA-bot's `line_follower_camera.py` uses this pipeline:

```text
Camera Image
    ↓
Crop bottom 40%        ← Only look at the road
    ↓
Convert to grayscale
    ↓
Threshold for WHITE    ← white_threshold = 150
    ↓
Column histogram       ← Sum white pixels per column
    ↓
Find peaks             ← Left line and right line positions
    ↓
Calculate midpoint     ← Center between the two lines
    ↓
Error = image_center - midpoint
    ↓
Publish /lane_error    ← Float32 value (-1.0 to 1.0)
```

### Try It

```bash
# Start camera + line follower
ros2 launch astra_camera astra_mini.launch.py
ros2 run risabot_automode line_follower_camera

# Watch the error value
ros2 topic echo /lane_error
```

Place white tape or paper lines in front of the camera — the error value should change as you move them!

## Exercise

1. Write the `color_detector` node and test it with colored objects
2. Hold a red object in front of the camera — does it detect "red"?
3. Try green — does it detect "green"?
4. **Challenge:** Add yellow detection to the color ranges
5. **Challenge:** Print which part of the image the color is in (left, center, right)

---

**Previous:** [Module 6 — Joystick Control](06-joystick-control.md)
**Next:** [Module 8 — Obstacle Detection](08-obstacle-detection.md)
