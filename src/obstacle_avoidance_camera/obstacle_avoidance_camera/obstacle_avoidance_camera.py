#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSPresetProfiles


class ObstacleAvoidanceCamera(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_camera')

        # Parameters (tunable at runtime)
        self.declare_parameter('white_threshold', 200)  # Bright pixels = white
        self.declare_parameter('hysteresis_on', 3)      # consecutive frames to trigger
        self.declare_parameter('hysteresis_off', 5)     # consecutive frames to clear

        # Publishers
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected_camera',
            10
        )

        # Subscribers
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State — hysteresis filtering
        self.obstacle_active = False
        self.white_count = 0
        self.clear_count = 0

        self.get_logger().info('Obstacle Avoidance Camera Node Started (White Surface Detection)')

    def color_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Get center region (20% of image)
            h, w = color_image.shape[:2]
            cx, cy = w // 2, h // 2
            roi_size = min(h, w) // 5
            roi = color_image[
                cy - roi_size: cy + roi_size,
                cx - roi_size: cx + roi_size
            ]

            # Calculate average BGR values
            avg_bgr = np.mean(roi, axis=(0, 1))
            avg_intensity = np.mean(avg_bgr)  # 0–255

            white_threshold = self.get_parameter('white_threshold').value
            is_white = avg_intensity > white_threshold

            # Hysteresis — require N consecutive frames to change state
            hysteresis_on = self.get_parameter('hysteresis_on').value
            hysteresis_off = self.get_parameter('hysteresis_off').value

            if is_white:
                self.white_count += 1
                self.clear_count = 0
            else:
                self.clear_count += 1
                self.white_count = 0

            if self.white_count >= hysteresis_on and not self.obstacle_active:
                self.get_logger().warn(f'🛑 White surface detected! Avg intensity: {avg_intensity:.1f}')
                self.obstacle_active = True
            elif self.clear_count >= hysteresis_off and self.obstacle_active:
                self.get_logger().info(f'✅ Non-white surface. Safe to move. Avg intensity: {avg_intensity:.1f}')
                self.obstacle_active = False

            # Publish obstacle status
            obstacle_msg = Bool()
            obstacle_msg.data = self.obstacle_active
            self.obstacle_pub.publish(obstacle_msg)

            # Live update
            status = "🛑 STOP" if self.obstacle_active else "✅ GO"
            print(
                f"\r[obstacle_avoidance_camera] Avg intensity: {avg_intensity:.1f} | Status: {status}",
                end='',
                flush=True
            )

        except Exception as e:
            self.get_logger().error(f'Color processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
