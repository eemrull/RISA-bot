#!/usr/bin/env python3
"""
Camera Obstacle Avoidance Node
==============================
Uses the Astra camera feed to detect obstacles by measuring edge density
in the center ROI. Objects (boxes, walls, bins) create significantly more
edges than a flat track surface (which only has smooth ground and lane lines).

Uses Canny edge detection + edge pixel ratio with hysteresis filtering.
Publishes a boolean flag to `/obstacle_detected_camera`.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSPresetProfiles


class ObstacleAvoidanceCamera(Node):
    """
    Analyzes the center ROI of the camera feed for high edge density.
    Objects have sharp edges; a flat track does not.
    Used as an auxiliary detection method alongside LiDAR.
    """
    def __init__(self):
        super().__init__('obstacle_avoidance_camera')

        # Parameters (tunable at runtime)
        self.declare_parameter('edge_threshold', 0.12)   # edge pixel ratio to trigger (0.0–1.0)
        self.declare_parameter('canny_low', 50)          # Canny lower threshold
        self.declare_parameter('canny_high', 150)        # Canny upper threshold
        self.declare_parameter('blur_kernel', 5)         # GaussianBlur kernel size (odd number)
        self.declare_parameter('hysteresis_on', 3)       # consecutive frames to trigger
        self.declare_parameter('hysteresis_off', 5)      # consecutive frames to clear
        self.declare_parameter('show_debug', False)      # publish annotated debug frame

        # Publishers
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected_camera',
            10
        )
        self.debug_pub = self.create_publisher(Image, '/camera/debug/obstacle', 10)

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
        self.detect_count = 0
        self.clear_count = 0

        self.get_logger().info('Obstacle Avoidance Camera Node Started (Edge Density Detection)')

    def color_callback(self, msg):
        try:
            # Read parameters once per frame
            edge_threshold = self.get_parameter('edge_threshold').value
            canny_low = self.get_parameter('canny_low').value
            canny_high = self.get_parameter('canny_high').value
            blur_k = self.get_parameter('blur_kernel').value
            hysteresis_on = self.get_parameter('hysteresis_on').value
            hysteresis_off = self.get_parameter('hysteresis_off').value

            # Convert ROS Image to OpenCV
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Get center region (25% of image)
            h, w = color_image.shape[:2]
            cx, cy = w // 2, h // 2
            roi_size = min(h, w) // 4
            roi = color_image[
                cy - roi_size: cy + roi_size,
                cx - roi_size: cx + roi_size
            ]

            # Convert to grayscale and blur to reduce noise
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)

            # Canny edge detection
            edges = cv2.Canny(blurred, canny_low, canny_high)

            # Calculate edge density (ratio of edge pixels to total pixels)
            total_pixels = edges.shape[0] * edges.shape[1]
            edge_pixels = np.count_nonzero(edges)
            edge_density = edge_pixels / total_pixels if total_pixels > 0 else 0.0

            is_obstacle = edge_density > edge_threshold

            if is_obstacle:
                self.detect_count += 1
                self.clear_count = 0
            else:
                self.clear_count += 1
                self.detect_count = 0

            if self.detect_count >= hysteresis_on and not self.obstacle_active:
                self.get_logger().warn(f'🛑 Obstacle detected! Edge density: {edge_density:.3f}')
                self.obstacle_active = True
            elif self.clear_count >= hysteresis_off and self.obstacle_active:
                self.get_logger().info(f'✅ Path clear. Edge density: {edge_density:.3f}')
                self.obstacle_active = False

            # Publish obstacle status
            obstacle_msg = Bool()
            obstacle_msg.data = self.obstacle_active
            self.obstacle_pub.publish(obstacle_msg)

            # Debug Visualization
            if self.get_parameter('show_debug').value:
                debug = color_image.copy()
                color = (0, 0, 255) if self.obstacle_active else (0, 255, 0)

                # Draw ROI box
                cv2.rectangle(debug,
                              (cx - roi_size, cy - roi_size),
                              (cx + roi_size, cy + roi_size),
                              color, 2)

                # Overlay the edge map inside the ROI for visibility
                edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                # Tint edges with the status color
                edges_tinted = np.zeros_like(edges_bgr)
                edges_tinted[:, :] = color
                edge_mask = edges > 0
                edges_bgr[edge_mask] = edges_tinted[edge_mask]
                # Blend edges into the ROI area
                roi_slice = debug[cy - roi_size: cy + roi_size,
                                  cx - roi_size: cx + roi_size]
                blended = cv2.addWeighted(roi_slice, 0.6, edges_bgr, 0.4, 0)
                debug[cy - roi_size: cy + roi_size,
                      cx - roi_size: cx + roi_size] = blended

                # Add status text
                status_text = "STOP" if self.obstacle_active else "CLEAR"
                pct = edge_density * 100
                cv2.putText(debug, f"{status_text} | Edge: {pct:.1f}%",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                debug_msg_ros = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                self.debug_pub.publish(debug_msg_ros)

            self.get_logger().debug(
                f"Edge density: {edge_density:.3f} | "
                f"{'STOP' if self.obstacle_active else 'CLEAR'}"
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
