#!/usr/bin/env python3
"""
Traffic Light Detector Node
Detects red, yellow, and green traffic light circles using HSV thresholding
and HoughCircles on the Astra Mini color camera feed.
Publishes the detected state on /traffic_light_state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSPresetProfiles


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # --- Parameters (tunable) ---
        self.declare_parameter('red_h_low1', 0)
        self.declare_parameter('red_h_high1', 10)
        self.declare_parameter('red_h_low2', 160)
        self.declare_parameter('red_h_high2', 180)
        self.declare_parameter('yellow_h_low', 20)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('green_h_low', 40)
        self.declare_parameter('green_h_high', 85)
        self.declare_parameter('sat_min', 80)
        self.declare_parameter('val_min', 80)
        self.declare_parameter('min_circle_radius', 8)
        self.declare_parameter('max_circle_radius', 80)
        self.declare_parameter('min_pixel_count', 50)
        self.declare_parameter('show_debug', False)

        # Publisher
        self.state_pub = self.create_publisher(String, '/traffic_light_state', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug/traffic_light', 10)

        # Subscriber
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.current_state = 'unknown'
        self.confidence_count = 0
        self.declare_parameter('required_confidence', 3)  # N consecutive same detections

        self.get_logger().info('Traffic Light Detector started (R/Y/G circle detection)')

    def _get_hsv_ranges(self):
        """Get current HSV parameters."""
        p = self.get_parameter
        sat_min = p('sat_min').value
        val_min = p('val_min').value

        ranges = {
            'red': [
                # Red wraps around H=0, so two ranges
                (np.array([p('red_h_low1').value, sat_min, val_min]),
                 np.array([p('red_h_high1').value, 255, 255])),
                (np.array([p('red_h_low2').value, sat_min, val_min]),
                 np.array([p('red_h_high2').value, 255, 255])),
            ],
            'yellow': [
                (np.array([p('yellow_h_low').value, sat_min, val_min]),
                 np.array([p('yellow_h_high').value, 255, 255])),
            ],
            'green': [
                (np.array([p('green_h_low').value, sat_min, val_min]),
                 np.array([p('green_h_high').value, 255, 255])),
            ],
        }
        return ranges

    def color_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]

            # Focus on the upper-right region where traffic light is likely
            # (right side of course, elevated position)
            roi = bgr[0:int(h * 0.7), int(w * 0.3):]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            ranges = self._get_hsv_ranges()
            min_pixels = self.get_parameter('min_pixel_count').value

            # Score each color by pixel count in mask
            best_color = 'unknown'
            best_score = 0

            for color_name, hsv_ranges in ranges.items():
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                for (lower, upper) in hsv_ranges:
                    mask |= cv2.inRange(hsv, lower, upper)

                # Clean up mask
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                # Find contours and look for circular ones
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < min_pixels:
                        continue

                    # Check circularity
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                    circularity = 4 * np.pi * area / (perimeter * perimeter)

                    # circularity > 0.6 means roughly circular
                    if circularity > 0.6 and area > best_score:
                        best_score = area
                        best_color = color_name

            # Confidence filtering — require N consecutive same detections
            if best_color == self.current_state:
                self.confidence_count += 1
            else:
                self.confidence_count = 1
                self.current_state = best_color

            # Only publish when confident
            required_confidence = self.get_parameter('required_confidence').value
            if self.confidence_count >= required_confidence:
                state_msg = String()
                state_msg.data = self.current_state
                self.state_pub.publish(state_msg)

            # Debug visualization
            if self.get_parameter('show_debug').value:
                debug = roi.copy()
                if best_color != 'unknown':
                    # To draw the circle, we need to find the contours again 
                    # for the best color (or we could store it from the loop above).
                    # For simplicity, we'll re-run just the bounding box on the winning mask
                    best_ranges = ranges[best_color]
                    b_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                    for (lower, upper) in best_ranges:
                        b_mask |= cv2.inRange(hsv, lower, upper)
                    b_mask = cv2.morphologyEx(b_mask, cv2.MORPH_OPEN, kernel)
                    b_mask = cv2.morphologyEx(b_mask, cv2.MORPH_CLOSE, kernel)
                    b_cnts, _ = cv2.findContours(b_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    if b_cnts:
                        largest_cnt = max(b_cnts, key=cv2.contourArea)
                        ((cx, cy), radius) = cv2.minEnclosingCircle(largest_cnt)
                        color_rgb = (0, 0, 255) if best_color == 'red' else ((0, 255, 255) if best_color == 'yellow' else (0, 255, 0))
                        
                        cv2.circle(debug, (int(cx), int(cy)), int(radius), color_rgb, 2)
                        cv2.putText(debug, f"{best_color.upper()} ({self.confidence_count}/{required_confidence})", 
                                    (int(cx) - 20, int(cy) - int(radius) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_rgb, 2)
                
                # Add overall status text
                cv2.putText(debug, f"State: {self.current_state.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                self.debug_pub.publish(debug_msg)

            # Debug output
            if self.current_state != 'unknown':
                emoji = {'red': '🔴', 'yellow': '🟡', 'green': '🟢'}.get(self.current_state, '❓')
                print(f"\r[TLD] {emoji} {self.current_state.upper()} (conf: {self.confidence_count}/{required_confidence}, score: {best_score:.0f})", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Traffic light detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
