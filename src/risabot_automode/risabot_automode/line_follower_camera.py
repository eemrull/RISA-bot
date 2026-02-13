#!/usr/bin/env python3
"""
Line Follower Camera Node
Detects white lane lines using grayscale thresholding and computes a steering
error from the midpoint between the left and right lane peaks.

Algorithm:
  1. Crop bottom portion of camera image (road surface)
  2. Threshold for white pixels (lane markings)
  3. Build column histogram to find left/right lane peaks
  4. Error = offset of lane midpoint from image center
  5. Smoothed via dead zone + exponential moving average (EMA)

Publishes Float32 on /lane_error (range -1.0 to +1.0).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSPresetProfiles


class LineFollowerCamera(Node):

    def __init__(self):
        super().__init__('line_follower_camera')
        self.lane_error = 0.0
        self.filtered_error = 0.0     # smoothed output
        self.last_valid_error = 0.0   # hold last command

        # --- Tunable parameters ---
        self.declare_parameter('smoothing_alpha', 0.3)   # EMA factor (0=very smooth, 1=no filter)
        self.declare_parameter('dead_zone', 0.03)        # ignore errors below this
        self.declare_parameter('white_threshold', 150)    # grayscale threshold for white lines
        self.declare_parameter('crop_ratio', 0.4)         # bottom portion of image to use
        self.declare_parameter('show_debug', False)       # set True to show debug window

        self.error_pub = self.create_publisher(Float32, '/lane_error', 10)
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info('Line Follower Camera: Ready (Smoothed Two-Line Midpoint Mode)')

    def color_callback(self, msg):
        try:
            # Convert to BGR, then grayscale
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]

            # Focus on bottom portion (more road visibility for high mount)
            crop_ratio = self.get_parameter('crop_ratio').value
            crop_h = int(h * crop_ratio)
            road = bgr[h - crop_h:, :]

            # Threshold white lines
            white_thresh = self.get_parameter('white_threshold').value
            gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # reduce noise before threshold
            _, binary = cv2.threshold(blurred, white_thresh, 255, cv2.THRESH_BINARY)

            # Sum white pixels per column → histogram
            hist = np.sum(binary, axis=0)

            # Find left and right peaks (simple: left 1/3, right 1/3 of image)
            left_region = hist[:w//3]
            right_region = hist[2*w//3:]

            left_peak = np.argmax(left_region)  # x in left 1/3
            right_peak = np.argmax(right_region) + 2*w//3  # x in right 1/3

            # If either region is all zeros, use last valid value
            if np.max(left_region) < 10:
                left_peak = self.last_valid_error * w/2 + w/2  # rough estimate
            if np.max(right_region) < 10:
                right_peak = self.last_valid_error * w/2 + w/2

            # Midline = average of two line positions
            lane_center_x = (left_peak + right_peak) // 2

            # Error: positive = lane center right of image center → robot should turn LEFT
            image_center = w / 2.0
            error = (image_center - lane_center_x) / (w / 2.0)  # NO negative sign!

            # Clamp raw error
            raw_error = float(np.clip(error, -1.0, 1.0))

            # Dead zone — ignore tiny errors to prevent jitter on straight roads
            dead_zone = self.get_parameter('dead_zone').value
            if abs(raw_error) < dead_zone:
                raw_error = 0.0

            # Low-pass filter (exponential moving average) to smooth output
            alpha = self.get_parameter('smoothing_alpha').value
            self.filtered_error = alpha * raw_error + (1 - alpha) * self.filtered_error
            self.lane_error = self.filtered_error

            # Only update last_valid_error if we have real lines
            if np.max(left_region) > 10 and np.max(right_region) > 10:
                self.last_valid_error = self.lane_error

            # Publish the smoothed error
            self.error_pub.publish(Float32(data=self.lane_error))

            # Debug visualization (disabled by default for headless operation)
            if self.get_parameter('show_debug').value:
                debug = bgr.copy()
                cv2.line(debug, (int(left_peak), h-crop_h), (int(left_peak), h), (255, 0, 0), 2)   # blue: left line
                cv2.line(debug, (int(right_peak), h-crop_h), (int(right_peak), h), (0, 0, 255), 2) # red: right line
                cv2.line(debug, (int(lane_center_x), h-crop_h), (int(lane_center_x), h), (0, 255, 0), 2) # green: center
                cv2.imshow('Lane', debug)
                cv2.waitKey(1)

            status = "CENTER" if abs(self.lane_error) < 0.05 else ("TURN RIGHT" if self.lane_error < 0 else "TURN LEFT")
            print(f"\r[LF] C:{lane_center_x} | Err:{self.lane_error:.2f} | {status}", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Line follower error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
