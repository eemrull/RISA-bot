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

import time
from typing import Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .topics import CAMERA_DEBUG_LINE_TOPIC, CAMERA_IMAGE_TOPIC, LANE_ERROR_TOPIC


class LineFollowerCamera(Node):
    """Lane detection and steering error estimation from camera frames."""

    def __init__(self):
        super().__init__('line_follower_camera')
        self.lane_error = 0.0
        self.filtered_error = 0.0     # smoothed output
        self.last_valid_error = 0.0   # hold last command

        # --- Tunable parameters ---
        self.declare_parameter('smoothing_alpha', 0.3)   # EMA factor (0=very smooth, 1=no filter)
        self.declare_parameter('dead_zone', 0.08)        # ignore errors below this (straight line stability)
        self.declare_parameter('white_threshold', 150)    # grayscale threshold for white lines
        self.declare_parameter('crop_ratio', 0.4)         # bottom portion of image to use
        self.declare_parameter('show_debug', False)       # set True to show debug window
        self.declare_parameter('resize_width', 320)       # downscale width for faster processing
        self.declare_parameter('print_debug', False)      # print debug line to stdout
        self.declare_parameter('debug_print_rate', 0.5)   # seconds between prints
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)
        self._last_debug_print = 0.0

        self.error_pub = self.create_publisher(Float32, LANE_ERROR_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, CAMERA_DEBUG_LINE_TOPIC, 10)
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_TOPIC,
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info('Line Follower Camera: Ready (Smoothed Two-Line Midpoint Mode)')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-frame lookups."""
        self._param_cache = {
            'smoothing_alpha': float(self.get_parameter('smoothing_alpha').value),
            'dead_zone': float(self.get_parameter('dead_zone').value),
            'white_threshold': int(self.get_parameter('white_threshold').value),
            'crop_ratio': float(self.get_parameter('crop_ratio').value),
            'show_debug': bool(self.get_parameter('show_debug').value),
            'resize_width': int(self.get_parameter('resize_width').value),
            'print_debug': bool(self.get_parameter('print_debug').value),
            'debug_print_rate': float(self.get_parameter('debug_print_rate').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def color_callback(self, msg: Image) -> None:
        """Process a camera frame and publish lane error."""
        try:
            # Convert to BGR, then grayscale
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]
            resize_w = self._param_cache['resize_width']
            if resize_w > 0 and w > resize_w:
                scale = resize_w / float(w)
                bgr = cv2.resize(bgr, (resize_w, int(h * scale)))
                h, w = bgr.shape[:2]

            # Focus on bottom portion (more road visibility for high mount)
            crop_ratio = self._param_cache['crop_ratio']
            crop_h = int(h * crop_ratio)
            road = bgr[h - crop_h:, :]

            # Threshold white lines
            white_thresh = self._param_cache['white_threshold']
            gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # reduce noise before threshold
            _, binary = cv2.threshold(blurred, white_thresh, 255, cv2.THRESH_BINARY)

            # Sliding Windows to trace curved lanes
            n_windows = 10
            window_height = crop_h // n_windows
            
            left_points = []
            right_points = []
            center_points = []
            
            # Start from the bottom window (closest to robot) and move up
            valid_frames = 0
            
            # Base lane width from the bottom-most valid frame
            estimated_lane_width = w // 2 # reasonable default
            
            for window in range(n_windows):
                win_y_low = crop_h - (window + 1) * window_height
                win_y_high = crop_h - window * window_height
                
                # Protect against slicing issues if height isn't perfectly divisible
                if win_y_high <= win_y_low:
                    break
                    
                hist = np.sum(binary[win_y_low:win_y_high, :], axis=0)
                
                # Default regions
                left_start, left_end = 0, w//3
                right_start, right_end = 2*w//3, w
                
                # Apply bounding box constraints if we have a previous point
                if window > 0 and len(left_points) > 0:
                    prev_l = left_points[-1][0]
                    left_start = max(0, prev_l - 60)
                    left_end = min(w//2, prev_l + 60)
                if window > 0 and len(right_points) > 0:
                    prev_r = right_points[-1][0]
                    right_start = max(w//2, prev_r - 60)
                    right_end = min(w, prev_r + 60)
                
                left_region = hist[left_start:left_end]
                right_region = hist[right_start:right_end]
                
                # Check validity
                valid_left = np.max(left_region) >= 10 if len(left_region) > 0 else False
                valid_right = np.max(right_region) >= 10 if len(right_region) > 0 else False
                
                # Minimum lane width in pixels — if detected width is less than
                # this, both peaks are likely on the SAME white border line
                min_lane_width = 80
                
                if valid_left and valid_right:
                    left_peak = np.argmax(left_region) + left_start
                    right_peak = np.argmax(right_region) + right_start
                    detected_width = right_peak - left_peak
                    if detected_width >= min_lane_width:
                        valid_frames += 1
                        # Only update lane width when it's reasonable
                        if detected_width < w * 2 // 3:
                            estimated_lane_width = detected_width
                    else:
                        # Both peaks are on the same border line! Use memory.
                        left_peak = left_points[-1][0] if left_points else w // 2 - estimated_lane_width // 2
                        right_peak = left_peak + estimated_lane_width
                elif valid_left and not valid_right:
                    left_peak = np.argmax(left_region) + left_start
                    right_peak = left_peak + estimated_lane_width 
                elif valid_right and not valid_left:
                    right_peak = np.argmax(right_region) + right_start
                    left_peak = right_peak - estimated_lane_width
                else:
                    left_peak = left_points[-1][0] if left_points else int(self.last_valid_error * w/2 + w/2) - (estimated_lane_width // 2)
                    right_peak = right_points[-1][0] if right_points else int(self.last_valid_error * w/2 + w/2) + (estimated_lane_width // 2)
                
                lane_center_x = (left_peak + right_peak) // 2
                lane_center_y = int(h - crop_h + win_y_low + window_height / 2)
                
                left_points.append((int(left_peak), lane_center_y))
                right_points.append((int(right_peak), lane_center_y))
                center_points.append((int(lane_center_x), lane_center_y))

            # Error calculation: FLAT weights across all windows so the robot
            # follows where the road GOES (upper windows = road ahead on curves)
            # instead of just centering on the nearest lane position.
            n_pts = len(center_points)
            if n_pts > 0:
                weighted_center_x = sum(pt[0] for pt in center_points) / n_pts
            else:
                weighted_center_x = w / 2.0
            
            # Error: positive = lane center left of image center → robot turns LEFT
            #         negative = lane center right → robot turns RIGHT
            image_center = w / 2.0
            error = (image_center - weighted_center_x) / (w / 2.0)

            # Curvature bias is no longer needed — flat weights naturally
            # follow the curve direction because upper windows shift the
            # average toward where the road goes.
            curvature_bias = 0.0

            # Clamp
            raw_error = float(np.clip(error, -1.0, 1.0))

            # Dead zone — ignore tiny errors to prevent jitter on straight roads
            dead_zone = self._param_cache['dead_zone']
            if abs(raw_error) < dead_zone:
                raw_error = 0.0

            # Low-pass filter (exponential moving average) to smooth output
            alpha = self._param_cache['smoothing_alpha']
            self.filtered_error = alpha * raw_error + (1 - alpha) * self.filtered_error
            self.lane_error = self.filtered_error

            # Only update last_valid_error if we have real lines in the bottom frame
            if valid_frames > 0:
                self.last_valid_error = self.lane_error

            # Publish the smoothed error
            self.error_pub.publish(Float32(data=self.lane_error))

            # Debug visualization
            if self._param_cache['show_debug']:
                debug = bgr.copy()
                
                crop_top = h - crop_h  # top of cropped region in full image coords
                
                def draw_polyfit_curve(pts_list, color, thickness=2):
                    if len(pts_list) < 3: return None
                    pts = np.array(pts_list)
                    x, y = pts[:, 0], pts[:, 1]
                    poly_fit = np.polyfit(y, x, 2)
                    # Extend curve from crop top to image bottom
                    y_eval = np.linspace(crop_top, h - 1, 80)
                    x_eval = np.polyval(poly_fit, y_eval)
                    x_eval = np.clip(x_eval, 0, w - 1)
                    curve_pts = np.vstack((x_eval, y_eval)).astype(np.int32).T
                    cv2.polylines(debug, [curve_pts], isClosed=False, color=(0,0,0), thickness=thickness+2)
                    cv2.polylines(debug, [curve_pts], isClosed=False, color=color, thickness=thickness)
                    return poly_fit
                
                # Draw thin left/right lane boundary curves
                draw_polyfit_curve(left_points, (255, 130, 130), 2)   # Blue left
                draw_polyfit_curve(right_points, (130, 130, 255), 2)  # Red right
                
                # Draw prominent center PATH line (white + green, thicker)
                center_poly = draw_polyfit_curve(center_points, (0, 255, 0), 3)
                
                # Small dot markers at window sampling heights only
                for i, (l_pt, r_pt, c_pt) in enumerate(zip(left_points, right_points, center_points)):
                    if i % 2 == 0:  # every other window for less clutter
                        cv2.circle(debug, l_pt, 3, (255, 130, 130), -1)
                        cv2.circle(debug, r_pt, 3, (130, 130, 255), -1)
                    cv2.circle(debug, c_pt, 4, (0, 0, 0), -1)
                    cv2.circle(debug, c_pt, 3, (0, 255, 0), -1)
                
                # Helper: text with black outline for readability
                def put_outlined_text(img, text, pos, scale, color, thick=2):
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick+2)
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)
                
                # --- Educational info for DR ---
                # Line 1: Steering direction
                if abs(self.lane_error) < 0.05:
                    direction = "CENTERED"
                    text_color = (0, 255, 0)
                elif self.lane_error > 0:
                    direction = "STEER LEFT"
                    text_color = (0, 165, 255)
                else:
                    direction = "STEER RIGHT"
                    text_color = (0, 165, 255)
                    
                steer_angle_deg = abs(self.lane_error) * 50.0  # approx servo degrees
                put_outlined_text(debug, f"{direction} ({steer_angle_deg:.0f} deg)",
                            (10, 30), 0.7, text_color)
                
                # Line 2: Lane width & curve radius estimate
                lane_w_cm = estimated_lane_width * 40.0 / (w * 0.4)  # rough cm scale
                if center_poly is not None and abs(center_poly[0]) > 0.001:
                    # Radius of curvature from polynomial: R = 1/|2a| (in pixels)
                    R_px = abs(1.0 / (2.0 * center_poly[0]))
                    R_cm = R_px * 40.0 / estimated_lane_width  # scale to real world
                    curve_dir = "LEFT" if center_poly[0] < 0 else "RIGHT"
                    put_outlined_text(debug, f"Curve: {curve_dir} R={R_cm:.0f}cm W={lane_w_cm:.0f}cm",
                                (10, 60), 0.55, (0, 255, 255))
                else:
                    put_outlined_text(debug, f"Straight | W={lane_w_cm:.0f}cm",
                                (10, 60), 0.55, (0, 255, 255))
                
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                self.debug_pub.publish(debug_msg)

            if self._param_cache['print_debug']:
                now = time.monotonic()
                if now - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    status = "CENTER" if abs(self.lane_error) < 0.05 else ("TURN RIGHT" if self.lane_error < 0 else "TURN LEFT")
                    print(f"\r[LF] C:{int(weighted_center_x)} | Err:{self.lane_error:.2f} | {status}", end='', flush=True)
                    self._last_debug_print = now

        except Exception as e:
            self.get_logger().error(f'Line follower error: {e}')


def main(args=None) -> None:
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
