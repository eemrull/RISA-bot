#!/usr/bin/env python3
"""
Line Follower Camera Node
Detects white lane lines using grayscale thresholding and computes a steering
error from the midpoint between the left and right lane peaks.

Algorithm:
  1. Crop bottom portion of camera image (road surface)
  2. Threshold for white pixels (lane markings)
  3. Sliding windows to trace left/right lane boundaries
  4. Error = offset of lane midpoint from image center
  5. Smoothed via dead zone + exponential moving average (EMA)

Advanced Features:
  - Dynamic look-ahead: automatically raises crop window when lane confidence
    drops, so the robot can "see" further into sharp curves
  - Steering persistence: holds the last known steering intent and decays it
    slowly when both lines are fully lost, preventing snap-to-centre behaviour

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
        self.filtered_error = 0.0    # EMA-smoothed output
        self.last_valid_error = 0.0  # last error from a confident lane lock

        # ── Tunable parameters ─────────────────────────────────────────────
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('dead_zone', 0.08)
        self.declare_parameter('white_threshold', 150)
        # Dynamic look-ahead
        self.declare_parameter('crop_ratio_base', 0.4)   # normal operation
        self.declare_parameter('crop_ratio_max', 0.65)   # expanded on lane loss
        # Steering persistence on total lane loss
        self.declare_parameter('hold_error_frames', 15)  # frames to hold intent
        self.declare_parameter('error_decay_rate', 0.92) # per-frame decay factor
        self.declare_parameter('min_valid_windows', 3)   # windows needed for confident lock
        # Display / debug
        self.declare_parameter('show_debug', False)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('print_debug', False)
        self.declare_parameter('debug_print_rate', 0.5)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)
        self._last_debug_print = 0.0

        # ── Internal state ──────────────────────────────────────────────────
        self.current_hold_frames = 0
        self.dynamic_crop_ratio = self._param_cache['crop_ratio_base']
        self.frames_since_valid = 0  # consecutive frames without a confident lane lock

        # ── ROS publishers / subscribers ────────────────────────────────────
        self.error_pub = self.create_publisher(Float32, LANE_ERROR_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, CAMERA_DEBUG_LINE_TOPIC, 10)
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_TOPIC,
            self.color_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info(
            'Line Follower Camera: Ready (Dynamic Look-Ahead + Steering Persistence)'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Parameter helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-frame lookups."""
        self._param_cache = {
            'smoothing_alpha':   float(self.get_parameter('smoothing_alpha').value),
            'dead_zone':         float(self.get_parameter('dead_zone').value),
            'white_threshold':   int(self.get_parameter('white_threshold').value),
            'crop_ratio_base':   float(self.get_parameter('crop_ratio_base').value),
            'crop_ratio_max':    float(self.get_parameter('crop_ratio_max').value),
            'hold_error_frames': int(self.get_parameter('hold_error_frames').value),
            'error_decay_rate':  float(self.get_parameter('error_decay_rate').value),
            'min_valid_windows': int(self.get_parameter('min_valid_windows').value),
            'show_debug':        bool(self.get_parameter('show_debug').value),
            'resize_width':      int(self.get_parameter('resize_width').value),
            'print_debug':       bool(self.get_parameter('print_debug').value),
            'debug_print_rate':  float(self.get_parameter('debug_print_rate').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────────
    # Main camera callback
    # ──────────────────────────────────────────────────────────────────────────

    def color_callback(self, msg: Image) -> None:
        """Process a camera frame and publish lane error."""
        try:
            # ── 1. Resize ───────────────────────────────────────────────────
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]
            resize_w = self._param_cache['resize_width']
            if resize_w > 0 and w > resize_w:
                scale = resize_w / float(w)
                bgr = cv2.resize(bgr, (resize_w, int(h * scale)))
                h, w = bgr.shape[:2]

            image_center = w / 2.0

            # ── 2. Dynamic look-ahead crop ──────────────────────────────────
            # When lane confidence drops, dynamic_crop_ratio grows so we
            # scan a taller region that reaches further ahead into the curve.
            crop_h = int(h * self.dynamic_crop_ratio)
            road = bgr[h - crop_h:, :]

            # ── 3. Threshold white lane markings ────────────────────────────
            white_thresh = self._param_cache['white_threshold']
            gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(blurred, white_thresh, 255, cv2.THRESH_BINARY)

            # ── 4. Sliding window lane detection ────────────────────────────
            n_windows = 10
            window_height = max(1, crop_h // n_windows)

            left_points = []
            right_points = []
            center_points = []
            valid_frames = 0                   # windows where both lines confirmed
            estimated_lane_width = w // 2      # maintained across windows
            min_lane_width = 80                # pixels; avoids fusing both lines

            for window in range(n_windows):
                win_y_low  = crop_h - (window + 1) * window_height
                win_y_high = crop_h - window * window_height
                if win_y_high <= win_y_low:
                    break

                hist = np.sum(binary[win_y_low:win_y_high, :], axis=0)

                # Search regions — follow previous point if available
                left_start,  left_end  = 0,      w // 3
                right_start, right_end = 2*w//3, w

                if window > 0 and left_points:
                    prev_l = left_points[-1][0]
                    left_start = max(0,     prev_l - 60)
                    left_end   = min(w//2,  prev_l + 60)
                if window > 0 and right_points:
                    prev_r = right_points[-1][0]
                    right_start = max(w//2, prev_r - 60)
                    right_end   = min(w,    prev_r + 60)

                left_region  = hist[left_start:left_end]
                right_region = hist[right_start:right_end]

                valid_left  = (np.max(left_region)  >= 10) if len(left_region)  > 0 else False
                valid_right = (np.max(right_region) >= 10) if len(right_region) > 0 else False

                if valid_left and valid_right:
                    lp = np.argmax(left_region)  + left_start
                    rp = np.argmax(right_region) + right_start
                    detected_width = rp - lp
                    if detected_width >= min_lane_width:
                        valid_frames += 1
                        if detected_width < w * 2 // 3:
                            estimated_lane_width = detected_width
                        left_peak, right_peak = lp, rp
                    else:
                        left_peak  = left_points[-1][0]  if left_points  else w//2 - estimated_lane_width//2
                        right_peak = left_peak + estimated_lane_width

                elif valid_left:
                    left_peak  = np.argmax(left_region)  + left_start
                    right_peak = left_peak + estimated_lane_width

                elif valid_right:
                    right_peak = np.argmax(right_region) + right_start
                    left_peak  = right_peak - estimated_lane_width

                else:
                    # No lines found — project from last known position
                    cx_est = int(self.last_valid_error * (w / 2.0) * -1 + image_center)
                    left_peak  = left_points[-1][0]  if left_points  else cx_est - estimated_lane_width//2
                    right_peak = right_points[-1][0] if right_points else cx_est + estimated_lane_width//2

                lane_center_x = (left_peak + right_peak) // 2
                lane_center_y = int(h - crop_h + win_y_low + window_height / 2)

                left_points.append((int(left_peak),  lane_center_y))
                right_points.append((int(right_peak), lane_center_y))
                center_points.append((int(lane_center_x), lane_center_y))

            # ── 5. Compute weighted lane centre ─────────────────────────────
            n_pts    = len(center_points)
            conf_min = self._param_cache['min_valid_windows']

            if valid_frames >= conf_min:
                # Confident lane lock — use real detection
                weighted_center_x = sum(pt[0] for pt in center_points) / n_pts
                self.dynamic_crop_ratio  = self._param_cache['crop_ratio_base']
                self.current_hold_frames = self._param_cache['hold_error_frames']
                self.frames_since_valid  = 0

            elif self.current_hold_frames > 0:
                # Lane partially lost — hold last intent with decay
                weighted_center_x = image_center - (self.last_valid_error * (w / 2.0))
                self.last_valid_error   *= self._param_cache['error_decay_rate']
                self.current_hold_frames -= 1
                self.frames_since_valid  += 1
                # Expand crop to search higher for the lane
                self.dynamic_crop_ratio = min(
                    self._param_cache['crop_ratio_max'],
                    self.dynamic_crop_ratio + 0.05
                )

            else:
                # Lane fully lost — hold image centre, keep vision wide
                weighted_center_x = image_center
                self.frames_since_valid  += 1
                self.dynamic_crop_ratio   = self._param_cache['crop_ratio_max']

            # ── 6. Error calculation ────────────────────────────────────────
            # Positive error → lane centre is to the LEFT → steer LEFT
            # Negative error → lane centre is to the RIGHT → steer RIGHT
            raw_error = float(
                np.clip((image_center - weighted_center_x) / image_center, -1.0, 1.0)
            )

            # Dead zone — suppress tiny jitter on straight sections
            if abs(raw_error) < self._param_cache['dead_zone']:
                raw_error = 0.0

            # EMA smoothing
            alpha = self._param_cache['smoothing_alpha']
            self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
            self.lane_error = self.filtered_error

            # Update last_valid_error only on confident frames
            if valid_frames >= conf_min:
                self.last_valid_error = self.lane_error

            # ── 7. Publish ──────────────────────────────────────────────────
            self.error_pub.publish(Float32(data=self.lane_error))

            # ── 8. Debug visualisation ──────────────────────────────────────
            if self._param_cache['show_debug']:
                debug    = bgr.copy()
                crop_top = h - crop_h

                def draw_polyfit_curve(pts_list, color, thickness=2):
                    if len(pts_list) < 3:
                        return None
                    pts      = np.array(pts_list)
                    poly_fit = np.polyfit(pts[:, 1], pts[:, 0], 2)
                    y_eval   = np.linspace(crop_top, h - 1, 80)
                    x_eval   = np.clip(np.polyval(poly_fit, y_eval), 0, w - 1)
                    curve_pts = np.vstack((x_eval, y_eval)).astype(np.int32).T
                    cv2.polylines(debug, [curve_pts], False, (0, 0, 0), thickness + 2)
                    cv2.polylines(debug, [curve_pts], False, color, thickness)
                    return poly_fit

                draw_polyfit_curve(left_points,   (255, 130, 130), 2)
                draw_polyfit_curve(right_points,  (130, 130, 255), 2)
                center_poly = draw_polyfit_curve(center_points, (0, 255, 0), 3)

                for i, (l_pt, r_pt, c_pt) in enumerate(
                        zip(left_points, right_points, center_points)):
                    if i % 2 == 0:
                        cv2.circle(debug, l_pt, 3, (255, 130, 130), -1)
                        cv2.circle(debug, r_pt, 3, (130, 130, 255), -1)
                    cv2.circle(debug, c_pt, 4, (0, 0, 0), -1)
                    cv2.circle(debug, c_pt, 3, (0, 255, 0), -1)

                def put_text(img, text, pos, scale, color, thick=2):
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2)
                    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

                if abs(self.lane_error) < 0.05:
                    direction, t_color = 'CENTERED', (0, 255, 0)
                elif self.lane_error > 0:
                    direction, t_color = 'STEER LEFT', (0, 165, 255)
                else:
                    direction, t_color = 'STEER RIGHT', (0, 165, 255)

                steer_deg = abs(self.lane_error) * 50.0
                put_text(debug, f'{direction} ({steer_deg:.0f} deg)', (10, 30), 0.7, t_color)

                status_str = (
                    f'LOCK({valid_frames}w)' if valid_frames >= conf_min
                    else (f'HOLD({self.current_hold_frames})' if self.current_hold_frames > 0
                          else f'LOST({self.frames_since_valid}f)')
                )
                put_text(debug, status_str, (10, 55), 0.55, (0, 255, 255))

                lane_w_cm = estimated_lane_width * 40.0 / (w * 0.4)
                if center_poly is not None and abs(center_poly[0]) > 0.001:
                    R_px = abs(1.0 / (2.0 * center_poly[0]))
                    R_cm = R_px * 40.0 / max(estimated_lane_width, 1)
                    curve_dir = 'LEFT' if center_poly[0] < 0 else 'RIGHT'
                    put_text(debug, f'Curve:{curve_dir} R={R_cm:.0f}cm W={lane_w_cm:.0f}cm',
                             (10, 78), 0.5, (0, 255, 255))
                else:
                    put_text(debug, f'Straight | W={lane_w_cm:.0f}cm', (10, 78), 0.5, (0, 255, 255))

                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

            if self._param_cache['print_debug']:
                now = time.monotonic()
                if now - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    status = ('CENTER' if abs(self.lane_error) < 0.05
                              else ('TURN RIGHT' if self.lane_error < 0 else 'TURN LEFT'))
                    hold_str = f'h={self.current_hold_frames}' if self.current_hold_frames > 0 else ''
                    print(
                        f'\r[LF] Err:{self.lane_error:.2f} | {status} | '
                        f'crop={self.dynamic_crop_ratio:.2f} | {hold_str}',
                        end='', flush=True
                    )
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
