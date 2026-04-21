#!/usr/bin/env python3
"""
Line Follower Camera Node  (Cytron-style scanline detection)
Detects white lane lines using CLAHE + Otsu's adaptive thresholding and
computes a steering error from multi-scanline pixel scanning.

Algorithm (adapted from Cytron Differential Line Following for dual white lanes):
  1. Resize + crop bottom portion of camera image (road surface)
  2. CLAHE histogram equalization (adaptive lighting compensation)
  3. Gaussian blur + Otsu's auto-threshold (no fixed white_threshold!)
  4. Multiple horizontal scanlines scan for left/right white lane edges
  5. Error = average offset of lane midpoints from image center
  6. Deadband filter (tolerance threshold) + EMA smoothing
  7. Publish Float32 on /lane_error (range -1.0 to +1.0)

Reference:
  Cytron Technologies — Differential Line Following Algorithm
  https://my.cytron.io/tutorial/differential-line-following-algorithm
"""

import time
from typing import Dict, List, Optional, Tuple

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
        self.last_valid_error = 0.0  # last error from a confident scan

        # ── Tunable parameters ─────────────────────────────────────────────
        # Scanline detection
        self.declare_parameter('n_scanlines', 8)             # number of horizontal scanlines
        self.declare_parameter('min_valid_scanlines', 3)     # minimum for confident lock
        self.declare_parameter('min_line_width_px', 5)       # min white region to count as lane line
        self.declare_parameter('crop_ratio_base', 0.4)       # bottom crop ratio (road surface)
        # CLAHE adaptive lighting
        self.declare_parameter('clahe_enabled', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        # Error conditioning (Cytron-style)
        self.declare_parameter('smoothing_alpha', 0.3)       # EMA α
        self.declare_parameter('dead_zone', 0.05)            # tolerance threshold T
        # Steering persistence on lane loss
        self.declare_parameter('hold_error_frames', 15)      # frames to hold last intent
        self.declare_parameter('error_decay_rate', 0.92)     # per-frame decay factor
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
        self.frames_since_valid = 0       # consecutive frames without confident lock
        self.last_lane_width = 0          # last detected lane width in pixels

        # CLAHE object (reused across frames)
        self._clahe = cv2.createCLAHE(
            clipLimit=self._param_cache['clahe_clip_limit'],
            tileGridSize=(8, 8)
        )

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
            'Line Follower Camera: Ready (Cytron-style scanline + CLAHE + Otsu)'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Parameter helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-frame lookups."""
        self._param_cache = {
            'n_scanlines':        int(self.get_parameter('n_scanlines').value),
            'min_valid_scanlines': int(self.get_parameter('min_valid_scanlines').value),
            'min_line_width_px':  int(self.get_parameter('min_line_width_px').value),
            'crop_ratio_base':    float(self.get_parameter('crop_ratio_base').value),
            'clahe_enabled':      bool(self.get_parameter('clahe_enabled').value),
            'clahe_clip_limit':   float(self.get_parameter('clahe_clip_limit').value),
            'smoothing_alpha':    float(self.get_parameter('smoothing_alpha').value),
            'dead_zone':          float(self.get_parameter('dead_zone').value),
            'hold_error_frames':  int(self.get_parameter('hold_error_frames').value),
            'error_decay_rate':   float(self.get_parameter('error_decay_rate').value),
            'show_debug':         bool(self.get_parameter('show_debug').value),
            'resize_width':       int(self.get_parameter('resize_width').value),
            'print_debug':        bool(self.get_parameter('print_debug').value),
            'debug_print_rate':   float(self.get_parameter('debug_print_rate').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or dashboard."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                # Recreate CLAHE if clip limit changed
                if p.name == 'clahe_clip_limit':
                    self._clahe = cv2.createCLAHE(
                        clipLimit=float(p.value), tileGridSize=(8, 8)
                    )
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────────
    # Scanline detection — Cytron-style pixel scanning
    # ──────────────────────────────────────────────────────────────────────────

    def _scan_row_for_white_region(
        self, row: np.ndarray, start: int, end: int, step: int,
        min_width: int
    ) -> Optional[int]:
        """Scan a binary row to find the center of the first white region.

        Args:
            row: 1D array of binary pixel values (0 or 255).
            start: pixel index to start scanning from.
            end: pixel index to stop scanning at (exclusive).
            step: +1 for left→right, -1 for right→left.
            min_width: minimum consecutive white pixels to count as a line.

        Returns:
            Center x-coordinate of the detected white region, or None.
        """
        in_white = False
        white_start = 0
        x = start
        while (step > 0 and x < end) or (step < 0 and x >= end):
            if row[x] == 255:
                if not in_white:
                    white_start = x
                    in_white = True
            else:
                if in_white:
                    white_end = x
                    width = abs(white_end - white_start)
                    if width >= min_width:
                        # Return center of this white region
                        return (white_start + white_end) // 2
                    in_white = False
            x += step

        # Handle case where white region extends to the edge
        if in_white:
            white_end = x
            width = abs(white_end - white_start)
            if width >= min_width:
                return (white_start + white_end) // 2

        return None

    def _detect_scanlines(
        self, binary: np.ndarray, crop_h: int, w: int
    ) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]], List[Tuple[int, int]], int]:
        """Run multi-scanline detection on the binary image.

        For each scanline:
          - Scan left→right from left edge to find LEFT lane line
          - Scan right→left from right edge to find RIGHT lane line
          - Compute midpoint between the two lines

        Returns:
            (left_points, right_points, center_points, valid_count)
        """
        n_scanlines = self._param_cache['n_scanlines']
        min_width = self._param_cache['min_line_width_px']

        left_points = []
        right_points = []
        center_points = []
        valid_count = 0

        # Start scanning radially outwards from the track center.
        # This prevents locking onto parallel adjacent lanes!
        search_center = w // 2

        for i in range(n_scanlines):
            # Distribute scanlines evenly across the crop region
            # Bottom scanline is closest to the robot, top is furthest ahead
            y_frac = (i + 0.5) / n_scanlines
            y_in_crop = int(crop_h * (1.0 - y_frac))
            y_in_crop = max(0, min(crop_h - 1, y_in_crop))

            row = binary[y_in_crop, :]

            # Scan OUTWARDS from the track center.
            # Scan for left lane line: right→left
            left_x = self._scan_row_for_white_region(
                row, min(search_center + 10, w - 1), 0, -1, min_width
            )

            # Scan for right lane line: left→right
            right_x = self._scan_row_for_white_region(
                row, max(search_center - 10, 0), w, +1, min_width
            )

            # Determine lane center
            if left_x is not None and right_x is not None:
                # Both lines found — confident detection
                if right_x > left_x:  # sanity check
                    valid_count += 1
                    self.last_lane_width = right_x - left_x
                    center_x = (left_x + right_x) // 2
                    # Trace the curve: next row up will start from THIS center!
                    search_center = center_x 
                else:
                    # Lines crossed (noise) — use last known width
                    center_x = search_center
                    left_x = search_center - self.last_lane_width // 2
                    right_x = search_center + self.last_lane_width // 2

            elif left_x is not None and self.last_lane_width > 0:
                # Only left line found — estimate right from known width
                right_x = left_x + self.last_lane_width
                center_x = (left_x + right_x) // 2
                search_center = center_x

            elif right_x is not None and self.last_lane_width > 0:
                # Only right line found — estimate left from known width
                left_x = right_x - self.last_lane_width
                center_x = (left_x + right_x) // 2
                search_center = center_x

            else:
                # Neither line found — skip this scanline
                continue

            left_points.append((int(left_x), y_in_crop))
            right_points.append((int(right_x), y_in_crop))
            center_points.append((int(center_x), y_in_crop))

        return left_points, right_points, center_points, valid_count

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

            # ── 2. Crop bottom portion (road surface) ───────────────────────
            crop_ratio = self._param_cache['crop_ratio_base']
            crop_h = int(h * crop_ratio)
            road = bgr[h - crop_h:, :]

            # ── 3. CLAHE + Otsu's adaptive threshold ────────────────────────
            gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)

            # CLAHE: normalize lighting across the image
            if self._param_cache['clahe_enabled']:
                gray = self._clahe.apply(gray)

            # Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Otsu's auto-threshold — no more fixed white_threshold!
            _, binary = cv2.threshold(
                blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )

            # ── 4. Multi-scanline detection ─────────────────────────────────
            left_pts, right_pts, center_pts, valid_count = \
                self._detect_scanlines(binary, crop_h, w)

            # ── 5. Compute average error (Cytron formula) ───────────────────
            conf_min = self._param_cache['min_valid_scanlines']

            if valid_count >= conf_min and len(center_pts) > 0:
                # Confident lock — average error across all scanlines
                # e_i = x_center,i - x_image_center
                # e_avg = (1/N) * Σ e_i
                avg_center_x = sum(pt[0] for pt in center_pts) / len(center_pts)
                self.current_hold_frames = self._param_cache['hold_error_frames']
                self.frames_since_valid = 0

            elif self.current_hold_frames > 0:
                # Lane partially lost — hold last error with decay
                avg_center_x = image_center - (self.last_valid_error * image_center)
                self.last_valid_error *= self._param_cache['error_decay_rate']
                self.current_hold_frames -= 1
                self.frames_since_valid += 1

            else:
                # Lane fully lost — center error
                avg_center_x = image_center
                self.frames_since_valid += 1

            # ── 6. Error calculation ────────────────────────────────────────
            # Positive error → lane centre is to the LEFT → steer LEFT
            raw_error = float(
                np.clip((avg_center_x - image_center) / image_center, -1.0, 1.0)
            )

            # Deadband filter (Cytron tolerance threshold T)
            # e_deadband = 0 if |e_avg| < T, else e_avg
            if abs(raw_error) < self._param_cache['dead_zone']:
                raw_error = 0.0

            # EMA smoothing (Cytron formula)
            # e_smooth(t) = α · e_deadband(t) + (1 - α) · e_smooth(t - 1)
            alpha = self._param_cache['smoothing_alpha']
            self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
            self.lane_error = self.filtered_error

            # Update last_valid_error only on confident frames
            if valid_count >= conf_min:
                self.last_valid_error = self.lane_error

            # ── 7. Publish ──────────────────────────────────────────────────
            self.error_pub.publish(Float32(data=self.lane_error))

            # ── 8. Debug visualisation ──────────────────────────────────────
            if self._param_cache['show_debug']:
                debug = bgr.copy()
                crop_top = h - crop_h

                # Draw scanline detection points
                for lp, rp, cp in zip(left_pts, right_pts, center_pts):
                    # Offset y to full image coordinates
                    ly = lp[1] + crop_top
                    ry = rp[1] + crop_top
                    cy = cp[1] + crop_top

                    # Left line point (blue)
                    cv2.circle(debug, (lp[0], ly), 4, (255, 130, 130), -1)
                    # Right line point (pink)
                    cv2.circle(debug, (rp[0], ry), 4, (130, 130, 255), -1)
                    # Center point (green)
                    cv2.circle(debug, (cp[0], cy), 5, (0, 255, 0), -1)
                    # Scanline visualization
                    cv2.line(debug, (lp[0], ly), (rp[0], ry), (50, 50, 50), 1)

                # Draw center reference line
                cv2.line(debug, (w // 2, crop_top), (w // 2, h), (0, 0, 255), 1)

                # Status text
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
                    f'LOCK({valid_count}/{self._param_cache["n_scanlines"]})'
                    if valid_count >= conf_min
                    else (f'HOLD({self.current_hold_frames})'
                          if self.current_hold_frames > 0
                          else f'LOST({self.frames_since_valid}f)')
                )
                put_text(debug, status_str, (10, 55), 0.55, (0, 255, 255))

                if self.last_lane_width > 0:
                    lane_w_cm = self.last_lane_width * 40.0 / (w * 0.4)
                    put_text(debug, f'W={lane_w_cm:.0f}cm', (10, 78), 0.5, (0, 255, 255))

                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

            if self._param_cache['print_debug']:
                now = time.monotonic()
                if now - self._last_debug_print >= self._param_cache['debug_print_rate']:
                    status = ('CENTER' if abs(self.lane_error) < 0.05
                              else ('TURN RIGHT' if self.lane_error < 0 else 'TURN LEFT'))
                    hold_str = f'h={self.current_hold_frames}' if self.current_hold_frames > 0 else ''
                    print(
                        f'\r[LF] Err:{self.lane_error:.2f} | {status} | '
                        f'valid={valid_count}/{self._param_cache["n_scanlines"]} | {hold_str}',
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
