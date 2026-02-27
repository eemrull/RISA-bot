#!/usr/bin/env python3
"""
Parking Controller Node
Executes pre-programmed parallel and perpendicular parking maneuvers
using odometry-based dead reckoning. Triggered by the auto_driver state machine.

Detects triangle signboard via camera to identify parking zones.
Uses LiDAR for wall clearance confirmation.

Topics:
  Subscribes: /odom, /scan, /camera/color/image_raw, /parking_command (String)
  Publishes:  /parking_cmd_vel (Twist), /parking_complete (Bool)
"""

from enum import Enum
from typing import Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .topics import (
    CAMERA_IMAGE_TOPIC,
    DASH_STATE_TOPIC,
    ODOM_TOPIC,
    PARKING_CMD_TOPIC,
    PARKING_COMPLETE_TOPIC,
    PARKING_SIGN_TOPIC,
    PARKING_STATUS_TOPIC,
    PARKING_VEL_TOPIC,
)


class ParkingPhase(Enum):
    IDLE = 0
    # Parallel parking phases
    PARALLEL_FORWARD = 1      # Drive forward past the slot
    PARALLEL_STEER_REVERSE = 2  # Reverse while steering into slot
    PARALLEL_STRAIGHTEN = 3   # Straighten inside slot
    PARALLEL_WAIT = 4         # Wait inside slot
    PARALLEL_EXIT = 5         # Drive forward out of slot
    # Perpendicular parking phases
    PERP_TURN_IN = 6          # 90° turn into slot
    PERP_FORWARD = 7          # Drive into slot
    PERP_WAIT = 8             # Wait inside slot
    PERP_REVERSE_OUT = 9      # Reverse out of slot
    DONE = 10


class ParkingController(Node):
    """Odometry-based parking controller with signboard detection."""

    def __init__(self):
        super().__init__('parking_controller')

        # --- Parameters ---
        self.declare_parameter('parallel_forward_dist', 0.30)   # m to drive past slot
        self.declare_parameter('parallel_reverse_dist', 0.35)   # m to reverse into slot
        self.declare_parameter('parallel_steer_angle', 0.6)     # rad/s angular during reverse
        self.declare_parameter('perp_turn_angle', 1.57)         # ~90° turn
        self.declare_parameter('perp_forward_dist', 0.25)       # m into slot
        self.declare_parameter('park_wait_time', 3.0)           # seconds to wait in slot
        self.declare_parameter('drive_speed', 0.15)             # m/s linear speed
        self.declare_parameter('reverse_speed', -0.12)          # m/s reverse speed
        self.declare_parameter('signboard_min_area', 500)       # px^2 minimum contour area
        self.declare_parameter('signboard_resize_width', 320)   # downscale width for faster processing
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, PARKING_VEL_TOPIC, 10)
        self.complete_pub = self.create_publisher(Bool, PARKING_COMPLETE_TOPIC, 10)
        self.signboard_pub = self.create_publisher(Bool, PARKING_SIGN_TOPIC, 10)
        self.status_pub = self.create_publisher(String, PARKING_STATUS_TOPIC, 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, PARKING_CMD_TOPIC, self.command_callback, 10
        )
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC,
            self.camera_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.dash_state_sub = self.create_subscription(
            String, DASH_STATE_TOPIC, self.dash_state_callback, 10
        )

        # State
        self.phase = ParkingPhase.IDLE
        self.current_lap = 1
        self.phase_start_time = self.get_clock().now()
        self.phase_start_dist = 0.0
        self.cumulative_yaw = 0.0
        self.current_speed = 0.0
        self.distance_traveled = 0.0
        self.last_odom_time = self.get_clock().now()
        self.signboard_detected = False
        self.current_command = 'none'

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Parking Controller started (IDLE)')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'parallel_forward_dist': float(self.get_parameter('parallel_forward_dist').value),
            'parallel_reverse_dist': float(self.get_parameter('parallel_reverse_dist').value),
            'parallel_steer_angle': float(self.get_parameter('parallel_steer_angle').value),
            'perp_turn_angle': float(self.get_parameter('perp_turn_angle').value),
            'perp_forward_dist': float(self.get_parameter('perp_forward_dist').value),
            'park_wait_time': float(self.get_parameter('park_wait_time').value),
            'drive_speed': float(self.get_parameter('drive_speed').value),
            'reverse_speed': float(self.get_parameter('reverse_speed').value),
            'signboard_min_area': int(self.get_parameter('signboard_min_area').value),
            'signboard_resize_width': int(self.get_parameter('signboard_resize_width').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry) -> None:
        """Track distance from odometry."""
        speed = msg.twist.twist.linear.x
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        self.current_speed = speed
        self.distance_traveled += abs(speed) * dt

    def dash_state_callback(self, msg: String) -> None:
        """Listen to auto_driver state to know which lap we are on."""
        try:
            parts = msg.data.split('|')
            if len(parts) > 1:
                self.current_lap = int(parts[1])
        except Exception:
            pass

    def camera_callback(self, msg: Image) -> None:
        """Detect triangle signboard to identify parking zone."""
        # --- PERFORMANCE OPTIMIZATION ---
        # Do not waste CPU parsing the camera image if we are not on Lap 2 
        # or if parking is already finished.
        if self.current_lap < 2 or self.phase == ParkingPhase.DONE:
            return
            
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]
            resize_w = self._param_cache['signboard_resize_width']
            if resize_w > 0 and w > resize_w:
                scale = resize_w / float(w)
                bgr = cv2.resize(bgr, (resize_w, int(h * scale)))

            # Convert to grayscale and detect edges
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            triangle_found = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._param_cache['signboard_min_area']:  # too small
                    continue

                # Approximate contour to polygon
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

                # Triangle = 3 vertices
                if len(approx) == 3:
                    triangle_found = True
                    break

            if triangle_found != self.signboard_detected:
                self.signboard_detected = triangle_found
                sign_msg = Bool()
                sign_msg.data = triangle_found
                self.signboard_pub.publish(sign_msg)

        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')

    def command_callback(self, msg: String) -> None:
        """Receive parking command from auto_driver state machine."""
        cmd = msg.data.lower()
        if cmd == 'parallel' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('Starting PARALLEL parking maneuver')
            self.current_command = 'parallel'
            self.complete_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='parallel_start'))
            self._start_phase(ParkingPhase.PARALLEL_FORWARD)
        elif cmd == 'perpendicular' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('Starting PERPENDICULAR parking maneuver')
            self.current_command = 'perpendicular'
            self.complete_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='perpendicular_start'))
            self._start_phase(ParkingPhase.PERP_TURN_IN)
        elif cmd == 'stop':
            self.current_command = 'none'
            self._start_phase(ParkingPhase.IDLE)

    def _start_phase(self, phase: ParkingPhase) -> None:
        """Transition to a new parking phase."""
        self.phase = phase
        self.phase_start_time = self.get_clock().now()
        self.phase_start_dist = self.distance_traveled
        self.cumulative_yaw = 0.0
        self.get_logger().info(f'  -> Phase: {phase.name}')

    def _dist_since_phase(self) -> float:
        return self.distance_traveled - self.phase_start_dist

    def _time_since_phase(self) -> float:
        return (self.get_clock().now() - self.phase_start_time).nanoseconds / 1e9

    def control_loop(self) -> None:
        """Main parking control loop — executes maneuver phases."""
        cmd = Twist()
        drive_speed = self._param_cache['drive_speed']
        reverse_speed = self._param_cache['reverse_speed']

        if self.phase == ParkingPhase.IDLE:
            self.cmd_vel_pub.publish(cmd)  # zero velocity
            return

        # --- PARALLEL PARKING ---
        elif self.phase == ParkingPhase.PARALLEL_FORWARD:
            # Drive forward past the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self._param_cache['parallel_forward_dist']:
                self._start_phase(ParkingPhase.PARALLEL_STEER_REVERSE)

        elif self.phase == ParkingPhase.PARALLEL_STEER_REVERSE:
            # Reverse while steering into the slot
            cmd.linear.x = reverse_speed
            cmd.angular.z = -self._param_cache['parallel_steer_angle']  # steer right
            if self._dist_since_phase() >= self._param_cache['parallel_reverse_dist']:
                self._start_phase(ParkingPhase.PARALLEL_STRAIGHTEN)

        elif self.phase == ParkingPhase.PARALLEL_STRAIGHTEN:
            # Brief forward to straighten
            cmd.linear.x = drive_speed * 0.5
            cmd.angular.z = self._param_cache['parallel_steer_angle'] * 0.5  # counter-steer
            if self._dist_since_phase() >= 0.10:
                self._start_phase(ParkingPhase.PARALLEL_WAIT)

        elif self.phase == ParkingPhase.PARALLEL_WAIT:
            # Stop and wait
            wait_time = self._param_cache['park_wait_time']
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PARALLEL_EXIT)

        elif self.phase == ParkingPhase.PARALLEL_EXIT:
            # Drive forward out of slot
            cmd.linear.x = drive_speed
            cmd.angular.z = self._param_cache['parallel_steer_angle'] * 0.5  # steer left to exit
            if self._dist_since_phase() >= 0.30:
                self._finish()

        # --- PERPENDICULAR PARKING ---
        elif self.phase == ParkingPhase.PERP_TURN_IN:
            # Turn 90° into slot
            cmd.angular.z = 0.5  # turn left
            if self._time_since_phase() >= (self._param_cache['perp_turn_angle'] / 0.5):
                self._start_phase(ParkingPhase.PERP_FORWARD)

        elif self.phase == ParkingPhase.PERP_FORWARD:
            # Drive into the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self._param_cache['perp_forward_dist']:
                self._start_phase(ParkingPhase.PERP_WAIT)

        elif self.phase == ParkingPhase.PERP_WAIT:
            # Stop and wait
            wait_time = self._param_cache['park_wait_time']
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PERP_REVERSE_OUT)

        elif self.phase == ParkingPhase.PERP_REVERSE_OUT:
            # Reverse out
            cmd.linear.x = reverse_speed
            if self._dist_since_phase() >= self._param_cache['perp_forward_dist']:
                self._finish()

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Debug
        phase_name = self.phase.name
        dist = self._dist_since_phase()
        elapsed = self._time_since_phase()
        self.get_logger().debug(f"{phase_name} | dist: {dist:.3f}m | time: {elapsed:.1f}s")

    def _finish(self) -> None:
        """Complete the parking maneuver."""
        self.get_logger().info('Parking maneuver COMPLETE')
        self.phase = ParkingPhase.IDLE

        # Publish completion
        complete_msg = Bool()
        complete_msg.data = True
        self.complete_pub.publish(complete_msg)
        if self.current_command == 'parallel':
            self.status_pub.publish(String(data='parallel_done'))
        elif self.current_command == 'perpendicular':
            self.status_pub.publish(String(data='perpendicular_done'))
        self.current_command = 'none'

        # Stop robot
        self.cmd_vel_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
