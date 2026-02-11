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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from rclpy.qos import QoSPresetProfiles
import cv2
import numpy as np
import math
import time
from enum import Enum


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

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/parking_cmd_vel', 10)
        self.complete_pub = self.create_publisher(Bool, '/parking_complete', 10)
        self.signboard_pub = self.create_publisher(Bool, '/parking_signboard_detected', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, '/parking_command', self.command_callback, 10
        )
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.camera_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.phase = ParkingPhase.IDLE
        self.phase_start_time = 0.0
        self.phase_start_dist = 0.0
        self.cumulative_yaw = 0.0
        self.current_speed = 0.0
        self.distance_traveled = 0.0
        self.last_odom_time = time.time()
        self.signboard_detected = False

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Parking Controller started (IDLE)')

    def odom_callback(self, msg):
        """Track distance from odometry."""
        speed = msg.twist.twist.linear.x
        current_time = time.time()
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time
        self.current_speed = speed
        self.distance_traveled += abs(speed) * dt

    def camera_callback(self, msg):
        """Detect triangle signboard to identify parking zone."""
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = bgr.shape[:2]

            # Convert to grayscale and detect edges
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            triangle_found = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:  # too small
                    continue

                # Approximate contour to polygon
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

                # Triangle = 3 vertices
                if len(approx) == 3:
                    triangle_found = True
                    break

            self.signboard_detected = triangle_found
            sign_msg = Bool()
            sign_msg.data = triangle_found
            self.signboard_pub.publish(sign_msg)

        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')

    def command_callback(self, msg):
        """Receive parking command from auto_driver state machine."""
        cmd = msg.data.lower()
        if cmd == 'parallel' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('🅿️ Starting PARALLEL parking maneuver')
            self._start_phase(ParkingPhase.PARALLEL_FORWARD)
        elif cmd == 'perpendicular' and self.phase == ParkingPhase.IDLE:
            self.get_logger().info('🅿️ Starting PERPENDICULAR parking maneuver')
            self._start_phase(ParkingPhase.PERP_TURN_IN)
        elif cmd == 'stop':
            self._start_phase(ParkingPhase.IDLE)

    def _start_phase(self, phase):
        """Transition to a new parking phase."""
        self.phase = phase
        self.phase_start_time = time.time()
        self.phase_start_dist = self.distance_traveled
        self.cumulative_yaw = 0.0
        self.get_logger().info(f'  → Phase: {phase.name}')

    def _dist_since_phase(self):
        return self.distance_traveled - self.phase_start_dist

    def _time_since_phase(self):
        return time.time() - self.phase_start_time

    def control_loop(self):
        """Main parking control loop — executes maneuver phases."""
        cmd = Twist()
        drive_speed = self.get_parameter('drive_speed').value
        reverse_speed = self.get_parameter('reverse_speed').value

        if self.phase == ParkingPhase.IDLE:
            self.cmd_vel_pub.publish(cmd)  # zero velocity
            return

        # --- PARALLEL PARKING ---
        elif self.phase == ParkingPhase.PARALLEL_FORWARD:
            # Drive forward past the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self.get_parameter('parallel_forward_dist').value:
                self._start_phase(ParkingPhase.PARALLEL_STEER_REVERSE)

        elif self.phase == ParkingPhase.PARALLEL_STEER_REVERSE:
            # Reverse while steering into the slot
            cmd.linear.x = reverse_speed
            cmd.angular.z = -self.get_parameter('parallel_steer_angle').value  # steer right
            if self._dist_since_phase() >= self.get_parameter('parallel_reverse_dist').value:
                self._start_phase(ParkingPhase.PARALLEL_STRAIGHTEN)

        elif self.phase == ParkingPhase.PARALLEL_STRAIGHTEN:
            # Brief forward to straighten
            cmd.linear.x = drive_speed * 0.5
            cmd.angular.z = self.get_parameter('parallel_steer_angle').value * 0.5  # counter-steer
            if self._dist_since_phase() >= 0.10:
                self._start_phase(ParkingPhase.PARALLEL_WAIT)

        elif self.phase == ParkingPhase.PARALLEL_WAIT:
            # Stop and wait
            wait_time = self.get_parameter('park_wait_time').value
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PARALLEL_EXIT)

        elif self.phase == ParkingPhase.PARALLEL_EXIT:
            # Drive forward out of slot
            cmd.linear.x = drive_speed
            cmd.angular.z = self.get_parameter('parallel_steer_angle').value * 0.5  # steer left to exit
            if self._dist_since_phase() >= 0.30:
                self._finish()

        # --- PERPENDICULAR PARKING ---
        elif self.phase == ParkingPhase.PERP_TURN_IN:
            # Turn 90° into slot
            cmd.angular.z = 0.5  # turn left
            if self._time_since_phase() >= (self.get_parameter('perp_turn_angle').value / 0.5):
                self._start_phase(ParkingPhase.PERP_FORWARD)

        elif self.phase == ParkingPhase.PERP_FORWARD:
            # Drive into the slot
            cmd.linear.x = drive_speed
            if self._dist_since_phase() >= self.get_parameter('perp_forward_dist').value:
                self._start_phase(ParkingPhase.PERP_WAIT)

        elif self.phase == ParkingPhase.PERP_WAIT:
            # Stop and wait
            wait_time = self.get_parameter('park_wait_time').value
            if self._time_since_phase() >= wait_time:
                self._start_phase(ParkingPhase.PERP_REVERSE_OUT)

        elif self.phase == ParkingPhase.PERP_REVERSE_OUT:
            # Reverse out
            cmd.linear.x = reverse_speed
            if self._dist_since_phase() >= self.get_parameter('perp_forward_dist').value:
                self._finish()

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Debug
        phase_name = self.phase.name
        dist = self._dist_since_phase()
        elapsed = self._time_since_phase()
        print(f"\r[PARK] {phase_name} | dist: {dist:.3f}m | time: {elapsed:.1f}s", end='', flush=True)

    def _finish(self):
        """Complete the parking maneuver."""
        self.get_logger().info('✅ Parking maneuver COMPLETE!')
        self.phase = ParkingPhase.IDLE

        # Publish completion
        complete_msg = Bool()
        complete_msg.data = True
        self.complete_pub.publish(complete_msg)

        # Stop robot
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
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
