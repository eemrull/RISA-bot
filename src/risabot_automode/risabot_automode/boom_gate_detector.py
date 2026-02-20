#!/usr/bin/env python3
"""
Boom Gate Detector Node
Detects whether a boom gate barrier is blocking the lane ahead using LiDAR.
The gate appears as a narrow horizontal line of closely-spaced points in the
forward arc at a specific distance.

Publishes Bool on /boom_gate_open (True = open/clear, False = blocked).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSPresetProfiles
import math
import numpy as np


class BoomGateDetector(Node):
    def __init__(self):
        super().__init__('boom_gate_detector')

        # --- Parameters ---
        # Detection zone: only check for gate within this distance range
        self.declare_parameter('min_detect_dist', 0.15)   # meters
        self.declare_parameter('max_detect_dist', 0.80)   # meters
        # Angular window: check ±angle_window radians from front (0°)
        self.declare_parameter('angle_window', 0.35)       # ~20° each side
        # Gate detection: if a dense cluster of points spans this angular width
        # at roughly the same distance, it's a gate
        self.declare_parameter('min_gate_points', 5)       # min points forming gate
        self.declare_parameter('distance_variance_max', 0.05)  # points must be at similar dist
        # Lidar mount offset (same 90° correction as obstacle_avoidance)
        self.declare_parameter('lidar_angle_offset', 1.5708)  # pi/2

        # Publisher
        self.gate_pub = self.create_publisher(Bool, '/boom_gate_open', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        # Hysteresis (require N consecutive readings to change state)
        self.declare_parameter('hysteresis', 3)

        # State
        self.gate_blocked = False
        self.blocked_count = 0
        self.clear_count = 0

        self.get_logger().info('Boom Gate Detector started')

    def scan_callback(self, msg):
        min_dist = self.get_parameter('min_detect_dist').value
        max_dist = self.get_parameter('max_detect_dist').value
        angle_win = self.get_parameter('angle_window').value
        min_pts = self.get_parameter('min_gate_points').value
        dist_var_max = self.get_parameter('distance_variance_max').value
        angle_offset = self.get_parameter('lidar_angle_offset').value

        # Collect points in the forward detection zone
        forward_distances = []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # Apply mount correction
            angle += angle_offset
            # Normalize to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Only look at the front arc
            if -angle_win <= angle <= angle_win:
                if min_dist <= r <= max_dist:
                    forward_distances.append(r)

        # Gate detection logic:
        # A boom gate creates a dense cluster of points at roughly the same distance
        is_blocked = False
        if len(forward_distances) >= min_pts:
            distances = np.array(forward_distances)
            # Check if points cluster tightly (low variance = solid barrier)
            dist_std = np.std(distances)
            dist_mean = np.mean(distances)
            # Require both: low spread AND reasonable distance (not noise at range_max)
            if dist_std < dist_var_max and dist_mean < max_dist * 0.9:
                is_blocked = True

        # Hysteresis to avoid flicker
        if is_blocked:
            self.blocked_count += 1
            self.clear_count = 0
        else:
            self.clear_count += 1
            self.blocked_count = 0

        hysteresis = self.get_parameter('hysteresis').value
        if self.blocked_count >= hysteresis and not self.gate_blocked:
            self.gate_blocked = True
            self.get_logger().warn('🚧 Boom gate CLOSED — barrier detected!')
        elif self.clear_count >= hysteresis and self.gate_blocked:
            self.gate_blocked = False
            self.get_logger().info('✅ Boom gate OPEN — path clear.')

        # Publish
        gate_msg = Bool()
        gate_msg.data = not self.gate_blocked  # True = open
        self.gate_pub.publish(gate_msg)

        # Debug
        status = "🚧 CLOSED" if self.gate_blocked else "✅ OPEN"
        pts = len(forward_distances)
        self.get_logger().debug(f"{status} | fwd points: {pts} | blocked_cnt: {self.blocked_count} | clear_cnt: {self.clear_count}")


def main(args=None):
    rclpy.init(args=args)
    node = BoomGateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
