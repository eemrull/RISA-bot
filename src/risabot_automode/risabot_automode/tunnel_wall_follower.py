#!/usr/bin/env python3
"""
Tunnel Wall Follower Node
LiDAR-based wall following for the tunnel section where camera lane detection
may not work due to poor lighting/visibility.

Maintains equal distance from left and right walls using a PD controller.
Publishes Twist on /tunnel_cmd_vel for auto_driver to use when in TUNNEL state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSPresetProfiles
import math
import numpy as np


class TunnelWallFollower(Node):
    def __init__(self):
        super().__init__('tunnel_wall_follower')

        # --- Parameters ---
        self.declare_parameter('target_center_dist', 0.0)    # 0 = center between walls
        self.declare_parameter('forward_speed', 0.15)         # m/s
        self.declare_parameter('kp', 1.2)                     # proportional gain
        self.declare_parameter('kd', 0.3)                     # derivative gain
        self.declare_parameter('max_angular', 0.8)            # max turn rate rad/s
        self.declare_parameter('left_angle_min', 0.52)        # ~30° in radians
        self.declare_parameter('left_angle_max', 1.57)        # ~90°
        self.declare_parameter('right_angle_min', -1.57)      # ~-90°
        self.declare_parameter('right_angle_max', -0.52)      # ~-30°
        self.declare_parameter('lidar_angle_offset', 1.5708)  # 90° mount correction
        self.declare_parameter('min_wall_points', 3)          # minimum points to consider a wall
        self.declare_parameter('max_wall_dist', 0.60)         # ignore points beyond this

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/tunnel_cmd_vel', 10)
        self.in_tunnel_pub = self.create_publisher(Bool, '/tunnel_detected', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info('Tunnel Wall Follower started')

    def scan_callback(self, msg):
        angle_offset = self.get_parameter('lidar_angle_offset').value
        left_min = self.get_parameter('left_angle_min').value
        left_max = self.get_parameter('left_angle_max').value
        right_min = self.get_parameter('right_angle_min').value
        right_max = self.get_parameter('right_angle_max').value
        max_wall = self.get_parameter('max_wall_dist').value
        min_pts = self.get_parameter('min_wall_points').value

        left_dists = []
        right_dists = []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue
            if r > max_wall:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle += angle_offset
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Classify into left wall or right wall
            if left_min <= angle <= left_max:
                left_dists.append(r)
            elif right_min <= angle <= right_max:
                right_dists.append(r)

        # Detect if we're in a tunnel-like environment (walls on both sides)
        in_tunnel = len(left_dists) >= min_pts and len(right_dists) >= min_pts
        tunnel_msg = Bool()
        tunnel_msg.data = in_tunnel
        self.in_tunnel_pub.publish(tunnel_msg)

        cmd = Twist()

        if in_tunnel:
            # Calculate average distances to each wall
            left_avg = np.mean(left_dists)
            right_avg = np.mean(right_dists)

            # Error = difference between left and right wall distances
            # Positive error = closer to right wall → steer left
            # Negative error = closer to left wall → steer right
            target = self.get_parameter('target_center_dist').value
            error = (right_avg - left_avg) + target

            # PD controller
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt > 0:
                derivative = (error - self.last_error) / dt
            else:
                derivative = 0.0

            kp = self.get_parameter('kp').value
            kd = self.get_parameter('kd').value
            max_ang = self.get_parameter('max_angular').value

            angular_z = kp * error + kd * derivative
            angular_z = max(-max_ang, min(max_ang, angular_z))

            cmd.linear.x = self.get_parameter('forward_speed').value
            cmd.angular.z = angular_z

            self.last_error = error
            self.last_time = now

            # Debug
            print(f"\r[TUNNEL] L: {left_avg:.2f}m  R: {right_avg:.2f}m  err: {error:.3f}  ang: {angular_z:.2f}", end='', flush=True)
        else:
            # Not in tunnel — publish zero
            self.last_error = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TunnelWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
