#!/usr/bin/env python3
"""
Tunnel Wall Follower Node
LiDAR-based wall following for the tunnel section where camera lane detection
may not work due to poor lighting/visibility.

Maintains equal distance from left and right walls using a PD controller.
Publishes Twist on /tunnel_cmd_vel for auto_driver to use when in TUNNEL state.
"""

import math
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .topics import TUNNEL_CMD_TOPIC, TUNNEL_DETECTED_TOPIC


class TunnelWallFollower(Node):
    """LiDAR-based wall following for tunnel sections."""

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
        self.declare_parameter('heartbeat_sec', 0.2)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, TUNNEL_CMD_TOPIC, 10)
        self.in_tunnel_pub = self.create_publisher(Bool, TUNNEL_DETECTED_TOPIC, 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd = Twist()
        self.last_in_tunnel = False
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )

        self.get_logger().info('Tunnel Wall Follower started')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-scan lookups."""
        self._param_cache = {
            'target_center_dist': float(self.get_parameter('target_center_dist').value),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'kp': float(self.get_parameter('kp').value),
            'kd': float(self.get_parameter('kd').value),
            'max_angular': float(self.get_parameter('max_angular').value),
            'left_angle_min': float(self.get_parameter('left_angle_min').value),
            'left_angle_max': float(self.get_parameter('left_angle_max').value),
            'right_angle_min': float(self.get_parameter('right_angle_min').value),
            'right_angle_max': float(self.get_parameter('right_angle_max').value),
            'lidar_angle_offset': float(self.get_parameter('lidar_angle_offset').value),
            'min_wall_points': int(self.get_parameter('min_wall_points').value),
            'max_wall_dist': float(self.get_parameter('max_wall_dist').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _heartbeat_publish(self) -> None:
        """Republish last state on a fixed heartbeat."""
        self.in_tunnel_pub.publish(Bool(data=self.last_in_tunnel))
        self.cmd_vel_pub.publish(self.last_cmd)

    def scan_callback(self, msg: LaserScan) -> None:
        angle_offset = self._param_cache['lidar_angle_offset']
        left_min = self._param_cache['left_angle_min']
        left_max = self._param_cache['left_angle_max']
        right_min = self._param_cache['right_angle_min']
        right_max = self._param_cache['right_angle_max']
        max_wall = self._param_cache['max_wall_dist']
        min_pts = self._param_cache['min_wall_points']

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
        self.last_in_tunnel = in_tunnel

        cmd = Twist()

        if in_tunnel:
            # Calculate average distances to each wall
            left_avg = np.mean(left_dists)
            right_avg = np.mean(right_dists)

            # Error = difference between left and right wall distances
            # Positive error = closer to right wall → steer left
            # Negative error = closer to left wall → steer right
            target = self._param_cache['target_center_dist']
            error = (right_avg - left_avg) + target

            # PD controller
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt > 0:
                derivative = (error - self.last_error) / dt
            else:
                derivative = 0.0

            kp = self._param_cache['kp']
            kd = self._param_cache['kd']
            max_ang = self._param_cache['max_angular']

            angular_z = kp * error + kd * derivative
            angular_z = max(-max_ang, min(max_ang, angular_z))

            cmd.linear.x = self._param_cache['forward_speed']
            cmd.angular.z = angular_z

            self.last_error = error
            self.last_time = now

            # Debug
            self.get_logger().debug(f"L: {left_avg:.2f}m  R: {right_avg:.2f}m  err: {error:.3f}  ang: {angular_z:.2f}")
        else:
            # Not in tunnel — publish zero
            self.last_error = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.last_cmd = cmd


def main(args=None) -> None:
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
