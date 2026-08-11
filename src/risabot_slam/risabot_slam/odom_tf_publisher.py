#!/usr/bin/env python3
"""
Publish the odom -> base_link transform for slam_toolbox.

slam_toolbox consumes TF and /scan only -- it never subscribes to an odometry
topic -- so this node is the entire odometry interface to SLAM.

servo_controller integrates its heading from the *commanded* servo position with
no steering feedback, so its published orientation drifts without bound. This
node keeps only the distance travelled and takes heading from the IMU.

Distance comes from /odom/path_length (signed cumulative metres, straight off the
encoder ticks). An older control_servo does not publish it, so the /odom pose
chord remains as a fallback until the first path-length message arrives.
"""

import json
import math

from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String

from tf2_ros import TransformBroadcaster


ODOM_TOPIC = '/odom'
ODOM_PATH_LENGTH_TOPIC = '/odom/path_length'
IMU_RPY_TOPIC = '/imu/rpy'


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomTfPublisher(Node):
    """Fuse /odom distance with IMU heading and broadcast odom -> base_link."""

    def __init__(self) -> None:
        """Set up parameters, subscriptions and the broadcast timer."""
        super().__init__('odom_tf_publisher')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_odom_step', 0.5)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.max_odom_step = float(self.get_parameter('max_odom_step').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self._yaw_unwrapped = 0.0
        self._yaw_raw_prev = None
        self._yaw_offset = None
        self._odom_prev = None
        self._path_prev = None
        self._use_path_length = False

        self.br = TransformBroadcaster(self)

        self.create_subscription(String, IMU_RPY_TOPIC, self._imu_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(Float64, ODOM_PATH_LENGTH_TOPIC, self._path_cb, 10)

        # Broadcast on an independent timer rather than from _odom_cb:
        # servo_controller's encoder loop early-returns on a long dt or on its
        # first read, which would freeze the TF and make slam_toolbox's lookups
        # fail to extrapolate. tf2 also discards a second transform sharing a
        # stamp, so this must stay the only source of the edge.
        self.create_timer(1.0 / publish_rate, self._broadcast)

        self.get_logger().info(
            f'odom_tf_publisher up: {self.odom_frame} -> {self.base_frame}, '
            f'yaw_sign={self.yaw_sign:+.0f}, {publish_rate:.0f} Hz'
        )

    def _imu_cb(self, msg: String) -> None:
        """Track heading from the IMU, unwrapped and zeroed at startup."""
        try:
            yaw_deg = float(json.loads(msg.data)['yaw'])
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Bad /imu/rpy payload: {exc}')
            return

        raw = self.yaw_sign * math.radians(yaw_deg)

        # /imu/rpy is normalised to [-180, 180]; accumulate deltas so a +-180
        # crossing does not become a discontinuity in the transform.
        if self._yaw_raw_prev is None:
            self._yaw_unwrapped = raw
            self._yaw_offset = raw
        else:
            self._yaw_unwrapped += normalize_angle(raw - self._yaw_raw_prev)

        self._yaw_raw_prev = raw
        self.yaw = self._yaw_unwrapped - self._yaw_offset

    def _path_cb(self, msg: Float64) -> None:
        """Advance the pose by servo_controller's signed cumulative path length."""
        if not self._use_path_length:
            self._use_path_length = True
            self.get_logger().info('/odom/path_length seen; chord fallback disabled')

        path = float(msg.data)
        prev = self._path_prev
        self._path_prev = path

        if prev is None or self._yaw_offset is None:
            return

        self._advance(path - prev)

    def _odom_cb(self, msg: Odometry) -> None:
        """Recover distance from the pose chord, for a control_servo without path length."""
        if self._use_path_length:
            return

        p = msg.pose.pose.position
        prev = self._odom_prev
        self._odom_prev = (p.x, p.y)

        if prev is None or self._yaw_offset is None:
            return

        # The chord between consecutive poses recovers |v*dt| regardless of how
        # wrong servo_controller's own yaw was, but the sign has to come from the
        # twist -- which lags through zero, so a reversal is briefly wrong. That
        # is why path length is preferred whenever it is available.
        ds = math.hypot(p.x - prev[0], p.y - prev[1])
        if msg.twist.twist.linear.x < 0.0:
            ds = -ds

        self._advance(ds)

    def _advance(self, ds: float) -> None:
        """Move the pose ds metres along the current IMU heading."""
        if abs(ds) > self.max_odom_step:
            self.get_logger().warn(f'Ignoring {ds:.2f} m odom jump (publisher restart?)')
            return

        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

    def _broadcast(self) -> None:
        """Send the current pose as odom -> base_link, stamped now."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.br.sendTransform(t)


def main(args=None) -> None:
    """Run the odom -> base_link transform publisher."""
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
