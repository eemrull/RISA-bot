#!/usr/bin/env python3
"""
Republish /scan with a corrected ROS timestamp.

ydlidar_ros2_driver stamps scans from the vendor SDK's own clock rather than
node->now(). If that clock is monotonic-since-boot instead of epoch, every
slam_toolbox TF lookup at the scan stamp fails and no map is ever produced.

Only needed if `ros2 topic delay /scan` reports a large or negative delay.
Wired in by launching slam_test.launch.py with restamp:=true.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    """Copy /scan through, replacing the header stamp with the current ROS time."""

    def __init__(self) -> None:
        """Set up the sensor-QoS subscription and republisher."""
        super().__init__('scan_restamper')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_restamped')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)

        # Sensor QoS on both sides: the driver publishes BEST_EFFORT and
        # slam_toolbox subscribes with sensor-data QoS, so anything else here
        # silently breaks one end or the other.
        self.pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, in_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(f'scan_restamper: {in_topic} -> {out_topic}')

    def _cb(self, msg: LaserScan) -> None:
        """Restamp and republish a scan."""
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None) -> None:
    """Run the scan restamper."""
    rclpy.init(args=args)
    node = ScanRestamper()
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
