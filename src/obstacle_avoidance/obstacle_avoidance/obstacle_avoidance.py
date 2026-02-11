#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import math
import numpy as np

sensor_data_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters
        self.min_obstacle_distance = 0.48

        # Publishers
        self.obstacle_all_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_front_pub = self.create_publisher(Bool, '/obstacle_front', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_data_qos
        )

        # State
        self.obstacle_active_any = False
        self.distance_buffer = []
        self.buffer_size = 5

        self.get_logger().info('Obstacle Avoidance Node Started (Precise Directional Detection)')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        min_front = float('inf')

        for i, r in enumerate(ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            angle = angle_min + i * angle_increment
            # Compensate for 90° clockwise mount
            angle += math.pi / 2

            # Normalize
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # ✅ Robot front = angle near 0 (±30° = ±π/6)
            if -math.pi/6 <= angle <= math.pi/6:
                min_front = min(min_front, r)

        # Publish front-only detection
        front_obstacle = min_front < self.min_obstacle_distance

        # Debug: log only when obstacle detected
        if front_obstacle:
            self.get_logger().warn(f'🛑 FRONT obstacle! min_front = {min_front:.2f} m')

        # Publish
        msg_front = Bool()
        msg_front.data = front_obstacle
        self.obstacle_front_pub.publish(msg_front)

        # Also publish any-obstacle (optional)
        any_obstacle = min_front < self.min_obstacle_distance  # simplify for now
        self.obstacle_all_pub.publish(Bool(data=any_obstacle))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
