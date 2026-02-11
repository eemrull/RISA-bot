#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import serial
import struct
import time
import math
import threading
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        self.get_logger().info('Auto Driver Node Starting...')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Continuous cmd_vel publisher
        self.cmd_vel_timer = self.create_timer(0.02, self.publish_cmd_vel)

        # Mode state
        self.in_auto_mode = False

        # State
        self.distance = 0.0
        self.last_time = time.time()
        self.lidar_obstacle = False
        self.camera_obstacle = False
        self.obstacle_active = False

        # Open serial port (optional — can be disabled if not needed)
        try:
            self.ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 115200, timeout=1)
            self.get_logger().info('Serial port opened')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            self.ser = None

        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_reader)
            self.serial_thread.daemon = True
            self.serial_thread.start()

        # Subscribers
        self.lidar_sub = self.create_subscription(
            Bool, '/obstacle_front', self.lidar_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.camera_sub = self.create_subscription(
            Bool, '/obstacle_detected_camera', self.camera_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.mode_sub = self.create_subscription(
            Bool, '/auto_mode', self.mode_callback, 10
        )
        self.lane_sub = self.create_subscription(
            Float32, '/lane_error', self.lane_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # Subscribe to raw LiDAR scan (for future use — optional)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # Publisher for fused obstacle signal (forwarded from inputs)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected_fused', 10)

        self.lane_error = 0.0

    def scan_callback(self, msg):
        """Store latest scan for potential future use."""
        angles = []
        distances = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max and not math.isnan(r) and not math.isinf(r)):
                continue
            angle = angle_min + i * angle_increment
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi
            angles.append(angle)
            distances.append(r)

        self.latest_scan = (angles, distances)

    def lidar_callback(self, msg):
        self.lidar_obstacle = msg.data
        self.update_combined_obstacle_state()

    def camera_callback(self, msg):
        self.camera_obstacle = msg.data
        self.update_combined_obstacle_state()

    def lane_callback(self, msg):
        self.lane_error = msg.data

    def mode_callback(self, msg):
        self.in_auto_mode = msg.data

    def publish_cmd_vel(self):
        cmd = Twist()

        # Only publish lane-following commands when in auto mode AND no obstacle
        if self.in_auto_mode and not self.obstacle_active:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8 * self.lane_error

        self.cmd_vel_pub.publish(cmd)

    def update_combined_obstacle_state(self):
        new_obstacle_state = self.lidar_obstacle or self.camera_obstacle

        if new_obstacle_state != self.obstacle_active:
            self.obstacle_active = new_obstacle_state
            msg = Bool()
            msg.data = new_obstacle_state
            self.obstacle_pub.publish(msg)

            if new_obstacle_state:
                self.get_logger().warn('🛑 Obstacle detected!')
            else:
                self.get_logger().info('✅ Obstacle cleared.')

    def parse(self, packet):
        try:
            v_mm_per_sec = struct.unpack('<h', packet[4:6])[0]
            v_raw = v_mm_per_sec / 1000.0
            if abs(v_raw) < 0.05:
                v_raw = 0.0

            if not self.obstacle_active:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                self.distance += v_raw * dt

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.twist.twist.linear.x = v_raw if not self.obstacle_active else 0.0
            self.odom_pub.publish(odom_msg)

            print(f"\r Distance: {self.distance:.3f} m | Speed: {v_raw:.3f} m/s", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

    def serial_reader(self):
        buffer = bytearray()
        while rclpy.ok() and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    while len(buffer) >= 4:
                        if buffer[0] == 0xFF and buffer[1] == 0xFB:
                            length = buffer[2]
                            if len(buffer) >= length:
                                packet = buffer[:length]
                                buffer = buffer[length:]
                                if len(packet) >= 4 and packet[3] == 0x0C:
                                    self.parse(packet)
                                continue
                            else:
                                break
                        else:
                            buffer = buffer[1:]
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    def on_shutdown(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
