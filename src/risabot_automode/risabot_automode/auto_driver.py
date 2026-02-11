#!/usr/bin/env python3
"""
Auto Driver Node — Competition Edition
Central brain of the RISA-bot. Implements a state machine that sequences
through the 9 competition challenges:

  START → 1.Obstruction → 2.Roundabout → 3.Tunnel → BoomGate →
  4.BoomGate → 5.Hill → 6.Bumper → 7.TrafficLight →
  (Lap 2 roundabout: boom gate closed → parking path)
  8.ParallelParking → 9.PerpendicularParking → FINISH

Subscribes to all sensor/module topics and selects the appropriate
cmd_vel source for each challenge phase.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSPresetProfiles
import serial
import struct
import time
import math
import threading
from enum import Enum


class ChallengeState(Enum):
    LANE_FOLLOW = 0          # Default: camera lane following
    OBSTRUCTION = 1          # Challenge 1: lateral obstruction avoidance
    ROUNDABOUT = 2           # Challenge 2: curved lane following
    TUNNEL = 3               # Challenge 3: LiDAR wall following
    BOOM_GATE_TUNNEL = 4     # Boom gate near tunnel
    BOOM_GATE_MAIN = 5       # Challenge 4: main boom gate
    HILL = 6                 # Challenge 5: lane follow (no special handling)
    BUMPER = 7               # Challenge 6: lane follow (no special handling)
    TRAFFIC_LIGHT = 8        # Challenge 7: stop on red/yellow
    PARALLEL_PARK = 9        # Challenge 8: parallel parking
    PERPENDICULAR_PARK = 10  # Challenge 9: perpendicular parking
    FINISHED = 11


class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        self.get_logger().info('Auto Driver Node Starting (Competition Mode)...')

        # ===== Publishers =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.parking_cmd_pub = self.create_publisher(String, '/parking_command', 10)

        # Continuous cmd_vel publisher at 50 Hz
        self.cmd_vel_timer = self.create_timer(0.02, self.publish_cmd_vel)

        # ===== State =====
        self.state = ChallengeState.LANE_FOLLOW
        self.in_auto_mode = False
        self.distance = 0.0
        self.last_time = time.time()

        # Obstacle / sensor flags
        self.lidar_obstacle = False
        self.camera_obstacle = False
        self.obstacle_active = False
        self.lane_error = 0.0

        # Module inputs
        self.traffic_light_state = 'unknown'
        self.boom_gate_open = True
        self.tunnel_detected = False
        self.tunnel_cmd = Twist()
        self.obstruction_active = False
        self.obstruction_cmd = Twist()
        self.parking_complete = False
        self.parking_cmd = Twist()
        self.signboard_detected = False

        # State transition tracking
        self.state_entry_time = time.time()
        self.state_entry_dist = 0.0

        # ===== Serial (motor board) =====
        try:
            self.ser = serial.Serial(
                '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                115200, timeout=1
            )
            self.get_logger().info('Serial port opened')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            self.ser = None

        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_reader)
            self.serial_thread.daemon = True
            self.serial_thread.start()

        # ===== Subscribers — existing =====
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
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # ===== Subscribers — new modules =====
        self.create_subscription(
            String, '/traffic_light_state', self.traffic_light_callback, 10
        )
        self.create_subscription(
            Bool, '/boom_gate_open', self.boom_gate_callback, 10
        )
        self.create_subscription(
            Bool, '/tunnel_detected', self.tunnel_detected_callback, 10
        )
        self.create_subscription(
            Twist, '/tunnel_cmd_vel', self.tunnel_cmd_callback, 10
        )
        self.create_subscription(
            Bool, '/obstruction_active', self.obstruction_active_callback, 10
        )
        self.create_subscription(
            Twist, '/obstruction_cmd_vel', self.obstruction_cmd_callback, 10
        )
        self.create_subscription(
            Bool, '/parking_complete', self.parking_complete_callback, 10
        )
        self.create_subscription(
            Twist, '/parking_cmd_vel', self.parking_cmd_callback, 10
        )
        self.create_subscription(
            Bool, '/parking_signboard_detected', self.signboard_callback, 10
        )

        # Manual state override (for testing)
        self.create_subscription(
            String, '/set_challenge', self.set_challenge_callback, 10
        )

        # Publishers for fused obstacle
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected_fused', 10)

        self.get_logger().info(f'State: {self.state.name}')

    # ========== Existing callbacks ==========
    def scan_callback(self, msg):
        pass  # Raw scan stored if needed

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

    # ========== New module callbacks ==========
    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data

    def boom_gate_callback(self, msg):
        self.boom_gate_open = msg.data

    def tunnel_detected_callback(self, msg):
        self.tunnel_detected = msg.data

    def tunnel_cmd_callback(self, msg):
        self.tunnel_cmd = msg

    def obstruction_active_callback(self, msg):
        self.obstruction_active = msg.data

    def obstruction_cmd_callback(self, msg):
        self.obstruction_cmd = msg

    def parking_complete_callback(self, msg):
        if msg.data:
            self.parking_complete = True

    def parking_cmd_callback(self, msg):
        self.parking_cmd = msg

    def signboard_callback(self, msg):
        self.signboard_detected = msg.data

    def set_challenge_callback(self, msg):
        """Manual state override: ros2 topic pub /set_challenge std_msgs/String 'data: TUNNEL'"""
        try:
            new_state = ChallengeState[msg.data.upper()]
            self._transition_to(new_state)
        except KeyError:
            self.get_logger().error(f'Unknown challenge: {msg.data}')

    # ========== State machine ==========
    def _transition_to(self, new_state):
        """Transition to a new challenge state."""
        self.get_logger().info(f'🔄 State: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_entry_time = time.time()
        self.state_entry_dist = self.distance
        self.parking_complete = False

    def _time_in_state(self):
        return time.time() - self.state_entry_time

    def _dist_in_state(self):
        return self.distance - self.state_entry_dist

    def publish_cmd_vel(self):
        """Main control loop — selects cmd_vel based on current challenge state."""
        cmd = Twist()

        if not self.in_auto_mode:
            # Manual mode — don't publish anything, let the controller handle /cmd_vel
            return

        # ---- State-specific behavior ----

        if self.state == ChallengeState.LANE_FOLLOW:
            if self.obstacle_active:
                pass  # Stop (cmd stays zero)
            elif self.obstruction_active:
                # Hand off to obstruction avoidance module
                cmd = self.obstruction_cmd
            else:
                # Normal lane following
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state == ChallengeState.OBSTRUCTION:
            if self.obstruction_active:
                cmd = self.obstruction_cmd
            else:
                # Obstruction cleared, resume lane following
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state == ChallengeState.ROUNDABOUT:
            # Lane following works in roundabout (curved lines)
            if not self.obstacle_active:
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state == ChallengeState.TUNNEL:
            if self.tunnel_detected:
                # Use wall follower output
                cmd = self.tunnel_cmd
            else:
                # Not in tunnel yet or exited — lane follow
                if not self.obstacle_active:
                    cmd.angular.z = -0.8 * self.lane_error

        elif self.state in (ChallengeState.BOOM_GATE_TUNNEL, ChallengeState.BOOM_GATE_MAIN):
            if not self.boom_gate_open:
                # Gate closed — stop and wait
                pass
            else:
                # Gate open — proceed with lane following
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state in (ChallengeState.HILL, ChallengeState.BUMPER):
            # Normal lane following — the robot just drives through
            if not self.obstacle_active:
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state == ChallengeState.TRAFFIC_LIGHT:
            if self.traffic_light_state in ('red', 'yellow'):
                pass  # Stop
            elif self.traffic_light_state == 'green':
                cmd.angular.z = -0.8 * self.lane_error

        elif self.state in (ChallengeState.PARALLEL_PARK, ChallengeState.PERPENDICULAR_PARK):
            if not self.parking_complete:
                cmd = self.parking_cmd
            # else: parking done, zero velocity (wait for state transition)

        elif self.state == ChallengeState.FINISHED:
            pass  # Stop

        self.cmd_vel_pub.publish(cmd)

        # Debug
        state_name = self.state.name
        t = self._time_in_state()
        d = self._dist_in_state()
        print(f"\r[AUTO] {state_name} | dist: {d:.2f}m | t: {t:.0f}s | lane_err: {self.lane_error:.2f} | obs: {self.obstacle_active}", end='', flush=True)

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

    # ========== Serial / Odometry (unchanged from original) ==========
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
