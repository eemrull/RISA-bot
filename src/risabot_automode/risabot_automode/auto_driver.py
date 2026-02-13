#!/usr/bin/env python3
"""
Auto Driver Node — Competition Edition
Central brain of the RISA-bot. Implements a state machine that sequences
through the competition challenges over 2 laps:

  Lap 1: START → Lane Follow → 1.Obstruction → 2.Roundabout (exit 1) →
         BoomGate1 (open) → 3.Tunnel → 4.BoomGate2 (random) →
         5.Hill → 6.Bumper → 7.TrafficLight → back to start

  Lap 2: Lane Follow → 1.Obstruction → 2.Roundabout (exit 2, gate closed) →
         8.ParallelParking → drive → 9.PerpendicularParking → FINISH

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
    BOOM_GATE_1 = 3          # Boom gate after roundabout exit 1 (open lap 1)
    TUNNEL = 4               # Challenge 3: LiDAR wall following (dark tunnel)
    BOOM_GATE_2 = 5          # Challenge 4: boom gate after tunnel (random)
    HILL = 6                 # Challenge 5: lane follow uphill
    BUMPER = 7               # Challenge 6: lane follow over bumps
    TRAFFIC_LIGHT = 8        # Challenge 7: stop on red/yellow, go on green
    PARALLEL_PARK = 9        # Challenge 8: parallel parking
    DRIVE_TO_PERP = 10       # Drive from parallel to perpendicular parking area
    PERPENDICULAR_PARK = 11  # Challenge 9: perpendicular parking
    FINISHED = 12


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
        self.current_lap = 1  # Tracks lap 1 vs lap 2

        # Obstacle / sensor flags
        self.lidar_obstacle = False
        self.camera_obstacle = False
        self.obstacle_active = False
        self.lane_error = 0.0

        # Tunable parameters
        self.declare_parameter('steering_gain', 0.5)
        self.declare_parameter('forward_speed', 0.15)  # m/s base forward speed
        self.declare_parameter('stale_timeout', 3.0)   # seconds before treating module data as stale

        # Transition distance thresholds (meters) — tune on the physical track
        self.declare_parameter('dist_obstruction_clear', 0.3)   # min distance after obstruction clears
        self.declare_parameter('dist_roundabout', 1.5)          # distance through roundabout
        self.declare_parameter('dist_boom_gate_1_pass', 0.5)    # distance after passing boom gate 1
        self.declare_parameter('dist_boom_gate_2_pass', 0.5)    # distance after passing boom gate 2
        self.declare_parameter('dist_hill', 1.0)                # distance over the hill
        self.declare_parameter('dist_bumper', 0.8)              # distance over bumpers
        self.declare_parameter('dist_traffic_light_pass', 0.5)  # distance after passing green light
        self.declare_parameter('dist_drive_to_perp', 1.0)       # distance from parallel to perp parking

        # Module inputs (with timestamps for stale detection)
        self.traffic_light_state = 'unknown'
        self.traffic_light_last_time = 0.0
        self.boom_gate_open = True
        self.boom_gate_last_time = 0.0
        self.tunnel_detected = False
        self.tunnel_last_time = 0.0
        self.tunnel_cmd = Twist()
        self.obstruction_active = False
        self.obstruction_last_time = 0.0
        self.obstruction_cmd = Twist()
        self.parking_complete = False
        self.parking_cmd = Twist()
        self.signboard_detected = False

        # Transition tracking
        self.state_entry_time = time.time()
        self.state_entry_dist = 0.0
        self.tunnel_was_active = False       # tracks if we entered the tunnel
        self.traffic_light_was_green = False # tracks if green was detected
        self.obstruction_was_active = False  # tracks if obstruction was engaged

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
        self.traffic_light_last_time = time.time()

    def boom_gate_callback(self, msg):
        self.boom_gate_open = msg.data
        self.boom_gate_last_time = time.time()

    def tunnel_detected_callback(self, msg):
        self.tunnel_detected = msg.data
        self.tunnel_last_time = time.time()

    def tunnel_cmd_callback(self, msg):
        self.tunnel_cmd = msg

    def obstruction_active_callback(self, msg):
        self.obstruction_active = msg.data
        self.obstruction_last_time = time.time()

    def obstruction_cmd_callback(self, msg):
        self.obstruction_cmd = msg

    def parking_complete_callback(self, msg):
        if msg.data:
            self.parking_complete = True

    def parking_cmd_callback(self, msg):
        self.parking_cmd = msg

    def signboard_callback(self, msg):
        self.signboard_detected = msg.data

    def _is_stale(self, last_time):
        """Check if a module's data is stale (no updates for stale_timeout seconds)."""
        if last_time == 0.0:
            return True  # Never received
        return (time.time() - last_time) > self.get_parameter('stale_timeout').value

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
        self.tunnel_was_active = False
        self.traffic_light_was_green = False
        self.obstruction_was_active = False

    def _time_in_state(self):
        return time.time() - self.state_entry_time

    def _dist_in_state(self):
        return self.distance - self.state_entry_dist

    def _lane_follow_cmd(self):
        """Build a Twist for standard lane following."""
        cmd = Twist()
        cmd.linear.x = self.get_parameter('forward_speed').value
        cmd.angular.z = -self.get_parameter('steering_gain').value * self.lane_error
        return cmd

    def _check_transitions(self):
        """Check if the current state should auto-advance based on sensors/distance."""
        dist = self._dist_in_state()
        t = self._time_in_state()

        if self.state == ChallengeState.LANE_FOLLOW:
            # Advance to OBSTRUCTION when obstruction module detects something
            if self.obstruction_active:
                self._transition_to(ChallengeState.OBSTRUCTION)

        elif self.state == ChallengeState.OBSTRUCTION:
            # Track that we engaged obstruction avoidance
            if self.obstruction_active:
                self.obstruction_was_active = True
            # After obstruction clears and we've moved past it
            if self.obstruction_was_active and not self.obstruction_active:
                if dist > self.get_parameter('dist_obstruction_clear').value:
                    self._transition_to(ChallengeState.ROUNDABOUT)

        elif self.state == ChallengeState.ROUNDABOUT:
            # After traveling through the roundabout, pick exit based on lap
            if dist > self.get_parameter('dist_roundabout').value:
                if self.current_lap == 1:
                    # Lap 1: exit 1 → boom gate 1 (open) → tunnel
                    self._transition_to(ChallengeState.BOOM_GATE_1)
                else:
                    # Lap 2: boom gate 1 is closed → exit 2 → parking
                    self._transition_to(ChallengeState.PARALLEL_PARK)
                    # Send parking command for parallel
                    self.parking_cmd_pub.publish(String(data='parallel'))

        elif self.state == ChallengeState.BOOM_GATE_1:
            # Lap 1: gate is always open, pass through
            if self.boom_gate_open and dist > self.get_parameter('dist_boom_gate_1_pass').value:
                self._transition_to(ChallengeState.TUNNEL)

        elif self.state == ChallengeState.TUNNEL:
            # Track if tunnel was entered
            if self.tunnel_detected:
                self.tunnel_was_active = True
            # Transition when tunnel exits (was active, now not)
            if self.tunnel_was_active and not self.tunnel_detected:
                self._transition_to(ChallengeState.BOOM_GATE_2)

        elif self.state == ChallengeState.BOOM_GATE_2:
            # Random gate: wait if closed, pass if open
            if self.boom_gate_open and dist > self.get_parameter('dist_boom_gate_2_pass').value:
                self._transition_to(ChallengeState.HILL)

        elif self.state == ChallengeState.HILL:
            if dist > self.get_parameter('dist_hill').value:
                self._transition_to(ChallengeState.BUMPER)

        elif self.state == ChallengeState.BUMPER:
            if dist > self.get_parameter('dist_bumper').value:
                self._transition_to(ChallengeState.TRAFFIC_LIGHT)

        elif self.state == ChallengeState.TRAFFIC_LIGHT:
            # Track if green was seen
            if self.traffic_light_state == 'green':
                self.traffic_light_was_green = True
            # After green and moved past the light
            if self.traffic_light_was_green and dist > self.get_parameter('dist_traffic_light_pass').value:
                # Lap complete — start lap 2
                self.current_lap = 2
                self.get_logger().info('\U0001f3c1 Lap 1 complete — starting Lap 2')
                self._transition_to(ChallengeState.LANE_FOLLOW)

        elif self.state == ChallengeState.PARALLEL_PARK:
            if self.parking_complete:
                self.get_logger().info('\U0001f17f\ufe0f Parallel parking done → driving to perpendicular area')
                self._transition_to(ChallengeState.DRIVE_TO_PERP)

        elif self.state == ChallengeState.DRIVE_TO_PERP:
            # Drive forward to perpendicular parking area
            if dist > self.get_parameter('dist_drive_to_perp').value:
                self._transition_to(ChallengeState.PERPENDICULAR_PARK)
                self.parking_cmd_pub.publish(String(data='perpendicular'))

        elif self.state == ChallengeState.PERPENDICULAR_PARK:
            if self.parking_complete:
                self._transition_to(ChallengeState.FINISHED)
                self.get_logger().info('\U0001f389 Competition FINISHED!')

    def publish_cmd_vel(self):
        """Main control loop — selects cmd_vel based on current challenge state."""
        cmd = Twist()

        if not self.in_auto_mode:
            return

        # Reset stale module data to safe defaults
        if self._is_stale(self.obstruction_last_time):
            self.obstruction_active = False
        if self._is_stale(self.tunnel_last_time):
            self.tunnel_detected = False
        if self._is_stale(self.traffic_light_last_time):
            self.traffic_light_state = 'unknown'
        if self._is_stale(self.boom_gate_last_time):
            self.boom_gate_open = True  # Fail-open: assume gate is open

        # Check for automatic state transitions
        self._check_transitions()

        # ---- State-specific behavior ----

        if self.state == ChallengeState.LANE_FOLLOW:
            if self.obstacle_active:
                pass  # Stop
            else:
                cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.OBSTRUCTION:
            if self.obstruction_active:
                cmd = self.obstruction_cmd
            else:
                cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.ROUNDABOUT:
            if not self.obstacle_active:
                cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.BOOM_GATE_1:
            # Lap 1: always open — lane follow through
            if self.boom_gate_open:
                cmd = self._lane_follow_cmd()
            # else: stop and wait (shouldn't happen on lap 1)

        elif self.state == ChallengeState.TUNNEL:
            if self.tunnel_detected:
                cmd = self.tunnel_cmd
            else:
                if not self.obstacle_active:
                    cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.BOOM_GATE_2:
            if not self.boom_gate_open:
                pass  # Gate closed — stop and wait
            else:
                cmd = self._lane_follow_cmd()

        elif self.state in (ChallengeState.HILL, ChallengeState.BUMPER):
            if not self.obstacle_active:
                cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.TRAFFIC_LIGHT:
            if self.traffic_light_state in ('red', 'yellow'):
                pass  # Stop
            elif self.traffic_light_state == 'green':
                cmd = self._lane_follow_cmd()
            else:
                # 'unknown' — proceed with caution
                cmd = self._lane_follow_cmd()
                cmd.linear.x *= 0.5

        elif self.state in (ChallengeState.PARALLEL_PARK, ChallengeState.PERPENDICULAR_PARK):
            if not self.parking_complete:
                cmd = self.parking_cmd

        elif self.state == ChallengeState.DRIVE_TO_PERP:
            # Lane follow from parallel parking to perpendicular parking area
            cmd = self._lane_follow_cmd()

        elif self.state == ChallengeState.FINISHED:
            pass  # Stop

        self.cmd_vel_pub.publish(cmd)

        # Debug
        state_name = self.state.name
        t = self._time_in_state()
        d = self._dist_in_state()
        print(f"\r[AUTO] Lap{self.current_lap} | {state_name} | dist: {d:.2f}m | t: {t:.0f}s | err: {self.lane_error:.2f} | obs: {self.obstacle_active}", end='', flush=True)

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
