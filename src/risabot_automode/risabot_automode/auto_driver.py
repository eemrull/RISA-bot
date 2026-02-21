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
from rclpy.qos import QoSPresetProfiles
import time
import math
from enum import Enum


class ChallengeState(Enum):
    MANUAL = 0               # Manual override active
    FINISHED = 1             # Competition complete
    TRAFFIC_LIGHT = 2        # Stopped at Red/Yellow light
    BOOM_GATE = 3            # Stopped at closed gate
    OBSTRUCTION = 4          # Active obstacle avoidance
    TUNNEL = 5               # Tunnel wall following
    PARALLEL_PARK = 6        # Parallel parking sequence
    PERPENDICULAR_PARK = 7   # Perpendicular parking sequence
    LANE_FOLLOW = 8          # Default driving mode
    REVERSE_ADJUST = 9       # Backing up from close obstacle


class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        self.get_logger().info('Auto Driver Node Starting (Competition Mode)...')

        # ===== Publishers =====
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.parking_cmd_pub = self.create_publisher(String, '/parking_command', 10)
        self.dash_state_pub = self.create_publisher(String, '/dashboard_state', 10)

        # Continuous cmd_vel publisher at 50 Hz
        self.cmd_vel_timer = self.create_timer(0.02, self.publish_cmd_vel)

        # ===== State =====
        self.state = ChallengeState.MANUAL
        self.in_auto_mode = False
        self.current_lap = 1  # Tracks lap 1 vs lap 2
        
        # High-level mission tracking flags
        self.lap_1_complete = False

        # Obstacle / sensor flags
        self.lidar_obstacle = False
        self.camera_obstacle = False
        self.obstacle_active = False
        self.stop_reason = ''
        self.lane_error = 0.0

        # Tunable parameters
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('forward_speed', 0.15)  # m/s base forward speed
        self.declare_parameter('stale_timeout', 3.0)   # seconds before treating module data as stale

        # Distance threshold (only for determining if a lap is complete after passing traffic light)
        self.declare_parameter('dist_lap_complete', 1.0)
        self.declare_parameter('enable_subsumption_obstacle', False) 
        self.distance_past_light = 0.0

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

        # Odometry integration (used minimally now)
        self.last_odom_time = time.time()
        self.distance = 0.0

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
        
        # Subscribe to Odometry (from servo_controller)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )


        # Manual state override (for testing)
        self.create_subscription(
            String, '/set_challenge', self.set_challenge_callback, 10
        )

        # Publishers for fused obstacle
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected_fused', 10)

        self.get_logger().info(f'State: {self.state.name}')



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
    def _publish_dash_state(self):
        """Publish state info for the dashboard (~5 Hz throttle)."""
        if not hasattr(self, '_dash_counter'):
            self._dash_counter = 0
        self._dash_counter += 1
        if self._dash_counter % 10 != 0:  # 50Hz timer / 10 = 5Hz
            return
        msg = String()
        msg.data = f'{self.state.name}|{self.current_lap}|{self.distance:.2f}|{self.stop_reason}'
        self.dash_state_pub.publish(msg)

    def _lane_follow_cmd(self):
        """Build a Twist for standard lane following."""
        cmd = Twist()
        cmd.linear.x = self.get_parameter('forward_speed').value
        # Positive error = lane on left. Positive Z = left turn.
        cmd.angular.z = self.get_parameter('steering_gain').value * self.lane_error
        return cmd

    def publish_cmd_vel(self):
        """Main control loop — Hybrid Priority Evaluator."""
        cmd = Twist()
        self.stop_reason = ''
        target_state = ChallengeState.LANE_FOLLOW

        # Priority 1: Manual Mode Override
        if not self.in_auto_mode:
            self.state = ChallengeState.MANUAL
            self.stop_reason = 'MANUAL MODE'
            self._publish_dash_state()
            return

        # Handle Stale Data Safety Fallbacks
        if self._is_stale(self.obstruction_last_time):
            self.obstruction_active = False
        if self._is_stale(self.tunnel_last_time):
            self.tunnel_detected = False
        if self._is_stale(self.traffic_light_last_time):
            self.traffic_light_state = 'unknown'
        if self._is_stale(self.boom_gate_last_time):
            self.boom_gate_open = True

        # Lap Sequence Tracking
        if self.traffic_light_state == 'green':
            self.lap_1_complete = True
            self.distance_past_light = 0.0
        
        if self.lap_1_complete and self.distance_past_light > self.get_parameter('dist_lap_complete').value:
            if self.current_lap == 1:
                self.get_logger().info('🏁 Lap 1 complete — starting Lap 2')
                self.current_lap = 2
                self.lap_1_complete = False

        # --- Priority Evaluation Engine ---
        
        # Priority 2: Terminal Constraints (Finished)
        if self.current_lap == 2 and self.parking_complete:
            target_state = ChallengeState.FINISHED
            self.stop_reason = 'COMPETITION FINISHED'

        # Priority 3: Hard Safety Stops (immediate halt required)
        elif self.traffic_light_state in ('red', 'yellow'):
            target_state = ChallengeState.TRAFFIC_LIGHT
            self.stop_reason = f'TRAFFIC LIGHT {self.traffic_light_state.upper()}'

        elif not self.boom_gate_open:
            target_state = ChallengeState.BOOM_GATE
            self.stop_reason = 'BOOM GATE CLOSED'

        # Priority 4: Active Maneuvers (Steering overriding normal driving)
        elif self.obstruction_active:
            target_state = ChallengeState.OBSTRUCTION
            cmd = self.obstruction_cmd

        # Priority 4.5: Front Obstacle Backup (Too close, but not actively avoiding)
        elif self.obstacle_active:
            target_state = ChallengeState.REVERSE_ADJUST
            self.stop_reason = 'TOO CLOSE TO OBSTACLE'
            cmd.linear.x = -self.get_parameter('forward_speed').value * 0.8
            cmd.angular.z = 0.0

        # Priority 5: Contextual Overrides (Parking / Tunnel)
        elif self.current_lap == 2 and self.signboard_detected and not self.parking_complete:
            # We detected a parking signboard on lap 2, assuming we have entered a parking area
            target_state = ChallengeState.PARALLEL_PARK
            cmd = self.parking_cmd
            # Send parking command for parallel (assume parallel for now based on lap 2 logic)
            self.parking_cmd_pub.publish(String(data='parallel'))
            
        elif self.tunnel_detected:
            target_state = ChallengeState.TUNNEL
            cmd = self.tunnel_cmd

        # Priority 6: Default Action
        else:
            target_state = ChallengeState.LANE_FOLLOW
            cmd = self._lane_follow_cmd()

        # Update and publish
        if self.state != target_state:
            self.get_logger().info(f'🔄 Behavior Switch: {self.state.name} → {target_state.name}')
            self.state = target_state

        self.cmd_vel_pub.publish(cmd)
        self._publish_dash_state()

        # Debug
        self.get_logger().debug(f"Lap{self.current_lap} | {self.state.name} | err: {self.lane_error:.2f} | obs: {self.obstruction_active}")

    def update_combined_obstacle_state(self):
        # Allow disabling standard obstacle subsumption for testing/recording
        if not self.get_parameter('enable_subsumption_obstacle').value:
            new_obstacle_state = False
        else:
            new_obstacle_state = self.lidar_obstacle or self.camera_obstacle

        if new_obstacle_state != self.obstacle_active:
            self.obstacle_active = new_obstacle_state
            msg = Bool()
            msg.data = new_obstacle_state
            self.obstacle_pub.publish(msg)

    # ========== Odometry (from servo_controller) ==========

    def odom_callback(self, msg):
        """Integrate distance from odometry for state transition thresholds."""
        now = time.time()
        dt = min(now - self.last_odom_time, 0.1)  # Cap dt to prevent jumps
        self.last_odom_time = now

        v = msg.twist.twist.linear.x
        d = v * dt
        self.distance += d
        if self.lap_1_complete:
            self.distance_past_light += d

    # Serial reader removed (moved to servo_controller)

    def on_shutdown(self):
        pass


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
