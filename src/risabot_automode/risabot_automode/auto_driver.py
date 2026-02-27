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

import time
from enum import Enum
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool, Float32, String

from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BOOM_GATE_TOPIC,
    DASH_STATE_TOPIC,
    LANE_ERROR_TOPIC,
    OBSTACLE_CAMERA_TOPIC,
    OBSTACLE_FUSED_TOPIC,
    OBSTACLE_LIDAR_TOPIC,
    ODOM_TOPIC,
    OBSTRUCTION_ACTIVE_TOPIC,
    OBSTRUCTION_CMD_TOPIC,
    PARKING_CMD_TOPIC,
    PARKING_COMPLETE_TOPIC,
    PARKING_SIGN_TOPIC,
    PARKING_STATUS_TOPIC,
    PARKING_VEL_TOPIC,
    SET_CHALLENGE_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_CMD_TOPIC,
    TUNNEL_DETECTED_TOPIC,
)


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
    """Central competition state machine for RISA-bot."""

    def __init__(self):
        super().__init__('auto_driver')
        self.get_logger().info('Auto Driver Node Starting (Competition Mode)...')

        # ===== Publishers =====
        self.cmd_vel_pub = self.create_publisher(Twist, AUTO_CMD_VEL_TOPIC, 10)
        self.parking_cmd_pub = self.create_publisher(String, PARKING_CMD_TOPIC, 10)
        self.dash_state_pub = self.create_publisher(String, DASH_STATE_TOPIC, 10)

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
        self.declare_parameter('forward_speed', 0.12)  # m/s base forward speed
        self.declare_parameter('stale_timeout', 3.0)   # seconds before treating module data as stale
        self.declare_parameter('max_odom_speed', 1.0)  # ignore odom speed spikes beyond this

        # Distance threshold (only for determining if a lap is complete after passing traffic light)
        self.declare_parameter('dist_lap_complete', 1.0)
        self.declare_parameter('enable_subsumption_obstacle', False) 
        self.distance_past_light = 0.0
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

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
        self.parallel_done = False
        self.perpendicular_done = False
        self.parking_sequence_active = False
        self.parking_cmd = Twist()
        self.signboard_detected = False
        self._parking_parallel_sent = False
        self._parking_perp_sent = False

        # Transition tracking
        self.state_entry_time = time.monotonic()

        # Odometry integration (used minimally now)
        self.last_odom_time = time.monotonic()
        self.distance = 0.0
        self._last_tl_state = 'unknown'

        # ===== Subscribers — existing =====
        self.lidar_sub = self.create_subscription(
            Bool, OBSTACLE_LIDAR_TOPIC, self.lidar_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.camera_sub = self.create_subscription(
            Bool, OBSTACLE_CAMERA_TOPIC, self.camera_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.mode_sub = self.create_subscription(
            Bool, AUTO_MODE_TOPIC, self.mode_callback, 10
        )
        self.lane_sub = self.create_subscription(
            Float32, LANE_ERROR_TOPIC, self.lane_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # ===== Subscribers — new modules =====
        self.create_subscription(
            String, TRAFFIC_LIGHT_TOPIC, self.traffic_light_callback, 10
        )
        self.create_subscription(
            Bool, BOOM_GATE_TOPIC, self.boom_gate_callback, 10
        )
        self.create_subscription(
            Bool, TUNNEL_DETECTED_TOPIC, self.tunnel_detected_callback, 10
        )
        self.create_subscription(
            Twist, TUNNEL_CMD_TOPIC, self.tunnel_cmd_callback, 10
        )
        self.create_subscription(
            Bool, OBSTRUCTION_ACTIVE_TOPIC, self.obstruction_active_callback, 10
        )
        self.create_subscription(
            Twist, OBSTRUCTION_CMD_TOPIC, self.obstruction_cmd_callback, 10
        )
        self.create_subscription(
            Bool, PARKING_COMPLETE_TOPIC, self.parking_complete_callback, 10
        )
        self.create_subscription(
            Twist, PARKING_VEL_TOPIC, self.parking_cmd_callback, 10
        )
        self.create_subscription(
            Bool, PARKING_SIGN_TOPIC, self.signboard_callback, 10
        )
        self.create_subscription(
            String, PARKING_STATUS_TOPIC, self.parking_status_callback, 10
        )
        
        # Subscribe to Odometry (from servo_controller)
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10
        )


        # Manual state override (for testing)
        self.create_subscription(
            String, SET_CHALLENGE_TOPIC, self.set_challenge_callback, 10
        )

        # Publishers for fused obstacle
        self.obstacle_pub = self.create_publisher(Bool, OBSTACLE_FUSED_TOPIC, 10)

        self.get_logger().info(f'State: {self.state.name}')



    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'steering_gain': float(self.get_parameter('steering_gain').value),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'stale_timeout': float(self.get_parameter('stale_timeout').value),
            'dist_lap_complete': float(self.get_parameter('dist_lap_complete').value),
            'enable_subsumption_obstacle': bool(self.get_parameter('enable_subsumption_obstacle').value),
            'max_odom_speed': float(self.get_parameter('max_odom_speed').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def lidar_callback(self, msg: Bool) -> None:
        """Store LiDAR obstacle state and update fused obstacle output."""
        self.lidar_obstacle = msg.data
        self.update_combined_obstacle_state()

    def camera_callback(self, msg: Bool) -> None:
        """Store camera obstacle state and update fused obstacle output."""
        self.camera_obstacle = msg.data
        self.update_combined_obstacle_state()

    def lane_callback(self, msg: Float32) -> None:
        """Store latest lane error."""
        self.lane_error = msg.data

    def mode_callback(self, msg: Bool) -> None:
        """Store auto/manual mode flag."""
        self.in_auto_mode = msg.data

    # ========== New module callbacks ==========
    def traffic_light_callback(self, msg: String) -> None:
        """Store traffic light state with timestamp."""
        self.traffic_light_state = msg.data
        self.traffic_light_last_time = time.monotonic()

    def boom_gate_callback(self, msg: Bool) -> None:
        """Store boom gate state with timestamp."""
        self.boom_gate_open = msg.data
        self.boom_gate_last_time = time.monotonic()

    def tunnel_detected_callback(self, msg: Bool) -> None:
        """Store tunnel detection flag with timestamp."""
        self.tunnel_detected = msg.data
        self.tunnel_last_time = time.monotonic()

    def tunnel_cmd_callback(self, msg: Twist) -> None:
        """Store tunnel follower command."""
        self.tunnel_cmd = msg

    def obstruction_active_callback(self, msg: Bool) -> None:
        """Store obstruction avoidance active flag with timestamp."""
        self.obstruction_active = msg.data
        self.obstruction_last_time = time.monotonic()

    def obstruction_cmd_callback(self, msg: Twist) -> None:
        """Store obstruction avoidance command."""
        self.obstruction_cmd = msg

    def parking_complete_callback(self, msg: Bool) -> None:
        """Latch parking completion."""
        if msg.data:
            self.parking_complete = True
            # Backward-compatibility fallback when status topic is unavailable.
            if self._parking_perp_sent:
                self.perpendicular_done = True
            elif self._parking_parallel_sent:
                self.parallel_done = True

    def parking_cmd_callback(self, msg: Twist) -> None:
        """Store parking controller command."""
        self.parking_cmd = msg

    def signboard_callback(self, msg: Bool) -> None:
        """Store parking signboard detection flag."""
        self.signboard_detected = msg.data

    def parking_status_callback(self, msg: String) -> None:
        """Track parking stage completion."""
        status = msg.data.lower()
        if status == 'parallel_done':
            self.parallel_done = True
            self.parking_complete = True
        elif status == 'perpendicular_done':
            self.perpendicular_done = True
            self.parking_complete = True

    def _is_stale(self, last_time: float) -> bool:
        """Check if a module's data is stale (no updates for stale_timeout seconds)."""
        if last_time == 0.0:
            return True  # Never received
        return (time.monotonic() - last_time) > self._param_cache['stale_timeout']

    def set_challenge_callback(self, msg: String) -> None:
        """Manual state override: ros2 topic pub /set_challenge std_msgs/String 'data: TUNNEL'"""
        try:
            new_state = ChallengeState[msg.data.upper()]
            if self.state != new_state:
                self.get_logger().info(f'Manual override: {self.state.name} -> {new_state.name}')
                self.state = new_state
        except KeyError:
            self.get_logger().error(f'Unknown challenge: {msg.data}')

    # ========== State machine ==========
    def _publish_dash_state(self) -> None:
        """Publish state info for the dashboard (~5 Hz throttle)."""
        if not hasattr(self, '_dash_counter'):
            self._dash_counter = 0
        self._dash_counter += 1
        if self._dash_counter % 10 != 0:  # 50Hz timer / 10 = 5Hz
            return
        msg = String()
        msg.data = f'{self.state.name}|{self.current_lap}|{self.distance:.2f}|{self.stop_reason}'
        self.dash_state_pub.publish(msg)

    def _lane_follow_cmd(self) -> Twist:
        """Build a Twist for standard lane following."""
        cmd = Twist()
        cmd.linear.x = self._param_cache['forward_speed']
        # Positive error = lane on left. Positive Z = left turn.
        cmd.angular.z = self._param_cache['steering_gain'] * self.lane_error
        return cmd

    def publish_cmd_vel(self) -> None:
        """Main control loop: selects behavior and publishes cmd_vel."""
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

        # Lap Sequence Tracking (latch on green rising edge)
        if self.current_lap == 1:
            if self.traffic_light_state == 'green' and self._last_tl_state != 'green':
                self.lap_1_complete = True
                self.distance_past_light = 0.0
        self._last_tl_state = self.traffic_light_state
        
        if self.lap_1_complete and self.distance_past_light > self._param_cache['dist_lap_complete']:
            if self.current_lap == 1:
                self.get_logger().info('Lap 1 complete -> starting Lap 2')
                self.current_lap = 2
                self.lap_1_complete = False
                self._parking_parallel_sent = False
                self._parking_perp_sent = False
                self.parallel_done = False
                self.perpendicular_done = False
                self.parking_sequence_active = False

        # --- Priority Evaluation Engine ---
        
        # Priority 2: Terminal Constraints (Finished)
        if self.current_lap == 2 and self.perpendicular_done:
            target_state = ChallengeState.FINISHED
            self.stop_reason = 'COMPETITION FINISHED'
            self.parking_sequence_active = False

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
            cmd.linear.x = -self._param_cache['forward_speed'] * 0.8
            cmd.angular.z = 0.0

        # Priority 5: Contextual Overrides (Parking / Tunnel)
        elif self.current_lap == 2 and not self.perpendicular_done and (self.parking_sequence_active or self.signboard_detected):
            # Latch parking sequence once entered so temporary signboard loss does not abort the maneuver.
            self.parking_sequence_active = True
            cmd = self.parking_cmd
            if not self.parallel_done:
                target_state = ChallengeState.PARALLEL_PARK
                if not self._parking_parallel_sent:
                    self.parking_cmd_pub.publish(String(data='parallel'))
                    self._parking_parallel_sent = True
            else:
                target_state = ChallengeState.PERPENDICULAR_PARK
                if not self._parking_perp_sent:
                    self.parking_cmd_pub.publish(String(data='perpendicular'))
                    self._parking_perp_sent = True
            
        elif self.tunnel_detected:
            target_state = ChallengeState.TUNNEL
            cmd = self.tunnel_cmd

        # Priority 6: Default Action
        else:
            target_state = ChallengeState.LANE_FOLLOW
            cmd = self._lane_follow_cmd()

        # Update and publish
        if self.state != target_state:
            self.get_logger().info(f'Behavior switch: {self.state.name} -> {target_state.name}')
            self.state = target_state

        self.cmd_vel_pub.publish(cmd)
        self._publish_dash_state()

        # Debug
        self.get_logger().debug(f"Lap{self.current_lap} | {self.state.name} | err: {self.lane_error:.2f} | obs: {self.obstruction_active}")

    def update_combined_obstacle_state(self) -> None:
        """Fuse LiDAR and camera obstacle flags with optional subsumption."""
        # Allow disabling standard obstacle subsumption for testing/recording
        if not self._param_cache['enable_subsumption_obstacle']:
            new_obstacle_state = False
        else:
            new_obstacle_state = self.lidar_obstacle or self.camera_obstacle

        if new_obstacle_state != self.obstacle_active:
            self.obstacle_active = new_obstacle_state
            msg = Bool()
            msg.data = new_obstacle_state
            self.obstacle_pub.publish(msg)

    # ========== Odometry (from servo_controller) ==========

    def odom_callback(self, msg: Odometry) -> None:
        """Integrate distance from odometry for state transition thresholds."""
        now = time.monotonic()
        dt = min(now - self.last_odom_time, 0.1)  # Cap dt to prevent jumps
        self.last_odom_time = now

        v = msg.twist.twist.linear.x
        max_v = float(self._param_cache['max_odom_speed'])
        if abs(v) > max_v:
            self.get_logger().debug(f'Ignoring odom spike v={v:.2f} > max={max_v:.2f}')
            return
        d = v * dt
        self.distance += d
        if self.lap_1_complete:
            self.distance_past_light += max(0.0, d)

    # Serial reader removed (moved to servo_controller)

    def on_shutdown(self) -> None:
        pass


def main(args=None) -> None:
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
