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

import json
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

from .loop_monitor import LoopMonitor
from .topics import (
    AUTO_CMD_VEL_RAW_TOPIC,
    AUTO_MODE_TOPIC,
    BOOM_GATE_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    DASH_STATE_TOPIC,
    LANE_ERROR_TOPIC,
    LOOP_STATS_TOPIC,
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
    EMERGENCY_STOP = 10      # External e-stop asserted
    ROUNDABOUT = 11          # Traversing roundabout (lane follow on Lap 1)


class AutoDriver(Node):
    """Central competition state machine for RISA-bot."""

    def __init__(self):
        super().__init__('auto_driver')
        self.get_logger().info('Auto Driver Node Starting (Competition Mode)...')

        # ===== Publishers =====
        self.cmd_vel_pub = self.create_publisher(Twist, AUTO_CMD_VEL_RAW_TOPIC, 10)
        self.parking_cmd_pub = self.create_publisher(String, PARKING_CMD_TOPIC, 10)
        self.dash_state_pub = self.create_publisher(String, DASH_STATE_TOPIC, 10)
        self.loop_stats_pub = self.create_publisher(String, LOOP_STATS_TOPIC, 10)

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
        self.cmd_safety_estop = False
        self.cmd_safety_last_time = 0.0

        # PID controller state
        self._pid_prev_error = 0.0
        self._pid_integral = 0.0
        self._pid_last_time = time.monotonic()

        # Tunable parameters
        self.declare_parameter('steering_gain', 1.0)   # legacy — kept for compatibility
        self.declare_parameter('forward_speed', 0.12)  # m/s maximum forward speed (straight)
        self.declare_parameter('stale_timeout', 3.0)   # seconds before treating module data as stale
        self.declare_parameter('max_odom_speed', 1.0)  # ignore odom speed spikes beyond this
        self.declare_parameter('min_state_dwell_sec', 0.25)
        self.declare_parameter('publish_loop_stats', True)

        # PID gains for steering angular.z
        self.declare_parameter('pid_kp', 1.2)   # Proportional — how hard to steer for a given error
        self.declare_parameter('pid_ki', 0.01)  # Integral — correct steady-state drift/bias
        self.declare_parameter('pid_kd', 0.15)  # Derivative — dampen oscillations / prevent overshoot
        self.declare_parameter('pid_integral_max', 0.3)  # Anti-windup clamp for integral term

        # Adaptive speed control
        # forward speed = forward_speed * max(min_turn_speed, 1 - speed_error_scale * |error|)
        self.declare_parameter('speed_error_scale', 1.5)  # how aggressively speed drops with error
        self.declare_parameter('min_turn_speed', 0.5)     # minimum speed multiplier in sharp turns (0.5 = half)

        # Distance threshold (only for determining if a lap is complete after passing traffic light)
        self.declare_parameter('dist_lap_complete', 1.0)
        self.declare_parameter('enable_subsumption_obstacle', False)

        # Challenge sequencing — time-based gating
        self.declare_parameter('t_post_obstacle_sec', 1.5)  # delay after obstacle clears before entering roundabout
        self.declare_parameter('t_roundabout_sec', 8.0)      # time to traverse roundabout arc
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

        # Challenge sequencing state
        self._obs_cleared_time: float = 0.0    # monotonic timestamp when obstruction cleared
        self._obs_was_active: bool = False      # edge detector for obstruction_active
        self._boom_gate_armed: bool = False     # True after roundabout exit
        self._tl_armed: bool = False            # True after tunnel exit

        # Odometry integration (used minimally now)
        self.last_odom_time = time.monotonic()
        self.distance = 0.0
        self._last_tl_state = 'unknown'
        self.last_cmd = Twist()
        self.loop_monitor = LoopMonitor('auto_driver_cmd', 50.0)

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
        self.create_subscription(
            String, CMD_SAFETY_STATUS_TOPIC, self.cmd_safety_status_callback, 10
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
        self.create_timer(1.0, self._publish_loop_stats)



    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'steering_gain': float(self.get_parameter('steering_gain').value),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'stale_timeout': float(self.get_parameter('stale_timeout').value),
            'dist_lap_complete': float(self.get_parameter('dist_lap_complete').value),
            'enable_subsumption_obstacle': bool(self.get_parameter('enable_subsumption_obstacle').value),
            'max_odom_speed': float(self.get_parameter('max_odom_speed').value),
            'min_state_dwell_sec': float(self.get_parameter('min_state_dwell_sec').value),
            'publish_loop_stats': bool(self.get_parameter('publish_loop_stats').value),
            # PID
            'pid_kp':            float(self.get_parameter('pid_kp').value),
            'pid_ki':            float(self.get_parameter('pid_ki').value),
            'pid_kd':            float(self.get_parameter('pid_kd').value),
            'pid_integral_max':  float(self.get_parameter('pid_integral_max').value),
            # Adaptive speed
            'speed_error_scale': float(self.get_parameter('speed_error_scale').value),
            'min_turn_speed':    float(self.get_parameter('min_turn_speed').value),
            # Challenge sequencing
            't_post_obstacle_sec': float(self.get_parameter('t_post_obstacle_sec').value),
            't_roundabout_sec':    float(self.get_parameter('t_roundabout_sec').value),
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

    def cmd_safety_status_callback(self, msg: String) -> None:
        """Track command safety status payload."""
        try:
            payload = json.loads(msg.data)
            self.cmd_safety_estop = bool(payload.get('estop', False))
            self.cmd_safety_last_time = time.monotonic()
        except Exception:
            pass

    def _publish_loop_stats(self) -> None:
        """Publish loop timing stats for diagnostics."""
        if not self._param_cache['publish_loop_stats']:
            return
        payload = self.loop_monitor.snapshot()
        payload['node'] = self.get_name()
        self.loop_stats_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

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
        """Build a Twist using a full PID controller + adaptive speed.

        PID terms:
          P — proportional to current error (immediate correction)
          I — proportional to accumulated error (corrects persistent drift)
          D — proportional to rate-of-change of error (dampens oscillation)

        Adaptive speed:
          Forward speed scales DOWN as |error| increases so the robot
          naturally slows for sharp turns and accelerates on straights.
        """
        now = time.monotonic()
        dt = now - self._pid_last_time
        self._pid_last_time = now

        # Guard against very large dt (e.g. first tick, state transition pause)
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02  # assume 50 Hz nominal

        error = self.lane_error

        # --- Integral term with anti-windup clamp ---
        self._pid_integral += error * dt
        i_max = self._param_cache['pid_integral_max']
        self._pid_integral = max(-i_max, min(i_max, self._pid_integral))

        # --- Derivative term ---
        derivative = (error - self._pid_prev_error) / dt
        self._pid_prev_error = error

        # --- PID output → steering ---
        kp = self._param_cache['pid_kp']
        ki = self._param_cache['pid_ki']
        kd = self._param_cache['pid_kd']
        angular_z = kp * error + ki * self._pid_integral + kd * derivative
        # Hard clamp so we never command an impossible turn rate
        angular_z = max(-2.0, min(2.0, angular_z))

        # --- Adaptive speed: slower in turns, faster on straights ---
        scale = self._param_cache['speed_error_scale']
        min_spd = self._param_cache['min_turn_speed']
        speed_mult = max(min_spd, 1.0 - scale * abs(error))
        linear_x = self._param_cache['forward_speed'] * speed_mult

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        return cmd

    def publish_cmd_vel(self) -> None:
        """Main control loop: selects behavior and publishes cmd_vel."""
        self.loop_monitor.tick()
        cmd = Twist()
        self.stop_reason = ''
        target_state = ChallengeState.LANE_FOLLOW

        # Priority 1: Manual Mode Override
        if not self.in_auto_mode:
            if self.state != ChallengeState.MANUAL:
                self.state_entry_time = time.monotonic()
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
                # Reset sequencing flags for Lap 2
                self._boom_gate_armed = False
                self._tl_armed = False
                self._obs_cleared_time = 0.0
                self._obs_was_active = False

        # ── Obstruction edge detection (runs every tick) ──
        if self.obstruction_active:
            self._obs_was_active = True
        elif self._obs_was_active:
            # Falling edge: obstacle just cleared
            self._obs_cleared_time = time.monotonic()
            self._obs_was_active = False
            self.get_logger().info('Obstruction cleared → roundabout countdown started')

        # --- Priority Evaluation Engine (Sequenced) ---

        # Priority 1: Terminal (Finished)
        if self.current_lap == 2 and self.perpendicular_done:
            target_state = ChallengeState.FINISHED
            self.stop_reason = 'COMPETITION FINISHED'
            self.parking_sequence_active = False

        # Priority 2: Hard Safety (E-Stop)
        elif self.cmd_safety_estop:
            target_state = ChallengeState.EMERGENCY_STOP
            self.stop_reason = 'E-STOP ACTIVE'

        # Priority 3: Obstruction — Challenge 1 (reactive)
        elif self.obstruction_active:
            target_state = ChallengeState.OBSTRUCTION
            cmd = self.obstruction_cmd

        # Priority 3.5: Front obstacle backup (unchanged)
        elif self.obstacle_active:
            target_state = ChallengeState.REVERSE_ADJUST
            self.stop_reason = 'TOO CLOSE TO OBSTACLE'
            cmd.linear.x = -self._param_cache['forward_speed'] * 0.8
            cmd.angular.z = 0.0

        # Priority 4: Roundabout — Challenge 2 (time-gated)
        elif (self._obs_cleared_time > 0
              and not self._boom_gate_armed
              and (time.monotonic() - self._obs_cleared_time) >= self._param_cache['t_post_obstacle_sec']):
            target_state = ChallengeState.ROUNDABOUT
            cmd = self._lane_follow_cmd()  # Roundabout has painted lane lines
            # Check exit: dwell time expired
            if self.state == ChallengeState.ROUNDABOUT:
                time_in_roundabout = time.monotonic() - self.state_entry_time
                if time_in_roundabout >= self._param_cache['t_roundabout_sec']:
                    target_state = ChallengeState.LANE_FOLLOW
                    cmd = self._lane_follow_cmd()
                    self._boom_gate_armed = True
                    self._obs_cleared_time = 0.0  # prevent re-entry
                    self.get_logger().info('Roundabout complete → boom gate armed')

        # Priority 5: Parking (Lap 2 only)
        elif self.current_lap == 2 and not self.perpendicular_done and (self.parking_sequence_active or self.signboard_detected):
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

        # Priority 6: Tunnel — Challenge 3 (reactive)
        elif self.tunnel_detected:
            target_state = ChallengeState.TUNNEL
            cmd = self.tunnel_cmd

        # # Priority 7: Boom Gate — Challenge 4 (GATED: only after roundabout)
        # # TODO: Enable when boom gate is on the test field
        # elif not self.boom_gate_open and self._boom_gate_armed:
        #     target_state = ChallengeState.BOOM_GATE
        #     self.stop_reason = 'BOOM GATE CLOSED'

        # # Priority 8: Traffic Light — Challenge 5 (GATED: only after tunnel)
        # # TODO: Enable when traffic light is on the test field
        # elif self.traffic_light_state in ('red', 'yellow') and self._tl_armed:
        #     target_state = ChallengeState.TRAFFIC_LIGHT
        #     self.stop_reason = f'TRAFFIC LIGHT {self.traffic_light_state.upper()}'

        # Priority 9: Default — Lane Follow
        else:
            target_state = ChallengeState.LANE_FOLLOW
            cmd = self._lane_follow_cmd()

        # ── Arm traffic light on tunnel exit (edge detect) ──
        if self.state == ChallengeState.TUNNEL and target_state != ChallengeState.TUNNEL:
            self._tl_armed = True
            self.get_logger().info('Tunnel exited → traffic light armed')

        # Update and publish
        if self.state != target_state:
            now = time.monotonic()
            dwell = now - self.state_entry_time
            min_dwell = float(self._param_cache['min_state_dwell_sec'])
            immediate_states = {
                ChallengeState.MANUAL,
                ChallengeState.FINISHED,
                ChallengeState.TRAFFIC_LIGHT,
                ChallengeState.BOOM_GATE,
                ChallengeState.REVERSE_ADJUST,
                ChallengeState.EMERGENCY_STOP,
            }
            allow_switch = (
                target_state in immediate_states
                or self.state in immediate_states
                or dwell >= min_dwell
            )
            if allow_switch:
                self.get_logger().info(f'Behavior switch: {self.state.name} -> {target_state.name}')
                self.state = target_state
                self.state_entry_time = now
            else:
                target_state = self.state
                cmd = self.last_cmd

        self.cmd_vel_pub.publish(cmd)
        self.last_cmd = cmd
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
