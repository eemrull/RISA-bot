#!/usr/bin/env python3
"""
Servo Controller V9 — Competition Ready
=============================================================================
Hardware interface between joystick, auto_driver, and Rosmaster motor board.
Handles mode toggling, manual driving, challenge cycling, and safety watchdog.

Features:
  - Joy watchdog: auto-stops robot if controller disconnects (no /joy for >0.5s)
  - Manual driving via set_motor(PWM) + set_pwm_servo(4, angle)
  - Auto mode: forwards /cmd_vel and /cmd_vel_auto to hardware
  - LB/RB challenge cycling via /set_challenge topic
  - D-Pad gear shifting with 4 speed levels

Controls:
  Left Stick Y (Axis 1):     Throttle (PWM ±255)
  Right Stick X (Axis 2):    Steering (Servo 4 Angle 40–140)
  D-Pad UP/DOWN (Axis 7):    Speed Limiter [25, 40, 60, 100]%
  Start (Btn 11) / Y (Btn 4): Toggle Auto/Manual
  LB (Btn 6):                Previous Challenge State
  RB (Btn 7):                Next Challenge State
"""

import json
import math
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from Rosmaster_Lib import Rosmaster
from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BASE_FRAME,
    CMD_VEL_TOPIC,
    DASH_CTRL_TOPIC,
    JOY_TOPIC,
    LOOP_STATS_TOPIC,
    ODOM_FRAME,
    ODOM_TOPIC,
    SET_CHALLENGE_TOPIC,
)

# --- Defaults ---
DEFAULT_SERVO_STEER_ID = 4
DEFAULT_SERVO_CENTER = 90
DEFAULT_SERVO_RANGE = 50   # 90 +/- 50 = [40, 140]

CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]


class LoopMonitor:
    """Tracks loop timing statistics."""

    def __init__(self, loop_name: str, target_hz: float, overrun_ratio: float = 1.5) -> None:
        self.loop_name = loop_name
        self.target_hz = float(target_hz) if target_hz > 0 else 1.0
        self.expected_dt = 1.0 / self.target_hz
        self.overrun_dt = self.expected_dt * float(overrun_ratio)
        self.last_t = 0.0
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.sum_dt = 0.0
        self.max_dt = 0.0
        self.overruns = 0

    def tick(self) -> None:
        now = time.monotonic()
        if self.last_t > 0.0:
            dt = now - self.last_t
            self.count += 1
            self.sum_dt += dt
            if dt > self.max_dt:
                self.max_dt = dt
            if dt > self.overrun_dt:
                self.overruns += 1
        self.last_t = now

    def snapshot(self) -> Dict[str, float]:
        avg_dt = (self.sum_dt / self.count) if self.count > 0 else 0.0
        avg_hz = (1.0 / avg_dt) if avg_dt > 0 else 0.0
        data = {
            'loop': self.loop_name,
            'target_hz': round(self.target_hz, 3),
            'avg_hz': round(avg_hz, 3),
            'max_dt_ms': round(self.max_dt * 1000.0, 3),
            'overruns': int(self.overruns),
            'samples': int(self.count),
        }
        self.reset()
        return data

class ServoControllerV9(Node):
    """Interface between joystick, auto driver, and Rosmaster motor board."""

    def __init__(self):
        super().__init__('servo_controller')

        # --- Parameters ---
        self.declare_parameter('servo_steer_id', DEFAULT_SERVO_STEER_ID)
        self.declare_parameter('servo_center', DEFAULT_SERVO_CENTER)
        self.declare_parameter('servo_range', DEFAULT_SERVO_RANGE)
        self.declare_parameter('speed_levels', [15, 25, 40, 60, 100])
        self.declare_parameter('default_speed_index', 1)
        self.declare_parameter('joy_timeout', 0.8)
        self.declare_parameter('auto_cmd_timeout', 0.4)
        self.declare_parameter('unlock_requires_neutral', True)
        self.declare_parameter('unlock_neutral_threshold', 0.15)
        self.declare_parameter('hw_heartbeat_sec', 1.0)
        self.declare_parameter('hw_fail_limit', 5)
        self.declare_parameter('ticks_per_meter', 1050.0)
        self.declare_parameter('odom_distance_scale', 1.0)
        self.declare_parameter('odom_yaw_scale', 1.0)
        self.declare_parameter('encoder_jump_threshold', 800.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 6.0)
        self.declare_parameter('odom_reverse_polarity', False)
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('steering_max_deg', 50.0)
        self.declare_parameter('odom_vel_alpha', 0.3)
        self.declare_parameter('odom_velocity_deadband', 0.02)
        self.declare_parameter('publish_loop_stats', True)
        self.declare_parameter('odom_frame_id', ODOM_FRAME)
        self.declare_parameter('base_frame_id', BASE_FRAME)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.servo_steer_id = int(self._param_cache['servo_steer_id'])
        self.servo_center = int(self._param_cache['servo_center'])
        self.servo_range = int(self._param_cache['servo_range'])

        # Connect to Hardware
        self.bot = None
        try:
            self.bot = Rosmaster()
            self.bot.create_receive_threading()
            
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(self.servo_steer_id, self.servo_center)
            self.get_logger().info("✅ Rosmaster Connected (V9 Competition)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Rosmaster: {e}")
            exit(1)

        # Publishers
        self.dash_pub = self.create_publisher(String, DASH_CTRL_TOPIC, 10)
        self.auto_mode_pub = self.create_publisher(Bool, AUTO_MODE_TOPIC, 10)
        self.challenge_pub = self.create_publisher(String, SET_CHALLENGE_TOPIC, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.loop_stats_pub = self.create_publisher(String, LOOP_STATS_TOPIC, 10)

        # Subscribers
        self.create_subscription(Joy, JOY_TOPIC, self.joy_callback, 10)
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, self.cmd_vel_auto_callback, 10)

        # State
        self.manual_mode = True
        self.challenge_index = 0
        self.speed_levels = list(self._param_cache['speed_levels'])
        if not self.speed_levels:
            self.speed_levels = [25]
        default_idx = int(self._param_cache['default_speed_index'])
        self.speed_idx = max(0, min(default_idx, len(self.speed_levels) - 1))
        self.current_speed_limit = self.speed_levels[self.speed_idx]

        self.target_motor_val = 0
        self.target_servo_val = self.servo_center
        # Hardware continuous update loop (10 Hz)
        self.create_timer(0.1, self._hardware_update_loop)

        # Odometry Odometry properties
        # Ackermann kinematics parameters for RISA-Bot 
        # Ticks to meters needs calibration based on exact gear ratio and wheel diameter
        # Using a default scaling factor for now
        self.wheel_base = float(self._param_cache['wheel_base'])
        self.ticks_per_meter = float(self._param_cache['ticks_per_meter'])

        self.last_odom_time = time.monotonic()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self._filtered_linear = 0.0
        self._filtered_angular = 0.0
        self._encoder_glitch_count = 0
        self._last_glitch_warn_t = 0.0
        self.last_encoder_ticks = [0, 0, 0, 0] # FL, FR, RL, RR
        self.encoder_first_read = True
        self.encoder_loop_monitor = LoopMonitor('servo_encoder', 20.0)
        self.hw_loop_monitor = LoopMonitor('servo_hw', 10.0)

        # Fast encoder loop (20 Hz)
        self.create_timer(0.05, self._encoder_read_loop)

        # Debounce
        self.prev_buttons = [0] * 15
        self.prev_axes = [0.0] * 8

        # Safety: Joy watchdog & Ghost Input Protection
        self.joy_alive = False          # True once first /joy received
        self.joy_unlocked = False       # True once input changes from initial ghost state
        self.initial_joy_axes = None
        self.initial_joy_buttons = None
        
        self.last_joy_time = 0.0        # timestamp of last /joy message (monotonic)
        self.joy_timeout = float(self._param_cache['joy_timeout'])
        self.joy_lost_reported = False   # avoid spamming log
        self.create_timer(0.3, self._joy_watchdog)
        self.create_timer(1.0, self._publish_loop_stats)
        self.hw_error_count = 0
        self.hw_error_tripped = False
        self.awaiting_neutral = False
        self.last_auto_cmd_time = 0.0
        self.auto_cmd_stale_reported = False

        self.get_logger().info("🎮 V9 Ready: Right Stick X = Steer | LB/RB = Challenges")
        self._update_dash()

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        self._param_cache = {
            'servo_steer_id': int(self.get_parameter('servo_steer_id').value),
            'servo_center': int(self.get_parameter('servo_center').value),
            'servo_range': int(self.get_parameter('servo_range').value),
            'speed_levels': list(self.get_parameter('speed_levels').value),
            'default_speed_index': int(self.get_parameter('default_speed_index').value),
            'joy_timeout': float(self.get_parameter('joy_timeout').value),
            'auto_cmd_timeout': float(self.get_parameter('auto_cmd_timeout').value),
            'unlock_requires_neutral': bool(self.get_parameter('unlock_requires_neutral').value),
            'unlock_neutral_threshold': float(self.get_parameter('unlock_neutral_threshold').value),
            'hw_heartbeat_sec': float(self.get_parameter('hw_heartbeat_sec').value),
            'hw_fail_limit': int(self.get_parameter('hw_fail_limit').value),
            'ticks_per_meter': float(self.get_parameter('ticks_per_meter').value),
            'odom_distance_scale': float(self.get_parameter('odom_distance_scale').value),
            'odom_yaw_scale': float(self.get_parameter('odom_yaw_scale').value),
            'encoder_jump_threshold': float(self.get_parameter('encoder_jump_threshold').value),
            'max_linear_velocity': float(self.get_parameter('max_linear_velocity').value),
            'max_angular_velocity': float(self.get_parameter('max_angular_velocity').value),
            'odom_reverse_polarity': bool(self.get_parameter('odom_reverse_polarity').value),
            'wheel_base': float(self.get_parameter('wheel_base').value),
            'steering_max_deg': float(self.get_parameter('steering_max_deg').value),
            'odom_vel_alpha': float(self.get_parameter('odom_vel_alpha').value),
            'odom_velocity_deadband': float(self.get_parameter('odom_velocity_deadband').value),
            'publish_loop_stats': bool(self.get_parameter('publish_loop_stats').value),
            'odom_frame_id': str(self.get_parameter('odom_frame_id').value),
            'base_frame_id': str(self.get_parameter('base_frame_id').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                if p.name == 'joy_timeout':
                    self.joy_timeout = float(p.value)
                elif p.name == 'unlock_requires_neutral':
                    self.awaiting_neutral = bool(p.value)
                elif p.name == 'servo_steer_id':
                    self.servo_steer_id = int(p.value)
                elif p.name == 'servo_center':
                    self.servo_center = int(p.value)
                elif p.name == 'servo_range':
                    self.servo_range = int(p.value)
                elif p.name == 'speed_levels':
                    self.speed_levels = list(p.value)
                    if not self.speed_levels:
                        self.speed_levels = [25]
                    self.speed_idx = max(0, min(self.speed_idx, len(self.speed_levels) - 1))
                    self.current_speed_limit = self.speed_levels[self.speed_idx]
                elif p.name == 'ticks_per_meter':
                    self.ticks_per_meter = float(p.value)
                elif p.name == 'wheel_base':
                    self.wheel_base = float(p.value)
        return SetParametersResult(successful=True)

    def _update_dash(self) -> None:
        """Publish controller state to dashboard (speed|index|state)."""
        state_str = CHALLENGE_STATES[self.challenge_index]
        msg = String()
        msg.data = f"{self.current_speed_limit}|{self.challenge_index}|{state_str}"
        self.dash_pub.publish(msg)

    def _publish_loop_stats(self) -> None:
        """Publish loop timing diagnostics."""
        if not bool(self._param_cache['publish_loop_stats']):
            return
        for monitor in (self.encoder_loop_monitor, self.hw_loop_monitor):
            payload = monitor.snapshot()
            payload['node'] = self.get_name()
            self.loop_stats_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def _joy_watchdog(self) -> None:
        """Stop robot if controller disconnects (no /joy for >joy_timeout)."""
        if not self.joy_alive:
            return  # Haven't received first joy yet, nothing to do
        elapsed = time.monotonic() - self.last_joy_time
        if elapsed > self.joy_timeout:
            if not self.joy_lost_reported:
                self.get_logger().warn(
                    f"⚠️ Controller lost (no /joy for {elapsed:.1f}s) — STOPPING ROBOT")
                self.joy_lost_reported = True
                self.joy_unlocked = False  # Lock to ignore phantom inputs on next receive
                # Force back to manual mode so auto_driver stops publishing
                if not self.manual_mode:
                    self.manual_mode = True
                    self.auto_mode_pub.publish(Bool(data=False))
                    self.get_logger().warn("⚠️ Forced MANUAL mode (controller lost)")
            # Always stop hardware (regardless of mode)
            self.stop_robot()
            # Publish zero cmd_vel to stop dashboard odometry
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

    def joy_callback(self, msg: Joy) -> None:
        """Handle joystick input for mode toggle, driving, and state cycling."""
        # Mark controller as alive
        self.last_joy_time = time.monotonic()
        
        if not self.joy_alive:
            self.joy_alive = True
            self.initial_joy_axes = list(msg.axes)
            self.initial_joy_buttons = list(msg.buttons)
            self.get_logger().info("🎮 Controller receiver connected (Awaiting sync...)")
            
        if self.joy_lost_reported:
            self.get_logger().info("🎮 Controller module synced. Press any BUTTON to unlock...")
            self.joy_lost_reported = False

        # Ghost state protection: require an explicit BUTTON PRESS to unlock.
        # Axis changes alone are not safe because controllers often have drift
        # or non-zero default axis values (e.g. axis 1 = 1.0) on power-on.
        if not self.joy_unlocked:
            any_button_pressed = any(b == 1 for b in msg.buttons)
            if any_button_pressed:
                self.joy_unlocked = True
                self.awaiting_neutral = bool(self._param_cache['unlock_requires_neutral'])
                self.get_logger().info("Controller unlocked (Button press detected)")
                # Consume unlock frame: do not process toggles/driving in same packet.
                self.prev_buttons = list(msg.buttons)
                self.prev_axes = list(msg.axes)
                self.stop_robot()
                return
            else:
                return  # Return early, ignore all input until a button is pressed

        if self.awaiting_neutral:
            neutral_thr = float(self._param_cache['unlock_neutral_threshold'])
            raw_throttle = msg.axes[1] if 1 < len(msg.axes) else 0.0
            raw_steer = msg.axes[2] if 2 < len(msg.axes) else 0.0
            if abs(raw_throttle) <= neutral_thr and abs(raw_steer) <= neutral_thr:
                self.awaiting_neutral = False
                self.get_logger().info('Controller neutral detected, manual drive enabled')
            else:
                self.stop_robot()
                self.prev_buttons = list(msg.buttons)
                self.prev_axes = list(msg.axes)
                return

        # Helpers
        def btn(idx): return msg.buttons[idx] if idx < len(msg.buttons) else 0
        def axis(idx): return msg.axes[idx] if idx < len(msg.axes) else 0.0
        def rose(idx): # Rising edge
            return btn(idx) == 1 and (idx >= len(self.prev_buttons) or self.prev_buttons[idx] == 0)

        # 1. Toggle Mode (Start=11, Y=4)
        if rose(11) or rose(4): 
            self.manual_mode = not self.manual_mode
            self.auto_mode_pub.publish(Bool(data=not self.manual_mode))
            self.get_logger().info(f"Mode: {'MANUAL' if self.manual_mode else 'AUTO'}")
            if self.manual_mode:
                self.stop_robot()
            else:
                self.last_auto_cmd_time = time.monotonic()
                self.auto_cmd_stale_reported = False
            self._update_dash()

        # 2. Challenge Cycling (LB=6, RB=7)
        if rose(7): # RB -> Next
            self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
            self.challenge_pub.publish(String(data=CHALLENGE_STATES[self.challenge_index]))
            self.get_logger().info(f"RB Pressed -> Challenge: {CHALLENGE_STATES[self.challenge_index]}")
            self._update_dash()
        
        if rose(6): # LB -> Prev
            self.challenge_index = (self.challenge_index - 1) % len(CHALLENGE_STATES)
            self.challenge_pub.publish(String(data=CHALLENGE_STATES[self.challenge_index]))
            self.get_logger().info(f"LB Pressed -> Challenge: {CHALLENGE_STATES[self.challenge_index]}")
            self._update_dash()

        # 3. Speed Control (D-Pad UP/DOWN - axis 7)
        dpad = axis(7)
        prev_dpad = self.prev_axes[7] if 7 < len(self.prev_axes) else 0.0
        if dpad > 0.5 and prev_dpad <= 0.5:
            self.speed_idx = min(len(self.speed_levels)-1, self.speed_idx + 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()
        elif dpad < -0.5 and prev_dpad >= -0.5:
            self.speed_idx = max(0, self.speed_idx - 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()

        # 4. MANUAL DRIVING
        if self.manual_mode and self.joy_alive:
            # Throttle: Left Stick Y (Axis 1)
            throttle_raw = axis(1)
            
            # Steering: Right Stick X (Axis 2)
            # Ghost state protection handles startup noise from triggers
            steer_raw = axis(2)
            
            # Deadzone
            if abs(throttle_raw) < 0.1: throttle_raw = 0.0
            if abs(steer_raw) < 0.1: steer_raw = 0.0

            # Drive (PWM)
            motor_pwm = int(throttle_raw * self.current_speed_limit * 2.55)
            
            # Steer (Servo 4)
            # Center (90) - (Input * Range)
            # Left (+1) -> 90 - 50 = 40 (Right?)
            # Wait, Main logic: target_angle -= joy_val * SWING_LEFT
            # joy > 0 (Left Stick?) -> Angle Decrease
            # If Angle Decrease = Right Turn? Or Left Turn?
            # Standard: Left Stick -> Left Turn.
            # If main branch says: joy > 0 -> Angle Decrease...
            # And user says "push left and right sometimes steers correctly".
            # I will trust the main branch math: 90 - (joy * 50)
            steer_angle = int(self.servo_center - (steer_raw * self.servo_range))
            steer_angle = max(self.servo_center - self.servo_range, min(self.servo_center + self.servo_range, steer_angle))

            self.apply_hardware(motor_pwm, steer_angle)

            # Publish to /cmd_vel so dashboard can track manual velocity for odom
            cmd = Twist()
            cmd.linear.x = throttle_raw * self.current_speed_limit / 100.0  # approximate m/s
            cmd.angular.z = steer_raw  # normalized steering
            self.cmd_vel_pub.publish(cmd)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

    def cmd_vel_auto_callback(self, msg: Twist) -> None:
        """Forward auto commands to hardware when in auto mode."""
        self.last_auto_cmd_time = time.monotonic()
        self.auto_cmd_stale_reported = False
        if not self.manual_mode:
            self.process_twist(msg)

    def process_twist(self, msg: Twist) -> None:
        """Convert Twist message to hardware PWM + servo angle."""
        # Auto Mode Driving
        pwm_val = int(msg.linear.x * 255.0)
        
        # Steering
        angle_offset = msg.angular.z * float(self.servo_range)
        steer_angle = int(self.servo_center - angle_offset)
        
        # Clamp
        pwm_val = max(-255, min(255, pwm_val))
        steer_angle = max(self.servo_center - self.servo_range, min(self.servo_center + self.servo_range, steer_angle))
        
        self.apply_hardware(pwm_val, steer_angle)
        
        # Also publish to /cmd_vel so dashboard can track velocity/odometry
        self.cmd_vel_pub.publish(msg)

    def apply_hardware(self, motor_pwm: int, steer_angle: int) -> None:
        """Update target hardware state; timer loop handles transmission."""
        self.target_motor_val = motor_pwm
        self.target_servo_val = steer_angle

    def _hardware_update_loop(self) -> None:
        """Send 10Hz heartbeat to Rosmaster, but only update on change or 1-second timeout to prevent serial spam."""
        self.hw_loop_monitor.tick()
        now = time.monotonic()
        if not self.manual_mode:
            timeout = float(self._param_cache['auto_cmd_timeout'])
            auto_age = now - self.last_auto_cmd_time if self.last_auto_cmd_time > 0.0 else float('inf')
            if auto_age > timeout:
                if not self.auto_cmd_stale_reported:
                    self.get_logger().warn(
                        f'Auto command stream stale for {auto_age:.2f}s (> {timeout:.2f}s), forcing MANUAL stop'
                    )
                    self.auto_cmd_stale_reported = True
                self.stop_robot()
                self.manual_mode = True
                self.auto_mode_pub.publish(Bool(data=False))
                self._update_dash()
                return

        heartbeat = float(self._param_cache['hw_heartbeat_sec'])
        # If values changed, OR every 1 second (safety heartbeat)
        if (self.target_motor_val != getattr(self, 'sent_motor_val', None) or 
            self.target_servo_val != getattr(self, 'sent_servo_val', None) or
            now - getattr(self, 'last_hw_send_time', 0.0) > heartbeat):
            
            try:
                self.bot.set_motor(self.target_motor_val, 0, 0, 0)
                self.bot.set_pwm_servo(self.servo_steer_id, self.target_servo_val)
                self.sent_motor_val = self.target_motor_val
                self.sent_servo_val = self.target_servo_val
                self.last_hw_send_time = now
                self.hw_error_count = 0
                self.hw_error_tripped = False
            except Exception as e:
                self.hw_error_count += 1
                self.get_logger().warn(f'Hardware send failed: {e}')
                if self.hw_error_count >= int(self._param_cache['hw_fail_limit']) and not self.hw_error_tripped:
                    self.hw_error_tripped = True
                    self.get_logger().error('Hardware send failure limit reached, stopping robot')
                    self.stop_robot()
                    if not self.manual_mode:
                        self.manual_mode = True
                        self.auto_mode_pub.publish(Bool(data=False))

    def _encoder_read_loop(self) -> None:
        """Read hardware encoders and compute/publish /odom"""
        self.encoder_loop_monitor.tick()
        now = time.monotonic()
        dt = now - self.last_odom_time
        
        # Don't divide by zero if timer fires too fast
        if dt <= 0.001 or dt > 0.3:
            self.last_odom_time = now
            return

        try:
            # Reads 4 motors: FL, FR, RL, RR 
            ticks = self.bot.get_motor_encoder()
            
            # First initialization
            if self.encoder_first_read or ticks is None or len(ticks) < 4:
                if ticks and len(ticks) == 4:
                    self.last_encoder_ticks = list(ticks)
                    self.encoder_first_read = False
                self.last_odom_time = now
                return
            
            # Calculate delta ticks
            d_ticks = [
                ticks[0] - self.last_encoder_ticks[0],
                ticks[1] - self.last_encoder_ticks[1],
                ticks[2] - self.last_encoder_ticks[2],
                ticks[3] - self.last_encoder_ticks[3]
            ]
            self.last_encoder_ticks = list(ticks)
            self.last_odom_time = now
            jump_thresh = float(self._param_cache['encoder_jump_threshold'])
            valid_d_ticks = []
            for d in d_ticks:
                if abs(d) > jump_thresh:
                    valid_d_ticks.append(0.0)
                    self._encoder_glitch_count += 1
                else:
                    valid_d_ticks.append(float(d))
            if self._encoder_glitch_count > 0 and now - self._last_glitch_warn_t > 1.0:
                self.get_logger().warn(f'Encoder jump filtered, count={self._encoder_glitch_count}')
                self._last_glitch_warn_t = now
                self._encoder_glitch_count = 0

            # RISA-Bot uses a single motor driving all wheels, or separate motors
            # Usually left/right speed is averaged.
            # Assuming standard Ackermann driving, rear wheels are driven, front steer.
            # If all 4 are driven (4WD), average left and right side
            left_ticks = (valid_d_ticks[0] + valid_d_ticks[2]) / 2.0
            right_ticks = (valid_d_ticks[1] + valid_d_ticks[3]) / 2.0
            avg_ticks = (left_ticks + right_ticks) / 2.0
            if bool(self._param_cache['odom_reverse_polarity']):
                avg_ticks = -avg_ticks

            # Convert to distance
            if self.ticks_per_meter <= 0:
                return
            distance = (avg_ticks / self.ticks_per_meter) * float(self._param_cache['odom_distance_scale'])
            linear_velocity = distance / dt
            max_lin = float(self._param_cache['max_linear_velocity'])
            if abs(linear_velocity) > max_lin:
                linear_velocity = max(-max_lin, min(max_lin, linear_velocity))
                distance = linear_velocity * dt
            vel_deadband = float(self._param_cache['odom_velocity_deadband'])
            if abs(linear_velocity) < vel_deadband:
                linear_velocity = 0.0
                distance = 0.0

            # Calculate steering angle from servo command (for Ackermann kinematics)
            # Servo range [40, 140], center 90
            steer_norm = (self.servo_center - self.target_servo_val) / float(self.servo_range)
            steer_norm = max(-1.0, min(1.0, steer_norm))
            steering_angle_deg = steer_norm * float(self._param_cache['steering_max_deg'])
            steering_angle_rad = math.radians(steering_angle_deg)
            
            # Ackermann kinematics: w = v / R, where R = L / tan(delta)
            # L is wheel base, delta is steering angle
            if abs(steering_angle_rad) > 0.01:
                turning_radius = self.wheel_base / math.tan(steering_angle_rad)
                angular_velocity = linear_velocity / turning_radius
            else:
                angular_velocity = 0.0
            angular_velocity *= float(self._param_cache['odom_yaw_scale'])
            max_ang = float(self._param_cache['max_angular_velocity'])
            angular_velocity = max(-max_ang, min(max_ang, angular_velocity))

            alpha = float(self._param_cache['odom_vel_alpha'])
            self._filtered_linear = alpha * linear_velocity + (1 - alpha) * self._filtered_linear
            self._filtered_angular = alpha * angular_velocity + (1 - alpha) * self._filtered_angular

            yaw_delta = self._filtered_angular * dt

            # Update pose
            self.odom_yaw += yaw_delta
            self.odom_yaw = math.atan2(math.sin(self.odom_yaw), math.cos(self.odom_yaw))
            self.odom_x += self._filtered_linear * math.cos(self.odom_yaw) * dt
            self.odom_y += self._filtered_linear * math.sin(self.odom_yaw) * dt

            # Build and publish Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = str(self._param_cache['odom_frame_id'])
            odom.child_frame_id = str(self._param_cache['base_frame_id'])

            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            
            # Quaternion from yaw
            odom.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)

            odom.twist.twist.linear.x = self._filtered_linear
            odom.twist.twist.angular.z = self._filtered_angular

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Failed to read encoders: {e}")

    def stop_robot(self) -> None:
        self.target_motor_val = 0
        self.target_servo_val = self.servo_center
        try:
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(self.servo_steer_id, self.servo_center)
        except Exception:
            pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoControllerV9()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        try:
            del node.bot
            print("Rosmaster closed")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
