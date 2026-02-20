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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from Rosmaster_Lib import Rosmaster
import time

# --- Configuration ---
SERVO_STEER_ID = 4
SERVO_CENTER = 90
SERVO_RANGE = 50   # 90 +/- 50 = [40, 140]

CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]

class ServoControllerV9(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Connect to Hardware
        self.bot = None
        try:
            self.bot = Rosmaster()
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(SERVO_STEER_ID, SERVO_CENTER)
            self.get_logger().info("✅ Rosmaster Connected (V9 Competition)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Rosmaster: {e}")
            exit(1)

        # Publishers
        self.dash_pub = self.create_publisher(String, '/dashboard_ctrl', 10)
        self.auto_mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.challenge_pub = self.create_publisher(String, '/set_challenge', 10)

        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.cmd_vel_auto_callback, 10)

        # State
        self.manual_mode = True
        self.challenge_index = 0
        self.speed_levels = [25, 40, 60, 100]
        self.speed_idx = 0 # Index 0 -> 25% Default
        self.current_speed_limit = self.speed_levels[self.speed_idx]

        self.target_motor_val = 0
        self.target_servo_val = SERVO_CENTER
        # Hardware continuous update loop (10 Hz)
        self.create_timer(0.1, self._hardware_update_loop)

        # Debounce
        self.prev_buttons = [0] * 15
        self.prev_axes = [0.0] * 8

        # Safety: Joy watchdog & Ghost Input Protection
        self.joy_alive = False          # True once first /joy received
        self.joy_unlocked = False       # True once input changes from initial ghost state
        self.initial_joy_axes = None
        self.initial_joy_buttons = None
        
        self.last_joy_time = 0.0        # timestamp of last /joy message
        self.joy_timeout = 0.5          # seconds before declaring controller lost
        self.joy_lost_reported = False   # avoid spamming log
        self.create_timer(0.3, self._joy_watchdog)

        self.get_logger().info("🎮 V9 Ready: Right Stick X = Steer | LB/RB = Challenges")
        self._update_dash()

    def _update_dash(self):
        """Publish controller state to dashboard (speed|index|state)."""
        state_str = CHALLENGE_STATES[self.challenge_index]
        msg = String()
        msg.data = f"{self.current_speed_limit}|{self.challenge_index}|{state_str}"
        self.dash_pub.publish(msg)

    def _joy_watchdog(self):
        """Stop robot if controller disconnects (no /joy for >0.5s)."""
        if not self.joy_alive:
            return  # Haven't received first joy yet, nothing to do
        elapsed = time.time() - self.last_joy_time
        if elapsed > self.joy_timeout:
            if not self.joy_lost_reported:
                self.get_logger().warn(
                    f"⚠️ Controller lost (no /joy for {elapsed:.1f}s) — STOPPING ROBOT")
                self.joy_lost_reported = True
                self.joy_unlocked = False  # Lock to ignore phantom inputs on next receive
            if self.manual_mode:
                self.stop_robot()

    def joy_callback(self, msg):
        # Mark controller as alive
        self.last_joy_time = time.time()
        
        if not self.joy_alive:
            self.joy_alive = True
            self.initial_joy_axes = list(msg.axes)
            self.initial_joy_buttons = list(msg.buttons)
            self.get_logger().info("🎮 Controller receiver connected (Awaiting sync...)")
            
        if self.joy_lost_reported:
            self.get_logger().info("🎮 Controller module synced. Awaiting explicit input to unlock...")
            self.joy_lost_reported = False
            # Do NOT aggressively unlock here; rely on the deviation check below

        # Ghost state protection: USB dongles often send a stream of [0, 1.0, 0...] when the gamepad is off.
        # Ignore all commands until the state deviates from the initial generic signature.
        if not self.joy_unlocked:
            if list(msg.axes) != self.initial_joy_axes or list(msg.buttons) != self.initial_joy_buttons:
                self.joy_unlocked = True
                self.get_logger().info("🎮 Controller unlocked (Valid human input detected)")
            else:
                return  # Return early, ignore phantom inputs

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
            if self.manual_mode: self.stop_robot()
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
            steer_angle = int(SERVO_CENTER - (steer_raw * SERVO_RANGE))
            steer_angle = max(SERVO_CENTER - SERVO_RANGE, min(SERVO_CENTER + SERVO_RANGE, steer_angle))

            self.apply_hardware(motor_pwm, steer_angle)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

    def cmd_vel_callback(self, msg):
        if not self.manual_mode: self.process_twist(msg)

    def cmd_vel_auto_callback(self, msg):
        if not self.manual_mode: self.process_twist(msg)

    def process_twist(self, msg):
        """Convert Twist message to hardware PWM + servo angle."""
        # Auto Mode Driving
        pwm_val = int(msg.linear.x * 255.0)
        
        # Steering
        # twist.angular.z > 0 is Left
        # Main branch: joy > 0 -> Angle Decrease
        # So Z > 0 -> Angle Decrease
        angle_offset = msg.angular.z * (50.0 / 1.0)
        steer_angle = int(SERVO_CENTER - angle_offset)
        
        # Clamp
        pwm_val = max(-255, min(255, pwm_val))
        steer_angle = max(SERVO_CENTER - SERVO_RANGE, min(SERVO_CENTER + SERVO_RANGE, steer_angle))
        
        self.apply_hardware(pwm_val, steer_angle)

    def apply_hardware(self, motor_pwm, steer_angle):
        """Update target hardware state; timer loop handles transmission."""
        self.target_motor_val = motor_pwm
        self.target_servo_val = steer_angle

    def _hardware_update_loop(self):
        """Send 10Hz heartbeat to Rosmaster, but only update on change or 1-second timeout to prevent serial spam."""
        now = time.time()
        # If values changed, OR every 1 second (safety heartbeat)
        if (self.target_motor_val != getattr(self, 'sent_motor_val', None) or 
            self.target_servo_val != getattr(self, 'sent_servo_val', None) or
            now - getattr(self, 'last_hw_send_time', 0.0) > 1.0):
            
            try:
                self.bot.set_motor(self.target_motor_val, 0, 0, 0)
                self.bot.set_pwm_servo(SERVO_STEER_ID, self.target_servo_val)
                self.sent_motor_val = self.target_motor_val
                self.sent_servo_val = self.target_servo_val
                self.last_hw_send_time = now
            except Exception:
                pass

    def stop_robot(self):
        self.target_motor_val = 0
        self.target_servo_val = SERVO_CENTER
        try:
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(SERVO_STEER_ID, SERVO_CENTER)
        except Exception:
            pass

def main(args=None):
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
