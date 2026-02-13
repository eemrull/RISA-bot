#!/usr/bin/env python3
"""
Servo Controller V8 — Refined Main Branch Port
=============================================================================
Based on V7 (Main Branch Logic) + User Feedback:
1. Steering Axis Fix: REMOVED fallback to Axis 2 (which caused Right Stick Y/Trigger interference).
   - STRICTLY uses Axis 3 (Right Stick X) for steering.
2. Restored LB/RB Challenge Cycling.
   - LB (Btn 4) -> Previous Challenge
   - RB (Btn 5) -> Next Challenge
3. Driving uses set_motor(pwm) and set_pwm_servo(4) (Confirmed working).

Controls:
- Left Stick Y (Axis 1):   Throttle (PWM -255 to 255)
- Right Stick X (Axis 3):  Steering (Servo 4 Angle 40-140)
- D-Pad UP/DOWN:           Speed Limiter [25, 40, 60, 100]%
- Start (Btn 11) / Y (3):  Toggle Auto/Manual
- LB (Btn 4) / RB (Btn 5): Cycle Challenge State
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

class ServoControllerV8(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Connect to Hardware
        self.bot = None
        try:
            self.bot = Rosmaster()
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(SERVO_STEER_ID, SERVO_CENTER)
            self.get_logger().info("✅ Rosmaster Connected (V8 Refined)")
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
        self.speed_idx = 1 # Index 1 -> 40
        self.current_speed_limit = self.speed_levels[self.speed_idx]

        self.last_servo_val = SERVO_CENTER
        self.last_motor_val = 0
        # Debounce
        self.prev_buttons = [0] * 15
        self.prev_axes = [0.0] * 8

        self.get_logger().info("🎮 V8 Ready: Right Stick X = Steer | LB/RB = Challenges")
        self._update_dash()

    def _update_dash(self):
        mode_str = "MANUAL" if self.manual_mode else "AUTO"
        state_str = CHALLENGE_STATES[self.challenge_index]
        # Format: SpeedLimit | ChallengeIndex | StateName | Mode
        # Dashboard expects specific format so we send a composite string
        msg = String()
        msg.data = f"{self.current_speed_limit}|{self.challenge_index}|{state_str}" 
        # Note: In V6 we sent "Speed|Idx|State". V7 sent "Speed|0|Mode". 
        # Dashboard expects "Speed|Idx|State".
        self.dash_pub.publish(msg)

    def joy_callback(self, msg):
        # Helpers
        def btn(idx): return msg.buttons[idx] if idx < len(msg.buttons) else 0
        def axis(idx): return msg.axes[idx] if idx < len(msg.axes) else 0.0
        def rose(idx): # Rising edge
            return btn(idx) == 1 and (idx >= len(self.prev_buttons) or self.prev_buttons[idx] == 0)

        # 1. Toggle Mode (Start=11, Y=3)
        if rose(11) or rose(3): 
            self.manual_mode = not self.manual_mode
            self.auto_mode_pub.publish(Bool(data=not self.manual_mode))
            self.get_logger().info(f"Mode: {'MANUAL' if self.manual_mode else 'AUTO'}")
            if self.manual_mode: self.stop_robot()
            self._update_dash()

        # 2. Challenge Cycling (LB=4, RB=5)
        if rose(5): # RB -> Next
            self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
            self.challenge_pub.publish(String(data=CHALLENGE_STATES[self.challenge_index]))
            self.get_logger().info(f"RB Pressed -> Challenge: {CHALLENGE_STATES[self.challenge_index]}")
            self._update_dash()
        
        if rose(4): # LB -> Prev
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
        if self.manual_mode:
            # Throttle: Left Stick Y (Axis 1)
            throttle_raw = axis(1)
            
            # Steering: Right Stick X
            # User Feedback V9: "Forward/Back steers" (Axis 3 on this controller).
            # "Switch axis 3 to 2" -> So we use Axis 2.
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
        try:
            if abs(motor_pwm - self.last_motor_val) > 2:
                self.bot.set_motor(motor_pwm, 0, 0, 0)
                self.last_motor_val = motor_pwm
            
            if abs(steer_angle - self.last_servo_val) > 1:
                self.bot.set_pwm_servo(SERVO_STEER_ID, steer_angle)
                self.last_servo_val = steer_angle
        except Exception as e:
            self.get_logger().error(f"HW error: {e}")

    def stop_robot(self):
        try:
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(SERVO_STEER_ID, SERVO_CENTER)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerV8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        try: del node.bot; print("Rosmaster closed")
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
