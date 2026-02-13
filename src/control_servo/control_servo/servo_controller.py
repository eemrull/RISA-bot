#!/usr/bin/env python3
"""
Servo Controller V7 — The "Main Branch" Port
=============================================================================
Based on the analysis of the WORKING 'JoyRobotController' from main branch.

CRITICAL HARDWARE DIFFERENCES DISCOVERED:
1. Steering is on SERVO 4 (not 1 or 2).
   - Range: ~40 (Right) to ~140 (Left), Center=90.
2. Driving uses set_motor(pwm, 0, 0, 0) (not set_car_motion).
   - Range: -255 to 255 (PWM value).
   - NO Ackermann kinematics helper on board — we must drive motor directly.

Controls (mapped to standard ROS2 Joy):
- Left Stick Y (axes[1]):  Throttle (PWM -255 to 255)
- Right Stick X (axes[3]): Steering (Servo 4 angle)
- D-Pad UP/DOWN:           Speed Limiter (PWM cap)
- Start (btn 11):          Toggle Auto/Manual
- RB (btn 5) + LB (btn 4): Special functions (if needed)

This node listens to /cmd_vel and converts Twist messages to these hardware calls,
allowing 'auto_driver' to work transparently.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rclpy.qos import QoSPresetProfiles
from Rosmaster_Lib import Rosmaster
import time
import math

# --- Configuration ---
SERVO_STEER_ID = 4
SERVO_CENTER = 90
SERVO_RANGE = 50   # 90 +/- 50 = [40, 140]

# Hardware limits
PWM_MAX = 100      # Cap for safety (Main branch used 100 for max speed level)
PWM_MIN = 15       # Minimum to move

class ServoControllerV7(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Connect to Hardware
        self.bot = None
        try:
            self.bot = Rosmaster()
            # Note: Main branch JoyRobotController did NOT call set_car_type()
            # self.bot.set_car_type(1) 
            self.bot.set_motor(0, 0, 0, 0)
            self.bot.set_pwm_servo(SERVO_STEER_ID, SERVO_CENTER)
            self.get_logger().info("✅ Rosmaster Connected (V7 Main-Port)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Rosmaster: {e}")
            exit(1)

        # Publishers
        self.dash_pub = self.create_publisher(String, '/dashboard_ctrl', 10)
        self.auto_mode_pub = self.create_publisher(Bool, '/auto_mode', 10)

        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.cmd_vel_auto_callback, 10)

        # State
        self.manual_mode = True
        self.current_speed_limit = 50  # Start at 50% PWM
        self.speed_levels = [25, 40, 60, 100]
        self.speed_idx = 1 # Index 1 -> 40
        self.current_speed_limit = self.speed_levels[self.speed_idx]

        self.last_servo_val = SERVO_CENTER
        self.last_motor_val = 0
        self.last_joy_time = time.time()
        
        # Debounce
        self.prev_buttons = [0] * 15
        self.prev_axes = [0.0] * 8

        self.get_logger().info("🎮 V7 Ready: LeftStick=Drive, RightStick=Steer (Servo 4)")
        self._update_dash()

    def _update_dash(self):
        msg = String()
        mode_str = "MANUAL" if self.manual_mode else "AUTO"
        msg.data = f"{self.current_speed_limit}|0|{mode_str}"
        self.dash_pub.publish(msg)

    def joy_callback(self, msg):
        self.last_joy_time = time.time()
        
        # Mapping helpers
        def btn(idx): return msg.buttons[idx] if idx < len(msg.buttons) else 0
        def axis(idx): return msg.axes[idx] if idx < len(msg.axes) else 0.0
        def rose(idx): # Rising edge
            return btn(idx) == 1 and (idx >= len(self.prev_buttons) or self.prev_buttons[idx] == 0)

        # Toggle Mode (Start Button = 11 usually, Main branch used Y=3?? user said Start)
        # We will use Start (11) as standard, but also Y (3) to match main branch habit
        if rose(11) or rose(3): 
            self.manual_mode = not self.manual_mode
            self.auto_mode_pub.publish(Bool(data=not self.manual_mode))
            self.get_logger().info(f"Mode toggled: {'MANUAL' if self.manual_mode else 'AUTO'}")
            if self.manual_mode:
                self.stop_robot()
            self._update_dash()

        # Speed Control (D-Pad UP/DOWN - axes[7])
        dpad = axis(7)
        prev_dpad = self.prev_axes[7] if 7 < len(self.prev_axes) else 0.0
        
        if dpad > 0.5 and prev_dpad <= 0.5: # UP
            self.speed_idx = min(len(self.speed_levels)-1, self.speed_idx + 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()
        elif dpad < -0.5 and prev_dpad >= -0.5: # DOWN
            self.speed_idx = max(0, self.speed_idx - 1)
            self.current_speed_limit = self.speed_levels[self.speed_idx]
            self.get_logger().info(f"Speed Limit: {self.current_speed_limit}")
            self._update_dash()

        # MANUAL DRIVING
        if self.manual_mode:
            # Throttle: Left Stick Y (Axis 1)
            throttle_raw = axis(1) 
            # Steering: Right Stick X (Axis 3) -- Note: Main used Axis 2? Standard is 3.
            # We will use 3 (Standard) but also check 2 if 3 is zero, just in case.
            steer_raw = axis(3)
            if abs(steer_raw) < 0.1 and abs(axis(2)) > 0.1:
                steer_raw = axis(2) 

            # Deadzone
            if abs(throttle_raw) < 0.1: throttle_raw = 0.0
            if abs(steer_raw) < 0.1: steer_raw = 0.0

            # Calculate Hardware Values
            
            # 1. Drive (PWM -255 to 255)
            # Map stick -1..1 to -MAX..MAX
            motor_pwm = int(throttle_raw * self.current_speed_limit * 2.55) # Scale limit to [0-255]
            
            # 2. Steer (Servo 4 Angle 40-140)
            # Stick Left (+1) -> Angle +50 (140)
            # Stick Right (-1) -> Angle -50 (40)
            # Main branch used: target -= joy * 50 (Inverted?)
            # Let's try standard mapping: Left stick positive -> Turn Left -> Servo Increase?
            # Main branch: joy_val > 0 (Left?) -> target_angle -= val (Decrease)
            # So Joy Left -> Servo Decrease (Smaller angle)
            # Joy Right -> Servo Increase (Larger angle)
            # Let's stick to Main Branch logic:
            # target = CENTER - (joy * SWING)
            steer_angle = int(SERVO_CENTER - (steer_raw * SERVO_RANGE))
            steer_angle = max(SERVO_CENTER - SERVO_RANGE, min(SERVO_CENTER + SERVO_RANGE, steer_angle))

            self.apply_hardware(motor_pwm, steer_angle)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

    def cmd_vel_callback(self, msg):
        if not self.manual_mode:
            self.process_twist(msg)

    def cmd_vel_auto_callback(self, msg):
        if not self.manual_mode:
            self.process_twist(msg)

    def process_twist(self, msg):
        # Convert Twist (linear.x m/s, angular.z rad/s) to Hardware (PWM, Servo Angle)
        
        # 1. Linear X -> Motor PWM
        # Assume 1.0 m/s ~= 255 PWM (approx)
        # We clamp to our current speed limit for safety
        
        pwm_val = int(msg.linear.x * 255.0) # Simple scaling
        
        # 2. Angular Z -> Servo Angle
        # Twist angular.z > 0 is Left Turn
        # Servo: Left is Decrease (from Main Logic: target -= joy)
        # So Z > 0 -> Angle < 90
        # Scale: 1.0 rad/s -> Full Lock (50 deg)?
        
        angle_offset = msg.angular.z * (50.0 / 1.0) # 1 rad/s = 50 deg steer
        steer_angle = int(SERVO_CENTER - angle_offset)
        
        # Clamp
        pwm_val = max(-255, min(255, pwm_val))
        steer_angle = max(SERVO_CENTER - SERVO_RANGE, min(SERVO_CENTER + SERVO_RANGE, steer_angle))
        
        self.apply_hardware(pwm_val, steer_angle)

    def apply_hardware(self, motor_pwm, steer_angle):
        # Send to bot if changed
        try:
            # Motor
            if abs(motor_pwm - self.last_motor_val) > 2: # Jitter filter
                self.bot.set_motor(motor_pwm, 0, 0, 0)
                self.last_motor_val = motor_pwm
            
            # Servo
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
    node = ServoControllerV7()
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
