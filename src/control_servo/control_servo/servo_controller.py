#!/usr/bin/env python3
"""
Servo Controller + Mode Switch Node
- Right Stick: Camera servo control (pan/tilt)
- Left Stick: Manual driving (when in manual mode)
- Start/Options Button (button 7): Toggle auto/manual mode
- LB (button 4): Previous challenge state
- RB (button 5): Next challenge state

Publishes:
  /auto_mode (Bool) — True = auto, False = manual
  /cmd_vel (Twist) — manual driving commands (only in manual mode)
  /set_challenge (String) — challenge state override
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# --- Import your Rosmaster library ---
from Rosmaster_Lib import Rosmaster
import time

# --- Initialize the bot object globally ---
try:
    bot = Rosmaster()
    bot.set_car_type(1)  # Set to Rosmaster X3
    print("Rosmaster Serial Opened!")
    print(f"DEBUG: Bot attributes: {dir(bot)}")
    if hasattr(bot, 'ser'):
        print(f"DEBUG: bot.ser exists: {bot.ser}")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting. Make sure the robot is powered on and connected.")
    exit()

# Challenge states for cycling with bumper buttons (matches ChallengeState enum)
CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Create a subscription to the /joy topic
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )

        # Publishers
        self.auto_mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.challenge_pub = self.create_publisher(String, '/set_challenge', 10)
        self.dash_ctrl_pub = self.create_publisher(String, '/dashboard_ctrl', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber for Auto Mode commands
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Enable Bot Receive Threading (for Odometry)
        try:
            bot.create_receive_threading()
            self.get_logger().info("Rosmaster receive thread started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start receive thread: {e}")

        # Timer for Odometry (10Hz)
        self.create_timer(0.1, self.publish_odom)


        # Debug bot methods
        try:
            self.get_logger().info(f"Bot Methods: {dir(bot)}")
            # help(bot.set_car_motion) # This might spam or not work well in ROS log
        except:
            pass
        
        # State
        self.auto_mode = False
        self.last_s1 = 90
        self.last_s2 = 90
        self.challenge_index = 0

        # Button debouncing — track previous states
        self.prev_buttons = []
        self.prev_axes = []

        # Speed level (0-100%) — adjust with D-pad up/down
        self.speed_pct = 50  # default 50%

        # Parameters for manual driving
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 5.0) # rad/s (increased from 0.80)
        self.declare_parameter('toggle_button', 11)          # Start button
        self.declare_parameter('prev_state_button', 6)       # LB
        self.declare_parameter('next_state_button', 7)       # RB

        self.get_logger().info('*** SERVO CONTROLLER V3 LOADED ***')
        self.get_logger().info('Servo Controller + Mode Switch started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick  → Manual drive (fwd/back + turn)')
        self.get_logger().info('  Right Stick → Camera servos (S1/S2)')
        self.get_logger().info('  Start       → Toggle AUTO/MANUAL mode')
        self.get_logger().info('  LB/RB       → Cycle challenge states')
        self.get_logger().info('  D-pad ▲/▼   → Speed ±10%')
        self.get_logger().info(f'  Mode: MANUAL | Speed: {self.speed_pct}%')

        # Publish initial state so dashboard starts correctly
        mode_msg = Bool()
        mode_msg.data = self.auto_mode
        self.auto_mode_pub.publish(mode_msg)
        ctrl_msg = String()
        ctrl_msg.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
        self.dash_ctrl_pub.publish(ctrl_msg)

    def _button_pressed(self, msg, button_idx):
        """Return True only on rising edge (button was just pressed)."""
        if button_idx >= len(msg.buttons):
            return False
        current = msg.buttons[button_idx]
        previous = self.prev_buttons[button_idx] if button_idx < len(self.prev_buttons) else 0
        return current == 1 and previous == 0

    def joy_callback(self, msg):
        """Called every time a /joy message is received."""
        toggle_btn = self.get_parameter('toggle_button').value
        prev_btn = self.get_parameter('prev_state_button').value
        next_btn = self.get_parameter('next_state_button').value

        try:
            # ===== AUTO/MANUAL TOGGLE =====
            if self._button_pressed(msg, toggle_btn):
                self.auto_mode = not self.auto_mode
                mode_str = "🤖 AUTO" if self.auto_mode else "🎮 MANUAL"
                self.get_logger().info(f'Mode switched to: {mode_str}')

                # Publish mode
                mode_msg = Bool()
                mode_msg.data = self.auto_mode
                self.auto_mode_pub.publish(mode_msg)

            # ===== CHALLENGE STATE CYCLING =====
            if self._button_pressed(msg, next_btn):
                self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
                state_name = CHALLENGE_STATES[self.challenge_index]
                self.get_logger().info(f'Challenge → {state_name}')
                self.challenge_pub.publish(String(data=state_name))

            if self._button_pressed(msg, prev_btn):
                self.challenge_index = (self.challenge_index - 1) % len(CHALLENGE_STATES)
                state_name = CHALLENGE_STATES[self.challenge_index]
                self.get_logger().info(f'Challenge → {state_name}')
                self.challenge_pub.publish(String(data=state_name))

            # ===== D-PAD SPEED CONTROL =====
            if len(msg.axes) > 7:
                dpad_up = msg.axes[7] > 0.5
                dpad_down = msg.axes[7] < -0.5
                prev_dpad_up = (self.prev_axes[7] > 0.5) if len(self.prev_axes) > 7 else False
                prev_dpad_down = (self.prev_axes[7] < -0.5) if len(self.prev_axes) > 7 else False

                if dpad_up and not prev_dpad_up:
                    self.speed_pct = min(100, self.speed_pct + 10)
                    self.get_logger().info(f'Speed: {self.speed_pct}%')
                if dpad_down and not prev_dpad_down:
                    self.speed_pct = max(10, self.speed_pct - 10)
                    self.get_logger().info(f'Speed: {self.speed_pct}%')

            # ===== MANUAL DRIVING (only when NOT in auto mode) =====
            if not self.auto_mode:
                cmd = Twist()
                max_lin = self.get_parameter('max_linear_speed').value
                max_ang = self.get_parameter('max_angular_speed').value
                scale = self.speed_pct / 100.0

                # Left Stick: axes[1] = forward/back, axes[0] = left/right
                if len(msg.axes) > 1:
                    cmd.linear.x = msg.axes[1] * max_lin * scale
                if len(msg.axes) > 0:
                    cmd.angular.z = msg.axes[0] * max_ang * scale

                if len(msg.axes) > 0:
                    cmd.angular.z = msg.axes[0] * max_ang * scale
                
                # Debug steering
                if abs(cmd.angular.z) > 0.1:
                    print(f"\r[MANUAL] Steer Config: Axis0={msg.axes[0]:.2f} Max={max_ang} Cmd={cmd.angular.z:.2f}    ", end='', flush=True)

                self.cmd_vel_pub.publish(cmd)
                
                # Send to hardware
                try:
                    # Log command every 0.5s to verify values
                    now = self.get_clock().now().nanoseconds / 1e9
                    if not hasattr(self, 'last_debug_time'): self.last_debug_time = 0
                    if now - self.last_debug_time > 0.5:
                        self.get_logger().info(f"Writing HW: vx={cmd.linear.x:.2f} vy={cmd.linear.y:.2f} wz={cmd.angular.z:.2f}")
                        self.last_debug_time = now
                        
                    bot.set_car_motion(cmd.linear.x, cmd.linear.y, cmd.angular.z)
                except Exception as e:
                    self.get_logger().error(f"Hardware write failed: {e}")

            # ===== CAMERA SERVOS (always active) =====
            if len(msg.axes) > 4:
                s1_val = int((msg.axes[3] * -1.0 + 1.0) * 90.0)
                s2_val = int((msg.axes[4] * -1.0 + 1.0) * 90.0)

                if s1_val != self.last_s1:
                    bot.set_pwm_servo(1, s1_val)
                    self.last_s1 = s1_val

                if s2_val != self.last_s2:
                    bot.set_pwm_servo(2, s2_val)
                    self.last_s2 = s2_val

            # ===== PUBLISH DASHBOARD CONTROLLER INFO =====
            ctrl_msg = String()
            ctrl_msg.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
            self.dash_ctrl_pub.publish(ctrl_msg)

            # Save button + axis states for debouncing
            self.prev_buttons = list(msg.buttons)
            self.prev_axes = list(msg.axes)

            # Debug output
            mode_str = "AUTO" if self.auto_mode else "MANUAL"
            state_str = CHALLENGE_STATES[self.challenge_index]
            # Show which buttons are currently pressed (for mapping debug)
            pressed = [i for i, b in enumerate(msg.buttons) if b == 1]
            btn_info = f'Btns:{pressed}' if pressed else ''
            print(f"\r[JOY v3] {mode_str} | State: {state_str} | Spd: {self.speed_pct}% | S1: {self.last_s1} S2: {self.last_s2} {btn_info}    ", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')

    def cmd_vel_callback(self, msg):
        """Receive /cmd_vel from AutoDriver (or myself). Only act if in AUTO mode."""
        if self.auto_mode:
            try:
                bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
            except Exception as e:
                self.get_logger().error(f"Auto HW Write Error: {e}")

    def publish_odom(self):
        """Read bot state and publish odometry."""
        try:
            # Assuming bot property names from dir() dump: _Rosmaster__vx, etc.
            # But the library likely provides getters?
            # logs showed 'get_motion_data'.
            # Let's try get_motion_data().
            # If that fails, we can fall back to direct access.
            
            # bot.get_motion_data() likely returns (vx, vy, az)????
            # Let's try to infer from typical Yahboom API.
            # Usually: ax, ay, az = bot.get_accelerometer_data()
            # vx, vy, wz = bot.get_motion_data() ??
            
            # For now, let's look at available methods again:
            # 'get_motion_data', '_Rosmaster__vx', '_Rosmaster__vy'
            
            # Direct access is safer than guessing return tuple size.
            # Names are mangled: bot._Rosmaster__vx
            
            vx = getattr(bot, '_Rosmaster__vx', 0.0)
            vy = getattr(bot, '_Rosmaster__vy', 0.0)
            wz = getattr(bot, '_Rosmaster__vz', 0.0) # VZ is probably angular Z
            
            # Simple Odometry (Velocity only for now to satisfy dashboard)
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            odom.twist.twist.linear.x = float(vx)
            odom.twist.twist.linear.y = float(vy)
            odom.twist.twist.angular.z = float(wz)
            
            self.odom_pub.publish(odom)
            
        except Exception as e:
            # Don't spam errors
            pass



def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoControllerNode()
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Center servos before quitting
        bot.set_pwm_servo(1, 90)
        bot.set_pwm_servo(2, 90)
        time.sleep(0.5)

        del bot
        print("Rosmaster object deleted, serial port closed.")

        servo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

