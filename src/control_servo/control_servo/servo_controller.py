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

# --- Import your Rosmaster library ---
from Rosmaster_Lib import Rosmaster
import time

# --- Initialize the bot object globally ---
try:
    bot = Rosmaster()
    bot.set_car_type(1)  # Set to Rosmaster X3
    print("Rosmaster Serial Opened!")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting. Make sure the robot is powered on and connected.")
    exit()

# Challenge states for cycling with bumper buttons
CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT', 'TUNNEL',
    'BOOM_GATE_TUNNEL', 'BOOM_GATE_MAIN', 'HILL', 'BUMPER',
    'TRAFFIC_LIGHT', 'PARALLEL_PARK', 'PERPENDICULAR_PARK'
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

        # State
        self.auto_mode = False
        self.last_s1 = 90
        self.last_s2 = 90
        self.challenge_index = 0

        # Button debouncing — track previous button states
        self.prev_buttons = []

        # Parameters for manual driving
        self.declare_parameter('max_linear_speed', 0.20)   # m/s
        self.declare_parameter('max_angular_speed', 0.80)   # rad/s
        self.declare_parameter('toggle_button', 7)          # Start/Options button
        self.declare_parameter('prev_state_button', 4)      # LB
        self.declare_parameter('next_state_button', 5)      # RB

        self.get_logger().info('Servo Controller + Mode Switch started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick  → Manual drive (fwd/back + turn)')
        self.get_logger().info('  Right Stick → Camera servos (S1/S2)')
        self.get_logger().info('  Start       → Toggle AUTO/MANUAL mode')
        self.get_logger().info('  LB/RB       → Cycle challenge states')
        self.get_logger().info(f'  Mode: MANUAL')

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

            # ===== MANUAL DRIVING (only when NOT in auto mode) =====
            if not self.auto_mode:
                cmd = Twist()
                max_lin = self.get_parameter('max_linear_speed').value
                max_ang = self.get_parameter('max_angular_speed').value

                # Left Stick: axes[1] = forward/back, axes[0] = left/right
                if len(msg.axes) > 1:
                    cmd.linear.x = msg.axes[1] * max_lin   # forward/back
                if len(msg.axes) > 0:
                    cmd.angular.z = msg.axes[0] * max_ang   # turn left/right

                self.cmd_vel_pub.publish(cmd)

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

            # Save button states for debouncing
            self.prev_buttons = list(msg.buttons)

            # Debug output
            mode_str = "AUTO" if self.auto_mode else "MANUAL"
            state_str = CHALLENGE_STATES[self.challenge_index]
            print(f"\r[JOY] {mode_str} | State: {state_str} | S1: {self.last_s1} S2: {self.last_s2}", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')


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

