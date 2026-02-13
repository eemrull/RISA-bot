#!/usr/bin/env python3
"""
Servo Controller + Mode Switch Node  (V6)
Based on the WORKING main-branch servo_controller, extended with:
  - Manual driving (left stick → set_car_motion)
  - Auto/Manual mode toggle
  - Challenge state cycling
  - D-pad speed control
  - Odometry publishing
  - Dashboard info publishing
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Import the Joy message type
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# --- Import your Rosmaster library ---
from Rosmaster_Lib import Rosmaster
import time

# --- Initialize the bot object globally ---
# This is exactly like the main branch: bot = Rosmaster()
bot = None
try:
    bot = Rosmaster()
    bot.set_car_type(1)  # Set to Rosmaster X3
    bot.set_car_motion(0, 0, 0)  # Stop motors on startup
    print("Rosmaster Serial Opened!")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting. Make sure the robot is powered on and connected.")
    exit()

# Challenge states for cycling
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
            Joy,
            'joy',
            self.joy_callback,
            10)

        # ===== Publishers =====
        self.auto_mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.cmd_vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.challenge_pub = self.create_publisher(String, '/set_challenge', 10)
        self.dash_ctrl_pub = self.create_publisher(String, '/dashboard_ctrl', 10)
        self.odom_pub      = self.create_publisher(Odometry, '/odom', 10)

        # ===== Subscriber for auto-mode velocity commands =====
        self.create_subscription(Twist, '/cmd_vel_auto', self.cmd_vel_callback, 10)

        # ===== Rosmaster receive threading (for odometry) =====
        try:
            bot.create_receive_threading()
            self.get_logger().info("Rosmaster receive thread started.")
        except Exception as e:
            self.get_logger().error(f"Receive thread error: {e}")

        # Stop again after threading is up
        try:
            bot.set_car_motion(0, 0, 0)
        except Exception:
            pass

        # ===== Timers =====
        self.create_timer(0.1, self.publish_odom)          # 10 Hz odometry
        self._stop_timer = self.create_timer(0.05, self._startup_stop)  # 20 Hz stop spam

        # ===== State =====
        # Store the last servo values to avoid spamming the I2C bus
        # (exact same as main branch)
        self.last_s1 = 90
        self.last_s2 = 90

        # New state for extended features
        self.auto_mode = False
        self.challenge_index = 0
        self.prev_buttons = []
        self.prev_axes = []
        self._got_joy = False
        self._start_t = time.time()
        self._dbg_t = 0.0

        # Speed levels — cycle with D-pad
        self.SPEED_LEVELS = [25, 50, 75, 100]
        self.speed_idx = 1          # Start at 50%
        self.speed_pct = self.SPEED_LEVELS[self.speed_idx]

        # Parameters
        # Rosmaster set_car_motion: vx ∈ [-1.0, 1.0] m/s, vz ∈ [-5.0, 5.0] rad/s
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 5.0)
        self.declare_parameter('toggle_button', 11)       # Start/Options
        self.declare_parameter('prev_state_button', 6)    # LB
        self.declare_parameter('next_state_button', 7)    # RB

        self.get_logger().info('*** SERVO CONTROLLER V6 ***')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick Y  → Throttle (fwd/back)')
        self.get_logger().info('  Left Stick X  → Turn (left/right)')
        self.get_logger().info('  Right Stick   → Servo S1/S2 (axes[3]/axes[4]) ← same as main branch')
        self.get_logger().info('  D-pad ▲/▼     → Speed [25,50,75,100]%')
        self.get_logger().info('  Start         → Toggle AUTO/MANUAL')
        self.get_logger().info('  LB/RB         → Cycle challenge state')
        self.get_logger().info(f'  Mode: MANUAL | Speed: {self.speed_pct}%')

        # Publish initial state
        self.auto_mode_pub.publish(Bool(data=False))
        ctrl = String()
        ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
        self.dash_ctrl_pub.publish(ctrl)

    # ---- Startup stop spam ----
    def _startup_stop(self):
        """Send stop commands for first 2s to clear residual board velocity."""
        if self._got_joy or (time.time() - self._start_t) > 2.0:
            return
        try:
            bot.set_car_motion(0, 0, 0)
        except Exception:
            pass

    # ---- Button edge detection ----
    def _btn_pressed(self, msg, idx):
        if idx >= len(msg.buttons):
            return False
        cur = msg.buttons[idx]
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return cur == 1 and prev == 0

    def joy_callback(self, msg):
        """This function is called every time a /joy message is received."""
        self._got_joy = True
        toggle_btn = self.get_parameter('toggle_button').value
        prev_btn   = self.get_parameter('prev_state_button').value
        next_btn   = self.get_parameter('next_state_button').value

        try:
            # ===== AUTO/MANUAL TOGGLE =====
            if self._btn_pressed(msg, toggle_btn):
                self.auto_mode = not self.auto_mode
                label = "🤖 AUTO" if self.auto_mode else "🎮 MANUAL"
                self.get_logger().info(f'Mode: {label}')
                self.auto_mode_pub.publish(Bool(data=self.auto_mode))
                if not self.auto_mode:
                    try: bot.set_car_motion(0, 0, 0)
                    except: pass

            # ===== CHALLENGE STATE CYCLING =====
            if self._btn_pressed(msg, next_btn):
                self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
                name = CHALLENGE_STATES[self.challenge_index]
                self.get_logger().info(f'Challenge → {name}')
                self.challenge_pub.publish(String(data=name))
            if self._btn_pressed(msg, prev_btn):
                self.challenge_index = (self.challenge_index - 1) % len(CHALLENGE_STATES)
                name = CHALLENGE_STATES[self.challenge_index]
                self.get_logger().info(f'Challenge → {name}')
                self.challenge_pub.publish(String(data=name))

            # ===== D-PAD SPEED CONTROL =====
            if len(msg.axes) > 7:
                dup   = msg.axes[7] > 0.5
                ddown = msg.axes[7] < -0.5
                pup   = (self.prev_axes[7] > 0.5) if len(self.prev_axes) > 7 else False
                pdown = (self.prev_axes[7] < -0.5) if len(self.prev_axes) > 7 else False
                if dup and not pup:
                    self.speed_idx = min(len(self.SPEED_LEVELS) - 1, self.speed_idx + 1)
                    self.speed_pct = self.SPEED_LEVELS[self.speed_idx]
                    self.get_logger().info(f'Speed: {self.speed_pct}%')
                if ddown and not pdown:
                    self.speed_idx = max(0, self.speed_idx - 1)
                    self.speed_pct = self.SPEED_LEVELS[self.speed_idx]
                    self.get_logger().info(f'Speed: {self.speed_pct}%')

            # ===== MANUAL DRIVING (left stick — only in manual mode) =====
            if not self.auto_mode:
                max_lin = self.get_parameter('max_linear_speed').value   # 1.0
                max_ang = self.get_parameter('max_angular_speed').value  # 5.0
                scale   = self.speed_pct / 100.0

                # Left Stick: axes[1] = fwd/back, axes[0] = left/right
                raw_y = msg.axes[1] if len(msg.axes) > 1 else 0.0
                raw_x = msg.axes[0] if len(msg.axes) > 0 else 0.0

                # Software deadzone
                if abs(raw_y) < 0.12: raw_y = 0.0
                if abs(raw_x) < 0.12: raw_x = 0.0

                vx = raw_y * max_lin * scale
                vz = raw_x * max_ang * scale

                # Clamp to Rosmaster safe range
                vx = max(-1.0, min(1.0, vx))
                vz = max(-5.0, min(5.0, vz))

                # Publish cmd_vel (for dashboard / other nodes)
                cmd = Twist()
                cmd.linear.x = vx
                cmd.angular.z = vz
                self.cmd_vel_pub.publish(cmd)

                # Send to hardware
                try:
                    now = self.get_clock().now().nanoseconds / 1e9
                    if now - self._dbg_t > 0.5:
                        self.get_logger().info(
                            f"HW: vx={vx:.3f} vz={vz:.3f} "
                            f"(raw_y={raw_y:.2f} raw_x={raw_x:.2f} scale={scale})")
                        self._dbg_t = now
                    bot.set_car_motion(vx, 0.0, vz)
                except Exception as e:
                    self.get_logger().error(f"HW write error: {e}")

            # ======================================================
            # SERVO CONTROL — IDENTICAL TO MAIN BRANCH (lines 49-66)
            # ======================================================
            # msg.axes[3] is the Right Stick Left/Right (-1.0 to 1.0)
            # msg.axes[4] is the Right Stick Up/Down (-1.0 to 1.0)
            # Map [-1.0, 1.0] range to [0, 180] for the servo

            # Map Axis 3 (Right Stick L/R) to Servo 1 (S1)
            s1_val = int((msg.axes[3] * -1.0 + 1.0) * 90.0)

            # Map Axis 4 (Right Stick U/D) to Servo 2 (S2)
            s2_val = int((msg.axes[4] * -1.0 + 1.0) * 90.0)

            # Only send the command if the value has changed
            if s1_val != self.last_s1:
                bot.set_pwm_servo(1, s1_val)  # Control S1
                self.last_s1 = s1_val

            if s2_val != self.last_s2:
                bot.set_pwm_servo(2, s2_val)  # Control S2
                self.last_s2 = s2_val

            # ===== DASHBOARD INFO =====
            ctrl = String()
            ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
            self.dash_ctrl_pub.publish(ctrl)

            # Save for debouncing
            self.prev_buttons = list(msg.buttons)
            self.prev_axes = list(msg.axes)

            # Console debug — show raw axes so we can diagnose
            mode_s = "AUTO" if self.auto_mode else "MAN"
            st_s   = CHALLENGE_STATES[self.challenge_index]
            axes_s = ' '.join(f'{a:+.2f}' for a in msg.axes[:8]) if len(msg.axes) >= 6 else ''
            print(f"\r[V6] {mode_s} {st_s} Spd:{self.speed_pct}% S1:{self.last_s1} S2:{self.last_s2} [{axes_s}]   ", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')

    # ===== Auto-mode cmd_vel (from auto_driver) =====
    def cmd_vel_callback(self, msg):
        if self.auto_mode:
            try:
                bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
            except Exception as e:
                self.get_logger().error(f"Auto HW error: {e}")

    # ===== Odometry =====
    def publish_odom(self):
        try:
            vx, vy, wz = 0.0, 0.0, 0.0
            if hasattr(bot, 'get_motion_data'):
                try:
                    data = bot.get_motion_data()
                    if isinstance(data, (list, tuple)) and len(data) >= 3:
                        vx, vy, wz = float(data[0]), float(data[1]), float(data[2])
                    elif isinstance(data, (list, tuple)) and len(data) >= 2:
                        vx, wz = float(data[0]), float(data[1])
                except Exception:
                    pass
            if abs(vx) > 10.0 or abs(vy) > 10.0 or abs(wz) > 20.0:
                vx, vy, wz = 0.0, 0.0, 0.0

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = wz
            self.odom_pub.publish(odom)
        except Exception:
            pass


def main(args=None):
    global bot
    rclpy.init(args=args)
    servo_node = ServoControllerNode()
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        pass
    finally:
        # --- Cleanup (like main branch + extras) ---
        try:
            bot.set_car_motion(0, 0, 0)
            bot.set_pwm_servo(1, 90)
            bot.set_pwm_servo(2, 90)
            time.sleep(0.3)
        except Exception:
            pass
        try:
            del bot
            print("\nRosmaster object deleted, serial port closed.")
        except Exception:
            pass
        servo_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
