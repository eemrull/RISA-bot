#!/usr/bin/env python3
"""
Servo Controller + Mode Switch Node  (V4)
=========================================
Controls:
  Left Stick Y  (axes[1])  → Throttle (forward/back)
  Right Stick X (axes[3])  → Steering (left/right)
  Right Stick Y (axes[4])  → Camera tilt servo (S2)
  Start/Options (btn 11)   → Toggle AUTO / MANUAL
  LB / RB       (btn 6/7)  → Cycle challenge state
  D-pad ▲/▼     (axes[7])  → Speed level [25,50,75,100]%

Topic map:
  Publishes:  /auto_mode, /cmd_vel, /set_challenge, /dashboard_ctrl, /odom
  Subscribes: /joy, /cmd_vel_auto
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from Rosmaster_Lib import Rosmaster
import time

# ============================================================
# Global bot — initialised before any ROS node starts
# ============================================================
bot = None
try:
    bot = Rosmaster()
    bot.set_car_type(1)
    bot.set_car_motion(0, 0, 0)
    print("Rosmaster Serial Opened!")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting. Make sure the robot is powered on and connected.")
    exit()

# Challenge states (matches ChallengeState enum in auto_driver)
CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]

# Joystick axis deadzone (applied in software on top of joy_node deadzone)
STICK_DEADZONE = 0.15


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # ---- Publishers ----
        self.auto_mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.cmd_vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.challenge_pub = self.create_publisher(String, '/set_challenge', 10)
        self.dash_ctrl_pub = self.create_publisher(String, '/dashboard_ctrl', 10)
        self.odom_pub      = self.create_publisher(Odometry, '/odom', 10)

        # ---- Subscribers ----
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.cmd_vel_callback, 10)

        # ---- Rosmaster receive thread (odometry) ----
        try:
            bot.create_receive_threading()
            self.get_logger().info("Rosmaster receive thread started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start receive thread: {e}")

        # ---- Timers ----
        self.create_timer(0.1, self.publish_odom)        # 10 Hz odometry
        self.create_timer(0.05, self.startup_stop_spam)   # 20 Hz stop-spam (first 2s)

        # ---- State ----
        self.auto_mode = False
        self.last_s1 = 90
        self.last_s2 = 90
        self.challenge_index = 0
        self.prev_buttons = []
        self.prev_axes = []
        self.got_first_joy = False  # Flag to stop the startup spam
        self.start_time = time.time()

        # ---- Speed levels ----
        self.SPEED_LEVELS = [25, 50, 75, 100]
        self.speed_idx = 1          # Start at 50%
        self.speed_pct = self.SPEED_LEVELS[self.speed_idx]

        # ---- Parameters ----
        # Rosmaster set_car_motion expects values roughly in range -100..100
        self.declare_parameter('max_linear_speed', 50.0)
        self.declare_parameter('max_angular_speed', 80.0)
        self.declare_parameter('toggle_button', 11)       # Start / Options
        self.declare_parameter('prev_state_button', 6)    # LB
        self.declare_parameter('next_state_button', 7)    # RB

        # ---- Startup banner ----
        self.get_logger().info('*** SERVO CONTROLLER V4 ***')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick Y  → Throttle (fwd/back)')
        self.get_logger().info('  Right Stick X → Steering (left/right)')
        self.get_logger().info('  D-pad ▲/▼     → Speed [25,50,75,100]%')
        self.get_logger().info('  Start         → Toggle AUTO/MANUAL')
        self.get_logger().info('  LB/RB         → Cycle challenge state')
        self.get_logger().info(f'  Mode: MANUAL | Speed: {self.speed_pct}%')
        self.get_logger().info(f'  max_lin={self.get_parameter("max_linear_speed").value}  max_ang={self.get_parameter("max_angular_speed").value}')

        # Publish initial dashboard state
        self.auto_mode_pub.publish(Bool(data=self.auto_mode))
        ctrl = String()
        ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
        self.dash_ctrl_pub.publish(ctrl)

    # ----------------------------------------------------------------
    # Startup stop-spam: repeatedly send stop until first joy arrives
    # ----------------------------------------------------------------
    def startup_stop_spam(self):
        """Send stop commands for the first 2 seconds to clear residual board state."""
        elapsed = time.time() - self.start_time
        if self.got_first_joy or elapsed > 2.0:
            return  # Stop spamming once joy arrives or 2s elapsed
        try:
            bot.set_car_motion(0, 0, 0)
        except Exception:
            pass

    # ----------------------------------------------------------------
    # Helper
    # ----------------------------------------------------------------
    def _button_pressed(self, msg, idx):
        """Rising-edge detection for a button."""
        if idx >= len(msg.buttons):
            return False
        cur = msg.buttons[idx]
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return cur == 1 and prev == 0

    @staticmethod
    def _deadzone(val, dz=STICK_DEADZONE):
        """Zero out small stick values."""
        return 0.0 if abs(val) < dz else val

    # ----------------------------------------------------------------
    # Joy callback
    # ----------------------------------------------------------------
    def joy_callback(self, msg):
        toggle_btn = self.get_parameter('toggle_button').value
        prev_btn   = self.get_parameter('prev_state_button').value
        next_btn   = self.get_parameter('next_state_button').value
        self.got_first_joy = True

        try:
            # ===== AUTO / MANUAL TOGGLE =====
            if self._button_pressed(msg, toggle_btn):
                self.auto_mode = not self.auto_mode
                label = "🤖 AUTO" if self.auto_mode else "🎮 MANUAL"
                self.get_logger().info(f'Mode: {label}')
                self.auto_mode_pub.publish(Bool(data=self.auto_mode))

            # ===== CHALLENGE STATE CYCLING =====
            if self._button_pressed(msg, next_btn):
                self.challenge_index = (self.challenge_index + 1) % len(CHALLENGE_STATES)
                name = CHALLENGE_STATES[self.challenge_index]
                self.get_logger().info(f'Challenge → {name}')
                self.challenge_pub.publish(String(data=name))
            if self._button_pressed(msg, prev_btn):
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

            # ===== MANUAL DRIVING =====
            if not self.auto_mode:
                max_lin = self.get_parameter('max_linear_speed').value   # ~50
                max_ang = self.get_parameter('max_angular_speed').value  # ~80
                scale   = self.speed_pct / 100.0

                # ROS2 joy_node axis mapping (SDL2 backend):
                #   0 = Left Stick X      1 = Left Stick Y
                #   2 = Left Trigger       3 = Right Stick X
                #   4 = Right Stick Y      5 = Right Trigger
                #   6 = D-pad X            7 = D-pad Y
                throttle = 0.0
                steering = 0.0

                if len(msg.axes) > 1:
                    throttle = self._deadzone(msg.axes[1]) * max_lin * scale
                if len(msg.axes) > 3:
                    steering = self._deadzone(msg.axes[3]) * max_ang * scale

                # Publish cmd_vel for dashboard / other nodes
                cmd = Twist()
                cmd.linear.x = throttle
                cmd.angular.z = steering
                self.cmd_vel_pub.publish(cmd)

                # Drive hardware
                try:
                    now = self.get_clock().now().nanoseconds / 1e9
                    if not hasattr(self, '_dbg_t'):
                        self._dbg_t = 0
                    if now - self._dbg_t > 0.5:
                        self.get_logger().info(
                            f"HW: thr={throttle:.1f} steer={steering:.1f} "
                            f"(axes1={msg.axes[1]:.2f} axes3={msg.axes[3]:.2f} "
                            f"scale={scale:.2f})")
                        self._dbg_t = now
                    bot.set_car_motion(throttle, 0, steering)
                except Exception as e:
                    self.get_logger().error(f"HW write fail: {e}")

            # ===== CAMERA TILT SERVO (Right Stick Y → S2) =====
            if len(msg.axes) > 4:
                ry = self._deadzone(msg.axes[4])
                s2_val = int((ry * -1.0 + 1.0) * 90.0)
                if s2_val != self.last_s2:
                    bot.set_pwm_servo(2, s2_val)
                    self.last_s2 = s2_val

            # ===== DASHBOARD INFO =====
            ctrl = String()
            ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
            self.dash_ctrl_pub.publish(ctrl)

            # Save states for debouncing
            self.prev_buttons = list(msg.buttons)
            self.prev_axes = list(msg.axes)

            # Console debug
            mode_s = "AUTO" if self.auto_mode else "MAN"
            st_s   = CHALLENGE_STATES[self.challenge_index]
            pressed = [i for i, b in enumerate(msg.buttons) if b == 1]
            btn_s   = f' Btns:{pressed}' if pressed else ''
            axes_s  = ' '.join(f'{a:.2f}' for a in msg.axes[:6]) if len(msg.axes) >= 6 else ''
            print(f"\r[V4] {mode_s} {st_s} Spd:{self.speed_pct}% S2:{self.last_s2} Axes:[{axes_s}]{btn_s}    ", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'joy_callback error: {e}')

    # ----------------------------------------------------------------
    # Auto-mode cmd_vel
    # ----------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        if self.auto_mode:
            try:
                bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
            except Exception as e:
                self.get_logger().error(f"Auto HW error: {e}")

    # ----------------------------------------------------------------
    # Odometry publisher
    # ----------------------------------------------------------------
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
            # Reject garbage
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


# ================================================================
# main
# ================================================================
def main(args=None):
    global bot                        # ← prevents UnboundLocalError
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bot.set_car_motion(0, 0, 0)   # Stop motors
            bot.set_pwm_servo(1, 90)       # Centre servos
            bot.set_pwm_servo(2, 90)
            time.sleep(0.3)
        except Exception:
            pass
        try:
            del bot
            print("\nRosmaster closed.")
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
