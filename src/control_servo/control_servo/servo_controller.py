#!/usr/bin/env python3
"""
Servo Controller + Mode Switch Node  (V5 — matches main-branch axis mapping)
=============================================================================
Rosmaster_Lib.set_car_motion(vx, vy, vz):
    vx: -1.0 .. 1.0  m/s   (forward/back)
    vy: -1.0 .. 1.0  m/s   (lateral — unused for Ackermann)
    vz: -5.0 .. 5.0  rad/s (rotation)

ROS2 joy_node (SDL2) axis mapping  (confirmed from main branch):
    0 = Left Stick X        1 = Left Stick Y
    2 = Left Trigger         3 = Right Stick X
    4 = Right Stick Y        5 = Right Trigger
    6 = D-pad X              7 = D-pad Y

Controls:
    Left Stick Y  (axes[1])  → Throttle   via set_car_motion vx
    Left Stick X  (axes[0])  → Steering   via set_car_motion vz
    Right Stick X (axes[3])  → Front Servo S1  (set_pwm_servo 1)
    Right Stick Y (axes[4])  → Front Servo S2  (set_pwm_servo 2)
    Start         (btn 11)   → AUTO / MANUAL toggle
    LB / RB       (btn 6/7)  → Cycle challenge state
    D-pad ▲/▼     (axes[7])  → Speed level [25,50,75,100]%
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
# Global bot
# ============================================================
bot = None
try:
    bot = Rosmaster()
    bot.set_car_type(1)
    bot.set_car_motion(0, 0, 0)        # stop motors immediately
    print("Rosmaster Serial Opened!")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting.")
    exit()

CHALLENGE_STATES = [
    'LANE_FOLLOW', 'OBSTRUCTION', 'ROUNDABOUT',
    'BOOM_GATE_1', 'TUNNEL', 'BOOM_GATE_2',
    'HILL', 'BUMPER', 'TRAFFIC_LIGHT',
    'PARALLEL_PARK', 'DRIVE_TO_PERP', 'PERPENDICULAR_PARK'
]

# Software deadzone — sticks below this magnitude are treated as 0
DEADZONE = 0.12


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

        # ---- Rosmaster receive thread ----
        try:
            bot.create_receive_threading()
            self.get_logger().info("Rosmaster receive thread started.")
        except Exception as e:
            self.get_logger().error(f"Receive thread error: {e}")

        # Send stop again AFTER threading is up (clears any residual velocity)
        try:
            bot.set_car_motion(0, 0, 0)
        except Exception:
            pass

        # ---- Timers ----
        self.create_timer(0.1, self.publish_odom)        # 10 Hz odom
        self._stop_spam_timer = self.create_timer(0.05, self._startup_stop)  # 20 Hz stop spam

        # ---- State ----
        self.auto_mode = False
        self.last_s1 = 90
        self.last_s2 = 90
        self.challenge_index = 0
        self.prev_buttons = []
        self.prev_axes = []
        self._got_joy = False
        self._start_t = time.time()
        self._dbg_t = 0.0

        # ---- Speed ----
        self.SPEED_LEVELS = [25, 50, 75, 100]
        self.speed_idx = 1          # start at 50%
        self.speed_pct = self.SPEED_LEVELS[self.speed_idx]

        # ---- Parameters ----
        # Rosmaster range: vx ±1.0 m/s, vz ±5.0 rad/s
        self.declare_parameter('max_linear_speed', 1.0)   # full range of motor
        self.declare_parameter('max_angular_speed', 5.0)   # full range of rotation
        self.declare_parameter('toggle_button', 11)
        self.declare_parameter('prev_state_button', 6)
        self.declare_parameter('next_state_button', 7)

        self.get_logger().info('*** SERVO CONTROLLER V5 ***')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick Y  → Throttle (set_car_motion vx)')
        self.get_logger().info('  Left Stick X  → Steering (set_car_motion vz)')
        self.get_logger().info('  Right Stick   → Servo S1/S2 (axes[3]/axes[4])')
        self.get_logger().info('  D-pad ▲/▼     → Speed [25,50,75,100]%')
        self.get_logger().info(f'  max_lin={self.get_parameter("max_linear_speed").value}  max_ang={self.get_parameter("max_angular_speed").value}')
        self.get_logger().info(f'  Mode: MANUAL | Speed: {self.speed_pct}%')

        # Publish initial
        self.auto_mode_pub.publish(Bool(data=False))
        ctrl = String()
        ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
        self.dash_ctrl_pub.publish(ctrl)

    # ---- Startup: spam stop commands until first joy msg or 2s ----
    def _startup_stop(self):
        if self._got_joy or (time.time() - self._start_t) > 2.0:
            return
        try:
            bot.set_car_motion(0, 0, 0)
        except Exception:
            pass

    # ---- Helpers ----
    def _btn_pressed(self, msg, idx):
        if idx >= len(msg.buttons):
            return False
        cur = msg.buttons[idx]
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return cur == 1 and prev == 0

    @staticmethod
    def _dz(val, dz=DEADZONE):
        return 0.0 if abs(val) < dz else val

    # ================================================================
    # JOY CALLBACK
    # ================================================================
    def joy_callback(self, msg):
        toggle_btn = self.get_parameter('toggle_button').value
        prev_btn   = self.get_parameter('prev_state_button').value
        next_btn   = self.get_parameter('next_state_button').value
        self._got_joy = True

        try:
            # ===== AUTO / MANUAL =====
            if self._btn_pressed(msg, toggle_btn):
                self.auto_mode = not self.auto_mode
                label = "🤖 AUTO" if self.auto_mode else "🎮 MANUAL"
                self.get_logger().info(f'Mode: {label}')
                self.auto_mode_pub.publish(Bool(data=self.auto_mode))
                if not self.auto_mode:
                    # Immediately stop motors when switching to manual
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

            # ===== D-PAD SPEED =====
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

            # ===== MANUAL DRIVING (left stick) =====
            if not self.auto_mode:
                max_lin = self.get_parameter('max_linear_speed').value   # 1.0
                max_ang = self.get_parameter('max_angular_speed').value  # 5.0
                scale   = self.speed_pct / 100.0

                # axes[1] = Left Stick Y (forward/back)
                # axes[0] = Left Stick X (steering left/right)
                raw_y = self._dz(msg.axes[1]) if len(msg.axes) > 1 else 0.0
                raw_x = self._dz(msg.axes[0]) if len(msg.axes) > 0 else 0.0

                vx = raw_y * max_lin * scale     # forward/back   [-1.0,1.0]
                vz = raw_x * max_ang * scale     # steering        [-5.0,5.0]

                # Clamp to Rosmaster safe range
                vx = max(-1.0, min(1.0, vx))
                vz = max(-5.0, min(5.0, vz))

                cmd = Twist()
                cmd.linear.x = vx
                cmd.angular.z = vz
                self.cmd_vel_pub.publish(cmd)

                try:
                    now = self.get_clock().now().nanoseconds / 1e9
                    if now - self._dbg_t > 0.5:
                        self.get_logger().info(
                            f"HW: vx={vx:.3f} vz={vz:.3f} | "
                            f"raw_y={raw_y:.2f} raw_x={raw_x:.2f} | "
                            f"scale={scale:.2f}")
                        self._dbg_t = now
                    bot.set_car_motion(vx, 0.0, vz)
                except Exception as e:
                    self.get_logger().error(f"HW error: {e}")

            # ===== FRONT SERVOS (right stick — matches main branch) =====
            #   axes[3] = Right Stick X → S1 (Left/Right)
            #   axes[4] = Right Stick Y → S2 (Up/Down)
            if len(msg.axes) > 4:
                rx = msg.axes[3]
                ry = msg.axes[4]
                s1_val = int((rx * -1.0 + 1.0) * 90.0)   # [-1,1] → [180,0]
                s2_val = int((ry * -1.0 + 1.0) * 90.0)

                s1_val = max(0, min(180, s1_val))
                s2_val = max(0, min(180, s2_val))

                if s1_val != self.last_s1:
                    bot.set_pwm_servo(1, s1_val)
                    self.last_s1 = s1_val
                if s2_val != self.last_s2:
                    bot.set_pwm_servo(2, s2_val)
                    self.last_s2 = s2_val

            # ===== DASHBOARD INFO =====
            ctrl = String()
            ctrl.data = f'{self.speed_pct}|{self.challenge_index}|{CHALLENGE_STATES[self.challenge_index]}'
            self.dash_ctrl_pub.publish(ctrl)

            # Save for debouncing
            self.prev_buttons = list(msg.buttons)
            self.prev_axes = list(msg.axes)

            # Console — show ALL raw axes for debugging
            mode_s = "AUTO" if self.auto_mode else "MAN"
            st_s   = CHALLENGE_STATES[self.challenge_index]
            axes_s = ' '.join(f'{a:+.2f}' for a in msg.axes[:8]) if len(msg.axes) >= 6 else ''
            print(f"\r[V5] {mode_s} {st_s} Spd:{self.speed_pct}% S1:{self.last_s1} S2:{self.last_s2} [{axes_s}]   ", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'joy_callback error: {e}')

    # ================================================================
    # AUTO-MODE cmd_vel (from auto_driver)
    # ================================================================
    def cmd_vel_callback(self, msg):
        if self.auto_mode:
            try:
                bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
            except Exception as e:
                self.get_logger().error(f"Auto HW error: {e}")

    # ================================================================
    # ODOMETRY
    # ================================================================
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
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bot.set_car_motion(0, 0, 0)
            bot.set_pwm_servo(1, 90)
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
