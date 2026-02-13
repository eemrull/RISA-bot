#!/usr/bin/env python3

import subprocess
import atexit
import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# --- Configuration ---
STEER_AXIS = 2      # Right Stick Left/Right
MOTOR_AXIS = 1      # Left Stick Up/Down
DPAD_AXIS_UD = 7    # D-Pad Vertical Axis
Y_BUTTON = 3        # Y Button index
X_BUTTON = 2        # Trigger parking

JOY_DEADZONE = 0.08
TIMEOUT_SEC = 1.0   # Safety: Stop if no signal for 1 second

# --- Speed Levels (Gears) ---
SPEED_LEVELS = [25, 50, 75, 100]
TURN_BOOST = 20

# --- Asymmetrical Steering Config ---
ANGLE_CENTER = 90
SWING_LEFT = 50
SWING_RIGHT = 50
ANGLE_MAX_LEFT = ANGLE_CENTER - SWING_LEFT
ANGLE_MAX_RIGHT = ANGLE_CENTER + SWING_RIGHT


class JoyRobotController(Node):

    def __init__(self):
        super().__init__('joy_robot_controller')
        self.bot = Rosmaster()

        # Initialize Steering
        self.current_angle = ANGLE_CENTER
        self.bot.set_pwm_servo(4, self.current_angle)

        # Speed state
        self.speed_index = 1
        self.current_speed = SPEED_LEVELS[self.speed_index]
        self.dpad_pressed = False

        # Obstacle state
        self.obstacle_detected = False

        # Mode state
        self.auto_mode_active = False

        # Mode toggle debounce
        self.last_mode_toggle_time = 0
        self.mode_toggle_delay = 1.0

        # State machine for smooth transitions
        self.state = "LANE_FOLLOWING"
        self.dodge_start_time = 0.0
        self.dodge_duration = 1.7
        self.latest_scan = None
        self.dodge_dir = 1

        # Publisher for mode state
        self.mode_pub = self.create_publisher(Bool, '/auto_mode', 10)

        # Lane tracking
        self.lane_error_history = []
        self.history_size = 8
        self.current_lane_error = 0.0
        self.prev_lane_error = 0.0
        self.derivative_gain = 12  # Optimized damping
        self.last_valid_error = 0.0
        self.lane_visible = False
        self.turn_direction = 0  # -1=left, 0=straight, 1=right
        self.turn_start_time = 0.0
        self.min_turn_duration = 0.8  # seconds to keep turning

        self.dodge_in_progress = False
        # Add this with your other lane tracking variables
        self.steering_buffer = []
        self.buffer_size = 3  # ~60ms delay at 50Hz

        self.in_roundabout = False
        self.roundabout_start_time = 0.0

        # Auto-start joy_node
        try:
            topics = self.get_topic_names_and_types()
            joy_exists = any(name == '/joy' for name, _ in topics)
            if not joy_exists:
                self.get_logger().info('Starting internal joy_node...')
                self.joy_process = subprocess.Popen(
                    ['ros2', 'run', 'joy', 'joy_node']
                )
                atexit.register(self.joy_process.terminate)
                time.sleep(1)
            else:
                self.get_logger().info('/joy topic already active.')
        except Exception as e:
            self.get_logger().warn(f'Failed to start joy_node: {e}')

        # Safety watchdog
        self.last_joy_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.joystick_signal_lost_logged = False

        # Subscribers
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_front', self.obstacle_callback, 10
        )

        self.lane_sub = self.create_subscription(
            Float32, '/lane_error', self.lane_callback, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        self.drive_timer = self.create_timer(0.02, self.drive_motors)

        self.get_logger().info(
            f'Ackermann Controller Started. Speed Level: {self.current_speed}'
        )

    def lane_callback(self, msg):
        current_error = msg.data

        # Check if lane is visible (error within reasonable range)
        if abs(current_error) < 0.8:
            self.lane_error_history.append(current_error)
            if len(self.lane_error_history) > self.history_size:
                self.lane_error_history.pop(0)
            self.last_valid_error = current_error
            self.lane_visible = True
        else:
            # Lane not visible - mark as invisible
            self.lane_visible = False

        # Use historical data if available
        if self.lane_error_history:
            self.current_lane_error = float(np.mean(self.lane_error_history[-3:]))
        else:
            self.current_lane_error = self.last_valid_error

    def watchdog_callback(self):
        if self.auto_mode_active:
            return

        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_joy_time).nanoseconds / 1e9

        if time_diff > TIMEOUT_SEC:
            if not self.joystick_signal_lost_logged:
                self.get_logger().warn('⚠️ Joystick Signal Lost! Stopping Robot...')
                self.joystick_signal_lost_logged = True
            self.set_motor_safe(0)
        else:
            if self.joystick_signal_lost_logged:
                self.get_logger().info('✅ Joystick signal restored.')
                self.joystick_signal_lost_logged = False

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

        if not self.auto_mode_active:
            return

        # Only start dodge if not already dodging
        if msg.data and not self.dodge_in_progress:
            self.get_logger().warn('🛑 Obstacle detected! Starting dodge...')
            self.decide_dodge_direction()
            self.state = "DODGING_LEFT" if self.dodge_dir == -1 else "DODGING_RIGHT"
            self.dodge_start_time = time.time()
            self.dodge_in_progress = True  # 🔑 Lock state

    def decide_dodge_direction(self):
        if not self.latest_scan or len(self.latest_scan) != 2:
            self.dodge_dir = 1
            self.get_logger().warn('⚠️ No LiDAR data — defaulting to RIGHT')
            return

        angles, distances = self.latest_scan
        min_left = float('inf')
        min_right = float('inf')

        for angle, dist in zip(angles, distances):
            if dist < 0.1 or dist > 5.0:
                continue
            if 30 <= math.degrees(angle) <= 90:
                min_left = min(min_left, dist)
            elif -90 <= math.degrees(angle) <= -30:
                min_right = min(min_right, dist)

        if min_left > 2.0 and min_right > 2.0:
            self.dodge_dir = 1
        elif min_left > min_right:
            self.dodge_dir = -1
        else:
            self.dodge_dir = 1

    def scan_callback(self, msg):
        angles = []
        distances = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            if math.isnan(r) or math.isinf(r):
                continue

            angle = angle_min + i * angle_increment
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            angles.append(angle)
            distances.append(r)

        self.latest_scan = (angles, distances)

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

        # ALWAYS allow Y button to toggle mode (critical!)
        current_time = time.time()
        if len(msg.buttons) > Y_BUTTON and msg.buttons[Y_BUTTON] == 1:
            if current_time - self.last_mode_toggle_time >= self.mode_toggle_delay:
                self.auto_mode_active = not self.auto_mode_active
                if self.auto_mode_active:
                    self.state = "LANE_FOLLOWING"
                    self.get_logger().info('✅ Switched to AUTO MODE')
                else:
                    self.get_logger().info('🎮 Switched to JOYSTICK MODE')
                self.last_mode_toggle_time = current_time

        # Handle X button: trigger PARKING (only in auto mode)
        if self.auto_mode_active:
            if len(msg.buttons) > X_BUTTON and msg.buttons[X_BUTTON] == 1:
                if current_time - self.last_mode_toggle_time >= self.mode_toggle_delay:
                    if not self.in_parking:
                        self.in_parking = True
                        self.parking_start_time = time.time()
                        self.get_logger().info(f'🅿️ Automatic parking started ({self.parking_duration}s)')
                    self.last_mode_toggle_time = current_time

        # Only process other controls if in manual mode
        if self.auto_mode_active:
            return

        self.joystick_signal_lost_logged = False

        # --- GEAR SHIFTING (D-Pad) ---
        if len(msg.axes) > DPAD_AXIS_UD:
            dpad_val = msg.axes[DPAD_AXIS_UD]
            if abs(dpad_val) < 0.1:
                self.dpad_pressed = False
            elif not self.dpad_pressed:
                if dpad_val > 0.5 and self.speed_index < len(SPEED_LEVELS) - 1:
                    self.speed_index += 1
                    self.current_speed = SPEED_LEVELS[self.speed_index]
                    self.get_logger().info(f'Shift UP -> Level {self.current_speed}')
                    self.dpad_pressed = True
                elif dpad_val < -0.5 and self.speed_index > 0:
                    self.speed_index -= 1
                    self.current_speed = SPEED_LEVELS[self.speed_index]
                    self.get_logger().info(f'Shift DOWN -> Level {self.current_speed}')
                    self.dpad_pressed = True

        # --- STEERING ---
        target_angle = ANGLE_CENTER
        if len(msg.axes) > STEER_AXIS:
            joy_val = msg.axes[STEER_AXIS]
            if joy_val > JOY_DEADZONE:
                target_angle -= joy_val * SWING_LEFT
            elif joy_val < -JOY_DEADZONE:
                target_angle += (-joy_val) * SWING_RIGHT

        target_angle = int(max(ANGLE_MAX_LEFT, min(ANGLE_MAX_RIGHT, target_angle)))
        if target_angle != self.current_angle:
            self.bot.set_pwm_servo(4, target_angle)
            self.current_angle = target_angle

        # --- MOTOR POWER ---
        motor_power = 0
        if len(msg.axes) > MOTOR_AXIS:
            joy_val_motor = msg.axes[MOTOR_AXIS]
            if abs(joy_val_motor) > JOY_DEADZONE:
                base_speed = self.current_speed
                # Apply turn boost if steering
                is_turning = abs(target_angle - ANGLE_CENTER) > 10
                boosted_speed = min(base_speed + (TURN_BOOST if is_turning else 0), 255)
                motor_power = boosted_speed if joy_val_motor > 0 else -base_speed

        self.set_motor_safe(motor_power)

    def set_motor_safe(self, power):
        current_time = time.time()

        if not hasattr(self, 'last_motor_time'):
            self.last_motor_time = 0

        if current_time - self.last_motor_time >= 0.05:
            self.bot.set_motor(power, 0, 0, 0)
            self.last_motor_time = current_time

    def _interpolate_angle(self, current, target, factor):
        """Smoothly transition between angles"""
        return current + (target - current) * factor

    def drive_motors(self):
        if not self.auto_mode_active:
            return

        target_angle = ANGLE_CENTER
        motor_power = 40

        if self.state.startswith("DODGING"):
            # 🚨 OBSTACLE AVOIDANCE HAS FULL CONTROL - NO LANE FOLLOWER INTERFERENCE
            elapsed = time.time() - self.dodge_start_time
            if elapsed < self.dodge_duration:
                dodge_target = ANGLE_CENTER + (self.dodge_dir * 45)
                target_angle = self._interpolate_angle(self.current_angle, dodge_target, 0.3)
                motor_power = 45
            else:
                # Dodge complete - return to lane following
                self.state = "LANE_FOLLOWING"
                self.get_logger().info('✅ Back to lane following')
                self.dodge_in_progress = False  # 🔑 Release lock

                # Use current lane error for immediate correction
                error_abs = abs(self.current_lane_error)
                if error_abs < 0.03:
                    target_angle = ANGLE_CENTER
                elif error_abs < 0.15:
                    gain = 25
                    derivative = self.current_lane_error - self.prev_lane_error
                    self.prev_lane_error = self.current_lane_error
                    damped_error = self.current_lane_error - (derivative * 0.10)
                    target_angle = ANGLE_CENTER + int(damped_error * gain)
                else:
                    # Aggressive recovery
                    gain = 60
                    derivative = self.current_lane_error - self.prev_lane_error
                    self.prev_lane_error = self.current_lane_error
                    damped_error = self.current_lane_error - (derivative * 0.12)
                    target_angle = ANGLE_CENTER + int(damped_error * gain)
                motor_power = 40

        else:  # LANE_FOLLOWING state
            if not self.lane_visible:
                self.current_lane_error *= 0.95

            error_abs = abs(self.current_lane_error)

            if error_abs < 0.03:
                target_angle = ANGLE_CENTER
            elif error_abs < 0.15:
                gain = 30
                derivative = self.current_lane_error - self.prev_lane_error
                self.prev_lane_error = self.current_lane_error
                damped_error = self.current_lane_error - (derivative * 0.10)
                target_angle = ANGLE_CENTER + int(damped_error * gain)
            else:
                # Large error handling with turn persistence
                new_direction = 1 if self.current_lane_error > 0 else -1
                if new_direction != self.turn_direction:
                    self.turn_direction = new_direction
                    self.turn_start_time = time.time()

                current_time = time.time()
                if (self.turn_direction != 0 and 
                    current_time - self.turn_start_time < self.min_turn_duration):
                    target_angle = ANGLE_CENTER + (45 * self.turn_direction)
                else:
                    gain = 60
                    derivative = self.current_lane_error - self.prev_lane_error
                    self.prev_lane_error = self.current_lane_error
                    damped_error = self.current_lane_error - (derivative * 0.12)
                    target_angle = ANGLE_CENTER + int(damped_error * gain)

                if error_abs < 0.05:
                    self.turn_direction = 0

            motor_power = 40

        # === START OF ROUNDABOUT CODE ===
        # Roundabout detection
        if not self.in_roundabout:
            # If consistently turning left for >1.5 seconds, assume roundabout
            if self.current_lane_error < -0.25:  # Strong left error
                if not hasattr(self, 'left_turn_start'):
                    self.left_turn_start = time.time()
                elif time.time() - self.left_turn_start > 1.5:
                    self.in_roundabout = True
                    self.roundabout_start_time = time.time()
                    self.get_logger().info('🔄 Entering roundabout — locking left turn')
            else:
                # Reset timer if not turning left
                if hasattr(self, 'left_turn_start'):
                    delattr(self, 'left_turn_start')
        else:
            # Already in roundabout — stay in left turn
            target_angle = ANGLE_CENTER - 45  # Full left
            motor_power = 40
            
            # Exit after ~3-4 seconds (adjust based on roundabout size)
            if time.time() - self.roundabout_start_time > 3.5:
                self.in_roundabout = False
                self.get_logger().info('✅ Exiting roundabout')
            return  # Skip normal steering
        # === END OF ROUNDABOUT CODE ===

        # === START OF AUTOMATIC PARKING LOGIC ===
        if self.in_parking:
            elapsed = time.time() - self.parking_start_time
            if elapsed < self.parking_duration:
                self.bot.set_pwm_servo(4, ANGLE_CENTER)
                self.set_motor_safe(0)
                return
            else:
                self.in_parking = False
                self.get_logger().info('▶️ Auto-resume after parking')
        # === END OF AUTOMATIC PARKING LOGIC ===

        # Apply steering
        target_angle = max(ANGLE_MAX_LEFT, min(ANGLE_MAX_RIGHT, target_angle))
        if abs(target_angle - self.current_angle) > 1:
            self.bot.set_pwm_servo(4, int(target_angle))
            self.current_angle = target_angle

        self.set_motor_safe(motor_power)

def main(args=None):
    rclpy.init(args=args)
    node = JoyRobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bot.set_motor(0, 0, 0, 0)
        node.bot.set_pwm_servo(4, ANGLE_CENTER)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
