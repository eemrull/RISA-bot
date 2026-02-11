#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Import the Joy message type

# --- Import your Rosmaster library ---
from Rosmaster_Lib import Rosmaster
import time

# --- Initialize the bot object globally ---
# This is exactly like your test code: bot = Rosmaster()
try:
    bot = Rosmaster()
    bot.set_car_type(1) # Set to Rosmaster X3
    print("Rosmaster Serial Opened!")
except Exception as e:
    print(f"Failed to initialize Rosmaster: {e}")
    print("Exiting. Make sure the robot is powered on and connected.")
    exit()

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Create a subscription to the /joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # Store the last servo values to avoid spamming the I2C bus
        self.last_s1 = 90
        self.last_s2 = 90
        
        self.get_logger().info('Servo Controller Node has started.')
        self.get_logger().info('Controls: Right Stick L/R -> Servo 1 | Right Stick U/D -> Servo 2')

    def joy_callback(self, msg):
        """This function is called every time a /joy message is received."""
        
        # --- MAPPING ---
        # msg.axes[3] is typically the Right Stick Left/Right (-1.0 to 1.0)
        # msg.axes[4] is typically the Right Stick Up/Down (-1.0 to 1.0)
        # We need to map this [-1.0, 1.0] range to [0, 180] for the servo
        
        # Note: Your joystick axes might be different!
        # Use 'jstest /dev/input/js0' to find your axis numbers.
        try:
            # Map Axis 3 (Right Stick L/R) to Servo 1 (S1)
            # (-1.0 to 1.0) -> (1.0 to -1.0) to invert
            # (1.0 to -1.0) + 1.0 -> (2.0 to 0.0)
            # (2.0 to 0.0) * 90.0 -> (180 to 0)
            s1_val = int((msg.axes[3] * -1.0 + 1.0) * 90.0)
            
            # Map Axis 4 (Right Stick U/D) to Servo 2 (S2)
            s2_val = int((msg.axes[4] * -1.0 + 1.0) * 90.0)

            # Only send the command if the value has changed
            if s1_val != self.last_s1:
                bot.set_pwm_servo(1, s1_val) # Control S1
                self.last_s1 = s1_val
            
            if s2_val != self.last_s2:
                bot.set_pwm_servo(2, s2_val) # Control S2
                self.last_s2 = s2_val
            
            # Log the values to the terminal
            self.get_logger().info(f'S1: {s1_val}  S2: {s2_val}', throttle_duration_sec=0.1)

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
        # --- This is your cleanup code! ---
        # Center the servos before quitting
        bot.set_pwm_servo(1, 90)
        bot.set_pwm_servo(2, 90)
        time.sleep(0.5)
        
        # Delete the object just like in your test code
        del bot
        print("Rosmaster object deleted, serial port closed.")
        
        servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
