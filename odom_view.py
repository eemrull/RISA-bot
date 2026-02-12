import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import math

class OdomViewer(Node):
    def __init__(self):
        super().__init__('odom_view')
        self.create_subscription(Odometry, 'odom', self.listener_callback, 10)
        self.distance = 0.0
        self.last_time = time.time()

    def listener_callback(self, msg):
        # Calculate time passed since last message
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get raw speed from driver
        speed = msg.twist.twist.linear.x
        
        # Calculate distance: Distance = Previous Distance + (Speed * Time)
        self.distance += speed * dt
        
        # Translate to cm for human reading
        dist_cm = self.distance * 100.0
        
        print(f"\rDistance: {self.distance:.3f}m ({dist_cm:.1f} cm) | Speed: {speed:.2f} m/s", end='', flush=True)

def main():
    rclpy.init()
    rclpy.spin(OdomViewer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
