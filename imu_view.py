import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuViewer(Node):
    def __init__(self):
        super().__init__('imu_view')
        # Subscribe to the topic published by auto_driver
        self.create_subscription(Imu, 'imu/data', self.listener_callback, 10)

    def listener_callback(self, msg):
        # 1. Get Quaternion from message
        q = msg.orientation
        
        # 2. Convert Quaternion to Euler Angles (The Math)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. Convert Radians to Degrees
        roll_deg = roll * (180.0 / math.pi)
        pitch_deg = pitch * (180.0 / math.pi)
        yaw_deg = yaw * (180.0 / math.pi)

        # 4. Print nicely
        # \033[K clears the line so it doesn't clutter the terminal
        print(f"heading (Yaw): {yaw_deg:.1f}°  |  Tilt (Pitch): {pitch_deg:.1f}°  |  Lean (Roll): {roll_deg:.1f}°      ", end='\r')

def main():
    rclpy.init()
    node = ImuViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
