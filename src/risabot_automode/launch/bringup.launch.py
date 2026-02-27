"""
Bringup Launch File — RISA-bot (Main Branch)
Minimal launch for individual node testing. Starts LiDAR + TF + auto_driver.
Camera, joystick, and line follower must be launched separately.
  Usage: ros2 launch risabot_automode bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    risabot_pkg = get_package_share_directory('risabot_automode')
    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')
    # --- Serial port mapping ---
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    return LaunchDescription([
        # --- A. YDLiDAR Tmini Plus (direct node — matches tested alias) ---
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': lidar_port,
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 4,
                'support_motor_dtr': True,
                'intensity': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 16.0,
                'range_min': 0.02,
                'frequency': 10.0,
            }],
        ),

        # --- B. TF Publisher ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # --- C. Motor Driver (Locked to 1a86 Chip) ---
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='risabot_automode',
                    executable='auto_driver',
                    name='auto_driver',
                    output='screen',
                    parameters=[params_file]
                )
            ]
        ),
        Node(
            package='risabot_automode',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[params_file]
        ),

        # Note: Camera (astra_camera), joystick (joy_node), and line_follower_camera
        # are launched separately via CLI aliases (see main_branch.md).
    ])
