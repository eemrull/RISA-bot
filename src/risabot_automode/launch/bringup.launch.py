import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path to Tmini Plus file
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    tmini_launch_path = os.path.join(ydlidar_pkg, 'launch', 'tmini_plus.launch.py')

    # 2. DEFINITIVE PORTS (Based on your screenshot)
    # The Silicon Labs chip is ALWAYS the LiDAR
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    
    # The 1a86 chip is ALWAYS the Motor Board
    motor_port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'

    return LaunchDescription([
        # --- A. LiDAR (Locked to Silicon Labs Chip) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tmini_launch_path),
            launch_arguments={
                'port': lidar_port,              # <--- Connect ONLY to Silicon Labs
                'frame_id': 'laser_frame',
                'ignore_array': motor_port,      # <--- STRICTLY IGNORE the Motor Chip
                'support_motor_dtr': 'true'
            }.items()
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
                    output='screen'
                )
            ]
        )
    ])
