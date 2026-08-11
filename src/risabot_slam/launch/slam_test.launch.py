"""
SLAM Test Launch File — RISA-bot.

Isolated slam_toolbox mapping stack. Brings up ONLY what mapping needs:
LiDAR + TF + teleop + odometry TF + slam_toolbox.

Deliberately excluded: astra_camera, auto_driver, cmd_safety_controller and the
dashboard. servo_controller starts in manual mode and drives the motors straight
from /joy, so teleop needs nothing else, and the camera + YOLO would starve the
X5's CPU and with it the scan matcher.

Stop the competition stack first — it holds the LiDAR serial port and ports
8080/1985:

    sudo systemctl stop risabot
    ros2 launch risabot_slam slam_test.launch.py

Arguments:
    restamp:=true     republish /scan with corrected stamps (see scan_restamper)
    reversion:=false  flip if the scan appears rotated 180 deg
    inverted:=false   flip if the scan appears mirrored left/right
    yaw_sign:=-1.0    flip if the IMU reports clockwise-positive yaw
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the isolated SLAM test launch description."""
    risabot_pkg = get_package_share_directory('risabot_automode')
    slam_pkg = get_package_share_directory('risabot_slam')

    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')
    shm_xml = os.path.join(risabot_pkg, 'config', 'disable_shm.xml')
    slam_params = os.path.join(slam_pkg, 'config', 'mapper_params_online_async.yaml')

    lidar_port = ('/dev/serial/by-id/'
                  'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')

    restamp = LaunchConfiguration('restamp')
    reversion = LaunchConfiguration('reversion')
    inverted = LaunchConfiguration('inverted')
    yaw_sign = LaunchConfiguration('yaw_sign')

    scan_topic = PythonExpression([
        "'/scan_restamped' if '", restamp, "' == 'true' else '/scan'"
    ])

    return LaunchDescription([

        DeclareLaunchArgument('restamp', default_value='false'),
        DeclareLaunchArgument('reversion', default_value='true'),
        DeclareLaunchArgument('inverted', default_value='true'),
        DeclareLaunchArgument('yaw_sign', default_value='1.0'),

        # Disable shared memory transport (prevents DDS communication failures)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),

        # ==================== SENSOR ====================

        # A. YDLiDAR Tmini Plus
        # Own node rather than bringup's so reversion/inverted can be corrected
        # here without touching the competition launch.
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
                # 0.15 not bringup's 0.02: a 360 deg scanner at 0.12 m ranges the
                # chassis, and self-hits smear the map and poison scan matching.
                'range_min': 0.15,
                'frequency': 10.0,
                'fixed_resolution': True,
                'reversion': ParameterValue(reversion, value_type=bool),
                'inverted': ParameterValue(inverted, value_type=bool),
                'auto_reconnect': True,
                'isSingleChannel': False,
                'invalid_range_is_inf': False,
                'abnormal_check_count': 4,
            }],
        ),

        # B. TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # C. Scan restamper (only when the driver's stamps are unusable)
        Node(
            package='risabot_slam',
            executable='scan_restamper',
            name='scan_restamper',
            output='screen',
            condition=IfCondition(restamp),
        ),

        # ==================== TELEOP ====================

        # D. Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }]
        ),

        # E. Servo controller — motor/steering bridge, and the source of
        #    /odom and /imu/rpy that odom_tf_publisher consumes.
        Node(
            package='control_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[params_file]
        ),

        # ==================== SLAM ====================

        # F. odom → base_link TF (the entire odometry interface to slam_toolbox)
        Node(
            package='risabot_slam',
            executable='odom_tf_publisher',
            name='odom_tf_publisher',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'yaw_sign': ParameterValue(yaw_sign, value_type=float),
                'publish_rate': 50.0,
                'max_odom_step': 0.5,
            }]
        ),

        # G. slam_toolbox — delayed 5s so /scan and the TF tree are live first
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'scan_topic': scan_topic}]
            ),
        ]),
    ])
