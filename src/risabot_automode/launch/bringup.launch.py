"""
Bringup Launch File — RISA-bot (refactor-test)
Launches ALL nodes in one command — no separate terminals needed.
  Usage: ros2 launch risabot_automode bringup.launch.py
         ros2 launch risabot_automode bringup.launch.py slam:=false

SLAM (slam_toolbox + odom_tf_publisher) runs here so the dashboard on :8080 —
and therefore the companion app — can see the map. risabot_slam's own
slam_test.launch.py remains for standalone RViz debugging; the two cannot run
at once because both claim the LiDAR and the motor board.

Pass slam:=false to drop the scan matcher if it starves the camera/YOLO
pipeline on competition day.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    astra_pkg = get_package_share_directory('astra_camera')
    risabot_pkg = get_package_share_directory('risabot_automode')
    slam_pkg = get_package_share_directory('risabot_slam')
    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')
    slam_params = os.path.join(slam_pkg, 'config', 'mapper_params_online_async.yaml')

    slam = LaunchConfiguration('slam')

    # --- Disable FastRTPS shared memory to prevent /dev/shm corruption ---
    shm_xml = os.path.join(risabot_pkg, 'config', 'disable_shm.xml')

    # --- Serial port mapping ---
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    return LaunchDescription([

        DeclareLaunchArgument('slam', default_value='true'),

        # Disable shared memory transport (prevents DDS communication failures)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', shm_xml),

        # ==================== SENSORS ====================

        # A. Astra Mini Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_pkg, 'launch', 'astra_mini.launch.py')
            )
        ),

        # B. YDLiDAR Tmini Plus
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
                # 0.15, not 0.02: a 360 deg scanner 0.12 m above base_link ranges
                # the chassis itself. Those self-hits smear the SLAM map and poison
                # scan matching, and obstacle_avoidance was taking them as real
                # returns too -- it filters on the driver's declared range_min.
                'range_min': 0.15,
                'frequency': 10.0,
                'fixed_resolution': True,
                'reversion': True,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'invalid_range_is_inf': False,
                'abnormal_check_count': 4,
            }],
        ),

        # C. TF: base_link → laser_frame
        #
        # The yaw is pi, not 0. The LiDAR is mounted backwards -- its 0 deg points
        # to the rear of the car -- which params.yaml has recorded all along as
        # `lidar_angle_offset: 3.1416` on boom_gate_detector, tunnel_wall_follower
        # and obstruction_avoidance (and hardcoded in dashboard.py's scan callback).
        #
        # Those nodes subscribe to /scan and rotate the angles themselves, so they
        # never needed this transform to be right. slam_toolbox is the first
        # consumer that takes the mounting from TF instead, and with a zero yaw it
        # placed every return reflected through the robot: p_hat = 2t - p. A parked
        # robot still maps a plausible room that way (the reflection of a rectangle
        # is a rectangle), but the moment it translates, a stationary wall appears
        # to slide the same way the robot did at twice the speed, and the scan
        # matcher shreds the map trying to reconcile it.
        #
        # Changing this affects TF consumers only -- the perception nodes above
        # apply their own offset to the raw scan and are untouched.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            # positional form is: x y z yaw pitch roll frame_id child_frame_id
            arguments=['0', '0', '0.12', '3.14159265', '0', '0',
                       'base_link', 'laser_frame']
        ),

        # ==================== PERCEPTION ====================
        # Delayed 3s to give astra_camera time to start publishing

        # D. LiDAR obstacle detection
        TimerAction(period=3.0, actions=[
            Node(
                package='obstacle_avoidance',
                executable='obstacle_avoidance',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # E. Camera obstacle detection
        TimerAction(period=3.0, actions=[
            Node(
                package='obstacle_avoidance_camera',
                executable='obstacle_avoidance_camera',
                name='obstacle_avoidance_camera',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # F. Line follower camera (Cytron-style scanline detection)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='line_follower_camera',
                name='line_follower_camera',
                output='screen',
                parameters=[params_file]
            ),
        ]),


        # G1. Boom Gate Detector (LiDAR + Camera Red Bar)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='boom_gate_detector',
                name='boom_gate_detector',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # G2. Tunnel wall follower
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='tunnel_wall_follower',
                name='tunnel_wall_follower',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # G3. Signage detector (YOLOv5 BPU model)
        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='signage_detector',
                name='signage_detector',
                output='screen',
                parameters=[params_file]
            ),
        ]),

        # ==================== CONTROL ====================

        # H. Auto Driver (brain — delayed 5s to let sensors initialize)
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

        # I. Command safety controller
        Node(
            package='risabot_automode',
            executable='cmd_safety_controller',
            name='cmd_safety_controller',
            output='screen',
            parameters=[params_file]
        ),

        # J. Joystick driver
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

        # K. Servo controller (motor + steering hardware bridge)
        Node(
            package='control_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[params_file]
        ),

        # L. Health monitor
        Node(
            package='risabot_automode',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[params_file]
        ),

        # M. Dashboard (web UI at http://<robot_ip>:8080)
        Node(
            package='risabot_automode',
            executable='dashboard',
            name='dashboard',
            output='screen',
            parameters=[params_file]
        ),

        # N. go2rtc camera bridge (replaces OpenCV MJPEG server in dashboard.py)
        #    Serves MJPEG on :1985; go2rtc pulls from it and republishes on :1984.
        Node(
            package='risabot_automode',
            executable='ros2go2rtc_bridge',
            name='ros2go2rtc_bridge',
            output='screen',
            parameters=[{
                'active_view':  'raw',
                'jpeg_quality': 60,
                'resize_width': 320,
            }]
        ),

        # ==================== SLAM ====================

        # O. odom → base_link TF. The entire odometry interface to slam_toolbox,
        #    which consumes TF and /scan only. servo_controller broadcasts no TF
        #    itself, so this is the sole source of the edge -- tf2 silently drops
        #    a second transform sharing a stamp. Delayed 2 s so servo_controller
        #    is already publishing /odom/path_length and /imu/rpy.
        TimerAction(period=2.0, actions=[
            Node(
                package='risabot_slam',
                executable='odom_tf_publisher',
                name='odom_tf_publisher',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(slam),
            ),
        ]),

        # P. slam_toolbox (async: drops scans under load rather than blocking).
        #    Delayed 8 s -- after perception (3 s) and auto_driver (5 s) -- so the
        #    scan matcher is not competing with node startup for the X5's CPU.
        #
        #    respawn is what makes "Restart Mapping" possible. This build advertises
        #    no reset service (ros2 service list: save_map, pause_new_measurements,
        #    serialize/deserialize, clear_changes -- and nothing else), and there is
        #    no other way to clear the pose graph in place. So the dashboard clears
        #    it by signalling this process and letting launch bring it back with an
        #    empty map, which takes ~2 s instead of restarting the whole stack.
        #
        #    It is also the right setting on its own merits: slam_toolbox is the one
        #    node here that can be killed by the OOM reaper under memory pressure
        #    without taking the robot with it, and coming back beats staying dead.
        TimerAction(period=8.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params],
                condition=IfCondition(slam),
                respawn=True,
                respawn_delay=2.0,
            ),
        ]),
    ])
