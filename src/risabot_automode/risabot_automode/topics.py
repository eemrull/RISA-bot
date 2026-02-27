"""Shared topic and frame constants for risabot_automode nodes."""

# Command and control
AUTO_CMD_VEL_TOPIC = '/cmd_vel_auto'
CMD_VEL_TOPIC = '/cmd_vel'
AUTO_MODE_TOPIC = '/auto_mode'
SET_CHALLENGE_TOPIC = '/set_challenge'

# Dashboard
DASH_STATE_TOPIC = '/dashboard_state'
DASH_CTRL_TOPIC = '/dashboard_ctrl'

# Perception inputs
LANE_ERROR_TOPIC = '/lane_error'
OBSTACLE_LIDAR_TOPIC = '/obstacle_front'
OBSTACLE_CAMERA_TOPIC = '/obstacle_detected_camera'
OBSTACLE_FUSED_TOPIC = '/obstacle_detected_fused'
TRAFFIC_LIGHT_TOPIC = '/traffic_light_state'
BOOM_GATE_TOPIC = '/boom_gate_open'
TUNNEL_DETECTED_TOPIC = '/tunnel_detected'
TUNNEL_CMD_TOPIC = '/tunnel_cmd_vel'
OBSTRUCTION_ACTIVE_TOPIC = '/obstruction_active'
OBSTRUCTION_CMD_TOPIC = '/obstruction_cmd_vel'

# Parking
PARKING_CMD_TOPIC = '/parking_command'
PARKING_VEL_TOPIC = '/parking_cmd_vel'
PARKING_COMPLETE_TOPIC = '/parking_complete'
PARKING_SIGN_TOPIC = '/parking_signboard_detected'
PARKING_STATUS_TOPIC = '/parking_status'

# Diagnostics
HEALTH_STATUS_TOPIC = '/health_status'

# Sensors
ODOM_TOPIC = '/odom'
JOY_TOPIC = '/joy'
CAMERA_IMAGE_TOPIC = '/camera/color/image_raw'
CAMERA_DEBUG_LINE_TOPIC = '/camera/debug/line_follower'
CAMERA_DEBUG_TL_TOPIC = '/camera/debug/traffic_light'
CAMERA_DEBUG_OBS_TOPIC = '/camera/debug/obstacle'

# Frames
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
