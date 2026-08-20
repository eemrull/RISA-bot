# RISA-bot Documentation & Operator Guides

Quick reference documentation, hardware testing protocols, and tuning guides for operating RISA-bot.

![Competition Course Layout](competition_layout_overview.jpeg)

---

## 📖 Available Guides

| Guide | Document | Key Topics Covered |
|---|---|---|
| **Architecture** | [architecture.md](architecture.md) | Node communication graph, 17-state machine, perception pipelines, network ports |
| **Commands Reference** | [commands_reference.md](commands_reference.md) | Launch files, ROS 2 CLI commands, live param setting, topic monitors, aliases |
| **Physical Course Tuning** | [tuning_guide.md](tuning_guide.md) | Step-by-step physical course parameter calibration order, symptoms, and fixes |
| **Challenge Breakdown** | [challenges_breakdown.md](challenges_breakdown.md) | In-depth engineering breakdown of all 9 competition challenges and state logic |
| **Hardware Deployment** | [deployment_guide.md](deployment_guide.md) | Full setup checklist for new robot hardware, udev rules, BPU model deployment |
| **Odometry Guide** | [Odometer_Guide.md](Odometer_Guide.md) | Hardware encoder odometry, Ackermann yaw kinematics, calibration, and TF |
| **Competition Validation** | [competition_validation_guide.md](competition_validation_guide.md) | Hardware acceptance criteria, failsafe verification, and rosbag regression testing |
| **BPU AI Training** | [bpu_model_training_guide.md](bpu_model_training_guide.md) | Google Colab YOLOv5s training, ONNX patching, and Docker BPU quantization |

---

## 🚀 Which Launch File Should I Use?

| Task / Use Case | Launch Command | Subsystems Launched |
|---|---|---|
| **Full Competition Run** (Recommended) | `ros2 launch risabot_automode bringup.launch.py` | Camera, LiDAR, IMU, Joystick, All Perception, Brain, Safety, Dashboard, go2rtc, SLAM |
| **Full Run Without SLAM** (Low CPU) | `ros2 launch risabot_automode bringup.launch.py slam:=false` | Full autonomous stack with scan matching disabled |
| **Lane Follower Testing** | `ros2 launch risabot_automode lane_test.launch.py` | Camera, Joystick, Servo Controller, Line Follower, Auto Driver |
| **SLAM Mapping Testing** | `ros2 launch risabot_slam slam_test.launch.py` | LiDAR, Servo Controller, odom TF publisher, SLAM Toolbox |
| **Manual RC Driving** | `ros2 launch control_servo robot_rc.launch.py` | Joystick driver, Servo Controller hardware bridge |
| **Multi-Terminal GUI Debug** | `run_risabot` | Spawns separate desktop tabs for each node |
| **Multi-Window SSH Debug** | `run_trisabot` | Spawns a tmux session with windows for each node |
