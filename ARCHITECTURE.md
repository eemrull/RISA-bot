# RISA-Bot Architecture

Competition-mode autonomous vehicle built on ROS 2 Humble.

## Node / Topic Graph

```mermaid
graph TD
    subgraph Sensors
        CAM["Astra Mini Camera"]
        LIDAR["YDLiDAR Tmini Plus"]
        JOY["Joy Node"]
    end

    subgraph Perception
        LF["line_follower_camera"]
        OA_LID["obstacle_avoidance"]
        OA_CAM["obstacle_avoidance_camera"]
        TL["traffic_light_detector"]
        BG["boom_gate_detector"]
        TUN["tunnel_wall_follower"]
        OBS["obstruction_avoidance"]
        PARK["parking_controller"]
    end

    subgraph Control
        AD["auto_driver"]
        CSC["cmd_safety_controller"]
        SC["servo_controller"]
        HM["health_monitor"]
    end

    subgraph Interface
        DASH["dashboard (port 8080)"]
    end

    CAM -->|"/camera/color/image_raw"| LF
    CAM -->|"/camera/color/image_raw"| OA_CAM
    CAM -->|"/camera/color/image_raw"| TL
    CAM -->|"/camera/color/image_raw"| PARK

    LIDAR -->|"/scan"| OA_LID
    LIDAR -->|"/scan"| BG
    LIDAR -->|"/scan"| TUN
    LIDAR -->|"/scan"| OBS

    LF -->|"/lane_error"| AD
    OA_LID -->|"/obstacle_front"| AD
    OA_CAM -->|"/obstacle_detected_camera"| AD
    TL -->|"/traffic_light_state"| AD
    BG -->|"/boom_gate_open"| AD
    TUN -->|"/tunnel_detected + /tunnel_cmd_vel"| AD
    OBS -->|"/obstruction_active + /obstruction_cmd_vel"| AD
    PARK -->|"/parking_signboard_detected + /parking_cmd_vel + /parking_complete + /parking_status"| AD

    AD -->|"/cmd_vel_auto_raw"| CSC
    CSC -->|"/cmd_vel_auto"| SC
    AD -->|"/parking_command"| PARK
    AD -->|"/obstacle_detected_fused"| DASH
    JOY -->|"/joy"| SC
    JOY -->|"/joy"| DASH
    SC -->|"Rosmaster_Lib (serial)"| HW["Motor Board"]
    SC -->|"/cmd_vel"| DASH
    SC -->|"/auto_mode"| AD
    SC -->|"/odom"| DASH
    SC -->|"/odom"| AD

    AD -->|"/dashboard_state"| DASH
    SC -->|"/dashboard_ctrl"| DASH
    HM -->|"/health_status"| DASH
    CSC -->|"/cmd_safety_status + /loop_stats"| DASH
```

## State Machine (auto_driver)

| Priority | State          | Trigger               | Action                      |
| -------- | -------------- | --------------------- | --------------------------- |
| 1        | MANUAL         | `auto_mode=false`     | No cmd_vel published        |
| 2        | FINISHED       | Lap 2 + perpendicular done | Full stop              |
| 3        | TRAFFIC_LIGHT  | Red/yellow detected   | Full stop                   |
| 4        | BOOM_GATE      | Gate closed           | Full stop                   |
| 4.2      | EMERGENCY_STOP | `/cmd_safety_status` estop true | Full stop        |
| 5        | OBSTRUCTION    | LiDAR lateral avoid   | Use `/obstruction_cmd_vel`  |
| 5.5      | REVERSE_ADJUST | Too close to obstacle | Reverse slowly              |
| 6        | PARALLEL_PARK  | Lap 2 + signboard (latched) | Use `/parking_cmd_vel` |
| 6        | PERPENDICULAR_PARK | Parallel complete  | Use `/parking_cmd_vel`      |
| 6        | TUNNEL         | Walls on both sides   | Use `/tunnel_cmd_vel`       |
| 7        | LANE_FOLLOW    | Default               | Steering from `/lane_error` |

## Competition Flow

```
Lap 1: START → Lane Follow → Obstruction → Roundabout →
        BoomGate1 → Tunnel → BoomGate2 →
        Hill → Bumper → TrafficLight → START

Lap 2: Lane Follow → Obstruction → Roundabout →
        Parallel Park → Drive → Perpendicular Park → FINISH
```

## Key Files

| File                        | Purpose                                 |
| --------------------------- | --------------------------------------- |
| `auto_driver.py`            | Central state machine (brain)           |
| `servo_controller.py`       | Hardware interface to Rosmaster board   |
| `dashboard.py`              | Web dashboard server                    |
| `dashboard_templates.py`    | HTML/CSS/JS for dashboard UI            |
| `line_follower_camera.py`   | Lane detection via camera               |
| `traffic_light_detector.py` | R/Y/G circle detection                  |
| `obstruction_avoidance.py`  | LiDAR lateral steering around obstacles |
| `tunnel_wall_follower.py`   | PD wall following in tunnel             |
| `boom_gate_detector.py`     | LiDAR gate barrier detection            |
| `parking_controller.py`     | Odometry-based parking maneuvers        |
| `health_monitor.py`         | Topic freshness and runtime health      |
| `config/params.yaml`        | Centralized tunable parameters          |
