# RISA-bot Workshop

Hands-on workshop modules for learning ROS 2 using the RISA-bot platform.

## Prerequisites

- Ubuntu 22.04 (on robot or VM)
- ROS 2 Humble installed
- SSH access to the robot (`ssh risabot`)
- Basic Python and Linux terminal knowledge

## Modules

| #   | Topic                                                  | Duration | Description                             |
| --- | ------------------------------------------------------ | -------- | --------------------------------------- |
| 1   | [Intro to ROS 2](01-intro-to-ros2.md)                  | 30 min   | What is ROS, workspace setup, building  |
| 2   | [Nodes & Packages](02-nodes-and-packages.md)           | 45 min   | Create a package, write your first node |
| 3   | [Topics & Messages](03-topics-and-messages.md)         | 45 min   | Publishers, subscribers, message types  |
| 4   | [Launch Files](04-launch-files.md)                     | 30 min   | Multi-node launch, parameters           |
| 5   | [Working with Sensors](05-working-with-sensors.md)     | 45 min   | Camera and LiDAR data                   |
| 6   | [Joystick Control](06-joystick-control.md)             | 30 min   | Reading joystick, driving the robot     |
| 7   | [Computer Vision Basics](07-computer-vision-basics.md) | 60 min   | OpenCV, HSV filtering, line detection   |
| 8   | [Obstacle Detection](08-obstacle-detection.md)         | 45 min   | LiDAR processing, obstacle avoidance    |
| 9   | [Putting It Together](09-putting-it-together.md)       | 60 min   | Combining modules into a system         |

## How to Use

- Each module is **self-contained** — pick the ones relevant to your workshop
- Modules build on each other sequentially, but you can skip ahead
- All exercises use the RISA-bot hardware and codebase
- Code examples reference actual files in this repository

## Setup Before Workshop

```bash
# On the robot
cd ~/risabotcar_ws
git checkout main && git pull
cb
sos
```
