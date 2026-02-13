# RISA-bot Guide

Quick reference documentation for operating and tuning the robot.

![Competition Course Layout](competition_layout.jpeg)

## Branch Guides

| Guide | Description |
|---|---|
| [Main Branch](Main/) | Individual node testing, original controller |
| [Test Branch](Test/) | Competition mode — state machine, all 9 challenges |

## Common References

| Guide | Description |
|---|---|
| [Challenge Breakdown](challenges_breakdown.md) | Deep dive into each challenge's code logic |
| [Commands Reference](commands_reference.md) | All ROS topics, launch files, aliases, parameters |
| [Tuning Guide](tuning_guide.md) | Step-by-step physical course tuning |
| [Architecture](architecture.md) | Node graphs, data flow, package structure |

## Quick Start

| I want to... | Branch | Command |
|---|---|---|
| Test individual nodes | `main` | `run_risabot` |
| Full competition run | `test` | `ros2 launch risabot_automode competition.launch.py` |
| Drive with joystick only | `main` | `ros2 launch control_servo robot_rc.launch.py` |
