# RISA-bot Guide

Quick reference documentation for operating and tuning the robot.

## Contents

| Guide | Description |
|---|---|
| [Main Branch](main_branch.md) | How `main` works — individual node testing, original controller |
| [Test Branch](test_branch.md) | How `new-control-code` works — competition state machine, all 9 challenges |
| [Challenge Code Breakdown](challenges_breakdown.md) | **Deep dive** into the code logic for each challenge (Traffic, Parking, etc.) |
| [Commands Reference](commands_reference.md) | All ROS topics, launch files, useful commands |
| [Tuning Guide](tuning_guide.md) | Step-by-step parameter tuning on the physical course |
| [Architecture](architecture.md) | Node graphs, data flow, how it all fits together |

## Which Branch?

| I want to... | Branch | Command |
|---|---|---|
| Test individual nodes | `main` | `run_risabot` |
| Test to competition run | `new-control-code` | `ros2 launch risabot_automode competition.launch.py` |
| Drive with joystick only | `main` | `ros2 launch control_servo robot_rc.launch.py` |
