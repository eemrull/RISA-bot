# Module 1: Introduction to ROS 2

## Learning Objectives

By the end of this module, you will:

- Understand what ROS 2 is and why robots use it
- Know the key concepts: nodes, topics, messages, services
- Set up and build a ROS 2 workspace
- Run your first ROS 2 commands

## What is ROS 2?

**ROS** (Robot Operating System) is not an operating system — it's a **framework** that helps robot software components talk to each other.

Think of it like a messaging system:

- Each piece of your robot (camera, motor, AI brain) runs as a **separate program** called a **node**
- Nodes communicate by sending **messages** through named channels called **topics**
- This modular design means you can develop, test, and replace parts independently

### Why ROS 2?

| Without ROS                        | With ROS                             |
| ---------------------------------- | ------------------------------------ |
| One giant program does everything  | Each function is a separate node     |
| Change one thing, break everything | Change one node, others keep working |
| Hard to reuse code across robots   | Packages can be shared and reused    |
| Must write your own communication  | Built-in messaging system            |

## Key Concepts

```text
┌──────────┐     /topic_name     ┌──────────┐
│  Node A  │ ──── message ────▶  │  Node B  │
│ (camera) │                     │ (viewer) │
└──────────┘                     └──────────┘
```

| Concept       | What It Is                   | Example on RISA-bot            |
| ------------- | ---------------------------- | ------------------------------ |
| **Node**      | A running program            | `line_follower_camera`         |
| **Topic**     | A named message channel      | `/lane_error`                  |
| **Message**   | Data sent on a topic         | `Float32` (a number like 0.15) |
| **Package**   | A folder of related nodes    | `risabot_automode`             |
| **Workspace** | A folder containing packages | `~/risabotcar_ws`              |

## Hands-On: Your First ROS 2 Commands

### 1. Check ROS 2 is installed

```bash
ros2 --version
# Expected: ros2 humble (or your version)
```

### 2. Explore the workspace

```bash
cd ~/risabotcar_ws
ls src/
# You should see: risabot_automode, control_servo, obstacle_avoidance, etc.
```

### 3. Build the workspace

```bash
cd ~/risabotcar_ws
cb
sos
```

> **What just happened?**
>
> - `cb` compiles all packages in `src/` (alias for `colcon build --symlink-install`)
> - `sos` adds the compiled packages to your environment (alias for `source install/setup.bash`)

### 4. Run a demo node

```bash
# Terminal 1: Run a simple publisher (built into ROS 2)
ros2 run demo_nodes_py talker

# Terminal 2: Run a subscriber
ros2 run demo_nodes_py listener
```

You should see the listener printing messages from the talker!

### 5. Explore topics

```bash
# List all active topics
ros2 topic list

# See what a topic is publishing
ros2 topic echo /chatter

# Check how fast messages arrive
ros2 topic hz /chatter
```

## Exercise

1. Run the talker and listener demo
2. Use `ros2 topic list` to find the topic name
3. Use `ros2 topic echo` to see the messages
4. Stop the talker (Ctrl+C) — what happens to the listener?
5. Start the talker again — does the listener reconnect?

## Key Takeaways

- ROS 2 is a framework for robot software communication
- **Nodes** are programs, **topics** are channels, **messages** are data
- `cb` compiles, `sos` loads the environment
- Nodes can start and stop independently — the system is resilient

---

**Next:** [Module 2 — Nodes & Packages](02-nodes-and-packages.md)
