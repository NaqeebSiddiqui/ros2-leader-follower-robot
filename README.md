# ROS2 Leader–Follower Autonomous Robot

This project implements a leader–follower autonomous mobile robot using ROS2 and Python.

A second robot (follower) automatically tracks and follows a target robot (leader) using real-time pose feedback and a proportional controller.

---

## System Overview

The control loop follows the classical robotics pipeline:

Sense → Plan → Act

1. The follower subscribes to the leader robot's pose (sensor data)
2. Computes distance and angular error
3. Applies a feedback control law
4. Publishes velocity commands to the follower robot

---

## Features

- ROS2 (Jazzy) Python node
- Real-time topic communication
- Velocity command publishing (`cmd_vel`)
- Goal tolerance to prevent oscillation
- Closed-loop feedback control

---

## Technologies Used

- ROS2 Jazzy
- Ubuntu Linux
- Python (rclpy)
- turtlesim
- geometry_msgs

---

## How to Run

### 1. Start simulator
ros2 run turtlesim turtlesim_node

### 2. Control leader
ros2 run turtlesim turtle_teleop_key

### 3. Spawn follower
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 5.0, theta: 0.0, name: 'turtle2'}"

### 4. Run follower node
ros2 run turtle_follower follower

---

## Demonstration
The follower robot continuously tracks the leader and stabilizes near the target position using a proportional controller.

---

## Concepts Demonstrated
- ROS Nodes
- Publishers & Subscribers
- Topics & Messages
- Services
- Feedback Control
- Coordinate error calculation

---

## Future Improvements
- Path planning
- Obstacle avoidance
- SLAM integration
- Real robot deployment (TurtleBot3)
