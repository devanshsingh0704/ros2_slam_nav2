# ROS2 SLAM + Nav2 Workspace

A complete ROS2 (Humble) based mobile robot project implementing:

- ✅ Custom Differential Drive Robot (slam_bot)
- ✅ URDF / Xacro Robot Modeling
- ✅ Gazebo Classic Simulation
- ✅ SLAM Mapping using SLAM Toolbox
- ✅ Autonomous Navigation using Nav2
- ✅ Teleoperation Control
- ✅ RViz Visualization

# Project Overview

This project demonstrates a fully simulated differential-drive mobile robot capable of:

- Performing SLAM (Simultaneous Localization and Mapping)
- Generating a 2D occupancy grid map
- Navigating autonomously using Nav2
- Publishing velocity commands via `/cmd_vel`
- Subscribing to `/scan`, `/odom`, and `/tf`
- Operating inside Gazebo Classic

The robot includes:

- Two drive wheels
- One caster wheel
- 2D LiDAR sensor
- Differential drive plugin
- Proper TF tree configuration


# Workspace Structure

ros2_slam_nav_ws
│
├── src
│   ├── robot_description  # URDF/Xacro robot model
│   ├── robot_gazebo       # Gazebo plugins & simulation
│   ├── robot_bringup      # Launch files & teleop
│   ├── robot_slam         # SLAM configuration
│   └── robot_nav2         # Navigation2 configuration
│
├── build      # Ignored in Git
├── install    # Ignored in Git
└── log        # Ignored in Git




# System Architecture

Full system architecture diagram:

🔗 https://miro.com/app/board/uXjVGEQIZ90=/?share_link_id=695027947490

The architecture includes:

- Robot Description Layer (URDF/Xacro)
- Gazebo Simulation Layer
- ROS2 Nodes and Topic Communication
- SLAM Toolbox Integration
- Navigation2 Stack
- Teleoperation Node
- TF Frame Hierarchy

---

# System Flow

1. Robot is spawned in Gazebo.
2. LiDAR publishes LaserScan data on `/scan`.
3. SLAM Toolbox subscribes to `/scan` and `/odom`.
4. A 2D occupancy grid map is generated.
5. Nav2 stack uses the map for localization and path planning.
6. Velocity commands are sent to `/cmd_vel`.
7. Differential drive plugin moves the robot.
8. TF frames update continuously for visualization in RViz.

---

# 🚀 How To Use

## 1️ Clone Repository

```bash
git clone https://github.com/devanshsingh0704/ros2_slam_nav2.git
cd ros2_slam_nav2
```

## 2️ Build Workspace

```bash
colcon build
```

## 3️ Source Workspace

```bash
source install/setup.bash
```

#  Launch Simulation

```bash
ros2 launch robot_bringup bringup.launch.py
```

#  Run SLAM

```bash
ros2 launch robot_slam slam.launch.py
```

# 🧭 Run Navigation (Nav2)

```bash
ros2 launch robot_nav2 nav2.launch.py
```

# 📡 ROS2 Topics Used

| Topic Name | Message Type | Purpose |
|------------|-------------|----------|
| /cmd_vel | geometry_msgs/Twist | Robot velocity control |
| /scan | sensor_msgs/LaserScan | LiDAR sensor data |
| /odom | nav_msgs/Odometry | Robot position & velocity |
| /tf | tf2_msgs/TFMessage | Coordinate frame transforms |

---

# Technologies Used

- ROS2 Humble
- Gazebo Classic
- RViz2
- SLAM Toolbox
- Nav2
- Python (ament_python)
- URDF / Xacro
- TF2


# Project Goal

To design and simulate a fully functional SLAM-capable mobile robot and implement autonomous navigation using the ROS2 Navigation stack in a structured and professional workspace.

---

# Author

**Devansh Kumar Singh**  
ROS2 SLAM & Navigation Developer  

GitHub: https://github.com/devanshsingh0704

# Notes

- `build/`, `install/`, and `log/` folders are excluded from version control.
- Developed and tested on Ubuntu 22.04 with ROS2 Humble.
- Designed for simulation-first development workflow.
