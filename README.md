# 🚀 ROS2 SLAM + Nav2 Autonomous Navigation Workspace

A complete **ROS2 Humble** based mobile robot simulation project implementing:

- ✅ Custom Differential Drive Robot (**slam_bot**)
- ✅ URDF / Xacro Robot Modeling
- ✅ Gazebo Classic Simulation
- ✅ SLAM Mapping (SLAM Toolbox)
- ✅ Map Saving
- ✅ AMCL Localization
- ✅ Autonomous Navigation (Nav2)
- ✅ Teleoperation
- ✅ RViz Visualization
- ✅ Modular Launch Architecture

---

# 📌 Project Overview

This project demonstrates a fully simulated **differential-drive mobile robot** capable of:

- Performing SLAM (Simultaneous Localization and Mapping)
- Generating a 2D occupancy grid map
- Saving and loading maps
- Localizing using AMCL
- Planning and executing paths using Nav2
- Avoiding obstacles using costmaps
- Navigating autonomously to goal poses

The system is built entirely in **ROS2 Humble** and runs in **Gazebo Classic**.

---

# 🤖 Robot Model – slam_bot

## Robot Hardware (Simulated)

- Cylindrical Base  
- Two Drive Wheels  
- Rear Caster Wheel  
- 2D LiDAR Sensor  
- Differential Drive Plugin  
- Laser Plugin  
- Proper TF Frame Hierarchy  

## TF Tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                └── laser
```

---

# 📂 Workspace Structure

```
ros2_slam_nav_ws
│
├── src
│   ├── robot_description   # URDF/Xacro model
│   ├── robot_gazebo        # Gazebo plugins & world
│   ├── robot_bringup       # Main simulation launch
│   ├── robot_slam          # SLAM Toolbox config
│   └── robot_nav2          # Nav2 + AMCL configuration
│
├── build      # Ignored
├── install    # Ignored
└── log        # Ignored
```

---

# 🧠 System Architecture

🔗 **Full Architecture Diagram (Miro Board):**  
https://miro.com/app/board/uXjVGEQIZ90=/?share_link_id=595685209121

## Architecture Layers

1. Robot Description Layer (URDF/Xacro)
2. Gazebo Simulation Layer
3. Sensor & Odometry Layer
4. SLAM Toolbox Layer
5. Map Server
6. AMCL Localization
7. Navigation2 Stack
   - Planner Server
   - Controller Server
   - BT Navigator
   - Recovery Behaviors
8. RViz Visualization
9. TF Frame Hierarchy

---

# 🔄 FINAL Navigation & Localization Workflow

Below are the exact commands used for the fully working localization and navigation system.

---

## 🟢 Step 1 – Launch Robot + Gazebo

```bash
ros2 launch robot_bringup bringup.launch.py
```

---

## 🗺 Step 2 – Launch Localization (AMCL)

```bash
ros2 launch robot_nav2 localization.launch.py \
map:=/home/ubuntu/ros2_slam_nav_ws/src/robot_nav2/config/nav2_params.yaml
```

---

## 🧭 Step 3 – Launch RViz

```bash
ros2 launch nav2_bringup rviz_launch.py
```

---

## 🚀 Step 4 – Launch Nav2 Stack

```bash
ros2 launch nav2_bringup bringup_launch.py \
use_sim_time:=true \
map:=/home/ubuntu/ros2_slam_nav_ws/src/robot_nav2/config/nav2_params.yaml
```

---

# 📡 ROS2 Topics Used

| Topic | Type | Purpose |
|-------|------|----------|
| /cmd_vel | geometry_msgs/Twist | Velocity commands |
| /scan | sensor_msgs/LaserScan | LiDAR data |
| /odom | nav_msgs/Odometry | Robot odometry |
| /map | nav_msgs/OccupancyGrid | 2D Map |
| /amcl_pose | geometry_msgs/PoseWithCovarianceStamped | Localization |
| /tf | tf2_msgs/TFMessage | Frame transforms |

---

# 🔁 Complete System Flow

## 🔵 Localization Mode

1. Map is loaded  
2. AMCL estimates robot pose  
3. `/amcl_pose` updates continuously  
4. TF tree: `map → odom → base_link`  

## 🟣 Navigation Mode

1. Set Initial Pose in RViz  
2. Send 2D Goal Pose  
3. Nav2 Planner computes global path  
4. Controller computes velocity  
5. `/cmd_vel` moves robot  
6. Costmaps update dynamically  

---

# ⚙️ Technologies Used

- ROS2 Humble  
- Gazebo 11 (Classic)  
- RViz2  
- SLAM Toolbox  
- Nav2  
- AMCL  
- Map Server  
- Python (ament_python)  
- URDF / Xacro  
- TF2  

---

# 🎯 Project Goal

To design and simulate a fully autonomous SLAM-capable mobile robot and implement complete localization and navigation using the ROS2 Navigation Stack.

This workspace demonstrates:

- End-to-end SLAM pipeline  
- Map saving & loading  
- AMCL localization  
- Path planning & control  
- Obstacle avoidance  
- Proper ROS2 modular architecture  

---

# 🧪 Development Environment

- Ubuntu 22.04  
- ROS2 Humble  
- Gazebo Classic  
- VS Codium  
- Simulation-first workflow  

---

# 👨‍💻 Author

**Devansh Kumar Singh**  
ROS2 SLAM & Navigation Developer  

GitHub:  
https://github.com/devanshsingh0704  

---

# 📝 Notes

- `build/`, `install/`, and `log/` folders are excluded via `.gitignore`
- Fully tested localization + navigation workflow
- Ready for real robot adaptation
