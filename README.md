# 🚀 ROS 2 SLAM + Nav2 Autonomous Navigation

A complete **ROS 2 Humble** simulation of a differential-drive robot that maps an
unknown maze with SLAM Toolbox, then localises and navigates it autonomously with
Nav2 — all in Gazebo Classic.

Two commands get you there. One brings up the whole SLAM pipeline, the other the
whole navigation pipeline; each starts Gazebo, the robot, the stack and a
preconfigured RViz together.

---

## What's in it

- Custom differential-drive robot (**slam_bot**) in URDF/Xacro
- 360° 2D LiDAR and wheel odometry via Gazebo plugins
- SLAM mapping with **SLAM Toolbox** (async)
- Map saving and reloading
- **AMCL** localisation
- Autonomous navigation with the full **Nav2** stack
- Keyboard teleoperation
- Preconfigured RViz layouts for mapping and for navigation
- Custom maze world

---

## The robot — slam_bot

| | |
|---|---|
| Drive | Differential, two powered wheels + rear caster |
| Sensor | 2D LiDAR, 360 samples, 0.12–10 m, 15 Hz |
| Odometry | Wheel odometry from the diff-drive plugin |
| Base frame | `base_footprint` |

### TF tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── left_wheel_link
                ├── right_wheel_link
                ├── caster_link
                └── lidar_link
```

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (Gazebo 11)

---

## Build

```bash
git clone https://github.com/devanshsingh0704/ros2_slam_nav2.git
cd ros2_slam_nav2
```

Install the ROS dependencies. Every package declares what it needs in its
`package.xml`, so `rosdep` resolves the lot:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build and source:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source the workspace in **every new terminal** you use below.

---

## Mapping a new area (SLAM)

**Terminal 1** — Gazebo, the robot, SLAM Toolbox and RViz:

```bash
ros2 launch robot_bringup slam_bringup.launch.py
```

**Terminal 2** — drive the robot around to build the map:

```bash
ros2 run robot_bringup teleop_keyboard
```

| Key | Action |
|---|---|
| `w` / `s` | forward / backward |
| `a` / `d` | turn left / right |
| `space` | stop |
| `q` | quit |

The robot stops on its own if no key is held, so it will not run away from you.
Watch the map fill in inside RViz.

**Terminal 3** — save the map once you are happy with it:

```bash
ros2 run nav2_map_server map_saver_cli -f maze_map
```

That writes `maze_map.pgm` and `maze_map.yaml`. To make it the map the navigation
launch uses, drop both into the package and rebuild:

```bash
cp maze_map.pgm maze_map.yaml src/robot_nav2/maps/
colcon build --symlink-install
```

---

## Navigating a saved map

A map of the maze is already included, so this works straight after building:

```bash
ros2 launch robot_bringup nav_bringup.launch.py
```

That starts Gazebo, the robot, map server, AMCL, planner, controller, behaviour
server, BT navigator and RViz.

Then, in RViz:

1. Click **2D Pose Estimate** and click-drag on the map where the robot actually
   is, pointing the arrow the way it faces. AMCL needs this before it can
   localise.
2. Click **2D Goal Pose** and click-drag anywhere you want it to drive to.
3. The planner draws a path, the controller follows it, and the costmaps update
   around obstacles as it goes.

---

## Running the pieces separately

The bringup launches above are the easy path. Each layer can also be run on its
own, which is handy when you are debugging one of them.

```bash
# Gazebo + robot only, no SLAM and no Nav2
ros2 launch robot_bringup bringup.launch.py

# SLAM Toolbox alone (needs the robot already running)
ros2 launch robot_slam slam.launch.py

# Map server + AMCL only, no path planning
ros2 launch robot_nav2 localization.launch.py

# The full Nav2 stack, no Gazebo
ros2 launch robot_nav2 nav2_bringup.launch.py
```

Every one of these resolves its own map, config and world paths from the
installed package, so none of them need a path argument. All default to
`use_sim_time:=true`.

---

## Packages

| Package | What it holds |
|---|---|
| `robot_description` | URDF/Xacro model, Gazebo sensor and diff-drive plugins |
| `robot_gazebo` | The maze world and its model |
| `robot_bringup` | The three bringup launches, RViz layouts, teleop node |
| `robot_slam` | SLAM Toolbox launch and tuning parameters |
| `robot_nav2` | Nav2 + AMCL launches, parameters, and the saved map |

### Where to tune things

| File | Controls |
|---|---|
| `src/robot_slam/config/slam_params.yaml` | SLAM resolution, loop closure, scan matching |
| `src/robot_nav2/config/nav2_params.yaml` | AMCL, costmaps, planner, controller, recovery |
| `src/robot_description/urdf/robot.urdf.xacro` | Robot geometry, LiDAR range, wheel sizes |
| `src/robot_nav2/maps/maze_map.yaml` | Map resolution and origin |

---

## Topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Localised pose |
| `/tf` | `tf2_msgs/TFMessage` | Frame transforms |

---

## Architecture

Full diagram on Miro:
https://miro.com/app/board/uXjVGEQIZ90=/?share_link_id=595685209121

```
URDF/Xacro  →  Gazebo  →  /scan + /odom + TF
                              │
                    ┌─────────┴─────────┐
              SLAM Toolbox          Map Server + AMCL
                    │                    │
                  /map              localised pose
                                         │
                                    Nav2: planner → controller → /cmd_vel
```

---

## Known limits

- Gazebo Classic only. Not ported to Gazebo Sim (Ignition).
- The bringup launches wait a fixed 3 s for Gazebo before spawning the robot and
  5 s before starting the stack. On a slow machine that may not be long enough;
  raise the `TimerAction` periods in `src/robot_bringup/launch/`.
- AMCL will not localise until you give it a **2D Pose Estimate** in RViz.
- The included map is of `maze_1` only. A different world needs a new map.
- Simulation only — no hardware driver layer.

---

## License

Apache-2.0. See [LICENSE](LICENSE).

---

## Author

**Devansh Kumar Singh**
https://github.com/devanshsingh0704
