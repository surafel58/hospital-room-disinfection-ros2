# Hospital Room Disinfection Robot (ROS 2)

> An autonomous mobile robot that navigates hospital environments and performs UV-C / simulated disinfection using ROS 2 Humble and Gazebo Classic.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Robot Hardware Model](#robot-hardware-model)
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Launch Files Reference](#launch-files-reference)
- [Topics & Interfaces](#topics--interfaces)
- [Navigation Configuration](#navigation-configuration)
- [Roadmap](#roadmap)

---

## Overview

This project implements a fully simulated autonomous disinfection robot designed for hospital environments. The robot is capable of:

- Navigating pre-mapped hospital floor plans using ROS 2 Nav2.
- Localising itself inside the hospital using Adaptive Monte Carlo Localisation (AMCL).
- Performing or simulating UV-C light disinfection across designated areas.
- Detecting obstacles in real time and replanning paths dynamically.

The simulation runs in **Gazebo Classic** with a detailed, realistic hospital world model. The stack is built on **ROS 2 Humble** and is being progressively migrated to **ROS 2 Jazzy + Gazebo Harmonic**.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        ROS 2 Node Graph                          │
│                                                                  │
│  ┌─────────────────┐     /scan      ┌──────────────────────┐    │
│  │   Gazebo Sim    │ ─────────────► │   nav2_amcl (AMCL)   │    │
│  │  (ajrobot +     │               └──────────┬───────────┘    │
│  │  hospital world)│     /odom               │ /map → /odom TF  │
│  │                 │ ─────────────►          │                  │
│  │                 │               ┌──────────▼───────────┐    │
│  │                 │ ◄──────────── │   nav2 (Nav2 Stack)   │    │
│  │                 │   /cmd_vel    │  BT Navigator         │    │
│  └─────────────────┘               │  Controller Server    │    │
│                                    │  Planner Server (A*)  │    │
│  ┌─────────────────┐               │  Costmap (Local+Global│    │
│  │ robot_state_pub │               │  Recovery Behaviours  │    │
│  │  (URDF → TF)    │               └──────────────────────┘    │
│  └─────────────────┘                                            │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              disinfection_controller node                 │   │
│  │   Subscribes to navigation events, activates UV-C emitter│   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

### ROS 2 Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF/Xacro robot model, mesh assets, Gazebo plugin configs, display/Gazebo launch files |
| `simulation_world` | Gazebo world files (hospital environment), world launch files |
| `navigation` | Nav2 bringup, AMCL, map server, costmap parameters, hospital map (`.pgm` + `.yaml`), RViz2 configs |
| `disinfection_controller` | ROS 2 node that controls the UV-C disinfection mechanism |
| `object_detection` | Placeholder package for future object/person detection integration |

---

## Robot Hardware Model

The robot is named **ajrobot** — a custom differential-drive platform designed in CAD and exported as STL meshes. It includes the following physical links and sensors:

### Drivetrain
- **Differential drive** with left (`lwheel_1`) and right (`rwheel_1`) continuous-rotation wheels (wheel separation: 137.15 mm, wheel diameter: 54 mm).
- Two passive **caster wheels** (front and rear) for stability.

### Sensors

| Sensor | Type | ROS Topic(s) | Notes |
|---|---|---|---|
| 2D LiDAR | Ray sensor (180° FOV, 0.1–2.5 m) | `/scan`, `/ray/pointcloud`, `/ray/pointcloud2`, `/ray/range` | Used by AMCL and Nav2 costmap |
| RGB-D Camera | Depth camera (640×480, 30 fps, FOV 60°) | `/depth_camera/image_raw`, `/depth_camera/points`, `/camera_info` | Used for future object detection |
| IMU | 6-DOF inertial sensor (10 Hz) | `/imu` | Angular velocity + linear acceleration with Gaussian noise |
| PIR Sensor | Passive infrared (presence detection) | — | Mounted on tube frame for occupancy detection |

### Structural
- Tubular frame assembly modelling a UV-C lamp scaffold.
- Total robot mass: ~16.9 kg (base + wheels + frame + sensors).

---

## Features

- **Autonomous navigation** via ROS 2 Nav2 with A* global planner and DWB local planner.
- **Monte Carlo localisation** (AMCL) using LiDAR scan matching on a pre-built 2D occupancy map.
- **Hospital-grade world simulation** — two Gazebo worlds (`hosp.world`, `hospital_area.world`) modelling realistic hospital room layouts.
- **Pre-built hospital map** — a 0.05 m/pixel occupancy grid generated from the simulation.
- **Configurable navigation stack** — all Nav2 parameters (costmap layers, DWB critic weights, recovery behaviours) tunable via `nav2_params.yaml`.
- **Recovery behaviours** — automatic spin, back-up, and wait recoveries when the robot gets stuck.
- **3D sensor fusion-ready** — depth camera and LiDAR both publishing standard ROS 2 sensor messages for future integration.
- **Disinfection controller node** — modular Python node (`disinfection_node`) extensible to publish UV-C activation commands.
- **RViz2 visualisation** — custom `.rviz` configs for URDF inspection and navigation monitoring.
- **ROS-Gazebo bridge** — `ros_gz_bridge` integration for `cmd_vel`, keyboard teleoperation, camera, and depth camera topics.

---

## Tech Stack

| Component | Technology |
|---|---|
| Robot middleware | ROS 2 Humble Hawksbill |
| Simulator | Gazebo Classic (Gazebo 11) |
| Navigation | Nav2 (nav2_bringup, nav2_amcl, nav2_map_server, nav2_controller, nav2_planner, nav2_bt_navigator, nav2_recoveries) |
| Global planner | NavFn (A*) |
| Local planner | DWB (Dynamic Window Approach with critics) |
| Localisation | AMCL (Adaptive Monte Carlo Localisation) |
| Robot model | URDF / Xacro + STL meshes |
| Simulation plugins | `libgazebo_ros_diff_drive`, `libgazebo_ros_ray_sensor`, `libgazebo_ros_camera`, `libgazebo_ros_imu_sensor` |
| Programming language | Python 3 |
| Build system | colcon + ament_python |
| Visualisation | RViz2 |
| Bridging | ros_gz_bridge |

---

## Repository Structure

```
hospital-room-disinfection-ros2/
├── src/
│   ├── robot_description/            # Robot URDF model and Gazebo setup
│   │   ├── urdf/
│   │   │   ├── ajrobot.xacro         # Main robot description
│   │   │   ├── ajrobot.urdf          # Generated URDF
│   │   │   └── materials.xacro
│   │   ├── gazebo/
│   │   │   ├── ajrobot_detailed_plugins.gazebo
│   │   │   ├── ajrobot_detailed_physics.gazebo
│   │   │   └── ajrobot_detailed_materials.gazebo
│   │   ├── meshes/                   # STL mesh files
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py      # Full Gazebo + RViz2 launch
│   │   │   ├── hosp.launch.py        # Lightweight hospital world launch
│   │   │   ├── display.launch.py     # URDF visualisation in RViz2
│   │   │   └── controller.launch.py  # ros2_control controller launch
│   │   └── rviz/
│   │
│   ├── simulation_world/             # Gazebo world environments
│   │   ├── worlds/
│   │   │   ├── hospital_area.world
│   │   │   ├── hospital_area_modified.world
│   │   │   ├── hosp.world
│   │   │   └── hosp_modified.world
│   │   └── launch/
│   │
│   ├── navigation/                   # Nav2 navigation stack
│   │   ├── maps/
│   │   │   ├── hospital_map.pgm      # 2D occupancy grid map
│   │   │   └── hospital_map.yaml     # Map metadata (0.05 m/px)
│   │   ├── param/
│   │   │   ├── nav2_params.yaml      # Full Nav2 configuration
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   └── local_costmap_params.yaml
│   │   ├── launch/
│   │   │   ├── ajrobot_navigation.launch.py
│   │   │   ├── a.launch.py
│   │   │   ├── amcl.launch.py
│   │   │   └── move_base.launch.py
│   │   └── rviz/
│   │
│   ├── disinfection_controller/
│   │   ├── disinfection_controller/
│   │   │   └── disinfection_node.py
│   │   └── launch/
│   │       └── disinfection_launch.py
│   │
│   └── object_detection/             # Future: camera-based detection
│
└── frames_*.pdf / *.gv               # TF frame tree diagrams
```

---

## Prerequisites

Ensure the following are installed on **Ubuntu 22.04**:

- **ROS 2 Humble** — [Installation guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Classic (Gazebo 11)**
  ```bash
  sudo apt install gazebo
  ```
- **ROS 2 Nav2**
  ```bash
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  ```
- **Gazebo ROS packages**
  ```bash
  sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
  ```
- **Additional dependencies**
  ```bash
  sudo apt install ros-humble-joint-state-publisher \
                   ros-humble-robot-state-publisher \
                   ros-humble-xacro \
                   ros-humble-ros-gz-bridge \
                   ros-humble-nav2-map-server \
                   ros-humble-nav2-amcl \
                   python3-colcon-common-extensions
  ```

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/surafel58/hospital-room-disinfection-ros2.git
cd hospital-room-disinfection-ros2

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Install rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash
```

---

## Running the Simulation

Open a separate terminal for each step and run `source install/setup.bash` in each.

### Step 1 — Launch the Gazebo Simulation

```bash
ros2 launch robot_description gazebo.launch.py
```

Starts Gazebo Classic with the `hospital_area.world` environment, spawns **ajrobot**, and opens RViz2.

### Step 2 — Launch the Navigation Stack

```bash
ros2 launch navigation ajrobot_navigation.launch.py
```

Brings up Nav2, AMCL, map server, and RViz2 with the navigation config.

### Step 3 — Launch the Disinfection Controller

```bash
ros2 launch disinfection_controller disinfection_launch.py
```

### Step 4 — Send Navigation Goals

Use the **RViz2 "2D Nav Goal"** tool to set a target pose on the map, or publish programmatically:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}"
```

### Keyboard Teleoperation (Optional)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Launch Files Reference

| Launch File | Package | Description |
|---|---|---|
| `gazebo.launch.py` | `robot_description` | Gazebo + robot spawn + RViz2 (hospital_area world) |
| `hosp.launch.py` | `robot_description` | Gazebo + robot spawn (hosp world) |
| `display.launch.py` | `robot_description` | URDF viewer in RViz2 only |
| `controller.launch.py` | `robot_description` | ros2_control joint controllers |
| `ajrobot_navigation.launch.py` | `navigation` | Full Nav2 + AMCL + RViz2 |
| `a.launch.py` | `navigation` | Map server + AMCL + Nav2 composite |
| `amcl.launch.py` | `navigation` | AMCL standalone |
| `disinfection_launch.py` | `disinfection_controller` | Disinfection controller node |

---

## Topics & Interfaces

| Topic | Message Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan used by AMCL and costmap |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry from diff-drive plugin |
| `/imu` | `sensor_msgs/Imu` | IMU data |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from Nav2 |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | RGB-D camera stream |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | Depth point cloud |
| `/map` | `nav_msgs/OccupancyGrid` | Static hospital occupancy map |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Full transform tree |

---

## Navigation Configuration

Key parameters from `nav2_params.yaml`:

| Parameter | Value | Notes |
|---|---|---|
| Global planner | NavFn (A*) | `allow_unknown: true` |
| Local planner | DWB | RotateToGoal, PathAlign, GoalAlign critics |
| Max linear velocity | 0.26 m/s | Conservative for hospital corridors |
| Max angular velocity | 1.0 rad/s | |
| Robot radius | 0.22 m | |
| Inflation radius | 0.55 m | Keeps robot clear of walls |
| Local costmap size | 3×3 m rolling window | |
| Global costmap resolution | 0.05 m/px | Matches map resolution |
| AMCL particles | 500–2000 | Adaptive particle count |
| AMCL laser model | `likelihood_field` | |

---

## Roadmap

- [x] Custom differential-drive robot model (URDF/Xacro + STL meshes)
- [x] Gazebo Classic hospital world simulation
- [x] LiDAR, RGB-D camera, and IMU sensor plugins
- [x] ROS 2 Nav2 autonomous navigation with AMCL
- [x] Pre-built hospital occupancy map
- [x] Disinfection controller node scaffold
- [ ] UV-C coverage path planning (boustrophedon / coverage planner)
- [ ] Object/person detection to pause disinfection when room is occupied
- [ ] PIR sensor integration for real-time occupancy detection
- [ ] Multi-room waypoint mission execution
- [ ] Migration to ROS 2 Jazzy + Gazebo Harmonic
- [ ] Hardware deployment on physical robot

---

## Author

**Surafel Sentayehu** — [GitHub](https://github.com/surafel58).
