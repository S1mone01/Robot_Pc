# 🤖 iRobot Create 2 Simulation & PC Control System

A comprehensive ROS2-based control system for iRobot Roomba 670 using Create 2 drivers, featuring autonomous navigation, mapping, and simulation capabilities.

## 📋 Table of Contents
- [Prerequisites](#prerequisites)
- [System Overview](#system-overview)
- [PC Operations](#pc-operations)
- [Simulation Environment](#simulation-environment)
- [Navigation & Mapping](#navigation--mapping)
- [Advanced Features](#advanced-features)
- [Troubleshooting](#troubleshooting)

## 🛠 Prerequisites

- **ROS2 Jazzy** installed and configured
- **Ubuntu 24.04** (recommended)
- **Create 2 ROS2 drivers** for iRobot compatibility
- **Gazebo Harmonic** for simulation
- Network connectivity for remote operations

## 🔍 System Overview

This system transforms an **iRobot Roomba 670** into a fully controllable robot using Create 2 drivers, providing:
- Remote PC-based control and monitoring
- High-fidelity Gazebo simulation environment
- Autonomous navigation and mapping capabilities
- Real-time visualization with RViz2


## 🎮 Robot Control

### Manual Control Options

#### ⌨️ Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 🎮 Joystick Control
```bash
ros2 launch create_bringup joy_teleop.launch.py
```

## 🎮 Simulation Environment

### Full Gazebo Simulation
```bash
# Set up Gazebo environment
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share

# Launch complete simulation with GUI
ros2 launch gazebo robot.launch.py
```

### Headless Simulation
```bash
# For server or resource-constrained environments
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share

ros2 launch gazebo robot.launch.py headless:=true
```

## 🗺 Navigation & Mapping

### Simulation Navigation
```bash
# Navigate with pre-built map
ros2 launch create_bringup visual.py \
  map:=/home/simone/robot/src/create_robot/create_bringup/map/map_simulazione.yaml \
  use_sim_time:=true
```

### Simulation Mapping (SLAM)
```bash
# Start mapping in simulation
ros2 launch create_bringup visualM.py use_sim_time:=true

# Launch autonomous exploration
ros2 run custom_explorer explorer
```

### Map Management
```bash
# Save generated map
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
