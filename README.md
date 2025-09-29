
# 🤖 ROS2 Robot Control System

Fully integrated robotic control system based on ROS2 Jazzy, designed for iRobot 670 with Create 2 driver. Supports autonomous navigation, real-time mapping, camera integration, and remote control with LDS02RR LiDAR.

----------

## 📋 Table of Contents

-   [System Requirements](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-system-requirements)
-   [Quick Start](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-quick-start)
-   [Gazebo Simulation](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-gazebo-simulation)
-   [Launch Modes](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-launch-modes)
-   [Mapping Operations](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-mapping-operations)
-   [Robot Control](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-robot-control)
-   [Zone Management System](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-zone-management-system)
-   [Localization](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-localization)
-   [Diagnostics and Troubleshooting](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-diagnostics-and-troubleshooting)
-   [Additional Resources](https://claude.ai/chat/1d2bef63-79c5-42d9-b94a-6ca683edf56b#-additional-resources)

----------

## 🛠 System Requirements

### Required Software

-   **ROS2 Jazzy** installed and configured
-   **Ubuntu 24.04** (recommended)
-   **Gazebo** for simulation

### Supported Hardware

-   iRobot 670 with Create 2 driver
-   LDS02RR LiDAR
-   Camera (optional)
-   Joystick (optional for manual control)

----------

## 🚀 Quick Start

### 1. Environment Setup

Before any operation, configure the ROS2 environment:

```bash
source ~/robot/install/setup.bash
```

### 2. Installation Verification

Check that all nodes are available:

```bash
ros2 node list
ros2 topic list
```

----------

## 🎮 Gazebo Simulation

### Environment Variables Configuration

Before launching Gazebo, set the resource paths:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
```

### Simulation Modes

#### Full Interface

Launch Gazebo with graphical interface:

```bash
ros2 launch gazebo robot.launch.py
```

#### Headless Mode

For simulations without GUI (resource saving):

```bash
ros2 launch gazebo robot.launch.py headless:=true
```

----------

## 🗺️ Launch Modes

### Complete Navigation System

Full launch with navigation for simulation:

```bash
ros2 launch create_bringup visual.py map:=map_simulazione.yaml use_sim_time:=true
```

**Parameters:**

-   `map`: Map file name (without path)
-   `use_sim_time`: Use simulation time instead of real time

### AI-Enhanced System

Complete system with all features, including ROS bridge for MCP:

```bash
ros2 launch create_bringup create_2.py \
  camera:=true \
  navigation:=true \
  foxglove:=true \
  rosbridge:=true \
  map:=map_file.yaml
```

**Active Features:**

-   `camera`: Activates camera integration
-   `navigation`: Enables Nav2 navigation system
-   `foxglove`: Activates Foxglove Bridge for visualization
-   `rosbridge`: Enables ROS Bridge for MCP connection
-   `map`: Specifies the map to use

----------

## 🗺️ Mapping Operations

### 1. Launch Mapping Mode

Launch the robot in mapping configuration:

```bash
ros2 launch create_bringup visualM.py use_sim_time:=true
```

### 2. Autonomous Exploration

Start automatic exploration to create the map:

```bash
ros2 run custom_explorer explorer
```

The robot will autonomously explore the environment and build the map in real-time.

### 3. Save Map

Once exploration is complete, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

This will create two files:

-   `<map_name>.yaml` - Configuration file
-   `<map_name>.pgm` - Map image

### Map Management

**Storage path:**

```
~/robot/src/create_bringup/map/
```

**Best Practices:**

-   Use descriptive names for maps
-   Keep backups of important maps
-   Verify file permissions after saving

----------

## 🎮 Robot Control

### Manual Control

#### Keyboard

Keyboard control with on-screen instructions:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Main commands:**

-   `i`: Forward
-   `k`: Stop
-   `j`: Rotate left
-   `l`: Rotate right
-   `u/o`: Diagonal movements
-   `q/z`: Increase/decrease speed

#### Joystick

Controller control:

```bash
ros2 launch create_bringup joy_teleop.launch.py
```

----------

## 🛡 Zone Management System

The system allows defining special zones on the map through a point-and-click interface.

### How to Create a Zone

1.  Launch the zone maker (in exclusion or speed limit mode)
2.  Click 4 points on the map to define the polygon
    -   **Topic:** `/clicked_point`
    -   **Message Type:** `geometry_msgs/msg/PointStamped`
3.  The zone is created automatically

### Exclusion Zones

Areas the robot must avoid (no-go zones):

```bash
ros2 launch create_bringup zone_maker.py
```

**Usage:**

-   Block dangerous areas
-   Protect fragile objects
-   Delimit private spaces

### Speed Reduction Zones

Areas with custom speed limits:

```bash
ros2 launch create_bringup zone_maker.py node:=speed slow_value:=40
```

**Parameters:**

-   `node`: Zone type (`speed` for speed limit)
-   `slow_value`: Percentage of maximum speed (1-99%)

**Examples:**

```bash
# Very slow zone (20% maximum speed)
ros2 launch create_bringup zone_maker.py node:=speed slow_value:=20

# Moderate zone (60% maximum speed)
ros2 launch create_bringup zone_maker.py node:=speed slow_value:=60
```


----------

## 📍 Localization

Launch the robot's localization system on the map:

```bash
ros2 launch create_bringup localization.py
```

This allows the robot to:

-   Determine its position on the map

----------

## 🔍 Diagnostics and Troubleshooting

### System Verification Commands

#### List Active Topics

```bash
ros2 topic list
```

#### Monitor Specific Topic

```bash
ros2 topic echo /topic_name
```

**Important topics:**

-   `/cmd_vel` - Velocity commands
-   `/scan` - LiDAR data
-   `/odom` - Odometry
-   `/map` - Current map
-   `/amcl_pose` - Estimated position

#### Check Active Nodes

```bash
ros2 node list
```

#### Info on Specific Node

```bash
ros2 node info /node_name
```

#### Verify Connections

```bash
ros2 topic info /topic_name
```



### Important Configuration Files

-   **Launch files:** `~/robot/src/create_bringup/launch/`
-   **Nav2 parameters:** `~/robot/src/create_bringup/config/`
-   **Maps:** `~/robot/src/create_bringup/map/`

----------

_This guide provides complete instructions for operating the ROS2-based robot control system. For additional support, consult the official ROS2 documentation or hardware manufacturer guidelines._
