# 🤖 ROS2 Robot Control System

A comprehensive robotics control system built with ROS2 Jazzy, featuring autonomous navigation, mapping, camera integration, and remote operation capabilities.

----------

## 📋 Table of Contents

-   [Prerequisites](#-prerequisites)
-   [Quick Start](#-quick-start)
-   [System Configuration](#-system-configuration)
-   [Robot Control](#-robot-control)
-   [Advanced Robot Operations](#-advanced-robot-operations)
-   [Zone Management System](#-zone-management-system)
-   [Integration & Monitoring](#-integration--monitoring)
-   [Camera Configuration](#-camera-configuration)
-   [Troubleshooting & Diagnostics](#-troubleshooting--diagnostics)
-   [Additional Resources](#-additional-resources)

----------

## 🛠 Prerequisites

### System Requirements

-   **ROS2 Jazzy** installed and configured
-   **Ubuntu 24.04** (recommended)
-   Robot hardware with camera, LIDAR, and motor controllers
-   Network connectivity between robot and control PC

### Hardware Components

-   Raspberry Pi 5 (recommended)
-   Compatible camera module
-   LIDAR sensor
-   Motor controllers for brushes and vacuum
-   Docking station

----------

## 🚀 Quick Start

### 1. Environment Setup

#### Source ROS2 Environment

```bash
source ~/robot/install/setup.bash
```

#### Check USB Connections

```bash
ls /dev/ttyUSB*
```

#### Synchronize System Time

```bash
sudo systemctl restart chrony
```

> This command was used when the Raspberry Pi was acting as a hotspot to synchronize the system time.

### 2. Launch Configurations

#### 🎥 Camera Only Mode

```bash
ros2 launch create_bringup create_2.py camera:=true navigation:=false foxglove:=false
```

#### 🗺️ Full System (Raspberry Pi)

```bash
ros2 launch create_bringup create_2.py \
  camera:=true \
  navigation:=true \
  foxglove:=true \
  map:=map_file.yaml
```

#### 🧠 AI-Enhanced System

```bash
ros2 launch create_bringup create_2.py \
  camera:=true \
  navigation:=true \
  foxglove:=true \
  rosbridge:=true \
  map:=map_file.yaml
```

----------


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

### Hardware Component Control

#### 🧹 Cleaning System Motors

**Main Brush Motor**

```bash
ros2 topic pub --once /main_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"

```

**Side Brush Motor**

```bash
ros2 topic pub --once /side_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"

```

**Vacuum Motor**

```bash
ros2 topic pub --once /vacuum_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"

```

> **Note:** Duty cycle values range from 0.0 to 1.0 (0% to 100%)

----------

## ⚡ Advanced Robot Operations

### Map Management

**Maps Storage Location:**

```
~/robot/src/create_bringup/map/
```

#### Map Operations

-   Store custom maps in the maps directory
-   Specify map files using the `map:=map_file.yaml` parameter
-   Ensure proper file permissions for map access

----------

## 🛡 Zone Management System

To create zones:

1.  **Launch zone maker** (cleaning or speed limiter mode)
2.  **Click 4 points** on the map interface to define polygon corners with the **Topic:** `/clicked_point`  
    **Message Type:** `geometry_msgs/msg/PointStamped`
3.  **Zone is automatically created** from the 4-point polygon

### Exclusion Zones Setup

```bash
ros2 launch create_bringup zone_maker.py
```

### Speed Limit Zones Configuration

```bash
ros2 launch create_bringup zone_maker.py node:=speed slow_value:=40
```

> **Speed Range:** Values from 1% to 99% of maximum speed

### Zone Types

-   **Exclusion Zones:** Areas the robot should avoid
-   **Speed Zones:** Areas with reduced speed limits



## 🔍 Troubleshooting & Diagnostics

### System Status Checks

**List all active ROS2 topics**

```bash
ros2 topic list
```

**Monitor specific topic data**

```bash
ros2 topic echo /topic_name
```

**Check running nodes**

```bash
ros2 node list
```

### Network Diagnostics

**Check network connectivity**

```bash
ping <robot_ip_address>
```

**Test ROS2 communication**

```bash
ros2 topic hz /cmd_vel
```

## 📚 Additional Resources

### Configuration Files

-   Launch files: `~/robot/src/create_bringup/launch/`
-   Parameter files: `~/robot/src/create_bringup/config/`
-   Map files: `~/robot/src/create_bringup/map/`

----------

_This guide provides comprehensive instructions for operating your ROS2-based robot control system. For additional support, consult the official ROS2 documentation or your hardware manufacturer's guidelines._
