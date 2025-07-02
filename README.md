# LeKiwi ROS2 - Holonomic Mobile Manipulator

A complete ROS2 system for the LeKiwi robot - a holonomic mobile manipulator featuring a 5-DOF robotic arm on an omnidirectional mobile base. This repository provides comprehensive hardware control, motion planning, autonomous navigation, and perception capabilities.

## 🤖 Robot Overview

The LeKiwi robot combines:
- **5-DOF Robotic Arm** with servo-based joints and gripper
- **Holonomic Mobile Base** with 3-wheel omnidirectional movement
- **Vision System** with USB cameras for perception
- **Autonomous Navigation** with Nav2 stack integration
- **Motion Planning** with MoveIt for inverse kinematics

## 📦 Package Structure

### Core Packages

#### `lekiwi` - Main Robot Package
- **Launch Files**: Modular launch system for complete robot operation
- **Configuration**: URDF models, MoveIt configs, Nav2 parameters
- **Models**: 3D meshes and Gazebo simulation models
- **Dependencies**: MoveIt, Nav2, visualization tools

#### `lekiwi_hardware` - Hardware Interface Package
- **Hardware Interface**: ROS2 Control integration for servo motors
- **Controllers**: Holonomic movement and arm control scripts
- **Calibration**: Interactive calibration and testing tools
- **Real-time Control**: Odometry publishing and safety systems

## 🚀 Quick Start

### Installation

1. **Clone the repository:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> lekiwi-ros2
```

2. **Install dependencies:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**
```bash
colcon build
source install/setup.bash
```

### Basic Usage

**Launch the complete robot system:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py
```

**Launch with visualization:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=true
```

**Launch with MoveIt motion planning interface:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py use_moveit_rviz:=true
```

## 🔧 System Architecture

### Hardware Layer (`robot_hardware.launch.py`)
- Robot State Publisher
- Controller Manager (arm, gripper, wheels)
- Holonomic Controller
- Odometry Publisher
- Safety Systems

### Motion Planning Layer (`motion_planning.launch.py`)
- MoveIt move_group node
- Inverse Kinematics solver
- Trajectory execution
- Optional MoveIt RViz interface

### Navigation Layer (`navigation.launch.py`)
- Nav2 Controller Server
- Nav2 Planner Server
- Nav2 Behavior Server
- Nav2 BT Navigator
- Lifecycle Manager

### Perception Layer (`cameras.launch.py`)
- USB Camera nodes (bottom and optional wrist cameras)
- Image processing pipeline

### Visualization Layer (`visualization.launch.py`)
- RViz2 with robot model visualization

## 🎛️ Launch System

### Main Launch File
**`lekiwi_full.launch.py`** - Complete robot system with modular architecture
- **Default includes:** Hardware, MoveIt (IK), Nav2, cameras enabled
- **Usage:** `ros2 launch lekiwi lekiwi_full.launch.py`

### Modular Components  
- **`robot_hardware.launch.py`** - Hardware controllers and low-level systems
- **`navigation.launch.py`** - Nav2 autonomous navigation  
- **`motion_planning.launch.py`** - MoveIt motion planning and IK
- **`cameras.launch.py`** - Camera system
- **`visualization.launch.py`** - RViz visualization

### Legacy/Specialized
- **`hardware.launch.py`** - Original standalone hardware launcher
- **`nav2_bringup.launch.py`** - Advanced Nav2 with extra features  
- **`rviz.launch.py`** - DOF-configurable RViz
- **`gz.launch.py`** - Gazebo simulation

## 🛠️ Hardware Control Scripts

### Core Controllers
- **`holonomic_controller.py`** - Omnidirectional movement control with safety timeout
- **`odometry_publisher.py`** - Real-time pose estimation from wheel velocities

### Calibration & Setup
- **`calibrate_arm.py`** - Interactive arm calibration tool
- **`zero_pose.py`** - Send arm to home position

### Manual Control
- **`manual_holonomic.py`** - Keyboard-based robot control
- **`jog_joints.py`** - Interactive joint control with terminal UI

### Testing Tools
- **`test_holonomic.py`** - Automated movement testing
- **`test_odometry.py`** - Odometry validation
- **`test_safety_timeout.py`** - Safety system validation
- **`test_wheel_velocity.py`** - Low-level wheel control testing

## 📋 Launch Parameters

### Common Parameters
- `use_fake_hardware` (default: false) - Use simulated hardware
- `start_moveit` (default: true) - Enable MoveIt motion planning
- `start_nav2` (default: true) - Enable autonomous navigation
- `start_cameras` (default: true) - Enable camera feeds
- `use_rviz` (default: false) - Launch basic RViz
- `use_moveit_rviz` (default: false) - Launch MoveIt RViz interface
- `zero_pose` (default: false) - Test zero pose after startup

### Advanced Parameters
- `use_sim_time` (default: false) - Use simulation clock
- `autostart` (default: true) - Auto-start Nav2 lifecycle nodes
- `nav2_params_file` - Custom Nav2 parameters file

## 🗂️ Configuration Files

### Robot Description
- `lekiwi.urdf.xacro` - Main robot URDF with xacro macros
- `lekiwi.ros2_control.xacro` - ROS2 Control hardware interface
- `lekiwi.srdf` - MoveIt semantic robot description

### Control Configuration
- `controllers.yaml` - ROS2 Control joint controllers
- `moveit_controllers.yaml` - MoveIt controller configuration
- `joint_limits.yaml` - Joint limits and safety parameters
- `kinematics.yaml` - MoveIt kinematics solver configuration

### Navigation Configuration
- `nav2_params.yaml` - Nav2 navigation stack parameters

### Hardware Configuration
- `hardware_config.yaml` - Hardware interface settings
- `calibration.yaml` - Joint calibration data
- `joint_offsets.yaml` - Joint offset corrections

## 🔍 Development & Debugging

### Modular Testing
```bash
# Hardware only
ros2 launch lekiwi robot_hardware.launch.py

# Navigation only  
ros2 launch lekiwi navigation.launch.py

# Motion planning only
ros2 launch lekiwi motion_planning.launch.py

# Cameras only
ros2 launch lekiwi cameras.launch.py

# Visualization only
ros2 launch lekiwi visualization.launch.py use_rviz:=true
```

### Manual Control
```bash
# Manual robot movement
ros2 run lekiwi_hardware manual_holonomic.py

# Interactive joint control
ros2 run lekiwi_hardware jog_joints.py
```

### Calibration
```bash
# Calibrate arm joints
ros2 run lekiwi_hardware calibrate_arm.py

# Test movements
ros2 run lekiwi_hardware test_holonomic.py
```

## 🏗️ Robot Specifications

### Mobile Base
- **Type**: 3-wheel holonomic (omnidirectional)
- **Wheel Configuration**: 120° spacing (left=240°, rear=0°, right=120°)
- **Wheel Radius**: 5cm
- **Base Radius**: 12.5cm
- **Max Velocity**: 3.0 rad/s per wheel
- **Safety**: 1-second command timeout

### Robotic Arm
- **DOF**: 5 (Shoulder Rotation, Shoulder Pitch, Elbow, Wrist Pitch, Wrist Roll)
- **Gripper**: 1 DOF parallel gripper
- **Motors**: Servo-based joints with position control
- **Control**: Individual joint control with trajectory execution

### Sensors
- **Cameras**: USB cameras (bottom-mounted, optional wrist-mounted)
- **Odometry**: Wheel encoder-based pose estimation
- **IMU**: Optional (configurable)

## 🛡️ Safety Features

- **Command Timeout**: Robot stops if no commands received for 1 second
- **Joint Limits**: Software and hardware joint limit enforcement
- **Velocity Limiting**: Proportional velocity scaling when limits exceeded
- **Emergency Stop**: Immediate motor disable capability
- **Torque Control**: Individual joint torque enable/disable

## 📚 Dependencies

### ROS2 Packages
- `moveit_ros_move_group` - Motion planning
- `nav2_bringup` - Autonomous navigation
- `controller_manager` - Hardware control
- `robot_state_publisher` - Robot model publishing
- `usb_cam` - Camera interface
- `rviz2` - Visualization

### Hardware Dependencies
- Servo motor communication library
- USB camera drivers
- Serial communication interface

## 🐛 Troubleshooting

### Common Issues
- **Hardware not responding**: Check servo connections and power
- **Camera not found**: Verify `/dev/video*` devices and permissions
- **Controllers not loading**: Check `controllers.yaml` configuration
- **Navigation not working**: Ensure proper odometry and TF setup
- **MoveIt errors**: Verify SRDF and kinematics configuration

### Debugging Tools
```bash
# Check system status
ros2 node list
ros2 topic list
ros2 service list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor odometry
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames
```

## 📄 License

This project is licensed under the Apache 2.0 License (lekiwi_hardware) and BSD License (lekiwi). See the LICENSE files in each package for details.

## 👥 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📞 Support

For issues, questions, or contributions:
- Create an issue in the repository
- Contact the maintainer: simon.roy@example.com

---

**For detailed launch file documentation, see:** [src/lekiwi/launch/README.md](src/lekiwi/launch/README.md)