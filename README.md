# LeKiwi ROS2 - Holonomic Mobile Manipulator

A ROS2 system for the LeKiwi robot - a holonomic mobile manipulator with a 5-DOF robotic arm on an omnidirectional mobile base.

## 🤖 Robot Overview

The LeKiwi robot features:
- **5-DOF Robotic Arm** with servo-based joints and gripper
- **Holonomic Mobile Base** with 3-wheel omnidirectional movement
- **Vision System** with USB cameras
- **Autonomous Navigation** with Nav2 stack
- **Motion Planning** with MoveIt

## 📦 Package Structure

- **`lekiwi`** - Main robot package with launch files, URDF models, and configuration
- **`lekiwi_hardware`** - Hardware interface with ROS2 Control integration and controller scripts

## 🚀 Quick Start

### Installation

1. **Clone and build:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> lekiwi-ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Usage

**Launch the complete robot:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py
```

**With visualization:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=true
```

**With MoveIt interface:**
```bash
ros2 launch lekiwi lekiwi_full.launch.py use_moveit_rviz:=true
```

## 🎛️ Launch System

### Main Launch Files
- **`lekiwi_full.launch.py`** - Complete robot system
- **`robot_hardware.launch.py`** - Hardware controllers only
- **`navigation.launch.py`** - Nav2 navigation only
- **`motion_planning.launch.py`** - MoveIt motion planning only
- **`cameras.launch.py`** - Camera system only

### Common Parameters
- `use_fake_hardware` (default: false) - Use simulated hardware
- `start_moveit` (default: true) - Enable MoveIt motion planning
- `start_nav2` (default: true) - Enable autonomous navigation
- `start_cameras` (default: true) - Enable camera feeds
- `use_rviz` (default: false) - Launch basic RViz
- `use_moveit_rviz` (default: false) - Launch MoveIt RViz interface

## 🛠️ Hardware Control

### Manual Control
```bash
# Manual robot movement
ros2 run lekiwi_hardware manual_holonomic.py
```


## 🏗️ Robot Specifications

### Mobile Base
- 3-wheel holonomic configuration
- 120° spacing between wheels
- Max velocity: 3.0 rad/s per wheel
- Safety timeout: 1 second

### Robotic Arm
- 5 DOF + 1 DOF gripper
- Servo-based joints with position control
- Individual joint control with trajectory execution

## 🐛 Troubleshooting

### Common Issues
- **Hardware not responding**: Check servo connections and power
- **Camera not found**: Verify `/dev/video*` devices and permissions
- **Controllers not loading**: Check `controllers.yaml` configuration

### Debugging
```bash
# Check system status
ros2 node list
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor odometry
ros2 topic echo /odom
```

## 📄 License

Apache 2.0 License (lekiwi_hardware) and BSD License (lekiwi).

## 👥 Contributing

1. Fork the repository
2. Create a feature branch
3. Test thoroughly
4. Submit a pull request

---

## 🙏 Acknowledgments

This project is inspired by the [SO-100 arm ROS2 package](https://github.com/brukg/SO-100-arm) by brukg.