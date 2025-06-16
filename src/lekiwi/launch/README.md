# LeKiwi Launch Files

This directory contains launch files for the LeKiwi robot system.

## Main Launch File

### `lekiwi_full.launch.py`
This is the main consolidated launch file that starts the complete LeKiwi system including hardware, Nav2 navigation, and MoveIt motion planning.

#### Usage
```bash
# Basic launch with default settings (Nav2 + MoveIt enabled, no RViz)
ros2 launch lekiwi lekiwi_full.launch.py

# Launch with basic RViz visualization
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=true

# Launch with MoveIt RViz interface  
ros2 launch lekiwi lekiwi_full.launch.py use_moveit_rviz:=true

# Launch with both RViz interfaces
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=true use_moveit_rviz:=true

# Launch only hardware and Nav2 (no MoveIt)
ros2 launch lekiwi lekiwi_full.launch.py start_moveit:=false

# Launch only hardware and MoveIt (no Nav2)
ros2 launch lekiwi lekiwi_full.launch.py start_nav2:=false

# Launch with fake hardware for testing
ros2 launch lekiwi lekiwi_full.launch.py use_fake_hardware:=true

# Test zero pose after startup
ros2 launch lekiwi lekiwi_full.launch.py zero_pose:=true
```

#### Parameters
- `use_sim_time` (default: false) - Use simulation clock
- `use_fake_hardware` (default: false) - Use fake hardware for testing
- `autostart` (default: true) - Automatically start Nav2 lifecycle nodes
- `nav2_params_file` - Path to Nav2 parameter file
- `use_rviz` (default: false) - Launch basic RViz visualization
- `use_moveit_rviz` (default: false) - Launch MoveIt RViz interface
- `start_nav2` (default: true) - Start Nav2 navigation stack
- `start_moveit` (default: true) - Start MoveIt motion planning
- `zero_pose` (default: false) - Test zero pose after startup

## Other Launch Files

### `hardware.launch.py`
Standalone hardware launch file for testing individual hardware components.

### `simple_nav2.launch.py`
Standalone Nav2 launch file for navigation-only testing.

### `nav2_bringup.launch.py`
Comprehensive Nav2 launch file with all navigation components.

### `gz.launch.py`
Gazebo simulation launch file.

### `rviz.launch.py`
Standalone RViz launch file with DOF configuration.

### `view_mobile_base.launch.py`
Mobile base visualization launch file.

## System Components

The main launch file starts these components in sequence:

1. **Hardware Layer**: Robot state publisher, controller manager, joint controllers
2. **Navigation Layer**: Nav2 controller, planner, behavior server, BT navigator
3. **Motion Planning Layer**: MoveIt move_group node
4. **Visualization Layer**: RViz (optional)

Components are started with proper sequencing to ensure dependencies are met. 