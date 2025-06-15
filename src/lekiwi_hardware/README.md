# SO-ARM100 Hardware Interface

## Overview
The `so_arm_100_hardware` package provides a ROS 2 Control hardware interface plugin for the SO-ARM100 low-cost 5DoF robotic manipulator. This interface supports both direct serial communication with the physical robot and simulation via ROS topics.

## Features
- ROS 2 Control hardware interface implementation
- Configurable communication modes:
  - Direct serial communication with the robot
  - ROS topic-based communication for simulation
- Position control interface for all joints
- Thread-safe feedback handling
- Lifecycle-managed node implementation

## Package Details
- **Name**: so_arm_100_hardware
- **Version**: 0.0.0
- **Description**: ROS2 Control Hardware Interface for SO-ARM100 low-cost 5DoF robotic manipulator
- **Maintainer**: Simon Roy (<simon.roy@example.com>)
- **License**: Apache-2.0

## Dependencies
- `rclcpp`
- `hardware_interface`
- `pluginlib`
- `rclcpp_lifecycle`
- `sensor_msgs`

## Communication Interface
The hardware interface communicates using the following ROS topics:

- **Command Topic**: `command` (sensor_msgs/msg/JointState)
  - Publishes joint position commands to the robot
  
- **Feedback Topic**: `feedback` (sensor_msgs/msg/JointState)
  - Subscribes to joint state feedback from the robot

## Hardware Interface Details
The `SOARM100Interface` class implements:
- State and command interfaces for position control
- Lifecycle management (init, activate, deactivate)
- Read and write methods for communication
- Thread-safe feedback handling using mutex
- Asynchronous ROS communication using a dedicated executor thread

## Installation

1. **Clone the package into your ROS 2 workspace**:

   ```bash
   cd ~/ros2_ws/src
   git clone git@github.com:brukg/so_arm_100_hardware.git
   ```

2. **Install dependencies**:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:

   ```bash
   colcon build --packages-select so_arm_100_hardware
   ```

4. **Source the workspace**:

   ```bash
   source install/setup.bash
   ```

## Usage

### Configuration

Create a ROS 2 Control configuration file (YAML) that includes the hardware interface:

```yaml
so_arm_100:
  hardware_interface:
    use_serial: true  # Set to true for real robot, false for simulation
    serial_port: "/dev/ttyACM0"  # Serial port for real robot
    serial_baudrate: 1000000  # Serial baudrate
```

### Serial Communication Mode
When using serial communication (`use_serial: true`):
1. Ensure the robot is connected to the specified serial port
2. Grant serial port access permissions:
```bash
sudo usermod -a -G dialout $USER  # Log out and back in after this
```
3. Verify the serial port and baudrate settings match your robot's configuration

### Topic Communication Mode
When using topic communication (`use_serial: false`):
- Commands are published to the "command" topic
- Feedback is received from the "feedback" topic
- Both topics use the `sensor_msgs/JointState` message type

## Development and Testing
The hardware interface includes serial communication code that can be enabled for direct hardware control. By default, it operates using ROS topics for command and feedback.

## Troubleshooting

### Serial Communication Issues
1. Check serial port permissions
2. Verify the correct port is specified in the config
3. Ensure the baudrate matches the robot's settings
4. Check serial cable connections

### Topic Communication Issues
1. Verify topics are being published/subscribed:
```bash
ros2 topic list
ros2 topic echo /command
ros2 topic echo /feedback
```

2. Check for any error messages in the logs:
```bash
ros2 run rqt_console rqt_console
```

## Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License
This package is licensed under the Apache License 2.0. See the LICENSE file for details.

## Related Packages
- [so_100_arm](https://github.com/brukg/so-100-arm): Main robot package
