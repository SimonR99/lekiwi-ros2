#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import curses
import math
import sys

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        
        # Define wheel joints
        self.wheel_joints = [
            'left_wheel_drive',
            'rear_wheel_drive', 
            'right_wheel_drive'
        ]
        
        # Publisher for wheel commands
        self.wheel_pub = self.create_publisher(
            JointTrajectory,
            '/lekiwi_wheel_controller/joint_trajectory',
            10)
        
        # Subscribe to joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Track current wheel positions
        self.wheel_positions = {joint: 0.0 for joint in self.wheel_joints}
        self.selected_wheel = 0
        self.step_size = 0.1  # radians
        self.status_message = "Wheel control ready"
        
    def joint_state_callback(self, msg):
        """Update current wheel positions from joint states"""
        for i, name in enumerate(msg.name):
            if name in self.wheel_joints:
                self.wheel_positions[name] = msg.position[i]
    
    def move_wheel(self, wheel_index, delta):
        """Move a specific wheel by delta amount"""
        wheel_name = self.wheel_joints[wheel_index]
        current_pos = self.wheel_positions[wheel_name]
        new_pos = current_pos + delta
        
        # Create trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.wheel_joints
        
        point = JointTrajectoryPoint()
        # Start with current positions for all wheels
        point.positions = [self.wheel_positions[joint] for joint in self.wheel_joints]
        # Only modify the selected wheel
        point.positions[wheel_index] = new_pos
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500_000_000  # 0.5 seconds
        
        msg.points = [point]
        self.wheel_pub.publish(msg)
        
        self.status_message = f"Moving {wheel_name} from {current_pos:.3f} to {new_pos:.3f}"
    
    def move_all_wheels(self, delta):
        """Move all wheels by the same amount"""
        msg = JointTrajectory()
        msg.joint_names = self.wheel_joints
        
        point = JointTrajectoryPoint()
        # Move all wheels by delta
        point.positions = [self.wheel_positions[joint] + delta for joint in self.wheel_joints]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500_000_000  # 0.5 seconds
        
        msg.points = [point]
        self.wheel_pub.publish(msg)
        
        self.status_message = f"Moving all wheels by {delta:.3f} radians"
    
    def stop_all_wheels(self):
        """Stop all wheels (set to current position)"""
        msg = JointTrajectory()
        msg.joint_names = self.wheel_joints
        
        point = JointTrajectoryPoint()
        point.positions = [self.wheel_positions[joint] for joint in self.wheel_joints]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000  # 0.1 seconds
        
        msg.points = [point]
        self.wheel_pub.publish(msg)
        
        self.status_message = "All wheels stopped"

def main_curses(stdscr):
    """Main function with curses interface"""
    # Set up curses
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.timeout(100)
    
    rclpy.init()
    controller = WheelController()
    
    while True:
        # Clear screen and display interface
        stdscr.clear()
        
        # Display title and instructions
        stdscr.addstr(0, 0, "=== LeKiwi Individual Wheel Control ===")
        stdscr.addstr(1, 0, "Instructions:")
        stdscr.addstr(2, 0, "Up/Down: Select wheel")
        stdscr.addstr(3, 0, "Left/Right: Move selected wheel")
        stdscr.addstr(4, 0, "A/D: Move all wheels")
        stdscr.addstr(5, 0, "+/-: Adjust step size")
        stdscr.addstr(6, 0, "Space: Stop all wheels")
        stdscr.addstr(7, 0, "Q: Quit")
        stdscr.addstr(8, 0, "-" * 40)
        
        # Display wheel states
        for i, wheel_name in enumerate(controller.wheel_joints):
            prefix = ">" if i == controller.selected_wheel else " "
            pos = controller.wheel_positions[wheel_name]
            stdscr.addstr(9 + i, 0, f"{prefix} {wheel_name:<18} {pos:8.3f} rad")
        
        # Display controls info
        stdscr.addstr(13, 0, f"Step size: {controller.step_size:5.3f} rad")
        stdscr.addstr(14, 0, f"Selected: {controller.wheel_joints[controller.selected_wheel]}")
        stdscr.addstr(15, 0, f"Status: {controller.status_message}")
        
        stdscr.refresh()
        
        # Handle input
        try:
            key = stdscr.getch()
            
            if key == ord('q') or key == ord('Q'):
                break
            elif key == curses.KEY_UP:
                controller.selected_wheel = (controller.selected_wheel - 1) % len(controller.wheel_joints)
            elif key == curses.KEY_DOWN:
                controller.selected_wheel = (controller.selected_wheel + 1) % len(controller.wheel_joints)
            elif key == curses.KEY_LEFT:
                controller.move_wheel(controller.selected_wheel, -controller.step_size)
            elif key == curses.KEY_RIGHT:
                controller.move_wheel(controller.selected_wheel, controller.step_size)
            elif key == ord('a') or key == ord('A'):
                controller.move_all_wheels(-controller.step_size)
            elif key == ord('d') or key == ord('D'):
                controller.move_all_wheels(controller.step_size)
            elif key == ord('+') or key == ord('='):
                controller.step_size = min(controller.step_size * 1.5, 1.0)
            elif key == ord('-'):
                controller.step_size = max(controller.step_size / 1.5, 0.01)
            elif key == ord(' '):
                controller.stop_all_wheels()
                
        except KeyboardInterrupt:
            break
        
        # Process ROS callbacks
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    rclpy.shutdown()

def main():
    """Entry point - can be called directly or with curses"""
    if len(sys.argv) > 1 and sys.argv[1] == '--simple':
        # Simple command line interface
        rclpy.init()
        controller = WheelController()
        
        print("Simple wheel control mode")
        print("Enter commands: wheel_index delta (e.g., '0 0.5' to move left wheel)")
        print("Or 'all delta' to move all wheels")
        print("Ctrl+C to quit")
        
        try:
            while True:
                cmd = input("wheel> ").strip().split()
                if len(cmd) == 2:
                    if cmd[0] == 'all':
                        delta = float(cmd[1])
                        controller.move_all_wheels(delta)
                    else:
                        wheel_idx = int(cmd[0])
                        delta = float(cmd[1])
                        if 0 <= wheel_idx < len(controller.wheel_joints):
                            controller.move_wheel(wheel_idx, delta)
                        else:
                            print(f"Invalid wheel index. Use 0-{len(controller.wheel_joints)-1}")
                rclpy.spin_once(controller, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        
        rclpy.shutdown()
    else:
        # Use curses interface
        curses.wrapper(main_curses)

if __name__ == '__main__':
    main() 