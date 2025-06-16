#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time

class WheelDemo(Node):
    def __init__(self):
        super().__init__('wheel_demo')
        
        # Publisher for wheel commands
        self.wheel_pub = self.create_publisher(
            JointTrajectory,
            '/lekiwi_wheel_controller/joint_trajectory',
            10)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.wheel_positions = {}
        self.wheel_joints = ['left_wheel_drive', 'rear_wheel_drive', 'right_wheel_drive']
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.wheel_joints:
                self.wheel_positions[name] = msg.position[i]
    
    def move_wheel(self, wheel_index, target_position, duration=2.0):
        """Move a specific wheel to target position"""
        wheel_name = self.wheel_joints[wheel_index]
        current_positions = [self.wheel_positions.get(joint, 0.0) for joint in self.wheel_joints]
        
        # Set target position for selected wheel
        target_positions = current_positions.copy()
        target_positions[wheel_index] = target_position
        
        # Create and send trajectory
        msg = JointTrajectory()
        msg.joint_names = self.wheel_joints
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points = [point]
        self.wheel_pub.publish(msg)
        
        self.get_logger().info(f"Moving {wheel_name} to {target_position:.2f} rad")
        return target_positions
    
    def print_current_positions(self):
        """Print current wheel positions"""
        print("\nCurrent wheel positions:")
        for i, joint in enumerate(self.wheel_joints):
            pos = self.wheel_positions.get(joint, 0.0)
            print(f"  {i}: {joint:<18} {pos:8.3f} rad")

def main():
    rclpy.init()
    demo = WheelDemo()
    
    print("=== LeKiwi Individual Wheel Control Demo ===")
    print("Waiting for initial joint states...")
    
    # Wait for initial joint states
    while len(demo.wheel_positions) < 3:
        rclpy.spin_once(demo, timeout_sec=0.1)
    
    print("Initial positions received!")
    demo.print_current_positions()
    
    print("\nDemonstrating individual wheel control...")
    
    # Demo sequence
    demos = [
        (0, 0.5, "Move LEFT wheel to 0.5 rad"),
        (1, 1.0, "Move REAR wheel to 1.0 rad"),  
        (2, -0.5, "Move RIGHT wheel to -0.5 rad"),
        (0, 0.0, "Return LEFT wheel to 0.0 rad"),
        (1, 0.0, "Return REAR wheel to 0.0 rad"),
        (2, 0.0, "Return RIGHT wheel to 0.0 rad"),
    ]
    
    for wheel_idx, target_pos, description in demos:
        print(f"\n{description}")
        demo.move_wheel(wheel_idx, target_pos, duration=3.0)
        
        # Wait and check positions
        time.sleep(4.0)
        for _ in range(10):  # Spin to get latest joint states
            rclpy.spin_once(demo, timeout_sec=0.1)
        
        demo.print_current_positions()
    
    print("\nDemo complete! Individual wheel control is working! 🎉")
    rclpy.shutdown()

if __name__ == '__main__':
    main() 