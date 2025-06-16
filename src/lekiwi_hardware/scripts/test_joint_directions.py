#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math

class JointDirectionTest(Node):
    def __init__(self):
        super().__init__('joint_direction_test')
        
        # Create publisher for arm controller
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/lekiwi_controller/joint_trajectory', 
            10)
        
        # Subscribe to joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        # Joint names in order
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch', 
            'Elbow',           # Joint 3 (index 2) - FIXED
            'Wrist_Pitch',     # Joint 4 (index 3) - FIXED
            'Wrist_Roll',
            'Gripper'
        ]
        
        self.current_positions = [0.0] * len(self.joint_names)
        self.positions_received = False
        
        # Wait for initial joint state
        self.get_logger().info("Waiting for joint states...")
        while not self.positions_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Joint states received. Starting direction test...")
        self.run_direction_test()

    def joint_state_callback(self, msg):
        # Update current positions
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_positions[i] = msg.position[idx]
        self.positions_received = True

    def send_joint_command(self, positions, duration_sec=3.0):
        """Send a joint trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        msg.points = [point]
        self.arm_pub.publish(msg)
        
        self.get_logger().info(f"Sent command: {[f'{p:.2f}' for p in positions]}")

    def run_direction_test(self):
        """Test each joint direction individually"""
        
        # Start from current position
        base_positions = self.current_positions.copy()
        self.get_logger().info(f"Starting from: {[f'{p:.2f}' for p in base_positions]}")
        
        # Test each joint with small positive movement
        test_angle = 0.3  # ~17 degrees
        
        for joint_idx in range(len(self.joint_names) - 1):  # Skip gripper
            joint_name = self.joint_names[joint_idx]
            
            self.get_logger().info(f"\n=== Testing {joint_name} (Joint {joint_idx+1}) ===")
            
            # Move joint in positive direction
            test_positions = base_positions.copy()
            test_positions[joint_idx] += test_angle
            
            self.get_logger().info(f"Moving {joint_name} by +{test_angle:.2f} radians (+{test_angle*180/math.pi:.1f}°)")
            self.get_logger().info("⚠️  Check if the robot joint moves in the SAME direction as RViz!")
            
            self.send_joint_command(test_positions, 2.0)
            time.sleep(3.0)
            
            # Return to base position
            self.get_logger().info("Returning to base position...")
            self.send_joint_command(base_positions, 2.0)
            time.sleep(3.0)
            
            # Wait for user input
            input(f"Press Enter when ready to test next joint...")
        
        self.get_logger().info("\n=== Direction Test Complete ===")
        self.get_logger().info("If any joints moved in opposite direction to RViz, they need direction correction.")

def main():
    rclpy.init()
    
    try:
        test_node = JointDirectionTest()
        # Keep node alive
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 