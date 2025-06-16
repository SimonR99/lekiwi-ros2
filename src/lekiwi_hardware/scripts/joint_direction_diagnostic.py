#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math

class JointDirectionDiagnostic(Node):
    def __init__(self):
        super().__init__('joint_direction_diagnostic')
        
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
            
        # Joint names in order (matching servo indices)
        self.joint_names = [
            'Shoulder_Rotation',  # Index 0 - Joint 1
            'Shoulder_Pitch',     # Index 1 - Joint 2  
            'Elbow',              # Index 2 - Joint 3
            'Wrist_Pitch',        # Index 3 - Joint 4
            'Wrist_Roll',         # Index 4 - Joint 5
            'Gripper'             # Index 5 - Joint 6
        ]
        
        self.current_positions = [0.0] * len(self.joint_names)
        self.positions_received = False
        
        # Joint direction results
        self.direction_results = {}
        
        self.get_logger().info("=== Joint Direction Diagnostic ===")
        self.get_logger().info("This will test each joint individually to identify direction mismatches")

    def joint_state_callback(self, msg):
        # Update current positions
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_positions[i] = msg.position[idx]
        self.positions_received = True

    def send_joint_command(self, positions, duration_sec=2.0):
        """Send a joint trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        msg.points = [point]
        self.arm_pub.publish(msg)

    def wait_for_joint_states(self):
        """Wait for initial joint state"""
        self.get_logger().info("Waiting for joint states...")
        while not self.positions_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Joint states received!")

    def test_individual_joint(self, joint_idx):
        """Test a single joint direction"""
        joint_name = self.joint_names[joint_idx]
        
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"Testing {joint_name} (Index {joint_idx}, Physical Joint {joint_idx+1})")
        self.get_logger().info(f"{'='*50}")
        
        # Get starting position
        initial_positions = self.current_positions.copy()
        self.get_logger().info(f"Initial position: {initial_positions[joint_idx]:.3f} rad ({initial_positions[joint_idx]*180/math.pi:.1f}°)")
        
        # Test positive movement
        test_angle = 0.5  # ~28.6 degrees
        test_positions = initial_positions.copy()
        test_positions[joint_idx] += test_angle
        
        self.get_logger().info(f"Commanding +{test_angle:.3f} rad (+{test_angle*180/math.pi:.1f}°)")
        self.get_logger().info("👀 WATCH BOTH:")
        self.get_logger().info("   1. RViz robot model joint movement")  
        self.get_logger().info("   2. Real hardware joint movement")
        
        self.send_joint_command(test_positions, 3.0)
        time.sleep(4.0)
        
        # Ask user for direction assessment
        print("\nDo the joint movements match?")
        print("  's' = SAME direction (both move positive)")
        print("  'o' = OPPOSITE direction (one positive, one negative)")
        print("  'n' = NO movement on hardware")
        print("  'r' = RViz doesn't move but hardware does")
        
        while True:
            response = input("Enter choice (s/o/n/r): ").lower().strip()
            if response in ['s', 'o', 'n', 'r']:
                break
            print("Please enter 's', 'o', 'n', or 'r'")
        
        # Record result
        if response == 's':
            self.direction_results[joint_name] = "CORRECT"
            result_msg = "✅ CORRECT - Same direction"
        elif response == 'o':
            self.direction_results[joint_name] = "REVERSED" 
            result_msg = "❌ REVERSED - Opposite directions"
        elif response == 'n':
            self.direction_results[joint_name] = "NO_HARDWARE_MOVEMENT"
            result_msg = "⚠️  NO HARDWARE MOVEMENT"
        else:  # 'r'
            self.direction_results[joint_name] = "NO_RVIZ_MOVEMENT"
            result_msg = "⚠️  NO RVIZ MOVEMENT"
            
        self.get_logger().info(f"Result: {result_msg}")
        
        # Return to initial position
        self.get_logger().info("Returning to initial position...")
        self.send_joint_command(initial_positions, 2.0)
        time.sleep(3.0)

    def run_full_diagnostic(self):
        """Run diagnostic on all joints"""
        self.wait_for_joint_states()
        
        for joint_idx in range(len(self.joint_names)):
            self.test_individual_joint(joint_idx)
            
            if joint_idx < len(self.joint_names) - 1:
                input("\nPress Enter to test next joint...")
        
        # Print summary
        self.print_summary()

    def print_summary(self):
        """Print diagnostic summary and recommended fixes"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("DIAGNOSTIC SUMMARY")
        self.get_logger().info(f"{'='*60}")
        
        reversed_joints = []
        correct_joints = []
        problem_joints = []
        
        for i, joint_name in enumerate(self.joint_names):
            result = self.direction_results.get(joint_name, "NOT_TESTED")
            self.get_logger().info(f"Joint {i+1} ({joint_name:16}): {result}")
            
            if result == "REVERSED":
                reversed_joints.append(i)
            elif result == "CORRECT":
                correct_joints.append(i)
            else:
                problem_joints.append(i)
        
        # Recommendations
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("RECOMMENDED FIXES")
        self.get_logger().info(f"{'='*60}")
        
        if reversed_joints:
            self.get_logger().info("Option 1: Fix in URDF (Recommended)")
            for joint_idx in reversed_joints:
                joint_name = self.joint_names[joint_idx]
                self.get_logger().info(f"  - Reverse axis direction for {joint_name} in URDF")
            
            self.get_logger().info("\nOption 2: Fix in Software")
            direction_array = [1] * 6
            for joint_idx in reversed_joints:
                direction_array[joint_idx] = -1
            self.get_logger().info(f"  - Set servo_directions_ = {{{', '.join(map(str, direction_array))}}}")
        
        if problem_joints:
            self.get_logger().info("\nIssues to investigate:")
            for joint_idx in problem_joints:
                joint_name = self.joint_names[joint_idx]
                result = self.direction_results[joint_name]
                self.get_logger().info(f"  - {joint_name}: {result}")

def main():
    rclpy.init()
    
    try:
        diagnostic = JointDirectionDiagnostic()
        diagnostic.run_full_diagnostic()
    except KeyboardInterrupt:
        print("\nDiagnostic interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 