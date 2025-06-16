#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class PositionVerifier(Node):
    def __init__(self):
        super().__init__('position_verifier')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Joint names we care about
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch', 
            'Elbow',           # Joint 3 - Fixed direction
            'Wrist_Pitch',     # Joint 4 - Fixed direction  
            'Wrist_Roll',
            'Gripper'
        ]
        
        self.get_logger().info("Position Verifier started.")
        self.get_logger().info("Monitoring joint positions...")
        self.get_logger().info("Compare the published values with actual robot position!")
        
    def joint_state_callback(self, msg):
        self.get_logger().info("=== Current Joint Positions ===")
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    pos_rad = msg.position[idx]
                    pos_deg = pos_rad * 180.0 / math.pi
                    
                    # Special attention to the fixed joints
                    status = ""
                    if name in ['Elbow', 'Wrist_Pitch']:
                        status = " (DIRECTION FIXED)"
                    
                    self.get_logger().info(f"{name:16}: {pos_rad:+7.3f} rad ({pos_deg:+6.1f}°){status}")
                else:
                    self.get_logger().warn(f"{name}: No position data")
            else:
                self.get_logger().warn(f"{name}: Not found in joint_states")
        
        self.get_logger().info("================================")
        
    def run(self):
        self.get_logger().info("\n📋 VERIFICATION INSTRUCTIONS:")
        self.get_logger().info("1. Disable torque: ros2 service call /torque std_srvs/srv/Trigger")
        self.get_logger().info("2. Manually move each joint on the real robot")
        self.get_logger().info("3. Check if the published angle matches the actual joint position")
        self.get_logger().info("4. Pay special attention to Elbow and Wrist_Pitch (joints 3&4)")
        self.get_logger().info("\n✅ If angles match real hardware → Fix successful!")
        self.get_logger().info("❌ If angles don't match → Further calibration needed")

def main():
    rclpy.init()
    
    try:
        verifier = PositionVerifier()
        verifier.run()
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        print("\nPosition verification stopped")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 