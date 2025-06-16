#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class WheelVelocityTest(Node):
    def __init__(self):
        super().__init__('wheel_velocity_test')
        
        # Publisher for wheel velocity commands
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/lekiwi_wheel_controller/commands',
            10)
        
        self.get_logger().info("Wheel velocity test node started")
        
    def send_wheel_velocities(self, left_vel, rear_vel, right_vel):
        """Send velocity commands to wheels"""
        msg = Float64MultiArray()
        msg.data = [left_vel, rear_vel, right_vel]
        
        self.wheel_vel_pub.publish(msg)
        self.get_logger().info(f"Sent velocities: left={left_vel:.2f}, rear={rear_vel:.2f}, right={right_vel:.2f} rad/s")

def main():
    rclpy.init()
    tester = WheelVelocityTest()
    
    print("=== LeKiwi Wheel Velocity Control Test ===")
    print("This will test velocity control for wheels")
    print("Commands: [left_wheel, rear_wheel, right_wheel] in rad/s")
    
    # Wait a moment for things to initialize
    time.sleep(2)
    
    # Test sequence
    test_commands = [
        (0.0, 0.0, 0.0, "Stop all wheels"),
        (0.5, 0.0, 0.0, "Move left wheel forward"),
        (0.0, 0.0, 0.0, "Stop all wheels"),
        (0.0, 0.5, 0.0, "Move rear wheel forward"), 
        (0.0, 0.0, 0.0, "Stop all wheels"),
        (0.0, 0.0, 0.5, "Move right wheel forward"),
        (0.0, 0.0, 0.0, "Stop all wheels"),
        (0.2, 0.2, 0.2, "Move all wheels forward slowly"),
        (0.0, 0.0, 0.0, "Stop all wheels"),
        (-0.2, -0.2, -0.2, "Move all wheels backward slowly"),
        (0.0, 0.0, 0.0, "Final stop"),
    ]
    
    for left, rear, right, description in test_commands:
        print(f"\n{description}")
        tester.send_wheel_velocities(left, rear, right)
        
        # Process any callbacks
        rclpy.spin_once(tester, timeout_sec=0.1)
        
        # Wait between commands
        time.sleep(3.0)
    
    print("\nVelocity control test complete!")
    print("Wheels should now be stopped (0 velocity)")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 