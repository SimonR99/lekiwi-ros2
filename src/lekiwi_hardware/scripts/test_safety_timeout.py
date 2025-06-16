#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SafetyTimeoutTester(Node):
    def __init__(self):
        super().__init__('safety_timeout_tester')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Safety timeout tester started")
        
    def send_velocity(self, vx, vy, omega, description):
        """Send a velocity command"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(
            f"{description}: vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f}"
        )

def main():
    rclpy.init()
    tester = SafetyTimeoutTester()
    
    print("=== LeKiwi Safety Timeout Test ===")
    print("This test demonstrates the safety timeout mechanism.")
    print("The robot will stop automatically if no commands are sent.\n")
    
    # Wait for initialization
    time.sleep(2)
    
    print("1. Sending forward command...")
    tester.send_velocity(0.1, 0.0, 0.0, "Moving forward")
    
    print("2. Waiting 0.5 seconds and sending another command...")
    time.sleep(0.5)
    tester.send_velocity(0.1, 0.0, 0.0, "Still moving forward")
    
    print("3. Waiting another 0.5 seconds and sending command...")
    time.sleep(0.5)
    tester.send_velocity(0.1, 0.0, 0.0, "Still moving forward")
    
    print("4. Now waiting 2 seconds WITHOUT sending any commands...")
    print("   The robot should automatically stop due to safety timeout!")
    
    for i in range(20):  # 2 seconds at 10Hz
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)
        if i == 10:  # After 1 second
            print("   -> Should see safety timeout message around now...")
    
    print("\n5. Now sending a new command to clear the safety stop...")
    tester.send_velocity(0.0, 0.1, 0.0, "Strafing left - should clear safety stop")
    
    print("6. Waiting 0.5 seconds...")
    time.sleep(0.5)
    
    print("7. Sending stop command...")
    tester.send_velocity(0.0, 0.0, 0.0, "Explicit stop")
    
    print("\n=== Test Complete ===")
    print("You should have observed:")
    print("✅ Robot moving forward with regular commands")
    print("✅ Safety timeout warning after 1 second of no commands")
    print("✅ Robot automatically stopped") 
    print("✅ Safety stop cleared when new command received")
    print("✅ Manual stop command")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 