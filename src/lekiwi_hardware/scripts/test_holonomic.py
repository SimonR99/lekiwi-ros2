#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class HolonomicTester(Node):
    def __init__(self):
        super().__init__('holonomic_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Holonomic movement tester started")
        
    def send_velocity(self, vx, vy, omega, description):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(
            f"{description}: vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f}"
        )
    
    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, "STOP")

def main():
    rclpy.init()
    tester = HolonomicTester()
    
    print("=== LeKiwi Holonomic Control Test ===")
    print("Testing various movement patterns...")
    
    time.sleep(2)
    
    test_sequence = [
        (0.0, 0.0, 0.0, 1.0, "Initial stop"),
        
        (0.1, 0.0, 0.0, 3.0, "Forward"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (-0.1, 0.0, 0.0, 3.0, "Backward"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        
        (0.0, 0.1, 0.0, 3.0, "Strafe left"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (0.0, -0.1, 0.0, 3.0, "Strafe right"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        
        (0.0, 0.0, 0.3, 3.0, "Rotate counterclockwise"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (0.0, 0.0, -0.3, 3.0, "Rotate clockwise"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        
        (0.1, 0.1, 0.0, 3.0, "Diagonal forward-left"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (0.1, -0.1, 0.0, 3.0, "Diagonal forward-right"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        
        (0.1, 0.0, 0.2, 3.0, "Forward + rotate"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (0.0, 0.1, 0.2, 3.0, "Strafe left + rotate"),
        (0.0, 0.0, 0.0, 1.0, "Stop"),
        (0.1, 0.1, 0.2, 3.0, "Diagonal + rotate"),
        (0.0, 0.0, 0.0, 2.0, "Final stop"),
    ]
    
    for vx, vy, omega, duration, description in test_sequence:
        print(f"\n{description}")
        tester.send_velocity(vx, vy, omega, description)
        
        for _ in range(int(duration * 10)):
            rclpy.spin_once(tester, timeout_sec=0.1)
            time.sleep(0.1)
    
    print("\nHolonomic test complete!")
    print("Your robot should now demonstrate full holonomic capabilities:")
    print("✅ Forward/backward movement")
    print("✅ Side-to-side strafing")  
    print("✅ Pure rotation")
    print("✅ Combined movements (the holonomic advantage!)")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 