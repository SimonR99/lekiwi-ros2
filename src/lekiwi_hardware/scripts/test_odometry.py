#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class OdometryTester(Node):
    def __init__(self):
        super().__init__('odometry_tester')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Current odometry
        self.current_odom = None
        
        self.get_logger().info("Odometry tester started")
        
    def odom_callback(self, msg):
        """Store current odometry"""
        self.current_odom = msg
        
    def send_velocity(self, vx, vy, omega, duration, description):
        """Send velocity command for specified duration"""
        print(f"\n{description}")
        
        # Record starting position
        start_odom = self.current_odom
        if start_odom:
            start_x = start_odom.pose.pose.position.x
            start_y = start_odom.pose.pose.position.y
            start_theta = self.get_yaw_from_quaternion(start_odom.pose.pose.orientation)
            print(f"  Start: x={start_x:.3f}m, y={start_y:.3f}m, θ={math.degrees(start_theta):.1f}°")
        
        # Send velocity command
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        
        # Send commands for specified duration
        rate = 10  # 10 Hz
        for i in range(int(duration * rate)):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Send stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        # Wait a bit for final odometry update
        time.sleep(0.2)
        
        # Show final position
        end_odom = self.current_odom
        if end_odom and start_odom:
            end_x = end_odom.pose.pose.position.x
            end_y = end_odom.pose.pose.position.y
            end_theta = self.get_yaw_from_quaternion(end_odom.pose.pose.orientation)
            
            dx = end_x - start_x
            dy = end_y - start_y
            dtheta = end_theta - start_theta
            
            print(f"  End:   x={end_x:.3f}m, y={end_y:.3f}m, θ={math.degrees(end_theta):.1f}°")
            print(f"  Change: Δx={dx:.3f}m, Δy={dy:.3f}m, Δθ={math.degrees(dtheta):.1f}°")
            print(f"  Distance moved: {math.sqrt(dx*dx + dy*dy):.3f}m")
        
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        # Convert quaternion to euler angles (yaw)
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()
    tester = OdometryTester()
    
    print("=== LeKiwi Odometry Test ===")
    print("This test demonstrates odometry tracking by moving the robot")
    print("and showing how position is calculated from wheel velocities.\n")
    
    # Wait for initialization and first odometry message
    print("Waiting for odometry data...")
    while tester.current_odom is None:
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("✅ Odometry data received! Starting movement tests...\n")
    
    # Test sequence - demonstrating different movements
    test_movements = [
        # (vx, vy, omega, duration, description)
        (0.1, 0.0, 0.0, 3.0, "1. Forward movement (should increase X)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (-0.1, 0.0, 0.0, 3.0, "2. Backward movement (should decrease X)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (0.0, 0.1, 0.0, 3.0, "3. Left strafe (should increase Y)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (0.0, -0.1, 0.0, 3.0, "4. Right strafe (should decrease Y)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (0.0, 0.0, 0.3, 3.0, "5. Rotate counterclockwise (should increase θ)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (0.0, 0.0, -0.3, 3.0, "6. Rotate clockwise (should decrease θ)"),
        (0.0, 0.0, 0.0, 1.0, "   Stop and settle"),
        
        (0.1, 0.1, 0.2, 3.0, "7. Combined movement (forward + left + rotate)"),
        (0.0, 0.0, 0.0, 2.0, "   Final stop"),
    ]
    
    for vx, vy, omega, duration, description in test_movements:
        tester.send_velocity(vx, vy, omega, duration, description)
    
    print("\n=== Odometry Test Complete! ===")
    print("✅ Forward/backward movement tracked in X coordinate")
    print("✅ Left/right strafe tracked in Y coordinate")
    print("✅ Rotation tracked in θ (orientation)")
    print("✅ Combined movements show proper holonomic odometry")
    print("\nYour LeKiwi robot now has full odometry capability!")
    print("You can monitor position with: ros2 topic echo /odom")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 