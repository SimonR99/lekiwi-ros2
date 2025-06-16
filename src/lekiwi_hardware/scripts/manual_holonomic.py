#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time

class ManualHolonomicController(Node):
    def __init__(self):
        super().__init__('manual_holonomic_controller')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 0.1   # m/s
        self.angular_speed = 0.3  # rad/s
        self.speed_increment = 0.05
        
        # Current velocity
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Publishing timer
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info("Manual holonomic controller started")
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("LeKiwi Manual Holonomic Control")
        print("="*50)
        print("Movement Keys:")
        print("  W/S     - Forward/Backward")
        print("  A/D     - Strafe Left/Right")  
        print("  Q/E     - Rotate Left/Right")
        print("  Arrow ↑↓ - Forward/Backward")
        print("  Arrow ←→ - Strafe Left/Right")
        print("")
        print("Speed Control:")
        print("  +/-     - Increase/Decrease linear speed")
        print("  [/]     - Increase/Decrease angular speed")
        print("  SPACE   - Emergency stop")
        print("  X       - Quit")
        print("")
        print(f"Current speeds: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s")
        print("="*50)
        
    def publish_velocity(self):
        """Publish current velocity command"""
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_omega
        
        self.cmd_vel_pub.publish(msg)
        
    def stop_robot(self):
        """Stop all movement"""
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self.get_logger().info("Robot stopped")
        
    def handle_key(self, key):
        """Handle keyboard input"""
        if key.lower() == 'w' or key == '\x1b[A':  # W or Up arrow
            self.current_vx = self.linear_speed
            self.current_vy = 0.0
            self.get_logger().info("Moving forward")
            
        elif key.lower() == 's' or key == '\x1b[B':  # S or Down arrow
            self.current_vx = -self.linear_speed
            self.current_vy = 0.0
            self.get_logger().info("Moving backward")
            
        elif key.lower() == 'a' or key == '\x1b[D':  # A or Left arrow
            self.current_vx = 0.0
            self.current_vy = self.linear_speed
            self.get_logger().info("Strafing left")
            
        elif key.lower() == 'd' or key == '\x1b[C':  # D or Right arrow
            self.current_vx = 0.0
            self.current_vy = -self.linear_speed
            self.get_logger().info("Strafing right")
            
        elif key.lower() == 'q':  # Rotate left
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = self.angular_speed
            self.get_logger().info("Rotating left")
            
        elif key.lower() == 'e':  # Rotate right
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = -self.angular_speed
            self.get_logger().info("Rotating right")
            
        elif key == ' ':  # Space - Emergency stop
            self.stop_robot()
            
        elif key == '+' or key == '=':  # Increase linear speed
            self.linear_speed = min(0.5, self.linear_speed + self.speed_increment)
            self.get_logger().info(f"Linear speed: {self.linear_speed:.2f} m/s")
            
        elif key == '-':  # Decrease linear speed
            self.linear_speed = max(0.05, self.linear_speed - self.speed_increment)
            self.get_logger().info(f"Linear speed: {self.linear_speed:.2f} m/s")
            
        elif key == ']':  # Increase angular speed
            self.angular_speed = min(1.0, self.angular_speed + 0.1)
            self.get_logger().info(f"Angular speed: {self.angular_speed:.2f} rad/s")
            
        elif key == '[':  # Decrease angular speed
            self.angular_speed = max(0.1, self.angular_speed - 0.1)
            self.get_logger().info(f"Angular speed: {self.angular_speed:.2f} rad/s")
            
        elif key.lower() == 'x':  # Exit
            return False
            
        return True

def get_key():
    """Get a single keypress from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        
        # Handle arrow keys (escape sequences)
        if key == '\x1b':
            key += sys.stdin.read(2)
            
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rclpy.init()
    controller = ManualHolonomicController()
    
    # Run ROS2 node in separate thread
    def spin_node():
        rclpy.spin(controller)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        while True:
            key = get_key()
            if not controller.handle_key(key):
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutdown
        controller.stop_robot()
        time.sleep(0.1)  # Give time for stop command to send
        
        print("\nShutting down manual controller...")
        rclpy.shutdown()

if __name__ == '__main__':
    main() 