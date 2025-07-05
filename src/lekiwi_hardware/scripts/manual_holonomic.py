#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time
import select

class ManualHolonomicController(Node):
    def __init__(self):
        super().__init__('manual_holonomic_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_speed = 0.1   # m/s
        self.angular_speed = 0.3  # rad/s
        self.speed_increment = 0.05
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        self.last_key_time = time.time()
        self.key_timeout = 0.25
        
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        self.get_logger().info("Manual holonomic controller started")
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("LeKiwi Manual Holonomic Control")
        print("="*50)
        print("Movement Keys (HOLD to move):")
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
        print("NOTE: You must HOLD keys to move - releasing stops the robot")
        print("Holonomic drive control ready")
        
    def publish_velocity(self):
        if time.time() - self.last_key_time > self.key_timeout:
            if self.current_vx != 0.0 or self.current_vy != 0.0 or self.current_omega != 0.0:
                self.stop_robot()
        
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_omega
        
        self.cmd_vel_pub.publish(msg)
        
    def stop_robot(self):
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self.get_logger().info("Robot stopped")
        
    def handle_key(self, key):
        if key.lower() in ['w', 's', 'a', 'd', 'q', 'e'] or key.startswith('\x1b['):
            self.last_key_time = time.time()
        
        if key.lower() == 'w' or key == '\x1b[A':
            self.current_vx = self.linear_speed
            self.current_vy = 0.0
            self.current_omega = 0.0
            self.get_logger().info("Moving forward")
            
        elif key.lower() == 's' or key == '\x1b[B':
            self.current_vx = -self.linear_speed
            self.current_vy = 0.0
            self.current_omega = 0.0
            self.get_logger().info("Moving backward")
            
        elif key.lower() == 'a' or key == '\x1b[D':
            self.current_vx = 0.0
            self.current_vy = self.linear_speed
            self.current_omega = 0.0
            self.get_logger().info("Strafing left")
            
        elif key.lower() == 'd' or key == '\x1b[C':
            self.current_vx = 0.0
            self.current_vy = -self.linear_speed
            self.current_omega = 0.0
            self.get_logger().info("Strafing right")
            
        elif key.lower() == 'q':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = self.angular_speed
            self.get_logger().info("Rotating left")
            
        elif key.lower() == 'e':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = -self.angular_speed
            self.get_logger().info("Rotating right")
            
        elif key == ' ':
            self.stop_robot()
            
        elif key == '+' or key == '=':
            self.linear_speed = min(0.5, self.linear_speed + self.speed_increment)
            self.get_logger().info(f"Linear speed: {self.linear_speed:.2f} m/s")
            
        elif key == '-':
            self.linear_speed = max(0.05, self.linear_speed - self.speed_increment)
            self.get_logger().info(f"Linear speed: {self.linear_speed:.2f} m/s")
            
        elif key == ']':
            self.angular_speed = min(1.0, self.angular_speed + 0.1)
            self.get_logger().info(f"Angular speed: {self.angular_speed:.2f} rad/s")
            
        elif key == '[':
            self.angular_speed = max(0.1, self.angular_speed - 0.1)
            self.get_logger().info(f"Angular speed: {self.angular_speed:.2f} rad/s")
            
        elif key.lower() == 'x':
            return False
            
        return True

def get_key_non_blocking():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        
        if select.select([sys.stdin], [], [], 0.05)[0]:
            key = sys.stdin.read(1)
            
            if key == '\x1b':
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key += sys.stdin.read(2)
            return key
        else:
            return None
            
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    controller = ManualHolonomicController()
    
    def spin_node():
        rclpy.spin(controller)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        while True:
            key = get_key_non_blocking()
            if key is not None:
                if not controller.handle_key(key):
                    break
            time.sleep(0.02)
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        time.sleep(0.1)
        
        print("\nShutting down manual controller...")
        rclpy.shutdown()

if __name__ == '__main__':
    main() 