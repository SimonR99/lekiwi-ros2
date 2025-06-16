#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time

class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('wheel_radius', 0.05)  # 5cm wheel radius
        self.declare_parameter('base_radius', 0.125)  # 12.5cm from center to wheel
        self.declare_parameter('max_wheel_velocity', 3.0)  # max rad/s per wheel
        self.declare_parameter('cmd_timeout', 1.0)  # seconds - stop if no cmd_vel for this time
        self.declare_parameter('safety_check_rate', 10.0)  # Hz - rate to check for timeout
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_radius = self.get_parameter('base_radius').value
        self.max_wheel_velocity = self.get_parameter('max_wheel_velocity').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.safety_check_rate = self.get_parameter('safety_check_rate').value
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for wheel velocities
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/lekiwi_wheel_controller/commands',
            10)
        
        # Safety mechanism - track last command time
        self.last_cmd_time = time.time()
        self.is_stopped = False
        self.last_wheel_velocities = np.array([0.0, 0.0, 0.0])
        
        # Build kinematic matrix for 3-wheel omni setup
        # Wheel angles: left=240°, rear=0°, right=120° (with -90° offset)
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        
        # Kinematic matrix: each row converts [vx, vy, omega] to wheel linear velocity
        self.kinematic_matrix = np.array([
            [np.cos(angle), np.sin(angle), self.base_radius] 
            for angle in angles_rad
        ])
        
        # Safety timer to check for command timeout
        self.safety_timer = self.create_timer(
            1.0 / self.safety_check_rate, 
            self.safety_check_callback
        )
        
        self.get_logger().info(f"Holonomic controller started")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}m, Base radius: {self.base_radius}m")
        self.get_logger().info(f"Max wheel velocity: {self.max_wheel_velocity} rad/s")
        self.get_logger().info(f"Command timeout: {self.cmd_timeout}s, Safety check rate: {self.safety_check_rate}Hz")
        
    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to individual wheel velocities"""
        # Update last command time for safety mechanism
        self.last_cmd_time = time.time()
        
        # Extract body-frame velocities
        vx = msg.linear.x   # m/s forward
        vy = msg.linear.y   # m/s left  
        omega = msg.angular.z  # rad/s counterclockwise
        
        # Body velocity vector
        body_velocity = np.array([vx, vy, omega])
        
        # Convert to wheel linear velocities (m/s)
        wheel_linear_velocities = self.kinematic_matrix.dot(body_velocity)
        
        # Convert to wheel angular velocities (rad/s)
        wheel_angular_velocities = wheel_linear_velocities / self.wheel_radius
        
        # Apply velocity limits with proportional scaling
        max_computed = np.max(np.abs(wheel_angular_velocities))
        if max_computed > self.max_wheel_velocity:
            scale_factor = self.max_wheel_velocity / max_computed
            wheel_angular_velocities *= scale_factor
            self.get_logger().warn(f"Velocity scaled by {scale_factor:.3f} to respect limits")
        
        # Store last commanded velocities
        self.last_wheel_velocities = wheel_angular_velocities
        
        # Publish wheel velocities [left, rear, right]
        self.publish_wheel_velocities(wheel_angular_velocities)
        
        # Reset stopped flag if we were stopped
        if self.is_stopped:
            self.is_stopped = False
            self.get_logger().info("Safety stop cleared - new command received")
        
        # Debug info
        self.get_logger().debug(
            f"cmd_vel: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f} -> "
            f"wheels: [{wheel_angular_velocities[0]:.3f}, "
            f"{wheel_angular_velocities[1]:.3f}, "
            f"{wheel_angular_velocities[2]:.3f}] rad/s"
        )
    
    def safety_check_callback(self):
        """Check if command timeout has occurred and stop wheels if needed"""
        current_time = time.time()
        time_since_last_cmd = current_time - self.last_cmd_time
        
        if time_since_last_cmd > self.cmd_timeout:
            # Timeout occurred - stop the robot
            if not self.is_stopped:
                self.get_logger().warn(
                    f"Safety timeout! No cmd_vel received for {time_since_last_cmd:.2f}s "
                    f"(limit: {self.cmd_timeout}s) - stopping wheels"
                )
                
                # Send zero velocities
                zero_velocities = np.array([0.0, 0.0, 0.0])
                self.publish_wheel_velocities(zero_velocities)
                
                self.is_stopped = True
                self.last_wheel_velocities = zero_velocities
        
        # If stopped and no timeout, continue sending zeros to ensure wheels stay stopped
        elif self.is_stopped:
            zero_velocities = np.array([0.0, 0.0, 0.0])
            self.publish_wheel_velocities(zero_velocities)
    
    def publish_wheel_velocities(self, wheel_angular_velocities):
        """Helper method to publish wheel velocities"""
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = wheel_angular_velocities.tolist()
        self.wheel_vel_pub.publish(wheel_cmd)

def main():
    rclpy.init()
    controller = HolonomicController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutdown
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0, 0.0, 0.0]
        controller.wheel_vel_pub.publish(stop_cmd)
        controller.get_logger().info("Sent stop command")
        
        rclpy.shutdown()

if __name__ == '__main__':
    main() 