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
        
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('base_radius', 0.125)
        self.declare_parameter('max_wheel_velocity', 3.0)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('safety_check_rate', 10.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_radius = self.get_parameter('base_radius').value
        self.max_wheel_velocity = self.get_parameter('max_wheel_velocity').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.safety_check_rate = self.get_parameter('safety_check_rate').value
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            50)
        
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/lekiwi_wheel_controller/commands',
            50)
        
        self.last_cmd_time = time.time()
        self.is_stopped = False
        self.last_wheel_velocities = np.array([0.0, 0.0, 0.0])
        
        self.current_wheel_velocities = np.array([0.0, 0.0, 0.0])
        self.wheel_velocity_ramp_rate = 5.0
        
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        
        self.kinematic_matrix = np.array([
            [np.cos(angle), np.sin(angle), self.base_radius] 
            for angle in angles_rad
        ])
        
        self.safety_timer = self.create_timer(
            1.0 / self.safety_check_rate, 
            self.safety_check_callback
        )
        
        self.get_logger().info(f"Holonomic controller started")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}m, Base radius: {self.base_radius}m")
        self.get_logger().info(f"Max wheel velocity: {self.max_wheel_velocity} rad/s")
        self.get_logger().info(f"Command timeout: {self.cmd_timeout}s, Safety check rate: {self.safety_check_rate}Hz")
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        body_velocity = np.array([vx, vy, omega])
        
        wheel_linear_velocities = self.kinematic_matrix.dot(body_velocity)
        
        wheel_angular_velocities = wheel_linear_velocities / self.wheel_radius
        
        max_computed = np.max(np.abs(wheel_angular_velocities))
        if max_computed > self.max_wheel_velocity:
            scale_factor = self.max_wheel_velocity / max_computed
            wheel_angular_velocities *= scale_factor
            self.get_logger().warn(f"Velocity scaled by {scale_factor:.3f} to respect limits")
        
        self.last_wheel_velocities = wheel_angular_velocities
        
        dt = 0.1
        max_change = self.wheel_velocity_ramp_rate * dt
        
        for i in range(3):
            target = wheel_angular_velocities[i]
            current = self.current_wheel_velocities[i]
            
            if target > current:
                self.current_wheel_velocities[i] = min(target, current + max_change)
            elif target < current:
                self.current_wheel_velocities[i] = max(target, current - max_change)
        
        self.publish_wheel_velocities(self.current_wheel_velocities)
        
        if self.is_stopped:
            self.is_stopped = False
            self.get_logger().info("Safety stop cleared - new command received")
        
        self.get_logger().debug(
            f"cmd_vel: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f} -> "
            f"wheels: [{wheel_angular_velocities[0]:.3f}, "
            f"{wheel_angular_velocities[1]:.3f}, "
            f"{wheel_angular_velocities[2]:.3f}] rad/s"
        )
    
    def safety_check_callback(self):
        current_time = time.time()
        time_since_last_cmd = current_time - self.last_cmd_time
        
        if time_since_last_cmd > self.cmd_timeout:
            if not self.is_stopped:
                self.get_logger().warn(
                    f"Safety timeout! No cmd_vel received for {time_since_last_cmd:.2f}s "
                    f"(limit: {self.cmd_timeout}s) - stopping wheels"
                )
                
                zero_velocities = np.array([0.0, 0.0, 0.0])
                self.publish_wheel_velocities(zero_velocities)
                
                self.is_stopped = True
                self.last_wheel_velocities = zero_velocities
        
        elif self.is_stopped:
            zero_velocities = np.array([0.0, 0.0, 0.0])
            self.current_wheel_velocities = zero_velocities
            self.publish_wheel_velocities(zero_velocities)
    
    def publish_wheel_velocities(self, wheel_angular_velocities):
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
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0, 0.0, 0.0]
        controller.wheel_vel_pub.publish(stop_cmd)
        controller.get_logger().info("Sent stop command")
        
        rclpy.shutdown()

if __name__ == '__main__':
    main() 