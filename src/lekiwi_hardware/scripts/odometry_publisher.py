#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
import numpy as np
import math
import time
from tf_transformations import quaternion_from_euler

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.declare_parameter('wheel_radius', 0.05)  # 5cm wheel radius
        self.declare_parameter('base_radius', 0.125)  # 12.5cm from center to wheel
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_radius = self.get_parameter('base_radius').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.x = 0.0          # Robot position X (m)
        self.y = 0.0          # Robot position Y (m)
        self.theta = 0.0      # Robot orientation (rad)
        self.vx = 0.0         # Robot linear velocity X (m/s)
        self.vy = 0.0         # Robot linear velocity Y (m/s)
        self.vtheta = 0.0     # Robot angular velocity (rad/s)
        
        self.last_time = time.time()
        
        # Build inverse kinematic matrix for 3-wheel omni setup
        # Wheel angles: left=240°, rear=0°, right=120° (with -90° offset)
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        
        # Forward kinematic matrix: [vx, vy, omega] = K * [wheel_velocities]
        kinematic_matrix = np.array([
            [np.cos(angle), np.sin(angle), self.base_radius] 
            for angle in angles_rad
        ])
        
        # Inverse kinematic matrix: [wheel_velocities] = K_inv * [vx, vy, omega]
        # For odometry: [vx, vy, omega] = K_inv * [wheel_velocities]
        # Using Moore-Penrose pseudoinverse since 3x3 matrix may not be perfectly invertible
        self.inverse_kinematic_matrix = np.linalg.pinv(kinematic_matrix)
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.wheel_joint_names = ['left_wheel_drive', 'rear_wheel_drive', 'right_wheel_drive']
        
        self.get_logger().info(f"Odometry publisher started")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}m, Base radius: {self.base_radius}m")
        self.get_logger().info(f"Publishing TF: {self.publish_tf}")
        self.get_logger().info(f"Frames: {self.odom_frame_id} -> {self.base_frame_id}")
        
        self.publish_odometry(time.time())
        if self.publish_tf:
            self.publish_transform(time.time())
            
        self.timer = self.create_timer(0.1, self.publish_static_tf_callback)  # 10Hz
        
    def joint_state_callback(self, msg):
        try:
            wheel_velocities = []
            
            for wheel_name in self.wheel_joint_names:
                try:
                    idx = msg.name.index(wheel_name)
                    if len(msg.velocity) > idx and msg.velocity[idx] is not None:
                        wheel_vel = msg.velocity[idx]  # rad/s
                    else:
                        return
                    wheel_velocities.append(wheel_vel)
                except ValueError:
                    return
            
            wheel_linear_vels = np.array(wheel_velocities) * self.wheel_radius
            
            body_velocities = self.inverse_kinematic_matrix.dot(wheel_linear_vels)
            
            self.vx = body_velocities[0]      # m/s forward
            self.vy = body_velocities[1]      # m/s left
            self.vtheta = body_velocities[2]  # rad/s counterclockwise
            
            self.update_odometry()
            
        except Exception as e:
            self.get_logger().debug(f"Error processing joint states: {e}")
    
    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return
            
        # Integrate velocities to get pose change in global frame
        # Robot moves in its local frame, so we need to transform to global frame
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vtheta * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.publish_odometry(current_time)
        
        if self.publish_tf:
            self.publish_transform(current_time)
        
        self.last_time = current_time
        self.get_logger().debug(
            f"Odom: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°, "
            f"vx={self.vx:.3f}, vy={self.vy:.3f}, vθ={math.degrees(self.vtheta):.1f}°/s"
        )
    
    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vtheta
        
        # Covariance matrices (can be tuned based on wheel encoder accuracy)
        # For now, using conservative estimates
        pose_covar = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        twist_covar = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        
        odom.pose.covariance = pose_covar
        odom.twist.covariance = twist_covar
        
        self.odom_pub.publish(odom)
    
    def publish_transform(self, current_time):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_static_tf_callback(self):
        current_time = time.time()
        
        self.publish_odometry(current_time)
        
        if self.publish_tf:
            self.publish_transform(current_time)

def main():
    rclpy.init()
    odometry_publisher = OdometryPublisher()
    
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_publisher.get_logger().info("Odometry publisher shutting down")
        rclpy.shutdown()

if __name__ == '__main__':
    main() 