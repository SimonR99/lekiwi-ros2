#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import argparse

class ZeroPoseTest(Node):
    def __init__(self, wait_time=2.0):
        super().__init__('zero_pose_test')
        
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/lekiwi_controller/joint_trajectory', 
            10)
            
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/lekiwi_gripper_controller/joint_trajectory',
            10)
            
        self.arm_joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]
        
        self.gripper_joint_names = [
            'Gripper'
        ]
        
        self.create_timer(wait_time, self.send_zero_pose)
        self.get_logger().info(f'Will send zero pose command in {wait_time} seconds')

    def send_zero_pose(self):
        # Send arm joints to zero
        arm_msg = JointTrajectory()
        arm_msg.joint_names = self.arm_joint_names
        
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [0.0] * len(self.arm_joint_names)
        arm_point.time_from_start.sec = 2
        
        arm_msg.points = [arm_point]
        self.arm_pub.publish(arm_msg)
        
        # Send gripper to zero
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = self.gripper_joint_names
        
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [0.0] * len(self.gripper_joint_names)
        gripper_point.time_from_start.sec = 2
        
        gripper_msg.points = [gripper_point]
        self.gripper_pub.publish(gripper_msg)
        
        self.get_logger().info('Sent zero pose command to arm and gripper')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait', type=float, default=2.0,
                      help='Time to wait before sending zero pose (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = ZeroPoseTest(wait_time=args.wait)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 