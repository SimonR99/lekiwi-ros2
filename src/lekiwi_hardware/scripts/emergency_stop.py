#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Create service client for torque control
        self.torque_client = self.create_client(Trigger, '/torque')
        
        # Wait for service to be available
        self.get_logger().info("Waiting for torque service...")
        if not self.torque_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Torque service not available!")
            sys.exit(1)
        
        self.get_logger().info("Emergency stop ready. Disabling torque...")
        self.disable_torque()
    
    def disable_torque(self):
        request = Trigger.Request()
        
        future = self.torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Torque service response: {response.message}")
            if "disabled" in response.message.lower():
                self.get_logger().info("✅ Torque successfully disabled!")
            else:
                self.get_logger().warn("⚠️  Torque may still be enabled. Check robot state.")
        else:
            self.get_logger().error("Failed to call torque service")

def main():
    rclpy.init()
    
    try:
        emergency_stop = EmergencyStop()
        print("Emergency stop completed. Robot should be safe to handle.")
    except Exception as e:
        print(f"Emergency stop failed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 