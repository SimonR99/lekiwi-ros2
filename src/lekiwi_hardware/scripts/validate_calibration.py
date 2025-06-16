#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import sys

class CalibrationValidator(Node):
    def __init__(self):
        super().__init__('calibration_validator')
        
        # Load calibration data
        self.load_calibration()
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info("Calibration validator started. Monitoring joint states...")
        
    def load_calibration(self):
        try:
            with open('/home/simon/workspace/new_tentative/src/lekiwi_hardware/config/calibration.yaml', 'r') as file:
                self.calib_data = yaml.safe_load(file)
                self.get_logger().info("Loaded calibration data")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            self.calib_data = None
    
    def joint_state_callback(self, msg):
        if not self.calib_data:
            return
            
        self.get_logger().info("=== Joint State Analysis ===")
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos_rad = msg.position[i]
                pos_deg = pos_rad * 180.0 / 3.14159
                
                # Check if joint is in calibration data
                if name in self.calib_data['joints']:
                    calib = self.calib_data['joints'][name]
                    min_ticks = calib.get('min', {}).get('ticks', 0)
                    max_ticks = calib.get('max', {}).get('ticks', 0)
                    center_ticks = calib.get('center', {}).get('ticks', 0)
                    
                    # Calculate expected ticks for current position
                    # Assuming 4096 ticks = 360 degrees
                    expected_ticks = 2048 + (pos_deg / 360.0 * 4096)
                    
                    self.get_logger().info(f"{name}: {pos_deg:.1f}° ({pos_rad:.3f}rad)")
                    self.get_logger().info(f"  Calib range: {min_ticks} - {max_ticks} (center: {center_ticks})")
                    self.get_logger().info(f"  Expected ticks: {expected_ticks:.0f}")
                    
                    # Check for issues
                    if min_ticks > max_ticks:
                        self.get_logger().warn(f"  ⚠️  INVERTED RANGE: min > max")
                    if center_ticks < min_ticks or center_ticks > max_ticks:
                        self.get_logger().warn(f"  ⚠️  CENTER OUTSIDE RANGE")
                else:
                    self.get_logger().info(f"{name}: {pos_deg:.1f}° ({pos_rad:.3f}rad) - NO CALIBRATION DATA")
        
        self.get_logger().info("========================")

def main():
    rclpy.init()
    validator = CalibrationValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 