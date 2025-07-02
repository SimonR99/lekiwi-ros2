#!/usr/bin/env python3
"""
Simulation Adapter for Dino AI

Implements the abstract interfaces using simulation for testing and development.
Allows Dino AI to work without any hardware for testing purposes.
"""

import time
import random
import threading
from typing import Dict, Any, List, Optional
import logging
import numpy as np
from PIL import Image as PILImage

# Core interfaces
from dino_ai.core.interfaces import (
    PlatformInterface, NavigationInterface, ManipulationInterface,
    VisionInterface, SpeechInterface, SystemInterface,
    Pose, JointState, SystemHealth, SystemStatus
)

logger = logging.getLogger(__name__)


class SimulationNavigationAdapter(NavigationInterface):
    """Simulation implementation of navigation interface"""
    
    def __init__(self):
        self.current_pose = Pose(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self.navigation_status = {"status": "idle"}
        self.max_speed = 1.0  # m/s
    
    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose
    
    def navigate_to_pose(self, pose: Pose) -> bool:
        """Simulate navigation to target pose"""
        try:
            logger.info(f"Simulating navigation to ({pose.x:.2f}, {pose.y:.2f}, {pose.yaw:.2f})")
            
            # Simulate movement over time
            self.navigation_status = {"status": "navigating", "target": pose}
            
            # Simple linear interpolation to target
            start_pose = self.current_pose
            
            def simulate_movement():
                steps = 20
                for i in range(steps + 1):
                    t = i / steps
                    self.current_pose = Pose(
                        x=start_pose.x + t * (pose.x - start_pose.x),
                        y=start_pose.y + t * (pose.y - start_pose.y),
                        z=start_pose.z + t * (pose.z - start_pose.z),
                        yaw=start_pose.yaw + t * (pose.yaw - start_pose.yaw)
                    )
                    time.sleep(0.1)  # 2 second total movement
                
                self.navigation_status = {"status": "reached"}
            
            # Run in background thread
            threading.Thread(target=simulate_movement, daemon=True).start()
            
            return True
        except Exception as e:
            logger.error(f"Simulated navigation failed: {e}")
            return False
    
    def move_relative(self, dx: float, dy: float, dyaw: float) -> bool:
        """Move relative to current position"""
        target = Pose(
            x=self.current_pose.x + dx,
            y=self.current_pose.y + dy,
            z=self.current_pose.z,
            yaw=self.current_pose.yaw + dyaw
        )
        return self.navigate_to_pose(target)
    
    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """Set velocity commands"""
        try:
            # Clamp velocities
            linear_x = max(-self.max_speed, min(self.max_speed, linear_x))
            linear_y = max(-self.max_speed, min(self.max_speed, linear_y))
            angular_z = max(-2.0, min(2.0, angular_z))
            
            logger.debug(f"Setting velocity: linear=({linear_x:.2f}, {linear_y:.2f}), angular={angular_z:.2f}")
            
            # Update pose based on velocity (simple integration)
            dt = 0.1
            self.current_pose.x += linear_x * dt
            self.current_pose.y += linear_y * dt
            self.current_pose.yaw += angular_z * dt
            
            return True
        except Exception as e:
            logger.error(f"Velocity command failed: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop all motion"""
        self.navigation_status = {"status": "stopped"}
        return True
    
    def get_navigation_status(self) -> Dict[str, Any]:
        return {
            **self.navigation_status,
            "current_pose": {
                "x": self.current_pose.x,
                "y": self.current_pose.y,
                "yaw": self.current_pose.yaw
            }
        }
    
    def is_path_clear(self, target_pose: Pose) -> bool:
        """Simulate path checking"""
        # Randomly return true/false for simulation
        return random.random() > 0.1  # 90% chance path is clear


class SimulationManipulationAdapter(ManipulationInterface):
    """Simulation implementation of manipulation interface"""
    
    def __init__(self):
        self.joint_names = ["shoulder_rotation", "shoulder_pitch", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial pose
        self.end_effector_pose = Pose(x=0.5, y=0.0, z=0.3)  # Default arm position
        self.manipulation_status = {"status": "idle"}
        self.gripper_open = True
    
    def get_joint_states(self) -> Optional[JointState]:
        return JointState(
            names=self.joint_names,
            positions=self.joint_positions,
            velocities=[0.0] * len(self.joint_names),
            efforts=[0.0] * len(self.joint_names)
        )
    
    def get_end_effector_pose(self) -> Optional[Pose]:
        return self.end_effector_pose
    
    def move_to_pose(self, pose: Pose) -> bool:
        """Simulate moving to target pose"""
        try:
            logger.info(f"Simulating arm movement to ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})")
            
            # Check if pose is reachable
            if not self.is_pose_reachable(pose):
                logger.warning("Target pose not reachable")
                return False
            
            self.manipulation_status = {"status": "moving", "target": pose}
            
            # Simulate movement over time
            start_pose = self.end_effector_pose
            
            def simulate_movement():
                steps = 30
                for i in range(steps + 1):
                    t = i / steps
                    self.end_effector_pose = Pose(
                        x=start_pose.x + t * (pose.x - start_pose.x),
                        y=start_pose.y + t * (pose.y - start_pose.y),
                        z=start_pose.z + t * (pose.z - start_pose.z),
                        roll=start_pose.roll + t * (pose.roll - start_pose.roll),
                        pitch=start_pose.pitch + t * (pose.pitch - start_pose.pitch),
                        yaw=start_pose.yaw + t * (pose.yaw - start_pose.yaw)
                    )
                    
                    # Update joint positions (simplified inverse kinematics)
                    self._update_joints_from_pose()
                    
                    time.sleep(0.1)  # 3 second total movement
                
                self.manipulation_status = {"status": "reached"}
            
            threading.Thread(target=simulate_movement, daemon=True).start()
            
            return True
        except Exception as e:
            logger.error(f"Simulated arm movement failed: {e}")
            return False
    
    def move_to_joint_positions(self, positions: List[float]) -> bool:
        """Move to joint positions"""
        try:
            if len(positions) != len(self.joint_names):
                logger.error(f"Expected {len(self.joint_names)} positions, got {len(positions)}")
                return False
            
            logger.info(f"Simulating joint movement to: {positions}")
            
            # Simulate movement
            start_positions = self.joint_positions.copy()
            
            def simulate_movement():
                steps = 20
                for i in range(steps + 1):
                    t = i / steps
                    for j in range(len(positions)):
                        self.joint_positions[j] = start_positions[j] + t * (positions[j] - start_positions[j])
                    
                    # Update end effector pose (simplified forward kinematics)
                    self._update_pose_from_joints()
                    
                    time.sleep(0.1)
            
            threading.Thread(target=simulate_movement, daemon=True).start()
            
            return True
        except Exception as e:
            logger.error(f"Joint movement failed: {e}")
            return False
    
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """Move end effector relative to current position"""
        target = Pose(
            x=self.end_effector_pose.x + dx,
            y=self.end_effector_pose.y + dy,
            z=self.end_effector_pose.z + dz,
            roll=self.end_effector_pose.roll,
            pitch=self.end_effector_pose.pitch,
            yaw=self.end_effector_pose.yaw
        )
        return self.move_to_pose(target)
    
    def open_gripper(self) -> bool:
        """Open gripper"""
        try:
            logger.info("Opening gripper")
            self.gripper_open = True
            self.joint_positions[-1] = 1.0  # Open position
            return True
        except Exception as e:
            logger.error(f"Gripper open failed: {e}")
            return False
    
    def close_gripper(self) -> bool:
        """Close gripper"""
        try:
            logger.info("Closing gripper")
            self.gripper_open = False
            self.joint_positions[-1] = 0.0  # Closed position
            return True
        except Exception as e:
            logger.error(f"Gripper close failed: {e}")
            return False
    
    def get_manipulation_status(self) -> Dict[str, Any]:
        return {
            **self.manipulation_status,
            "gripper_open": self.gripper_open,
            "joint_positions": self.joint_positions
        }
    
    def is_pose_reachable(self, pose: Pose) -> bool:
        """Check if pose is reachable (simplified workspace check)"""
        # Simple spherical workspace
        distance = (pose.x**2 + pose.y**2 + pose.z**2)**0.5
        return 0.1 <= distance <= 0.8  # Arm reach limits
    
    def _update_joints_from_pose(self):
        """Simplified inverse kinematics"""
        # This is a very simplified version
        # In reality, you'd use proper IK
        pose = self.end_effector_pose
        
        # Simple approximation
        self.joint_positions[0] = np.arctan2(pose.y, pose.x)  # Shoulder rotation
        self.joint_positions[1] = -np.arctan2(pose.z - 0.1, (pose.x**2 + pose.y**2)**0.5)  # Shoulder pitch
        # Other joints would be calculated properly in a real system
    
    def _update_pose_from_joints(self):
        """Simplified forward kinematics"""
        # This is a very simplified version
        # In reality, you'd use proper FK
        
        # Simple approximation based on joint angles
        r = 0.5  # Approximate arm length
        self.end_effector_pose.x = r * np.cos(self.joint_positions[0]) * np.cos(self.joint_positions[1])
        self.end_effector_pose.y = r * np.sin(self.joint_positions[0]) * np.cos(self.joint_positions[1])
        self.end_effector_pose.z = 0.1 + r * np.sin(self.joint_positions[1])


class SimulationVisionAdapter(VisionInterface):
    """Simulation implementation of vision interface"""
    
    def __init__(self):
        self.vision_status = {"status": "active"}
        self._generate_test_image()
    
    def _generate_test_image(self):
        """Generate a test image"""
        # Create a simple test image
        img_array = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Add some geometric shapes
        # Red rectangle
        img_array[100:200, 100:300] = [255, 0, 0]
        
        # Green circle (approximation)
        center_y, center_x = 300, 400
        radius = 50
        y, x = np.ogrid[:480, :640]
        mask = (x - center_x)**2 + (y - center_y)**2 <= radius**2
        img_array[mask] = [0, 255, 0]
        
        # Blue square
        img_array[350:450, 500:600] = [0, 0, 255]
        
        self.current_image = PILImage.fromarray(img_array)
    
    def get_current_image(self) -> Optional[PILImage.Image]:
        # Occasionally generate a new random image
        if random.random() < 0.1:  # 10% chance
            self._generate_test_image()
        
        return self.current_image
    
    def analyze_scene(self, prompt: str = "Describe what you see") -> Optional[str]:
        """Simulate AI vision analysis"""
        scene_descriptions = [
            "I can see a room with colorful geometric shapes. There's a red rectangle on the left, a green circle in the center, and a blue square on the right.",
            "The scene shows abstract geometric patterns with bright colors against a noisy background.",
            "I observe three distinct objects: a red rectangular shape, a circular green object, and a blue square shape.",
            "The image contains geometric forms in primary colors arranged across the frame."
        ]
        
        # Return a random description
        description = random.choice(scene_descriptions)
        logger.info(f"Vision analysis: {description}")
        return description
    
    def detect_objects(self) -> List[Dict[str, Any]]:
        """Simulate object detection"""
        objects = [
            {"name": "red_rectangle", "confidence": 0.95, "bbox": [100, 100, 300, 200]},
            {"name": "green_circle", "confidence": 0.88, "bbox": [350, 350, 450, 450]},
            {"name": "blue_square", "confidence": 0.92, "bbox": [500, 350, 600, 450]}
        ]
        
        # Randomly vary the objects
        return random.sample(objects, random.randint(1, len(objects)))
    
    def find_object(self, object_name: str) -> Optional[Dict[str, Any]]:
        """Find specific object in view"""
        objects = self.detect_objects()
        for obj in objects:
            if object_name.lower() in obj["name"].lower():
                return obj
        return None
    
    def save_image(self, filename: str) -> bool:
        """Save current image to file"""
        try:
            if self.current_image:
                self.current_image.save(filename)
                logger.info(f"Saved simulated image to {filename}")
                return True
        except Exception as e:
            logger.error(f"Image save failed: {e}")
        return False
    
    def get_vision_status(self) -> Dict[str, Any]:
        return {
            **self.vision_status,
            "has_image": True,
            "image_size": (640, 480),
            "simulated": True
        }


class SimulationSpeechAdapter(SpeechInterface):
    """Simulation implementation of speech interface"""
    
    def __init__(self):
        self.speech_status = {"status": "active"}
        self.current_voice = "simulation_voice"
    
    def speak(self, text: str) -> bool:
        """Simulate text-to-speech"""
        print(f"🤖💬 {text}")
        logger.info(f"Simulated TTS: {text}")
        return True
    
    def listen(self) -> Optional[str]:
        """Simulate speech-to-text"""
        try:
            # In simulation, we use text input
            user_input = input("🎤 Speak (or type): ").strip()
            if user_input:
                logger.info(f"Simulated STT: {user_input}")
                return user_input
        except (EOFError, KeyboardInterrupt):
            pass
        return None
    
    def set_voice(self, voice_id: str) -> bool:
        """Set TTS voice"""
        self.current_voice = voice_id
        logger.info(f"Set voice to: {voice_id}")
        return True
    
    def get_speech_status(self) -> Dict[str, Any]:
        return {
            **self.speech_status,
            "current_voice": self.current_voice,
            "simulated": True
        }
    
    def is_listening(self) -> bool:
        return False


class SimulationSystemAdapter(SystemInterface):
    """Simulation implementation of system interface"""
    
    def __init__(self):
        self.start_time = time.time()
        self.system_mode = "simulation"
        self.emergency_stopped = False
    
    def get_system_health(self) -> SystemHealth:
        """Get simulated system health"""
        return SystemHealth(
            overall_status=SystemStatus.READY if not self.emergency_stopped else SystemStatus.ERROR,
            components={
                "navigation": SystemStatus.READY,
                "manipulation": SystemStatus.READY,
                "vision": SystemStatus.READY,
                "speech": SystemStatus.READY
            },
            errors=["Emergency stop active"] if self.emergency_stopped else [],
            warnings=[],
            metrics={
                "uptime": time.time() - self.start_time,
                "cpu_usage": random.uniform(10, 30),
                "memory_usage": random.uniform(40, 60)
            }
        )
    
    def emergency_stop(self) -> bool:
        """Simulate emergency stop"""
        self.emergency_stopped = True
        logger.warning("SIMULATION: Emergency stop activated")
        return True
    
    def reset_system(self) -> bool:
        """Reset system to initial state"""
        self.emergency_stopped = False
        logger.info("SIMULATION: System reset")
        return True
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get simulated system information"""
        return {
            "platform": "Simulation Platform",
            "version": "1.0.0",
            "mode": self.system_mode,
            "uptime": time.time() - self.start_time,
            "simulation": True
        }
    
    def set_system_mode(self, mode: str) -> bool:
        """Set system operation mode"""
        self.system_mode = mode
        logger.info(f"SIMULATION: Set system mode to {mode}")
        return True


class SimulationPlatformAdapter(PlatformInterface):
    """Main simulation platform adapter"""
    
    def __init__(self):
        super().__init__()
        self._initialized = False
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize the simulation platform"""
        try:
            # Initialize all interfaces
            self.navigation = SimulationNavigationAdapter()
            self.manipulation = SimulationManipulationAdapter()
            self.vision = SimulationVisionAdapter()
            self.speech = SimulationSpeechAdapter()
            self.system = SimulationSystemAdapter()
            
            self._initialized = True
            logger.info("Simulation Platform Adapter initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Simulation platform initialization failed: {e}")
            return False
    
    def shutdown(self) -> bool:
        """Shutdown the simulation platform"""
        try:
            self._initialized = False
            logger.info("Simulation Platform Adapter shutdown complete")
            return True
            
        except Exception as e:
            logger.error(f"Simulation platform shutdown failed: {e}")
            return False
    
    def get_available_capabilities(self) -> List[str]:
        """Get list of available capabilities"""
        return [
            "navigation",
            "manipulation",
            "vision", 
            "speech",
            "simulation"
        ]
    
    def is_capability_available(self, capability: str) -> bool:
        """Check if specific capability is available"""
        return capability in self.get_available_capabilities()


def create_simulation_platform() -> PlatformInterface:
    """Factory function to create simulation platform adapter"""
    return SimulationPlatformAdapter()