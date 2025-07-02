#!/usr/bin/env python3
"""
Abstract Hardware Interfaces for Dino AI

Defines abstract interfaces that can be implemented for different platforms:
- ROS-based robots
- Direct hardware control
- Simulation environments
- Cloud/remote systems
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import Enum
import numpy as np
from PIL import Image


class SystemStatus(Enum):
    """System status enumeration"""
    UNKNOWN = "unknown"
    INITIALIZING = "initializing"
    READY = "ready"
    ACTIVE = "active"
    ERROR = "error"
    DISABLED = "disabled"


@dataclass
class Pose:
    """3D pose representation"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class JointState:
    """Joint state representation"""
    names: List[str]
    positions: List[float]
    velocities: Optional[List[float]] = None
    efforts: Optional[List[float]] = None


@dataclass
class SystemHealth:
    """System health information"""
    overall_status: SystemStatus
    components: Dict[str, SystemStatus]
    errors: List[str]
    warnings: List[str]
    metrics: Dict[str, float]


class NavigationInterface(ABC):
    """Abstract interface for robot navigation"""
    
    @abstractmethod
    def get_current_pose(self) -> Optional[Pose]:
        """Get current robot pose"""
        pass
    
    @abstractmethod
    def navigate_to_pose(self, pose: Pose) -> bool:
        """Navigate to target pose"""
        pass
    
    @abstractmethod
    def move_relative(self, dx: float, dy: float, dyaw: float) -> bool:
        """Move relative to current position"""
        pass
    
    @abstractmethod
    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """Set velocity commands"""
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """Stop all motion"""
        pass
    
    @abstractmethod
    def get_navigation_status(self) -> Dict[str, Any]:
        """Get navigation system status"""
        pass
    
    @abstractmethod
    def is_path_clear(self, target_pose: Pose) -> bool:
        """Check if path to target is clear"""
        pass


class ManipulationInterface(ABC):
    """Abstract interface for robot manipulation"""
    
    @abstractmethod
    def get_joint_states(self) -> Optional[JointState]:
        """Get current joint states"""
        pass
    
    @abstractmethod
    def get_end_effector_pose(self) -> Optional[Pose]:
        """Get current end effector pose"""
        pass
    
    @abstractmethod
    def move_to_pose(self, pose: Pose) -> bool:
        """Move end effector to target pose"""
        pass
    
    @abstractmethod
    def move_to_joint_positions(self, positions: List[float]) -> bool:
        """Move to joint positions"""
        pass
    
    @abstractmethod
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """Move end effector relative to current position"""
        pass
    
    @abstractmethod
    def open_gripper(self) -> bool:
        """Open gripper"""
        pass
    
    @abstractmethod
    def close_gripper(self) -> bool:
        """Close gripper"""
        pass
    
    @abstractmethod
    def get_manipulation_status(self) -> Dict[str, Any]:
        """Get manipulation system status"""
        pass
    
    @abstractmethod
    def is_pose_reachable(self, pose: Pose) -> bool:
        """Check if pose is reachable"""
        pass


class VisionInterface(ABC):
    """Abstract interface for computer vision"""
    
    @abstractmethod
    def get_current_image(self) -> Optional[Image.Image]:
        """Get current camera image"""
        pass
    
    @abstractmethod
    def analyze_scene(self, prompt: str = "Describe what you see") -> Optional[str]:
        """Analyze current scene with AI vision"""
        pass
    
    @abstractmethod
    def detect_objects(self) -> List[Dict[str, Any]]:
        """Detect objects in current view"""
        pass
    
    @abstractmethod
    def find_object(self, object_name: str) -> Optional[Dict[str, Any]]:
        """Find specific object in view"""
        pass
    
    @abstractmethod
    def save_image(self, filename: str) -> bool:
        """Save current image to file"""
        pass
    
    @abstractmethod
    def get_vision_status(self) -> Dict[str, Any]:
        """Get vision system status"""
        pass


class SpeechInterface(ABC):
    """Abstract interface for speech/audio"""
    
    @abstractmethod
    def speak(self, text: str) -> bool:
        """Convert text to speech"""
        pass
    
    @abstractmethod
    def listen(self) -> Optional[str]:
        """Listen for speech input"""
        pass
    
    @abstractmethod
    def set_voice(self, voice_id: str) -> bool:
        """Set TTS voice"""
        pass
    
    @abstractmethod
    def get_speech_status(self) -> Dict[str, Any]:
        """Get speech system status"""
        pass
    
    @abstractmethod
    def is_listening(self) -> bool:
        """Check if actively listening"""
        pass


class SystemInterface(ABC):
    """Abstract interface for system control and monitoring"""
    
    @abstractmethod
    def get_system_health(self) -> SystemHealth:
        """Get overall system health"""
        pass
    
    @abstractmethod
    def emergency_stop(self) -> bool:
        """Emergency stop all systems"""
        pass
    
    @abstractmethod
    def reset_system(self) -> bool:
        """Reset system to initial state"""
        pass
    
    @abstractmethod
    def get_system_info(self) -> Dict[str, Any]:
        """Get system information"""
        pass
    
    @abstractmethod
    def set_system_mode(self, mode: str) -> bool:
        """Set system operation mode"""
        pass


class PlatformInterface(ABC):
    """Main platform interface that aggregates all subsystems"""
    
    def __init__(self):
        self.navigation: Optional[NavigationInterface] = None
        self.manipulation: Optional[ManipulationInterface] = None
        self.vision: Optional[VisionInterface] = None
        self.speech: Optional[SpeechInterface] = None
        self.system: Optional[SystemInterface] = None
    
    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize the platform"""
        pass
    
    @abstractmethod
    def shutdown(self) -> bool:
        """Shutdown the platform"""
        pass
    
    @abstractmethod
    def get_available_capabilities(self) -> List[str]:
        """Get list of available capabilities"""
        pass
    
    @abstractmethod
    def is_capability_available(self, capability: str) -> bool:
        """Check if specific capability is available"""
        pass
    
    def has_navigation(self) -> bool:
        """Check if navigation is available"""
        return self.navigation is not None
    
    def has_manipulation(self) -> bool:
        """Check if manipulation is available"""
        return self.manipulation is not None
    
    def has_vision(self) -> bool:
        """Check if vision is available"""
        return self.vision is not None
    
    def has_speech(self) -> bool:
        """Check if speech is available"""
        return self.speech is not None


# Null implementations for systems without certain capabilities

class NullNavigationInterface(NavigationInterface):
    """Null implementation for systems without navigation"""
    
    def get_current_pose(self) -> Optional[Pose]:
        return None
    
    def navigate_to_pose(self, pose: Pose) -> bool:
        return False
    
    def move_relative(self, dx: float, dy: float, dyaw: float) -> bool:
        return False
    
    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        return False
    
    def stop(self) -> bool:
        return True
    
    def get_navigation_status(self) -> Dict[str, Any]:
        return {"status": "not_available"}
    
    def is_path_clear(self, target_pose: Pose) -> bool:
        return False


class NullManipulationInterface(ManipulationInterface):
    """Null implementation for systems without manipulation"""
    
    def get_joint_states(self) -> Optional[JointState]:
        return None
    
    def get_end_effector_pose(self) -> Optional[Pose]:
        return None
    
    def move_to_pose(self, pose: Pose) -> bool:
        return False
    
    def move_to_joint_positions(self, positions: List[float]) -> bool:
        return False
    
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        return False
    
    def open_gripper(self) -> bool:
        return False
    
    def close_gripper(self) -> bool:
        return False
    
    def get_manipulation_status(self) -> Dict[str, Any]:
        return {"status": "not_available"}
    
    def is_pose_reachable(self, pose: Pose) -> bool:
        return False


class NullVisionInterface(VisionInterface):
    """Null implementation for systems without vision"""
    
    def get_current_image(self) -> Optional[Image.Image]:
        return None
    
    def analyze_scene(self, prompt: str = "Describe what you see") -> Optional[str]:
        return "Vision system not available"
    
    def detect_objects(self) -> List[Dict[str, Any]]:
        return []
    
    def find_object(self, object_name: str) -> Optional[Dict[str, Any]]:
        return None
    
    def save_image(self, filename: str) -> bool:
        return False
    
    def get_vision_status(self) -> Dict[str, Any]:
        return {"status": "not_available"}


class NullSpeechInterface(SpeechInterface):
    """Null implementation for systems without speech"""
    
    def speak(self, text: str) -> bool:
        print(f"🤖: {text}")  # Fallback to text output
        return True
    
    def listen(self) -> Optional[str]:
        return None
    
    def set_voice(self, voice_id: str) -> bool:
        return False
    
    def get_speech_status(self) -> Dict[str, Any]:
        return {"status": "not_available"}
    
    def is_listening(self) -> bool:
        return False


# Factory functions for creating null implementations

def create_null_platform() -> PlatformInterface:
    """Create a null platform for systems without hardware"""
    
    class NullPlatform(PlatformInterface):
        def __init__(self):
            super().__init__()
            self.navigation = NullNavigationInterface()
            self.manipulation = NullManipulationInterface()
            self.vision = NullVisionInterface()
            self.speech = NullSpeechInterface()
            self.system = None  # Will be implemented per platform
        
        def initialize(self, config: Dict[str, Any]) -> bool:
            return True
        
        def shutdown(self) -> bool:
            return True
        
        def get_available_capabilities(self) -> List[str]:
            return ["text_interface"]
        
        def is_capability_available(self, capability: str) -> bool:
            return capability in ["text_interface"]
    
    return NullPlatform()