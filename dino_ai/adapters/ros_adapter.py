#!/usr/bin/env python3
"""
ROS Adapter for Dino AI

Implements the abstract interfaces using ROS2 for the Lekiwi robot platform.
This allows Dino AI to control ROS-based robots without being tightly coupled to ROS.
"""

import time
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

# ROS imports (make them optional)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import JointState as ROSJointState, Image as ROSImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    import cv2
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS2 not available - ROS adapter will not work")

logger = logging.getLogger(__name__)


class ROSNavigationAdapter(NavigationInterface):
    """ROS implementation of navigation interface"""
    
    def __init__(self, node: 'ROSAdapterNode'):
        self.node = node
        self.current_pose = None
        self.navigation_status = {"status": "idle"}
        
        # Publishers
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
    
    def _odom_callback(self, msg):
        """Update current pose from odometry"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Convert quaternion to euler (simplified)
        yaw = 2 * np.arctan2(orient.z, orient.w)
        
        self.current_pose = Pose(
            x=pos.x, y=pos.y, z=pos.z,
            roll=0.0, pitch=0.0, yaw=yaw
        )
    
    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose
    
    def navigate_to_pose(self, pose: Pose) -> bool:
        """Navigate to target pose using Nav2"""
        try:
            # This would implement Nav2 goal sending
            # For now, just log the request
            logger.info(f"Navigation request to ({pose.x}, {pose.y}, {pose.yaw})")
            self.navigation_status = {"status": "navigating", "target": pose}
            return True
        except Exception as e:
            logger.error(f"Navigation failed: {e}")
            return False
    
    def move_relative(self, dx: float, dy: float, dyaw: float) -> bool:
        """Move relative to current position"""
        try:
            # Simple relative movement using velocity commands
            cmd = Twist()
            
            # Simple proportional control
            cmd.linear.x = dx * 0.5  # Scale down for safety
            cmd.linear.y = dy * 0.5
            cmd.angular.z = dyaw * 0.5
            
            # Send command for 2 seconds
            for _ in range(20):  # 20 * 0.1s = 2s
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
            
            # Stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            return True
        except Exception as e:
            logger.error(f"Relative move failed: {e}")
            return False
    
    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """Set velocity commands"""
        try:
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.linear.y = linear_y
            cmd.angular.z = angular_z
            
            self.cmd_vel_pub.publish(cmd)
            return True
        except Exception as e:
            logger.error(f"Velocity command failed: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop all motion"""
        return self.set_velocity(0.0, 0.0, 0.0)
    
    def get_navigation_status(self) -> Dict[str, Any]:
        return self.navigation_status
    
    def is_path_clear(self, target_pose: Pose) -> bool:
        # This would implement collision checking
        return True


class ROSManipulationAdapter(ManipulationInterface):
    """ROS implementation of manipulation interface"""
    
    def __init__(self, node: 'ROSAdapterNode'):
        self.node = node
        self.current_joints = None
        self.current_pose = None
        self.manipulation_status = {"status": "idle"}
        
        # Subscribers
        self.joint_sub = node.create_subscription(
            ROSJointState, '/joint_states', self._joint_callback, 10
        )
    
    def _joint_callback(self, msg):
        """Update joint states"""
        self.current_joints = JointState(
            names=list(msg.name),
            positions=list(msg.position),
            velocities=list(msg.velocity) if msg.velocity else None,
            efforts=list(msg.effort) if msg.effort else None
        )
    
    def get_joint_states(self) -> Optional[JointState]:
        return self.current_joints
    
    def get_end_effector_pose(self) -> Optional[Pose]:
        return self.current_pose
    
    def move_to_pose(self, pose: Pose) -> bool:
        """Move end effector to target pose using MoveIt"""
        try:
            logger.info(f"Moving arm to pose ({pose.x}, {pose.y}, {pose.z})")
            self.manipulation_status = {"status": "moving", "target": pose}
            return True
        except Exception as e:
            logger.error(f"Arm movement failed: {e}")
            return False
    
    def move_to_joint_positions(self, positions: List[float]) -> bool:
        """Move to joint positions"""
        try:
            logger.info(f"Moving to joint positions: {positions}")
            return True
        except Exception as e:
            logger.error(f"Joint movement failed: {e}")
            return False
    
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """Move end effector relative to current position"""
        if self.current_pose:
            target = Pose(
                x=self.current_pose.x + dx,
                y=self.current_pose.y + dy,
                z=self.current_pose.z + dz,
                roll=self.current_pose.roll,
                pitch=self.current_pose.pitch,
                yaw=self.current_pose.yaw
            )
            return self.move_to_pose(target)
        return False
    
    def open_gripper(self) -> bool:
        """Open gripper"""
        try:
            logger.info("Opening gripper")
            return True
        except Exception as e:
            logger.error(f"Gripper open failed: {e}")
            return False
    
    def close_gripper(self) -> bool:
        """Close gripper"""
        try:
            logger.info("Closing gripper")
            return True
        except Exception as e:
            logger.error(f"Gripper close failed: {e}")
            return False
    
    def get_manipulation_status(self) -> Dict[str, Any]:
        return self.manipulation_status
    
    def is_pose_reachable(self, pose: Pose) -> bool:
        # This would implement reachability checking
        return True


class ROSVisionAdapter(VisionInterface):
    """ROS implementation of vision interface"""
    
    def __init__(self, node: 'ROSAdapterNode'):
        self.node = node
        self.current_image = None
        self.cv_bridge = CvBridge()
        self.vision_status = {"status": "idle"}
        
        # Subscribers
        self.image_sub = node.create_subscription(
            ROSImage, '/bottom_camera/image_raw', self._image_callback, 10
        )
        
        # Vision analysis client (Moondream integration)
        self.moondream_available = self._check_moondream_availability()
    
    def _image_callback(self, msg):
        """Update current image"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.current_image = PILImage.fromarray(rgb_image)
        except Exception as e:
            logger.error(f"Image conversion failed: {e}")
    
    def get_current_image(self, timeout: float = 5.0) -> Optional[PILImage.Image]:
        """Get current camera image, with optional timeout to wait for first image"""
        if self.current_image is not None:
            return self.current_image
        
        # Wait for first image with timeout
        logger.info(f"Waiting for camera image (timeout: {timeout}s)...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.current_image is not None:
                logger.info("Camera image received!")
                return self.current_image
            time.sleep(0.1)  # Check every 100ms
        
        logger.warning(f"No camera image received within {timeout}s timeout")
        return None
    
    def analyze_scene(self, prompt: str = "Describe what you see") -> Optional[str]:
        """Analyze current scene with AI vision"""
        # Wait for image with timeout
        image = self.get_current_image(timeout=5.0)
        if not image:
            return "No camera image available"
        
        try:
            # Send image to Moondream for analysis
            if self.moondream_available:
                return self._analyze_with_moondream(image, prompt)
            else:
                return f"AI vision analysis: [Moondream not available - Image captured successfully with prompt: '{prompt}']"
        except Exception as e:
            logger.error(f"Vision analysis failed: {e}")
            return None
    
    def detect_objects(self) -> List[Dict[str, Any]]:
        """Detect objects in current view"""
        # Wait for image with timeout
        image = self.get_current_image(timeout=5.0)
        if not image:
            return []
        
        # Placeholder object detection
        return [
            {"name": "table", "confidence": 0.85, "bbox": [100, 100, 200, 200]},
            {"name": "cup", "confidence": 0.72, "bbox": [150, 50, 180, 90]}
        ]
    
    def find_object(self, object_name: str) -> Optional[Dict[str, Any]]:
        """Find specific object in view"""
        objects = self.detect_objects()
        for obj in objects:
            if obj["name"].lower() == object_name.lower():
                return obj
        return None
    
    def save_image(self, filename: str) -> bool:
        """Save current image to file"""
        if self.current_image:
            try:
                self.current_image.save(filename)
                return True
            except Exception as e:
                logger.error(f"Image save failed: {e}")
        return False
    
    def get_vision_status(self) -> Dict[str, Any]:
        return {
            **self.vision_status,
            "has_image": self.current_image is not None,
            "moondream_available": self.moondream_available
        }
    
    def _check_moondream_availability(self) -> bool:
        """Check if Moondream server is available"""
        try:
            import requests
            # Try to connect to Moondream server (adjust URL as needed)
            response = requests.get("http://localhost:5000/health", timeout=2)
            if response.status_code == 200:
                logger.info("Moondream server is available")
                return True
        except Exception:
            pass
        
        # Try alternative: local Moondream model
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
            import torch
            
            # Just check if we can import the classes, don't actually load the model yet
            # The actual model loading will happen in _try_local_moondream with proper error handling
            logger.info("Transformers library available for Moondream inference")
            return True
        except ImportError as e:
            logger.debug(f"Transformers not available: {e}")
            pass
        except Exception as e:
            logger.debug(f"Other error checking transformers: {e}")
            pass
        
        logger.info("Moondream not available (no server or transformers)")
        return False
    
    def _analyze_with_moondream(self, image: PILImage.Image, prompt: str) -> str:
        """Analyze image with Moondream"""
        try:
            # Method 1: Try Moondream server first
            server_result = self._try_moondream_server(image, prompt)
            if server_result:
                return server_result
        except Exception as e:
            logger.warning(f"Moondream server failed: {e}")
        
        try:
            # Method 2: Try local Moondream model
            return self._try_local_moondream(image, prompt)
        except RuntimeError as e:
            logger.warning(f"Local Moondream failed: {e}")
            # Don't log full stack trace for known issues
        except Exception as e:
            logger.warning(f"Local Moondream failed unexpectedly: {e}")
        
        # Fallback: describe image properties
        logger.info("Using fallback image description (Moondream not available)")
        return self._describe_image_properties(image, prompt)
    
    def _try_moondream_server(self, image: PILImage.Image, prompt: str) -> Optional[str]:
        """Try to use Moondream server"""
        try:
            import requests
            import base64
            from io import BytesIO
            
            # Convert image to base64
            buffer = BytesIO()
            image.save(buffer, format="JPEG")
            image_b64 = base64.b64encode(buffer.getvalue()).decode()
            
            # Send to Moondream server
            response = requests.post(
                "http://localhost:5000/analyze",
                json={"image": image_b64, "prompt": prompt},
                timeout=10
            )
            
            logger.debug(f"Moondream server response status: {response.status_code}")
            
            if response.status_code == 200:
                result = response.json()
                return result.get("description", "No description returned")
            else:
                logger.warning(f"Moondream server returned status {response.status_code}: {response.text}")
            
        except Exception as e:
            logger.warning(f"Moondream server request failed: {e}")
        
        return None
    
    def _try_local_moondream(self, image: PILImage.Image, prompt: str) -> str:
        """Try to use local Moondream model"""
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
            import torch
            
            # Load Moondream model (cache after first load)
            if not hasattr(self, '_moondream_model'):
                logger.info("Loading Moondream model...")
                
                # Try loading with better error handling
                try:
                    self._moondream_model = AutoModelForCausalLM.from_pretrained(
                        "vikhyatk/moondream2", 
                        trust_remote_code=True,
                        torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                        device_map="auto" if torch.cuda.is_available() else None
                    )
                    self._moondream_tokenizer = AutoTokenizer.from_pretrained("vikhyatk/moondream2")
                    
                    if torch.cuda.is_available() and not hasattr(self._moondream_model, 'device'):
                        self._moondream_model = self._moondream_model.cuda()
                    
                    logger.info("Moondream model loaded successfully")
                    
                except Exception as model_load_error:
                    logger.error(f"Failed to load Moondream model: {model_load_error}")
                    # Mark as failed to avoid repeated attempts
                    self._moondream_model = None
                    raise model_load_error
            
            # Check if model loading failed previously
            if self._moondream_model is None:
                raise RuntimeError("Moondream model is not available (previous load failure)")
            
            # Generate description
            encoded_image = self._moondream_model.encode_image(image)
            answer = self._moondream_model.answer_question(encoded_image, prompt, self._moondream_tokenizer)
            
            return answer
            
        except ImportError as e:
            logger.error(f"Missing transformers dependencies: {e}")
            raise RuntimeError(f"Transformers library issue: {e}")
        except Exception as e:
            logger.error(f"Local Moondream inference failed: {e}")
            raise RuntimeError(f"Moondream model error: {e}")
    
    def _describe_image_properties(self, image: PILImage.Image, prompt: str) -> str:
        """Fallback: describe basic image properties"""
        width, height = image.size
        mode = image.mode
        
        # Try to get dominant colors
        try:
            import numpy as np
            img_array = np.array(image)
            mean_color = np.mean(img_array, axis=(0, 1))
            
            if len(mean_color) == 3:
                r, g, b = mean_color.astype(int)
                dominant_color = f"RGB({r}, {g}, {b})"
            else:
                dominant_color = "grayscale"
        except:
            dominant_color = "unknown"
        
        description = f"I can see an image that is {width}x{height} pixels in {mode} format. "
        description += f"The dominant color appears to be {dominant_color}. "
        description += f"This is a real camera feed from the robot's vision system."
        
        return description


class ROSSpeechAdapter(SpeechInterface):
    """ROS implementation of speech interface with advanced VAD/Whisper/Piper"""
    
    def __init__(self, node: 'ROSAdapterNode'):
        self.node = node
        self.speech_status = {"status": "idle", "is_speaking": False, "is_listening": False}
        self.speech_engine = None
        
        # Publishers for ROS integration
        self.speak_pub = node.create_publisher(String, '/tts_text', 10)
        
        # Try to initialize advanced speech engine
        self._initialize_speech_engine()
        
        logger.info("ROS Speech Adapter initialized")
    
    def _initialize_speech_engine(self):
        """Initialize advanced speech engine with VAD/Whisper/Piper"""
        try:
            from dino_ai.core.speech_engine import create_speech_engine
            
            # Create speech engine with callback to process messages
            self.speech_engine = create_speech_engine(
                message_callback=self._handle_speech_message
            )
            
            if self.speech_engine.is_available():
                logger.info("Advanced speech engine (VAD/Whisper/Piper) available")
            else:
                logger.warning("Advanced speech engine not available, using fallback")
                
        except ImportError as e:
            logger.warning(f"Advanced speech engine not available: {e}")
            self.speech_engine = None
    
    def _handle_speech_message(self, text: str):
        """Handle transcribed speech from VAD/Whisper"""
        logger.info(f"Speech recognized: {text}")
        
        # Publish to ROS topic for other nodes
        msg = String()
        msg.data = f"SPEECH_INPUT: {text}"
        self.speak_pub.publish(msg)
        
        # TODO: Send back to Dino Brain for processing
        # This would need a callback mechanism to parent system
    
    def speak(self, text: str) -> bool:
        """Speak text using Piper TTS or ROS fallback"""
        if not text:
            return False
        
        # Try advanced TTS first
        if self.speech_engine and self.speech_engine.is_available():
            success = self.speech_engine.speak(text)
            if success:
                self.speech_status["is_speaking"] = True
                return True
        
        # Fallback to ROS topic + console
        try:
            msg = String()
            msg.data = text
            self.speak_pub.publish(msg)
            
            # Also print to console as fallback
            print(f"🤖 Speaking: {text}")
            return True
        except Exception as e:
            logger.error(f"Speech failed: {e}")
            return False
    
    def listen(self) -> Optional[str]:
        """Listen for speech input using VAD/Whisper or fallback"""
        if self.speech_engine and self.speech_engine.is_available():
            # Start listening if not already
            if not self.speech_engine.is_listening():
                self.speech_engine.start_listening()
                self.speech_status["is_listening"] = True
            
            # For interactive mode, we'll handle this differently
            # Return None here as speech will be processed via callback
            return None
        else:
            # Fallback to text input
            try:
                return input("🎤 Speak (or type): ").strip()
            except:
                return None
    
    def start_continuous_listening(self) -> bool:
        """Start continuous speech recognition"""
        if self.speech_engine and self.speech_engine.is_available():
            success = self.speech_engine.start_listening()
            if success:
                self.speech_status["is_listening"] = True
                logger.info("🎤 Started continuous listening")
            return success
        return False
    
    def stop_listening(self):
        """Stop speech recognition"""
        if self.speech_engine:
            self.speech_engine.stop_listening()
        self.speech_status["is_listening"] = False
        logger.info("🎤 Stopped listening")
    
    def set_voice(self, voice_id: str) -> bool:
        """Set TTS voice"""
        logger.info(f"Setting voice to: {voice_id}")
        return True
    
    def get_speech_status(self) -> Dict[str, Any]:
        """Get speech system status"""
        status = {
            **self.speech_status,
            "tts_available": True,
            "stt_available": self.speech_engine is not None,
            "vad_available": self.speech_engine and self.speech_engine.is_available(),
            "advanced_engine": self.speech_engine is not None
        }
        
        if self.speech_engine:
            status["is_listening"] = self.speech_engine.is_listening()
            status["is_speaking"] = self.speech_engine.is_speaking()
        
        return status
    
    def is_listening(self) -> bool:
        """Check if actively listening"""
        if self.speech_engine:
            return self.speech_engine.is_listening()
        return self.speech_status["is_listening"]
    
    def is_speaking(self) -> bool:
        """Check if currently speaking"""
        if self.speech_engine:
            return self.speech_engine.is_speaking()
        return self.speech_status["is_speaking"]
    
    def shutdown(self):
        """Shutdown speech adapter"""
        if self.speech_engine:
            self.speech_engine.shutdown()
        logger.info("ROS Speech Adapter shutdown")


class ROSSystemAdapter(SystemInterface):
    """ROS implementation of system interface"""
    
    def __init__(self, node: 'ROSAdapterNode'):
        self.node = node
        self.system_health = SystemHealth(
            overall_status=SystemStatus.READY,
            components={},
            errors=[],
            warnings=[],
            metrics={}
        )
    
    def get_system_health(self) -> SystemHealth:
        return self.system_health
    
    def emergency_stop(self) -> bool:
        """Emergency stop all systems"""
        try:
            # Stop navigation
            cmd = Twist()
            self.node.navigation.cmd_vel_pub.publish(cmd)
            logger.warning("EMERGENCY STOP activated")
            return True
        except Exception as e:
            logger.error(f"Emergency stop failed: {e}")
            return False
    
    def reset_system(self) -> bool:
        """Reset system to initial state"""
        logger.info("System reset requested")
        return True
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get system information"""
        return {
            "platform": "ROS2 Lekiwi Robot",
            "ros_version": "Humble",
            "node_name": self.node.get_name(),
            "uptime": time.time() - self.node.start_time
        }
    
    def set_system_mode(self, mode: str) -> bool:
        """Set system operation mode"""
        logger.info(f"Setting system mode to: {mode}")
        return True


class ROSAdapterNode(Node):
    """ROS node that handles communication"""
    
    def __init__(self):
        super().__init__('dino_ai_ros_adapter')
        self.start_time = time.time()
        
        # Initialize interface adapters
        self.navigation = ROSNavigationAdapter(self)
        self.manipulation = ROSManipulationAdapter(self)
        self.vision = ROSVisionAdapter(self)
        self.speech = ROSSpeechAdapter(self)
        self.system = ROSSystemAdapter(self)
        
        self.get_logger().info("ROS Adapter Node initialized")


class ROSPlatformAdapter(PlatformInterface):
    """Main ROS platform adapter"""
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.spin_thread = None
        self._initialized = False
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize the ROS platform"""
        if not ROS_AVAILABLE:
            logger.error("ROS2 not available")
            return False
        
        try:
            # Initialize ROS
            if not rclpy.ok():
                rclpy.init()
            
            # Create node
            self.node = ROSAdapterNode()
            
            # Set up interfaces
            self.navigation = self.node.navigation
            self.manipulation = self.node.manipulation
            self.vision = self.node.vision
            self.speech = self.node.speech
            self.system = self.node.system
            
            # Start executor in background thread
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            self.spin_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.spin_thread.start()
            
            self._initialized = True
            logger.info("ROS Platform Adapter initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"ROS platform initialization failed: {e}")
            return False
    
    def shutdown(self) -> bool:
        """Shutdown the ROS platform"""
        try:
            if self.executor:
                self.executor.shutdown()
            
            if self.node:
                self.node.destroy_node()
            
            if rclpy.ok():
                rclpy.shutdown()
            
            self._initialized = False
            logger.info("ROS Platform Adapter shutdown complete")
            return True
            
        except Exception as e:
            logger.error(f"ROS platform shutdown failed: {e}")
            return False
    
    def get_available_capabilities(self) -> List[str]:
        """Get list of available capabilities"""
        return [
            "navigation",
            "manipulation", 
            "vision",
            "speech",
            "ros_integration"
        ]
    
    def is_capability_available(self, capability: str) -> bool:
        """Check if specific capability is available"""
        return capability in self.get_available_capabilities()


def create_ros_platform() -> PlatformInterface:
    """Factory function to create ROS platform adapter"""
    return ROSPlatformAdapter()