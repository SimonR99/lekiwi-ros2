#!/usr/bin/env python3
"""
Dino Brain - Core AI System

ROS-independent AI orchestration system that can work with any platform
through abstract interfaces.
"""

import asyncio
import time
from typing import Dict, Any, Optional, List
import logging
from dataclasses import dataclass

# Import our interfaces
from dino_ai.core.interfaces import (
    PlatformInterface, SystemStatus, SystemHealth,
    create_null_platform
)

# Import MCP client
from dino_ai.core.mcp_client import create_mcp_client, MCPClient, ToolNotFoundError

# Import AI components (make these optional)
try:
    from langchain.schema import HumanMessage, AIMessage
    from langgraph.graph import StateGraph, END
    from langgraph.checkpoint.memory import MemorySaver
    LANGRAPH_AVAILABLE = True
except ImportError:
    LANGRAPH_AVAILABLE = False
    print("LangGraph not available, using simplified processing")

logger = logging.getLogger(__name__)


@dataclass
class ConversationState:
    """State for conversation management"""
    messages: List[Dict[str, str]]
    current_task: Optional[str] = None
    platform_status: str = "unknown"
    last_vision_analysis: Optional[str] = None
    conversation_active: bool = False


class DinoBrain:
    """
    Core AI system that orchestrates robot behavior through abstract interfaces.
    Platform-independent and can work with or without physical embodiment.
    """
    
    def __init__(self, platform: Optional[PlatformInterface] = None, mcp_config_path: Optional[str] = None, use_react: bool = True):
        self.platform = platform or create_null_platform()
        self.conversation_state = ConversationState(messages=[])
        self.running = False
        
        # AI components
        self.llm = None
        self.graph = None
        
        # MCP client for dynamic tools
        self.mcp_client = create_mcp_client(mcp_config_path)
        
        # ReAct brain for complex reasoning
        self.react_brain = None
        self.use_react = use_react
        
        if use_react:
            try:
                from dino_ai.core.react_brain import create_react_brain
                self.react_brain = create_react_brain(platform)
                if self.react_brain.is_available():
                    logger.info("ReAct capabilities enabled")
                else:
                    logger.warning("ReAct capabilities not available, using fallback")
            except ImportError as e:
                logger.warning(f"ReAct brain not available: {e}")
        
        # MCP will be initialized later when needed
        logger.info(f"Dino Brain initialized with platform: {type(self.platform).__name__}")
    
    async def _initialize_ai_system(self):
        """Initialize AI components if available"""
        # Initialize MCP client
        await self.mcp_client.initialize()
        
        if LANGRAPH_AVAILABLE:
            self._setup_langgraph()
        else:
            logger.warning("LangGraph not available, using rule-based processing")
    
    def _setup_langgraph(self):
        """Setup LangGraph workflow"""
        try:
            # This would normally set up the full LangGraph system
            # For now, we'll use a simplified approach
            logger.info("LangGraph system initialized")
        except Exception as e:
            logger.error(f"Failed to setup LangGraph: {e}")
            LANGRAPH_AVAILABLE = False
    
    async def process_message(self, message: str) -> str:
        """
        Process a user message and return response.
        This is the main entry point for all interactions.
        """
        logger.info(f"Processing message: {message}")
        
        # Initialize MCP if not done yet
        if not hasattr(self, '_mcp_initialized'):
            await self._initialize_ai_system()
            self._mcp_initialized = True
        
        # Add to conversation history
        self.conversation_state.messages.append({
            "role": "user",
            "content": message,
            "timestamp": time.time()
        })
        
        # Analyze message intent
        intent = self._analyze_intent(message)
        
        # Process based on intent
        response = await self._process_intent(intent, message)
        
        # Try to use MCP tools for enhanced response (e.g., TTS)
        await self._enhance_response_with_mcp(response, intent)
        
        # Add response to history
        self.conversation_state.messages.append({
            "role": "assistant", 
            "content": response,
            "timestamp": time.time()
        })
        
        return response
    
    def _analyze_intent(self, message: str) -> str:
        """
        Analyze user message to determine intent.
        Enhanced with ReAct detection for complex tasks.
        """
        message_lower = message.lower()
        
        # Check for ReAct-worthy complex/iterative tasks first
        if self.use_react and self.react_brain and self.react_brain.is_available():
            complex_patterns = [
                "until", "while", "keep", "continue", "repeatedly", 
                "don't see", "stop when", "first", "then", "after",
                "and then", "followed by", "next"
            ]
            
            if any(pattern in message_lower for pattern in complex_patterns):
                return "react"
        
        # Navigation intents
        if any(word in message_lower for word in ["move", "go", "navigate", "drive", "forward", "backward", "left", "right"]):
            return "navigation"
        
        # Manipulation intents
        if any(word in message_lower for word in ["pick", "grab", "move arm", "gripper", "open", "close", "reach"]):
            return "manipulation"
        
        # Vision intents
        if any(word in message_lower for word in ["see", "look", "vision", "camera", "view", "observe", "what do you see"]):
            return "vision"
        
        # System intents
        if any(word in message_lower for word in ["status", "health", "check", "diagnostic", "info"]):
            return "system"
        
        # Speech intents
        if any(word in message_lower for word in ["speak", "say", "voice", "quiet", "volume"]):
            return "speech"
        
        # Default to conversation
        return "conversation"
    
    async def _process_intent(self, intent: str, message: str) -> str:
        """Process message based on detected intent"""
        
        if intent == "react":
            return await self._handle_react(message)
        elif intent == "navigation":
            return await self._handle_navigation(message)
        elif intent == "manipulation":
            return await self._handle_manipulation(message)
        elif intent == "vision":
            return await self._handle_vision(message)
        elif intent == "system":
            return await self._handle_system(message)
        elif intent == "speech":
            return await self._handle_speech(message)
        else:
            return await self._handle_conversation(message)
    
    async def _handle_navigation(self, message: str) -> str:
        """Handle navigation-related requests"""
        if not self.platform.has_navigation():
            return "I don't have navigation capabilities on this platform."
        
        nav = self.platform.navigation
        message_lower = message.lower()
        
        try:
            if "forward" in message_lower:
                success = nav.move_relative(1.0, 0.0, 0.0)
                return "Moving forward 1 meter" if success else "Failed to move forward"
            
            elif "backward" in message_lower or "back" in message_lower:
                success = nav.move_relative(-1.0, 0.0, 0.0)
                return "Moving backward 1 meter" if success else "Failed to move backward"
            
            elif "left" in message_lower:
                success = nav.move_relative(0.0, 1.0, 0.0)
                return "Moving left 1 meter" if success else "Failed to move left"
            
            elif "right" in message_lower:
                success = nav.move_relative(0.0, -1.0, 0.0)
                return "Moving right 1 meter" if success else "Failed to move right"
            
            elif "stop" in message_lower:
                success = nav.stop()
                return "Stopped all motion" if success else "Failed to stop"
            
            elif "where" in message_lower or "position" in message_lower:
                pose = nav.get_current_pose()
                if pose:
                    return f"I'm at position ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})"
                else:
                    return "I can't determine my current position"
            
            else:
                return "I understand you want me to navigate, but I need more specific instructions like 'move forward', 'turn left', etc."
        
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            return f"Navigation error: {str(e)}"
    
    async def _handle_manipulation(self, message: str) -> str:
        """Handle manipulation-related requests"""
        if not self.platform.has_manipulation():
            return "I don't have manipulation capabilities on this platform."
        
        manip = self.platform.manipulation
        message_lower = message.lower()
        
        try:
            if "open" in message_lower and "gripper" in message_lower:
                success = manip.open_gripper()
                return "Opened gripper" if success else "Failed to open gripper"
            
            elif "close" in message_lower and "gripper" in message_lower:
                success = manip.close_gripper()
                return "Closed gripper" if success else "Failed to close gripper"
            
            elif "arm position" in message_lower or "where is" in message_lower:
                pose = manip.get_end_effector_pose()
                if pose:
                    return f"Arm is at position ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})"
                else:
                    return "I can't determine the arm position"
            
            elif "joints" in message_lower:
                joints = manip.get_joint_states()
                if joints:
                    joint_info = ", ".join([f"{name}: {pos:.2f}" for name, pos in zip(joints.names, joints.positions)])
                    return f"Joint positions: {joint_info}"
                else:
                    return "I can't get joint information"
            
            else:
                return "I understand you want me to manipulate something, but I need more specific instructions."
        
        except Exception as e:
            logger.error(f"Manipulation error: {e}")
            return f"Manipulation error: {str(e)}"
    
    async def _handle_vision(self, message: str) -> str:
        """Handle vision-related requests"""
        if not self.platform.has_vision():
            return "I don't have vision capabilities on this platform."
        
        vision = self.platform.vision
        
        try:
            if "what do you see" in message.lower() or "describe" in message.lower():
                analysis = vision.analyze_scene("Describe what you see in detail")
                if analysis:
                    self.conversation_state.last_vision_analysis = analysis
                    return f"I can see: {analysis}"
                else:
                    return "I'm having trouble analyzing the current view."
            
            elif "objects" in message.lower():
                objects = vision.detect_objects()
                if objects:
                    object_names = [obj.get("name", "unknown") for obj in objects]
                    return f"I can see these objects: {', '.join(object_names)}"
                else:
                    return "I don't see any recognizable objects right now."
            
            elif "save" in message.lower() and "image" in message.lower():
                timestamp = int(time.time())
                filename = f"dino_view_{timestamp}.jpg"
                success = vision.save_image(filename)
                return f"Saved current view as {filename}" if success else "Failed to save image"
            
            else:
                # General vision query
                analysis = vision.analyze_scene(message)
                if analysis:
                    return analysis
                else:
                    return "I couldn't analyze the scene for that query."
        
        except Exception as e:
            logger.error(f"Vision error: {e}")
            return f"Vision error: {str(e)}"
    
    async def _handle_system(self, message: str) -> str:
        """Handle system status and control requests"""
        try:
            if "status" in message.lower() or "health" in message.lower():
                capabilities = self.platform.get_available_capabilities()
                status_info = [
                    f"Platform: {type(self.platform).__name__}",
                    f"Available capabilities: {', '.join(capabilities)}",
                    f"Navigation: {'✓' if self.platform.has_navigation() else '✗'}",
                    f"Manipulation: {'✓' if self.platform.has_manipulation() else '✗'}",
                    f"Vision: {'✓' if self.platform.has_vision() else '✗'}",
                    f"Speech: {'✓' if self.platform.has_speech() else '✗'}",
                    f"Messages processed: {len(self.conversation_state.messages) // 2}"
                ]
                return "\n".join(status_info)
            
            elif "capabilities" in message.lower():
                capabilities = self.platform.get_available_capabilities()
                return f"My available capabilities are: {', '.join(capabilities)}"
            
            elif "restart" in message.lower() or "reset" in message.lower():
                return "System restart functionality would be implemented here."
            
            else:
                return "I can provide system status, capabilities, or help with system control."
        
        except Exception as e:
            logger.error(f"System error: {e}")
            return f"System error: {str(e)}"
    
    async def _handle_speech(self, message: str) -> str:
        """Handle speech-related requests"""
        if not self.platform.has_speech():
            return "I don't have speech capabilities on this platform, but I can respond with text."
        
        speech = self.platform.speech
        message_lower = message.lower()
        
        try:
            if "quiet" in message_lower or "silent" in message_lower:
                return "I'll respond with text only for now."
            
            elif "speak" in message_lower or "say" in message_lower:
                # Extract text to speak
                if "say" in message_lower:
                    text_to_speak = message.split("say", 1)[-1].strip()
                    if text_to_speak:
                        success = speech.speak(text_to_speak)
                        return f"Speaking: {text_to_speak}" if success else "Failed to speak"
                
                return "What would you like me to say?"
            
            else:
                return "I can speak text or adjust my voice settings."
        
        except Exception as e:
            logger.error(f"Speech error: {e}")
            return f"Speech error: {str(e)}"
    
    async def _handle_conversation(self, message: str) -> str:
        """Handle general conversation"""
        # Simple conversational responses
        message_lower = message.lower()
        
        if any(greeting in message_lower for greeting in ["hello", "hi", "hey", "good morning", "good afternoon"]):
            capabilities = self.platform.get_available_capabilities()
            return f"Hello! I'm Dino AI. I'm running on a {type(self.platform).__name__} platform with these capabilities: {', '.join(capabilities)}. How can I help you?"
        
        elif any(word in message_lower for word in ["thanks", "thank you"]):
            return "You're welcome! Is there anything else I can help you with?"
        
        elif any(word in message_lower for word in ["bye", "goodbye", "exit", "quit"]):
            return "Goodbye! It was nice talking with you."
        
        elif "help" in message_lower:
            help_text = [
                "I can help you with:",
                "• Navigation: 'move forward', 'turn left', 'where am I?'",
                "• Manipulation: 'open gripper', 'arm position'", 
                "• Vision: 'what do you see?', 'describe the scene'",
                "• System: 'status', 'capabilities'",
                "• Conversation: Just talk naturally!",
                "",
                "Available capabilities on this platform:",
                f"  {', '.join(self.platform.get_available_capabilities())}"
            ]
            return "\n".join(help_text)
        
        elif "what can you do" in message_lower:
            capabilities = []
            if self.platform.has_navigation():
                capabilities.append("navigate and move around")
            if self.platform.has_manipulation():
                capabilities.append("control robotic arms and grippers")
            if self.platform.has_vision():
                capabilities.append("see and analyze images")
            if self.platform.has_speech():
                capabilities.append("speak and listen")
            
            if capabilities:
                return f"I can {', '.join(capabilities)}. I'm also great at conversation!"
            else:
                return "I'm a conversational AI that can chat with you. On platforms with more capabilities, I can also control robots, see through cameras, and more!"
        
        else:
            # Default response for unrecognized input
            return "I understand you're talking to me, but I'm not sure how to respond to that. Try asking me about my capabilities or giving me specific commands!"
    
    async def _handle_react(self, message: str) -> str:
        """Handle complex tasks using ReAct reasoning"""
        if not self.react_brain or not self.react_brain.is_available():
            return "Complex reasoning not available. Please try a simpler command."
        
        try:
            logger.info(f"Processing ReAct task: {message}")
            result = await self.react_brain.process_message(message)
            return result
        except Exception as e:
            logger.error(f"ReAct processing failed: {e}")
            return f"I encountered an error while processing that complex task: {e}"
    
    async def start_interactive_mode(self, use_speech: bool = False):
        """Start interactive mode for direct user interaction"""
        self.conversation_state.conversation_active = True
        
        print("🦕 Dino AI Interactive Mode")
        print("=" * 40)
        print(f"Platform: {type(self.platform).__name__}")
        print(f"Capabilities: {', '.join(self.platform.get_available_capabilities())}")
        print("\nType 'help' for commands, 'exit' to quit")
        print("-" * 40)
        
        try:
            while self.conversation_state.conversation_active:
                try:
                    # Get user input
                    if use_speech and self.platform.has_speech():
                        print("🎤 Listening...")
                        user_input = self.platform.speech.listen()
                        if not user_input:
                            continue
                        print(f"You: {user_input}")
                    else:
                        user_input = input("You: ").strip()
                    
                    if not user_input:
                        continue
                    
                    # Check for exit
                    if user_input.lower() in ['exit', 'quit', 'bye', 'goodbye']:
                        response = "Goodbye! 👋"
                        print(f"🦕: {response}")
                        break
                    
                    # Process message
                    response = await self.process_message(user_input)
                    
                    # Output response
                    print(f"🦕: {response}")
                    
                    # Speak response if enabled
                    if use_speech and self.platform.has_speech():
                        self.platform.speech.speak(response)
                
                except KeyboardInterrupt:
                    print("\n🦕: Goodbye! 👋")
                    break
                except Exception as e:
                    print(f"🦕: Sorry, I encountered an error: {e}")
                    logger.error(f"Interactive mode error: {e}")
        
        finally:
            self.conversation_state.conversation_active = False
    
    def get_conversation_history(self) -> List[Dict[str, str]]:
        """Get conversation history"""
        return self.conversation_state.messages
    
    def clear_conversation_history(self):
        """Clear conversation history"""
        self.conversation_state.messages = []
    
    def get_platform_status(self) -> Dict[str, Any]:
        """Get platform status information"""
        return {
            "platform_type": type(self.platform).__name__,
            "capabilities": self.platform.get_available_capabilities(),
            "has_navigation": self.platform.has_navigation(),
            "has_manipulation": self.platform.has_manipulation(), 
            "has_vision": self.platform.has_vision(),
            "has_speech": self.platform.has_speech(),
            "conversation_active": self.conversation_state.conversation_active,
            "messages_count": len(self.conversation_state.messages),
            "mcp_tools": list(self.mcp_client.get_all_tools().keys()) if hasattr(self, '_mcp_initialized') else []
        }
    
    async def _enhance_response_with_mcp(self, response: str, intent: str):
        """Enhance response using MCP tools (e.g., TTS)"""
        try:
            # Try to speak the response if TTS is available
            if self.mcp_client.has_tool("speak_text"):
                await self.mcp_client.call_tool("speak_text", {"text": response})
                logger.debug("Response spoken via MCP TTS")
        except ToolNotFoundError:
            # TTS not available, that's fine
            pass
        except Exception as e:
            logger.warning(f"Failed to enhance response with MCP: {e}")
    
    async def call_mcp_tool(self, tool_name: str, args: dict) -> Any:
        """Public interface to call MCP tools"""
        try:
            return await self.mcp_client.call_tool(tool_name, args)
        except ToolNotFoundError as e:
            logger.warning(f"MCP tool not found: {e}")
            return None
        except Exception as e:
            logger.error(f"Error calling MCP tool {tool_name}: {e}")
            return None
    
    def get_available_mcp_tools(self) -> Dict[str, Any]:
        """Get information about available MCP tools"""
        if not hasattr(self, '_mcp_initialized'):
            return {}
        
        tools_info = {}
        for tool_name in self.mcp_client.get_all_tools().keys():
            tools_info[tool_name] = self.mcp_client.get_tool_info(tool_name)
        return tools_info
    
    def get_mcp_tools_by_category(self, category: str) -> List[str]:
        """Get MCP tools by category (speech, vision, math, etc.)"""
        if not hasattr(self, '_mcp_initialized'):
            return []
        
        tools = self.mcp_client.get_tools_by_category(category)
        return [tool.name for tool in tools]