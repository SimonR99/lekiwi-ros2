#!/usr/bin/env python3
"""
ReAct Brain for Dino AI

Implements the Reasoning and Acting (ReAct) pattern for complex multi-step tasks
like "move until you don't see headphones anymore" with iterative perception-action loops.
"""

import asyncio
import time
import logging
from typing import Dict, Any, List, Optional, Literal, cast
from dataclasses import dataclass, field

# LangGraph imports
try:
    from langchain_core.messages import AIMessage, HumanMessage, ToolMessage, AnyMessage
    from langchain.chat_models import init_chat_model
    from langchain_core.language_models import BaseChatModel
    from langgraph.graph import StateGraph, add_messages
    from langgraph.prebuilt import ToolNode
    from langgraph.managed import IsLastStep
    from typing_extensions import Annotated
    LANGGRAPH_AVAILABLE = True
except ImportError:
    LANGGRAPH_AVAILABLE = False
    AnyMessage = Any
    AIMessage = Any

# Import our interfaces
from dino_ai.core.interfaces import PlatformInterface, create_null_platform

logger = logging.getLogger(__name__)


@dataclass
class DinoInputState:
    """Input state for Dino AI ReAct agent"""
    
    messages: Annotated[List[AnyMessage], add_messages] = field(default_factory=list)
    """Messages tracking the conversation and tool interactions"""
    
    original_query: str = ""
    """The original user query/command"""
    
    task_type: str = "simple"
    """Type of task: 'simple', 'iterative', 'complex'"""


@dataclass  
class DinoState(DinoInputState):
    """Complete state for Dino AI ReAct agent"""
    
    is_last_step: IsLastStep = field(default=False)
    """Indicates if this is the last step before timeout"""
    
    platform_status: Dict[str, Any] = field(default_factory=dict)
    """Current platform status and capabilities"""
    
    loop_count: int = 0
    """Number of iterations for iterative tasks"""
    
    max_loops: int = 10
    """Maximum number of loops for iterative tasks"""
    
    stop_condition: Optional[str] = None
    """Condition to check for stopping iterative tasks"""
    
    last_vision_result: Optional[str] = None
    """Last vision analysis result"""
    
    last_action_result: Optional[str] = None
    """Result of the last action taken"""


@dataclass
class DinoConfiguration:
    """Configuration for Dino AI ReAct agent"""
    
    model: str = "anthropic/claude-3-5-sonnet-20240620"
    """LLM model to use"""
    
    fallback_mode: bool = True
    """Enable fallback to rule-based processing when LLM unavailable"""
    
    system_prompt: str = """You are Dino, an intelligent robot assistant with ReAct capabilities.

You can reason about complex tasks and take actions iteratively until you achieve the goal.

Available capabilities:
- Vision: Analyze what you see with your camera
- Navigation: Move around (forward, backward, left, right, rotate)  
- Manipulation: Control robotic arm and gripper
- Speech: Speak responses and listen to commands

For iterative tasks like "move until you don't see X", you should:
1. First observe the current state (use vision)
2. Reason about what action to take
3. Execute the action  
4. Observe the result
5. Check if the stop condition is met
6. Repeat if needed

Be concise but thorough in your reasoning. Always use tools when you need to perceive or act.

System time: {system_time}"""
    
    max_iterations: int = 15
    """Maximum number of ReAct iterations"""


class DinoReActBrain:
    """
    ReAct-enabled AI brain for Dino robot with iterative reasoning and acting capabilities.
    Supports complex multi-step tasks with perception-action loops.
    """
    
    def __init__(self, platform: Optional[PlatformInterface] = None, 
                 config: Optional[DinoConfiguration] = None):
        self.platform = platform or create_null_platform()
        self.config = config or DinoConfiguration()
        self.graph = None
        self.tools = []
        
        if not LANGGRAPH_AVAILABLE:
            logger.warning("LangGraph not available, using simplified processing")
            return
        
        # Initialize tools and graph
        self._setup_tools()
        self._setup_graph()
        
        logger.info(f"Dino ReAct Brain initialized with platform: {type(self.platform).__name__}")
    
    def _setup_tools(self):
        """Setup tools for the ReAct agent"""
        if not LANGGRAPH_AVAILABLE:
            return
        
        self.tools = []
        
        # Vision tool
        async def analyze_vision(prompt: str = "Describe what you see") -> str:
            """Analyze the current view using the robot's camera."""
            if self.platform.has_vision():
                result = self.platform.vision.analyze_scene(prompt)
                return result or "No vision data available"
            return "Vision not available on this platform"
        
        # Navigation tools
        async def move_forward(distance: float = 0.5) -> str:
            """Move the robot forward by the specified distance in meters."""
            if self.platform.has_navigation():
                success = self.platform.navigation.move_relative(distance, 0, 0)
                return f"Moved forward {distance}m" if success else "Failed to move forward"
            return "Navigation not available on this platform"
        
        async def move_backward(distance: float = 0.5) -> str:
            """Move the robot backward by the specified distance in meters."""
            if self.platform.has_navigation():
                success = self.platform.navigation.move_relative(-distance, 0, 0)
                return f"Moved backward {distance}m" if success else "Failed to move backward"
            return "Navigation not available on this platform"
        
        async def rotate(angle: float = 90) -> str:
            """Rotate the robot by the specified angle in degrees (positive = clockwise)."""
            if self.platform.has_navigation():
                import math
                angle_rad = math.radians(angle)
                success = self.platform.navigation.move_relative(0, 0, angle_rad)
                return f"Rotated {angle} degrees" if success else "Failed to rotate"
            return "Navigation not available on this platform"
        
        # Manipulation tools
        async def move_arm(position: str) -> str:
            """Move the robotic arm to a named position (up, down, home, extended)."""
            if self.platform.has_manipulation():
                # Map named positions to joint states
                positions = {
                    "up": [0, -1.0, 0, 0, 0],
                    "down": [0, 1.0, 0, 0, 0], 
                    "home": [0, 0, 0, 0, 0],
                    "extended": [0, 0, 1.0, 0, 0]
                }
                joints = positions.get(position, [0, 0, 0, 0, 0])
                success = self.platform.manipulation.move_to_joint_position(joints)
                return f"Moved arm to {position}" if success else f"Failed to move arm to {position}"
            return "Manipulation not available on this platform"
        
        async def control_gripper(action: str) -> str:
            """Control the gripper: 'open' or 'close'."""
            if self.platform.has_manipulation():
                if action == "open":
                    success = self.platform.manipulation.open_gripper()
                    return "Opened gripper" if success else "Failed to open gripper"
                elif action == "close":
                    success = self.platform.manipulation.close_gripper()
                    return "Closed gripper" if success else "Failed to close gripper"
                return "Invalid gripper action. Use 'open' or 'close'"
            return "Manipulation not available on this platform"
        
        # System tools
        async def get_status() -> str:
            """Get the current system status and capabilities."""
            status = self.platform.get_available_capabilities()
            return f"System status: {status}"
        
        async def wait(seconds: float = 1.0) -> str:
            """Wait for the specified number of seconds."""
            await asyncio.sleep(seconds)
            return f"Waited {seconds} seconds"
        
        # Add tools to the list
        self.tools = [
            analyze_vision,
            move_forward,
            move_backward, 
            rotate,
            move_arm,
            control_gripper,
            get_status,
            wait
        ]
    
    def _setup_graph(self):
        """Setup the LangGraph ReAct workflow"""
        if not LANGGRAPH_AVAILABLE:
            return
        
        async def call_model(state: DinoState) -> Dict[str, Any]:
            """Call the LLM to reason and decide on actions"""
            
            # Check for API keys and fallback to local/simple processing if unavailable
            import os
            
            provider = self.config.model.split("/")[0]
            model_name = self.config.model.split("/")[1]
            
            # Check for required API keys
            if provider == "anthropic" and not os.getenv("ANTHROPIC_API_KEY"):
                # Fallback to simple rule-based processing
                return await self._simple_fallback_processing(state)
            elif provider == "openai" and not os.getenv("OPENAI_API_KEY"):
                # Fallback to simple rule-based processing  
                return await self._simple_fallback_processing(state)
            
            try:
                # Load model
                model = init_chat_model(model_name, model_provider=provider).bind_tools(self.tools)
            except Exception as e:
                logger.warning(f"Failed to initialize LLM: {e}")
                return await self._simple_fallback_processing(state)
            
            # Format system prompt
            system_message = self.config.system_prompt.format(
                system_time=time.strftime("%Y-%m-%d %H:%M:%S")
            )
            
            # Add context about iterative tasks
            if state.task_type == "iterative" and state.stop_condition:
                system_message += f"\n\nYou are working on an iterative task. Stop condition: {state.stop_condition}"
                system_message += f"\nLoop count: {state.loop_count}/{state.max_loops}"
                if state.last_vision_result:
                    system_message += f"\nLast vision result: {state.last_vision_result}"
                if state.last_action_result:
                    system_message += f"\nLast action result: {state.last_action_result}"
            
            # Get model response
            response = cast(
                AIMessage,
                await model.ainvoke([
                    {"role": "system", "content": system_message},
                    *state.messages
                ])
            )
            
            # Handle last step
            if state.is_last_step and response.tool_calls:
                return {
                    "messages": [AIMessage(
                        id=response.id,
                        content="I've reached the maximum number of steps. Let me provide you with what I've accomplished so far."
                    )]
                }
            
            return {"messages": [response]}
        
        def route_model_output(state: DinoState) -> Literal["__end__", "tools", "check_condition"]:
            """Route based on model output and task type"""
            last_message = state.messages[-1]
            
            if not isinstance(last_message, AIMessage):
                return "__end__"
            
            # If there are tool calls, execute them
            if last_message.tool_calls:
                return "tools"
            
            # For iterative tasks, check if we should continue
            if state.task_type == "iterative" and state.stop_condition:
                return "check_condition"
            
            return "__end__"
        
        async def check_stop_condition(state: DinoState) -> Dict[str, Any]:
            """Check if the stop condition is met for iterative tasks"""
            
            # Increment loop count
            new_loop_count = state.loop_count + 1
            
            # Check if we've hit max loops
            if new_loop_count >= state.max_loops:
                return {
                    "messages": [AIMessage(content=f"Reached maximum loops ({state.max_loops}). Stopping.")],
                    "loop_count": new_loop_count
                }
            
            # For vision-based conditions, analyze current view
            if state.stop_condition and "see" in state.stop_condition.lower():
                if self.platform.has_vision():
                    vision_result = self.platform.vision.analyze_scene(
                        f"Check if this condition is met: {state.stop_condition}. Answer with YES or NO and explain."
                    )
                    
                    if vision_result and "yes" in vision_result.lower():
                        return {
                            "messages": [AIMessage(content=f"Stop condition met: {vision_result}")],
                            "loop_count": new_loop_count,
                            "last_vision_result": vision_result
                        }
                    else:
                        # Continue the loop
                        return {
                            "messages": [HumanMessage(content="Continue with the task. The stop condition is not yet met.")],
                            "loop_count": new_loop_count,
                            "last_vision_result": vision_result
                        }
            
            # Default: continue
            return {
                "messages": [HumanMessage(content="Continue with the task.")],
                "loop_count": new_loop_count
            }
        
        # Build the graph
        builder = StateGraph(DinoState, input=DinoInputState)
        
        # Add nodes
        builder.add_node("call_model", call_model)
        builder.add_node("tools", ToolNode(self.tools))
        builder.add_node("check_condition", check_stop_condition)
        
        # Add edges
        builder.add_edge("__start__", "call_model")
        builder.add_conditional_edges("call_model", route_model_output)
        builder.add_edge("tools", "call_model")
        builder.add_edge("check_condition", "call_model")
        
        # Compile graph
        self.graph = builder.compile(name="Dino ReAct Agent")
    
    async def _simple_fallback_processing(self, state: DinoState) -> Dict[str, Any]:
        """Simple rule-based processing when LLM is not available"""
        
        # Get the last user message
        last_human_msg = None
        for msg in reversed(state.messages):
            if hasattr(msg, 'content') and hasattr(msg, 'type') and msg.type == "human":
                last_human_msg = msg.content
                break
        
        if not last_human_msg:
            return {"messages": [AIMessage(content="I need a command to process.")]}
        
        message_lower = last_human_msg.lower()
        
        # For iterative tasks with vision conditions
        if state.task_type == "iterative" and state.stop_condition:
            # Check current loop count
            if state.loop_count >= state.max_loops:
                return {"messages": [AIMessage(content=f"Reached maximum loops ({state.max_loops}). Stopping task.")]}
            
            # For vision-based stop conditions
            if "see" in state.stop_condition.lower():
                # First check what we see
                if self.platform.has_vision():
                    from langchain_core.messages import ToolMessage
                    vision_analysis = self.platform.vision.analyze_scene(
                        f"Do you see {state.stop_condition}? Answer YES or NO and explain what you see."
                    )
                    
                    # Check if condition is met
                    if vision_analysis and "yes" in vision_analysis.lower():
                        return {"messages": [AIMessage(content=f"✅ Stop condition met: {vision_analysis}")]}
                    else:
                        # Continue - need to move
                        if "move" in message_lower and "forward" in message_lower:
                            if self.platform.has_navigation():
                                success = self.platform.navigation.move_relative(0.5, 0, 0)
                                action_result = "Moved forward 0.5m" if success else "Failed to move forward"
                            else:
                                action_result = "Navigation not available"
                            
                            return {"messages": [AIMessage(
                                content=f"🔍 Vision: {vision_analysis}\n🚶 Action: {action_result}\n📍 Loop {state.loop_count + 1}/{state.max_loops} - Continuing..."
                            )]}
                else:
                    return {"messages": [AIMessage(content="Vision not available for condition checking.")]}
        
        # Simple command processing
        if "move" in message_lower:
            if "forward" in message_lower:
                if self.platform.has_navigation():
                    success = self.platform.navigation.move_relative(0.5, 0, 0)
                    return {"messages": [AIMessage(content="Moved forward" if success else "Failed to move forward")]}
                else:
                    return {"messages": [AIMessage(content="Navigation not available")]}
        
        elif "look" in message_lower or "see" in message_lower:
            if self.platform.has_vision():
                analysis = self.platform.vision.analyze_scene("Describe what you see")
                return {"messages": [AIMessage(content=f"I can see: {analysis}")]}
            else:
                return {"messages": [AIMessage(content="Vision not available")]}
        
        # Default response
        return {"messages": [AIMessage(content="I understand the task but need an LLM API key for full ReAct reasoning. Using simple fallback processing.")]}
    
    def _analyze_task_type(self, message: str) -> tuple[str, Optional[str]]:
        """Analyze the message to determine task type and stop condition"""
        message_lower = message.lower()
        
        # Check for iterative patterns
        iterative_patterns = [
            "until", "while", "keep", "continue", "repeatedly", "don't see", "stop when"
        ]
        
        if any(pattern in message_lower for pattern in iterative_patterns):
            # Extract stop condition
            if "until" in message_lower:
                parts = message_lower.split("until", 1)
                if len(parts) > 1:
                    condition = parts[1].strip()
                    return "iterative", condition
            elif "while" in message_lower:
                parts = message_lower.split("while", 1) 
                if len(parts) > 1:
                    condition = f"not ({parts[1].strip()})"
                    return "iterative", condition
            elif "don't see" in message_lower or "stop when" in message_lower:
                return "iterative", message
            
            return "iterative", None
        
        # Check for complex multi-step tasks
        complex_patterns = [
            "first", "then", "after", "and then", "followed by", "next"
        ]
        
        if any(pattern in message_lower for pattern in complex_patterns):
            return "complex", None
        
        return "simple", None
    
    async def process_message(self, message: str) -> str:
        """Process a message using ReAct reasoning"""
        
        if not LANGGRAPH_AVAILABLE:
            # Fallback to simple processing
            return "ReAct processing not available. LangGraph dependencies missing."
        
        # Analyze task type
        task_type, stop_condition = self._analyze_task_type(message)
        
        logger.info(f"Processing {task_type} task: {message}")
        if stop_condition:
            logger.info(f"Stop condition: {stop_condition}")
        
        # Create initial state
        initial_state = DinoInputState(
            messages=[HumanMessage(content=message)],
            original_query=message,
            task_type=task_type
        )
        
        # Convert to full state for graph execution
        full_state = DinoState(
            messages=initial_state.messages,
            original_query=initial_state.original_query,
            task_type=initial_state.task_type,
            stop_condition=stop_condition,
            max_loops=10 if task_type == "iterative" else 5,
            platform_status=self.platform.get_available_capabilities()
        )
        
        try:
            # Run the ReAct graph
            config = {"recursion_limit": self.config.max_iterations}
            final_state = await self.graph.ainvoke(full_state, config=config)
            
            # Extract the final response
            last_message = final_state["messages"][-1]
            if isinstance(last_message, AIMessage):
                return last_message.content or "Task completed."
            else:
                return "Task completed successfully."
                
        except Exception as e:
            logger.error(f"ReAct processing error: {e}")
            return f"I encountered an error while processing your request: {e}"
    
    def is_available(self) -> bool:
        """Check if ReAct capabilities are available"""
        return LANGGRAPH_AVAILABLE and self.graph is not None


# Factory function for easy integration
def create_react_brain(platform: Optional[PlatformInterface] = None,
                      config: Optional[DinoConfiguration] = None) -> DinoReActBrain:
    """Create and return a Dino ReAct Brain"""
    return DinoReActBrain(platform, config)


# Export for LangGraph CLI
def create_graph_for_cli():
    """Create graph for LangGraph CLI"""
    try:
        from dino_ai.core.interfaces import create_null_platform
        brain = DinoReActBrain(create_null_platform())
        return brain.graph
    except Exception as e:
        logger.error(f"Failed to create graph for CLI: {e}")
        return None

# Initialize graph for CLI
graph = create_graph_for_cli()