"""
MCP Adapter for Dino AI

Provides integration between Dino AI agents and MCP (Model Context Protocol) servers
for dynamic tool loading and external service integration.
"""

import asyncio
import logging
from typing import Dict, List, Optional, Any
from pathlib import Path

from dino_ai.core.mcp_client import MCPClient, MCPConfig, ToolNotFoundError

logger = logging.getLogger(__name__)


class MCPAdapter:
    """
    Adapter that connects Dino AI agents with MCP servers
    to provide dynamic tool capabilities.
    """
    
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path
        self.mcp_client = None
        self.initialized = False
        self.agent_tools = {
            "conversation": [],
            "navigation": [],
            "manipulation": [],
            "vision": [],
            "system": []
        }
    
    async def initialize(self) -> bool:
        """Initialize MCP connections and discover tools"""
        try:
            config = MCPConfig(self.config_path)
            self.mcp_client = MCPClient(config)
            
            success = await self.mcp_client.initialize()
            if success:
                await self._distribute_tools_to_agents()
                self.initialized = True
                logger.info("MCP adapter initialized successfully")
                return True
            else:
                logger.info("MCP adapter initialization skipped (no servers configured)")
                return False
                
        except Exception as e:
            logger.error(f"Failed to initialize MCP adapter: {e}")
            return False
    
    async def _distribute_tools_to_agents(self):
        """Distribute MCP tools to appropriate agents based on functionality"""
        if not self.mcp_client:
            return
        
        # Get tools by category
        speech_tools = self.mcp_client.get_tools_by_category("speech")
        vision_tools = self.mcp_client.get_tools_by_category("vision")
        math_tools = self.mcp_client.get_tools_by_category("math")
        communication_tools = self.mcp_client.get_tools_by_category("communication")
        data_tools = self.mcp_client.get_tools_by_category("data")
        external_tools = self.mcp_client.get_tools_by_category("external")
        
        # Distribute to agents
        self.agent_tools["conversation"] = speech_tools + communication_tools
        self.agent_tools["vision"] = vision_tools
        self.agent_tools["system"] = math_tools + data_tools + external_tools
        
        # Log distribution
        for agent, tools in self.agent_tools.items():
            if tools:
                logger.info(f"Assigned {len(tools)} MCP tools to {agent} agent: {[t.name for t in tools]}")
    
    async def get_tools_for_agent(self, agent_name: str) -> List[Any]:
        """Get MCP tools assigned to a specific agent"""
        if not self.initialized:
            return []
        return self.agent_tools.get(agent_name, [])
    
    async def call_tool(self, tool_name: str, args: dict) -> Any:
        """Call an MCP tool"""
        if not self.initialized or not self.mcp_client:
            raise ToolNotFoundError("MCP adapter not initialized")
        
        return await self.mcp_client.call_tool(tool_name, args)
    
    def has_tool(self, tool_name: str) -> bool:
        """Check if a tool is available"""
        if not self.initialized or not self.mcp_client:
            return False
        return self.mcp_client.has_tool(tool_name)
    
    def get_tool_info(self, tool_name: str) -> Optional[dict]:
        """Get information about a specific tool"""
        if not self.initialized or not self.mcp_client:
            return None
        return self.mcp_client.get_tool_info(tool_name)
    
    def get_all_tools(self) -> Dict[str, Any]:
        """Get all available MCP tools"""
        if not self.initialized or not self.mcp_client:
            return {}
        return self.mcp_client.get_all_tools()
    
    async def enhance_agent_response(self, agent_name: str, response: str, context: dict) -> str:
        """
        Enhance agent response using MCP tools.
        
        For example:
        - Conversation agent: Add TTS
        - Vision agent: Add image processing
        - System agent: Add calculations
        """
        if not self.initialized:
            return response
        
        try:
            # Auto-enhance based on agent type
            if agent_name == "conversation":
                # Try to speak the response
                if self.has_tool("speak_text"):
                    await self.call_tool("speak_text", {"text": response})
                    logger.debug("Response enhanced with TTS")
            
            elif agent_name == "vision" and "image" in context:
                # Could enhance with image processing tools
                pass
            
            elif agent_name == "system":
                # Could enhance with calculation tools
                pass
                
        except Exception as e:
            logger.warning(f"Failed to enhance {agent_name} response: {e}")
        
        return response
    
    async def shutdown(self):
        """Shutdown MCP connections"""
        if self.mcp_client:
            await self.mcp_client.shutdown()
        self.initialized = False
        logger.info("MCP adapter shutdown")


def create_mcp_adapter(config_path: Optional[str] = None) -> MCPAdapter:
    """Create and return an MCP adapter"""
    return MCPAdapter(config_path)


# Example MCP configuration template
EXAMPLE_MCP_CONFIG = """
{
  "mcpServers": {
    "math": {
      "command": "python",
      "args": ["mcp_servers/math_server.py"],
      "transport": "sse",
      "host": "127.0.0.1",
      "port": 8000
    },
    "tts": {
      "command": "python",
      "args": ["mcp_servers/piper_server.py"],
      "transport": "sse",
      "host": "127.0.0.1",
      "port": 8001
    },
    "vision": {
      "command": "python",
      "args": ["mcp_servers/vision_server.py"],
      "transport": "sse",
      "host": "127.0.0.1",
      "port": 8002
    }
  }
}
"""


def create_example_config(config_path: str):
    """Create an example MCP configuration file"""
    with open(config_path, 'w') as f:
        f.write(EXAMPLE_MCP_CONFIG)
    logger.info(f"Created example MCP config at {config_path}")