"""
MCP (Model Context Protocol) Client Integration

Provides dynamic tool discovery and integration with external MCP servers
for extending Dino AI capabilities without code changes.
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Any
from pathlib import Path
from abc import ABC, abstractmethod

try:
    from langchain_mcp_adapters.client import MultiServerMCPClient
    from langchain_core.tools import Tool
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    Tool = None

logger = logging.getLogger(__name__)


class ToolNotFoundError(Exception):
    """Raised when a requested MCP tool is not available"""
    pass


class MCPConfig:
    """Configuration for MCP servers"""
    
    def __init__(self, config_path: Optional[str] = None):
        self.servers = {}
        if config_path and Path(config_path).exists():
            self.load_config(config_path)
    
    def load_config(self, config_path: str):
        """Load MCP server configuration from JSON file"""
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                self.servers = config.get("mcpServers", {})
        except Exception as e:
            logger.error(f"Failed to load MCP config from {config_path}: {e}")
    
    def add_server(self, name: str, config: dict):
        """Add MCP server configuration"""
        self.servers[name] = config
    
    def get_servers(self) -> dict:
        """Get all server configurations"""
        return self.servers


class MCPClient:
    """Client for managing MCP server connections and tools"""
    
    def __init__(self, config: MCPConfig):
        self.config = config
        self.client = None
        self.available_tools = {}
        self.tool_categories = {
            "speech": [],
            "vision": [], 
            "math": [],
            "communication": [],
            "data": [],
            "external": []
        }
        
        if not MCP_AVAILABLE:
            logger.warning("MCP libraries not available. Install with: pip install langchain-mcp-adapters")
    
    async def initialize(self) -> bool:
        """Initialize MCP client and connect to servers"""
        if not MCP_AVAILABLE:
            logger.info("MCP not available, skipping initialization")
            return False
        
        try:
            if self.config.get_servers():
                self.client = MultiServerMCPClient(self.config.get_servers())
                await self.discover_tools()
                logger.info(f"MCP client initialized with {len(self.available_tools)} tools")
                return True
            else:
                logger.info("No MCP servers configured")
                return False
        except Exception as e:
            logger.error(f"Failed to initialize MCP client: {e}")
            return False
    
    async def discover_tools(self) -> List[Tool]:
        """Discover available tools from all MCP servers"""
        if not self.client:
            return []
        
        try:
            tools = await self.client.get_tools()
            self.available_tools = {tool.name: tool for tool in tools}
            self._categorize_tools(tools)
            
            logger.info(f"Discovered {len(tools)} MCP tools:")
            for category, tool_list in self.tool_categories.items():
                if tool_list:
                    logger.info(f"  {category}: {[t.name for t in tool_list]}")
            
            return tools
        except Exception as e:
            logger.error(f"Failed to discover MCP tools: {e}")
            return []
    
    def _categorize_tools(self, tools: List[Tool]):
        """Categorize tools by their functionality"""
        self.tool_categories = {
            "speech": [],
            "vision": [], 
            "math": [],
            "communication": [],
            "data": [],
            "external": []
        }
        
        for tool in tools:
            name_lower = tool.name.lower()
            desc_lower = tool.description.lower() if tool.description else ""
            
            if any(keyword in name_lower or keyword in desc_lower 
                   for keyword in ["speak", "tts", "speech", "audio", "voice"]):
                self.tool_categories["speech"].append(tool)
            elif any(keyword in name_lower or keyword in desc_lower 
                     for keyword in ["vision", "image", "see", "detect", "ocr"]):
                self.tool_categories["vision"].append(tool)
            elif any(keyword in name_lower or keyword in desc_lower 
                     for keyword in ["math", "calculate", "compute", "multiply", "add"]):
                self.tool_categories["math"].append(tool)
            elif any(keyword in name_lower or keyword in desc_lower 
                     for keyword in ["email", "sms", "notify", "message"]):
                self.tool_categories["communication"].append(tool)
            elif any(keyword in name_lower or keyword in desc_lower 
                     for keyword in ["database", "data", "file", "storage"]):
                self.tool_categories["data"].append(tool)
            else:
                self.tool_categories["external"].append(tool)
    
    async def call_tool(self, name: str, args: dict) -> Any:
        """Execute a tool via MCP"""
        if not self.client:
            raise ToolNotFoundError("MCP client not initialized")
        
        tool = self.available_tools.get(name)
        if not tool:
            raise ToolNotFoundError(f"Tool '{name}' not available. Available tools: {list(self.available_tools.keys())}")
        
        try:
            result = await tool.ainvoke(args)
            logger.debug(f"MCP tool '{name}' executed successfully")
            return result
        except Exception as e:
            logger.error(f"Failed to execute MCP tool '{name}': {e}")
            raise
    
    def get_tools_by_category(self, category: str) -> List[Tool]:
        """Get tools by category"""
        return self.tool_categories.get(category, [])
    
    def get_all_tools(self) -> Dict[str, Tool]:
        """Get all available tools"""
        return self.available_tools
    
    def has_tool(self, name: str) -> bool:
        """Check if a tool is available"""
        return name in self.available_tools
    
    def get_tool_info(self, name: str) -> Optional[dict]:
        """Get information about a specific tool"""
        tool = self.available_tools.get(name)
        if tool:
            return {
                "name": tool.name,
                "description": tool.description,
                "args_schema": getattr(tool, 'args_schema', None)
            }
        return None
    
    async def shutdown(self):
        """Shutdown MCP client connections"""
        if self.client:
            try:
                # Close connections gracefully
                logger.info("Shutting down MCP client")
            except Exception as e:
                logger.error(f"Error during MCP shutdown: {e}")


class NullMCPClient(MCPClient):
    """Null implementation when MCP is not available"""
    
    def __init__(self):
        super().__init__(MCPConfig())
    
    async def initialize(self) -> bool:
        logger.info("Using Null MCP client (MCP not available)")
        return False
    
    async def discover_tools(self) -> List[Tool]:
        return []
    
    async def call_tool(self, name: str, args: dict) -> Any:
        raise ToolNotFoundError(f"MCP not available - cannot call tool '{name}'")


def create_mcp_client(config_path: Optional[str] = None) -> MCPClient:
    """Create MCP client with configuration"""
    if MCP_AVAILABLE:
        config = MCPConfig(config_path)
        return MCPClient(config)
    else:
        return NullMCPClient()