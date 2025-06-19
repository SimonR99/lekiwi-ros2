#!/usr/bin/env python3
"""
Dino AI Standalone - Main Entry Point

Platform-independent AI system that can work with or without ROS,
with or without physical embodiment.
"""

import argparse
import asyncio
import logging
import sys
import os
from typing import Optional

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    # Try to load manually if python-dotenv is not available
    def load_dotenv():
        env_path = os.path.join(os.path.dirname(__file__), '..', '.env')
        if os.path.exists(env_path):
            with open(env_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        key = key.strip()
                        value = value.strip().strip('"').strip("'")
                        os.environ[key] = value
    load_dotenv()

# Core system
from dino_ai.core.dino_brain import DinoBrain
from dino_ai.core.interfaces import PlatformInterface

# Platform adapters
from dino_ai.adapters.simulation_adapter import create_simulation_platform

# Optional ROS adapter
try:
    from dino_ai.adapters.ros_adapter import create_ros_platform
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def setup_logging(level: str = "INFO"):
    """Setup logging configuration"""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def create_platform(platform_type: str, config: dict) -> Optional[PlatformInterface]:
    """Create platform adapter based on type"""
    
    if platform_type == "simulation":
        platform = create_simulation_platform()
    elif platform_type == "ros" and ROS_AVAILABLE:
        platform = create_ros_platform()
    elif platform_type == "ros" and not ROS_AVAILABLE:
        print("❌ ROS platform requested but ROS not available")
        return None
    elif platform_type == "null":
        from dino_ai.core.interfaces import create_null_platform
        platform = create_null_platform()
    else:
        print(f"❌ Unknown platform type: {platform_type}")
        return None
    
    # Initialize platform
    if platform.initialize(config):
        print(f"✅ {platform_type.title()} platform initialized")
        return platform
    else:
        print(f"❌ Failed to initialize {platform_type} platform")
        return None


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Dino AI - Platform Independent AI System")
    
    parser.add_argument(
        "--platform",
        choices=["simulation", "ros", "null"],
        default="simulation",
        help="Platform to use (default: simulation)"
    )
    
    parser.add_argument(
        "--mode",
        choices=["interactive"],
        default="interactive", 
        help="Run mode (default: interactive)"
    )
    
    parser.add_argument(
        "--speech",
        action="store_true",
        help="Enable speech interface (if available)"
    )
    
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level (default: INFO)"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        help="Path to configuration file"
    )
    
    parser.add_argument(
        "--mcp-config",
        type=str,
        help="Path to MCP server configuration file"
    )
    
    parser.add_argument(
        "--text",
        type=str,
        help="Single text prompt to process (non-interactive)"
    )
    
    parser.add_argument(
        "--no-react",
        action="store_true",
        help="Disable ReAct reasoning for complex tasks"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging(args.log_level)
    
    # Print startup banner
    print("🦕 Dino AI - Platform Independent AI System")
    print("=" * 50)
    print(f"Platform: {args.platform}")
    print(f"Mode: {args.mode}")
    print(f"Speech: {'Enabled' if args.speech else 'Disabled'}")
    print(f"ROS Available: {'Yes' if ROS_AVAILABLE else 'No'}")
    print("-" * 50)
    
    # Create platform
    config = {}  # Load from file if specified
    platform = create_platform(args.platform, config)
    
    if not platform:
        print("❌ Failed to create platform")
        sys.exit(1)
    
    # Create Dino Brain with MCP config and ReAct settings
    use_react = not args.no_react
    dino = DinoBrain(platform, mcp_config_path=args.mcp_config, use_react=use_react)
    
    try:
        # Handle single text prompt
        if args.text:
            print(f"🎯 Processing single prompt: {args.text}")
            response = await dino.process_message(args.text)
            print(f"🦕: {response}")
            return
        
        # Start interactive mode
        print("🎯 Starting interactive mode...")
        print("Type 'help' for commands, 'exit' to quit")
        await dino.start_interactive_mode(use_speech=args.speech)
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    
    except Exception as e:
        print(f"❌ Runtime error: {e}")
        logging.error(f"Runtime error: {e}", exc_info=True)
    
    finally:
        # Cleanup
        print("🧹 Shutting down...")
        platform.shutdown()
        print("👋 Goodbye!")


def main_sync():
    """Synchronous main function for entry point"""
    asyncio.run(main())


if __name__ == "__main__":
    main_sync()