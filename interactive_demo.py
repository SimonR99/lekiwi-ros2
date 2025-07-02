#!/usr/bin/env python3
"""
Interactive Demo for Platform-Independent Dino AI
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import asyncio
import logging

from dino_ai.core.dino_brain import DinoBrain
from dino_ai.adapters.simulation_adapter import create_simulation_platform
from dino_ai.core.interfaces import create_null_platform


def setup_logging():
    """Setup logging configuration"""
    logging.basicConfig(
        level=logging.WARNING,  # Reduced logging for interactive demo
        format='%(name)s - %(levelname)s - %(message)s'
    )


def choose_platform():
    """Let user choose platform"""
    print("🦕 Welcome to Dino AI - Platform Independent!")
    print("=" * 50)
    print("Choose a platform:")
    print("1. 🎮 Simulation - Full robot simulation (recommended)")
    print("2. 💬 Text-only - Pure conversation AI")
    print("3. 🤖 ROS - Real robot (requires ROS)")
    print()
    
    while True:
        choice = input("Enter choice (1-3): ").strip()
        
        if choice == "1":
            platform = create_simulation_platform()
            platform_name = "Simulation"
            break
        elif choice == "2":
            platform = create_null_platform()
            platform_name = "Text-only"
            break
        elif choice == "3":
            try:
                from dino_ai.adapters.ros_adapter import create_ros_platform
                platform = create_ros_platform()
                platform_name = "ROS"
                break
            except ImportError:
                print("❌ ROS not available. Please choose another platform.")
                continue
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")
            continue
    
    print(f"\n🚀 Initializing {platform_name} platform...")
    
    if not platform.initialize({}):
        print(f"❌ Failed to initialize {platform_name} platform")
        return None, None
    
    print(f"✅ {platform_name} platform ready!")
    print(f"📋 Capabilities: {', '.join(platform.get_available_capabilities())}")
    
    return platform, platform_name


async def interactive_session(platform, platform_name):
    """Run interactive session"""
    
    # Create Dino Brain
    dino = DinoBrain(platform)
    
    print(f"\n🦕 Dino AI ({platform_name}) Interactive Mode")
    print("-" * 50)
    print("Try these commands:")
    print("• 'help' - Get help")
    print("• 'what can you do?' - See capabilities")
    
    if platform.has_navigation():
        print("• 'move forward' - Navigation")
        print("• 'where am I?' - Current position")
    
    if platform.has_manipulation():
        print("• 'open gripper' - Manipulation")
        print("• 'arm position' - Get arm pose")
    
    if platform.has_vision():
        print("• 'what do you see?' - Vision analysis")
        print("• 'describe the scene' - Scene description")
    
    print("• 'status' - System status")
    print("• 'exit' or 'quit' - Exit")
    print("-" * 50)
    
    # Initial greeting
    greeting = await dino.process_message("Hello")
    print(f"🦕: {greeting}")
    print()
    
    try:
        while True:
            user_input = input("You: ").strip()
            
            if not user_input:
                continue
            
            if user_input.lower() in ['exit', 'quit', 'bye', 'goodbye']:
                farewell = await dino.process_message("goodbye")
                print(f"🦕: {farewell}")
                break
            
            try:
                response = await dino.process_message(user_input)
                print(f"🦕: {response}")
            except Exception as e:
                print(f"🦕: Sorry, I encountered an error: {e}")
            
            print()
    
    except KeyboardInterrupt:
        print("\n🦕: Goodbye! 👋")
    
    finally:
        platform.shutdown()


async def main():
    """Main demo function"""
    setup_logging()
    
    # Choose platform
    platform, platform_name = choose_platform()
    
    if not platform:
        print("❌ Failed to initialize platform")
        return
    
    # Run interactive session
    await interactive_session(platform, platform_name)
    
    print("\n👋 Thanks for trying Dino AI!")


if __name__ == "__main__":
    asyncio.run(main())