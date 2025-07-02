#!/usr/bin/env python3
"""
Test script for Dino AI Standalone
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import asyncio
import logging

from dino_ai.core.dino_brain import DinoBrain
from dino_ai.adapters.simulation_adapter import create_simulation_platform


def setup_logging():
    """Setup logging configuration"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


async def test_simulation_platform():
    """Test Dino AI with simulation platform"""
    
    print("🦕 Testing Dino AI with Simulation Platform")
    print("=" * 50)
    
    # Create simulation platform
    platform = create_simulation_platform()
    
    if not platform.initialize({}):
        print("❌ Failed to initialize simulation platform")
        return
    
    print(f"✅ Platform initialized: {type(platform).__name__}")
    print(f"📋 Capabilities: {', '.join(platform.get_available_capabilities())}")
    print(f"🚗 Navigation: {'✓' if platform.has_navigation() else '✗'}")
    print(f"🦾 Manipulation: {'✓' if platform.has_manipulation() else '✗'}")
    print(f"👁️ Vision: {'✓' if platform.has_vision() else '✗'}")
    print(f"🔊 Speech: {'✓' if platform.has_speech() else '✗'}")
    print("-" * 50)
    
    # Create Dino Brain
    dino = DinoBrain(platform)
    
    # Test messages
    test_messages = [
        "Hello, introduce yourself",
        "What can you do?", 
        "What do you see?",
        "Move forward",
        "Move your arm up",
        "Open the gripper",
        "Check your status",
        "Thank you!"
    ]
    
    print("🎬 Running test sequence...")
    print()
    
    for i, message in enumerate(test_messages, 1):
        print(f"Test {i}: {message}")
        try:
            response = await dino.process_message(message)
            print(f"🦕: {response}")
        except Exception as e:
            print(f"❌ Error: {e}")
        print()
        await asyncio.sleep(1)  # Pause between tests
    
    print("✅ Test sequence completed!")
    
    # Cleanup
    platform.shutdown()


async def test_null_platform():
    """Test Dino AI with null platform (text-only)"""
    
    print("🦕 Testing Dino AI with Null Platform (Text-only)")
    print("=" * 50)
    
    # Create null platform
    from dino_ai.core.interfaces import create_null_platform
    platform = create_null_platform()
    
    if not platform.initialize({}):
        print("❌ Failed to initialize null platform")
        return
    
    print(f"✅ Platform initialized: {type(platform).__name__}")
    print(f"📋 Capabilities: {', '.join(platform.get_available_capabilities())}")
    print("-" * 50)
    
    # Create Dino Brain
    dino = DinoBrain(platform)
    
    # Test messages that should work without hardware
    test_messages = [
        "Hello",
        "What can you do?",
        "Help",
        "Move forward",  # Should gracefully fail
        "What do you see?",  # Should gracefully fail  
        "Thanks"
    ]
    
    print("🎬 Running test sequence...")
    print()
    
    for i, message in enumerate(test_messages, 1):
        print(f"Test {i}: {message}")
        try:
            response = await dino.process_message(message)
            print(f"🦕: {response}")
        except Exception as e:
            print(f"❌ Error: {e}")
        print()
        await asyncio.sleep(0.5)
    
    print("✅ Test sequence completed!")
    
    # Cleanup
    platform.shutdown()


async def main():
    """Main test function"""
    setup_logging()
    
    print("🧪 Testing Platform-Independent Dino AI")
    print("=" * 60)
    print()
    
    # Test simulation platform
    await test_simulation_platform()
    print()
    print("=" * 60)
    print()
    
    # Test null platform
    await test_null_platform()
    print()
    print("🎉 All tests completed!")


if __name__ == "__main__":
    asyncio.run(main())