# Dino AI - Platform Independent AI System

🦕 A versatile AI system that can work with or without ROS, with or without physical embodiment.

## Overview

Dino AI is designed as a **platform-independent AI system** that provides intelligent robot control through abstract interfaces. It can work equally well on:

- 🤖 **Physical robots** (via ROS adapters)
- 🎮 **Simulation environments** (built-in simulation)  
- 💬 **Text-only systems** (pure conversation AI)
- ☁️ **Cloud deployments** (no hardware needed)

## Key Features

- **Platform Independence**: Core AI logic works across different platforms
- **Capability Detection**: Automatically adapts to available hardware
- **Graceful Degradation**: Works even when some capabilities are missing
- **Modular Architecture**: Easy to add new platforms and capabilities
- **ROS Integration**: Optional ROS adapter for robotics platforms

## Quick Start

### Installation

```bash
# Basic installation
pip install dino-ai

# With ROS support
pip install dino-ai[ros]

# With AI capabilities
pip install dino-ai[ai]

# Full installation
pip install dino-ai[all]
```

### Usage Examples

#### 1. Text-Only AI (No Hardware)
```bash
dino-ai --platform null
```
Perfect for testing AI logic or deploying as a chatbot.

#### 2. Simulation Mode (Development)
```bash
dino-ai --platform simulation
```
Full robot simulation for development and testing.

#### 3. ROS Robot Control
```bash
dino-ai --platform ros
```
Control real robots via ROS2.

#### 4. Interactive Modes
```bash
# Text interaction
dino-ai --mode interactive

# Demo mode
dino-ai --mode demo

# With speech (if available)
dino-ai --speech
```

## Platform Capabilities

| Capability    | Null | Simulation | ROS |
|---------------|------|------------|-----|
| Conversation  | ✅   | ✅         | ✅  |
| Navigation    | ❌   | ✅ (sim)   | ✅  |
| Manipulation  | ❌   | ✅ (sim)   | ✅  |
| Vision        | ❌   | ✅ (sim)   | ✅  |
| Speech        | ⚠️   | ✅ (text)  | ✅  |

## Example Conversation

```
You: Hello Dino
🦕: Hello! I'm Dino AI running on a Simulation platform with these 
    capabilities: navigation, manipulation, vision, speech. How can I help?

You: What can you do?
🦕: I can navigate and move around, control robotic arms and grippers, 
    see and analyze images. I'm also great at conversation!

You: Move forward
🦕: Moving forward 1 meter
[Platform adapter executes movement]

You: What do you see?
🦕: I can see: A room with colorful geometric shapes. There's a red 
    rectangle on the left, a green circle in the center...
```

## Architecture

```
┌─────────────────────────────────────────┐
│            User Interface               │
├─────────────────────────────────────────┤
│             Dino Brain                  │
│         (Core AI Logic)                 │
├─────────────────────────────────────────┤
│         Abstract Interfaces             │
├─────────────────┬───────────────────────┤
│ ROS Adapter     │ Simulation │ Null     │
│ (Real Robot)    │ Adapter    │ Adapter  │
└─────────────────┴────────────┴──────────┘
```

## Development

### Creating Custom Adapters

```python
from dino_ai.core.interfaces import PlatformInterface

class MyCustomAdapter(PlatformInterface):
    def initialize(self, config):
        # Initialize your platform
        return True
    
    def get_available_capabilities(self):
        return ["my_capability"]
```

### Adding New Capabilities

```python
from dino_ai.core.interfaces import NavigationInterface

class MyNavigationAdapter(NavigationInterface):
    def get_current_pose(self):
        # Return current robot pose
        pass
    
    def navigate_to_pose(self, pose):
        # Navigate to target pose
        pass
```

## Requirements

### Core (Always Required)
- Python 3.8+
- numpy
- pillow

### Optional Dependencies
- **ROS**: rclpy, ROS2 message packages
- **AI**: langchain, langgraph, openai/anthropic
- **Vision**: opencv-python, torch, transformers  
- **Speech**: SpeechRecognition, pyttsx3, faster-whisper

## Configuration

Dino AI automatically detects available capabilities and adapts accordingly. For advanced configuration:

```python
config = {
    "log_level": "INFO",
    "vision_model": "moondream",
    "speech_enabled": True
}

platform = create_simulation_platform()
platform.initialize(config)
dino = DinoBrain(platform)
```

## Use Cases

### 🤖 **Robotics Development**
- Develop AI logic without hardware
- Test in simulation before deployment
- Easy transition from sim to real robot

### 💬 **Chatbot Deployment** 
- Deploy as pure conversation AI
- No robotics dependencies needed
- Cloud-friendly architecture

### 🔬 **Research & Education**
- Platform-independent AI research
- Easy to understand architecture
- Extensible for custom platforms

### 🏭 **Production Systems**
- Reliable robotics control
- Graceful error handling
- Platform flexibility

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions welcome! Please see CONTRIBUTING.md for guidelines.

## Support

- 📖 Documentation: See README and code comments
- 🐛 Issues: GitHub Issues  
- 💬 Discussions: GitHub Discussions