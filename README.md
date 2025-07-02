# Lekiwi Robot + Dino AI Project

This project combines the **Lekiwi robot platform** with **Dino AI**, a platform-independent AI system for robotics.

## Project Structure

```
new_tentative/
├── dino_ai/                    # 🦕 Platform-Independent AI System
│   ├── core/                   # Core AI logic (platform-agnostic)
│   │   ├── dino_brain.py      # Main AI orchestration
│   │   └── interfaces.py      # Abstract platform interfaces
│   ├── adapters/               # Platform-specific implementations
│   │   ├── ros_adapter.py     # ROS2/Lekiwi robot adapter
│   │   └── simulation_adapter.py # Simulation adapter
│   ├── main.py                # Main entry point
│   ├── pyproject.toml         # Python package configuration
│   └── README.md              # Dino AI documentation
│
├── src/                       # 🤖 ROS2 Packages
│   ├── lekiwi/               # Lekiwi robot package
│   └── lekiwi_hardware/      # Hardware interface
│
├── interactive_demo.py       # 🎮 Interactive demo script
├── test_dino_standalone.py   # 🧪 Test script
└── ARCHITECTURE_REVISED.md   # 📖 Architecture documentation
```

## Key Features

### 🦕 **Dino AI - Platform Independent**
- **No ROS dependency** - Works standalone
- **Multiple platforms** - ROS, simulation, text-only
- **Graceful degradation** - Adapts to available capabilities
- **Easy deployment** - Cloud, edge, or local

### 🤖 **Lekiwi Robot Platform**
- **5-DOF robotic arm** with gripper
- **Omnidirectional base** with 3 wheels
- **Full ROS2 integration** (Nav2 + MoveIt)
- **Camera system** for vision

## Quick Start

### 1. Text-Only AI (No Hardware Needed)
```bash
python -m dino_ai.main --platform null
# or
python interactive_demo.py  # Choose option 2
```

### 2. Full Robot Simulation
```bash
python -m dino_ai.main --platform simulation --mode demo
# or  
python interactive_demo.py  # Choose option 1
```

### 3. Real Robot Control (Requires ROS)
```bash
# First, launch the robot
ros2 launch lekiwi lekiwi_full.launch.py

# Then run Dino AI
python -m dino_ai.main --platform ros
# or
python interactive_demo.py  # Choose option 3
```

## Usage Examples

### **Interactive Conversation**
```
You: Hello Dino
🦕: Hello! I'm Dino AI running on a Simulation platform with these 
    capabilities: navigation, manipulation, vision, speech. How can I help?

You: What can you do?
🦕: I can navigate and move around, control robotic arms and grippers, 
    see and analyze images. I'm also great at conversation!

You: Move forward
🦕: Moving forward 1 meter

You: What do you see?
🦕: I can see: A room with colorful geometric shapes...
```

### **Vision Integration**
When you ask "what do you see?":
- **Simulation**: AI-generated scene descriptions
- **ROS Robot**: Real camera → AI vision analysis
- **Text-only**: Graceful "vision not available" response

## Platform Capabilities

| Feature       | Text-Only | Simulation | ROS Robot |
|---------------|-----------|------------|-----------|
| Conversation  | ✅        | ✅         | ✅        |
| Navigation    | ❌        | ✅ (sim)   | ✅        |
| Manipulation  | ❌        | ✅ (sim)   | ✅        |
| Vision        | ❌        | ✅ (sim)   | ✅        |
| Speech        | ⚠️ text   | ✅ (text)  | ✅        |

## Architecture Benefits

### **🔧 For Developers**
- Develop AI logic without hardware
- Test in simulation before deployment
- Platform-independent codebase

### **☁️ For Deployment**
- Deploy as pure chatbot (no ROS needed)
- Cloud-friendly architecture
- Easy scaling and distribution

### **🤖 For Robotics**
- Full ROS2 integration when needed
- Proven navigation and manipulation
- Real-time robot control

## Installation & Development

### Prerequisites
- **Python 3.8+** (for Dino AI)
- **ROS2 Humble** (for robot control)
- **Ubuntu 22.04** (recommended)

### Setup
```bash
# Install Dino AI
cd dino_ai
pip install -e .

# For ROS development
cd ..
colcon build --packages-select lekiwi lekiwi_hardware
source install/setup.bash
```

## Key Design Decisions

1. **Dino AI is NOT a ROS package** - It's a standalone Python package
2. **ROS integration is optional** - Via adapter pattern
3. **Clean separation** - AI logic independent of hardware
4. **Multiple deployment modes** - Same AI, different platforms

## Contributing

1. **Dino AI development** - Focus on `dino_ai/` directory
2. **Robot development** - Work in `src/lekiwi/` 
3. **Integration** - Modify `dino_ai/adapters/ros_adapter.py`

## License

MIT License - See individual package LICENSE files.

---

**🦕 Dino AI - Making robots intelligent, one conversation at a time!** 🤖✨