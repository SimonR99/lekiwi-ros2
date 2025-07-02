#!/usr/bin/env python3
"""
Launch script for Moondream Vision Server

Starts the Moondream server for Dino AI vision capabilities.
"""

import subprocess
import sys
import os
import time
import requests

def check_dependencies():
    """Check if required dependencies are installed"""
    try:
        import torch
        import transformers
        import fastapi
        import uvicorn
        print("✅ All dependencies available")
        return True
    except ImportError as e:
        print(f"❌ Missing dependency: {e}")
        print("Install with: pip install torch transformers fastapi uvicorn pillow")
        return False

def check_server_running(host="127.0.0.1", port=5000):
    """Check if server is already running"""
    try:
        response = requests.get(f"http://{host}:{port}/health", timeout=2)
        return response.status_code == 200
    except:
        return False

def main():
    """Main launch function"""
    print("🦕 Dino AI - Moondream Vision Server Launcher")
    print("=" * 50)
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Check if already running
    if check_server_running():
        print("✅ Moondream server is already running at http://127.0.0.1:5000")
        print("Use Ctrl+C to stop, or restart manually if needed")
        return
    
    # Change to servers directory
    servers_dir = os.path.join(os.path.dirname(__file__), "servers")
    if not os.path.exists(servers_dir):
        print(f"❌ Servers directory not found: {servers_dir}")
        sys.exit(1)
    
    server_script = os.path.join(servers_dir, "moondream_server.py")
    if not os.path.exists(server_script):
        print(f"❌ Moondream server script not found: {server_script}")
        sys.exit(1)
    
    print("🚀 Starting Moondream server...")
    print("This may take a while on first run (downloading model)")
    print("-" * 50)
    
    try:
        # Launch the server
        process = subprocess.run([
            sys.executable, server_script,
            "--host", "127.0.0.1",
            "--port", "5000"
        ], cwd=servers_dir)
        
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
    except Exception as e:
        print(f"❌ Failed to start server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()