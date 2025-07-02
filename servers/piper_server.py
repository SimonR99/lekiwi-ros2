#!/usr/bin/env python3
"""
Piper TTS MCP Server for Dino AI

Provides text-to-speech capabilities using Piper TTS via MCP protocol.
"""

import os
import queue
import logging
from pathlib import Path
import numpy as np
import sounddevice as sd
from fastmcp import FastMCP
import json

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize MCP server
mcp = FastMCP("PiperTTSServer")

# Try to import Piper, make it optional
try:
    from piper.voice import PiperVoice
    PIPER_AVAILABLE = True
except ImportError:
    PIPER_AVAILABLE = False
    logger.warning("Piper TTS not available. Install with: pip install piper-tts")

# Configuration
DEFAULT_MODEL = "en_US-lessac-medium.onnx"
MODELS_DIR = "models/piper_voices"

class TTSEngine:
    def __init__(self):
        self.voice = None
        self.stream = None
        self.speech_queue = queue.Queue()
        self.is_speaking = False
        
        if PIPER_AVAILABLE:
            self._initialize_voice()
    
    def _initialize_voice(self):
        """Initialize Piper voice"""
        try:
            model_path = os.path.join(MODELS_DIR, DEFAULT_MODEL)
            if os.path.exists(model_path):
                self.voice = PiperVoice.load(model_path)
                self.stream = sd.OutputStream(
                    samplerate=self.voice.config.sample_rate,
                    channels=1,
                    dtype='int16',
                    blocksize=1024
                )
                self.stream.start()
                logger.info(f"Piper voice loaded: {model_path}")
            else:
                logger.warning(f"Piper model not found: {model_path}")
        except Exception as e:
            logger.error(f"Failed to initialize Piper voice: {e}")

    def speak(self, text: str) -> bool:
        """Speak text using Piper TTS"""
        if not PIPER_AVAILABLE or not self.voice:
            logger.warning("Piper TTS not available")
            return False
            
        if not text.strip():
            return False
            
        try:
            self.is_speaking = True
            for audio_bytes in self.voice.synthesize_stream_raw(text):
                chunk = np.frombuffer(audio_bytes, dtype=np.int16)
                self.stream.write(chunk)
            self.is_speaking = False
            return True
        except Exception as e:
            logger.error(f"Error during speech synthesis: {e}")
            self.is_speaking = False
            return False

    def stop_speech(self):
        """Stop current speech"""
        self.is_speaking = False
        logger.info("Speech stopped")

# Initialize TTS engine
tts_engine = TTSEngine()

@mcp.tool()
def speak_text(text: str) -> dict:
    """
    Convert text to speech using Piper TTS and play it through speakers.
    
    Args:
        text: The text to convert to speech
        
    Returns:
        Dictionary with success status and message
    """
    if not text or not text.strip():
        return {"success": False, "message": "Empty text provided"}
    
    if not PIPER_AVAILABLE:
        return {
            "success": False, 
            "message": "Piper TTS not available. Install with: pip install piper-tts"
        }
    
    success = tts_engine.speak(text.strip())
    return {
        "success": success,
        "message": f"Spoken: {text[:50]}..." if success else "Failed to speak text"
    }

@mcp.tool()
def stop_speaking() -> dict:
    """
    Stop current speech (best effort).
    
    Returns:
        Dictionary with success status
    """
    tts_engine.stop_speech()
    return {"success": True, "message": "Speech stopped"}

@mcp.tool()
def play_sound_effect(effect: str) -> dict:
    """
    Play a predefined sound effect or notification sound using TTS.
    
    Args:
        effect: Name of the sound effect ("beep", "chime", "notification", "error", "success", "thinking")
        
    Returns:
        Dictionary with success status
    """
    sound_texts = {
        "beep": "*beep*",
        "chime": "*ding*", 
        "notification": "*attention*",
        "error": "*error*",
        "success": "*confirmed*",
        "thinking": "*thinking*"
    }
    
    if effect not in sound_texts:
        return {
            "success": False,
            "message": f"Unknown sound effect: {effect}",
            "available_effects": list(sound_texts.keys())
        }
    
    success = tts_engine.speak(sound_texts[effect])
    return {
        "success": success,
        "message": f"Played sound effect: {effect}",
        "effect": effect
    }

@mcp.tool()
def get_tts_status() -> dict:
    """
    Get TTS engine status.
    
    Returns:
        Dictionary with TTS status information
    """
    return {
        "piper_available": PIPER_AVAILABLE,
        "voice_loaded": tts_engine.voice is not None,
        "is_speaking": tts_engine.is_speaking,
        "model": DEFAULT_MODEL,
        "models_dir": MODELS_DIR
    }

@mcp.tool()
def set_volume(volume: float) -> dict:
    """
    Set TTS volume (placeholder - actual implementation would depend on system).
    
    Args:
        volume: Volume level (0.0 to 1.0)
        
    Returns:
        Dictionary with success status
    """
    if not 0.0 <= volume <= 1.0:
        return {
            "success": False,
            "message": "Volume must be between 0.0 and 1.0"
        }
    
    # This is a placeholder - actual volume control would need system integration
    return {
        "success": True,
        "message": f"Volume set to {volume:.1f} (placeholder implementation)",
        "volume": volume
    }

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Piper TTS MCP Server")
    parser.add_argument("--host", default="127.0.0.1", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8001, help="Port to bind to")
    
    args = parser.parse_args()
    
    try:
        logger.info(f"Starting Piper TTS MCP Server on {args.host}:{args.port}")
        logger.info(f"Piper TTS available: {PIPER_AVAILABLE}")
        mcp.run(transport="sse", host=args.host, port=args.port)
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")