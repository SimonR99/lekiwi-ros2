#!/usr/bin/env python3
"""
Advanced Speech Engine for Dino AI

Implements VAD (Voice Activity Detection), Whisper STT, and Piper TTS
based on the provided example with proper integration into Dino AI architecture.
"""

import os
import threading
import time
import queue
import json
import wave
import tempfile
import logging
from typing import Optional, Callable, Any
from pathlib import Path

import numpy as np

# Optional dependencies - graceful fallback if not available
try:
    import pyaudio
    import torch
    from faster_whisper import WhisperModel
    import sounddevice as sd
    from piper.voice import PiperVoice
    SPEECH_DEPS_AVAILABLE = True
except ImportError as e:
    SPEECH_DEPS_AVAILABLE = False
    missing_dep = str(e)

logger = logging.getLogger(__name__)

# Environment setup
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'


class VADListener:
    """Voice Activity Detection and Speech Recognition using Silero VAD + Whisper"""
    
    def __init__(self, 
                 transcription_callback: Callable[[str], None],
                 vad_threshold: float = 0.4,
                 silence_duration: float = 1.2,
                 buffer_duration: float = 0.65):
        
        if not SPEECH_DEPS_AVAILABLE:
            raise ImportError(f"Speech dependencies not available: {missing_dep}")
        
        # Audio settings
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 512
        
        # VAD settings
        self.vad_threshold = vad_threshold
        self.silence_duration = silence_duration
        self.buffer_duration = buffer_duration
        
        # Callback for transcribed text
        self.transcription_callback = transcription_callback
        
        # Audio processing
        self.audio_queue = queue.Queue()
        self.recording_data = []
        self.buffer_data = []
        self.running = False
        self.last_speech_time = 0
        
        # Models (loaded lazily)
        self.vad_model = None
        self.whisper_model = None
        self.pa = None
        self.stream = None
        self.process_thread = None
        
        logger.info("VAD Listener initialized")
    
    def _load_models(self):
        """Load VAD and Whisper models"""
        if self.vad_model is None:
            logger.info("Loading Silero VAD model...")
            self.vad_model, _ = torch.hub.load(
                repo_or_dir='snakers4/silero-vad',
                model='silero_vad',
                force_reload=False,
                onnx=False,
                verbose=False
            )
            self.vad_model.to('cpu')
            logger.info("VAD model loaded")
        
        if self.whisper_model is None:
            logger.info("Loading Whisper model...")
            self.whisper_model = WhisperModel('base', device='cpu', compute_type='int8')
            logger.info("Whisper model loaded")
    
    def start(self):
        """Start listening for speech"""
        if not SPEECH_DEPS_AVAILABLE:
            logger.error("Cannot start VAD: Speech dependencies not available")
            return False
        
        try:
            self._load_models()
            
            self.running = True
            self.pa = pyaudio.PyAudio()
            self.stream = self.pa.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=self._audio_callback
            )
            
            # Start processing thread
            self.process_thread = threading.Thread(target=self._process_audio)
            self.process_thread.daemon = True
            self.process_thread.start()
            
            logger.info("VAD Listener started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start VAD listener: {e}")
            return False
    
    def stop(self):
        """Stop listening"""
        self.running = False
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        if self.pa:
            self.pa.terminate()
        
        if self.process_thread:
            self.process_thread.join(timeout=2.0)
        
        logger.info("VAD Listener stopped")
    
    def _audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback for audio data"""
        self.audio_queue.put(in_data)
        return (in_data, pyaudio.paContinue)
    
    def _process_audio(self):
        """Process audio data for VAD and transcription"""
        while self.running:
            try:
                data = self.audio_queue.get(timeout=0.5)
                audio = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                
                # Run VAD
                speech_prob = self.vad_model(torch.from_numpy(audio), self.RATE).item()
                now = time.time()
                
                if speech_prob > self.vad_threshold:
                    # Speech detected
                    self.last_speech_time = now
                    if not self.recording_data:
                        logger.debug("Speech detected, listening...")
                        # Add buffered data to recording
                        self.recording_data.extend(self.buffer_data)
                        self.buffer_data = []
                    self.recording_data.append(data)
                else:
                    # No speech - check if we should transcribe
                    if self.recording_data and now - self.last_speech_time > self.silence_duration:
                        logger.debug("Speech ended, transcribing...")
                        self._transcribe()
                    else:
                        # Add to buffer for context
                        self.buffer_data.append(data)
                        # Limit buffer size
                        max_buffer_chunks = int(self.buffer_duration * self.RATE / self.CHUNK)
                        if len(self.buffer_data) > max_buffer_chunks:
                            self.buffer_data.pop(0)
                            
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Audio processing error: {e}")
    
    def _transcribe(self):
        """Transcribe recorded audio using Whisper"""
        if not self.recording_data:
            return
        
        try:
            # Combine audio data
            audio_bytes = b''.join(self.recording_data)
            self.recording_data = []
            
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name
                
                with wave.open(temp_path, 'wb') as wf:
                    wf.setnchannels(self.CHANNELS)
                    wf.setsampwidth(2)
                    wf.setframerate(self.RATE)
                    wf.writeframes(audio_bytes)
            
            # Transcribe with Whisper
            segments, _ = self.whisper_model.transcribe(
                temp_path, 
                language='en', 
                beam_size=5
            )
            
            text = ' '.join(seg.text for seg in segments).strip()
            
            # Clean up temp file
            os.unlink(temp_path)
            
            # Send transcription to callback
            if text:
                logger.debug(f"Transcribed: {text}")
                self.transcription_callback(text)
                
        except Exception as e:
            logger.error(f"Transcription error: {e}")


class PiperTTSEngine:
    """Text-to-Speech using Piper voice synthesis"""
    
    def __init__(self, model_path: str = "en_US-lessac-medium.onnx"):
        self.model_path = model_path
        self.voice = None
        self.stream = None
        self.speaking = False
        
        if not SPEECH_DEPS_AVAILABLE:
            logger.warning("Piper TTS not available: Speech dependencies missing")
            return
        
        self._load_voice()
        logger.info("Piper TTS Engine initialized")
    
    def _load_voice(self):
        """Load Piper voice model"""
        try:
            if os.path.exists(self.model_path):
                self.voice = PiperVoice.load(self.model_path)
                self.stream = sd.OutputStream(
                    samplerate=self.voice.config.sample_rate,
                    channels=1,
                    dtype='int16'
                )
                self.stream.start()
                logger.info(f"Piper voice loaded: {self.model_path}")
            else:
                logger.warning(f"Piper model not found: {self.model_path}")
        except Exception as e:
            logger.error(f"Failed to load Piper voice: {e}")
    
    def speak(self, text: str, pause_listener: Optional[Callable] = None, 
              resume_listener: Optional[Callable] = None) -> bool:
        """Speak text using Piper TTS"""
        if not self.voice or not text.strip():
            return False
        
        try:
            self.speaking = True
            
            # Pause listener if provided
            if pause_listener:
                pause_listener()
            
            # Synthesize and play audio
            for audio_bytes in self.voice.synthesize_stream_raw(text):
                if not self.speaking:  # Allow interruption
                    break
                chunk = np.frombuffer(audio_bytes, dtype=np.int16)
                self.stream.write(chunk)
            
            # Resume listener if provided
            if resume_listener:
                resume_listener()
            
            self.speaking = False
            return True
            
        except Exception as e:
            logger.error(f"TTS error: {e}")
            self.speaking = False
            return False
    
    def stop(self):
        """Stop current speech"""
        self.speaking = False
    
    def is_speaking(self) -> bool:
        """Check if currently speaking"""
        return self.speaking
    
    def shutdown(self):
        """Shutdown TTS engine"""
        self.stop()
        if self.stream:
            self.stream.stop()
            self.stream.close()


class AdvancedSpeechEngine:
    """
    Complete speech engine combining VAD, Whisper STT, and Piper TTS
    Integrated with Dino AI architecture
    """
    
    def __init__(self, 
                 message_callback: Callable[[str], Any],
                 model_path: str = "en_US-lessac-medium.onnx"):
        
        self.message_callback = message_callback
        self.vad_listener = None
        self.tts_engine = None
        self.running = False
        
        if not SPEECH_DEPS_AVAILABLE:
            logger.warning(f"Advanced Speech Engine not available: {missing_dep}")
            logger.warning("Install with: pip install pyaudio torch faster-whisper sounddevice piper-tts")
            return
        
        # Initialize components
        self.tts_engine = PiperTTSEngine(model_path)
        self.vad_listener = VADListener(self._handle_transcription)
        
        logger.info("Advanced Speech Engine initialized")
    
    def _handle_transcription(self, text: str):
        """Handle transcribed speech"""
        if text.lower() in ('exit', 'quit', 'goodbye', 'stop'):
            logger.info("Stop command received")
            self.stop()
            return
        
        logger.info(f"Speech input: {text}")
        
        # Send to Dino AI for processing
        try:
            self.message_callback(text)
        except Exception as e:
            logger.error(f"Error processing speech message: {e}")
    
    def start_listening(self) -> bool:
        """Start voice activity detection and listening"""
        if not SPEECH_DEPS_AVAILABLE:
            logger.error("Cannot start listening: Speech dependencies not available")
            return False
        
        if self.vad_listener:
            success = self.vad_listener.start()
            if success:
                self.running = True
                logger.info("🎤 Listening for speech...")
            return success
        return False
    
    def stop_listening(self):
        """Stop listening"""
        if self.vad_listener:
            self.vad_listener.stop()
        self.running = False
        logger.info("🎤 Stopped listening")
    
    def speak(self, text: str) -> bool:
        """Speak text with proper listener coordination"""
        if not self.tts_engine:
            return False
        
        # Pause listener during speech
        pause_func = None
        resume_func = None
        
        if self.vad_listener and self.running:
            def pause_listener():
                try:
                    if self.vad_listener.stream:
                        self.vad_listener.stream.stop_stream()
                except:
                    pass
            
            def resume_listener():
                try:
                    if self.vad_listener.stream:
                        self.vad_listener.stream.start_stream()
                except:
                    pass
            
            pause_func = pause_listener
            resume_func = resume_listener
        
        return self.tts_engine.speak(text, pause_func, resume_func)
    
    def is_available(self) -> bool:
        """Check if speech engine is available"""
        return SPEECH_DEPS_AVAILABLE and self.tts_engine is not None
    
    def is_listening(self) -> bool:
        """Check if currently listening"""
        return self.running
    
    def is_speaking(self) -> bool:
        """Check if currently speaking"""
        return self.tts_engine and self.tts_engine.is_speaking()
    
    def stop(self):
        """Stop all speech operations"""
        self.stop_listening()
        if self.tts_engine:
            self.tts_engine.stop()
    
    def shutdown(self):
        """Shutdown speech engine"""
        self.stop()
        if self.tts_engine:
            self.tts_engine.shutdown()
        logger.info("Advanced Speech Engine shutdown")


# Factory function for easy integration
def create_speech_engine(message_callback: Callable[[str], Any], 
                        model_path: str = "en_US-lessac-medium.onnx") -> AdvancedSpeechEngine:
    """Create and return an Advanced Speech Engine"""
    return AdvancedSpeechEngine(message_callback, model_path)