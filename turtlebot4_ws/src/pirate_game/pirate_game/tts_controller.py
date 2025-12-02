"""
Text-to-Speech controller using espeak for Linux.
Simple utility class for speaking text throughout the game.
"""
import subprocess
import logging


class TTSController:
    """Simple TTS controller using espeak."""
    
    def __init__(self, logger=None):
        """
        Initialize TTS controller.
        
        Args:
            logger: Optional logger instance for error logging
        """
        self.logger = logger
        self._check_espeak()
    
    def _check_espeak(self):
        """Check if espeak is available."""
        try:
            subprocess.run(["which", "espeak"], check=True, capture_output=True)
            self.available = True
            if self.logger:
                self.logger.info("TTS controller initialized with espeak")
        except subprocess.CalledProcessError:
            self.available = False
            if self.logger:
                self.logger.warn("espeak not found! TTS will be disabled. Install with: sudo apt-get install espeak")
    
    def speak(self, text):
        """
        Speak the given text using espeak.
        
        Args:
            text: Text to speak
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.available:
            return False
        
        if not text or not text.strip():
            return False
        
        try:
            subprocess.run(["espeak", text], check=True, capture_output=True)
            if self.logger:
                self.logger.debug(f"Spoke: {text}")
            return True
        except subprocess.CalledProcessError as e:
            if self.logger:
                self.logger.error(f"TTS error: {e}")
            return False
        except FileNotFoundError:
            if self.logger:
                self.logger.error("espeak not found!")
            self.available = False
            return False

