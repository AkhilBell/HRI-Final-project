"""
Speech Recognition controller using Google Speech Recognition.
Reusable class for voice command recognition in the pirate game.
"""
import os
import sys
import speech_recognition as sr
from contextlib import contextmanager
import re


@contextmanager
def suppress_stderr():
    """Context manager to suppress stderr output (ALSA warnings) at file descriptor level."""
    # Save original stderr file descriptor
    original_stderr_fd = sys.stderr.fileno()
    # Create a duplicate of the original stderr
    saved_stderr_fd = os.dup(original_stderr_fd)
    
    try:
        # Redirect stderr to /dev/null at the file descriptor level
        # This catches C library writes that bypass Python's stderr
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        try:
            os.dup2(devnull_fd, original_stderr_fd)
            yield
        finally:
            # Restore original stderr immediately
            os.dup2(saved_stderr_fd, original_stderr_fd)
            os.close(devnull_fd)
    finally:
        os.close(saved_stderr_fd)


class SpeechRecognitionController:
    """Speech recognition controller using Google Speech Recognition."""
    
    def __init__(self, logger=None):
        """
        Initialize speech recognition controller.
        
        Args:
            logger: Optional logger instance for logging
        """
        self.logger = logger
        self.available = False
        
        try:
            # Suppress ALSA errors during microphone initialization
            with suppress_stderr():
                self.recognizer = sr.Recognizer()
                self.microphone = sr.Microphone()
            self._setup_microphone()
            self.available = True
            if self.logger:
                self.logger.info("Speech recognition controller initialized")
        except Exception as e:
            self.available = False
            if self.logger:
                self.logger.warn(f"Speech recognition not available: {e}")
    
    def _setup_microphone(self):
        """Setup microphone and adjust for ambient noise."""
        try:
            if self.logger:
                self.logger.debug("Adjusting microphone for ambient noise...")
            # Suppress ALSA errors during microphone setup
            with suppress_stderr():
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=1)
            if self.logger:
                self.logger.debug("Microphone ready!")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Microphone setup error: {e}")
            raise
    
    def listen(self, timeout=5):
        """
        Listen for speech and return recognized text.
        
        Args:
            timeout: Maximum time to wait for speech in seconds
            
        Returns:
            str: Recognized text (lowercase), or None if no speech detected
        """
        if not self.available:
            return None
        
        try:
            if self.logger:
                self.logger.debug(f"Listening for speech (timeout: {timeout}s)...")
            
            # Suppress ALSA errors during listening
            with suppress_stderr():
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=timeout)
            
            if self.logger:
                self.logger.debug("Processing speech...")
            
            text = self.recognizer.recognize_google(audio).lower()
            if self.logger:
                self.logger.info(f"Recognized: {text}")
            return text
            
        except sr.WaitTimeoutError:
            if self.logger:
                self.logger.debug("No speech detected within timeout")
            return None
        except sr.UnknownValueError:
            if self.logger:
                self.logger.debug("Could not understand audio")
            return None
        except sr.RequestError as e:
            if self.logger:
                self.logger.error(f"Speech recognition service error: {e}")
            return None
        except Exception as e:
            if self.logger:
                self.logger.error(f"Speech recognition error: {e}")
            return None
    
    def parse_island_command(self, text):
        """
        Parse voice command to extract island number (1-3).
        
        Args:
            text: Recognized speech text
            
        Returns:
            int: Island ID (1, 2, or 3), or None if invalid
        """
        if not text:
            return None
        
        text = text.lower().strip()
        
        # Direct number words
        if "one" in text and "two" not in text and "three" not in text:
            return 1
        if "two" in text and "three" not in text:
            return 2
        if "three" in text:
            return 3
        
        # Extract digits from text
        digits = re.findall(r'\d+', text)
        if digits:
            num = int(digits[0])
            if 1 <= num <= 3:
                return num
        
        # Check for "island" followed by number
        island_match = re.search(r'island\s+(one|two|three|\d+)', text)
        if island_match:
            island_word = island_match.group(1)
            if island_word == "one" or island_word == "1":
                return 1
            elif island_word == "two" or island_word == "2":
                return 2
            elif island_word == "three" or island_word == "3":
                return 3
        
        return None
    
    def parse_game_command(self, text):
        """
        Parse voice command for game control.
        
        Args:
            text: Recognized speech text
            
        Returns:
            str: Command ("continue", "reset", "quit"), or None if invalid
        """
        if not text:
            return None
        
        text = text.lower().strip()
        
        # Continue commands
        if any(word in text for word in ["continue", "c", "next", "play again", "again"]):
            return "continue"
        
        # Reset commands
        if any(word in text for word in ["reset", "r", "restart", "start over", "new game", "new"]):
            return "reset"
        
        # Quit commands
        if any(word in text for word in ["quit", "q", "exit", "stop", "end", "done"]):
            return "quit"
        
        return None

