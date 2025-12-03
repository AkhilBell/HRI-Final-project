#!/usr/bin/env python3
"""
Simple test script for speech-to-text using Google Speech Recognition.
"""
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import os
import sys
from contextlib import contextmanager


@contextmanager
def suppress_stderr():
    """Context manager to suppress stderr output (ALSA warnings)."""
    with open(os.devnull, "w") as devnull:
        old_stderr = sys.stderr
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stderr = old_stderr


class SpeechTest(Node):
    """Simple speech recognition test using Google Speech Recognition."""
    
    def __init__(self):
        super().__init__('speech_test')
        # Suppress ALSA errors during microphone initialization
        with suppress_stderr():
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
        self.get_logger().info("Speech test initialized")
        self._setup_microphone()
    
    def _setup_microphone(self):
        """Setup microphone and adjust for ambient noise."""
        try:
            self.get_logger().info("Adjusting microphone for ambient noise...")
            # Suppress ALSA errors during microphone setup
            with suppress_stderr():
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=1)
            self.get_logger().info("Microphone ready!")
        except Exception as e:
            self.get_logger().error(f"Microphone setup error: {e}")
            raise
    
    def listen(self, timeout=5):
        """
        Listen for speech and return recognized text.
        
        Args:
            timeout: Maximum time to wait for speech in seconds
            
        Returns:
            str: Recognized text, or None if no speech detected
        """
        try:
            self.get_logger().info(f"Listening for speech (timeout: {timeout}s)...")
            print(f"\n🎤 Listening... (speak now, timeout: {timeout}s)")
            
            # Suppress ALSA errors during listening
            with suppress_stderr():
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=timeout)
            
            self.get_logger().info("Processing speech...")
            print("Processing...")
            
            text = self.recognizer.recognize_google(audio).lower()
            return text
            
        except sr.WaitTimeoutError:
            self.get_logger().warn("No speech detected within timeout")
            print("⏱️  No speech detected (timeout)")
            return None
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
            print("❓ Could not understand audio")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Speech recognition service error: {e}")
            print(f"❌ Speech recognition service error: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            print(f"❌ Error: {e}")
            return None
    
    def run_test(self):
        """Run speech recognition test."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting Speech Recognition Test")
        self.get_logger().info("=" * 60)
        print("\n" + "=" * 60)
        print("🎤 SPEECH RECOGNITION TEST")
        print("=" * 60)
        print("This will listen for your speech and convert it to text.")
        print("Say something and it will be displayed below.")
        print("=" * 60 + "\n")
        
        try:
            while True:
                # Listen for speech
                text = self.listen(timeout=5)
                
                if text:
                    print(f"\n✅ Recognized: '{text}'")
                    self.get_logger().info(f"Recognized text: {text}")
                    
                    # Check for quit command
                    if "quit" in text or "exit" in text or "stop" in text:
                        print("\n👋 Exiting test...")
                        break
                else:
                    print("\n💡 Try speaking again, or say 'quit' to exit\n")
                
        except KeyboardInterrupt:
            print("\n\n👋 Test interrupted by user")
            self.get_logger().info("Test interrupted by user")


def main(args=None):
    """Main entry point for the speech test."""
    rclpy.init(args=args)
    
    speech_test = SpeechTest()
    
    try:
        speech_test.run_test()
    except KeyboardInterrupt:
        speech_test.get_logger().info("\nTest interrupted by user")
    except Exception as e:
        speech_test.get_logger().error(f"Fatal error: {e}")
    finally:
        speech_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

