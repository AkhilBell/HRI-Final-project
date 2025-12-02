#!/usr/bin/env python3
"""
Simple test script for text-to-speech using espeak on Linux.
"""
import rclpy
from rclpy.node import Node
import subprocess


class TTSTest(Node):
    """Simple TTS test using espeak."""
    
    def __init__(self):
        super().__init__('tts_test')
        self.get_logger().info("TTS Test initialized")
    
    def speak(self, text):
        """Speak the given text using espeak."""
        try:
            subprocess.run(["espeak", text], check=True)
            self.get_logger().info(f"Spoke: {text}")
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"TTS error: {e}")
            return False
        except FileNotFoundError:
            self.get_logger().error("espeak not found! Install with: sudo apt-get install espeak")
            return False
    
    def run_test(self):
        """Run simple TTS test."""
        self.get_logger().info("Testing text-to-speech...")
        self.speak("Hello, this is a text to speech test.")
        self.get_logger().info("TTS test complete!")


def main(args=None):
    """Main entry point for the TTS test."""
    rclpy.init(args=args)
    
    tts_test = TTSTest()
    
    try:
        tts_test.run_test()
    except KeyboardInterrupt:
        tts_test.get_logger().info("\nTest interrupted by user")
    except Exception as e:
        tts_test.get_logger().error(f"Fatal error: {e}")
    finally:
        tts_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

