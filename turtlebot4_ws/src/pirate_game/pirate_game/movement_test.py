#!/usr/bin/env python3
"""
Test script for robot movement using cmd_vel.
Moves forward for 2 seconds, turns around, and returns to starting point.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MovementTest(Node):
    """Test movement controller using cmd_vel."""
    
    def __init__(self):
        super().__init__('movement_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        self.get_logger().info("Movement test node initialized")
    
    def stop(self):
        """Stop the robot immediately."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Robot stopped")
    
    def move_forward(self, duration, speed=0.2):
        """
        Move forward for a specified duration.
        
        Args:
            duration: Time to move forward in seconds
            speed: Speed in m/s (default: 0.2 m/s)
        """
        self.get_logger().info(f"Moving forward for {duration} seconds at {speed}m/s...")
        
        twist = Twist()
        twist.linear.x = float(speed)
        
        start_time = time.time()
        sleep_interval = 0.1  # 10 Hz (publish every 0.1 seconds)
        
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(sleep_interval)
        
        self.stop()
        self.get_logger().info("Forward movement complete")
    
    def turn_around(self, angular_speed=0.5):
        """
        Turn the robot 180 degrees.
        
        Args:
            angular_speed: Angular speed in rad/s (default: 0.5 rad/s)
        """
        self.get_logger().info("Turning around (180 degrees)...")
        
        # 180 degrees = π radians
        # Time needed = angle / angular_speed
        turn_time = 3.14159 / angular_speed  # π / angular_speed
        
        twist = Twist()
        twist.angular.z = float(angular_speed)
        
        start_time = time.time()
        sleep_interval = 0.1  # 10 Hz (publish every 0.1 seconds)
        
        while (time.time() - start_time) < turn_time:
            self.publisher.publish(twist)
            time.sleep(sleep_interval)
        
        self.stop()
        self.get_logger().info("Turn complete")
    
    def run_test(self):
        """Run the complete movement test sequence."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting Movement Test")
        self.get_logger().info("=" * 60)
        
        try:
            # Step 1: Move forward for 2 seconds
            self.get_logger().info("\nStep 1: Moving forward for 2 seconds...")
            self.move_forward(2.0, speed=0.2)
            time.sleep(0.5)  # Brief pause
            
            # Step 2: Turn around (180 degrees)
            self.get_logger().info("\nStep 2: Turning around...")
            self.turn_around(angular_speed=0.5)
            time.sleep(0.5)  # Brief pause
            
            # Step 3: Move forward for 2 seconds (back to start)
            self.get_logger().info("\nStep 3: Returning to starting point (2 seconds)...")
            self.move_forward(2.0, speed=0.2)
            time.sleep(0.5)  # Brief pause
            
            # Step 4: Turn to face original direction
            self.get_logger().info("\nStep 4: Turning to face original direction...")
            self.turn_around(angular_speed=0.5)
            
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("Movement Test Complete!")
            self.get_logger().info("=" * 60)
            
        except Exception as e:
            self.get_logger().error(f"Error during movement test: {e}")
            self.stop()
            raise


def main(args=None):
    """Main entry point for the movement test."""
    rclpy.init(args=args)
    
    movement_test = MovementTest()
    
    try:
        movement_test.run_test()
    except KeyboardInterrupt:
        movement_test.get_logger().info("\nTest interrupted by user")
        movement_test.stop()
    except Exception as e:
        movement_test.get_logger().error(f"Fatal error: {e}")
        movement_test.stop()
    finally:
        movement_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

