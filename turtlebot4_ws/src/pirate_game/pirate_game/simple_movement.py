"""
Simple movement controller using cmd_vel (no lidar required).
Reusable class for robot movement in the pirate game.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class SimpleMovement(Node):
    """Simple movement controller using cmd_vel."""
    
    def __init__(self):
        super().__init__('simple_movement')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        self.get_logger().info("Simple movement controller initialized")
        self.island_distance = 1.5  # Fixed distance to islands in meters
        self.forward_speed = 0.5  # Default forward speed in m/s
        self.angular_speed = 2.0  # Default angular speed in rad/s
    
    def stop(self):
        """Stop the robot immediately."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Robot stopped")
    
    def move_forward(self, duration, speed=None):
        """
        Move forward for a specified duration.
        
        Args:
            duration: Time to move forward in seconds
            speed: Speed in m/s (default: self.forward_speed)
        """
        if speed is None:
            speed = self.forward_speed
        
        self.get_logger().info(f"Moving forward for {duration:.2f} seconds at {speed}m/s...")
        
        twist = Twist()
        twist.linear.x = float(speed)
        
        start_time = time.time()
        sleep_interval = 0.1  # 10 Hz (publish every 0.1 seconds)
        
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(sleep_interval)
        
        self.stop()
        self.get_logger().info("Forward movement complete")
    
    def turn(self, angle, angular_speed=None):
        """
        Turn the robot by a specified angle.
        
        Args:
            angle: Angle to turn in radians (positive = counterclockwise, negative = clockwise)
            angular_speed: Angular speed in rad/s (default: self.angular_speed)
        """
        if angular_speed is None:
            angular_speed = self.angular_speed
        
        angle_deg = math.degrees(angle)
        self.get_logger().info(f"Turning {angle_deg:.1f} degrees ({angle:.3f} radians)...")
        
        # Calculate time needed
        turn_time = abs(angle) / angular_speed
        
        twist = Twist()
        twist.angular.z = float(angular_speed) if angle >= 0 else float(-angular_speed)
        
        start_time = time.time()
        sleep_interval = 0.1  # 10 Hz (publish every 0.1 seconds)
        
        while (time.time() - start_time) < turn_time:
            self.publisher.publish(twist)
            time.sleep(sleep_interval)
        
        self.stop()
        self.get_logger().info("Turn complete")
    
    def go_to_island(self, island_id, position_angle):
        """
        Move to a specific island position.
        
        Args:
            island_id: Island ID (1-3)
            position_angle: Angle in radians from starting position (0 = forward)
        """
        self.get_logger().info(f"Navigating to Island {island_id}...")
        
        # Step 1: Turn to face island direction
        self.turn(position_angle)
        time.sleep(0.3)  # Brief pause
        
        # Step 2: Move forward to reach island
        duration = self.island_distance / self.forward_speed
        self.move_forward(duration, self.forward_speed)
        time.sleep(0.3)  # Brief pause
        
        self.get_logger().info(f"Arrived at Island {island_id}!")
    
    def return_to_start(self, position_angle):
        """
        Return to starting position and face original direction (0°).
        
        Args:
            position_angle: The angle (in radians) the robot is currently facing
                           (same as the island's position_angle)
        """
        self.get_logger().info("Returning to starting position...")
        
        # Step 1: Turn 180 degrees to face start
        # Robot is currently facing position_angle, need to face position_angle + π
        self.turn(math.pi)  # 180 degrees
        time.sleep(0.3)  # Brief pause
        
        # Step 2: Move forward to return to start
        duration = self.island_distance / self.forward_speed
        self.move_forward(duration, self.forward_speed)
        time.sleep(0.3)  # Brief pause
        
        # Step 3: Turn to face original direction (0°)
        # Robot is now at start, facing (position_angle + π)
        # Need to turn to face 0°, so calculate: 0 - (position_angle + π) = -position_angle - π
        # Then normalize to [-π, π] to get the shortest turn
        current_orientation = position_angle + math.pi
        target_orientation = 0.0
        turn_to_zero = target_orientation - current_orientation
        
        # Normalize to [-π, π] range to get shortest path
        while turn_to_zero > math.pi:
            turn_to_zero -= 2 * math.pi
        while turn_to_zero < -math.pi:
            turn_to_zero += 2 * math.pi
        
        # Add a small correction factor to account for accumulated errors
        # This helps compensate for small drift during movements
        # Use a small percentage (5%) of the turn angle as correction
        correction_factor = 1.05  # 5% overcorrection to account for drift
        turn_to_zero *= correction_factor
        
        self.turn(turn_to_zero)
        time.sleep(0.3)  # Brief pause
        
        self.get_logger().info("Returned to starting position!")

