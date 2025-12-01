"""
Navigation Controller for TurtleBot4 Treasure Hunt Game
Handles movement to target position and back to start
"""
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import time


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.navigator = BasicNavigator()
        self.declare_parameter('target_distance', 1.83)  # 6 feet in meters
        self.target_distance = self.get_parameter('target_distance').value
        
    def go_to_target(self):
        """Navigate to the target position (6 feet away)"""
        self.get_logger().info(f'Navigating to target position ({self.target_distance}m away)')
        
        # Create target pose (straight ahead)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
   	target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = self.target_distance
        target_pose.pose.position.y = 0.0
        target_pose.pose.orientation.w = 1.0
        
        # Send goal
        self.navigator.goToPose(target_pose)
        
        # Wait for completion
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            time.sleep(0.1)
            if i % 50 == 0:
                self.get_logger().info('Navigating to target...')
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✓ Reached target position!')
            return True
        else:
            self.get_logger().error('✗ Navigation to target failed!')
            return False
    
    def return_to_start(self):
        """Navigate back to the start position"""
        self.get_logger().info('Returning to start position')
        
        # Create start pose
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.orientation.w = 1.0
        
        # Send goal
        self.navigator.goToPose(start_pose)
        
        # Wait for completion
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            time.sleep(0.1)
            if i % 50 == 0:
                self.get_logger().info('Returning to start...')
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✓ Returned to start position!')
            return True
        else:
            self.get_logger().error('✗ Return to start failed!')
            return False
