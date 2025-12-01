"""
Launch file for TurtleBot4 Treasure Hunt Game
Starts the treasure hunt game node with configuration
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('turtlebot_treasure_hunt')
    config_file = os.path.join(pkg_dir, 'config', 'game_params.yaml')
    
    # Declare launch arguments
    num_rounds_arg = DeclareLaunchArgument(
        'num_rounds',
        default_value='10',
        description='Number of treasure hunt rounds to play'
    )
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.83',
        description='Target distance in meters (default: 6 feet)'
    )
    
    # Create launch description
    ld = LaunchDescription([
        num_rounds_arg,
        target_distance_arg,
    ])
    
    # Add treasure hunt game node
    treasure_hunt_node = Node(
        package='turtlebot_treasure_hunt',
        executable='treasure_hunt_game',
        name='treasure_hunt_game',
        parameters=[
            config_file,
            {
                'num_rounds': LaunchConfiguration('num_rounds'),
                'target_distance': LaunchConfiguration('target_distance'),
            }
        ],
        output='screen'
    )
    
    ld.add_action(treasure_hunt_node)
    
    return ld
