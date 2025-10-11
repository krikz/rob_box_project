"""
Launch file for Animation Player Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rob_box_animations')
    animations_dir = os.path.join(pkg_dir, 'animations')
    
    # Declare launch arguments
    animations_dir_arg = DeclareLaunchArgument(
        'animations_dir',
        default_value=animations_dir,
        description='Path to animations directory'
    )
    
    autostart_animation_arg = DeclareLaunchArgument(
        'autostart_animation',
        default_value='',
        description='Animation to start automatically (without .yaml extension)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop animations by default'
    )
    
    # Animation player node
    animation_player_node = Node(
        package='rob_box_animations',
        executable='animation_player_node.py',
        name='animation_player',
        output='screen',
        parameters=[{
            'animations_dir': LaunchConfiguration('animations_dir'),
            'autostart_animation': LaunchConfiguration('autostart_animation'),
            'loop': LaunchConfiguration('loop'),
        }],
        remappings=[
            ('/panel_image', '/panel_image'),
        ]
    )
    
    return LaunchDescription([
        animations_dir_arg,
        autostart_animation_arg,
        loop_arg,
        animation_player_node,
    ])
