from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к конфигурационному файлу
    config_path = PathJoinSubstitution([
        FindPackageShare('led_matrix_driver'),
        'config',
        'led_matrix_driver.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='led_matrix_driver',
            executable='led_matrix_driver',
            name='led_matrix_driver',
            output='screen',
            parameters=[config_path]
        )
    ])