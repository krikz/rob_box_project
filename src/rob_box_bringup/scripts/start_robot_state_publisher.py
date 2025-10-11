#!/usr/bin/env python3
"""
Простой запуск robot_state_publisher для Rob Box
"""
import rclpy
from rclpy.node import Node
import subprocess
import os
import sys

def main():
    # Получаем путь к URDF
    xacro_file = os.path.join(
        os.path.dirname(__file__),
        '../../rob_box_description/urdf/rob_box_complete.xacro'
    )
    
    # Генерируем URDF из xacro
    print("🔧 Processing xacro file...")
    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"❌ ERROR: Failed to process xacro file: {result.stderr}")
        return 1
    
    robot_description = result.stdout
    
    print(f"✅ Generated URDF")
    print(f"📏 URDF size: {len(robot_description)} bytes")
    
    # Инициализируем ROS2
    print("\n🚀 Starting robot_state_publisher...")
    rclpy.init()
    
    # Создаем ноду
    node = rclpy.create_node('robot_description_publisher')
    
    # Устанавливаем параметр robot_description
    node.declare_parameter('robot_description', robot_description)
    
    print("✅ robot_description parameter set!")
    print("\nNow start robot_state_publisher in another terminal:")
    print("  ros2 run robot_state_publisher robot_state_publisher")
    print("\nOr just use this same node to publish...")
    
    # Запускаем robot_state_publisher как subprocess
    process = subprocess.Popen([
        'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
        '--ros-args',
        '--params-file', '/dev/stdin'
    ], stdin=subprocess.PIPE, text=True)
    
    # Отправляем YAML конфигурацию
    yaml_config = f"""
robot_state_publisher:
  ros__parameters:
    robot_description: |
{chr(10).join('      ' + line for line in robot_description.split(chr(10)))}
"""
    
    process.communicate(input=yaml_config)
    
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
