#!/usr/bin/env python3
"""
Главный launch file для запуска Voice Assistant системы
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description для Voice Assistant"""
    
    # Аргументы
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rob_box_voice'),
            'config',
            'voice_assistant.yaml'
        ]),
        description='Path to voice assistant config YAML'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace для всех нод'
    )
    
    # Конфигурация
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    
    # === Audio Node ===
    audio_node = Node(
        package='rob_box_voice',
        executable='audio_node',
        name='audio_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === LED Node ===
    led_node = Node(
        package='rob_box_voice',
        executable='led_node',
        name='led_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === Animation Player Node ===
    animation_node = Node(
        package='rob_box_animations',
        executable='animation_player_node.py',
        name='voice_animation_player',
        namespace=namespace,
        parameters=[{
            'animations_path': '/ws/install/rob_box_animations/share/rob_box_animations/animations',
            'default_animation': 'idle_subtle',
            'autoplay': True
        }],
        output='screen',
        respawn=True,
        respawn_delay=3.0
    )
    
    # === Dialogue Node (Phase 2: DeepSeek streaming + accent_replacer) ===
    dialogue_node = Node(
        package='rob_box_voice',
        executable='dialogue_node',
        name='dialogue_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === TTS Node (Phase 2: Silero TTS v4 с бурундуком) ===
    tts_node = Node(
        package='rob_box_voice',
        executable='tts_node',
        name='tts_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === STT Node (Phase 3: Vosk offline recognition) ===
    stt_node = Node(
        package='rob_box_voice',
        executable='stt_node',
        name='stt_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
        # === Sound Node (Phase 4: Sound Effects) ===
    sound_node = Node(
        package='rob_box_voice',
        executable='sound_node',
        name='sound_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === Command Node (Phase 5: Command Recognition) ===
    command_node = Node(
        package='rob_box_voice',
        executable='command_node',
        name='command_node',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # === Command Node (Phase 5: TODO) ===
    # sound_node = Node(
    #     package='rob_box_voice',
    #     executable='sound_node',
    #     name='sound_node',
    #     namespace=namespace,
    #     parameters=[config_file],
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=3.0
    # )
    
    # === Command Node (Phase 5: TODO) ===
    # command_node = Node(
    #     package='rob_box_voice',
    #     executable='command_node',
    #     name='command_node',
    #     namespace=namespace,
    #     parameters=[config_file],
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=3.0
    # )
    
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        audio_node,
        led_node,
        animation_node,
        dialogue_node,  # ✅ Phase 2: DeepSeek streaming
        tts_node,       # ✅ Phase 2: Silero TTS
        stt_node,       # ✅ Phase 3: Vosk STT
        sound_node,     # ✅ Phase 4: Sound Effects
        command_node,   # ✅ Phase 5: Command Recognition
    ])

