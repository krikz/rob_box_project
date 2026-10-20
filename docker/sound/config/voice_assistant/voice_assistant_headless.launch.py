#!/usr/bin/env python3
"""
Headless Voice Assistant Launch для Vision Pi
Лицо LED-матрицы здесь не запускается: face_node живёт в led-matrix на Vision Pi (ADR-0150 PR-3a)

Issue #1004 fix (ADR-0004): каждый Node грузит СВОЙ per-node YAML
(audio_node.yaml / tts_node.yaml / ...) из config_dir (по умолчанию —
src/rob_box_voice/config/ из образа; в docker-деплое оператор передаёт
config_dir:=/config/voice_assistant, см. start_voice_assistant.sh).
Докер-сборка копирует src/config/<node>.yaml в
install/rob_box_voice/share/rob_box_voice/config/<node>.yaml, поэтому
FindPackageShare('rob_box_voice').config отдаёт правильный путь.
Монолитного voice_assistant.yaml нет: вложенные <node>: секции создавали
dotted-имена, которые ноды не читали (issue #1004).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description для Voice Assistant (headless)"""

    # Аргументы
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value=PathJoinSubstitution([
            FindPackageShare('rob_box_voice'),
            'config',
        ]),
        description='Directory with per-node ROS2 config YAMLs'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace для всех нод'
    )

    # Конфигурация
    config_dir = LaunchConfiguration('config_dir')
    namespace = LaunchConfiguration('namespace')

    # Per-node config paths (issue #1004 fix).
    audio_node_yaml = PathJoinSubstitution([config_dir, 'audio_node.yaml'])
    led_node_yaml = PathJoinSubstitution([config_dir, 'led_node.yaml'])
    dialogue_node_yaml = PathJoinSubstitution([config_dir, 'dialogue_node.yaml'])
    tts_node_yaml = PathJoinSubstitution([config_dir, 'tts_node.yaml'])
    stt_node_yaml = PathJoinSubstitution([config_dir, 'stt_node.yaml'])
    sound_node_yaml = PathJoinSubstitution([config_dir, 'sound_node.yaml'])
    command_node_yaml = PathJoinSubstitution([config_dir, 'command_node.yaml'])
    speaker_id_node_yaml = PathJoinSubstitution([config_dir, 'speaker_id_node.yaml'])
    mcp_server_yaml = PathJoinSubstitution([config_dir, 'mcp_server.yaml'])

    # === Audio Node ===
    audio_node = Node(
        package='rob_box_voice',
        executable='audio_node',
        name='audio_node',
        namespace=namespace,
        parameters=[audio_node_yaml],
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
        parameters=[led_node_yaml],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Лицо LED-матрицы — face_node в контейнере led-matrix на Vision Pi (ADR-0150 PR-3a, #3689).

    # === Dialogue Node (Phase 2: DeepSeek streaming + accent_replacer) ===
    dialogue_node = Node(
        package='rob_box_voice',
        executable='dialogue_node',
        name='dialogue_node',
        namespace=namespace,
        parameters=[dialogue_node_yaml],
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
        parameters=[tts_node_yaml],
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
        parameters=[stt_node_yaml],
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
        parameters=[sound_node_yaml],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # === Command Node (Phase 5: Command recognition + Nav2) ===
    command_node = Node(
        package='rob_box_voice',
        executable='command_node',
        name='command_node',
        namespace=namespace,
        parameters=[command_node_yaml],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # === MCP Server (Model Context Protocol Tools) ===
    mcp_server = Node(
        package='rob_box_mcp_tools',
        executable='mcp_server',
        name='mcp_server',
        namespace=namespace,
        parameters=[mcp_server_yaml],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # === Startup Greeting: переехал в dialogue_node (issue #1003, 06.08) ===
    # Отдельная нода приветствия убрана — страдала гонкой с
    # инициализацией tts_node (фраза терялась). dialogue_node сам
    # говорит приветствие через startup_greeting_sec / startup_greeting_text.

    # === Speaker ID Node (issue #1077 — голосовая биометрия resemblyzer) ===
    speaker_id_node = Node(
        package='rob_box_voice',
        executable='speaker_id_node',
        name='speaker_id_node',
        namespace=namespace,
        parameters=[speaker_id_node_yaml],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        config_dir_arg,
        namespace_arg,
        audio_node,
        led_node,
        dialogue_node,
        tts_node,
        stt_node,
        sound_node,
        command_node,
        speaker_id_node,
        mcp_server
    ])
