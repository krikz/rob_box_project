#!/usr/bin/env python3
"""
Главный launch file для запуска Voice Assistant системы

Поддерживает opt-in MiniMax TTS (см. ADR-0003) через launch-аргумент
``provider:=minimax``. При включении секреты берутся из ENV
``MINIMAX_API_KEY`` / ``MINIMAX_GROUP_ID`` (docker-compose mount). Они
читаются нодой из ENV и не копируются в YAML или launch-логи.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rob_box_core import dialog_engine


def generate_launch_description():
    """Generate launch description для Voice Assistant.

    Issue #1004 fix (ADR-0004): per-node YAML files вместо одного
    монолитного ``voice_assistant.yaml``. Раньше каждый Node грузил общий
    YAML, и вложенные блоки ``<node>:`` / ``tts_node:`` создавали
    dotted-параметры ``<node>.llm_provider``, которые нода
    ``get_parameter("llm_provider")`` НЕ читала — параметр всегда был
    default. Теперь каждый Node грузит СВОЙ per-node файл
    (src/rob_box_voice/config/<node>.yaml), и ``get_parameter("foo")``
    находит foo напрямую.
    """

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

    # MiniMax TTS opt-in. The launch override selects the provider; credentials
    # stay in ENV and are read lazily by TTSNode, never copied into YAML/logs.
    provider_arg = DeclareLaunchArgument(
        'provider',
        default_value='yandex',
        choices=['yandex', 'silero', 'minimax'],
        description='TTS provider override for tts_node'
    )

    # Конфигурация
    config_dir = LaunchConfiguration('config_dir')
    namespace = LaunchConfiguration('namespace')

    # Per-node config paths (issue #1004 fix).
    audio_node_yaml = PathJoinSubstitution([config_dir, 'audio_node.yaml'])
    stt_node_yaml = PathJoinSubstitution([config_dir, 'stt_node.yaml'])
    tts_node_yaml = PathJoinSubstitution([config_dir, 'tts_node.yaml'])
    sound_node_yaml = PathJoinSubstitution([config_dir, 'sound_node.yaml'])
    led_node_yaml = PathJoinSubstitution([config_dir, 'led_node.yaml'])
    command_node_yaml = PathJoinSubstitution([config_dir, 'command_node.yaml'])

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

    # Нода диалога — dialog_node (v2): её строит dialog_engine.launch_nodes (OpaqueFunction ниже) из
    # config_dir/dialog_node.yaml. Нода v1 удалена в PR-15 (#3711, часть 15p).

    # === TTS Node (Yandex/Silero + opt-in MiniMax) ===
    # Issue #1004: tts_node.yaml первым, launch-override provider — чтобы
    # переключать TTS провайдера одной строкой без правки YAML.
    tts_node_args = dict(
        package='rob_box_voice',
        executable='tts_node',
        name='tts_node',
        namespace=namespace,
        parameters=[tts_node_yaml, {'provider': LaunchConfiguration('provider')}],
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

    # === Command Node (Phase 5: Command Recognition) ===
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

    # === Startup Greeting: в ноде диалога (issue #1003, 06.08; v2 — #3912) ===
    # Отдельная нода приветствия убрана — страдала гонкой с
    # инициализацией tts_node (фраза терялась). dialog_node сам
    # говорит приветствие через startup_greeting_sec / startup_greeting_text.

    # === Speaker ID Node (issue #1077 — голосовая биометрия resemblyzer) ===
    # Подписывается на /audio/speech_audio, вычисляет d-vector, идентифицирует
    # говорящего, публикует /voice/speaker/result (читает нода диалога).
    speaker_id_node = Node(
        package='rob_box_voice',
        executable='speaker_id_node',
        name='speaker_id_node',
        namespace=namespace,
        parameters=[PathJoinSubstitution([config_dir, 'speaker_id_node.yaml'])],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # === MCP Server (Agentive Tools Integration) ===
    # ⚠️ Workaround: Python entry points not recognized by ROS 2
    # Using ExecuteProcess instead of Node to run Python module directly
    mcp_server = ExecuteProcess(
        cmd=['python3', '-m', 'rob_box_mcp_tools.mcp_server', '--ros-args', '--log-level', 'info',
             '--params-file', PathJoinSubstitution([config_dir, 'mcp_server.yaml'])],
        output='screen',
        respawn=True,
        respawn_delay=5.0
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
        config_dir_arg,
        namespace_arg,
        provider_arg,
        audio_node,
        led_node,
        # ✅ dialog_node (v2) + tts_node (Silero/MiniMax opt-in); dialog_engine: v1 — ошибка (PR-15 #3711)
        OpaqueFunction(function=dialog_engine.launch_nodes, args=[tts_node_args]),
        stt_node,       # ✅ Phase 3: Vosk STT
        sound_node,     # ✅ Phase 4: Sound Effects
        command_node,   # ✅ Phase 5: Command Recognition
        speaker_id_node,  # ✅ Phase 6: Speaker ID (issue #1077, resemblyzer)
        mcp_server,     # ✅ MCP Tools: Agentive LLM Integration
    ])
