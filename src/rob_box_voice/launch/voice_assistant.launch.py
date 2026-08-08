#!/usr/bin/env python3
"""
Главный launch file для запуска Voice Assistant системы

Поддерживает opt-in MiniMax TTS (см. ADR-0003) через launch-аргумент
``provider:=minimax``. При включении секреты берутся из ENV
``MINIMAX_API_KEY`` / ``MINIMAX_GROUP_ID`` (docker-compose mount). Они
читаются нодой из ENV и не копируются в YAML или launch-логи.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description для Voice Assistant.

    Issue #1004 fix (ADR-0004): per-node YAML files вместо одного
    монолитного ``voice_assistant.yaml``. Раньше каждый Node грузил общий
    YAML, и вложенные блоки ``dialogue_node:`` / ``tts_node:`` создавали
    dotted-параметры ``dialogue_node.llm_provider``, которые нода
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

    # W5a: tool-provider backend selector. ``ros_mcp`` is the default
    # production path (LLMToolCallAdapter → ROSMCPToolProvider with the
    # 34-manifest ToolRegistry). ``fake`` swaps in FakeToolProvider for
    # smoke tests; ``none`` disables tools entirely (chat-only deploy).
    # The operator can also flip this in dialogue_node.yaml's
    # ``tool_provider`` key without re-launching the stack.
    tool_provider_arg = DeclareLaunchArgument(
        'tool_provider',
        default_value='ros_mcp',
        choices=['ros_mcp', 'fake', 'none'],
        description='Tool provider backend for dialogue_node (W5a)',
    )

    # Конфигурация
    config_dir = LaunchConfiguration('config_dir')
    namespace = LaunchConfiguration('namespace')

    # Per-node config paths (issue #1004 fix).
    audio_node_yaml = PathJoinSubstitution([config_dir, 'audio_node.yaml'])
    stt_node_yaml = PathJoinSubstitution([config_dir, 'stt_node.yaml'])
    tts_node_yaml = PathJoinSubstitution([config_dir, 'tts_node.yaml'])
    dialogue_node_yaml = PathJoinSubstitution([config_dir, 'dialogue_node.yaml'])
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

    # === Animation Player Node ===
    animation_node = Node(
        package='rob_box_animations',
        executable='animation_player_node.py',
        name='voice_animation_player',
        namespace=namespace,
        parameters=[{
            'animations_dir': '/ws/install/rob_box_animations/share/rob_box_animations/animations',
            'autostart_animation': 'idle',
            'loop': True
        }],
        output='screen',
        respawn=True,
        respawn_delay=3.0
    )

    # === Dialogue Node (Phase 2: DeepSeek streaming + accent_replacer) ===
    # Issue #1004: dialog_node.yaml грузится первым, потом launch-override
    # tool_provider (W5a) — чтобы можно было переключать без правки YAML.
    dialogue_node = Node(
        package='rob_box_voice',
        executable='dialogue_node',
        name='dialogue_node',
        namespace=namespace,
        parameters=[
            dialogue_node_yaml,
            {'tool_provider': LaunchConfiguration('tool_provider')},
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # === TTS Node (Yandex/Silero + opt-in MiniMax) ===
    # Issue #1004: tts_node.yaml первым, launch-override provider — чтобы
    # переключать TTS провайдера одной строкой без правки YAML.
    tts_node = Node(
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

    # === Startup Greeting: переехал в dialogue_node (issue #1003, 06.08) ===
    # Отдельная startup_greeting_node убрана — страдала гонкой с
    # инициализацией tts_node (фраза терялась). dialogue_node сам
    # говорит приветствие через startup_greeting_sec / startup_greeting_text.

    # === MCP Server (Agentive Tools Integration) ===
    # ⚠️ Workaround: Python entry points not recognized by ROS 2
    # Using ExecuteProcess instead of Node to run Python module directly
    mcp_server = ExecuteProcess(
        cmd=['python3', '-m', 'rob_box_mcp_tools.mcp_server', '--ros-args', '--log-level', 'info'],
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
        tool_provider_arg,
        audio_node,
        led_node,
        animation_node,
        dialogue_node,  # ✅ Phase 2: DeepSeek streaming
        tts_node,       # ✅ Phase 2: Silero TTS (MiniMax opt-in via provider=minimax)
        stt_node,       # ✅ Phase 3: Vosk STT
        sound_node,     # ✅ Phase 4: Sound Effects
        command_node,   # ✅ Phase 5: Command Recognition
        mcp_server,     # ✅ MCP Tools: Agentive LLM Integration
    ])
