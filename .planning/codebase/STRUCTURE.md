# Codebase Structure

**Analysis Date:** 2026-05-15

## Directory Layout

```
rob_box_project/
├── src/                          # ROS 2 Python/C++ packages (source of truth for robot code)
│   ├── rob_box_voice/            # Voice assistant: audio capture, STT, LLM dialogue, TTS
│   ├── rob_box_perception/       # Perception: context aggregator, reflection agent, health monitor
│   ├── rob_box_mcp_tools/        # MCP server + LLM tool-call bridge (shared lib)
│   ├── rob_box_animations/       # LED matrix animation player (JSON-based)
│   ├── rob_box_bringup/          # Launch files for complete system startup
│   ├── rob_box_teleop/           # Teleoperation node (keyboard/joystick)
│   ├── rob_box_telegram/         # Telegram bot operator interface
│   ├── rob_box_description/      # URDF robot model and meshes
│   ├── rob_box_perception_msgs/  # Custom ROS 2 messages (PerceptionEvent)
│   ├── robot_sensor_hub_msg/     # Custom messages for ESP32 sensor hub (DeviceSnapshot)
│   ├── vesc_nexus/               # VESC motor controller ros2_control plugin (C++, git submodule)
│   │   └── src/
│   │       ├── vesc_nexus/       # hardware_interface plugin source
│   │       └── vesc_msgs/        # VESC-specific ROS 2 messages
│   ├── led_matrix_driver/        # Low-level LED matrix hardware driver
│   └── ros2leds/                 # LED matrix compositor library (git submodule)
│       ├── led_matrix_compositor/
│       └── led_matrix_driver/
├── docker/                       # Docker infrastructure for both Pi nodes
│   ├── base/                     # Base Docker images (ros2-zenoh, rtabmap, depthai, pcl)
│   │   ├── Dockerfile.ros2-zenoh
│   │   ├── Dockerfile.rtabmap
│   │   ├── Dockerfile.depthai
│   │   └── Dockerfile.pcl
│   ├── main/                     # Main Pi (10.1.1.10) Docker Compose stack
│   │   ├── docker-compose.yaml   # All Main Pi services
│   │   ├── config/               # Configs mounted as volumes (NEVER COPY in Dockerfile)
│   │   ├── scripts/              # Startup scripts mounted as volumes
│   │   ├── lslidar/              # LiDAR service Dockerfile
│   │   ├── rtabmap/              # RTAB-Map service Dockerfile
│   │   ├── ros2_control/         # Motor control Dockerfile
│   │   ├── nav2/                 # Nav2 Dockerfile
│   │   ├── twist_mux/            # Velocity mux Dockerfile
│   │   ├── perception/           # Perception container Dockerfile
│   │   ├── robot_state_publisher/
│   │   ├── vesc_nexus/
│   │   ├── teleop/
│   │   └── zenoh-router/
│   ├── vision/                   # Vision Pi (10.1.1.11) Docker Compose stack
│   │   ├── docker-compose.yaml   # All Vision Pi services
│   │   ├── config/               # Configs mounted as volumes
│   │   ├── scripts/              # Startup scripts mounted as volumes
│   │   ├── oak-d/                # OAK-D camera + AprilTag Dockerfile
│   │   ├── led_matrix/           # LED matrix Dockerfile
│   │   ├── ceiling-camera/       # USB camera Dockerfile
│   │   ├── voice_assistant/      # Voice assistant Dockerfile
│   │   ├── voice_base/           # Base voice image Dockerfile
│   │   ├── voice_resources/      # Sound resources init container
│   │   ├── supercollider/        # SuperCollider audio server Dockerfile
│   │   ├── telegram_bot/         # Telegram bot Dockerfile
│   │   ├── apriltag/             # (legacy, now integrated in oak-d)
│   │   ├── cache/                # TTS cache (persistent)
│   │   └── zenoh-router/
│   ├── build/                    # Build scripts / CI helpers
│   ├── monitoring/               # Grafana + Prometheus + Loki monitoring stack
│   └── scripts/                  # Shared deployment scripts
├── docs/                         # Project documentation
│   ├── architecture/             # System architecture docs (SYSTEM_OVERVIEW.md, etc.)
│   ├── development/              # Dev guides (DOCKER_STANDARDS.md, PYTHON_STYLE_GUIDE.md)
│   ├── deployment/               # Deployment workflow docs
│   ├── guides/                   # Troubleshooting, monitoring quick ref
│   ├── fixes/                    # Post-mortem fix summaries
│   ├── reports/                  # Analysis/research reports
│   └── plans/                    # Implementation plans
├── .planning/                    # GSD planning artifacts
│   └── codebase/                 # Codebase map documents (this file)
├── .agents/                      # Agent skills and Claude commands
│   ├── skills/                   # Domain-specific skill guides (SKILL.md per skill)
│   └── ...
├── .github/                      # GitHub Actions CI/CD workflows
│   ├── workflows/                # Build, test, deploy pipelines
│   └── copilot-instructions.md   # Copilot project context
├── build/                        # colcon build output (COLCON_IGNORE present)
├── install/                      # colcon install output (COLCON_IGNORE present)
├── local_test/                   # Local test scripts and reference data
├── migrations/                   # SQLite DB migrations for VoiceMemory etc.
├── sound_pack/                   # Sound effect files (WAV/MP3)
├── scripts/                      # General project scripts
├── host/                         # Host machine setup (main Pi specific)
├── tools/                        # Dev tooling
├── tasks.json                    # Task backlog with acceptance criteria
├── ROADMAP.md                    # Development roadmap
└── coverage.json                 # Test coverage report
```

## Directory Purposes

**`src/rob_box_voice/rob_box_voice/`:**
- Purpose: All voice assistant ROS 2 nodes
- Key files:
  - `audio_node.py` — ReSpeaker capture → `/audio/speech_audio`
  - `stt_node.py` — Yandex gRPC STT + Vosk fallback → `/voice/stt/result`
  - `dialogue_node.py` — LLM dialogue agent (OpenAI Agents SDK) → `/voice/dialogue/response`
  - `tts_node.py` — Yandex TTS gRPC + Silero fallback → audio playback
  - `command_node.py` — Intent classifier → NavigateToPose action
  - `sound_node.py` — Sound effect player → `/voice/sound/trigger`
  - `led_node.py` — ReSpeaker ring LED controller
  - `core/` — DialogueManager, VoiceMemory, ConversationHistory, CommandParser, FAQStore
  - `llm/` — ProviderManager (Qwen/DeepSeek/OpenAI), StreamingHandler, ToolCallExecutor
  - `skills/` — BaseSkill, FAQSkill, MemorySkill, MusicSkill, NavigationSkill, StatusSkill
  - `utils/` — audio_utils.py, respeaker_interface.py

**`src/rob_box_perception/rob_box_perception/`:**
- Purpose: Perception pipeline — data aggregation and robot cognition
- Key files:
  - `context_aggregator_node.py` — subscribes all sensor topics → `/perception/context_update` (PerceptionEvent)
  - `reflection_node.py` — event-driven internal dialogue agent → `/reflection/internal_thought`, `/voice/tts/request`
  - `health_monitor.py` — monitors `/rosout` errors → `/voice/sound/trigger`
  - `startup_greeting_node.py` — plays startup sound/greeting
  - `core/` — EventDetector, MemoryManager, PromptFormatter
  - `utils/` — NodeAvailabilityMonitor, InternetConnectivityMonitor, TimeAwarenessProvider

**`src/rob_box_mcp_tools/rob_box_mcp_tools/`:**
- Purpose: MCP (Model Context Protocol) server exposing robot capabilities as LLM tools
- Key files:
  - `mcp_server.py` — ROS 2 node; `/mcp/tools`, `/mcp/execute`, `/mcp/result`
  - `llm_adapter.py` — LLMToolCallAdapter bridges OpenAI tool_calls to MCP pub/sub
  - `registry.py` — MCPToolRegistry
  - `waypoint_store.py` — SQLite waypoint persistence
  - `tools/` — navigation.py, animation.py, sound.py, perception.py, memory.py, mapping.py, music.py, system.py, dialogue.py

**`src/rob_box_animations/rob_box_animations/`:**
- Purpose: LED matrix animation engine
- Key files:
  - `animation_loader.py` — loads JSON animation files
  - `animation_player.py` — drives frame rendering loop
  - `frame_renderer.py` — renders frames to LED hardware

**`src/rob_box_animations/scripts/`:**
- Key files: `animation_player_node.py` — ROS 2 node entry point; subscribes `~/load_animation`

**`src/vesc_nexus/src/vesc_nexus/`:**
- Purpose: C++ ros2_control hardware_interface plugin for VESC motor controllers via CAN
- Key files: `include/`, `src/`, `vesc_nexus_plugins.xml`

**`src/rob_box_telegram/rob_box_telegram/`:**
- Purpose: Telegram bot bridging operator commands to robot
- Key files: `telegram_node.py`, `llm_chat.py`, `mcp_bridge.py`, `voice_processor.py`, `camera_cache.py`, `handlers/`

**`docker/main/config/`:**
- Purpose: All Main Pi runtime configuration (mounted read-only)
- Key files: `zenoh_router_config.json5`, `zenoh_session_config.json5`, `controllers/controller_manager.yaml`, `nav2/`, `rtabmap/rtabmap.ini`, `lslidar/`, `twist_mux/`, `perception/`

**`docker/vision/config/`:**
- Purpose: All Vision Pi runtime configuration (mounted read-only)
- Key files: `zenoh_router_config.json5`, `zenoh_session_config.json5`, `oak-d/`, `audio/asound.conf`, `led_matrix/`, `voice_assistant/`, `telegram_bot/`

**`migrations/`:**
- Purpose: SQL migration files applied idempotently at container startup (VoiceMemory, WaypointStore, TrackLibrary)
- Committed: Yes — mounted into voice-assistant container

**`sound_pack/`:**
- Purpose: Sound effect WAV/MP3 files for SoundNode
- Mounted read-only into voice-assistant container

## Key File Locations

**Entry Points:**
- `docker/main/docker-compose.yaml` — Main Pi full stack startup
- `docker/vision/docker-compose.yaml` — Vision Pi full stack startup
- `src/rob_box_voice/launch/voice_assistant.launch.py` — Voice assistant standalone launch
- `src/rob_box_bringup/launch/complete_system.launch.py` — Dev complete system launch
- `src/rob_box_bringup/launch/display.launch.py` — URDF visualization in RViz

**ROS 2 Node Entry Points (setup.py console_scripts):**
- `audio_node` → `src/rob_box_voice/rob_box_voice/audio_node.py`
- `stt_node` → `src/rob_box_voice/rob_box_voice/stt_node.py`
- `dialogue_node` → `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- `tts_node` → `src/rob_box_voice/rob_box_voice/tts_node.py`
- `command_node` → `src/rob_box_voice/rob_box_voice/command_node.py`
- `sound_node` → `src/rob_box_voice/rob_box_voice/sound_node.py`
- `led_node` → `src/rob_box_voice/rob_box_voice/led_node.py`
- `context_aggregator_node` → `src/rob_box_perception/rob_box_perception/context_aggregator_node.py`
- `reflection_node` → `src/rob_box_perception/rob_box_perception/reflection_node.py`
- `health_monitor` → `src/rob_box_perception/rob_box_perception/health_monitor.py`
- `mcp_server` → `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py`
- `animation_player_node.py` → `src/rob_box_animations/scripts/animation_player_node.py`
- `telegram_node` → `src/rob_box_telegram/rob_box_telegram/telegram_node.py`

**Configuration:**
- `docker/main/config/zenoh_session_config.json5` — Zenoh session config for Main Pi nodes
- `docker/vision/config/zenoh_session_config.json5` — Zenoh session config for Vision Pi nodes
- `docker/main/config/controllers/controller_manager.yaml` — ros2_control VESC parameters
- `docker/main/config/nav2/` — Nav2 parameters
- `docker/main/config/rtabmap/rtabmap.ini` — RTAB-Map algorithm parameters
- `src/rob_box_description/urdf/rob_box.urdf` — Robot URDF model

**Custom Messages:**
- `src/rob_box_perception_msgs/` — `PerceptionEvent.msg`
- `src/robot_sensor_hub_msg/` — `DeviceSnapshot.msg`
- `src/vesc_nexus/src/vesc_msgs/` — VESC-specific messages

**Tests:**
- `src/rob_box_voice/test/` — pytest tests for voice nodes and LLM logic
- `src/rob_box_perception/test/` — pytest tests for perception nodes
- `src/rob_box_mcp_tools/test/` — pytest tests for MCP tools

## Naming Conventions

**Files:**
- ROS 2 Python nodes: `snake_case_node.py` (e.g., `dialogue_node.py`, `stt_node.py`)
- ROS 2 C++ packages: `snake_case/` with standard `include/`, `src/` layout
- Docker services: `kebab-case` (e.g., `voice-assistant`, `led-matrix`, `oak-d`)
- Config files: `snake_case.yaml` or `snake_case.json5`
- Animation files: `snake_case.json` in `src/rob_box_animations/animations/`

**Directories:**
- ROS 2 packages: `rob_box_<subsystem>/` (e.g., `rob_box_voice`, `rob_box_perception`)
- Docker service dirs: `docker/<pi>/<service>/` (e.g., `docker/vision/oak-d/`)
- Shared config/scripts in Docker: `docker/<pi>/config/`, `docker/<pi>/scripts/` (mounted)

**Python Classes:**
- ROS 2 nodes: `PascalCaseNode` (e.g., `DialogueNode`, `STTNode`, `ContextAggregatorNode`)
- MCP tools: `PascalCaseTool` (e.g., `NavigateToWaypointTool`, `PlayAnimationTool`)
- Skills: `PascalCaseSkill` (e.g., `FAQSkill`, `MusicSkill`)

**ROS 2 Topics:**
- `/voice/*` — voice assistant pipeline (STT, TTS, dialogue, commands)
- `/audio/*` — raw audio data (capture, VAD, direction)
- `/perception/*` — perception events and context
- `/mcp/*` — MCP tool execution bus
- `/reflection/*` — internal thought output
- `/camera/*` — OAK-D camera data
- `/device/*` — ESP32 sensor hub data
- `/voice/sound/trigger` — sound effect requests

## Where to Add New Code

**New ROS 2 node (Python):**
- Implementation: `src/rob_box_<package>/rob_box_<package>/<new_node>.py`
- Register in: `src/rob_box_<package>/setup.py` under `console_scripts`
- Add to launch: `src/rob_box_<package>/launch/`
- Tests: `src/rob_box_<package>/test/test_<new_node>.py`

**New MCP tool (LLM capability):**
- Implementation: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/<category>.py`
- Register in: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/__init__.py`
- Import in: `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py`

**New voice assistant skill:**
- Implementation: `src/rob_box_voice/rob_box_voice/skills/<name>_skill.py` extending `BaseSkill`
- Register in: `src/rob_box_voice/rob_box_voice/skills/__init__.py`
- Import in: `src/rob_box_voice/rob_box_voice/dialogue_node.py`

**New Docker service on Main Pi:**
- Dockerfile: `docker/main/<service>/Dockerfile`
- Add service block to: `docker/main/docker-compose.yaml`
- Startup script: `docker/main/scripts/<service>/start_<service>.sh`
- Config: `docker/main/config/<service>/` (mounted as volume)

**New Docker service on Vision Pi:**
- Dockerfile: `docker/vision/<service>/Dockerfile`
- Add service block to: `docker/vision/docker-compose.yaml`
- Startup script: `docker/vision/scripts/<service>/start_<service>.sh`
- Config: `docker/vision/config/<service>/` (mounted as volume)

**New LED animation:**
- Add JSON animation file to: `src/rob_box_animations/animations/<name>.json`
- Send animation name to: `/animation_player/load_animation` topic

**New Zenoh config change:**
- Main Pi: `docker/main/config/zenoh_session_config.json5` or `zenoh_router_config.json5`
- Vision Pi: `docker/vision/config/zenoh_session_config.json5` or `zenoh_router_config.json5`

**New database migration:**
- Add SQL file to: `migrations/<NNN>_<description>.sql`
- Applied automatically at container startup

## Special Directories

**`build/` and `install/`:**
- Purpose: colcon build and install output
- Generated: Yes
- Committed: No (both have `COLCON_IGNORE`)

**`.planning/codebase/`:**
- Purpose: GSD codebase map documents used by planning agents
- Generated: Yes (by gsd-map-codebase)
- Committed: Yes

**`docker/main/maps/` (on Main Pi):**
- Purpose: Persistent RTAB-Map database (`rtabmap.db`) and saved maps
- Generated: Yes (at runtime)
- Committed: No

**`docker/vision/cache/tts/`:**
- Purpose: TTS audio cache (Yandex TTS synthesized files)
- Generated: Yes (at runtime)
- Committed: No

**`docker/vision/data/voice/`:**
- Purpose: VoiceMemory SQLite database (`voice_memory.db`)
- Generated: Yes (at runtime)
- Committed: No

**`src/vesc_nexus/` and `src/ros2leds/`:**
- Purpose: External git submodules for VESC motor controller and LED libraries
- Committed: Reference only (`.git` inside marks submodule boundary)

---

*Structure analysis: 2026-05-15*
