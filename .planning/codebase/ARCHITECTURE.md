<!-- refreshed: 2026-05-15 -->
# Architecture

**Analysis Date:** 2026-05-15

## System Overview

```text
┌──────────────────────────────────────────────────────────────────────────────────┐
│                    CLOUD (optional) — zenoh.robbox.online:7447                   │
└──────────────────────────────────┬───────────────────────────────────────────────┘
                                   │ TCP/TLS (Zenoh)
┌──────────────────────────────────▼───────────────────────────────────────────────┐
│              MAIN PI (10.1.1.10) — Navigation, Control, SLAM                     │
│                                                                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  zenoh-router   │  │   rtabmap        │  │    nav2         │                  │
│  │  (peer mode)    │  │  LiDAR SLAM +   │  │  Nav stack       │                  │
│  │  :7447          │  │  ICP odometry   │  │  planner+ctrl    │                  │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘                  │
│           │                    │                     │                           │
│  ┌────────┴────────┐  ┌────────┴────────┐  ┌────────┴────────┐                  │
│  │   lslidar       │  │   ros2-control  │  │  twist-mux       │                  │
│  │   LS N10 2D    │  │   VESC via CAN  │  │  vel multiplexer │                  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  perception     │  │  robot-state-   │  │  micro-ros-agent │                  │
│  │  context_aggr + │  │  publisher      │  │  ESP32 bridge    │                  │
│  │  reflection     │  │  TF tree        │  │  UART /ttyUSB0   │                  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘                  │
└──────────────────────────────────┬───────────────────────────────────────────────┘
                                   │ GbE Ethernet 10.1.1.x (Zenoh UDP/TCP)
┌──────────────────────────────────▼───────────────────────────────────────────────┐
│              VISION PI (10.1.1.11) — Sensors, Voice, LED                         │
│                                                                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                  │
│  │ zenoh-router    │  │   oak-d          │  │  voice-assistant │                  │
│  │ (client mode)   │  │  OAK-D Lite +   │  │  audio+STT+LLM  │                  │
│  │                 │  │  AprilTag det.  │  │  +TTS+MCP       │                  │
│  └────────┬────────┘  └─────────────────┘  └─────────────────┘                  │
│           │                                                                      │
│  ┌────────┴────────┐  ┌─────────────────┐  ┌─────────────────┐                  │
│  │   led-matrix    │  │  ceiling-camera  │  │  telegram-bot   │                  │
│  │  381 NeoPixel   │  │  USB UVC 720p   │  │  operator iface  │                  │
│  │  SPI /dev/spi0  │  │  /dev/video0    │  │  Telegram API    │                  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘                  │
│  ┌─────────────────┐  ┌─────────────────┐                                        │
│  │  supercollider  │  │  voice-resources │                                        │
│  │  audio server   │  │  -init (one-shot)│                                        │
│  └─────────────────┘  └─────────────────┘                                        │
└──────────────────────────────────────────────────────────────────────────────────┘
```

## Component Responsibilities

| Component | Responsibility | Location |
|-----------|----------------|----------|
| `zenoh-router` (Main) | Central Zenoh peer router; bridges Vision Pi, cloud, dev machines | `docker/main/docker-compose.yaml` |
| `zenoh-router` (Vision) | Zenoh client router; connects to Main Pi :7447 | `docker/vision/docker-compose.yaml` |
| `rtabmap` | LiDAR-only SLAM (ICP odometry + 2D occupancy grid + loop closure) | `docker/main/docker-compose.yaml` |
| `lslidar` | LS LiDAR N10 driver — publishes `/scan` | `docker/main/lslidar/` |
| `ros2-control` | VESC motor controller via CAN; publishes `/odom` | `docker/main/ros2_control/` |
| `twist-mux` | Velocity command multiplexer with priority lanes | `docker/main/twist_mux/` |
| `nav2` | Nav2 autonomous navigation stack (planner + controller) | `docker/main/nav2/` |
| `robot-state-publisher` | TF tree from URDF; wraps with Zenoh namespace script | `docker/main/robot_state_publisher/` |
| `micro-ros-agent` | Bridge between ROS 2 and ESP32 (XRCE-DDS over UART) | `docker/main/micro_ros_agent/` |
| `perception` | context_aggregator + reflection_node (Internal Dialogue Agent) | `docker/main/perception/`, `src/rob_box_perception/` |
| `oak-d` | OAK-D Lite RGB-D driver + integrated AprilTag detection | `docker/vision/oak-d/` |
| `led-matrix` | 381 NeoPixel LED matrix; subscribes to animation requests | `docker/vision/led_matrix/`, `src/rob_box_animations/` |
| `ceiling-camera` | USB UVC ceiling-mounted camera for SLAM/localization | `docker/vision/ceiling-camera/` |
| `voice-assistant` | AudioNode + STT + DialogueNode (LLM) + TTS + MCP server | `docker/vision/voice_assistant/`, `src/rob_box_voice/` |
| `supercollider` | SuperCollider scsynth audio synthesis server for music | `docker/vision/supercollider/` |
| `telegram-bot` | Telegram operator interface (photos, voice, cmd_vel, LLM) | `docker/vision/telegram_bot/`, `src/rob_box_telegram/` |

## Pattern Overview

**Overall:** Distributed microservices robotics architecture — each functional subsystem is an isolated Docker container connected via ROS 2 topics over Zenoh DDS middleware.

**Key Characteristics:**
- Split compute: Main Pi owns navigation/control; Vision Pi owns sensing/interaction
- All containers use `network_mode: host` with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
- Zenoh peer (Main Pi) ↔ Zenoh client (Vision Pi) over GbE Ethernet only
- Configuration via mounted volumes (`./config:/config:ro`) — never `COPY config/` in Dockerfiles
- LLM integration via OpenAI-compatible API (Qwen/DeepSeek) + MCP tool-call bridge

## Layers

**Hardware Layer:**
- Purpose: Physical sensors and actuators
- Contains: LS LiDAR N10, OAK-D Lite, ReSpeaker Mic v2, VESC×2 (CAN), 381 NeoPixels (SPI), ESP32 hub

**Driver Layer:**
- Purpose: Hardware abstraction into ROS 2 topics
- Location: `docker/main/lslidar/`, `docker/vision/oak-d/`, `src/vesc_nexus/`
- Contains: lslidar_driver, depthai_ros (OAK-D), vesc_nexus (ros2_control hardware_interface), micro_ros_agent
- Depends on: Hardware devices

**Perception & Control Layer:**
- Purpose: SLAM, odometry, localization, motor control, sensor fusion
- Location: `docker/main/rtabmap/`, `docker/main/ros2_control/`, `src/vesc_nexus/src/vesc_nexus/`
- Contains: RTAB-Map (ICP odom + loop closure), ros2_control, twist_mux
- Depends on: Driver layer topics (`/scan`, `/odom`, `/camera/*`)

**Application Layer:**
- Purpose: Autonomous navigation, voice interaction, LED display
- Location: `docker/main/nav2/`, `src/rob_box_voice/`, `src/rob_box_animations/`
- Contains: Nav2 stack, voice pipeline, animations, telegram bot
- Depends on: Perception layer topics (`/map`, `/rtabmap/localization_pose`)

**LLM/AI Layer:**
- Purpose: Natural language dialogue, robot cognition, tool execution
- Location: `src/rob_box_voice/rob_box_voice/`, `src/rob_box_mcp_tools/`, `src/rob_box_perception/`
- Contains: DialogueNode (OpenAI Agents SDK), MCPServer, ReflectionNode, skills
- Depends on: All ROS 2 topics (via MCP tool adapters)

## Data Flow

### Navigation Flow

1. `lslidar` driver reads USB CDC ACM → publishes `/scan` (LaserScan, 10 Hz)
2. `ros2-control` (vesc_nexus plugin) reads CAN → publishes `/odom` (Odometry, 50 Hz)
3. `rtabmap` consumes `/scan` + `/odom` → ICP odometry → `/icp_odom` + `/map` (OccupancyGrid)
4. `nav2` planner consumes `/map` + `/rtabmap/localization_pose` → publishes `/cmd_vel` (Twist)
5. `twist-mux` selects highest-priority `/cmd_vel` source → outputs to `ros2-control`
6. `ros2-control` sends velocity setpoints to VESC controllers via CAN bus

### Voice Dialogue Flow

1. `AudioNode` captures 16kHz mono from ReSpeaker USB → `/audio/speech_audio` (AudioData)
2. `STTNode` runs Yandex STT gRPC v3 (primary) or Vosk (fallback) → `/voice/stt/result` (String)
3. `DialogueNode` sends text to Qwen/DeepSeek via OpenAI Agents SDK; LLM may call tools
4. Tool calls → `LLMToolCallAdapter` → `/mcp/execute` → `MCPServer` → ROS 2 actions → `/mcp/result`
5. `DialogueNode` publishes final response → `/voice/dialogue/response` (JSON+SSML)
6. `TTSNode` synthesizes via Yandex TTS gRPC (primary) or Silero (fallback) → plays via sounddevice

### Internal Reflection Flow

1. `ContextAggregatorNode` subscribes to `/rtabmap/localization_pose`, `/odom`, `/device/snapshot`, `/apriltag/detections`, `/rosout`, `/voice/stt/result`
2. Aggregates at 2 Hz → publishes `/perception/context_update` (PerceptionEvent) + `/perception/user_speech` (String)
3. `ReflectionNode` consumes events → calls DeepSeek/Qwen LLM for internal thought
4. Publishes `/reflection/internal_thought` (String) and `/voice/tts/request` (String) for proactive speech

### AprilTag Localization Flow

1. OAK-D container runs integrated AprilTag detection on `/camera/color/image_raw`
2. Publishes `/detections` (AprilTagDetectionArray) + TF transforms for detected tags
3. `rtabmap` subscribes to `apriltag_topic:=/detections` for loop closure anchor

## Key Abstractions

**MCPServer (`src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py`):**
- Purpose: Exposes robot capabilities as OpenAI-compatible function tools for LLM
- Pattern: Publishes tool definitions on `/mcp/tools`, executes via `/mcp/execute`, returns on `/mcp/result`
- Tools: `NavigateToWaypoint`, `MoveDirection`, `PlayAnimation`, `PlaySound`, `StartMapping`, `GetPerceptionContext`, `GetRobotStatus`, `SaveWaypoint`, and more

**LLMToolCallAdapter (`src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py`):**
- Purpose: Bridges OpenAI Agents SDK tool_calls to ROS 2 MCP pub/sub
- Pattern: Async executor with request_id correlation; uses `ReentrantCallbackGroup` to avoid spin deadlock

**Skills (`src/rob_box_voice/rob_box_voice/skills/`):**
- Purpose: Modular LLM skill plugins for DialogueNode
- Classes: `FAQSkill`, `MemorySkill`, `MusicSkill`, `NavigationSkill`, `StatusSkill` (all extend `BaseSkill`)
- Enabled via `USE_SKILLS=true` env var

**VoiceMemory (`src/rob_box_voice/rob_box_voice/core/voice_memory.py`):**
- Purpose: Persistent SQLite memory store for the voice assistant
- DB path: `/data/voice_memory.db` (Docker volume `./data/voice:/data`)

**AnimationPlayer (`src/rob_box_animations/rob_box_animations/animation_player.py`):**
- Purpose: Loads JSON animation files and renders frames to LED matrix
- Topic: `~/load_animation` (String) — subscribes to animation name requests

## Entry Points

**Main Pi bringup:**
- Location: `docker/main/docker-compose.yaml`
- Triggers: `docker compose up -d` on Main Pi
- Startup order: `zenoh-router` → all other services (all `depends_on: zenoh-router`)

**Vision Pi bringup:**
- Location: `docker/vision/docker-compose.yaml`
- Triggers: `docker compose up -d` on Vision Pi
- Startup order: `zenoh-router` → `supercollider` + `voice-resources-init` → `voice-assistant`; `zenoh-router` → `oak-d`, `led-matrix`, `ceiling-camera`, `telegram-bot`

**Voice assistant launch (standalone dev):**
- Location: `src/rob_box_voice/launch/voice_assistant.launch.py`
- Nodes started: `audio_node`, `led_node`, `voice_animation_player`, `dialogue_node`, `stt_node`, `tts_node`, `sound_node`, `command_node`

**Complete system launch (dev):**
- Location: `src/rob_box_bringup/launch/complete_system.launch.py`
- Includes: `robot_state_publisher`, `slam_toolbox`, `nav2`, `rob_box_control`

## ROS 2 Node Graph

### Main Pi Nodes

```
/lslidar_driver          → publishes /scan (LaserScan, 10Hz)
/rtabmap                 ← /scan, /odom, /imu, /detections, /tf
                         → /map, /rtabmap/localization_pose, /rtabmap/cloud_map, /icp_odom
/ros2_control_node       ← /diff_drive_controller/cmd_vel (after twist_mux)
                         → /odom, /joint_states, vesc CAN commands
/twist_mux               ← /cmd_vel_nav (nav2), /cmd_vel_joy, /cmd_vel_web, /cmd_vel_safe
                         → /diff_drive_controller/cmd_vel
/nav2_*                  ← /map, /rtabmap/localization_pose, /scan
                         → /cmd_vel_nav
/robot_state_publisher   → /tf, /tf_static (from URDF)
/micro_ros_agent         ← /device/command
                         → /device/snapshot (DeviceSnapshot)
/context_aggregator      ← /rtabmap/localization_pose, /odom, /device/snapshot,
                            /detections, /rosout, /voice/stt/result
                         → /perception/context_update (PerceptionEvent), /perception/user_speech
/reflection_node         ← /perception/context_update, /perception/user_speech
                         → /reflection/internal_thought, /voice/tts/request
/health_monitor          ← /rosout
                         → /voice/sound/trigger
```

### Vision Pi Nodes

```
/oak_d_node              → /camera/color/image_raw, /camera/depth/image_rect_raw,
                            /camera/color/camera_info, /camera/imu/data
/apriltag_node           ← /camera/color/image_raw, /camera/color/camera_info
                         → /detections (AprilTagDetectionArray), /tf (tag frames)
/audio_node              → /audio/speech_audio (AudioData), /audio/vad (Bool),
                            /audio/direction (Int32)
/stt_node                ← /audio/speech_audio
                         → /voice/stt/result (String)
/dialogue_node           ← /voice/stt/result, /audio/vad
                         → /voice/dialogue/response (String JSON+SSML),
                            /voice/dialogue/state (String)
                         → /mcp/execute (tool calls)
                         ← /mcp/result (tool results)
/tts_node                ← /voice/dialogue/response, /voice/tts/request
                         → audio playback via sounddevice (ReSpeaker)
/command_node            ← /voice/stt/result, /voice/dialogue/state
                         → /voice/command/intent, /voice/command/feedback
                         → NavigateToPose (action client)
/sound_node              ← /voice/sound/trigger
/led_node                ← /voice/state, /audio/direction, /voice/animation/request
/mcp_server              ← /mcp/execute
                         → /mcp/result, /mcp/tools
                         ← various ROS 2 topics (navigation, animation, etc.)
/animation_player        ← ~/load_animation (String)
                         → LED matrix via rob_box_animations library
/telegram_node           ← /camera/camera/color/image_raw/compressed,
                            /ceiling_camera/image_raw/compressed,
                            /rtabmap/grid_prob_map, /mcp/result, /mcp/tools
                         → /voice/tts/request, /cmd_vel_web, /mcp/execute
```

### Key Cross-Pi Topics (via Zenoh)

| Topic | Type | Flow | Hz |
|-------|------|------|----|
| `/scan` | `sensor_msgs/LaserScan` | Main→own rtabmap | 10 |
| `/odom` | `nav_msgs/Odometry` | Main→own rtabmap/nav2 | 50 |
| `/map` | `nav_msgs/OccupancyGrid` | Main→nav2, Vision telegram | 1 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2/Voice/Telegram → twist_mux | 10 |
| `/detections` | `apriltag_msgs/...` | Vision oak-d → Main rtabmap | 5 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Vision oak-d → telegram | 5 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Main robot-state-pub → all | 50 |
| `/device/snapshot` | `robot_sensor_hub_msg/DeviceSnapshot` | Main micro_ros → perception | 1 |
| `/mcp/execute` | `std_msgs/String` | Vision dialogue/telegram → mcp_server | on-demand |
| `/mcp/result` | `std_msgs/String` | mcp_server → dialogue/telegram | on-demand |
| `/voice/stt/result` | `std_msgs/String` | Vision stt → dialogue, command, context_agg | on speech |
| `/voice/tts/request` | `std_msgs/String` | reflection/telegram → tts_node | on demand |
| `/perception/context_update` | `rob_box_perception_msgs/PerceptionEvent` | Main context_agg → reflection | 2 |
| `/voice/sound/trigger` | `std_msgs/String` | health_monitor → sound_node | on event |

## Architectural Constraints

- **Zenoh config**: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — nodes only discover local router; cross-Pi discovery happens through router-to-router connection
- **Network isolation**: ROS 2 / Zenoh exclusively use Ethernet (`10.1.1.x`); WiFi is SSH-only
- **All containers**: Must use `network_mode: host` and `depends_on: zenoh-router`
- **Configuration files**: Mounted via `./config:/config:ro` volume — never `COPY` into image
- **Zenoh namespace**: `ROBOT_ID` env var injected by `ros_with_namespace.sh` to prefix topics for cloud publishing
- **VESC control**: `ros2-control` requires `privileged: true` for CAN device access
- **Threading**: `dialogue_node.py` uses `asyncio` + `ReentrantCallbackGroup` for concurrent LLM calls and ROS 2 spin
- **LLM API**: Qwen (primary, `QWEN_API_KEY`) → DeepSeek (fallback, `DEEPSEEK_API_KEY`) — both OpenAI-compatible via `provider_manager.py`

## Anti-Patterns

### Hardcoded config in Docker image
**What happens:** Using `COPY config/` or `COPY scripts/` in a Dockerfile  
**Why it's wrong:** Config changes require full image rebuild and re-deploy  
**Do this instead:** Mount via `volumes: - ./config:/config:ro` (see `docker/main/docker-compose.yaml`)

### Spinning in dialogue_node without ReentrantCallbackGroup
**What happens:** Blocking `execute_tool_call_sync()` while waiting for `/mcp/result`  
**Why it's wrong:** ROS 2 spin is blocked; MCP result callback never fires → deadlock  
**Do this instead:** Use `ReentrantCallbackGroup` for the result subscriber as done in `src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py`

### Direct SSH file editing on robot
**What happens:** Editing files directly via `nano`/`vi`/`sed -i` on Main Pi or Vision Pi  
**Why it's wrong:** Bypasses git; robot repo becomes dirty; CI/CD can't deploy  
**Do this instead:** Edit on dev machine → commit → push → GitHub Actions deploys

## Error Handling

**Strategy:** Each Docker service has `restart: unless-stopped` with healthchecks; ROS 2 nodes use `respawn: True` in launch files.

**Patterns:**
- LLM API failures: `provider_manager.py` falls back from Qwen → DeepSeek → logs error
- TTS failures: Yandex TTS → Silero offline fallback (always available)
- STT failures: Yandex STT gRPC → Vosk local model fallback
- Zenoh connection failures: `ZENOH_ROUTER_CHECK_ATTEMPTS=10` retry loop in startup scripts
- MCP tool failures: Timeout + error JSON returned on `/mcp/result`; DialogueNode continues

## Cross-Cutting Concerns

**Logging:** `self.get_logger().info/warn/error()` in all ROS 2 nodes; Docker logs collected by Promtail (`labels: logging: "promtail"`) → Loki → Grafana
**Validation:** System boundary validation in `dialogue_node.py` (LLM response parsing); hardware access validated at container startup via healthchecks
**Authentication:** Telegram bot uses token from `.env.secrets` (`TELEGRAM_BOT_TOKEN`); LLM APIs via `QWEN_API_KEY`, `DEEPSEEK_API_KEY`, `YANDEX_API_KEY`

---

*Architecture analysis: 2026-05-15*
