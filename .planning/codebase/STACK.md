# Technology Stack

**Analysis Date:** 2026-05-15

## Languages

**Primary:**
- Python 3.10 — all ROS 2 nodes (`src/rob_box_voice/`, `src/rob_box_perception/`, `src/rob_box_mcp_tools/`, `src/rob_box_telegram/`, `src/led_matrix_driver/`)
- C++ (ament_cmake) — hardware interface plugin (`src/vesc_nexus/`), animations (`src/rob_box_animations/`)

**Secondary:**
- SQL (SQLite) — database migrations (`migrations/`)
- Bash/Shell — Docker entrypoints, deployment scripts (`docker/*/scripts/`)
- Xacro/URDF — robot description (`src/rob_box_description/urdf/`)
- JSON5 — Zenoh configuration (`docker/main/config/*.json5`, `docker/vision/config/*.json5`)

## Runtime

**Environment:**
- Ubuntu 22.04 (Jammy) — bundled with `ros:kilted-ros-base`
- Python 3.10 — default Python in ROS 2 kilted / Ubuntu 22.04

**Package Manager:**
- pip (Python packages, installed in Dockerfiles)
- apt (system packages)
- colcon (ROS 2 workspace build tool)

## Frameworks

**Core:**
- ROS 2 kilted (`ros:kilted-ros-base`) — middleware for all nodes; pub/sub, services, actions
- rmw_zenoh_cpp — DDS middleware replacing default FastDDS; configured via `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- ament_python — Python ROS 2 package build type (all Python packages)
- ament_cmake + ament_cmake_python — CMake-based ROS 2 build (vesc_nexus, rob_box_animations)

**AI / LLM:**
- `openai-agents` — OpenAI Agents SDK used for dialogue loop in `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- `openai>=1.0.0` — OpenAI-compatible HTTP client (connects to DeepSeek, Qwen, or OpenAI)

**Audio / Speech:**
- `vosk>=0.3.45` — Offline Russian STT (used in `src/rob_box_voice/rob_box_voice/stt_node.py`)
- `torch>=2.9.0`, `torchaudio>=2.9.0` — PyTorch for Silero TTS v5 (ARM64; **must be ≥2.9.0** to avoid MKLDNN matmul bug on aarch64)
- `scipy` — Required by Silero v5 for audio resampling
- `grpcio`, `grpcio-tools`, `protobuf`, `yandex-cloud-ml-sdk` — Yandex Cloud TTS/STT gRPC API v3

**Navigation / SLAM:**
- RTAB-Map (`introlab3it/rtabmap_ros:kilted-latest`) — LiDAR SLAM, loop closure, 2D/3D mapping
- Nav2 — autonomous navigation stack (`docker/main/nav2/`)
- ros2_control + `vesc_nexus` plugin — hardware abstraction for VESC motor controllers

**Vision:**
- DepthAI ROS (`luxonis/depthai-ros:v2.12.2-kilted`) — OAK-D depth camera driver; **pinned to v2.12.2** (last version with arm64 support)
- AprilTag detection — integrated into OAK-D container (`docker/vision/oak-d/`)

**Music / Sound:**
- `renardo-lib` — Python FoxDot successor for algorithmic music synthesis
- SuperCollider (`docker/vision/supercollider/`) — audio synthesis server; renardo connects via OSC on `localhost:57110`

**Testing:**
- pytest >= 6.0 — test runner for all Python packages
- pytest-cov — coverage measurement (80% minimum in mcp_tools)
- ament_flake8, ament_pep257, ament_copyright — ROS 2 linting wrappers

**Build/Dev:**
- colcon — ROS 2 workspace build tool
- Docker Buildx with QEMU — cross-compilation for `linux/arm64` on GitHub Actions x86 runners

## Key Dependencies

**Critical Python:**
- `openai>=1.0.0` — LLM API client (DeepSeek/Qwen/OpenAI compatible); `src/rob_box_voice/requirements.txt`, `docker/main/perception/requirements.txt`
- `openai-agents` — Agents SDK for agentic dialogue loops; `docker/vision/voice_assistant/requirements.txt`
- `torch>=2.9.0` — PyTorch for Silero TTS; **ARM64 constraint**: <2.9.0 has broken matmul primitives; `docker/vision/voice_assistant/requirements.txt`
- `vosk>=0.3.45` — offline STT; `src/rob_box_voice/requirements.txt`
- `yandex-cloud-ml-sdk` — Yandex Cloud TTS gRPC (primary TTS provider); `docker/vision/voice_assistant/requirements.txt`
- `grpcio`, `grpcio-tools`, `protobuf` — gRPC for Yandex Cloud; `src/rob_box_voice/requirements.txt`
- `sqlite-vec>=0.1.0` — SQLite vector extension for semantic memory search (optional, graceful degradation); `src/rob_box_voice/requirements.txt`
- `pyaudio>=0.2.11`, `sounddevice>=0.4.6`, `pydub>=0.25.1` — audio capture/playback; `src/rob_box_voice/requirements.txt`
- `pyusb>=1.2.1`, `pixel-ring>=0.1.0`, `spidev>=3.5` — ReSpeaker Mic Array USB interface; `src/rob_box_voice/requirements.txt`
- `pillow` — image processing for LED matrix animations; `docker/vision/voice_assistant/requirements.txt`
- `pyyaml>=6.0`, `omegaconf==2.3.0` — YAML/config management; `src/rob_box_voice/requirements.txt`
- `pi5neo`, `spidev` — NeoPixel SPI LED driver for LED matrix; `src/led_matrix_driver/package.xml`
- `renardo-lib` — music synthesis; `docker/vision/voice_assistant/requirements.txt`
- `pytz>=2023.3` — timezone support; `docker/main/perception/requirements.txt`
- `python-telegram-bot` — Telegram Bot API client (imported as `telegram`, `telegram.ext`); `src/rob_box_telegram/`

**ROS 2 Packages:**
- `rclpy` — Python ROS 2 client library (all Python packages)
- `rclcpp`, `rclcpp_lifecycle` — C++ ROS 2 client (vesc_nexus)
- `hardware_interface`, `pluginlib`, `controller_manager` — ros2_control plugin system (vesc_nexus)
- `nav2_msgs` — navigation action interfaces (mcp_tools)
- `tf2_ros` — transform tree (mcp_tools, perception)
- `audio_common_msgs` — audio data messages (voice)
- `rob_box_perception_msgs` — custom perception messages
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs` — standard ROS 2 messages

**Zenoh:**
- `eclipse/zenoh:1.6.2` — router image; deployed as `zenoh-router` on each Pi
- `rmw_zenoh_cpp` (from `ros-kilted-rmw-zenoh-cpp`) — ROS 2 RMW implementation

**Infrastructure:**
- `gcr.io/cadvisor/cadvisor:v0.52.1` — container metrics
- `prom/prometheus:v2.47.2` — metrics collection
- `grafana/grafana:10.2.0` — visualization
- `grafana/loki:2.9.2` — log aggregation
- `ollama/ollama:latest` — local LLM/embedding server (optional `--profile ai`)

## Configuration

**Environment:**
- All services configured via environment variables in `docker/main/docker-compose.yaml` and `docker/vision/docker-compose.yaml`
- Secrets in `docker/vision/.env.secrets` and `docker/main/.env.secrets` (not committed)
- Key runtime vars: `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROBOT_ID`, `ZENOH_SESSION_CONFIG_URI`

**Build:**
- `docker/base/Dockerfile.ros2-zenoh` — base for most services
- `docker/base/Dockerfile.rtabmap` — RTAB-Map base (extends `introlab3it/rtabmap_ros:kilted-latest`)
- `docker/base/Dockerfile.depthai` — DepthAI base (extends `luxonis/depthai-ros:v2.12.2-kilted`)
- `docker/base/Dockerfile.pcl` — PCL base (extends ros2-zenoh base)
- Image tags tracked per-service in `docker/main/.image-versions.{env}` and `docker/vision/.image-versions.{env}`

## Platform Requirements

**Development (GitHub Actions):**
- Runners: `ubuntu-latest` (x86_64) with Docker Buildx + QEMU for cross-compilation
- Also: self-hosted ARM64 runners at `docker/build/data/runner1..runner8`
- Registry: `ghcr.io` (GitHub Container Registry)

**Production:**
- Dual Raspberry Pi 5 (8GB RAM, aarch64, Ubuntu)
- Main Pi: `10.1.1.10` (eth0), `10.1.1.20` (wlan0)
- Vision Pi: `10.1.1.11` (eth0), `10.1.1.21` (wlan0)
- All containers use `network_mode: host`

---

*Stack analysis: 2026-05-15*
