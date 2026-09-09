# External Integrations

**Analysis Date:** 2026-05-15

## APIs & External Services

**LLM Providers (dialogue system):**
- **DeepSeek API** — Primary LLM provider for voice dialogue agent
  - SDK/Client: `openai>=1.0.0` (OpenAI-compatible base_url)
  - URL: `https://api.deepseek.com/v1`
  - Model: `deepseek-chat`
  - Auth: `DEEPSEEK_API_KEY` or `LLM_API_KEY` env var
  - Used in: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (class `DialogueNode`, `PROVIDERS` dict)
  - Also: `docker/main/perception/` (reflection node)

- **Alibaba Qwen (DashScope)** — Secondary/fallback LLM provider
  - SDK/Client: `openai>=1.0.0` (OpenAI-compatible base_url)
  - URL: `https://dashscope-intl.aliyuncs.com/compatible-mode/v1`
  - Auth: `QWEN_API_KEY` or `LLM_API_KEY` env var
  - Used in: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (class `DialogueNode`, `PROVIDERS` dict)

**Speech (TTS/STT):**
- **Yandex Cloud TTS API v3** — Primary text-to-speech (voice: `anton`)
  - Protocol: gRPC
  - SDK/Client: `yandex-cloud-ml-sdk`, `grpcio`, `grpcio-tools`, `protobuf`
  - Auth: `YANDEX_API_KEY`, `YANDEX_FOLDER_ID` env vars
  - Used in: `src/rob_box_voice/rob_box_voice/tts_node.py` (primary provider, `grpc.secure_channel`)
  - Fallback: Silero TTS (offline, PyTorch) when Yandex unavailable

- **Yandex Cloud STT API v3** — Speech-to-text (streaming, Russian)
  - Protocol: gRPC (same credentials as TTS)
  - Auth: `YANDEX_API_KEY`, `YANDEX_FOLDER_ID` env vars
  - Used in: `src/rob_box_voice/rob_box_voice/stt_node.py`, `src/rob_box_voice/scripts/record_yandex_voice.py`
  - Primary offline STT: Vosk (`vosk>=0.3.45`)

**Messaging:**
- **Telegram Bot API** — Operator remote-control interface
  - SDK/Client: `python-telegram-bot` (`telegram`, `telegram.ext`)
  - Auth: `TELEGRAM_BOT_TOKEN`, `TELEGRAM_ALLOWED_USERS` env vars (in `docker/vision/.env.secrets`)
  - Used in: `src/rob_box_telegram/rob_box_telegram/telegram_node.py`, `handlers/commands.py`, `handlers/messages.py`, `handlers/callbacks.py`
  - Capabilities: photo, voice messages, movement commands, LLM chat via MCP bridge

**Secrets location:** `docker/vision/.env.secrets` and `docker/main/.env.secrets` (git-ignored)

## Data Storage

**Databases:**
- **SQLite** (primary persistent store)
  - Connection: env var `VOICE_MEMORY_DB_PATH` (default: `/data/voice_memory.db`)
  - Client: Python stdlib `sqlite3`
  - Volume mount: `docker/vision/data/voice:/data`
  - Schema applied via migrations at startup (idempotent)
  - Extensions: FTS5 (built-in full-text search), `sqlite-vec` (optional 768d vector search)
  - Migrations: `migrations/001_init.sql` through `migrations/005_faq.sql`
  - Tables: `events`, `summaries`, `summary_events` (perception/reflection), `voice_memory` (dialogue history), `waypoints`, `maps` (navigation), `music_tracks` (music library), `faq_items` (FAQ)

- **RTAB-Map database** (SLAM map storage)
  - Path: `/maps/rtabmap.db`
  - Format: SQLite (RTAB-Map internal schema)
  - Volume mount: `docker/main/maps:/maps`
  - Managed by: `docker/main/rtabmap/` service

**File Storage:**
- TTS audio cache: `docker/vision/cache/tts:/cache/tts` (persistent volume mount)
- Renardo music samples: Docker named volume `renardo_samples` (populated by `voice-resources-init` one-shot container)
- Sound pack: `sound_pack/` (bind mount `../../sound_pack:/ws/sound_pack:ro`)
- Ollama models: `docker/vision/data/ollama:/root/.ollama` (persistent volume)

**Caching:**
- In-memory caching in individual ROS 2 nodes (no shared cache service)
- TTS audio file cache on disk (see above)

## Authentication & Identity

**Auth Provider:**
- No centralized auth service
- Telegram bot: per-user allowlist via `TELEGRAM_ALLOWED_USERS` env var; implemented in `src/rob_box_telegram/rob_box_telegram/auth.py`
- LLM APIs: API key auth via env vars (per-provider)
- Grafana: basic password auth via `GRAFANA_PASSWORD` env var

## Monitoring & Observability

**Metrics:**
- Prometheus v2.47.2 — scrapes cAdvisor (port 8080) on both Pis
  - Config: `docker/monitoring/config/prometheus.yml`
  - Retention: 7 days
  - Port: 9090

**Logs:**
- Loki v2.9.2 — log aggregation
  - Config: `docker/monitoring/config/loki-config.yaml`
  - Port: 3100
- Promtail — agent collecting Docker container logs (label `logging: "promtail"` on each service)

**Visualization:**
- Grafana v10.2.0 — unified dashboard
  - Config: `docker/monitoring/config/grafana/provisioning/`
  - Port: 3000
  - Dashboard: `docker/monitoring/config/grafana/provisioning/dashboards/rob_box_dashboard.json`

**Container Metrics:**
- cAdvisor v0.52.1 — deployed on both Pis via monitoring profile
  - Port: 8080
  - Interval: 30s

## CI/CD & Deployment

**Container Registry:**
- GitHub Container Registry (`ghcr.io`)
  - Base images: `ghcr.io/krikz/rob_box_base:{name}-{ros_distro}-{tag}`
    - `rob_box_base:ros2-zenoh-humble-{tag}`
    - `rob_box_base:rtabmap-humble-{tag}`
    - `rob_box_base:depthai-humble-{tag}`
    - `rob_box_base:pcl-humble-{tag}`
  - Service images: `ghcr.io/krikz/rob_box:{service}-humble-{tag}`
    - Vision Pi: `oak-d`, `led-matrix`, `ceiling-camera`, `supercollider`, `voice-assistant`, `voice-resources`, `telegram-bot`
    - Main Pi: `twist-mux`, `robot-state-publisher`, `rtabmap`, `ros2-control`, `lslidar`, `perception`, `nav2`, `teleop`

**CI Pipeline:**
- GitHub Actions (`.github/workflows/`)
  - `G-Build Base Images.yml` — builds base Docker images (manual/scheduled)
  - `G-Build Main Pi Services.yml` — builds Main Pi service images
  - `G-Build Vision Pi Services.yml` — builds Vision Pi service images
  - `G-Build All Services.yml` — builds all services
  - `G-Run Tests.yml` — runs Python tests with colcon
  - `G-Lint Code.yml` — black, isort, flake8 linting
  - `G-Validate Docker Compose.yml` — docker-compose validation
  - `G-Integration Tests.yml` — integration test suite
  - `L-Deploy and Verify.yml` — deploys to robots via self-hosted runners
- Build platform: `linux/arm64` (cross-compiled on x86 runners via Docker Buildx + QEMU)
- Local self-hosted runners: `docker/build/` (runners 1–8 for ARM native builds)

**Hosting:**
- Dual Raspberry Pi 5 (production hardware)
- All Docker services use `network_mode: host` (no Docker networking overhead)

## Local Services

**Zenoh DDS Middleware:**
- `eclipse/zenoh:1.6.2` — router deployed on each Pi as `zenoh-router` / `zenoh-router-vision`
- Router-to-router interconnect: Main Pi router ↔ Vision Pi router via TCP
- Cloud bridge: Main Pi router connects to `tcp/zenoh.robbox.online:7447` (cloud Zenoh router for remote access)
- Config: `docker/main/config/zenoh_router_config.json5`, `docker/vision/config/zenoh_router_config.json5`
- Session config generated per-container at startup via `ros_with_namespace.sh`

**Ollama (Local LLM/Embedding):**
- Image: `ollama/ollama:latest`
- Container: `ollama` on Vision Pi
- Port: 11434 (HTTP REST)
- Activation: `docker compose --profile ai up -d ollama`
- Model: `nomic-embed-text` (768d embeddings for semantic memory search)
- Used by: `src/rob_box_voice/rob_box_voice/core/voice_memory.py` (`OllamaEmbedder` class), `src/rob_box_voice/rob_box_voice/core/faq_store.py`
- Graceful degradation: system falls back to FTS5 keyword search if Ollama unavailable

**SuperCollider (Music Synthesis):**
- Image: custom `supercollider` built from `docker/vision/supercollider/Dockerfile`
- Container: `supercollider` on Vision Pi
- OSC port: 57110
- Used by: `renardo-lib` in voice-assistant container (connects via OSC to `localhost:57110`)
- IPC: `ipc: host` shared with `voice-assistant` for JACK/dmix SHM

## Hardware Interfaces

**VESC Motor Controllers:**
- Protocol: CAN bus (`can0` interface, enabled by `host/main/setup_can.sh`)
- Driver: `src/vesc_nexus/` — ros2_control `hardware_interface` plugin
- Container: `ros2-control` on Main Pi
- Systemd service: `host/main/can-setup.service`

**LSLIDAR N10 (2D LiDAR):**
- Protocol: USB CDC ACM serial
- Device: `/dev/ttyACM0`
- Driver: `docker/main/lslidar/` service
- Container: `lslidar` on Main Pi

**OAK-D Camera (DepthAI):**
- Protocol: USB 3.0
- Device: `/dev/bus/usb`
- SDK: `luxonis/depthai-ros:v2.12.2-humble`
- Container: `oak-d` on Vision Pi
- Includes integrated AprilTag detection (eliminates separate apriltag container)

**ReSpeaker Mic Array v2.0:**
- Protocol: USB audio + USB HID (LED control)
- Device: `/dev/bus/usb`, ALSA `ReSpeaker` card
- SDK: `pyusb>=1.2.1`, `pixel-ring>=0.1.0`
- Container: `voice-assistant` on Vision Pi

**NeoPixel LED Matrix (381 LEDs):**
- Protocol: SPI
- Device: `/dev/spidev0.0`
- SDK: `pi5neo`, `spidev>=3.5`
- Container: `led-matrix` on Vision Pi

**USB UVC Ceiling Camera:**
- Device: `/dev/video0`
- Container: `ceiling-camera` on Vision Pi
- Calibration: `docker/vision/config/ceiling-camera/ceiling_camera.yaml`

**WM8960 Sound Card:**
- Protocol: I2C + PCM
- Interface: 3.5mm audio jack for TTS output
- Used via ALSA dmix config: `docker/vision/config/audio/asound.conf`

## Webhooks & Callbacks

**Incoming:**
- No external incoming webhooks

**Outgoing:**
- Telegram Bot API — polling mode (no webhook server required)
- DeepSeek API — HTTPS POST (streaming responses via OpenAI SDK)
- Qwen DashScope API — HTTPS POST (streaming via OpenAI SDK)
- Yandex Cloud TTS/STT — gRPC streaming
- Ollama — HTTP POST to `http://localhost:11434/api/embeddings`
- Cloud Zenoh router — persistent TCP connection to `zenoh.robbox.online:7447`

## Environment Configuration

**Required env vars (per service):**

Vision Pi (`docker/vision/.env.secrets`):
- `QWEN_API_KEY` — Alibaba Qwen LLM (primary dialogue)
- `DEEPSEEK_API_KEY` — DeepSeek LLM (fallback)
- `YANDEX_API_KEY` — Yandex Cloud TTS/STT
- `YANDEX_FOLDER_ID` — Yandex Cloud folder ID
- `TELEGRAM_BOT_TOKEN` — Telegram bot token
- `TELEGRAM_ALLOWED_USERS` — comma-separated Telegram user IDs

Main Pi (`docker/main/.env.secrets`):
- `QWEN_API_KEY` — for perception/reflection node
- `DEEPSEEK_API_KEY` — fallback LLM

Both (`docker-compose.yaml`):
- `ROBOT_ID` — robot namespace identifier (e.g. `robbox1`)
- `ROS_DISTRO` — ROS 2 distribution (default: `humble`)
- `SERVICE_IMAGE_PREFIX` — registry prefix (default: `ghcr.io/krikz/rob_box`)

Monitoring (`docker/monitoring/`):
- `GRAFANA_PASSWORD` — Grafana admin password
- `MAIN_PI_IP` — Main Pi IP (default: `10.1.1.10`)
- `VISION_PI_IP` — Vision Pi IP (default: `10.1.1.11`)

---

*Integration audit: 2026-05-15*
