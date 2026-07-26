# Rob Box

## What This Is

Rob Box — автономный ровер на базе ROS 2 kilted + Zenoh DDS, построенный на двух Raspberry Pi 5 (Main: навигация/управление, Vision: сенсоры/голос/LED). Платформа служит R&D стендом для отработки технологий автономной мобильной робототехники: SLAM, голосового управления с LLM, 3D восприятия и надёжной локализации в больших помещениях.

## Core Value

Робот должен надёжно знать, где он находится — после запуска, перезапуска или физического перемещения — и уметь объезжать реальные 3D препятствия (диваны, стулья), которые 2D LiDAR не видит.

## Requirements

### Validated

<!-- Shipped and confirmed valuable. -->

- ✓ RTAB-Map SLAM с 2D LiDAR + ICP-одометрией — Phase 0
- ✓ Nav2 автономная навигация с plannerом и контроллером — Phase 0
- ✓ Voice assistant: Vosk STT + Silero TTS + LLM (DeepSeek/Qwen) — Phase 0
- ✓ 381 LED NeoPixel матрица с анимациями — Phase 0
- ✓ VESC CAN Bus управление моторами (ros2_control, 50 Hz) — Phase 0
- ✓ Мониторинг: Grafana + Prometheus + Loki — Phase 0
- ✓ Docker CI/CD: GitHub Actions → build → deploy на Pi — Phase 0
- ✓ Персистентная карта: RTAB-Map DB переживает рестарт — Phase 0

### Active

- [ ] Документация актуальна и отражает реальное состояние кода
- [ ] Ревью и очистка структуры проекта (пакеты, Dockerfile-ы, конфиги)
- [ ] Code quality review: все критические issues задокументированы и приоритезированы

### Out of Scope (Milestone 1)

- AprilTag-локализация после перезапуска — Milestone 2 (навигация)
- 3D obstacle avoidance (OAK-D voxel costmap) — Milestone 2 (навигация)
- AI HAT+ (Hailo-8L, 26 TOPS) — hardware не закуплено, Milestone 3+
- Mobile app / Web UI — нет бизнес-требований сейчас
- Face recognition / user database — зависит от AI HAT+
- dialogue_node рефакторинг / LLM-стабильность — Milestone 3

## Context

**Железо:**
- Main Pi 5 (16 GB, 10.1.1.10): rtabmap, nav2, ros2_control, VESC, lslidar, perception
- Vision Pi 5 (8 GB, 10.1.1.11): oak-d, voice-assistant, led-matrix, apriltag, ceiling-camera
- Сенсоры: LS N10 2D LiDAR, OAK-D Lite (RGB-D + стерео), потолочная USB-камера (720p), BLE-джойстик (заблокирован kernel 6.14.0-raspi)

**Ключевая проблема локализации:**
Карта сохраняется в RTAB-Map DB и переживает рестарт. Но AMCL/RTAB-Map не умеет надёжно инициализировать pose после рестарта или физического перемещения робота без подсказки. Потолочные AprilTags дают абсолютную позицию — нужно их использовать как initial pose provider.

**Ключевая проблема 3D препятствий:**
LiDAR видит только горизонтальный срез ~15 см от пола. Диван: видит изголовье (вертикальная поверхность), но не видит сиденье, которое нависает над полом — робот едет в него. OAK-D Lite даёт depth pointcloud — нужно интегрировать в Nav2 costmap через voxel_layer или obstacle_layer с 3D pointcloud.

**Текущие баги (critical for navigation):**
- command_node.py: get_current_position, detect_objects, follow_person — все stubs
- MCP get_robot_status: hardcoded position {0,0,0} и battery 85%
- BLE joystick: заблокирован upstream kernel bug (нет SW workaround)

## Constraints

- **Hardware**: Raspberry Pi 5 — ARM64, ограниченная RAM для тяжёлых ML моделей без NPU
- **Network**: Zenoh DDS, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, требует zenoh-router
- **Docker**: Все сервисы в Docker, network_mode: host, конфиги через volumes (не COPY)
- **ROS**: kilted, rmw_zenoh_cpp, RTAB-Map + Nav2
- **AI HAT+**: Hardware не закуплено — не планировать зависимости на него в v1

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Zenoh DDS вместо CycloneDDS | Облачная интеграция, лучшая производительность на Pi | ✓ Good |
| Dual Pi 5 | Разделение вычислительной нагрузки Nav/Vision | ✓ Good |
| RTAB-Map LiDAR-only SLAM | Нет необходимости в RGB-D для картографирования | — Pending (3D obstacle проблема) |
| OAK-D depth для 3D costmap | Дешевле и быстрее, чем AI HAT+ для препятствий | — Pending |
| AprilTag initial pose | Абсолютная позиция без дрейфа после рестарта | — Pending |

---
*Last updated: 2026-05-15 after GSD initialization*
