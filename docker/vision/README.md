# Vision Pi Docker Stack

Docker compose инфраструктура для Vision Pi (Raspberry Pi 5) - камеры (OAK-D, потолочная), лидар, Hailo (детекция и лица), LED-матрица.

> **ADR-0159:** голосовой стек (`voice-assistant`, `supercollider`, `voice-action-server`), ТАРС (`supervisor`),
> а с PR-7 (волна 2, В9, решение Шифу 20.10) и `avatar-arbiter`, `quest`, `telegram-bot` живут на Sound Pi — см.
> [`docker/sound/README.md`](../sound/README.md). Здесь остаётся тяжёлое: камеры, Hailo, распознавание лиц, LED-матрица.

## Архитектура

### Сервисы

1. **zenoh-router** - Локальный Zenoh роутер для коммуникации между контейнерами
2. **oak-d** - OAK-D камера с интегрированной детекцией AprilTag (RGB stream + AprilTag detection)
3. **lslidar** - LSLIDAR N10 лидар (2D сканы)
4. **led-matrix** - Драйвер NeoPixel LED матрицы
5. **vision-hailo** - AI inference на Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS); запускает `vision_hailo_node` (см. [ADR-0089](../../docs/adr/0089-ai-hat-plus-deployment.md), Phase 1 PoC, stub-режим `HAILO_ENABLED=false`); публикует события в топик `/vision/hailo/events` (`VisionEvent[]`). Hardware: `/dev/hailo0` (обязателен для Phase 1.5 real inference). Стартует декларативно через launch-файл `src/rob_box_perception/launch/vision_hailo.launch.py` (ADR-0110) — SSoT параметров `hailo_enabled`, `hef_path`, `confidence_threshold` и др. через `LaunchConfiguration`.

> ADR-0089 §3 touchpoint #6 (запуск через launch) — выполнен ADR-0110 (`vision_hailo.launch.py`). Downstream consumer `context_aggregator_node` → `/perception/context_update.vision_events_json` остаётся в scope отдельной задачи.

6. **ceiling-camera** - USB-потолочная MJPEG-камера (720p, ceiling context для LLM)
7. **vision-face** - распознавание лиц на Hailo, FaceStore в `data/faces` (ADR-0123)
8. **ollama** (profile `ai`) - локальный LLM inference (embeddings для VoiceMemory; голосовой стек на Sound Pi ходит сюда через `OLLAMA_HOST_URL`)
9. **cadvisor** + **promtail** (profile `monitoring`) - мониторинг Vision Pi

### AI HAT+ Vision (ADR-0089, ADR-0110)

Сервис **vision-hailo** живёт в отдельном контейнере (Vision Pi 10.1.1.21,
доступ к `/dev/hailo0` через `devices:` bind-mount). Стартует декларативно
через launch-файл `src/rob_box_perception/launch/vision_hailo.launch.py`
(ADR-0110) — SSoT параметров `hailo_enabled`, `hef_path`,
`confidence_threshold` и др. через `LaunchConfiguration`.

**Capability-honest режим** (ADR-0018): если `hailo_enabled=true`, но
`/dev/hailo0` или HEF отсутствуют — pre-flight check (OpaqueFunction)
логирует WARN, нода деградирует в stub-режим (события
`event_type="stub"`). Phase 1.5 stub-filter (mcp_server) блокирует
попадание stub-событий в LLM-контекст.

Конфигурация SSoT: `docker/vision/config/hailo_models.yaml`.

Контракт ADR-0089 §3 touchpoint #6 (запуск через launch) — выполнен
ADR-0110 (vision_hailo.launch.py).

### Схема коммуникации

```
Vision Pi (Raspberry Pi 5)
├── zenoh-router (порт 7447)
│   ├─ oak-d → /camera/color/image_raw, /camera/color/camera_info, /apriltag_detections
│   ├─ lslidar → /scan
│   ├─ led-matrix → слушает /animations/trigger
│   ├─ vision-hailo → /vision/hailo/events (VisionEvent[])
│   │       └─▶ (downstream) context_aggregator_node → /perception/context_update.vision_events_json
│
└── Zenoh Bridge → Main Pi (WiFi/Ethernet)
    └─ Main Pi zenoh-router → Navigation stack
```

## Запуск

### Полный стек (все сервисы)

```bash
cd docker/vision
docker-compose up -d
```

### Логи

```bash
# Все сервисы
docker-compose logs -f
```

### Остановка

```bash
docker-compose down
```

## Troubleshooting

### Zenoh не подключается

```bash
# Проверить router
curl http://localhost:8000/@/local/router

# Проверить Zenoh сессию
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

## Дополнительная информация

- **Голосовой стек, ReSpeaker, отладка звука:** [docker/sound/README.md](../sound/README.md)
