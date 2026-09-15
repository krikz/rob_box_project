# Vision Pi Docker Stack

Docker compose инфраструктура для Vision Pi (Raspberry Pi 5) - обработка визуальных данных, лидара, голосового управления и анимаций.

## Архитектура

### Сервисы

1. **zenoh-router** - Локальный Zenoh роутер для коммуникации между контейнерами
2. **oak-d** - OAK-D камера с интегрированной детекцией AprilTag (RGB stream + AprilTag detection)
3. **lslidar** - LSLIDAR N10 лидар (2D сканы)
4. **led-matrix** - Драйвер NeoPixel LED матрицы
5. **voice-assistant** - Голосовой ассистент + анимации (ReSpeaker Mic Array v2.0)
6. **vision-hailo** - AI inference на Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS); запускает `vision_hailo_node` (см. [ADR-0089](../../docs/adr/0089-ai-hat-plus-deployment.md), Phase 1 PoC, stub-режим `HAILO_ENABLED=false`); публикует события в топик `/vision/hailo/events` (`VisionEvent[]`). Hardware: `/dev/hailo0` (обязателен для Phase 1.5 real inference). Стартует декларативно через launch-файл `src/rob_box_perception/launch/vision_hailo.launch.py` (ADR-0096) — SSoT параметров `hailo_enabled`, `hef_path`, `confidence_threshold` и др. через `LaunchConfiguration`.

> ADR-0089 §3 touchpoint #6 (запуск через launch) — выполнен ADR-0096 (`vision_hailo.launch.py`). Downstream consumer `context_aggregator_node` → `/perception/context_update.vision_events_json` остаётся в scope отдельной задачи.

7. **ceiling-camera** - USB-потолочная MJPEG-камера (720p, ceiling context для LLM)
8. **supervisor** / **avatar-arbiter** - Avatar state machine + floor arbitration
   (ADR-0028, ADR-0051)
9. **quest** - WebXR-консоль для Meta Quest 3 (ADR-0027, ADR-0086)
10. **telegram-bot** - операторский Telegram-интерфейс
11. **supercollider** + **voice-resources-init** - SynthDef-сервер scsynth
    (renardo samples + synthdefs шарены с voice-assistant)
12. **voice-action-server** - sidecar для action/PASTE control plane (Phase 4)
13. **ollama** (profile `ai`) - локальный LLM inference (embeddings для VoiceMemory)
14. **cadvisor** + **promtail** (profile `monitoring`) - мониторинг Vision Pi

### AI HAT+ Vision (ADR-0089, ADR-0096)

Сервис **vision-hailo** живёт в отдельном контейнере (Vision Pi 10.1.1.21,
доступ к `/dev/hailo0` через `devices:` bind-mount). Стартует декларативно
через launch-файл `src/rob_box_perception/launch/vision_hailo.launch.py`
(ADR-0096) — SSoT параметров `hailo_enabled`, `hef_path`,
`confidence_threshold` и др. через `LaunchConfiguration`.

**Capability-honest режим** (ADR-0018): если `hailo_enabled=true`, но
`/dev/hailo0` или HEF отсутствуют — pre-flight check (OpaqueFunction)
логирует WARN, нода деградирует в stub-режим (события
`event_type="stub"`). Phase 1.5 stub-filter (mcp_server) блокирует
попадание stub-событий в LLM-контекст.

Конфигурация SSoT: `docker/vision/config/hailo_models.yaml`.

Контракт ADR-0089 §3 touchpoint #6 (запуск через launch) — выполнен
ADR-0096 (vision_hailo.launch.py).

### Схема коммуникации

```
Vision Pi (Raspberry Pi 5)
├── zenoh-router (порт 7447)
│   ├─ oak-d → /camera/color/image_raw, /camera/color/camera_info, /apriltag_detections
│   ├─ lslidar → /scan
│   ├─ led-matrix → слушает /animations/trigger
│   ├─ vision-hailo → /vision/hailo/events (VisionEvent[])
│   │       └─▶ (downstream) context_aggregator_node → /perception/context_update.vision_events_json
│   └─ voice-assistant → /voice/*, /audio/*, /animations/trigger
│
└── Zenoh Bridge → Main Pi (WiFi/Ethernet)
    └─ Main Pi zenoh-router → Navigation stack
```

## Voice Assistant Service

### Возможности

- **Распознавание речи** (Yandex STT)
- **Синтез речи** (Yandex TTS, voice "anton")
- **LLM диалог** (DeepSeek API)
- **12× RGB LED индикация** (ReSpeaker встроенные LED)
- **LED матрица анимации** (интеграция с rob_box_animations)
- **Звуковые эффекты** (sound_pack)
- **Управление роботом** (ROS2 команды через LLM)

### ROS2 Nodes в контейнере

- `audio_node` - Захват аудио, VAD, DoA
- `led_node` - Управление 12 RGB LED ReSpeaker
- `stt_node` - Speech-to-Text (Yandex Cloud)
- `tts_node` - Text-to-Speech (Yandex Cloud)
- `dialogue_node` - LLM диалог (DeepSeek)
- `sound_node` - Звуковые эффекты
- `command_node` - Парсинг и выполнение команд
- `voice_animation_player` - Проигрывание анимаций на LED матрице

### Конфигурация

**Основной конфиг (docker-деплой):** `config/voice_assistant/<node>.yaml`
(per-node файлы, issue #1004 / ADR-0004). Монтируются в
`/config/voice_assistant` и читаются launch через `config_dir:=`.
Монолитного `voice_assistant.yaml` больше нет — вложенные секции создавали
dotted-имена, которые ноды не читали.

```yaml
# config/voice_assistant/audio_node.yaml
audio_node:
  ros__parameters:
    device_name: "ReSpeaker"
    sample_rate: 16000
    vad_threshold: 3.5
```

**Секреты:** Скопировать `config/voice_assistant/secrets.yaml.example` → `secrets.yaml` и заполнить API ключи.

### Требования к железу

- **ReSpeaker Mic Array v2.0** (USB 0x2886:0x0018)
- **Audio device:** ALSA, USB audio class
- **Memory:** 2GB RAM (включая animations)
- **USB bandwidth:** High-speed USB 2.0

## Запуск

### Полный стек (все сервисы)

```bash
cd docker/vision
docker-compose up -d
```

### Только voice assistant + animations

```bash
docker-compose up -d zenoh-router voice-assistant
```

### Логи

```bash
# Все сервисы
docker-compose logs -f

# Только voice assistant
docker-compose logs -f voice-assistant

# ReSpeaker устройство
docker exec voice-assistant lsusb | grep 2886
docker exec voice-assistant arecord -l
```

### Остановка

```bash
docker-compose down
```

## Разработка

### Пересборка образа

```bash
# Voice assistant
docker build -t ghcr.io/krikz/rob_box:voice-assistant-humble-latest \
  -f voice_assistant/Dockerfile \
  ../..

# Загрузка в registry
docker push ghcr.io/krikz/rob_box:voice-assistant-humble-latest
```

### Тестирование локально

```bash
# Запуск в интерактивном режиме
docker run -it --rm \
  --network host \
  --privileged \
  --device /dev/snd \
  --device /dev/bus/usb \
  -v $(pwd)/config:/config \
  -v $(pwd)/../../sound_pack:/ws/sound_pack:ro \
  ghcr.io/krikz/rob_box:voice-assistant-humble-latest \
  /bin/bash

# Внутри контейнера
source /ws/install/setup.bash
ros2 launch rob_box_voice voice_assistant.launch.py
```

### Отладка ReSpeaker

```bash
# Войти в контейнер
docker exec -it voice-assistant bash

# Проверить USB
lsusb | grep 2886

# Проверить audio device
arecord -l | grep ReSpeaker

# Тест аудио записи (5 сек)
arecord -D plughw:CARD=ReSpeaker -f S16_LE -r 16000 -c 1 -d 5 test.wav

# Тест LED (красный цвет)
python3 << EOF
from rob_box_voice.utils.respeaker_interface import ReSpeakerInterface
dev = ReSpeakerInterface()
dev.write('GAMMAVAD_SR', 8.5)
EOF
```

### Мониторинг ROS2 топиков

```bash
# Voice state
ros2 topic echo /voice/state

# Audio VAD
ros2 topic echo /audio/vad

# Direction of Arrival
ros2 topic echo /audio/direction

# Animation trigger
ros2 topic echo /animations/trigger
```

## Troubleshooting

### ReSpeaker не найден

```bash
# Проверить подключение
lsusb | grep 2886

# Проверить udev rules
cat /etc/udev/rules.d/60-respeaker.rules

# Перезагрузить udev
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Audio не работает

```bash
# Проверить ALSA devices
aplay -l
arecord -l

# Проверить permissions
groups | grep audio

# Проверить занятость устройства
lsof /dev/snd/*
```

### Zenoh не подключается

```bash
# Проверить router
curl http://localhost:8000/@/local/router

# Проверить Zenoh сессию
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### Память переполняется (OOM)

```bash
# Проверить использование
docker stats voice-assistant

# Уменьшить лимиты в docker-compose.yaml
# mem_limit: 1.5g
# memswap_limit: 2g

# Очистить TTS кэш
rm -rf cache/tts/*
```

## Дополнительная информация

- **Architecture:** docs/development/VOICE_ASSISTANT_ARCHITECTURE.md
- **Hardware:** docs/HARDWARE.md (секция 3.4 ReSpeaker)
- **Package README:** src/rob_box_voice/README.md
- **Installation:** src/rob_box_voice/INSTALL.md
