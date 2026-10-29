# Sound Pi (docker/sound)

Третий узел робота, `10.1.1.12` (eth0) / `10.1.1.22` (wlan0). ADR-0159. Здесь ReSpeaker (микрофон, колонки,
LED-кольцо) и TTL-USB конвертер сенсорного хаба.

| Сервис | Профиль | Откуда |
|---|---|---|
| `zenoh-router` (`zenoh-router-sound`) | — | общий шаблон из `../common`; `connect` на Main и на Vision (вариант Б) |
| `voice-assistant`, `supercollider` | — | перенесены из `docker/vision` (`voice-action-server` удалён в ADR-0150 PR-7c ч.4в) |
| `supervisor` (`avatar-supervisor`, ТАРС) | — | перенесён из `docker/vision` (ADR-0159 В8): пишет в тот же `data/voice/harness_voice.db`, что и `voice-assistant` |
| `avatar-arbiter` | — | волна 2 (PR-7, В9, решение Шифу 20.10): образ `supervisor`, скрипт `scripts/supervisor/start_arbiter.sh` рядом со `start_supervisor.sh` |
| `quest` (`rob-box-quest`) | — | волна 2: WebXR-консоль Meta Quest, Caddy на `:443`/`:8443`, WS `:8766`; именованный том `quest-certs` (на Sound — новый, пустой) |
| `telegram-bot` | — | волна 2: операторский Telegram-интерфейс, метрики `:9101`; галерея лиц `data/faces` — копия-снимок (синхронизация — #3655) |
| `sensor-bridge` | `sensors` | образ `perception`, нода `perception_bridge`, `/dev/sensor_hub` |
| `cadvisor`, `promtail` | `monitoring` | как на Vision, лейбл `host: sound-pi` |

- Адреса узла — `ZENOH_LISTEN_IP`, `ZENOH_CONNECT_ENDPOINTS`, `OLLAMA_HOST_URL` (дефолты в compose, переопределение в `.env`).
- Секреты: `cp .env.secrets.template .env.secrets` (ключи голосового стека и Telegram-токены: `telegram-bot` живёт здесь с PR-7).
- Данные состояния (`data/voice`, `cache/tts`) в git не лежат; на робота их переносит cutover (ADR-0159 §3.5).
- Каталоги сборки образов (`voice_base`, `voice_assistant`, `supercollider`, `supervisor`, `quest`, `telegram_bot`) лежат здесь.
- Quest: адрес для шлема — `10.1.1.12` (DNS `quest.rob_box.local` на роутере должен указывать на него — переключает Шифу); при пустом `quest-certs` сертификат создаётся заново.
- Проверка: `cd docker/sound && docker compose --profile sensors --profile monitoring config`.

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
cd docker/sound
docker compose up -d
```

### Только voice assistant

```bash
docker compose up -d zenoh-router voice-assistant
```

### Логи

```bash
# Все сервисы
docker compose logs -f

# Только voice assistant
docker compose logs -f voice-assistant

# ReSpeaker устройство
docker exec voice-assistant lsusb | grep 2886
docker exec voice-assistant arecord -l
```

### Остановка

```bash
docker compose down
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
- **Hardware:** docs/architecture/HARDWARE.md (Sound Pi, ReSpeaker)
- **Package README:** src/rob_box_voice/README.md
- **Installation:** src/rob_box_voice/INSTALL.md
