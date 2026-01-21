# rob_box_voice

AI Voice Assistant для автономного ровера РОББОКС с ReSpeaker Mic Array v2.0

## Описание

Модульная ROS2 система голосового управления роботом с интеграцией:
- **ReSpeaker Mic Array v2.0** — захват аудио, VAD, DOA, LED индикация
- **STT (Speech-to-Text):**
  - **Vosk** (offline, fast, real-time) — основной выбор
  - **Whisper** (offline, high accuracy) — альтернатива
  - **Yandex SpeechKit** (online, fallback) — для сложных случаев
- **TTS (Text-to-Speech):**
  - **Yandex Cloud TTS** (primary, anton voice) — оригинальный голос ROBBOX
  - **Silero** (offline, fallback) — альтернатива
  - **SSML поддержка** — управление pitch и speed
  - **Динамическая громкость** — голосовые команды управления
- **Dialogue:**
  - **DeepSeek** / **Local LLM** — диалоговый AI агент
- **sound_pack** — звуковые эффекты
- **rob_box_animations** — визуальные анимации

**🎯 Основная стратегия:** Offline-First с fallback на облачные сервисы при необходимости.

**🎤 Новые функции (November 2025):**
- ✅ **Query Queue System** — накопление быстрых запросов и пакетная обработка
- ✅ Оригинальный голос ROBBOX (нормальная скорость, без эффекта "бурундука")
- ✅ SSML управление pitch и speed из dialogue_node
- ✅ Голосовые команды управления громкостью ("громче", "тише", "громко")
- ✅ Опциональный эффект "бурундука" через параметры

## Архитектура

Система состоит из 7 ROS2 нод:

```
AudioNode → STTNode → DialogueNode → TTSNode → SoundNode
    ↓                      ↓              ↓
LEDNode              CommandNode     Animations
```

### Ноды

1. **audio_node** — захват аудио с ReSpeaker, публикация VAD/DOA
2. **stt_node** — Speech-to-Text (Yandex/Whisper)
3. **dialogue_node** — State machine, LLM интеграция, история диалога
4. **tts_node** — Text-to-Speech с кэшированием (Yandex/Coqui)
5. **sound_node** — воспроизведение TTS и звуковых эффектов
6. **led_node** — управление 12× RGB LED на ReSpeaker
7. **command_node** — выполнение голосовых команд управления роботом

## Установка

### 1. Зависимости

```bash
# ROS2 пакеты
sudo apt install ros-jazzy-audio-common-msgs

# Python зависимости
pip install pyaudio sounddevice numpy pyusb grpcio openai pyyaml
```

### 2. ReSpeaker драйверы

```bash
# Клонировать репозиторий
cd ~/
git clone https://github.com/respeaker/usb_4_mic_array.git
cd usb_4_mic_array
sudo pip install -r requirements.txt

# Установить pixel_ring для LED
git clone https://github.com/respeaker/pixel_ring.git
cd pixel_ring
sudo python setup.py install

# Настроить udev правило
sudo cp config/60-respeaker.rules /etc/udev/rules.d/
sudo systemctl restart udev
```

### 3. Загрузка прошивки ReSpeaker

```bash
cd ~/usb_4_mic_array
# Загрузить 1-канальную прошивку (обработанное аудио для STT)
sudo python dfu.py --download 1_channel_firmware.bin
```

### 4. Сборка пакета

```bash
cd ~/rob_box_project
colcon build --packages-select rob_box_voice
source install/setup.bash
```

## Конфигурация

### 🎯 Рекомендуемая настройка (Offline-First)

Для максимальной автономности и минимальной зависимости от интернета:

```yaml
# config/voice_assistant.yaml

stt_node:
  provider: "vosk"  # Основной: быстрый, offline
  vosk:
    model_path: "/models/vosk-model-small-ru-0.22"  # 45 MB
    confidence_threshold: 0.7
  
  # Fallback для низкой уверенности
  fallback_provider: "yandex"
  yandex:
    use_when_confidence_below: 0.7

tts_node:
  provider: "piper"  # Основной: качественный, offline
  piper:
    model_path: "/models/ru_RU-dmitri-medium.onnx"  # 63 MB
    voice_speed: 1.0
  
  # Fallback для важных сообщений
  fallback_provider: "yandex"
  yandex:
    use_for_important: true

dialogue_node:
  llm_provider: "deepseek"  # или "local" для полного offline
```

**Memory footprint:**
- Vosk STT: ~500 MB
- Piper TTS: ~100 MB
- Total: ~1.5 GB (fits в 2GB budget ✅)

**Latency:**
- STT: <1s (real-time)
- TTS: <0.5s
- Total: ~1.5s (отлично для робота!)

---

### Основные параметры

Файл `config/voice_assistant.yaml`:

```yaml
audio_node:
  sample_rate: 16000
  channels: 1
  chunk_size: 1024
  vad_threshold: 3.5
  device_index: 2  # Проверить через pyaudio

dialogue_node:
  activation_phrases: ["роббокс", "робокс", "робо", "робот"]
  silence_timeout: 3.5  # секунды
  max_question_time: 20.0
  llm_provider: "deepseek"  # или "local"
  history_size: 10

stt_node:
  provider: "vosk"  # vosk | whisper | yandex
  language: "ru-RU"
  
tts_node:
  provider: "piper"  # piper | silero | yandex
  voice: "dmitri"  # dmitri (male) | irina (female)
  speed: 1.0
  cache_dir: "/cache/tts"
  cache_enabled: true

led_node:
  brightness: 16  # 0-31
  auto_mode: true
```

### API ключи

Создать `config/secrets.yaml`:

```yaml
yandex_api_key: "YOUR_YANDEX_API_KEY"
deepseek_api_key: "YOUR_DEEPSEEK_API_KEY"
```

**⚠️ Не коммитить secrets.yaml в git!**

## Запуск

### Все ноды (launch file)

```bash
ros2 launch rob_box_voice voice_assistant.launch.py
```

### Отдельные ноды для отладки

```bash
# Захват аудио + VAD/DOA
ros2 run rob_box_voice audio_node

# STT (требует audio_node)
ros2 run rob_box_voice stt_node

# TTS
ros2 run rob_box_voice tts_node

# Диалог + LLM
ros2 run rob_box_voice dialogue_node

# Звук
ros2 run rob_box_voice sound_node

# LED
ros2 run rob_box_voice led_node
```

## Использование

### Активация голосом

1. Скажите фразу активации: **"Роббокс"**
2. LED загорятся (listening mode)
3. Задайте вопрос или команду
4. После паузы (3.5 сек) — начнётся обработка
5. Робот ответит голосом

### Примеры команд

**Диалог:**
```
Пользователь: "Роббокс, какая твоя скорость?"
Робот: "Моя текущая скорость ноль метров в секунду."

Пользователь: "Роббокс, поезжай вперёд"
Робот: "Еду вперёд." [робот начинает движение]

Пользователь: "Роббокс, остановись"
Робот: "Остановился." [emergency stop]
```

**Управление громкостью:**
```
Пользователь: "Роббокс, громче"
Робот: "Делаю громче" [громкость увеличивается на 3 dB]

Пользователь: "Роббокс, тише"
Робот: "Делаю тише" [громкость уменьшается на 3 dB]

Пользователь: "Роббокс, говори громко"
Робот: "Максимальная громкость" [громкость +6 dB]

Пользователь: "Роббокс, нормальная громкость"
Робот: "Нормальная громкость" [громкость -3 dB]
```

**Управление высотой голоса (pitch):**
```
Пользователь: "Роббокс, говори выше"
Робот: "Говорю выше" [pitch_shift увеличивается на 0.2]

Пользователь: "Роббокс, говори ниже"
Робот: "Говорю ниже" [pitch_shift уменьшается на 0.2]

Пользователь: "Роббокс, говори нормально"
Робот: "Нормальный голос" [pitch_shift устанавливается в 2.0 - оригинальный ROBBOX]
```

**Query Queue System (пакетная обработка запросов):**
```
Пользователь: "Роббокс, сколько времени?"
Пользователь: "Какая погода?"
Пользователь: "Что ты умеешь?"
[Все 3 запроса накапливаются в очереди в течение 2.5 секунд]
Робот: "Сейчас 14 часов 30 минут. К сожалению, у меня нет доступа к данным о погоде. 
       Я умею навигировать по помещению, картографировать территорию, отвечать на вопросы 
       и выполнять голосовые команды." [один ответ на все вопросы]
```

**Преимущества Query Queue System:**
- ✅ Актуальные ответы — робот не отвечает на уже неактуальные вопросы
- ✅ Экономия API запросов — несколько вопросов = один запрос к LLM
- ✅ Лучший контекст — LLM видит все вопросы сразу
- ✅ Естественный диалог — можно задавать вопросы не дожидаясь ответа

**Конфигурация накопления:**
```yaml
dialogue_node:
  ros__parameters:
    query_accumulation_timeout: 2.5  # секунд для накопления запросов
```

Подробнее: [docs/QUERY_QUEUE_SYSTEM.md](docs/QUERY_QUEUE_SYSTEM.md)

### Параметры TTS Node

**Оригинальный голос ROBBOX:**
```yaml
tts_node:
  ros__parameters:
    yandex_voice: "anton"      # Оригинальный голос
    yandex_speed: 1.0          # Нормальная скорость (не 0.4!)
    chipmunk_mode: false       # Без эффекта "бурундука"
    pitch_shift: 1.0           # 1.0x = нормальная скорость
    volume_db: -3.0            # -3 dB = 70% громкости
```

**Эффект "бурундука" (опционально):**
```yaml
tts_node:
  ros__parameters:
    chipmunk_mode: true        # Включить эффект
    pitch_shift: 2.0           # 2.0x = 2x быстрее
    yandex_speed: 0.4          # Как в оригинале
```

**Динамическое изменение параметров:**
```bash
# Включить chipmunk mode
ros2 param set /tts_node chipmunk_mode true
ros2 param set /tts_node pitch_shift 2.0

# Изменить громкость
ros2 param set /tts_node volume_db 0.0   # 0 dB = 100%
ros2 param set /tts_node volume_db -6.0  # -6 dB = 50%

# Изменить высоту голоса (pitch через pitch_shift)
ros2 param set /tts_node pitch_shift 2.4  # Выше (более высокий chipmunk)
ros2 param set /tts_node pitch_shift 1.8  # Ниже (менее высокий)
ros2 param set /tts_node pitch_shift 2.0  # Нормально (оригинальный ROBBOX)
ros2 param set /tts_node pitch_shift 1.0  # Без эффекта бурундука
```

### SSML Управление

Dialogue Node может генерировать SSML для управления голосом:

```python
# Быстрая речь
ssml = '<speak><prosody rate="1.5">Я говорю быстро</prosody></speak>'

# Медленная речь
ssml = '<speak><prosody rate="0.7">Я говорю медленно</prosody></speak>'

# Ключевые слова
ssml = '<speak><prosody rate="fast">Очень быстро!</prosody></speak>'
ssml = '<speak><prosody rate="slow">Очень медленно.</prosody></speak>'

# Pitch (логируется, не применяется в Yandex)
ssml = '<speak><prosody pitch="high">Высокий голос</prosody></speak>'
```

См. [ROBBOX_ORIGINAL_VOICE_IMPLEMENTATION.md](../../docs/development/ROBBOX_ORIGINAL_VOICE_IMPLEMENTATION.md) для деталей.

## Топики

### Publications

```
/audio/audio (audio_common_msgs/AudioData) — аудио поток 16kHz
/audio/vad (std_msgs/Bool) — Voice Activity Detection
/audio/direction (std_msgs/Int32) — DoA угол 0-360°
/voice/transcript (std_msgs/String) — распознанный текст
/voice/response (std_msgs/String) — ответ ассистента
/voice/state (std_msgs/String) — состояние диалога
/voice/command (std_msgs/String) — команда управления
```

### Subscriptions

```
/nav_msgs/Odometry — позиция робота (для промпта)
/sensor_msgs/BatteryState — уровень батареи (для промпта)
```

## Сервисы

```
/voice/speak (Speak.srv) — синтез речи с текстом
/voice/set_led_mode (SetLEDMode.srv) — режим LED
/voice/interrupt (std_srvs/Trigger) — прервать текущий ответ
```

## 🔄 Архитектурный рефакторинг (2025)

> **⚠️ В процессе**: Пакет проходит рефакторинг для улучшения поддержки AI-assisted разработки ("vibe coding")

**Проблема**: Большие файлы (2000+ LOC) затрудняют работу с AI ассистентами

**Решение**: Разбиение на модули <300 LOC с четким разделением ответственности

**Документация**:
- [Архитектура Vibe Coding](../../docs/development/VIBE_CODING_ARCHITECTURE.md) - Принципы и паттерны
- [План рефакторинга Voice](../../docs/development/REFACTORING_PLAN_VOICE.md) - Детальный план
- [Quick Reference для AI](../../docs/development/VIBE_CODING_QUICK_REF.md) - Быстрый справочник

**Целевая структура**:
```
rob_box_voice/
├── core/          # Бизнес-логика (без ROS)
├── llm/           # LLM интеграция
├── audio/         # Аудио обработка
├── nodes/         # ROS ноды (тонкие обертки)
└── tests/         # Юнит и интеграционные тесты
```

**Статус**: Планирование завершено, начало реализации - Q1 2025

---

## Разработка

### Структура пакета

```
rob_box_voice/
├── rob_box_voice/
│   ├── __init__.py
│   ├── audio_node.py          # Захват аудио + VAD/DOA
│   ├── stt_node.py             # Speech-to-Text
│   ├── tts_node.py             # Text-to-Speech + кэш
│   ├── dialogue_node.py        # State machine + LLM
│   ├── sound_node.py           # Воспроизведение аудио
│   ├── led_node.py             # Управление LED
│   ├── command_node.py         # Выполнение команд
│   └── utils/
│       ├── audio_utils.py      # Утилиты аудио
│       ├── llm_client.py       # DeepSeek API client
│       └── cache_manager.py    # Кэширование TTS
├── config/
│   ├── voice_assistant.yaml    # Основные параметры
│   └── secrets.yaml.example    # Шаблон для API ключей
├── launch/
│   └── voice_assistant.launch.py
├── prompts/
│   ├── master_prompt.txt       # Основной промпт для LLM
│   └── system_prompts.yaml     # Системные фразы
├── srv/
│   ├── Speak.srv               # Сервис TTS
│   └── SetLEDMode.srv          # Сервис LED
└── README.md
```

### Добавление новых команд

1. Обновить промпт в `prompts/master_prompt.txt`
2. Добавить обработчик в `command_node.py`:

```python
COMMANDS = {
    "my_command": lambda param: execute_my_command(param),
}
```

3. Перезапустить dialogue_node и command_node

## Troubleshooting

### ReSpeaker не распознаётся

```bash
# Проверить USB устройство
lsusb | grep 2886:0018

# Проверить аудио устройство
arecord -l | grep ReSpeaker

# Переподключить udev
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Эхо (слышит сам себя)

- Проверить прошивку: должна быть `1_channel_firmware.bin`
- Проверить AEC параметры:
  ```bash
  python tuning.py AECFREEZEONOFF 0  # Включить адаптацию AEC
  python tuning.py ECHOONOFF 1       # Включить подавление эха
  ```

### Медленный ответ LLM

- Использовать streaming: уже реализовано в dialogue_node
- Переключиться на локальный LLM (Ollama + LLaMA)
- Уменьшить `history_size`

## Лицензия

MIT License

## Автор

krikz @ РОББОКС Project

## См. также

- [Документация ReSpeaker](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [Архитектура Voice Assistant](docs/VOICE_ASSISTANT_ARCHITECTURE.md)
- [Спецификация hardware](../../docs/architecture/HARDWARE.md#34-respeaker-microphone-array-v20)
