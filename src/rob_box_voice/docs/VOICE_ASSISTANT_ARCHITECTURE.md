# Архитектура голосового ассистента РОББОКС

<div align="center">
  <strong>Проектирование AI-агента для голосового управления роботом</strong>
  <br>
  <sub>Версия: 0.1.0 | Дата: 2025-10-12</sub>
</div>

---

## 📖 Содержание

1. [Обзор и цели](#1-обзор-и-цели)
2. [Анализ текущего решения](#2-анализ-текущего-решения)
3. [Архитектура ROS2 системы](#3-архитектура-ros2-системы)
4. [Компоненты системы](#4-компоненты-системы)
5. [Интеграция с hardware](#5-интеграция-с-hardware)
6. [Промптинг и LLM](#6-промптинг-и-llm)
7. [Звуковые эффекты и анимации](#7-звуковые-эффекты-и-анимации)
8. [План реализации](#8-план-реализации)

---

## 1. Обзор и цели

### 1.1. Назначение

Голосовой ассистент РОББОКС — это AI-агент, который:
- 🗣️ **Понимает** голосовые команды на русском языке
- 🤖 **Управляет** роботом через ROS2 топики и сервисы
- 💬 **Общается** с пользователем используя LLM (DeepSeek/local)
- 🎭 **Выражает эмоции** через LED анимации и звуковые эффекты
- 🧭 **Реагирует на окружение** используя сенсоры (DOA, VAD)

### 1.2. Отличия от прототипа

**Проблемы прототипа:**
- ❌ Слышит сам себя (self-listening) → циклы обратной связи
- ❌ Не интегрирован с ROS2 → нет доступа к датчикам и управлению
- ❌ Монолитный скрипт → сложно поддерживать и расширять
- ❌ Блокирующий I/O → медленная реакция
- ❌ Нет кэширования TTS → лишние запросы к Yandex API

**Новая архитектура:**
- ✅ Использует AEC (Acoustic Echo Cancellation) ReSpeaker
- ✅ Модульная ROS2 архитектура с нодами
- ✅ Асинхронная обработка через топики/сервисы
- ✅ Локальный кэш предгенерированных фраз
- ✅ Интеграция с sound_pack и rob_box_animations
- ✅ Доступ к состоянию робота через ROS2 topics

---

## 2. Анализ текущего решения

### 2.1. Структура прототипа

```
Прототип (монолитный скрипт)
├── Yandex STT (gRPC stream) → распознавание речи
├── DeepSeek API (streaming) → генерация ответов
├── Yandex TTS (gRPC) → синтез речи
├── PyAudio → захват микрофона
├── sounddevice → воспроизведение
├── gpiozero LED → индикация состояния
└── State Machine (3 состояния)
    ├── IDLE → ожидание активации
    ├── LISTENING → сбор вопроса
    └── GENERATING → генерация и озвучка
```

### 2.2. Ключевые концепции (сохранить)

**✅ Хорошо работает:**
1. **Фразы активации** — `["роббокс", "робокс", "робо", "робот"]`
2. **Таймауты**:
   - Тишина: 3.5 сек → конец вопроса
   - Максимальное время: 20 сек → принудительный конец
3. **История диалога** — `queue.Queue(maxsize=10)` с парами user/assistant
4. **Потоковый TTS** — разбивка ответа по предложениям для низкой латентности
5. **Мастер-промпт** — детальное описание робота и правил общения
6. **Голос Yandex TTS** — `anton`, `speed=0.4` (хорошо звучит)

**❌ Требует улучшения:**
1. **Self-listening** — нет изоляции воспроизведения от записи
2. **Модульность** — всё в одном файле
3. **ROS2 интеграция** — нет доступа к топикам робота
4. **Кэширование** — каждый раз генерирует TTS
5. **LED управление** — использует GPIO напрямую, не синхронизировано с анимациями

---

## 3. Архитектура ROS2 системы

### 3.1. Общая схема

```
┌────────────────────────────────────────────────────────────────┐
│                      ROS2 Graph (Vision Pi)                    │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │             rob_box_voice (Python Package)               │  │
│  │                                                           │  │
│  │  ┌─────────────┐   ┌─────────────┐   ┌──────────────┐  │  │
│  │  │ AudioNode   │   │   STTNode   │   │   TTSNode    │  │  │
│  │  │             │──>│             │──>│              │  │  │
│  │  │ (ReSpeaker) │   │  (Yandex/   │   │  (Yandex/    │  │  │
│  │  │  VAD, DOA   │   │   Whisper)  │   │   Coqui)     │  │  │
│  │  └─────────────┘   └─────────────┘   └──────────────┘  │  │
│  │         │                  │                  │          │  │
│  │         └──────────────────┼──────────────────┘          │  │
│  │                            ▼                             │  │
│  │                   ┌─────────────────┐                    │  │
│  │                   │  DialogueNode   │                    │  │
│  │                   │                 │                    │  │
│  │                   │  • State Machine│                    │  │
│  │                   │  • LLM (DeepSeek)│                   │  │
│  │                   │  • History      │                    │  │
│  │                   │  • Commands     │                    │  │
│  │                   └─────────────────┘                    │  │
│  │                            │                             │  │
│  │         ┌──────────────────┼──────────────────┐          │  │
│  │         ▼                  ▼                  ▼          │  │
│  │  ┌─────────────┐   ┌─────────────┐   ┌──────────────┐  │  │
│  │  │  LEDNode    │   │ SoundNode   │   │ CommandNode  │  │  │
│  │  │             │   │             │   │              │  │  │
│  │  │ (ReSpeaker  │   │ (sound_pack)│   │ (Nav2, etc)  │  │  │
│  │  │  12× RGB)   │   │  + TTS out  │   │              │  │  │
│  │  └─────────────┘   └─────────────┘   └──────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Topics:                                                        │
│    /audio/audio (AudioData)          ← from AudioNode         │
│    /audio/vad (Bool)                 ← from AudioNode         │
│    /audio/direction (Int32)          ← from AudioNode         │
│    /voice/transcript (String)        ← from STTNode           │
│    /voice/response (String)          ← from DialogueNode      │
│    /voice/state (String)             ← from DialogueNode      │
│    /voice/command (String)           ← from DialogueNode      │
│    /animations/trigger (Animation)   → to AnimationNode       │
│                                                                 │
│  Services:                                                      │
│    /voice/speak (Speak)              → TTSNode                │
│    /voice/set_led_mode (SetLEDMode)  → LEDNode                │
│    /voice/interrupt (Trigger)        → DialogueNode           │
└────────────────────────────────────────────────────────────────┘
```

### 3.2. Lifecycle управление

Используем **managed nodes** (rclpy.lifecycle) для корректной инициализации:

```
unconfigured → configuring → inactive → activating → active
                                  ↑            ↓
                                  └───cleanup──┘
```

**Порядок запуска:**
1. AudioNode → инициализация ReSpeaker, загрузка прошивки
2. LEDNode → подключение к USB control interface
3. STTNode → проверка credentials (Yandex API key)
4. TTSNode → загрузка кэша, проверка API
5. SoundNode → загрузка sound_pack
6. DialogueNode → загрузка промпта, инициализация LLM
7. CommandNode → подписка на топики управления

---

## 4. Компоненты системы

### 4.1. AudioNode (rob_box_voice/audio_node.py)

**Назначение:** Захват аудио, VAD, DOA

**Зависимости:**
- `pyaudio` — захват с ReSpeaker
- `usb_4_mic_array` (tuning.py) — чтение VAD/DOA

**Publishers:**
```python
/audio/audio (audio_common_msgs/AudioData)  # 16kHz, 1ch, обработанный
/audio/vad (std_msgs/Bool)                   # Voice Activity Detection
/audio/direction (std_msgs/Int32)            # DoA angle 0-360°
/audio/speech_detected (std_msgs/Bool)       # Trigger для STT
```

**Parameters:**
```yaml
sample_rate: 16000
channels: 1  # обработанный канал
chunk_size: 4096  # frames per buffer (issue #1050: 1024 → 4096, paInputOverflow)
vad_threshold: 3.5  # dB (GAMMAVAD_SR)
publish_rate: 10  # Hz для VAD/DOA
```

**Алгоритм:**
1. Открыть PyAudio stream на ReSpeaker (device index)
2. Запустить поток чтения (16kHz, 16-bit PCM)
3. Публиковать `/audio/audio` постоянно
4. Опрашивать `tuning.py VOICEACTIVITY` → `/audio/vad`
5. Опрашивать `tuning.py DOAANGLE` → `/audio/direction`
6. При `vad=True` → публиковать `/audio/speech_detected`

**Предотвращение self-listening:**
- Использовать прошивку `1_channel_firmware.bin` (встроенный AEC)
- Дополнительно: подписаться на `/voice/state` и игнорировать VAD во время `SPEAKING`

---

### 4.2. STTNode (rob_box_voice/stt_node.py)

**Назначение:** Speech-to-Text (потоковый)

**Providers:**
- **Yandex Cloud STT v3** (по умолчанию, из прототипа)
- **Whisper** (локальный, для оффлайн режима)

**Subscribers:**
```python
/audio/audio (audio_common_msgs/AudioData)    # аудио поток
/audio/speech_detected (std_msgs/Bool)        # триггер начала
```

**Publishers:**
```python
/voice/transcript (std_msgs/String)  # распознанный текст (partial + final)
```

**Parameters:**
```yaml
provider: "yandex"  # или "whisper"
yandex_api_key: "${YANDEX_API_KEY}"
language: "ru-RU"
model: "general"  # или "general:rc"
```

**Алгоритм (Yandex STT):**
1. Подписаться на `/audio/audio`
2. Буферизовать аудио в памяти (ring buffer)
3. При `/audio/speech_detected=True`:
   - Открыть gRPC stream к `stt.api.cloud.yandex.net:443`
   - Отправить `StreamingOptions` (настройки)
   - Стримить аудио чанки
4. Получать `partial` и `final` результаты
5. Публиковать `/voice/transcript` для каждого результата
6. При `vad=False` в течение `silence_timeout` → закрыть stream

---

### 4.3. DialogueNode (rob_box_voice/dialogue_node.py)

**Назначение:** Машина состояний, LLM, управление диалогом

**State Machine:**
```
      ┌─────┐
      │IDLE │ ◄──────────────────┐
      └──┬──┘                    │
         │ activation phrase    │ timeout / command done
         ▼                       │
   ┌──────────┐                 │
   │LISTENING │                 │
   └────┬─────┘                 │
        │ silence timeout       │
        ▼                       │
   ┌───────────┐                │
   │GENERATING │                │
   └────┬──────┘                │
        │ LLM done              │
        ▼                       │
   ┌──────────┐                 │
   │SPEAKING  │─────────────────┘
   └──────────┘
```

**Subscribers:**
```python
/voice/transcript (std_msgs/String)   # от STTNode
/voice/state (String)                 # feedback от SoundNode
```

**Publishers:**
```python
/voice/response (std_msgs/String)     # ответ для TTS
/voice/command (String)               # команда для CommandNode
/voice/state (String)                 # текущее состояние
```

**Services (server):**
```python
/voice/interrupt (std_srvs/Trigger)  # прервать текущий ответ
```

**Parameters:**
```yaml
activation_phrases: ["роббокс", "робокс", "робо", "робот"]
silence_timeout: 3.5  # секунды
max_question_time: 20.0  # секунды
llm_provider: "deepseek"  # или "local"
deepseek_api_key: "${DEEPSEEK_API_KEY}"
model: "deepseek-chat"
history_size: 10  # пар user/assistant
```

**Алгоритм:**
1. **IDLE state:**
   - Слушать `/voice/transcript`
   - Если содержит activation_phrase → LISTENING
   - Опубликовать `/voice/state="listening"`
   - Послать `/voice/speak` с текстом "Слушаю"

2. **LISTENING state:**
   - Накапливать `/voice/transcript` в буфер
   - Таймер `silence_timeout` сбрасывается при новом транскрипте
   - Если таймаут истёк → GENERATING
   - Если время > `max_question_time` → GENERATING

3. **GENERATING state:**
   - Опубликовать `/voice/state="thinking"`
   - Послать вопрос в LLM (DeepSeek API с историей)
   - Стримить ответ, разбивать по предложениям
   - Для каждого предложения → публиковать `/voice/response`
   - Сохранить в историю диалога
   - → SPEAKING

4. **SPEAKING state:**
   - Ждать окончания TTS (callback от SoundNode)
   - → IDLE

**LLM Integration (DeepSeek):**
```python
from openai import OpenAI
client = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")

messages = [
    {"role": "system", "content": MASTER_PROMPT},
    *conversation_history,  # последние 10 пар
    {"role": "user", "content": question}
]

response = client.chat.completions.create(
    model="deepseek-chat",
    messages=messages,
    stream=True
)

sentence_buffer = ""
for chunk in response:
    content = chunk.choices[0].delta.content
    if content:
        sentence_buffer += content
        if any(p in content for p in ['.', '!', '?']):
            sentences = re.split(r'(?<=[.!?])', sentence_buffer)
            for sentence in sentences[:-1]:
                pub_response.publish(String(data=sentence.strip()))
            sentence_buffer = sentences[-1]
```

**Команды управления:**
Если LLM возвращает специальные теги → парсим и публикуем:
```
Ответ: "Еду вперёд <CMD:move_forward speed=0.5/>"
→ Публикуем /voice/response="Еду вперёд"
→ Публикуем /voice/command="move_forward speed=0.5"
```

---

### 4.4. TTSNode (rob_box_voice/tts_node.py)

**Назначение:** Text-to-Speech

**Providers:**
- **Yandex Cloud TTS v3** (по умолчанию)
- **Coqui TTS** (локальный, для оффлайн)

**Subscribers:**
```python
/voice/response (std_msgs/String)  # текст для озвучки
```

**Services (server):**
```python
/voice/speak (Speak.srv)  # запрос синтеза с текстом
```

**Service definition (Speak.srv):**
```
string text
bool cache  # использовать кэш если доступно
---
bool success
string audio_path  # путь к WAV файлу
```

**Parameters:**
```yaml
provider: "yandex"
yandex_api_key: "${YANDEX_API_KEY}"
voice: "anton"
speed: 0.4
cache_dir: "~/.cache/rob_box_voice/tts"
pregenerate: true  # загрузить системные фразы при старте
```

**Алгоритм:**
1. При инициализации:
   - Создать `cache_dir`
   - Если `pregenerate=true` → сгенерировать системные фразы:
     - "Слушаю", "Дай подумаю", "Не понял", "Готово", и т.д.

2. При получении `/voice/response` или сервиса `/voice/speak`:
   - Проверить кэш: `hash(text) → cache_file.wav`
   - Если найден → вернуть путь
   - Если нет:
     - Синтезировать через Yandex TTS gRPC
     - Сохранить в кэш
     - Вернуть путь

3. Передать путь к аудио в SoundNode через внутренний топик:
   - `/voice/audio_ready (String)` с путём к файлу

---

### 4.5. SoundNode (rob_box_voice/sound_node.py)

**Назначение:** Воспроизведение TTS и звуковых эффектов

**Subscribers:**
```python
/voice/audio_ready (std_msgs/String)  # путь к TTS аудио
/voice/sound_effect (std_msgs/String) # имя эффекта из sound_pack
```

**Publishers:**
```python
/voice/playback_state (std_msgs/String)  # "playing" | "idle"
```

**Parameters:**
```yaml
sound_pack_dir: "~/rob_box_project/sound_pack"
output_device: "plughw:CARD=ArrayUAC10,DEV=0"  # ReSpeaker 3.5mm jack
```

**Алгоритм:**
1. При `/voice/audio_ready`:
   - Загрузить WAV файл
   - Воспроизвести через sounddevice (блокирующий)
   - Опубликовать `/voice/playback_state="playing"`
   - По завершении → опубликовать `"idle"`

2. При `/voice/sound_effect`:
   - Загрузить из `sound_pack/{name}.mp3`
   - Конвертировать в WAV (если нужно)
   - Воспроизвести

**Синхронизация с анимациями:**
- Публиковать `/animations/trigger` перед началом воспроизведения:
  ```python
  # Перед TTS
  pub_anim.publish(Animation(name="talk_1", loop=True))
  # После TTS
  pub_anim.publish(Animation(name="idle", loop=True))
  ```

---

### 4.6. LEDNode (rob_box_voice/led_node.py)

**Назначение:** Управление 12× RGB LED на ReSpeaker

**Subscribers:**
```python
/voice/state (std_msgs/String)      # состояние диалога
/audio/direction (std_msgs/Int32)   # DoA для trace mode
```

**Services (server):**
```python
/voice/set_led_mode (SetLEDMode.srv)  # ручная установка режима
```

**Service definition (SetLEDMode.srv):**
```
string mode  # "trace" | "listen" | "think" | "speak" | "off"
---
bool success
```

**Parameters:**
```yaml
brightness: 16  # 0-31
auto_mode: true  # автоматически менять по /voice/state
```

**Алгоритм:**
1. Инициализация:
   ```python
   from pixel_ring import pixel_ring
   dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
   pixel_ring.set_brightness(brightness)
   ```

2. При изменении `/voice/state`:
   - `"idle"` → `pixel_ring.off()`
   - `"listening"` → `pixel_ring.listen()`
   - `"thinking"` → `pixel_ring.think()`
   - `"speaking"` → `pixel_ring.speak()`

3. В режиме `"trace"`:
   - Подписаться на `/audio/direction`
   - Вызывать `pixel_ring.set_direction(angle)`

---

### 4.7. CommandNode (rob_box_voice/command_node.py)

**Назначение:** Выполнение голосовых команд управления

**Subscribers:**
```python
/voice/command (std_msgs/String)  # команда от DialogueNode
```

**Clients (service/action):**
```python
/nav2/navigate_to_pose (NavigateToPose.action)
/set_velocity (Twist)
/emergency_stop (std_srvs/Trigger)
# и т.д.
```

**Алгоритм:**
1. Парсить `/voice/command` (формат `"action param=value"`)
2. Маппинг команд:
   ```python
   COMMANDS = {
       "move_forward": lambda speed: publish_twist(linear=speed),
       "turn_left": lambda angle: publish_twist(angular=angle),
       "stop": lambda: call_emergency_stop(),
       "go_to": lambda x, y: send_nav_goal(x, y),
       # ...
   }
   ```
3. Выполнить соответствующий ROS2 вызов

---

## 5. Интеграция с hardware

### 5.1. ReSpeaker Mic Array v2.0

**Подключение:** USB 2.0 → Vision Pi 5

**Конфигурация при запуске:**
```bash
# Загрузка прошивки (если нужно)
python dfu.py --download 1_channel_firmware.bin

# Настройка параметров
python tuning.py AGCONOFF 1          # AGC ON
python tuning.py AGCMAXGAIN 30       # Max gain 30dB
python tuning.py STATNOISEONOFF 1    # Noise suppression ON
python tuning.py GAMMAVAD_SR 3.5     # VAD threshold
```

**Udev правило** (`/etc/udev/rules.d/60-respeaker.rules`):
```bash
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
```

---

### 5.2. Аудио маршрутизация

```
                        ┌─────────────────┐
   Микрофоны (4×) ─────>│ XMOS XVF-3000   │
                        │  (AEC, VAD,DoA) │
                        └────────┬─────────┘
                                 │
                        ┌────────▼─────────┐
                        │  USB Audio       │
                        │  (UAC 1.0)       │
                        └────────┬─────────┘
                                 │
           ┌─────────────────────┼─────────────────────┐
           │                     │                     │
           ▼                     ▼                     ▼
    ┌────────────┐        ┌───────────┐        ┌───────────┐
    │ AudioNode  │        │  STTNode  │        │  (future) │
    │ (PyAudio)  │        │ (stream)  │        │           │
    └────────────┘        └───────────┘        └───────────┘

                        ┌─────────────────┐
   TTS/Звуки ──────────>│  SoundNode      │
                        │  (sounddevice)  │
                        └────────┬─────────┘
                                 │
                        ┌────────▼─────────┐
                        │  USB Audio OUT   │
                        │  → WM8960 codec  │
                        └────────┬─────────┘
                                 │
                        ┌────────▼─────────┐
                        │  3.5mm jack      │────> Динамик
                        └──────────────────┘
```

**ALSA конфигурация** (`~/.asoundrc`):
```
pcm.respeaker_input {
    type plug
    slave.pcm "hw:ArrayUAC10,0"
}

pcm.respeaker_output {
    type plug
    slave.pcm "hw:ArrayUAC10,0"
}
```

---

## 6. Промптинг и LLM

### 6.1. Мастер-промпт (сохранён из прототипа)

**Файл:** `rob_box_voice/prompts/master_prompt.txt`

```
Вы — интеллектуальный агент, управляющий автономным колесным ровером РОББОКС.

Характеристики робота:
- Имя: РОББОКС
- Тип: Дифференциальный привод, 4 колеса 10" от гироскутера
- Драйверы: 2× ODrive 3.6
- Полетный контроллер: Matek H743-Slim (ArduPilot Rover)
- Бортовые компьютеры: 2× Raspberry Pi 5 (Vision 8GB, Main 16GB)
- ОС: Ubuntu 24.04.2 LTS + ROS2 Jazzy
- Навигация: LS LiDAR N10 (2D SLAM), OAK-D Lite (RGB-D)
- Визуализация: 5× LED Matrix 8×8 (NeoPixel)
- Голос: ReSpeaker Mic Array v2.0 (4 микрофона, 12 LED)
- Телеметрия: ESP32 с DroneBridge

Ваши возможности:
1. Принимать голосовые команды и вопросы
2. Управлять движением через ROS2 Nav2
3. Отображать эмоции через LED матрицы и анимации
4. Сообщать о состоянии сенсоров (LiDAR, камера, батарея)
5. Выполнять автономную навигацию по карте

Правила общения:
- Отвечайте чётко, кратко и по-русски
- Используйте полные слова (не "т.д.", а "так далее")
- Пишите как говорите — простыми фразами
- Если не можете выполнить — скажите честно
- НЕ используйте эмодзи и сложные знаки препинания
- Тон: спокойный, дружелюбный, профессиональный

Команды управления:
Если нужно управлять роботом, используйте теги:
- <CMD:move_forward speed=0.5/> — ехать вперёд
- <CMD:turn_left angle=90/> — повернуть налево
- <CMD:stop/> — остановиться
- <CMD:go_to x=1.5 y=2.0/> — поехать к точке
- <CMD:follow_me/> — следовать за человеком

Пример:
Пользователь: "Роббокс, поезжай к окну"
Ассистент: "Еду к окну. <CMD:go_to x=3.0 y=1.5/>"
```

### 6.2. Системные промпты

**Файл:** `rob_box_voice/prompts/system_prompts.yaml`

```yaml
# Короткие ответы для частых ситуаций
quick_responses:
  greeting: "Привет! Я Роббокс, чем могу помочь?"
  listening: "Слушаю."
  thinking: "Дай подумаю."
  done: "Готово."
  error: "Извините, что-то пошло не так."
  not_understood: "Не понял, повторите пожалуйста."
  offline: "Сервис недоступен, работаю в локальном режиме."

# Описания состояния для промпта
robot_state_template: |
  Текущее состояние робота:
  - Батарея: {battery_percent}%
  - Позиция: x={x:.2f}м, y={y:.2f}м, θ={theta:.1f}°
  - Скорость: {velocity:.2f} м/с
  - LiDAR: {obstacles_count} препятствий обнаружено
  - Режим: {mode}
```

---

## 7. Звуковые эффекты и анимации

### 7.1. Интеграция sound_pack

**Доступные звуки:**
```
sound_pack/
├── angry_1.mp3       → Недовольство
├── angry_2.mp3       → Раздражение
├── confused.mp3      → Непонимание
├── cute.mp3          → Милый звук
├── surprise.mp3      → Удивление
├── talk_1.mp3        → Звук разговора 1
├── talk_2.mp3        → Звук разговора 2
├── talk_3.mp3        → Звук разговора 3
├── talk_4.mp3        → Звук разговора 4
├── thinking.mp3      → Обдумывание
└── very_cute.mp3     → Очень милый звук
```

**Использование:**
```python
# В DialogueNode при определённых событиях
pub_sound.publish(String(data="thinking.mp3"))  # Перед LLM запросом
pub_sound.publish(String(data="confused.mp3"))  # Если не понял команду
pub_sound.publish(String(data="cute.mp3"))      # При выполнении задачи
```

---

### 7.2. Синхронизация с анимациями

**Анимации из rob_box_animations:**
- `idle` — спокойное состояние
- `talk_1` — рот открывается/закрывается (для TTS)
- `listening` — внимание
- `thinking` — обдумывание
- `angry` — недовольство
- `happy` — радость

**Интеграция:**
```python
# В SoundNode перед воспроизведением TTS
pub_anim.publish(Animation(name="talk_1", loop=True, speed=1.0))

# После завершения TTS
pub_anim.publish(Animation(name="idle", loop=True, speed=1.0))

# При звуковых эффектах
sound_to_animation = {
    "thinking.mp3": "thinking",
    "confused.mp3": "confused",
    "angry_1.mp3": "angry",
    "cute.mp3": "happy",
}
```

---

### 7.3. Кэширование системных фраз

**Предгенерированные фразы:**
```python
SYSTEM_PHRASES = [
    "Слушаю",
    "Дай подумаю",
    "Готово",
    "Не понял",
    "Повторите пожалуйста",
    "Выполняю",
    "Остановился",
    "Еду вперёд",
    "Поворачиваю",
    "Приехал",
    "Батарея разряжена",
    "Препятствие обнаружено",
]

# При инициализации TTSNode
for phrase in SYSTEM_PHRASES:
    if not cached(phrase):
        audio = synthesize_tts(phrase)
        save_cache(phrase, audio)
```

**Формат кэша:**
```
~/.cache/rob_box_voice/tts/
├── 3a7f8b12_слушаю.wav
├── 9d2e4c5a_дай_подумаю.wav
├── f1b3c7e8_готово.wav
└── ...
```

---

## 8. План реализации

### Phase 1: Базовая инфраструктура (Week 1-2)
- [x] ✅ Создать feature branch `feature/voice-assistant`
- [x] ✅ Обновить docs/HARDWARE.md с ReSpeaker
- [ ] 🔄 Создать пакет `rob_box_voice`
- [ ] Настроить ReSpeaker на Vision Pi (udev, firmware)
- [ ] Реализовать AudioNode (захват, VAD, DoA)
- [ ] Реализовать LEDNode (pixel_ring интеграция)

### Phase 2: STT/TTS (Week 3-4)
- [ ] Реализовать STTNode (Yandex Cloud STT)
- [ ] Реализовать TTSNode (Yandex Cloud TTS + кэш)
- [ ] Реализовать SoundNode (воспроизведение через ReSpeaker)
- [ ] Тестирование end-to-end: Audio → STT → TTS → Sound

### Phase 3: Диалог и LLM (Week 5-6)
- [ ] Реализовать DialogueNode (state machine)
- [ ] Интеграция с DeepSeek API
- [ ] История диалога (10 пар user/assistant)
- [ ] Потоковый вывод с разбивкой по предложениям
- [ ] Активация по фразам ["роббокс", "робокс"]

### Phase 4: Команды и интеграция (Week 7-8)
- [ ] Реализовать CommandNode (парсинг и выполнение)
- [ ] Интеграция с Nav2 (go_to, stop, follow)
- [ ] Интеграция с rob_box_animations
- [ ] Интеграция с sound_pack
- [ ] Синхронизация LED + анимации + звук

### Phase 5: Оптимизация (Week 9-10)
- [ ] Локальный STT (Whisper.cpp)
- [ ] Локальный TTS (Coqui TTS)
- [ ] Локальный LLM (Ollama, LLaMA)
- [ ] Кэширование всех системных фраз
- [ ] Системные меню (voice UI)

### Phase 6: Тестирование (Week 11-12)
- [ ] Unit тесты для каждого Node
- [ ] Integration тесты (ROS2 bag replay)
- [ ] Полевое тестирование (реальный робот)
- [ ] Документация (README, examples, tutorials)

---

## 9. Дополнительные возможности (Future)

### 9.1. Мультиязычность
- Английский язык (en-US)
- Автоопределение языка

### 9.2. Эмоциональный ответ
- Анализ тональности вопроса
- Выбор интонации TTS
- Соответствующие анимации и звуки

### 9.3. Визуальный контекст
- Передача изображения с OAK-D в LLM (vision model)
- "Что ты видишь?" → описание сцены

### 9.4. Персонализация
- Распознавание голоса пользователя
- История предпочтений
- Адаптация промпта

---

## 10. Ссылки

- [ReSpeaker Mic Array v2.0 Wiki](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [usb_4_mic_array GitHub](https://github.com/respeaker/usb_4_mic_array)
- [pixel_ring GitHub](https://github.com/respeaker/pixel_ring)
- [respeaker_ros (ROS1)](https://github.com/furushchev/respeaker_ros)
- [Yandex SpeechKit API](https://cloud.yandex.ru/docs/speechkit/)
- [DeepSeek API](https://api-docs.deepseek.com/)
- [ROS2 Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

---

<div align="center">
  <sub>Разработка: krikz | РОББОКС Project</sub>
</div>
