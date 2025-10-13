# Voice Assistant Architecture Overview

## Полная архитектура системы

### Nodes (8 штук)

| # | Node | Package | Описание | Phase |
|---|------|---------|----------|-------|
| 1 | `audio_node` | rob_box_voice | Захват микрофона | 1 |
| 2 | `led_node` | rob_box_voice | LED индикатор состояния | 1 |
| 3 | `animation_player` | rob_box_animations | Воспроизведение анимаций | 1 |
| 4 | `dialogue_node` | rob_box_voice | LLM диалог (DeepSeek) | 2 |
| 5 | `tts_node` | rob_box_voice | Синтез речи (Silero TTS) | 2 |
| 6 | `stt_node` | rob_box_voice | Распознавание речи (Vosk) | 3 |
| 7 | `sound_node` | rob_box_voice | Звуковые эффекты (MP3) | 4 |
| 8 | `command_node` | rob_box_voice | Управление роботом (Nav2) | 5 |

### Topics Flow

```
🎤 Microphone
  ↓
/voice/audio/recording (AudioData)
  ↓
🗣️ STT Node
  ↓
/voice/stt/result (String) ──────┬─────────────┐
  ↓                              ↓             ↓
💬 Dialogue Node          🎯 Command Node   (Other)
  ↓                              ↓
  ├→ /voice/dialogue/response    └→ NavigateToPose (Action)
  ├→ /voice/sound/trigger               ↓
  │                              🤖 Nav2 Stack
  ↓                                     ↓
🔊 TTS Node                       /cmd_vel (Twist)
  ↓                                     ↓
/voice/audio/speech (AudioData)  🎮 Twist Mux
  ↓                                     ↓
🔊 Audio Output                   /diff_cont/cmd_vel_unstamped
  ↓                                     ↓
🎤 Audio Reactive Node            ⚙️ ros2_control
  ↓                                     ↓
/audio/level (Float32)            🔧 VESC Motors
  ↓                                     ↓
🎨 Animation Player               🚗 Robot Moves
  ↓
LED Matrix (Mouth, Eyes)
```

## Критичные моменты

### 1. Twist Mux - Приоритеты команд

**⚠️ Command Node НЕ публикует напрямую в `/cmd_vel`!**

**Поток**:
```
Command Node → Nav2 Action → Nav2 Controller → /cmd_vel → Twist Mux → Motors
```

**Приоритеты** (`docker/main/config/twist_mux/twist_mux.yaml`):
```yaml
topics:
  - name: emergency_stop
    topic: cmd_vel_emergency
    priority: 255  # HIGHEST
    
  - name: joystick
    topic: cmd_vel_joy
    priority: 100  # HIGH (blocks Nav2!)
    
  - name: web_ui
    topic: cmd_vel_web
    priority: 50
    
  - name: navigation
    topic: cmd_vel
    priority: 10   # LOWEST (autonomous)
```

**Логика**:
- Джойстик активен → Nav2 блокируется
- Джойстик timeout (0.5s) → Nav2 возобновляется
- Emergency stop → все блокируются

### 2. Audio-Reactive Animations

**Рот робота двигается синхронно с речью!**

**Поток**:
```
TTS → Audio Output → PyAudio Loopback → RMS Volume → /audio/level → Frame Selection → LED Matrix
```

**Механизм**:
1. `audio_reactive_animation_node` захватывает аудио выход (loopback)
2. Вычисляет RMS громкость (0.0-1.0)
3. Публикует в `/audio/level`
4. `animation_player_node` выбирает кадр: `frame_index = int(audio_level * 11)`
5. LED матрица показывает соответствующий кадр (рот открыт/закрыт)

**Конфигурация** (`animations/manifests/talking.yaml`):
```yaml
mouth_panel:
  audio_controlled: true  # ВАЖНО!
  frames:
    - frame_0.png   # Рот закрыт (silence)
    - frame_1.png
    ...
    - frame_11.png  # Рот открыт (loud)
```

**Результат**: Эффект "говорящего робота" (как Bender из Futurama)

### 3. Эмоции в DeepSeek ответах

**Формат** (`prompts/master_prompt_simple.txt`):
```json
{
  "chunk": 1,
  "ssml": "<speak>Текст</speak>",
  "emotion": "happy"  ← Поле эмоции
}
```

**Поддерживаемые эмоции**:
- `neutral` - Нейтральное состояние
- `happy` - Радость
- `sad` - Грусть
- `thinking` - Размышление
- `alert` - Тревога
- `angry` - Злость
- `surprised` - Удивление

**Текущее состояние**: DeepSeek генерирует эмоции, но `dialogue_node` их пока не обрабатывает.

**TODO (Phase 6)**: Добавить в `dialogue_node`:
```python
def dialogue_callback(self, msg: String):
    chunk_data = json.loads(msg.data)
    
    # Текущее
    ssml = chunk_data['ssml']
    self.response_pub.publish(...)
    
    # TODO
    if 'emotion' in chunk_data:
        emotion = chunk_data['emotion']
        # Триггер анимации
        anim_msg = String()
        anim_msg.data = emotion  # "happy", "sad", etc.
        self.animation_trigger_pub.publish(anim_msg)
        
        # Триггер звука
        sound_msg = String()
        sound_msg.data = self._emotion_to_sound(emotion)  # "cute", "angry_1"
        self.sound_trigger_pub.publish(sound_msg)
```

**Mapping** (из Phase 4):
```python
emotion_to_sound = {
    'happy': ['cute', 'very_cute'],
    'angry': ['angry_1', 'angry_2'],
    'thinking': ['thinking'],
    'surprised': ['surprise'],
    'sad': ['confused']
}

emotion_to_animation = {
    'happy': 'happy',        # animations/manifests/happy.yaml
    'angry': 'angry',        # animations/manifests/angry.yaml
    'thinking': 'thinking',  # animations/manifests/thinking.yaml
    'surprised': 'surprised',
    'sad': 'sad',
    'neutral': 'eyes_neutral'
}
```

## Интеграция систем

### Voice Command → Robot Motion (end-to-end)

**Сценарий**: Пользователь говорит "Двигайся к точке три"

1. **Audio Node**: Захват микрофона → `/voice/audio/recording`
2. **STT Node**: Vosk → "двигайся к точке три" → `/voice/stt/result`
3. **Command Node**: 
   - Regex: Intent=NAVIGATE, waypoint="точка 3"
   - Lookup: `{x: 3.0, y: 0.0, theta: 0.0}`
   - Send Nav2 Goal: `NavigateToPose(x=3.0, y=0.0)`
   - Feedback: "Иду к точка 3" → `/voice/command/feedback`
4. **Dialogue Node**: Получает feedback → отправляет в TTS
5. **TTS Node**: Silero → "Иду к точка 3" → аудио выход
6. **Audio Reactive Node**: Мониторинг → `/audio/level` (0.0-1.0)
7. **Animation Player**: Выбор кадра → LED рот двигается
8. **Nav2 Stack**:
   - Planner: A* путь на карте
   - Controller: DWB генерирует команды скорости
   - Publishes: `/cmd_vel` (Twist)
9. **Twist Mux**:
   - Проверяет приоритеты (джойстик неактивен)
   - Пропускает Nav2 команды → `/diff_cont/cmd_vel_unstamped`
10. **ros2_control**: Differential drive → VESC моторы
11. **Робот едет к точке (3.0, 0.0), рот двигается при речи**
12. **Nav2 Result**: SUCCESS → Feedback: "Прибыл в точку назначения"
13. **TTS говорит "Прибыл..." → рот двигается снова**

### Блокировка джойстиком

**Сценарий**: Во время автономной навигации пользователь берёт джойстик

1. Nav2 публикует в `/cmd_vel` (priority 10)
2. Twist Mux пропускает → робот едет
3. **Джойстик активируется**: публикует в `/cmd_vel_joy` (priority 100)
4. **Twist Mux переключается**: блокирует Nav2, пропускает джойстик
5. **Робот останавливается**, ручное управление
6. Пользователь отпускает джойстик (timeout 0.5s)
7. **Twist Mux возвращается**: Nav2 снова активен
8. **Робот продолжает навигацию**

## Диагностика

### Проверка работы системы

```bash
# 1. Все ли ноды запущены?
ros2 node list | grep -E "audio|stt|tts|dialogue|command|sound|animation"

# 2. Публикуются ли топики?
ros2 topic hz /voice/stt/result           # STT output
ros2 topic hz /voice/command/feedback     # Command feedback
ros2 topic hz /audio/level                # Audio reactive
ros2 topic hz /cmd_vel                    # Nav2 commands
ros2 topic hz /diff_cont/cmd_vel_unstamped # Final motor commands

# 3. Twist Mux блокирует Nav2?
ros2 topic hz /cmd_vel_joy   # Должно быть 0 Hz для работы Nav2

# 4. Nav2 работает?
ros2 action list | grep navigate_to_pose
ros2 node list | grep -E "controller|planner"

# 5. Audio reactive работает?
ros2 topic echo /audio/level  # Должно меняться при звуке
```

### Типичные проблемы

| Проблема | Причина | Решение |
|----------|---------|---------|
| Робот не движется | Джойстик блокирует Nav2 | Проверить `ros2 topic hz /cmd_vel_joy` |
| Рот не двигается | PyAudio не захватывает аудио | `pactl load-module module-loopback` |
| Команды не распознаются | Low confidence | Снизить `confidence_threshold` |
| Нет эмоций | dialogue_node не обрабатывает | Добавить emotion handler (Phase 6) |

## Будущее развитие (Phase 6)

### Emotion Integration

**Добавить в dialogue_node.py**:
```python
self.animation_trigger_pub = self.create_publisher(
    String, '/animations/trigger', 10
)

def dialogue_callback(self, msg: String):
    chunk_data = json.loads(msg.data)
    
    # Existing: TTS
    if 'ssml' in chunk_data:
        self.response_pub.publish(...)
    
    # NEW: Emotion triggers
    if 'emotion' in chunk_data:
        emotion = chunk_data['emotion']
        
        # Trigger animation
        anim_msg = String()
        anim_msg.data = emotion
        self.animation_trigger_pub.publish(anim_msg)
        
        # Trigger sound
        sound = self._map_emotion_to_sound(emotion)
        sound_msg = String()
        sound_msg.data = sound
        self.sound_trigger_pub.publish(sound_msg)
```

### Advanced Navigation

- **Dynamic waypoints**: "Запомни эту точку как офис"
- **Multi-step paths**: "Иди к кухня потом к гостиная"
- **Person following**: "Следуй за мной"
- **Object detection**: "Найди красный кубик"

### Natural Language Understanding

- **Replace regex** with ML-based intent classifier
- **RASA NLU** or **spaCy** for better understanding
- **Context awareness**: "Там" → resolve to previous location

---

**Status**: ✅ All 8 nodes implemented and integrated  
**Documentation**: Complete architecture with all systems explained  
**Next**: Phase 6 - Emotion Integration + Advanced Features
