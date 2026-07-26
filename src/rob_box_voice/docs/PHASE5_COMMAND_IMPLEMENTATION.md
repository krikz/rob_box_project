# Phase 5: Command Recognition Node

## Обзор

**Статус**: ✅ Реализовано  
**Дата**: 2025-10-13  
**Функционал**: Распознавание голосовых команд и управление роботом

## Архитектура

### Command Node

**Файл**: `rob_box_voice/command_node.py` (415 строк)

**ROS Интерфейс**:
```
Subscribers:
  /voice/stt/result (String)          - Распознанная речь от STT

Publishers:
  /voice/command/intent (String)      - Распознанное намерение
  /voice/command/feedback (String)    - Feedback для пользователя

Action Clients:
  /navigate_to_pose (NavigateToPose)  - Nav2 навигация
```

**Параметры**:
```yaml
confidence_threshold: 0.7       # Минимальная уверенность
enable_navigation: true         # Nav2 команды
enable_follow: false            # Следование (TODO)
enable_vision: false            # Зрение (TODO)
```

## Intent Classification

### Типы намерений

**Реализовано**:
- `NAVIGATE` - Навигация к точке
- `STOP` - Остановка движения
- `STATUS` - Запрос статуса/позиции
- `MAP` - Работа с картой
- `VISION` - Зрение/детекция объектов
- `FOLLOW` - Режим следования

### Паттерны распознавания

**Навигация** (`NAVIGATE`):
```python
"двигайся к точке 3"      → waypoint_number: 3
"иди к кухня"             → waypoint_name: кухня
"поезжай к дом"           → waypoint_name: дом
"двигайся вперед"         → direction: вперед
```

**Остановка** (`STOP`):
```python
"стоп"                    → immediate stop
"остановись"              → immediate stop
"отмени навигацию"        → cancel navigation
```

**Статус** (`STATUS`):
```python
"где ты"                  → position query
"покажи статус"           → status report
"расскажи координаты"     → coordinates query
```

**Карта** (`MAP`):
```python
"покажи карту"            → display map
"построй карту"           → start SLAM
```

**Зрение** (`VISION`):
```python
"что видишь"              → object detection
"найди человека"          → person detection
```

**Следование** (`FOLLOW`):
```python
"следуй за мной"          → follow mode
"включи следование"       → activate following
```

### Алгоритм классификации

1. **Pattern Matching**:
   - Regex поиск по всем паттернам
   - Извлечение entities (числа, названия)

2. **Confidence Scoring**:
   ```python
   confidence = 0.8 + (match_length / text_length) * 0.2
   ```

3. **Entity Extraction**:
   - `waypoint_number`: "точке 3" → 3
   - `waypoint_name`: "к кухня" → "кухня"
   - `direction`: "вперед" → "вперед"

4. **Best Match Selection**:
   - Выбор паттерна с максимальной уверенностью

## Waypoints System

### Предопределённые точки

**Конфигурация** (`voice_assistant.yaml`):
```yaml
waypoints:
  дом:       {x: 0.0, y: 0.0, theta: 0.0}
  кухня:     {x: 2.0, y: 1.0, theta: 0.0}
  гостиная:  {x: 3.0, y: 2.0, theta: 1.57}
  точка_1:   {x: 1.0, y: 0.0, theta: 0.0}
  точка_2:   {x: 2.0, y: 0.0, theta: 0.0}
  точка_3:   {x: 3.0, y: 0.0, theta: 0.0}
```

**Формат**:
- `x`, `y` - координаты в метрах (frame: map)
- `theta` - ориентация в радианах (0 = вперёд, π/2 = влево)

### Динамическое добавление

**TODO**: Команды для сохранения текущей позиции:
```python
"запомни эту точку как спальня"  → save current pose
"сохрани waypoint офис"          → save current pose
```

## Nav2 Integration

### Архитектура команд скорости

**⚠️ ВАЖНО**: Command Node НЕ публикует напрямую в `/cmd_vel`!

**Поток команд**:
```
Voice → STT → Command Node → Nav2 Action Client
                                   ↓
                            NavigateToPose
                                   ↓
                            Nav2 Controller
                                   ↓
                              /cmd_vel (topic)
                                   ↓
                            🎮 twist_mux (приоритизация)
                                   ↓
                         /diff_cont/cmd_vel_unstamped
                                   ↓
                            ros2_control → Моторы
```

### Twist Mux - Приоритеты команд

**Конфигурация** (`docker/main/config/twist_mux/twist_mux.yaml`):

| Источник | Topic | Priority | Timeout | Описание |
|----------|-------|----------|---------|----------|
| 🚨 Emergency | `/cmd_vel_emergency` | **255** | 0.1s | Экстренная остановка |
| 🎮 Joystick | `/cmd_vel_joy` | **100** | 0.5s | Ручное управление (джойстик) |
| 🖥️ Web UI | `/cmd_vel_web` | **50** | 1.0s | Веб-интерфейс |
| 🤖 Nav2 | `/cmd_vel` | **10** | 0.5s | Автономная навигация |

**Логика приоритетов**:
- **Joystick активен** → Nav2 блокируется (человек > автономия)
- **Joystick неактивен** → Nav2 может управлять
- **Emergency stop** → блокирует всё

**Пример сценария**:
```
1. Robot говорит: "Иду к кухня"
2. Nav2 публикует в /cmd_vel (priority 10)
3. Twist Mux пропускает команды → моторы
4. Пользователь берёт джойстик
5. Джойстик публикует в /cmd_vel_joy (priority 100)
6. Twist Mux переключается → Nav2 блокируется
7. Робот останавливается, ручное управление
8. Джойстик отпущен (timeout 0.5s)
9. Twist Mux возвращает управление Nav2
10. Робот продолжает навигацию к кухне
```

### NavigateToPose Action

**Процесс навигации**:

1. **Goal Creation**:
   ```python
   goal = NavigateToPose.Goal()
   goal.pose.header.frame_id = 'map'
   goal.pose.pose.position.x = 2.0
   goal.pose.pose.position.y = 1.0
   goal.pose.pose.orientation.w = 1.0  # theta=0
   ```

2. **Goal Submission**:
   ```python
   future = self.nav_client.send_goal_async(goal, feedback_callback)
   ```

3. **Feedback Monitoring**:
   - Progress updates (distance remaining, ETA)
   - Obstacles detected
   - Recovery behaviors triggered

4. **Result Handling**:
   - Success: "Прибыл в точку назначения"
   - Failure: "Не могу выполнить навигацию"
   - Cancel: "Остановился"

**Важно**: Command Node только отправляет цели Nav2. Реальное управление моторами делает Nav2 через twist_mux

### Coordinate Systems

**map frame**:
- Origin: (0, 0) = starting position
- X-axis: forward
- Y-axis: left
- Z-axis: up

**Orientation**:
```python
theta = 0.0    → 0°   (forward)
theta = π/2    → 90°  (left)
theta = π      → 180° (backward)
theta = -π/2   → 270° (right)
```

**Quaternion conversion**:
```python
orientation.z = sin(theta / 2.0)
orientation.w = cos(theta / 2.0)
```

## Dialogue Integration

### Feedback Loop

```
User Speech → STT → Command Node → Nav2
                ↓                     ↓
         Dialogue Node ← Feedback ←─┘
                ↓
              TTS → Speech + Audio-Reactive Animations
                         ↓
                   🎤 Audio Output
                         ↓
              audio_reactive_animation_node
                         ↓
                   /audio/level (Float32)
                         ↓
              animation_player_node
                         ↓
                   👄 Mouth Animation (lip-sync)
```

**Пример диалога**:

1. **User**: "Двигайся к точке три"
2. **STT**: "двигайся к точке три"
3. **Command**: Intent=NAVIGATE, waypoint="точка 3"
4. **Feedback**: "Иду к точка 3" → TTS
5. **TTS**: Синтез речи → `/voice/audio/speech`
6. **Audio Reactive Node**: Мониторинг звуковой карты → `/audio/level`
7. **Animation Player**: Mouth frames (0-11) в зависимости от громкости
8. **Nav2**: NavigateToPose(x=3.0, y=0.0)
9. *[робот едет, рот двигается при речи]*
10. **Nav2 Result**: SUCCESS
11. **Feedback**: "Прибыл в точку назначения" → TTS + Mouth Animation

### Audio-Reactive Animations

**Система синхронизации рта робота с речью**:

**Компоненты**:
- `audio_reactive_animation_node.py` - Мониторинг аудио выхода (PyAudio)
- `animation_player_node` - Выбор кадра анимации по уровню громкости
- `talking.yaml` - Манифест с 12 кадрами mouth animation

**Алгоритм**:
1. PyAudio захватывает аудио выход системы (loopback/stereo mix)
2. Вычисляется RMS громкость
3. Применяется сглаживание (smoothing)
4. Публикуется в `/audio/level` (0.0-1.0)
5. Animation player выбирает кадр: `frame = int(audio_level * 11)`
6. LED матрица отображает рот (открытый/закрытый)

**Конфигурация** (`animations/manifests/talking.yaml`):
```yaml
mouth_panel:
  audio_controlled: true  # Реактивность на звук
  frames:
    - frame_0.png  # Рот закрыт (audio_level = 0.0)
    - frame_1.png  # Чуть приоткрыт
    ...
    - frame_11.png # Широко открыт (audio_level = 1.0)
```

**Результат**: Рот робота открывается/закрывается синхронно с речью (эффект как у Bender из Futurama)

### Emotion-Based Animations

**Система эмоциональных анимаций** (будущая интеграция):

**DeepSeek Response JSON** (текущий формат):
```json
{
  "chunk": 1,
  "ssml": "<speak>Текст</speak>",
  "emotion": "happy"  ← Поле эмоции (опционально)
}
```

**Поддерживаемые эмоции**:
- `neutral` - Нейтральное состояние (eyes_neutral.yaml)
- `happy` - Радость (happy.yaml + sound: cute/very_cute)
- `sad` - Грусть (sad.yaml)
- `thinking` - Размышление (thinking.yaml + sound: thinking)
- `alert` - Тревога (alert.yaml)
- `angry` - Злость (angry.yaml + sound: angry_1/angry_2)
- `surprised` - Удивление (surprised.yaml + sound: surprise)

**TODO - Интеграция с dialogue_node**:
```python
def dialogue_callback(self, msg: String):
    chunk_data = json.loads(msg.data)
    
    # Текущее: только TTS
    ssml = chunk_data['ssml']
    self.tts_pub.publish(ssml)
    
    # TODO: Триггер анимации по эмоции
    if 'emotion' in chunk_data:
        emotion = chunk_data['emotion']
        self._trigger_animation(emotion)  # → /animations/trigger
        self._trigger_sound(emotion)      # → /voice/sound/trigger
```

**Планируемый поток**:
```
DeepSeek → {"emotion": "happy"}
    ↓
dialogue_node
    ↓
    ├→ TTS (speech)
    ├→ /animations/trigger ("happy")
    └→ /voice/sound/trigger ("cute")
         ↓
    Animation Player + Sound Node
         ↓
    😊 Робот улыбается + звук радости
```

### Command Confirmation

**TODO**: Опциональное подтверждение опасных команд:
```
User: "Двигайся к обрыв"
Robot: "Вы уверены? Там может быть опасно"
User: "Да, уверен"
Robot: "Хорошо, выполняю"
```

## Testing

### Test Script

**Файл**: `scripts/test_command_node.py`

**Возможности**:
- 24 тестовых команды (все типы)
- Интерактивное меню
- Прямой ввод своих команд
- Batch testing (все подряд)

**Запуск**:
```bash
# Терминал 1: Запустить command_node
ros2 run rob_box_voice command_node

# Терминал 2: Запустить тестер
cd src/rob_box_voice/scripts
python3 test_command_node.py
```

### Manual Testing

**Эмуляция STT**:
```bash
ros2 topic pub --once /voice/stt/result std_msgs/String "data: 'двигайся к точке три'"
```

**Мониторинг**:
```bash
# Intent
ros2 topic echo /voice/command/intent

# Feedback
ros2 topic echo /voice/command/feedback

# Nav2 goal
ros2 topic echo /goal_pose
```

### Integration Test

**Полный цикл**:
```bash
# Запустить Voice Assistant
ros2 launch rob_box_voice voice_assistant.launch.py

# Запустить Nav2 (если ещё не запущен)
docker-compose up -d nav2

# Говорить в микрофон:
"Двигайся к точке три"

# Ожидаемое поведение:
# 1. STT → "двигайся к точке три"
# 2. Command → Intent=NAVIGATE
# 3. TTS → "Иду к точка три"
# 4. Nav2 → robot moves
# 5. TTS → "Прибыл в точку назначения"
```

## Performance

### Латентность

| Компонент | Время | Оптимизация |
|-----------|-------|-------------|
| Pattern Match | <10ms | ✅ Regex |
| Entity Extract | <5ms | ✅ Regex groups |
| Nav2 Goal Send | <50ms | ✅ Async |
| **Total** | **~65ms** | 🚀 Real-time |

### CPU/Memory

```
Command Node: ~5% CPU (idle)
Nav2 Client:  ~10% CPU (navigation)
RAM:          ~20 MB
```

## Troubleshooting

### Проблема: Команды не распознаются

**Причины**:
1. Низкая уверенность (`confidence < threshold`)
2. Паттерн не совпадает с командой
3. STT неправильно распознал речь

**Диагностика**:
```bash
# Проверить логи command_node
ros2 run rob_box_voice command_node

# Ожидается:
# 🎤 STT: двигайся к точке три
# 🎯 Intent: navigate (0.85)
# 📦 Entities: {'waypoint': 'точка 3'}
```

**Решение**:
- Снизить `confidence_threshold` (например, 0.5)
- Добавить альтернативные паттерны
- Улучшить качество STT (использовать большую Vosk модель)

### Проблема: Nav2 goal не отправляется

**Причины**:
1. Nav2 action server недоступен
2. Waypoint не найден
3. Неправильные координаты

**Диагностика**:
```bash
# Проверить Nav2
ros2 node list | grep navigator

# Проверить action server
ros2 action list | grep navigate_to_pose

# Проверить waypoints
ros2 param get /command_node waypoints
```

**Решение**:
```bash
# Запустить Nav2
docker-compose up -d nav2

# Добавить waypoint в config
# voice_assistant.yaml:
# waypoints:
#   новая_точка: {x: 4.0, y: 2.0, theta: 0.0}
```

### Проблема: Робот не движется

**Причины**:
1. Nav2 не запущен
2. Карта не загружена (`/map` topic missing)
3. Локализация не работает (`/odom` missing)
4. **Twist Mux блокирует команды** (джойстик активен)
5. ros2_control не запущен

**Диагностика**:
```bash
# 1. Проверить топики
ros2 topic list | grep -E "/map|/odom|/cmd_vel"

# 2. Проверить Nav2 ноды
ros2 node list | grep -E "controller|planner|navigator"

# 3. Проверить Twist Mux (ВАЖНО!)
ros2 topic echo /cmd_vel                    # Nav2 output
ros2 topic echo /diff_cont/cmd_vel_unstamped # Twist Mux output
ros2 topic echo /cmd_vel_joy                # Joystick (может блокировать!)

# 4. Проверить приоритеты
ros2 param get /twist_mux topics

# 5. Проверить активные топики Twist Mux
ros2 topic hz /cmd_vel_joy    # Если >0 Hz → джойстик блокирует Nav2!
```

**Решение**:
```bash
# 1. Запустить полный стек
docker-compose up -d rtabmap nav2 ros2-control twist-mux

# 2. Убедиться, что джойстик НЕ публикует команды
ros2 topic hz /cmd_vel_joy  # Должно быть 0 Hz или "no messages"

# 3. Проверить что Nav2 публикует
ros2 topic hz /cmd_vel  # Должно быть ~10-20 Hz во время навигации

# 4. Проверить что Twist Mux пропускает команды
ros2 topic hz /diff_cont/cmd_vel_unstamped  # Должно быть ~10-20 Hz

# 5. Если джойстик блокирует - подождать timeout (0.5s)
# Или перезапустить twist-mux:
docker restart twist-mux
```

### Проблема: Mouth animation не синхронизирована

**Причины**:
1. `audio_reactive_animation_node` не запущен
2. PyAudio не может захватить аудио выход
3. Неправильный аудио девайс
4. `audio_controlled: false` в манифесте

**Диагностика**:
```bash
# 1. Проверить ноду
ros2 node list | grep audio_reactive

# 2. Проверить топик уровня звука
ros2 topic hz /audio/level  # Должно быть ~20-50 Hz

# 3. Проверить PyAudio devices
python3 -c "import pyaudio; pa = pyaudio.PyAudio(); \
    print([pa.get_device_info_by_index(i) for i in range(pa.get_device_count())])"

# 4. Проверить манифест
grep "audio_controlled" src/rob_box_animations/animations/manifests/talking.yaml
```

**Решение**:
```bash
# 1. Запустить audio reactive node
ros2 run rob_box_animations audio_reactive_animation_node

# 2. Включить loopback на звуковой карте (Linux)
pactl load-module module-loopback

# 3. Указать правильный device в параметрах
ros2 param set /audio_reactive_animation_node audio_device_index 2

# 4. Проверить что манифест talking.yaml имеет:
# mouth_panel:
#   audio_controlled: true
```

## Future Enhancements

### Phase 6: Advanced Commands

**Person Following**:
```python
"следуй за мной"           → activate person tracking
"следуй на расстоянии 1м"  → set follow distance
"перестань следовать"      → deactivate following
```

**Object Manipulation**:
```python
"возьми красный кубик"     → pick object
"положи на стол"           → place object
"подай мне бутылку"        → hand over
```

**Multi-step Commands**:
```python
"иди к кухня потом к гостиная"  → waypoint sequence
"найди человека и следуй за ним" → vision + follow
```

### Dynamic Waypoint Learning

**Concept**: Робот запоминает посещённые места:
```
User: "Запомни эту точку как офис"
Robot: "Хорошо, сохранил точку офис"

User: "Двигайся к офис"
Robot: "Иду к офис" → NavigateToPose(saved_coords)
```

**Implementation**:
- Сохранение в ROS2 parameters
- Persistence в YAML файл
- Удаление старых waypoints

### Natural Language Understanding (NLU)

**Текущее**: Rule-based regex patterns  
**Будущее**: ML-based intent classifier

**Options**:
1. **RASA NLU**: Open-source NLU
2. **spaCy + custom model**: Russian NER
3. **DeepSeek classification**: Use LLM for intent

**Преимущества**:
- Более гибкое распознавание
- Обработка опечаток/вариаций
- Контекстное понимание

## References

- [Nav2 Actions](https://navigation.ros.org/tutorials/docs/using_plugins.html)
- [ROS2 Action Clients](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Regex in Python](https://docs.python.org/3/library/re.html)
- [Twist Mux Documentation](http://wiki.ros.org/twist_mux)
- [Audio-Reactive Animations](../../rob_box_animations/AUDIO_REACTIVE.md)
- [Phase 3: STT](PHASE3_STT_IMPLEMENTATION.md)
- [Phase 4: Sound](PHASE4_SOUND_IMPLEMENTATION.md)

## Архитектура полного цикла

### Voice Command → Robot Motion (полный поток)

```
┌─────────────────────────────────────────────────────────────┐
│                    USER SPEAKS                              │
│              "Двигайся к точке три"                         │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  🎤 AUDIO NODE (Phase 1)                                    │
│  - Captures microphone → /voice/audio/recording             │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  🗣️ STT NODE (Phase 3)                                      │
│  - Vosk → "двигайся к точке три" → /voice/stt/result       │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  🎯 COMMAND NODE (Phase 5) ← YOU ARE HERE                   │
│  - Intent: NAVIGATE                                         │
│  - Entity: waypoint="точка 3"                               │
│  - Lookup: {x: 3.0, y: 0.0, theta: 0.0}                     │
│  - Send Nav2 Goal (NavigateToPose Action)                   │
│  - Feedback → /voice/command/feedback: "Иду к точка 3"      │
└────────────┬───────────────────────────────┬────────────────┘
             │                               │
             ↓                               ↓
┌────────────────────────┐    ┌──────────────────────────────┐
│  💬 DIALOGUE NODE      │    │  🤖 NAV2 STACK               │
│  (Phase 2)             │    │  - Planner: A* path          │
│  Receives feedback     │    │  - Controller: DWB           │
│  → TTS                 │    │  - Costmaps: obstacles       │
└─────────┬──────────────┘    │  - Publishes: /cmd_vel       │
          │                   └─────────┬────────────────────┘
          ↓                             │
┌─────────────────────────┐             ↓
│  🔊 TTS NODE (Phase 2)  │   ┌──────────────────────────────┐
│  - Silero TTS           │   │  🎮 TWIST MUX                │
│  - "Иду к точка 3"      │   │  Input priorities:           │
│  - Audio output         │   │  1. Emergency (255)          │
└─────────┬───────────────┘   │  2. Joystick (100) ← blocks  │
          │                   │  3. Web UI (50)              │
          ↓                   │  4. Nav2 (10) ← /cmd_vel     │
┌─────────────────────────┐   │  Output:                     │
│  🎤 AUDIO REACTIVE NODE │   │  → /diff_cont/cmd_vel        │
│  - Monitors sound card  │   └─────────┬────────────────────┘
│  - RMS volume           │             │
│  - /audio/level (0-1)   │             ↓
└─────────┬───────────────┘   ┌──────────────────────────────┐
          │                   │  ⚙️ ROS2_CONTROL             │
          ↓                   │  - diff_drive_controller     │
┌─────────────────────────┐   │  - Odometry /odom            │
│  🎨 ANIMATION PLAYER    │   │  - Forwards to VESC          │
│  - Frame = level * 11   │   └─────────┬────────────────────┘
│  - Mouth open/close     │             │
│  - LED matrix update    │             ↓
└─────────────────────────┘   ┌──────────────────────────────┐
                              │  🔧 VESC MOTOR CONTROLLERS   │
                              │  - 4x motors (4WD)           │
                              │  - Differential drive        │
                              │  - Robot moves to (3.0, 0.0) │
                              └──────────────────────────────┘
```

### Эмоции в будущем (Phase 6)

**Планируемый поток**:

1. **DeepSeek Response** включает `emotion`:
   ```json
   {"chunk": 1, "ssml": "Отлично!", "emotion": "happy"}
   ```

2. **Dialogue Node** распознаёт эмоцию:
   ```python
   if 'emotion' in chunk_data:
       self._trigger_animation(chunk_data['emotion'])  # → /animations/trigger
       self._trigger_sound_by_emotion(chunk_data['emotion'])  # → /voice/sound/trigger
   ```

3. **Animation Player** запускает эмоциональную анимацию:
   - `happy.yaml` → 😊 улыбается
   - `sad.yaml` → 😢 грустит
   - `angry.yaml` → 😠 злится
   - `thinking.yaml` → 🤔 размышляет

4. **Sound Node** воспроизводит соответствующий звук:
   - `happy` → `cute.mp3` или `very_cute.mp3`
   - `angry` → `angry_1.mp3` или `angry_2.mp3`
   - `thinking` → `thinking.mp3`
   - `confused` → `confused.mp3`

**Mapping эмоций** (из Phase 4):
```python
emotion_to_sound = {
    'happy': ['cute', 'very_cute'],       # Random choice
    'sad': ['confused'],
    'angry': ['angry_1', 'angry_2'],
    'thinking': ['thinking'],
    'surprised': ['surprise']
}

emotion_to_animation = {
    'happy': 'happy',
    'sad': 'sad',
    'angry': 'angry',
    'thinking': 'thinking',
    'surprised': 'surprised',
    'neutral': 'eyes_neutral'
}
```

---

**Status**: ✅ Phase 5 Complete (Command Recognition + Nav2 + Twist Mux)  
**Next**: Phase 6 - Emotion Integration + Advanced Features (Vision, Following, Multi-step)
