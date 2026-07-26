# 🎵 Интеграция звуковых эффектов в Internal Dialogue

Описание использования sound_pack для эмоционального выражения робота в системе Internal Dialogue.

## 📦 Доступные звуки

| Файл | Эмоция | Использование |
|------|--------|---------------|
| `thinking.mp3` | Размышление | Обдумывание, анализ ситуации |
| `surprise.mp3` | Удивление | Неожиданная информация, новое событие |
| `confused.mp3` | Замешательство | Непонимание, неопределённость |
| `angry_1.mp3` | Недовольство | Проблемы, ошибки (лёгкая форма) |
| `angry_2.mp3` | Злость | Критические проблемы, аварии |
| `cute.mp3` | Радость | Успех, норма, позитив |
| `very_cute.mp3` | Очень рад | Особая радость, восстановление |
| `talk_1.mp3` | Разговор 1 | Фоновый звук беседы |
| `talk_2.mp3` | Разговор 2 | Фоновый звук беседы |
| `talk_3.mp3` | Разговор 3 | Фоновый звук беседы |
| `talk_4.mp3` | Разговор 4 | Фоновый звук беседы |

## 🎯 Компоненты с звуковыми эффектами

### 1. **Startup Greeting Node**

**Назначение:** Приветствие при запуске системы

**Последовательность:**
1. **Старт ноды** → `thinking.mp3` (система загружается)
2. **Пауза 5 сек** → ожидание готовности Context Aggregator
3. **Система готова** → `cute.mp3` или `very_cute.mp3` (случайно)
4. **TTS приветствие** → случайная фраза из набора

**Приветствия:**
- "Я вернулся! Системы в норме, готов к работе!"
- "Привет! Все модули загружены, жду команд!"
- "Здравствуйте! Загрузка завершена, готов помогать!"
- "Системы запущены! Как я могу помочь?"
- "Всё готово! Чем займёмся?"
- "Рад вернуться! Все датчики в норме!"
- "Онлайн! Диалоговая система активна!"
- "Загрузка завершена! Слушаю вас!"

**Параметры:**
```yaml
wait_time: 5.0          # Минимальное время ожидания перед проверкой
check_timeout: 30.0     # Максимальное время ожидания готовности
enable_greeting: true   # Включить приветствие
```

**Файл:** `src/rob_box_perception/rob_box_perception/startup_greeting_node.py`

---

### 2. **Reflection Node** (Internal Dialogue)

**Назначение:** Звуковое выражение эмоций при размышлениях

**Триггеры звуков:**

| Ключевые слова в мысли | Звук | Пример мысли |
|------------------------|------|--------------|
| удивительно, вау, неожиданно, странно, интересно | `surprise` | "Удивительно, батарея разряжается быстрее обычного" |
| думаю, размышляю, анализирую, рассматриваю, проверяю | `thinking` | "Думаю, стоит проверить состояние датчиков" |
| не уверен, сложно, непонятно, затрудняюсь, не знаю | `confused` | "Не уверен, что означает это поведение камеры" |
| ошибка, проблема, критично, degraded, сбой, авария | `angry` | "Проблема с нодой vision_node - нет данных!" |
| отлично, хорошо, успешно, готов, healthy, норма | `cute` | "Отлично, все системы в норме!" |

**Алгоритм:**
```python
def _trigger_sound_for_thought(thought: str):
    """Анализ эмоции мысли и триггер звука"""
    thought_lower = thought.lower()
    
    if 'удивительно' in thought_lower or 'вау' in thought_lower:
        play_sound('surprise')
    elif 'думаю' in thought_lower:
        play_sound('thinking')
    # ... и т.д.
```

**Интеграция:** Звук играется автоматически при `_publish_thought()`

**Файл:** `src/rob_box_perception/rob_box_perception/reflection_node.py` (строки 626-664)

---

### 3. **Health Monitor**

**Назначение:** Звуковая индикация изменения статуса системы

**Триггеры:**

| Изменение статуса | Звук | Описание |
|-------------------|------|----------|
| HEALTHY → DEGRADED | `confused` | Появились ошибки (5+ за минуту) |
| DEGRADED → CRITICAL | `angry_2` | Критическая ситуация (fatal errors) |
| CRITICAL/DEGRADED → HEALTHY | `cute` | Система восстановилась |

**Особенности:**
- Звук играется **ТОЛЬКО при изменении** статуса (не каждые 5 секунд)
- Использует `self.last_status` для отслеживания
- Параметр `enable_sounds=True` для включения/отключения

**Алгоритм:**
```python
if status != last_status:
    if status == "CRITICAL":
        play_sound('angry_2')
    elif status == "DEGRADED":
        play_sound('confused')
    elif status == "HEALTHY" and last_status is not None:
        play_sound('cute')
    
    last_status = status
```

**Параметры:**
```yaml
enable_sounds: true     # Включить звуковые эффекты
check_rate: 1.0         # Частота проверки (Hz)
degraded_threshold: 5   # Ошибок для degraded статуса
critical_threshold: 10  # Ошибок для critical
```

**Файл:** `src/rob_box_perception/rob_box_perception/health_monitor.py` (строки 88-107)

---

## 🔧 Конфигурация

### Launch файл (Docker)

```python
# docker/vision/config/perception/internal_dialogue_docker.launch.py

Node(
    package='rob_box_perception',
    executable='health_monitor',
    parameters=[{
        'enable_sounds': True,  # Звуки для статуса
    }],
),

Node(
    package='rob_box_perception',
    executable='reflection_node',
    # Звуки включены автоматически
),

Node(
    package='rob_box_perception',
    executable='startup_greeting',
    parameters=[{
        'enable_greeting': True,  # Приветствие при старте
        'wait_time': 5.0,
    }],
),
```

### Sound Node

Все звуки проигрываются через **Sound Node** (rob_box_voice):

**Topic:** `/voice/sound/trigger` (String)

**Примеры:**
```bash
# Проиграть звук вручную
ros2 topic pub /voice/sound/trigger std_msgs/msg/String "data: 'thinking'"

# Проверить что Sound Node готов
ros2 topic echo /voice/sound/state
```

## 📊 ROS Topics

| Topic | Type | Direction | Описание |
|-------|------|-----------|----------|
| `/voice/sound/trigger` | String | → Sound Node | Триггер звука (имя файла) |
| `/voice/sound/state` | String | ← Sound Node | Состояние: ready/playing_X |
| `/reflection/internal_thought` | String | ← Reflection | Мысли робота (с эмоциями) |
| `/perception/context_update` | PerceptionEvent | ← Context Aggregator | Контекст для размышлений |

## 🎬 Workflow примеры

### Пример 1: Запуск системы

```
1. [Startup Greeting] thinking.mp3
   ⏱ 2 секунды загрузки...

2. [Context Aggregator] Публикует контекст
   [Startup Greeting] Получает /perception/context_update

3. [Startup Greeting] very_cute.mp3
   ⏱ 1.5 секунды паузы...

4. [TTS] "Привет! Все модули загружены, жду команд!"

5. [Startup Greeting] Завершает работу (больше не нужна)
```

### Пример 2: Reflection обнаружил проблему

```
1. [Context Aggregator] Видит ошибки в /rosout
2. [Context Aggregator] Публикует PerceptionEvent с system_health='degraded'

3. [Reflection Node] Получает контекст
4. [Reflection Node] Генерирует мысль: "Проблема с навигацией - нет данных от LIDAR!"
5. [Reflection Node] Триггер звука: angry (по ключевому слову "проблема")
6. [Sound Node] Играет angry_1.mp3

7. [Reflection Node] Публикует мысль в /reflection/internal_thought
8. [Reflection Node] Если should_speak=true → /voice/tts/request
```

### Пример 3: Health Monitor видит восстановление

```
1. [Health Monitor] recent_errors = 7 → status = DEGRADED
   last_status = None → новый статус!
   
2. [Health Monitor] Играет confused.mp3
   last_status = DEGRADED

3. ⏱ 60 секунд проходит...
   recent_errors = 2 (старые ошибки вышли из окна 60 сек)

4. [Health Monitor] status = HEALTHY
   last_status = DEGRADED → изменение!
   
5. [Health Monitor] Играет cute.mp3
   last_status = HEALTHY
```

## 🐛 Отладка

### Проверить звуки вручную

```bash
# 1. Запустить Sound Node
ros2 run rob_box_voice sound_node

# 2. В другом терминале триггерить звуки
ros2 topic pub /voice/sound/trigger std_msgs/msg/String "data: 'thinking'"
ros2 topic pub /voice/sound/trigger std_msgs/msg/String "data: 'surprise'"
ros2 topic pub /voice/sound/trigger std_msgs/msg/String "data: 'angry'"
```

### Проверить Reflection эмоции

```bash
# Слушать мысли
ros2 topic echo /reflection/internal_thought

# Слушать триггеры звуков
ros2 topic echo /voice/sound/trigger

# Создать тестовый контекст с проблемой
# (в context_aggregator добавить fake ошибки)
```

### Логи звуков

```bash
# Reflection
docker logs perception | grep "🎵"

# Health Monitor
docker logs perception | grep "Health звук"

# Startup Greeting
docker logs perception | grep "Startup Greeting"
```

## 📝 Добавление новых звуков

1. **Добавить MP3 в sound_pack/**
   ```bash
   cd /home/ros2/rob_box_project/sound_pack
   cp ~/new_sound.mp3 ./excited.mp3
   ```

2. **Обновить триггер в Reflection Node**
   ```python
   # reflection_node.py, метод _trigger_sound_for_thought()
   elif any(word in thought_lower for word in ['круто', 'супер', 'восхитительно']):
       self._play_sound('excited')
   ```

3. **Пересобрать и запустить**
   ```bash
   cd ~/perception_ws
   colcon build --packages-select rob_box_perception
   source install/setup.bash
   ```

## 🎯 Best Practices

1. **НЕ спамить звуками** - один звук на одну эмоцию
2. **Использовать в Health Monitor** только при изменении статуса
3. **В Reflection** - звуки для важных мыслей, не каждую секунду
4. **Startup Greeting** запускается один раз при старте
5. **Громкость** регулируется в Sound Node параметром `volume_db`

## 🚀 Deployment

После изменений пересобрать Docker образ:

```bash
# Сборка perception образа
docker build -f docker/vision/perception/Dockerfile \
  -t ghcr.io/krikz/rob_box:perception-kilted-test .

# Или через CI/CD
git push origin feature/internal-dialogue
# → GitHub Actions пересоберёт автоматически
```

## 📚 См. также

- [Sound Node архитектура](../../src/rob_box_voice/docs/PHASE4_SOUND_IMPLEMENTATION.md)
- [Internal Dialogue + Voice Assistant](../architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md)
- [Animation Editor](../guides/ANIMATION_EDITOR.md) - синхронизация с анимациями
