# Phase 4: Sound Effects Node

## Обзор

**Статус**: ✅ Реализовано  
**Дата**: 2025-10-13  
**Функционал**: Воспроизведение звуковых эффектов с триггерами

## Архитектура

### Sound Node

**Файл**: `rob_box_voice/sound_node.py` (236 строк)

**ROS Интерфейс**:
```
Subscribers:
  /voice/sound/trigger (String)   - Триггер звукового эффекта

Publishers:
  /voice/sound/state (String)     - Состояние ноды
  /animations/trigger (String)    - Триггер анимации (опционально)
```

**Параметры**:
```yaml
sound_pack_dir: ~/rob_box_project/sound_pack
volume_db: 0                    # Регулировка громкости
trigger_animations: true        # Синхронизация с анимациями
animation_topic: /animations/trigger
```

## Sound Pack

### Доступные звуки

**Расположение**: `sound_pack/` (11 файлов)

| Файл | Назначение | Триггер |
|------|-----------|---------|
| `thinking.mp3` | Во время обработки LLM | `thinking` |
| `surprise.mp3` | Неожиданный вопрос | `surprise` |
| `confused.mp3` | Не понял вопрос | `confused` |
| `angry_1.mp3` | Негативная реакция 1 | `angry_1` или `angry` |
| `angry_2.mp3` | Негативная реакция 2 | `angry_2` или `angry` |
| `cute.mp3` | Приятный ответ | `cute` |
| `very_cute.mp3` | Очень милый ответ | `very_cute` |
| `talk_1.mp3` | Во время речи (вариант 1) | `talk_1` или `talk` |
| `talk_2.mp3` | Во время речи (вариант 2) | `talk_2` или `talk` |
| `talk_3.mp3` | Во время речи (вариант 3) | `talk_3` или `talk` |
| `talk_4.mp3` | Во время речи (вариант 4) | `talk_4` или `talk` |

### Группы звуков

Sound Node поддерживает группы с случайным выбором:

```python
sound_groups = {
    'talk': ['talk_1', 'talk_2', 'talk_3', 'talk_4'],  # Random
    'angry': ['angry_1', 'angry_2'],                   # Random
    'cute': ['cute', 'very_cute']                      # Random
}
```

**Использование**:
```bash
# Опубликовать "talk" → случайный из talk_1..4
ros2 topic pub /voice/sound/trigger std_msgs/String "data: 'talk'"
```

## Работа Sound Node

### Алгоритм воспроизведения

1. **Загрузка звуков** (при старте):
   - Сканирует `sound_pack/` для `.mp3` файлов
   - Загружает с помощью `pydub.AudioSegment`
   - Применяет регулировку громкости (`volume_db`)

2. **Обработка триггера**:
   - Получает триггер из `/voice/sound/trigger`
   - Выбирает звук (прямое совпадение или из группы)
   - Запускает воспроизведение в отдельном потоке

3. **Воспроизведение**:
   - Использует `pydub.playback.play()`
   - Блокирует новые триггеры во время игры
   - Опционально триггерит анимацию

4. **Завершение**:
   - Публикует состояние `ready`
   - Готов к следующему триггеру

### Защита от конфликтов

**Проблема**: Два звука не должны играть одновременно.

**Решение**:
```python
if self.is_playing:
    self.get_logger().warn('⚠️ Звук уже играет, пропускаю')
    return
```

## Интеграция с Dialogue Node

### Триггеры в dialogue_node

**Добавлено в Phase 4**:
```python
# dialogue_node.py
self.sound_trigger_pub = self.create_publisher(
    String, '/voice/sound/trigger', 10
)

# Перед запросом к LLM
self._trigger_sound('thinking')
```

### Data Flow

```
User Speech → STT → Dialogue → Sound + TTS
                        ↓
                   "thinking"
                        ↓
                   sound_node
                        ↓
                   thinking.mp3 ♪
```

**Пример диалога с звуками**:

1. Пользователь: "Привет робот"
2. `dialogue_node` → Триггер: **"thinking"**
3. `sound_node` → Играет `thinking.mp3` 🎵
4. DeepSeek → Ответ: "Привет!"
5. `tts_node` → Синтез речи
6. Аудио выход → "Привет!" 🗣️

### Будущие триггеры

**TODO**: Добавить emotion detection в dialogue_node:
```python
# Анализ ответа LLM для выбора звука
if "удивительно" in response or "вау" in response:
    self._trigger_sound('surprise')
elif "не знаю" in response:
    self._trigger_sound('confused')
elif sentiment == "positive":
    self._trigger_sound('cute')
```

## Синхронизация с анимациями

### Animation Mapping

Sound Node автоматически триггерит анимации:

```python
animation_map = {
    'thinking': 'thinking',      # Думает
    'surprise': 'surprise',      # Удивлён
    'confused': 'confused',      # Не понял
    'angry': 'angry',            # Злится
    'cute': 'happy',             # Радуется
    'very_cute': 'very_happy',   # Очень рад
    'talk': 'talking'            # Говорит
}
```

**Параметр**:
```yaml
trigger_animations: true   # Включить синхронизацию
```

**Отключение**:
```yaml
trigger_animations: false  # Только звуки, без анимаций
```

## Регулировка громкости

### Volume Control

**Параметр** `volume_db`:
- `0` - без изменений (по умолчанию)
- `+6` - в 2 раза громче
- `-6` - в 2 раза тише
- `+12` - в 4 раза громче
- `-12` - в 4 раза тише

**Формула**: `new_volume = original + volume_db`

**Пример**:
```yaml
sound_node:
  volume_db: -3  # Немного тише
```

## Тестирование

### Test Script

**Файл**: `scripts/test_sound_node.py`

**Возможности**:
- Интерактивное меню
- Выбор индивидуальных звуков
- Тест всех звуков подряд
- Тест групп (random)

**Запуск**:
```bash
# Терминал 1: Запустить sound_node
ros2 run rob_box_voice sound_node

# Терминал 2: Запустить тестер
cd src/rob_box_voice/scripts
python3 test_sound_node.py
```

### Manual Testing

**Индивидуальный звук**:
```bash
ros2 topic pub --once /voice/sound/trigger std_msgs/String "data: 'thinking'"
```

**Группа (random)**:
```bash
ros2 topic pub --once /voice/sound/trigger std_msgs/String "data: 'talk'"
```

**Проверка состояния**:
```bash
ros2 topic echo /voice/sound/state
```

### Integration Test

**Полный цикл с диалогом**:
```bash
# Запустить Voice Assistant
ros2 launch rob_box_voice voice_assistant.launch.py

# В другом терминале - эмуляция STT
ros2 topic pub --once /voice/stt/result std_msgs/String "data: 'привет робот'"

# Ожидаемое поведение:
# 1. thinking.mp3 ♪ (во время обработки)
# 2. Ответ робота 🗣️
```

## Performance

### Латентность

| Операция | Время | Оптимизация |
|----------|-------|-------------|
| Загрузка 11 MP3 | ~500ms | ✅ При старте |
| Trigger → Play | <50ms | ✅ Мгновенно |
| Воспроизведение | 1-3s | Зависит от файла |

### Системные требования

```
CPU: ~5-10% (во время игры)
RAM: ~50 MB (загруженные MP3)
Аудио выход: ReSpeaker 3.5mm jack или ALSA
```

## Troubleshooting

### Проблема: Звуки не играют

**Причины**:
1. Директория `sound_pack/` не найдена
2. MP3 файлы повреждены
3. Нет аудио устройства
4. PyDub не установлен

**Диагностика**:
```bash
# Проверить директорию
ls -lh ~/rob_box_project/sound_pack

# Проверить логи ноды
ros2 run rob_box_voice sound_node

# Ожидается:
# ✓ thinking: 2341ms, 44100Hz
# ✓ surprise: 1523ms, 44100Hz
# ...
# ✅ Загружено звуков: 11/11
```

**Решение**:
```bash
# Установить PyDub
pip install pydub

# Проверить MP3 файлы
file ~/rob_box_project/sound_pack/*.mp3
```

### Проблема: Нет звука на выходе

**Причины**:
1. ALSA неправильно настроена
2. Громкость = 0
3. Неправильное аудио устройство

**Решение**:
```bash
# Проверить аудио устройства
aplay -L

# Проверить громкость
alsamixer

# Тест звука
aplay /usr/share/sounds/alsa/Front_Center.wav
```

### Проблема: Звуки обрезаются

**Причина**: Конфликт с TTS или другим аудио потоком.

**Решение**: Sound Node проверяет `is_playing` флаг:
```python
if self.is_playing:
    return  # Не прерывать текущий звук
```

## Docker Integration

### Dockerfile Updates

**TODO**: Добавить в `docker/vision/Dockerfile`:
```dockerfile
# PyDub для sound_node
RUN pip3 install pydub

# FFmpeg для декодирования MP3
RUN apt-get update && apt-get install -y ffmpeg && rm -rf /var/lib/apt/lists/*
```

### Docker Compose

**Добавить** в `docker/vision/docker-compose.yaml`:
```yaml
voice-assistant:
  volumes:
    - ./../../sound_pack:/ws/sound_pack:ro  # Mount sound files
```

## Next Steps: Phase 5

**Command Node** - распознавание команд роботу:
- Парсинг команд из STT результата
- Классификация намерений (движение, навигация, действия)
- Публикация в ROS2 action servers
- Интеграция с Nav2 для навигации

**TODO**:
1. Создать `command_node.py`
2. Определить словарь команд
3. Реализовать intent classifier
4. Интегрировать с Nav2 actions

## References

- [PyDub Documentation](https://github.com/jiaaro/pydub)
- [ROS2 Publisher/Subscriber](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Threading in Python](https://docs.python.org/3/library/threading.html)

---

**Status**: ✅ Phase 4 Complete  
**Next**: Phase 5 - Command Recognition Node
