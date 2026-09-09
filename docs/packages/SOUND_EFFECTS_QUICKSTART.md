# 🎵 Звуковые эффекты - Быстрый старт

## Что реализовано

**3 компонента с автоматическими звуками:**

### 1️⃣ **Startup Greeting** - "Я вернулся!"
При запуске системы:
- 🎵 `thinking.mp3` → загрузка...
- ⏱ 5 сек ожидания
- 🎵 `cute.mp3` → готово!
- 🗣️ "Привет! Все модули загружены, жду команд!"

### 2️⃣ **Reflection** - эмоции в мыслях
Размышления робота автоматически сопровождаются звуками:
- "Думаю..." → `thinking.mp3`
- "Удивительно!" → `surprise.mp3`
- "Не уверен..." → `confused.mp3`
- "Ошибка!" → `angry.mp3`
- "Отлично!" → `cute.mp3`

### 3️⃣ **Health Monitor** - статус системы
При изменении здоровья:
- ✅ → ⚠️ DEGRADED → `confused.mp3`
- ⚠️ → 🚨 CRITICAL → `angry_2.mp3`
- 🚨 → ✅ HEALTHY → `cute.mp3`

## Как проверить

### На Vision Pi

```bash
# 1. Слушать триггеры звуков
ssh ros2@10.1.1.21
docker exec perception bash -c "ros2 topic echo /voice/sound/trigger"

# 2. Слушать мысли Reflection
docker exec perception bash -c "ros2 topic echo /reflection/internal_thought"

# 3. Перезапустить и услышать приветствие
docker restart perception voice-assistant
docker logs perception -f | grep "Startup Greeting"
```

### Ручная проверка звуков

```bash
# В voice-assistant контейнере должен быть Sound Node
docker exec voice-assistant bash -c "ros2 topic pub /voice/sound/trigger std_msgs/msg/String \"data: 'thinking'\""
docker exec voice-assistant bash -c "ros2 topic pub /voice/sound/trigger std_msgs/msg/String \"data: 'surprise'\""
```

## Доступные звуки

| Звук | Когда играет |
|------|--------------|
| `thinking` | Размышление, загрузка |
| `surprise` | Удивление, новая информация |
| `confused` | Непонимание, проблемы (лёгкие) |
| `angry` | Проблемы, ошибки |
| `angry_2` | Критические проблемы |
| `cute` | Радость, успех |
| `very_cute` | Особая радость |
| `talk_1..4` | Разговорные звуки |

## ROS Topics

```
/voice/sound/trigger (String)          → Sound Node (триггер)
/voice/sound/state (String)            ← Sound Node (состояние)
/reflection/internal_thought (String)  ← Reflection (мысли)
/perception/context_update (...)       ← Context Aggregator
```

## Параметры

### Startup Greeting
```yaml
wait_time: 5.0          # Минимальное ожидание
check_timeout: 30.0     # Максимальное ожидание
enable_greeting: true   # Включить приветствие
```

### Health Monitor
```yaml
enable_sounds: true     # Звуки при изменении статуса
```

## Отключить звуки

### Временно (один запуск)
```bash
# В launch файле установить параметр
enable_sounds: false
enable_greeting: false
```

### Навсегда
```bash
# В docker-compose.yml установить environment
ENABLE_SOUNDS: "false"
```

## 📚 Полная документация

См. [SOUND_EFFECTS_INTEGRATION.md](./SOUND_EFFECTS_INTEGRATION.md)

## 🐛 Troubleshooting

**Звуки не играют:**
1. Проверить Sound Node: `docker ps | grep voice`
2. Проверить топик: `ros2 topic echo /voice/sound/trigger`
3. Проверить логи: `docker logs voice-assistant | grep sound_node`

**Приветствие не звучит:**
1. Проверить логи: `docker logs perception | grep "Startup Greeting"`
2. Проверить параметр: `enable_greeting: true`
3. Проверить Context Aggregator: должен публиковать данные

**Reflection не издаёт звуки:**
1. Проверить мысли: `ros2 topic echo /reflection/internal_thought`
2. Проверить триггеры: `ros2 topic echo /voice/sound/trigger`
3. Проверить логи: `docker logs perception | grep "🎵"`
