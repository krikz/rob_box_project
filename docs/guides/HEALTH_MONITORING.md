# System Health Monitoring - Быстрый старт

## Что это?

**reflection_node** автоматически мониторит здоровье всех ROS2 нод через топик `/rosout`.

## Как это работает?

### Автоматический мониторинг
```bash
# Просто запусти reflection_node - мониторинг уже встроен!
ros2 launch rob_box_perception internal_dialogue.launch.py
```

**reflection_node** автоматически:
- ✅ Слушает `/rosout` (все логи системы)
- ✅ Собирает ERROR и WARN от всех нод
- ✅ Проверяет активность топиков
- ✅ Добавляет `system_health` в контекст для AI
- ✅ Сообщает пользователю о критичных проблемах

### Ручная диагностика
```bash
# Запустить health monitor для отладки
ros2 run rob_box_perception health_monitor
```

**Выводит каждые 5 секунд:**
```
======================================================================
🏥 HEALTH REPORT
======================================================================
Status: ⚠️  DEGRADED
Total Errors: 3 (последние 2 за минуту)
Total Warnings: 5

--- Recent Errors ---
  [ERROR] oak_d_node (15s ago): Failed to read frame from camera
  [ERROR] rtabmap (45s ago): Map optimization failed

--- Recent Warnings ---
  [WARN] nav2_controller (10s ago): Goal tolerance exceeded
  [WARN] battery_monitor (25s ago): Low battery voltage detected
======================================================================
```

## Health Statuses

| Статус | Описание | Действия робота |
|--------|----------|-----------------|
| ✅ **HEALTHY** | Всё работает нормально | Молчит |
| ⚠️ **DEGRADED** | Есть проблемы (5+ ошибок за минуту, отсутствие данных) | Может сообщить, если важно |
| 🚨 **CRITICAL** | Критичные ошибки (FATAL, fail, crash) | **ОБЯЗАТЕЛЬНО** сообщит пользователю |

## Примеры реакций робота

### Critical
```
Пользователь: "Почему ты стоишь?"
Робот: "У меня проблемы с камерой, не могу видеть окружение. Нужна помощь!"
```

### Degraded
```
# Робот замечает много ошибок навигации
Робот: "Много ошибок в SLAM, возможно карта устарела. Пересканировать?"
```

### Healthy
```
# Всё ОК - робот НЕ комментирует системные логи
# Говорит только о важных событиях (низкая батарея, обнаружение человека)
```

## Что попадает в контекст AI?

```json
{
  "system_health": {
    "status": "degraded",
    "issues": [
      "⚠️  5 ошибок за последнюю минуту",
      "👁️  Нет данных с камеры"
    ],
    "recent_errors": [
      {
        "node": "oak_d_node",
        "level": "ERROR",
        "message": "Failed to read frame",
        "timestamp": 1697123456.78
      }
    ],
    "recent_warnings": [
      {
        "node": "nav2_controller",
        "message": "Goal tolerance exceeded",
        "timestamp": 1697123450.12
      }
    ]
  }
}
```

## Проверка топиков

reflection_node автоматически проверяет (каждые 10 секунд):
- 👁️ `/perception/vision_context` - есть ли данные с камеры?
- 🚗 `/odom` - работает ли одометрия?

Если топик долго не обновляется → добавляется в `system_health.issues`

## Интеграция с диалогом

### Пример 1: Критичная ошибка
```python
# reflection_node видит в /rosout:
[FATAL] camera_driver: "Device disconnected!"

# AI получает:
system_health = {
  "status": "critical",
  "issues": ["🚨 1 критичных ошибок за 30 сек"],
  "recent_errors": [{"node": "camera_driver", "level": "FATAL", ...}]
}

# DeepSeek решает:
{
  "thought": "Камера отключилась, это критично для навигации",
  "should_speak": true,
  "speech": "Внимание! Камера отключилась, я не могу видеть окружение"
}
```

### Пример 2: Degraded, но не критично
```python
# Много WARNING от Nav2
system_health = {
  "status": "degraded",
  "issues": ["⚠️  10 ошибок за последнюю минуту"]
}

# DeepSeek решает:
{
  "thought": "Nav2 выдаёт много предупреждений, но пока работает",
  "should_speak": false,  # НЕ сообщать пока
  "speech": ""
}
```

## Тестирование

### Генерация тестовой ошибки
```bash
# Эмулируем ERROR от камеры
ros2 topic pub /rosout rcl_interfaces/msg/Log \
  "{level: 40, name: 'test_node', msg: 'Camera FAIL!'}" \
  --once

# Смотри логи reflection_node
ros2 topic echo /reflection/internal_thought
```

### Мониторинг в реальном времени
```bash
# Terminal 1: Health monitor
ros2 run rob_box_perception health_monitor

# Terminal 2: Внутренние мысли
ros2 topic echo /reflection/internal_thought

# Terminal 3: Речь робота
ros2 topic echo /voice/tts/request
```

## Настройка

В `docker-compose.yaml` или launch файле:
```yaml
environment:
  # Частота проверки здоровья (встроена в reflection rate)
  REFLECTION_RATE: "1.0"  # 1 Hz = проверка каждую секунду
```

reflection_node проверяет топики каждые 10 секунд автоматически.

## Отладка

### Смотреть все логи системы
```bash
ros2 topic echo /rosout
```

### Фильтровать только ошибки
```bash
ros2 topic echo /rosout | grep "level: 40\|level: 50"
```

### Список активных нод
```bash
ros2 node list
```

### Проверить, публикуются ли важные топики
```bash
ros2 topic hz /perception/vision_context
ros2 topic hz /odom
```

## Best Practices

1. **Не перегружай логами** - reflection_node игнорирует свои собственные логи
2. **FATAL = критично** - используй уровень 50 только для реальных аварий
3. **ERROR = важно** - используй уровень 40 для ошибок требующих внимания
4. **WARN = FYI** - используй уровень 30 для предупреждений
5. **health_monitor** - запускай при отладке, не нужен в production

## FAQ

**Q: Будет ли робот болтать о каждой ошибке?**
A: НЕТ! DeepSeek AI решает говорить только при critical или важных degraded случаях.

**Q: Как отключить мониторинг?**
A: Нельзя отключить - это часть reflection_node. Но можно настроить system_prompt чтобы AI игнорировал system_health.

**Q: Влияет ли мониторинг на производительность?**
A: Нет. `/rosout` уже существует, мы просто подписываемся. Храним max 10 errors + 5 warnings в памяти.

**Q: Работает ли это с Zenoh bridge?**
A: ДА! `/rosout` передаётся через Zenoh bridge между Main Pi и Vision Pi.

---

**Готово к использованию!** 🚀

Просто запусти `internal_dialogue.launch.py` и робот автоматически начнёт мониторить систему.
