# Тестирование Mapping Commands (RTABMap Control)

## ✅ Реализовано

### Интеграция в dialogue_node.py

**Добавлено**:
1. ✅ ROS2 Service Clients для RTABMap:
   - `/rtabmap/reset_memory` - сброс БД (новая карта)
   - `/rtabmap/set_mode_mapping` - переключение в SLAM mode
   - `/rtabmap/set_mode_localization` - переключение в Localization mode

2. ✅ Intent Detection (3 команды):
   - **start_mapping**: "исследуй территорию", "начни исследование", "создай новую карту"
   - **continue_mapping**: "продолжи исследование", "продолжай карту"
   - **finish_mapping**: "закончи исследование", "перейди в навигацию", "карта готова"

3. ✅ Confirmation System:
   - Запрос подтверждения для `start_mapping`
   - Timeout 30 секунд
   - Ответы: "да"/"нет", "давай"/"отмена"

4. ✅ Backup System:
   - Автоматический backup перед `reset_memory`
   - Путь: `/maps/backups/rtabmap_backup_<timestamp>.db`
   - Через Docker exec в контейнер rtabmap

5. ✅ Sound Triggers:
   - `confused` - при запросе подтверждения
   - `cute` - при успешном выполнении
   - `angry_2` - при ошибке

---

## 🧪 Локальное тестирование (симуляция)

### Тест 1: Start Mapping (с подтверждением)

```bash
# Терминал 1: Запуск dialogue_node
cd ~/voice_ws
source install/setup.bash
source /home/ros2/rob_box_project/src/rob_box_voice/.env.secrets
ros2 run rob_box_voice dialogue_node

# Терминал 2: Симуляция STT
# 1. Wake word + команда
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'робок исследуй территорию'}"

# Ожидаемый ответ:
# "Начать новое исследование? Старая карта будет сохранена в резервную копию."
# Звук: confused.mp3

# 2. Подтверждение
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'да'}"

# Ожидаемый результат:
# - Backup БД в /maps/backups/
# - Вызов /rtabmap/reset_memory
# - Ответ: "Начинаю исследование. Старая карта сохранена."
# - Звук: cute.mp3
```

### Тест 2: Continue Mapping

```bash
# Терминал 2: Симуляция
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'робок продолжи исследование'}"

# Ожидаемый результат:
# - Вызов /rtabmap/set_mode_mapping
# - Ответ: "Продолжаю исследование территории."
# - Звук: cute.mp3
```

### Тест 3: Finish Mapping

```bash
# Терминал 2: Симуляция
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'робок закончи исследование'}"

# Ожидаемый результат:
# - Вызов /rtabmap/set_mode_localization
# - Ответ: "Заканчиваю исследование. Переключаюсь в режим навигации."
# - Звук: cute.mp3
```

### Тест 4: Отмена подтверждения

```bash
# Терминал 2: Симуляция
# 1. Команда
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'робок исследуй территорию'}"

# 2. Отмена
ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: 'нет'}"

# Ожидаемый результат:
# - Ответ: "Хорошо, операция отменена."
# - Без вызова сервисов
```

---

## 🚀 Тестирование на Vision Pi (с Main Pi)

### Предварительная проверка

```bash
# На Main Pi (10.1.1.20) - проверить наличие RTABMap сервисов
ssh ros2@10.1.1.20
docker exec rtabmap bash -c "source /opt/ros/kilted/setup.bash && ros2 service list | grep rtabmap"

# Ожидаемый вывод:
# /rtabmap/reset_memory
# /rtabmap/set_mode_mapping
# /rtabmap/set_mode_localization
# /rtabmap/pause
# /rtabmap/resume
```

### Голосовое тестирование

**На Vision Pi (10.1.1.21)**:

```bash
# 1. Перезапустить voice-assistant контейнер с новым образом
ssh ros2@10.1.1.21
docker pull ghcr.io/krikz/rob_box:voice-assistant-kilted-test
docker restart voice-assistant

# 2. Проверить логи dialogue_node
docker logs -f voice-assistant | grep "Mapping"

# 3. Голосовые команды:
# - "Робок, исследуй территорию" → подтверждение → "Да"
# - "Робок, продолжи исследование"
# - "Робок, закончи исследование"
```

### Мониторинг RTABMap

**На Main Pi**:

```bash
# Проверить текущий режим
docker exec rtabmap bash -c "source /opt/ros/kilted/setup.bash && ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory"

# true = SLAM mode (mapping)
# false = Localization mode

# Проверить logи rtabmap
docker logs --tail 20 rtabmap | grep -E "reset|mapping|localization"

# Проверить backup
ls -lh /path/to/maps/backups/
```

---

## 📊 Ожидаемое поведение

### Workflow: Start Mapping

```
Пользователь: "Робок, исследуй территорию"
Робот: 🔔 confused.mp3 + "Начать новое исследование? Старая карта будет сохранена."
Пользователь: "Да"
Робот: 
  1. Docker exec → backup БД
  2. ros2 service call /rtabmap/reset_memory
  3. 🔔 cute.mp3 + "Начинаю исследование. Старая карта сохранена."
```

### Workflow: Continue Mapping

```
Пользователь: "Робок, продолжи исследование"
Робот: 
  1. ros2 service call /rtabmap/set_mode_mapping
  2. 🔔 cute.mp3 + "Продолжаю исследование территории."
```

### Workflow: Finish Mapping

```
Пользователь: "Робок, закончи исследование"
Робот: 
  1. ros2 service call /rtabmap/set_mode_localization
  2. 🔔 cute.mp3 + "Заканчиваю исследование. Переключаюсь в режим навигации."
```

---

## ⚠️ Известные ограничения

1. **Docker exec для backup**: Требует доступ к Docker socket или прямой запуск внутри контейнера
   - **Решение**: Можно вынести backup в отдельный сервис или использовать SSH к Main Pi

2. **Service clients без wait**: Используется `call_async()` без ожидания ответа
   - **Причина**: Избежать блокировки callback'а
   - **Альтернатива**: Можно добавить timeout с `rclpy.spin_until_future_complete()`

3. **Проверка доступности сервисов**: Не проверяем что rtabmap запущен
   - **Решение**: Добавить `wait_for_service()` с timeout в `__init__()`

---

## 🔧 Возможные улучшения

1. **Backup через ROS2 Action**:
   - Создать custom action для backup
   - Прогресс-бар во время backup

2. **Проверка состояния RTABMap**:
   - Подписаться на `/rtabmap/info`
   - Показывать текущий режим в dialogue

3. **Визуализация в Reflection Node**:
   - Добавить awareness о текущем режиме картографии
   - Упоминать в мыслях "Сейчас я исследую территорию"

4. **Rollback при ошибке**:
   - Если reset_memory failed → восстановить из backup

5. **Прогресс картографии**:
   - "Исследовано X% территории" (через /rtabmap/info)

---

## 📝 Checklist развёртывания

### Локальное тестирование
- [ ] Симуляция start_mapping с подтверждением
- [ ] Симуляция continue_mapping
- [ ] Симуляция finish_mapping
- [ ] Проверка отмены подтверждения
- [ ] Проверка timeout подтверждения

### На роботе (Vision Pi + Main Pi)
- [ ] Проверить наличие RTABMap сервисов на Main Pi
- [ ] Пересобрать voice-assistant образ (с 11aaa61 + mapping code)
- [ ] Голосовое тестирование 3 команд
- [ ] Проверка backup создан в /maps/backups/
- [ ] Проверка переключения режимов (Mem/IncrementalMemory)
- [ ] Мониторинг логов rtabmap

### Финальная проверка
- [ ] Звуковые эффекты работают (confused, cute, angry_2)
- [ ] TTS озвучивает ответы корректно
- [ ] Нет ошибок в логах dialogue_node
- [ ] Нет ошибок в логах rtabmap

---

**Статус**: ✅ Код реализован и собран  
**Следующий шаг**: Локальное тестирование симуляцией  
**Блокеры**: Нет (сборка успешна)

---

**Автор**: AI Assistant  
**Дата**: 2025-10-18  
**Коммит**: Pending (после тестирования)
