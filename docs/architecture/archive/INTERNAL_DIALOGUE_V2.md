# Internal Dialogue Architecture v2.0

> **⚠️ ВНИМАНИЕ:** Эта документация устарела. Актуальная полная документация:
> 
> **[dialog-v2-overview.md](dialog-v2-overview.md)**
>
> Новый документ включает:
> - Event-driven архитектура с Context Aggregator (MPC lite)
> - Суммаризация по типам: speech/vision/system
> - Интеграция с Voice Assistant и urgent hook
> - Полные диаграммы и примеры кода

---

# Internal Dialogue Architecture v2.0 (УСТАРЕЛО)

## 🏗️ Архитектура

```
┌─────────────────────────────────────────────────────────────┐
│  PERCEPTION LAYER (Context Aggregator - MPC lite)           │
│  Собирает данные → публикует события                        │
└─────────────────────────────────────────────────────────────┘
         │ /perception/context_update (PerceptionEvent)
         │ /perception/user_speech (String)
         ↓
┌─────────────────────────────────────────────────────────────┐
│  REFLECTION LAYER (Internal Dialogue Agent)                  │
│  Event-Driven: события → думает → речь                      │
└─────────────────────────────────────────────────────────────┘
         │ /reflection/internal_thought (String)
         │ /voice/tts/request (String)
         ↓
    Voice Assistant
```

## 📦 Компоненты

### 1. context_aggregator_node (MPC lite)
**Роль:** Сборщик данных восприятия

**Подписывается на:**
- `/perception/vision_context` - vision processing
- `/rtabmap/localization_pose` - позиция на карте
- `/odom` - одометрия
- `/device/snapshot` - сенсоры ESP32
- `/apriltag/detections` - AprilTag маркеры
- `/rosout` - системные логи (ERROR/WARN)
- `/voice/stt/result` - входящая речь пользователя

**Публикует:**
- `/perception/context_update` (PerceptionEvent) - агрегированное событие
- `/perception/user_speech` (String) - транзит STT для рефлексии

**Особенности:**
- **НЕ думает**, **НЕ принимает решений**
- Только сбор и публикация
- Частота событий: 2 Hz (configurable)
- Мониторинг здоровья системы
- Короткая память событий (60 сек)

### 2. reflection_node (Internal Dialogue Agent v2.0)
**Роль:** Внутренний диалог робота

**Подписывается на:**
- `/perception/context_update` (PerceptionEvent)
- `/perception/user_speech` (String)
- `/voice/dialogue/response` (String) - для отслеживания диалога

**Публикует:**
- `/reflection/internal_thought` (String) - внутренние мысли
- `/voice/tts/request` (String) - речь робота

**Особенности:**
- **Event-Driven**: размышляет при получении события (не по таймеру!)
- **Hook для срочных ответов**: детектор личных вопросов
  * "как дела?", "как ты?", "что у тебя?" → быстрый ответ (<2 сек)
  * Interrupt цикла размышлений
  * Специальный prompt для коротких ответов
- DeepSeek API для генерации мыслей
- Управление диалогом (тайм-аут 10 сек)

## 🔄 Поток данных

### Обычное размышление:
```
1. context_aggregator получает данные со всех топиков
2. Агрегирует в PerceptionEvent (каждые 0.5 сек)
3. Публикует /perception/context_update
4. reflection_node получает событие
5. Проверяет: диалог активен?
   - Да → ждёт тайм-аута
   - Нет → думает
6. DeepSeek анализирует контекст
7. Генерирует мысль + решение говорить или нет
8. Публикует thought и/или speech
```

### Срочный ответ на личный вопрос:
```
1. Пользователь: "Робот, как у тебя дела?"
2. STT → /voice/stt/result
3. context_aggregator → транзит → /perception/user_speech
4. reflection_node получает речь
5. Детектор: это личный вопрос! 🎯
6. Устанавливает pending_user_speech
7. При следующем context_update (макс 0.5 сек):
   - Interrupt обычного цикла
   - Краткий prompt для быстрого ответа
   - DeepSeek генерирует короткий ответ (1-2 предложения)
   - Публикует speech немедленно
8. Время ответа: <2 секунды
```

## 🎯 Детектор личных вопросов

Regex паттерны (case-insensitive):
- `\bкак\s+(у\s+тебя\s+)?дела\b`
- `\bкак\s+ты\b`
- `\bчто\s+у\s+тебя\b`
- `\bкак\s+твоё?\s+(настроение|самочувствие)\b`
- `\bчто\s+(делаешь|происходит)\b`
- `\bкак\s+себя\s+чувствуешь\b`

## 📊 Message: PerceptionEvent

```
builtin_interfaces/Time stamp
string vision_context           # JSON от vision processing
geometry_msgs/Pose pose          # Позиция на карте
geometry_msgs/Twist velocity     # Скорость движения
bool is_moving                   # Робот движется?
float32 battery_voltage          # Батарея
float32 temperature              # Температура
int32[] apriltag_ids             # Обнаруженные маркеры
string system_health_status      # "healthy", "degraded", "critical"
string[] health_issues           # Список проблем
string memory_summary            # Краткое резюме памяти
```

## 🚀 Запуск

### Локально (с vision stub):
```bash
ros2 launch rob_box_perception internal_dialogue.launch.py
```

Запускает:
- vision_stub_node (заглушка камеры)
- context_aggregator
- reflection_node

### Docker (production):
```bash
docker compose up perception
```

Запускает:
- context_aggregator
- reflection_node

## ⚙️ Параметры

### context_aggregator:
- `publish_rate` (float, default: 2.0) - Частота событий (Hz)
- `memory_window` (int, default: 60) - Окно памяти (сек)

### reflection_node:
- `dialogue_timeout` (float, default: 10.0) - Тайм-аут диалога (сек)
- `enable_speech` (bool, default: true) - Включить речь
- `system_prompt_file` (string, default: reflection_prompt.txt)
- `urgent_response_timeout` (float, default: 2.0) - Тайм-аут срочного ответа (сек)

### Environment variables (Docker):
```bash
DIALOGUE_TIMEOUT=10.0
URGENT_RESPONSE_TIMEOUT=2.0
ENABLE_SPEECH=true
CONTEXT_PUBLISH_RATE=2.0
MEMORY_WINDOW=60
DEEPSEEK_API_KEY=sk-xxx  # Required!
```

## 🧪 Тестирование

### Проверить топики:
```bash
ros2 topic list | grep perception
# /perception/context_update
# /perception/user_speech

ros2 topic hz /perception/context_update
# average rate: 2.000
```

### Смотреть события:
```bash
ros2 topic echo /perception/context_update
```

### Смотреть мысли:
```bash
ros2 topic echo /reflection/internal_thought
```

### Тест срочного ответа:
```bash
# Публикуем личный вопрос
ros2 topic pub --once /voice/stt/result std_msgs/String "data: 'Робот как у тебя дела'"

# Смотрим логи (должен быть быстрый ответ)
# 🎯 Обнаружен личный вопрос → срочный ответ
# ⚡ Срочный ответ на личный вопрос
# 🗣️  Говорю: "У меня всё отлично, готов к работе!"
```

## 📈 Преимущества новой архитектуры

✅ **Разделение ответственности**: сбор данных ≠ размышления  
✅ **Event-Driven**: нет пустых циклов, экономия CPU  
✅ **Масштабируемость**: легко добавить новые источники в aggregator  
✅ **Срочные ответы**: hook для личных вопросов (<2 сек)  
✅ **Unified events**: все данные в одном PerceptionEvent  
✅ **Тестируемость**: можно тестировать ноды независимо  

## 🔧 Разработка

### Добавить новый источник данных:
1. Открыть `context_aggregator_node.py`
2. Добавить subscription на новый топик
3. Обновить callback для сохранения данных
4. Обновить `publish_event()` для включения в PerceptionEvent
5. (Опционально) Обновить `PerceptionEvent.msg` если нужны новые поля

### Добавить новый паттерн личного вопроса:
1. Открыть `reflection_node.py`
2. Найти метод `_is_personal_question()`
3. Добавить regex паттерн в список `personal_patterns`

## 📝 История версий

**v2.0** (current) - Event-Driven архитектура
- Разделение на context_aggregator + reflection_node
- Hook для срочных ответов
- PerceptionEvent message

**v1.0** - Monolithic архитектура
- Всё в одной ноде (reflection_node_old.py)
- Timer-based размышления
- Прямые подписки на все топики
