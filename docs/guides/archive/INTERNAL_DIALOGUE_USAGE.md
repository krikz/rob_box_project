# Internal Dialogue + Wake Word Detection - Инструкции

> **⚠️ ВНИМАНИЕ:** Эта документация устарела. Актуальная полная документация:
> 
> **[dialog-v2-overview.md](../architecture/dialog-v2-overview.md)**
>
> Новый документ включает:
> - Объединённое описание Internal Dialogue + Voice Assistant
> - Суммаризацию по типам событий (speech/vision/system)
> - Urgent hook механизм (ask_reflection)
> - Wake word detection и команда "помолчи"
> - Hardware AEC на ReSpeaker
> - Полную конфигурацию и примеры

---

# Internal Dialogue + Wake Word Detection - Инструкции (УСТАРЕЛО)

## 🎯 Архитектура Системы

### Две Отдельные Сущности:

1. **Internal Dialogue (rob_box_perception/reflection_node)**
   - Всегда слушает ВСЁ (датчики, STT, ошибки, vision)
   - Думает внутренне, публикует в `/reflection/internal_thought`
   - Может вмешаться БЕЗ wake word когда релевантно
   - "Помолчи" → `silence_until` (5 мин), продолжает думать но не говорит

2. **Voice Assistant (rob_box_voice/dialogue_node)**
   - Требуется wake word: `робок`, `робот`, `роббокс`, `робокос`, `роббос`, `робокс`
   - State machine: `IDLE → LISTENING → DIALOGUE → SILENCED`
   - "Помолчи" → SILENCED state (5 мин), останавливает TTS, только команды

---

## 🔧 Настройка

### 1. Настройка AEC для ReSpeaker

Перед первым запуском настройте эхоподавление:

```bash
# Source workspace
cd ~/rob_box_project
source install/setup.bash

# Запустить конфигурацию AEC
ros2 run rob_box_voice configure_respeaker_aec
```

Скрипт настроит:
- `AECFREEZEONOFF=0` (адаптивный режим)
- `ECHOONOFF=1` (эхоподавление включено)
- `NLATTENONOFF=1` (нелинейное подавление включено)

**Важно:** Настройки сохраняются до перезагрузки устройства. Channel 0 используется для STT (обработанный сигнал).

---

## 🚀 Запуск Системы

### Запуск полной системы:

```bash
cd ~/rob_box_project
source install/setup.bash

# Запустить все компоненты
ros2 launch rob_box_bringup voice_assistant.launch.py
```

### Или по отдельности:

```bash
# Terminal 1: Audio + STT
ros2 run rob_box_voice audio_node
ros2 run rob_box_voice stt_node

# Terminal 2: Internal Dialogue
ros2 run rob_box_perception reflection_node

# Terminal 3: Voice Assistant
ros2 run rob_box_voice dialogue_node

# Terminal 4: TTS
ros2 run rob_box_voice tts_node

# Terminal 5: Commands (опционально)
ros2 run rob_box_voice command_node
```

---

## 📖 Примеры Использования

### 1. Internal Dialogue (без wake word)

**Пользователь:** "как у тебя дела?"  
**Reflection Node:** (думает) "личный вопрос, надо ответить"  
**Ответ:** "Хорошо, спасибо! Всё работает нормально."

**Характеристики:**
- ✅ Работает БЕЗ wake word
- ✅ Реагирует на личные вопросы, важные события
- ✅ Краткие ответы (1-2 предложения)

### 2. Voice Assistant (с wake word)

**Пользователь:** "робот привет!"  
**State:** `IDLE → LISTENING`  
**Ответ:** "Привет! Чем могу помочь?"

**Пользователь:** "расскажи про теорему Пифагора"  
**State:** `LISTENING → DIALOGUE`  
**Ответ:** (полный ответ от LLM через dialogue_node)

**Характеристики:**
- ✅ Требуется wake word для активации
- ✅ Полноценный диалог с LLM
- ✅ 30 сек таймаут → `IDLE`

### 3. Команда "Помолчи" (останавливает оба)

**Пользователь:** "помолчи" / "замолчи" / "хватит"  
**Действие:**
1. Немедленная остановка TTS (STOP команда)
2. Reflection: `silence_until = now + 300` (5 мин)
3. Dialogue: `state = SILENCED`, `silence_until = now + 300`

**В режиме Silence:**
- Reflection: думает, но не говорит
- Dialogue: только команды с wake word, диалог недоступен

**Восстановление:**
- Автоматически через 5 минут
- Или команда с wake word: "робот привет"

### 4. Dialogue vs Command Conflict (решено!)

**Пользователь:** "робот иди вперёд"  
**State:** `IDLE → LISTENING → DIALOGUE`  
**Command Node:** (игнорирует, dialogue активен)  
**Dialogue Node:** обрабатывает как команду или вопрос

**Пользователь:** (без wake word) "иди вперёд"  
**State:** `IDLE`  
**Command Node:** обрабатывает команду  
**Dialogue Node:** игнорирует (нет wake word)

---

## 🔍 Мониторинг Системы

### Просмотр состояний:

```bash
# State машины dialogue_node
ros2 topic echo /voice/dialogue/state

# Internal thoughts
ros2 topic echo /reflection/internal_thought

# TTS state
ros2 topic echo /voice/tts/state

# STT результаты
ros2 topic echo /voice/stt/result
```

### Тестирование silence команды:

```bash
# Отправить STOP в TTS
ros2 topic pub --once /voice/tts/control std_msgs/msg/String "data: 'STOP'"
```

---

## 🧪 Тестовые Сценарии

### Сценарий 1: Reflection без wake word
```
Действие: "как дела?"
Ожидание: reflection_node отвечает БЕЗ wake word
Проверка: /reflection/internal_thought публикуется
```

### Сценарий 2: Dialogue с wake word
```
Действие: "робот привет"
Ожидание: dialogue_node переходит в LISTENING
Проверка: /voice/dialogue/state = 'LISTENING'
```

### Сценарий 3: Silence команда
```
Действие: во время речи скажите "помолчи"
Ожидание: 
  - TTS немедленно останавливается
  - state = 'SILENCED' (5 мин)
Проверка: /voice/tts/state = 'stopped'
```

### Сценарий 4: IDLE игнорирует без wake word
```
Действие: случайная речь без wake word
Ожидание: dialogue_node игнорирует (state = IDLE)
Проверка: command_node НЕ говорит "не понял команду"
```

### Сценарий 5: SILENCED → только команды
```
Действие: после "помолчи" скажите "робот иди вперёд"
Ожидание: команда выполняется, диалог НЕТ
Проверка: /voice/command/intent публикуется
```

---

## 🐛 Отладка

### Reflection не отвечает:
```bash
# Проверить silence_until
ros2 topic echo /reflection/internal_thought

# Логи
ros2 run rob_box_perception reflection_node --ros-args --log-level DEBUG
```

### Dialogue не активируется:
```bash
# Проверить wake words
ros2 param get /dialogue_node wake_words

# Проверить state
ros2 topic echo /voice/dialogue/state
```

### TTS не останавливается:
```bash
# Проверить подписку
ros2 topic info /voice/tts/control

# Отправить STOP вручную
ros2 topic pub --once /voice/tts/control std_msgs/msg/String "data: 'STOP'"
```

### Command node мешает dialogue:
```bash
# Проверить dialogue_state в command_node
ros2 topic echo /voice/dialogue/state

# Должен быть: IDLE / LISTENING / DIALOGUE / SILENCED
```

---

## 📊 Topics Overview

| Topic | Type | Publisher | Subscriber | Описание |
|-------|------|-----------|------------|----------|
| `/voice/stt/result` | String | stt_node | dialogue, command, reflection | STT результаты |
| `/voice/dialogue/state` | String | dialogue_node | command_node | State машины |
| `/voice/dialogue/response` | String | dialogue_node | tts_node | Ответы LLM |
| `/voice/tts/request` | String | dialogue, reflection | tts_node | TTS запросы |
| `/voice/tts/control` | String | dialogue_node | tts_node | STOP команды |
| `/voice/tts/state` | String | tts_node | - | TTS статус |
| `/reflection/internal_thought` | String | reflection_node | - | Внутренние мысли |
| `/perception/user_speech` | String | - | reflection_node | User speech для reflection |

---

## ✅ Статус Реализации

- ✅ Task #1: Prompts для reflection_node
- ✅ Task #2: 'помолчи' команда для reflection_node
- ✅ Task #3: Wake Word Detection
- ✅ Task #4: SILENCED state
- ✅ Task #5: Dialogue vs Command конфликт
- ✅ Task #6: ReSpeaker Hardware AEC
- ✅ Task #7: Обновить dialogue_node prompts
- ✅ Task #8: TTS Stop API
- 🔄 Task #9: Полное тестирование системы

---

## 📝 Коммиты

- `d2c5aa6`: feat: добавить команду 'помолчи' и stop API для TTS
- `57fc7ba`: feat: реализовать Wake Word Detection + SILENCED state в dialogue_node
- `f46ab9b`: feat: добавить инструкции для команд молчания в dialogue_node prompt
- `023f934`: feat: решить конфликт dialogue vs command через state tracking
- `7895ef8`: feat: добавить настройку AEC для ReSpeaker

---

## 🎉 До Победы!

Система полностью реализована! Осталось финальное тестирование.

**Команда для проверки:**
```bash
ros2 launch rob_box_bringup voice_assistant.launch.py
```

**Да прибудет с нами сила! 🚀**
