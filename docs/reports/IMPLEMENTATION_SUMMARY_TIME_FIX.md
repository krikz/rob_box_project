# Fix: Robot Unable to Tell Time - Implementation Summary

## Problem Statement

When the user asked the robot "сколько времени?" (what's the time?), the robot responded:
```
"Извините, у меня нет доступа к точному времени."
(Sorry, I don't have access to the exact time.)
```

This occurred despite the time information being available through the `TimeAwarenessProvider` in the perception system.

## Root Cause Analysis

1. **Time data is available**: The `context_aggregator_node` collects time information via `TimeAwarenessProvider` and publishes it to `/perception/context_update` topic in the `PerceptionEvent` message.

2. **Dialogue node subscribed but not using it**: The `dialogue_node` was already subscribed to `/perception/context_update` for internet monitoring, but it only extracted the `internet_available` field.

3. **Missing context in LLM requests**: The time information was never passed to the DeepSeek API, so the language model couldn't answer time-related questions.

## Solution Implementation

### 1. Store Time Context in dialogue_node.py

Added storage for time information from perception updates:

```python
# Added in __init__
self.current_time_info = None  # Store time information from perception

# Updated _on_perception_update callback
def _on_perception_update(self, msg):
    # ... existing internet status code ...
    
    # Update time information
    if hasattr(msg, 'time_context_json') and msg.time_context_json:
        try:
            self.current_time_info = json.loads(msg.time_context_json)
            self.get_logger().debug(f'🕐 Обновлено время: {self.current_time_info.get("time_only", "N/A")}')
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'⚠️  Ошибка парсинга time_context_json: {e}')
            self.get_logger().debug(f'   Raw JSON: {msg.time_context_json[:100]}...')
```

**Time context includes:**
- `time_only`: "10:30"
- `date_only`: "2024-10-24"
- `period_ru`: "утро", "день", "вечер", "ночь"
- `weekday_ru`: "Понедельник", "Вторник", etc.
- Plus additional fields (hour, minute, timestamp, etc.)

### 2. Inject Time into System Prompt

Created a new method to build an enhanced system prompt with time context:

```python
# Constants for reliable prompt manipulation
TIME_CONTEXT_MARKER = '# Формат ответа'
TIME_CONTEXT_SECTION_TITLE = '# Текущее время'

def _build_system_prompt_with_context(self) -> str:
    """Построить system prompt с добавлением текущего времени"""
    base_prompt = self.system_prompt
    
    if self.current_time_info:
        time_context = []
        time_context.append(f"\n{self.TIME_CONTEXT_SECTION_TITLE}\n")
        time_context.append(f"**Сейчас:** {self.current_time_info.get('time_only', 'N/A')}")
        time_context.append(f"**Дата:** {self.current_time_info.get('date_only', 'N/A')}")
        time_context.append(f"**Период суток:** {self.current_time_info.get('period_ru', 'N/A')}")
        time_context.append(f"**День недели:** {self.current_time_info.get('weekday_ru', 'N/A')}")
        
        time_info = '\n'.join(time_context)
        
        # Insert before "# Формат ответа" section
        if self.TIME_CONTEXT_MARKER in base_prompt:
            prompt_parts = base_prompt.split(self.TIME_CONTEXT_MARKER, 1)
            return f"{prompt_parts[0]}{time_info}\n\n{self.TIME_CONTEXT_MARKER}{prompt_parts[1]}"
        else:
            # Fallback: append at end
            return f"{base_prompt}\n{time_info}"
    
    return base_prompt
```

**Modified `_ask_deepseek_streaming()`:**
```python
def _ask_deepseek_streaming(self):
    # Use system prompt with time context
    system_prompt_with_context = self._build_system_prompt_with_context()
    
    messages = [
        {"role": "system", "content": system_prompt_with_context},
        *self.conversation_history
    ]
    # ... rest of the method
```

### 3. Update Master Prompt Instructions

Added explicit instructions in `master_prompt_simple.txt` on how to answer time questions:

```markdown
# Вопросы о времени

Если пользователь спрашивает "сколько времени?", "который час?", "какое время?":
- ✅ Используй информацию из секции "Текущее время" (если доступна)
- ✅ Отвечай в формате: "Сейчас ЧЧ часов ММ минут" или "Сейчас ЧЧ:ММ"
- ✅ Добавляй период суток для контекста (утро, день, вечер, ночь)
- ✅ Используй приветствие соответствующее времени суток

**Примеры:**

Вопрос: "Сколько времени?"
Ответ (утро):
{"chunk": 1, "ssml": "<speak>Доброе утро!<break time='300ms'/>Сейчас девять часов тридцать минут.<break time='400ms'/></speak>", "emotion": "happy"}

Вопрос: "Который час?"
Ответ (вечер):
{"chunk": 1, "ssml": "<speak>Добрый вечер!<break time='300ms'/>Сейчас девятнадцать часов пятнадцать минут.<break time='400ms'/></speak>", "emotion": "neutral"}

**ВАЖНО:** Если информация о времени не доступна в системном промпте, скажи: "Извините, у меня нет доступа к точному времени."
```

## How It Works

### Data Flow

1. **Perception System** (`context_aggregator_node`):
   - Creates `TimeAwarenessProvider` instance
   - Gets current time context every cycle
   - Publishes to `/perception/context_update` with `time_context_json` field

2. **Dialogue Node** (`dialogue_node`):
   - Subscribes to `/perception/context_update`
   - Extracts and parses `time_context_json` 
   - Stores in `self.current_time_info`

3. **LLM Request**:
   - When user asks a question
   - `_build_system_prompt_with_context()` injects current time
   - Enhanced prompt sent to DeepSeek API
   - LLM can now answer time questions using the provided context

### Example Enhanced System Prompt

When time is available, the system prompt sent to DeepSeek will include:

```
Вы — интеллектуальный агент, управляющий автономным колесным ровером РОББОКС.

# Характеристики робота
...

# Текущее время

**Сейчас:** 10:30
**Дата:** 2024-10-24
**Период суток:** утро
**День недели:** Четверг

# Формат ответа - JSON streaming
...
```

## Testing

### Manual Testing

1. **Start the system**:
   ```bash
   # On Vision Pi
   cd ~/rob_box_project/docker/vision
   docker-compose up -d
   ```

2. **Monitor logs**:
   ```bash
   docker logs -f dialogue_node
   ```

3. **Ask the robot**:
   - "Роббокс, сколько времени?"
   - "Робот, который час?"
   - "Роббокс, какое сейчас время?"

4. **Expected response**:
   ```
   [dialogue_node] 👤 User: Робот сколько время [State: IDLE]
   [dialogue_node] 👋 Wake word обнаружен → LISTENING
   [dialogue_node] 🤔 Запрос к DeepSeek...
   [dialogue_node] 📤 Chunk 1: <speak>Доброе утро!<break time='300ms'/>...
   ```

   Robot should say something like:
   - "Доброе утро! Сейчас десять часов тридцать минут."
   - "Добрый вечер! Сейчас девятнадцать часов пятнадцать минут."

### Verification in Logs

Look for these log messages:

```
[dialogue_node] ✅ Подписан на /perception/context_update для мониторинга интернета и времени
[dialogue_node] 🕐 Обновлено время: 10:30
```

### Troubleshooting

**If robot still says "no access to time":**

1. Check perception updates are being received:
   ```bash
   ros2 topic echo /perception/context_update --once
   ```
   Verify `time_context_json` field is populated.

2. Check dialogue_node logs for time updates:
   ```bash
   docker logs dialogue_node 2>&1 | grep "Обновлено время"
   ```

3. If no time updates, check context_aggregator is running:
   ```bash
   ros2 node list | grep context_aggregator
   ```

## Files Changed

1. **src/rob_box_voice/rob_box_voice/dialogue_node.py**
   - Added `self.current_time_info` storage
   - Enhanced `_on_perception_update()` to parse time context
   - Created `_build_system_prompt_with_context()` method
   - Modified `_ask_deepseek_streaming()` to use enhanced prompt
   - Added constants for marker and section title

2. **src/rob_box_voice/prompts/master_prompt_simple.txt**
   - Added "Вопросы о времени" section
   - Provided examples for time-related responses
   - Added instructions for time-appropriate greetings

## Code Quality

- ✅ Python syntax validated
- ✅ No security issues (CodeQL scan passed)
- ✅ Code review suggestions implemented
- ✅ Follows existing code patterns
- ✅ Maintains backward compatibility

## Benefits

1. **User Experience**: Robot can now answer time questions naturally
2. **Context Awareness**: Time-appropriate greetings (good morning, good evening)
3. **Minimal Changes**: Only modified necessary files, no breaking changes
4. **Robust Design**: Uses constants and error handling for reliability
5. **Debuggable**: Added debug logging for troubleshooting

## Notes

- Time information updates automatically from perception system
- No additional dependencies required (uses existing perception infrastructure)
- Gracefully handles cases where time info is not available
- Compatible with existing fallback mechanisms for offline mode
