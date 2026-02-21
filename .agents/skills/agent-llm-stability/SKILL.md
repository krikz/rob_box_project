---
name: agent-llm-stability
description: >
  Специализированный скилл для диагностики и исправления зависаний, деградации контекста и
  нестабильности LLM в агентском режиме (dialogue_node.py + MCP tools). Используй когда:
  робот говорит "не в настроении думать", зависает после нескольких диалогов, смешивает темы,
  вызывает memory_context без запроса, или token count на первой итерации нового диалога > 10k.
---

# Agent LLM Stability — Rob Box

## Архитектура агентного цикла

```
STT → stt_callback → pending_queries → _check_and_process_queue
  → _ask_llm_streaming [ThreadPoolExecutor]
    → client.chat.completions.create(tools=21 MCP tools)
    → for chunk in stream → publish /voice/dialogue/response
    → tool_calls detected → _execute_tool_calls → MCP сервер
    → _continue_after_tool_calls [ThreadPoolExecutor] (рекурсия до 30 итераций)
```

**Ключевые файлы:**
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` — весь агентный цикл
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/` — MCP инструменты
- `src/rob_box_voice/prompts/master_prompt_compact.txt` — системный промпт
- `src/rob_box_voice/rob_box_voice/core/conversation_history.py` — история (скользящее окно max_messages=20)

---

## База знаний: все найденные баги и фиксы

### BUG-1: ThreadPoolExecutor вечное зависание (commit 28aa193)

**Симптом:** Робот перестаёт реагировать на голос после того как LLM завис — STT получает STOP но stt_callback никогда не запускается.

**Root cause:** Паттерн `with ThreadPoolExecutor() as executor:` при FuturesTimeoutError → `__exit__` вызывает `shutdown(wait=True)` → ждёт завершения потока, который навсегда завис на сетевом I/O → ROS2 callback thread заблокирован.

**Фикс:**
```python
# НЕПРАВИЛЬНО:
with ThreadPoolExecutor(max_workers=1) as executor:
    future = executor.submit(_do_streaming)
    future.result(timeout=60.0)  # timeout → __exit__ → shutdown(wait=True) → deadlock

# ПРАВИЛЬНО:
_stream_executor = ThreadPoolExecutor(max_workers=1)
future = _stream_executor.submit(_do_streaming)
try:
    future.result(timeout=60.0)
finally:
    _stream_executor.shutdown(wait=False)  # НЕ блокировать!
```
Применено в **двух местах**: `_ask_llm_streaming` (~line 1165) и `_continue_after_tool_calls` (~line 2385).

---

### BUG-2: preload прошлых сессий в conversation_history (commit 092291d)

**Симптом:** Попросил спеть про мамонтёнка → робот запел про **енотов** (тема прошлой сессии). Первая итерация `input=8853` токенов вместо ~8.7k базовых.

**Root cause:** При инициализации `__init_voice_memory()` загружала `load_recent_turns(limit=15)` из прошлых сессий прямо в `conversation_history`. LLM видел историю где-то про енотов и продолжал её.

**Фикс:** Убрать preload полностью. Прошлые сессии доступны ТОЛЬКО через `memory_context` инструмент когда пользователь явно спрашивает.
```python
# УДАЛИТЬ этот блок из _init_voice_memory():
past_turns = self.voice_memory.load_recent_turns(limit=15, exclude_current_session=True)
if past_turns:
    for turn in past_turns:
        ...conversation_history.add_...
```

---

### BUG-3: conversation_history не очищается между диалогами (commit 092291d)

**Симптом:** После диалога про енотов (30 итераций = ~20k токенов) следующий диалог про мамонтёнка стартует с 8853 токенов уже на первой итерации. LLM запутывается.

**Root cause:** `conversation_history.clear()` никогда не вызывалась — история накапливалась бесконечно (ограничена только `max_messages=20` по количеству сообщений, НЕ токенам). 1 итерация agent = 5+ сообщений (assistant+tool_calls+tool results), 30 итераций = 150+ сообщений.

**Фикс:** Очищать историю при каждом wake word из IDLE:
```python
# В stt_callback при переходе IDLE → LISTENING:
old_len = len(self.conversation_history.get_messages())
self.conversation_history.clear()
if old_len > 0:
    self.get_logger().info(f"🧹 conversation_history очищена при новом wake word ({old_len} сообщений удалено)")
```

---

### BUG-4: provider_error_count не сбрасывается между диалогами (commit 43e9e9c)

**Симптом:** После сессии с таймаутами (например мамонтёнок + лимит 30 итераций), следующий диалог про чечётку показывает `Ошибка 4/3 для DeepSeek` на первом же таймауте → сразу "не в настроении".

**Root cause:** `provider_error_count` инкрементируется при каждом timeout и сбрасывается только при успешном первом токене в стриме. Если попались несколько таймаутов подряд, счётчик `>= threshold(3)` — следующий провал мгновенно срабатывает.

**Фикс:** Сбрасывать счётчик при каждом wake word:
```python
if self.provider_error_count > 0:
    self.get_logger().info(f"♻️ provider_error_count сброшен при wake word ({self.provider_error_count} → 0)")
    self.provider_error_count = 0
```

---

### BUG-5: Нет try/except вокруг create() в рекурсивном стриме (commit 43e9e9c)

**Симптом:** При ошибке создания стрима (DNS, httpx, сеть) в `_do_recursive_streaming` — unhandled exception в потоке, `recursive_result["error"]` не заполняется, внешний поток получает FuturesTimeoutError только через 60с.

**Фикс:**
```python
self.get_logger().info("📞 Создание рекурсивного stream...")
try:
    stream = self.client.chat.completions.create(**request_params)
except Exception as e:
    self.get_logger().error(f"❌ Ошибка создания рекурсивного stream: {e}")
    recursive_result["error"] = f"Failed to create stream: {e}"
    return
self.get_logger().info("✅ Stream создан, начинаю итерацию...")
```

---

### BUG-6: Авто-retry при timeout в _continue_after_tool_calls (commit 43e9e9c)

**Симптом:** Один случайный медленный ответ DeepSeek при `create()` (~60с вместо 0.8с) → сразу "я слишком долго думал".

**Root cause:** DeepSeek иногда принимает TCP соединение но долго не шлёт первый токен. Второй запрос на свежем соединении обычно проходит за 0.8с.

**Фикс:** Один retry с флагом `is_retry` (защита от infinite loop):
```python
def _continue_after_tool_calls(self, ..., iteration=1, is_retry=False):
    ...
    except (FuturesTimeoutError, TimeoutError) as e:
        if not is_retry:
            self.get_logger().warning(f"♻️ Retry рекурсивного запроса (итерация {iteration})...")
            self._continue_after_tool_calls(messages, tool_calls, tool_results, iteration, is_retry=True)
            return
        self._speak_simple("Извините, я слишком долго думал", show_error_animation=True)
```

---

### BUG-7: Время инжектировалось в system_prompt → KV cache miss (commit 154b484)

**Симптом:** DeepSeek KV cache всегда промахивается — каждый запрос биллится по полной цене (1₽/M вместо 0.1₽/M при cache hit). Причина: `_build_system_prompt_with_context()` добавлял текущее время в промпт → prefix всегда разный → cache invalidate.

**Фикс:** Удалить `_build_system_prompt_with_context()` полностью. Создать `GetCurrentTimeTool` (INSTANT execution):
```python
class GetCurrentTimeTool(MCPTool):
    name = "get_current_time"
    execution_type = ToolExecutionType.INSTANT
    def execute(self, **kwargs):
        now = datetime.now()
        return {"time": now.strftime("%H:%M"), "date": ..., "weekday": "пятница", ...}
```
Системный промпт теперь статичен → cache hit на ~8000 токенах промпта.

---

### BUG-8: memory_context вызывается автономно (commit 092291d + промпт)

**Симптом:** При запросе "спой про мамонтёнка" LLM на итерации 13/30 вызвал `memory_context` без запроса и произнёс "В прошлый раз мы говорили о разных вещах!" — пользователь ничего не спрашивал о прошлом.

**Root cause:** В `master_prompt_compact.txt` отсутствовали правила когда вызывать `memory_context`.

**Фикс в промпте:**
```markdown
# Память и прошлые разговоры

**memory_context** — инструмент для получения того, о чём вы говорили РАНЬШЕ.

✅ Вызывай ТОЛЬКО когда пользователь явно спрашивает о прошлом:
- "О чём мы говорили?"
- "Ты помнишь про меня?"
- "В прошлый раз ты говорил..."

❌ НЕ вызывай memory_context при:
- Обычных запросах (спой, расскажи, сделай что-то)
- Вопросах о роботе, природе, математике
- Любых задачах, которые не требуют истории прошлых бесед

**Принцип:** Отвечай на текущий запрос. Если пользователь хочет вспомнить прошлое — он спросит.
```

---

### BUG-9: GetCurrentTimeTool не зарегистрирован в _register_tools() (commit 79456db)

**Симптом:** Пользователь спрашивает "сколько времени?" → робот отвечает "у меня нет доступа к часам" / "нет инструмента для определения времени". Причём на второй попытке говорит "сейчас проверю" но снова отказывает.

**Root cause:** `GetCurrentTimeTool` был создан в `tools/system.py` и экспортирован из `tools/__init__.py`, но **забыли добавить `registry.register()` в `mcp_server.py::_register_tools()`**. MCP сервер не публиковал инструмент на `/mcp/tools` топик → LLM его не видел среди 21 инструмента.

Как проверить что инструмент не зарегистрирован:
```bash
# В контейнере: смотрим что реально опубликовано на /mcp/tools
docker exec voice-assistant bash -c "source /ws/install/setup.bash && ros2 topic echo /mcp/tools --once 2>/dev/null" | grep -o '"name": "[^"]*"'
# Если get_current_time отсутствует → баг
```

**Фикс** в [mcp_server.py](src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py):
```python
# 1. Добавить в импорт:
from .tools import (
    ...
    GetCurrentTimeTool,  # ← добавить
)

# 2. Добавить в _register_tools():
# System tools
self.registry.register(SetVolumeTool(self))
self.registry.register(SetPitchTool(self))
self.registry.register(SetSpeedTool(self))
self.registry.register(GetRobotStatusTool(self))
self.registry.register(GetCurrentTimeTool(self))  # ← добавить
```

**Урок:** Когда добавляешь новый `MCPTool` — нужно обновить **три** места:
1. `tools/<category>.py` — класс инструмента
2. `tools/__init__.py` — экспорт
3. `mcp_server.py::_register_tools()` — **регистрация!** Без этого LLM его не видит.

---

### BUG-10: Робот зависает на 60+ секунд после double LLM timeout (не исправлен)

**Симптом:** Во время агентного диалога (итерация 2/30) срабатывает `⏱️ TIMEOUT → ♻️ Retry`. Retry тоже виснет на 60с. Всё это время робот **молчит и не реагирует** на "Робот покукарекай" — STT логирует запросы, но dialogue_node не отвечает.

**Последовательность из логов:**
```
[6183] ⏱️ TIMEOUT рекурсивного запроса (итерация 2)
[6184] ♻️ Retry рекурсивного запроса (итерация 2)...
[6185] итерация 2/30  [← retry стартовал, ждёт до 60с]
... 38 секунд тишины от dialogue_node ...
[6191-6211] STT видит "Робот покукарекай" (3 раза) — игнорируется!
```

**Root cause:**
1. При State=DIALOGUE (**даже с явным wake word**) `stt_callback` добавляет запрос в `pending_queries` вместо прерывания — обрабатывается только после завершения текущего агентного цикла
2. Retry внутри `_continue_after_tool_calls` занимает ещё до 60с (ThreadPoolExecutor с `future.result(timeout=60.0)`)  
3. Итого: первый timeout(60с) + retry timeout(60с) = **до 120с** пока робот не вернётся в IDLE

**Возможные фиксы (не реализованы):**
```python
# Вариант A: Уменьшить TOTAL_REQUEST_TIMEOUT в recursive запросе с 60с до 30с
RETRY_REQUEST_TIMEOUT = 30.0  # вместо 60.0 для retry

# Вариант B: Приоритетное прерывание диалога по wake word из DIALOGUE
# В stt_callback — если State=DIALOGUE и есть wake word:
if self.llm_processing and is_wake_word:
    self.interrupt_agent_loop = True  # прерываем текущий LLM запрос
    self.conversation_history.clear()
    # перейти в LISTENING немедленно

# Вариант C: Ограничить retry timeout на половину от основного
```

**Как диагностировать:**
```bash
# Признак BUG-10: после TIMEOUT и Retry — тишина в dialogue_node на 60+с
docker logs voice-assistant 2>&1 | grep -n "TIMEOUT\|Retry\|итерация\|State: DIALOGUE" | tail -20
# Если между "Retry" и следующим "Новый диалог" > 60с — это BUG-10
```

---

### BUG-11: Roleplay контекст теряется между запросами (commit d36616b)

**Симптом:** Робот входит в роль (Шариков, пират, учитель) → хорошо отыгрывает первый ответ → на вторый запрос ("Робот расскажи анекдот") говорит уже без роли.

**Последовательность из логов:**
```
t=0s:  "Робот теперь ты Шариков" → wake word IDLE → 🧹 clear history → Новый диалог e1c40b1f
t=22s: LLM 4 итерации обработки → token=10032
t=22s: listen_for_response → ОСТАНОВКА → finally сетает dialogue_in_progress=False
t=30s: _check_dialogue_timeout: elapsed = 30s - 8s_from_start_of_DIALOGUE > 30s → state=IDLE
t=38s: "Робот расскажи анекдот" → wake word IDLE → 🧹 13 сообщений удалено ← Шариков забыт
```

**Root cause (цепочка 3 причин):**
1. `last_interaction_time` обновлялся только через `transition_state()` — один раз при входе в DIALOGUE
2. 22секунды LLM итераций истрачивали большую часть 30с timeout
3. `finally` блок `_continue_after_tool_calls` всегда ставил `dialogue_in_progress = False` (даже при ОСТАНОВКА) → через 8с `check_timeout` → state=IDLE

**Фикс (3 изменения):**
```python
# 1. Обновлять last_interaction_time при каждой итерации (dialogue_node.py ~line 1353):
while iteration < max_iterations:
    iteration += 1
    self.dialogue_manager.last_interaction_time = time.time()  # ← добавить

# 2. Обновлять при ОСТАНОВКА listen_for_response (_continue_after_tool_calls):
if tool_name == 'listen_for_response':
    self.dialogue_manager.last_interaction_time = time.time()  # ← добавить
    self._listen_response_waiting = True  # ← добавить
    self.llm_processing = False
    return

# 3. Защитить finally блок:
finally:
    self.llm_processing = False
    if not self._listen_response_waiting:  # ← добавить
        self.dialogue_in_progress = False
    self._listen_response_waiting = False
```

**Результат:** state=DIALOGUE сохраняется → следующий wake word идёт через DIALOGUE ветку `stt_callback` (не IDLE) → `conversation_history.clear()` **не вызывается** → Шариков помнится.

---

## Диагностика: как читать логи

```bash
# Проверить token context на первой итерации каждого диалога
docker logs voice-assistant 2>&1 | grep "Token usage:" | head -20
# Ожидаемо: input=8600-9000 — НОРМ, input=15000+ — история не чистится

# Проверить tail ошибок провайдера
docker logs voice-assistant 2>&1 | grep -E "TIMEOUT|Ошибка [0-9]+/[0-9]+|provider_error_count"

# Проверить очистку истории при wake word
docker logs voice-assistant 2>&1 | grep "conversation_history очищена\|🧹"

# Проверить что memory_context не вызывается без запроса
docker logs voice-assistant 2>&1 | grep "Выполнение: memory_context"
```

**Признак BUG-3 (не очищается):** `Token usage:` первой итерации нового диалога >> 9k

**Признак BUG-4 (cascade errors):** `Ошибка N/3` где N > 1 при первом же timeout нового диалога

**Признак BUG-1 (deadlock):** `stt_node: STOP команда` получена, но больше нет логов от `dialogue_node` — ROS2 callback заблокирован

**Признак BUG-8 (memory pollution):** `Выполнение: memory_context` в логах когда пользователь не спрашивал о прошлом

**Признак BUG-9 (инструмент не зарегистрирован):** LLM отвечает "нет инструмента" / "нет доступа" хотя класс есть в `tools/`. Проверка: `ros2 topic echo /mcp/tools --once` — нет `get_current_time` в списке.

**Признак BUG-10 (double timeout hang):** После `⏱️ TIMEOUT + ♻️ Retry` — тишина от `dialogue_node` на 60+ секунд. STT логирует wake words но робот молчит.

**Признак BUG-11 (roleplay context lost):** Робот вошёл в роль (Шариков, пират и т.п.) → ответил на первый запрос → на втором уже не в роли. Лог покажет `🧹 conversation_history очищена при новом wake word` после того как был вызван `listen_for_response`.

**Признак BUG-17 (premature "не в настроении"):** `⚠️ Ошибка 1/3` сразу за `TIMEOUT`, затем немедленно `🔊 TTS: Извините, я сейчас не в настроении думать`. `provider_error_count` ещё не достиг threshold (3), но ошибка всё равно произносится.

---

### BUG-17: "не в настроении думать" при первом же таймауте (commit текущий)

**Симптом:** Один timeout на DeepSeek → лог показывает `⚠️ Ошибка 1/3` → сразу произносит "Извините, я сейчас не в настроении думать". При threshold=3 это некорректно.

**Root cause:** В `_ask_llm_streaming` блок `except (FuturesTimeoutError, TimeoutError)` всегда вызывал `_speak_simple("не в настроении думать")` после проверки `if count >= threshold`. Если threshold не достигнут — код падал дальше на `_speak_simple` в любом случае:

```python
# БЫЛО (неправильно):
if self.provider_error_count >= self.provider_error_threshold:
    if self._try_fallback_provider():
        ...
        return  # выходим только если fallback успешен
# Если threshold не достигнут ИЛИ fallback не помог — падаем сюда:
self._speak_simple("не в настроении думать")  # ← ВСЕГДА!
```

**Фикс:** Переделать на `if/else` — тихий retry при count < threshold:

```python
# ПРАВИЛЬНО:
if self.provider_error_count >= self.provider_error_threshold:
    # ... fallback попытки ...
    self._speak_simple("не в настроении думать")  # только при >= threshold
    self.llm_processing = False
    self.dialogue_in_progress = False
else:
    # count < threshold — тихий retry через 2s
    self.get_logger().warning(f"♻️ Тихий retry через 2s ({count}/{threshold})")
    self.llm_processing = False
    self._timeout_retry_timer = self.create_timer(2.0, self._do_timeout_retry)
```

Новый метод `_do_timeout_retry` (one-shot таймер):
```python
def _do_timeout_retry(self):
    if self._timeout_retry_timer is not None:
        self._timeout_retry_timer.cancel()
        self._timeout_retry_timer = None
    if not self.llm_processing and self.dialogue_in_progress:
        self.get_logger().info("♻️ Выполняю тихий retry запроса к LLM...")
        self.llm_processing = True
        self._ask_llm_streaming()  # conversation_history уже содержит user message
```

Также добавлено отмена `_timeout_retry_timer` при wake word (не запускать retry в середине нового диалога).

**Результат:** 1-2 случайных таймаута DeepSeek → тихий retry через 2s → пользователь получает ответ. "не в настроении" произносится только после 3 провалов подряд.

---

### BUG-18: `pending_queries` зависают навсегда после `interrupt_agent_loop` (commit `afc83d2`)

**Симптом:** Робот перестаёт отвечать после серии сообщений. В логах видно: несколько `👤 User: ... [State: IDLE]` подряд без единого `🤔 Запрос к DeepSeek` или `🔄 Обрабатываю N накопленных запросов`. Остаётся только `🛠️ Получено 22 инструментов из MCP сервера` каждые 10s.

**Признак в логах:**
```
[dialogue_node] 👤 User: "первое сообщение" [State: IDLE]    ← LLM начал
[dialogue_node] 🛑 ОСТАНОВКА: listen_for_response — жду ответа
[dialogue_node] 👤 User: "ответ пользователя" [State: DIALOGUE]  ← продолжение
[dialogue_node] ⏰ Dialogue timeout → IDLE                      ← таймаут, поток всё ещё жив
[dialogue_node] 👤 User: "новое сообщение 1" [State: IDLE]   ← нет LLM!
[dialogue_node] 👤 User: "новое сообщение 2" [State: IDLE]   ← нет LLM!
[dialogue_node] 👤 User: "новое сообщение 3" [State: IDLE]   ← нет LLM!
```

**Root cause:** В `_on_stt_result`, когда `llm_processing=True`:
```python
# БЫЛО (неправильно):
if self.llm_processing:
    self.interrupt_agent_loop = True
    return  # ← accumulation_timer НЕ создаётся!
```
Фоновый поток получает `interrupt_agent_loop=True`, выходит через `break`, `finally` устанавливает `llm_processing=False` — но `accumulation_timer` никогда не был создан, поэтому `_check_and_process_queue` никогда не вызывается. Сообщения из `pending_queries` зависают навсегда.

**Фикс:**
1. В `_on_stt_result` при interrupt: создать `accumulation_timer` чтобы очередь обработалась ПОСЛЕ завершения потока:
```python
if self.llm_processing:
    self.interrupt_agent_loop = True
    if self.accumulation_timer is None:
        self.accumulation_timer = self.create_timer(
            self.dialogue_manager.query_accumulation_timeout,
            self._check_and_process_queue,
        )
    return
```

2. В `_check_and_process_queue` — защита от гонки (таймер сработал раньше чем `finally` в потоке):
```python
def _check_and_process_queue(self):
    if self.accumulation_timer is not None:
        self.accumulation_timer.cancel()
        self.accumulation_timer = None

    # Если поток ещё не завершился — перепланируем
    if self.llm_processing:
        self.accumulation_timer = self.create_timer(0.5, self._check_and_process_queue)
        return
    # ... остальной код
```

**Результат:** После interrupt потока и `llm_processing=False` все накопленные в `pending_queries` сообщения обрабатываются штатно.

---

## Best Practices: канонический агентный цикл

Изучены источники: [ghuntley/how-to-build-a-coding-agent](https://github.com/ghuntley/how-to-build-a-coding-agent) (Go, ~5k ⭐), [Claude Code design analysis](https://jannesklaas.github.io/ai/2025/07/20/claude-code-agent-design.html), [Anthropic context engineering](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents), [Deep Agent Architecture](https://dev.to/apssouza22/a-deep-dive-into-deep-agent-architecture-for-ai-coding-assistants-3c8b).

### Канонический паттерн (ghuntley Go agent)

```go
// OUTER LOOP: ждём пользователя
for {
    userInput = getUserMessage()
    conversation = append(conversation, userMessage)
    message = runInference(ctx, conversation)          // LLM call
    conversation = append(conversation, message)

    // INNER LOOP: пока LLM требует tool calls
    for {
        var toolResults []ToolResult
        var hasToolUse bool
        
        for _, content := range message.Content {
            if content.Type == "tool_use" {
                hasToolUse = true
                result = executeTool(content)    // выполняем КАЖДЫЙ инструмент
                toolResults = append(toolResults, result)
            }
        }
        
        if !hasToolUse { break }                // нет tool_calls → выходим
        
        // Отправляем ВСЕ результаты разом → ОДИН message с множеством tool results
        conversation = append(conversation, NewUserMessage(toolResults...))
        message = runInference(ctx, conversation) // следующая итерация
        conversation = append(conversation, message)
    }
}
```

**Ключевые свойства канонического паттерна:**
- Остановка — естественная: LLM не возвращает `tool_calls` → цикл завершается. Нет явного `stop` инструмента, нет регулярок
- Нет `max_iterations` guard в базовой реализации — достаточно при надёжном LLM. Rob Box добавляет `max_iterations=30` как защитный предохранитель — это правильно
- Обработка ошибки инструмента: даже при `toolError != nil` — результат **добавляется** как `isError=true`, а не прерывает цикл. LLM сам решает что делать с ошибкой
- Нет timeout retry на уровне цикла — в простом Go-агенте `runInference` блокирует, но не зависает (ctx cancel снаружи)

### Что отличает rob_box от канона (потенциальные проблемы)

| Аспект | Канон (ghuntley) | rob_box dialogue_node | Риск |
|--------|-----------------|----------------------|------|
| Структура цикла | Двойной nested `for{}` | `while iteration < 30` (рекурсия в _continue_after_tool_calls) | Рекурсия сложнее прервать |
| Tool errors | `isError=true` → LLM решает | Счётчик `failed_tools_count >= 3` → `break` + речевое сообщение | LLM теряет управление |
| Прерывание | Через `ctx.Cancel()` (Go context) | `interrupt_agent_loop` flag + VAD | Flag проверяется только **между итерациями**, не во время LLM I/O |
| Timeout | Нет (синхронный I/O) | `future.result(timeout=60s)` × 2 | BUG-10: до 120с тишины |
| История | Один `conversation []Message` | `messages` local + `conversation_history` в памяти | Риск рассинхронизации |

### Context Engineering: ключевые выводы (Anthropic)

**Context rot** — деградация качества при росте токенов. По Anthropic: LLM имеет "attention budget" (n² pairwise relations). Каждый новый токен его тратит.

**Рекомендации из статьи:**
1. **Минимальный viable tool set** — если человек не может сказать какой инструмент использовать в конкретной ситуации без раздумий, LLM тем более не скажет. 21 инструмент у rob_box > рекомендованного минимума
2. **Compaction** — при приближении к window limit: суммаризировать историю, начать новый контекст с summary + last N файлов (именно так Claude Code делает)
3. **Tool result clearing** — "один из самых безопасных видов compaction": raw результаты инструментов из глубокой истории уже не нужны → удалять
4. **Just-in-time context** — не загружать всё заранее. Агент сам подтягивает данные инструментами. Rob Box и так делает через `memory_context` — правильно
5. **System prompt: правильная altitude** — не too prescriptive (brittle), не too vague. XML-секции: `<instructions>`, `<tool_guidance>`, `<examples>`

### Claude Code: система напоминаний (System Reminders)

Claude Code решает проблему "забывания" промпта за сотнями шагов через **динамические system reminders**, инжектируемые в user message:

```xml
<system-reminder>
Your todo list has changed. DO NOT mention this explicitly to the user.
Here are the latest contents: [...]
Continue on with the tasks if applicable.
</system-reminder>
```

**Ключевые наблюдения:**
- Reminder инжектируется **статически** (не генерируется LLM) на основе state TODO-листа
- "Do NOT mention this to the user" — reminder для LLM, не часть ответа
- Повторение инструкций в tool results повышает adherence vs. только в system prompt

**Применимость для rob_box:** если диалог выходит за 10+ итераций (roleplay, сложные задачи), LLM может "забыть" что не надо вызывать `memory_context`. System reminder после каждого tool call мог бы усилить это ограничение.

### Deep Agent: Context Store и компрессия знаний

Паттерн из продакшн-систем (Claude Code, Manus, Deep Research):
- Sub-agent работает с **полным** контекстом (все файлы, поиски)
- По завершении возвращает только **distilled contexts** (~1000-2000 токенов), не весь working context
- Orchestrator никогда не видит raw investigation data — только refined artifacts

**Аналогия для rob_box:** `memory_context` уже работает по этому принципу — возвращает compact summary прошлых сессий. Но tool results текущей сессии (например `play_animation` → `{"success": true, "message": "animation_id=123 ..."}`) накапливаются в messages без очистки.

---

## BUG-12: Неограниченный рост local `messages` внутри одного диалога (TASK-043)

**Симптом:** Кто-то просит "расскажи историю" → 20+ итераций → на итерации 25 LLM начинает повторяться, забывает что уже рассказал или говорит противоречивые вещи

**Root cause:** В `_ask_llm_non_streaming` создаётся `messages = self.conversation_history.get_messages()`. Далее в while-loop каждая итерация добавляет:
- 1 assistant message с tool_calls
- N tool result messages (по числу вызванных инструментов)

`conversation_history` ограничена `max_messages=20`, НО **local `messages` list растёт без ограничения** пока идёт один диалог. При 30 итерациях с 2 инструментами: 30 × 3 сообщений = 90 дополнительных сообщений в messages → context rot.

```python
# Как это выглядит в коде (~line 1260):
messages = self.conversation_history.get_messages()  # начало: OK
# ...
while iteration < max_iterations:  # каждая итерация:
    messages.append(assistant_message)   # +1
    messages.append(tool_result_message) # +N (unbounded!)
    request_params["messages"] = messages  # растёт без ограничений
```

**Диагностика:**
```bash
# В логах: смотреть token count по итерациям одного диалога
docker logs voice-assistant 2>&1 | grep "Token usage:" | head -40
# Норма: input растёт ~500-1000 токенов за итерацию
# Признак проблемы: после 15+ итераций input > 20k токенов
```

**Возможный фикс:** Применить скользящее окно к `messages` при каждой итерации, сохраняя system prompt + последние K сообщений. Или компрессия tool results: заменить старые подробные results на краткое summary.

---

## BUG-13: Tool results в messages не очищаются (context pollution) (TASK-044)

**Симптом:** В длинном roleplay сессии (Шариков, 30 итераций) каждый вызов `play_animation`, `play_sound` оставляет в messages raw JSON результат. К итерации 20 history содержит десятки `{"success": true, "animation_id": "bounce_15", "duration_ms": 1200}` которые LLM обязан "видеть" и которые занимают ~50-100 токенов каждый.

**Root cause:** Нет механизма tool result clearing. По Anthropic — это "самый безопасный вид compaction": `play_animation` результат с итерации 5 абсолютно бесполезен на итерации 25.

**Фикс (идея, не реализован):**
```python
# После K итераций — сворачивать старые tool results в placeholder
def _compact_tool_results_in_messages(messages, keep_last=5):
    """Заменяет старые tool result messages на краткий placeholder."""
    tool_messages = [i for i, m in enumerate(messages) if m["role"] == "tool"]
    if len(tool_messages) > keep_last * 2:  # keep_last=5 последних пар
        for idx in tool_messages[:-keep_last * 2]:
            messages[idx]["content"] = "[tool result cleared - no longer relevant]"
    return messages
```

---

## BUG-14: 21 MCP-инструмент — потенциальная перегрузка LLM (TASK-045)

**Симптом:** Неочевидный — LLM иногда выбирает "не тот" инструмент, или вызывает лишние инструменты "на всякий случай" перед финальным ответом.

**Root cause:** По Anthropic — "bloated tool sets lead to ambiguous decision points". 21 инструмент × ~150 токенов = ~3150 токенов только на tool definitions в каждом запросе. Кроме token cost, это создаёт ambiguity: у rob_box есть `play_sound` и `play_animation` и `play_sound_and_animation` — три перекрывающихся инструмента.

**Диагностика:**
```bash
# Посмотреть распределение tool calls
docker logs voice-assistant 2>&1 | grep "Выполнение:" | sort | uniq -c | sort -rn | head -10
# Если один инструмент используется редко но вызывает confusion - кандидат на удаление
```

**Возможное решение:** Разделить инструменты по группам доступности (tool filtering per context), или объединить overlap-инструменты.

---

## BUG-15: `interrupt_agent_loop` срабатывает только на границе итерации (TASK-042)

**Симптом:** VAD обнаруживает новую речь → устанавливает `interrupt_agent_loop = True` → но LLM-запрос в текущей итерации уже идёт (занимает 0.5-5с) → прерывание произойдёт только **после** этого запроса.

**Root cause:** Флаг проверяется в начале `while` цикла:
```python
while iteration < max_iterations:
    if self.interrupt_agent_loop:  # ← только здесь
        break
    # ...
    response = self.client.chat.completions.create(...)  # ← 0.5-60с, прерывание невозможно
```

**В `_continue_after_tool_calls`** — та же проблема: флаг проверяется до `future.result(timeout=60.0)`, но не во время ожидания.

**Связь с BUG-10:** Именно это делает BUG-10 таким тяжёлым: при double timeout (60с + 60с retry) = 120с — прерывание флагом `interrupt_agent_loop = True` ничего не даст пока не истечёт `future.result(timeout=60.0)`.

**Возможное решение:**
```python
# Передавать cancellation token в executor
import threading

_cancel_event = threading.Event()

def _do_recursive_streaming():
    if _cancel_event.is_set():
        return  # early exit
    stream = self.client.chat.completions.create(...)
    # ...

# В VAD callback:
if self.llm_processing:
    self.interrupt_agent_loop = True
    self._cancel_event.set()  # сигналим потоку
```

---

## BUG-16 (потенциальный): Нет system reminders — LLM "забывает" промпт за 15+ итераций (TASK-046)

**Симптом:** В длинных roleplay сессиях или сложных задачах (20+ итераций) LLM может нарушать правила из системного промпта: снова вызывать `memory_context` без запроса (BUG-8 был частично исправлен промптом), добавлять "ладно" и "конечно" которые запрещены, нарушать правила длины ответа.

**Root cause:** Системный промпт читается LLM один раз в начале. При 20+ итерациях с большим контекстом tool results (BUG-12/13) — attention dilution по Anthropic. Инструкции из начала контекста "забываются".

**Паттерн из Claude Code:** Динамические system reminders инжектируются в user message через каждые N шагов:

```python
# Возможный фикс: добавлять reminder в tool result каждые 5 итераций
def _get_tool_result_with_reminder(self, tool_result: str, iteration: int) -> str:
    if iteration % 5 == 0 and iteration > 0:
        reminder = (
            "\n\n<system-reminder>"
            "Напоминание: НЕ вызывай memory_context если пользователь "
            "не спрашивал о прошлом. Отвечай кратко (1-3 предложения)."
            "</system-reminder>"
        )
        return tool_result + reminder
    return tool_result
```

---

## Статус и история коммитов

| Commit | Что исправлено |
|--------|----------------|
| `21de3db` | dmix asound.conf, PlaySoundTool INSTANT, no time.sleep() в sound |
| `28aa193` | BUG-1: ThreadPoolExecutor `shutdown(wait=False)` в 2 местах, interrupt_agent_loop |
| `154b484` | BUG-7: GetCurrentTimeTool класс создан, статичный system_prompt, KV cache |
| `092291d` | BUG-2+3+8: убран preload past_turns, clear() на wake word, memory_context guidance в промпте |
| `43e9e9c` | BUG-4+5+6: reset error counter на wake word, try/except в create(), retry с is_retry флагом |
| `a46175a` | TASK-042 + этот SKILL.md создан |
| `79456db` | BUG-9: GetCurrentTimeTool добавлен в _register_tools() в mcp_server.py |
| `d36616b` | BUG-11: обновление last_interaction_time по итерациям, _listen_response_waiting флаг |
| (текущее)  | BUG-17: тихий retry при первых таймаутах вместо немедленного "не в настроении" |
| `afc83d2`  | BUG-18: pending_queries зависают навсегда после interrupt_agent_loop |
| `2739ba9`  | BUG-19: дедлок в _recreate_llm_client — client.close() vs стейл-потоки |

**Оставшиеся задачи (TASK-042):**
- [x] BUG-11 исправлен: roleplay контекст сохраняется между запросами
- [x] BUG-18 исправлен: pending_queries зависают после interrupt_agent_loop
- [x] BUG-19 исправлен: дедлок client.close() vs стейл-потоки httpcore connection pool
- [ ] Исправить BUG-10: double timeout hang — уменьшить retry timeout или добавить wake word прерывание
- [ ] BUG-15 (TASK-042): cancellation token при interrupt_agent_loop — прерывать LLM I/O немедленно
- [ ] BUG-12 (TASK-043): скользящее окно для local `messages` внутри одного диалога
- [ ] BUG-13 (TASK-044): tool result clearing после K итераций
- [ ] BUG-16 (TASK-046): system reminders каждые 5 итераций при длинных сессиях
- [ ] Проверить Device unavailable [PaErrorCode -9985] — dmix проблема после деплоя
- [ ] Проверить что 10 диалогов подряд работают без деградации
- [ ] Измерить реальный token count (ожидаемо ~8.7k на старт каждого диалога)
- [ ] BUG-14 (TASK-045): аудит 21 MCP-инструмента, найти overlap, объединить или удалить неиспользуемые

---

### BUG-19: Дедлок `_recreate_llm_client` calling `client.close()` (commit `2739ba9`)

**Симптом:** Робот полностью замолкает (~10+ минут), dialogue_node не публикует ни одного лога. `py-spy dump --pid <pid>` показывает все потоки заблокированы на `httpcore/_synchronization.py:268 __enter__` (Semaphore.acquire).

**Диагностика:** `py-spy dump --pid <pid>` без `--native` (на ARM `--native` не поддерживается).

**Root cause:** Три независимые причины из прошлых фиксов создают цепочку:

1. `ThreadPoolExecutor.shutdown(wait=False)` — стейл-потоки `_do_streaming` / `_do_recursive_streaming` продолжают работу
2. Стейл-потоки держат `httpcore connection_pool._pool_lock` пока делают HTTP-запрос к DeepSeek
3. Новый запрос получает TIMEOUT → `_recreate_llm_client()` вызывает `self.client.close()` → внутри `close()` тоже пытается взять `_pool_lock` → **взаимная блокировка навсегда**

**Стеки из py-spy:**
```
Thread ThreadPoolExecutor-0_0:   ← НОВЫЙ поток, хочет пересоздать клиент
    __enter__ (httpcore/_synchronization.py:268)   ← ждёт lock
    close (httpcore/_sync/connection_pool.py:350)
    _recreate_llm_client (dialogue_node.py:426)

Thread ThreadPoolExecutor-3_0:   ← СТЕЙЛ-ПОТОК, держит lock, ждёт сети
    __enter__ (httpcore/_synchronization.py:268)
    close (httpcore/_sync/connection_pool.py:416)   ← внутри with _pool_lock, ждёт connection close
    __stream__ (openai/_streaming.py:102)
    _do_recursive_streaming (dialogue_node.py:2343)

Thread ThreadPoolExecutor-4/5_0:  ← ещё стейл-потоки в handle_request
    __enter__ (httpcore/_synchronization.py:268)
    handle_request (httpcore/_sync/connection_pool.py:218)
    _do_streaming / _do_recursive_streaming
```

**Фикс:** убрать `client.close()` — просто создаём новый клиент, старый подберёт GC:
```python
def _recreate_llm_client(self, log_init: bool = False):
    # НЕ вызываем client.close() — дедлок со стейл-потоками httpcore
    # Старый клиент подберёт GC когда стейл-потоки завершатся.
    http_client = httpx.Client(...)
    self.client = OpenAI(api_key=..., http_client=http_client)
```

**Итог:** Дедлок устранён. Старые сокеты CLOSE_WAIT исчезнут сами через TCP timeout (~4 минуты).

---

## Workflow для агента при новом баге в этой области

1. Получить логи: `docker logs voice-assistant 2>&1 > /tmp/voice.log`
2. Найти паттерн из таблицы выше
3. Посмотреть token usage: `grep "Token usage:" /tmp/voice.log | head -30`
4. Найти таймауты: `grep -E "TIMEOUT|Ошибка [0-9]+" /tmp/voice.log`
5. Посмотреть контекст вокруг бага: `grep -n "STT\|Новый диалог\|итерация" /tmp/voice.log | head -50`
6. После фикса — **обязательно обновить этот SKILL.md** с новой секцией BUG-N
