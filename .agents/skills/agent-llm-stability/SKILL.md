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

**Оставшиеся задачи (TASK-042):**
- [x] BUG-11 исправлен: roleplay контекст сохраняется между запросами
- [ ] Исправить BUG-10: double timeout hang — уменьшить retry timeout или добавить wake word прерывание
- [ ] Проверить Device unavailable [PaErrorCode -9985] — dmix проблема после деплоя
- [ ] Проверить что 10 диалогов подряд работают без деградации
- [ ] Измерить реальный token count (ожидаемо ~8.7k на старт каждого диалога)

---

## Workflow для агента при новом баге в этой области

1. Получить логи: `docker logs voice-assistant 2>&1 > /tmp/voice.log`
2. Найти паттерн из таблицы выше
3. Посмотреть token usage: `grep "Token usage:" /tmp/voice.log | head -30`
4. Найти таймауты: `grep -E "TIMEOUT|Ошибка [0-9]+" /tmp/voice.log`
5. Посмотреть контекст вокруг бага: `grep -n "STT\|Новый диалог\|итерация" /tmp/voice.log | head -50`
6. После фикса — **обязательно обновить этот SKILL.md** с новой секцией BUG-N
