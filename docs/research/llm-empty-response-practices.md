# Пустота ответа LLM: finish_reason=None и практики харнессов (issue #1253)

**Дата:** 14.08.2026
**Контекст:** deepseek-v4-flash возвращает пустой ответ `finish_reason=None tools=[] raw=''`;
робот говорит «Принял.» и НЕ выполняет команду. Фикс #1220 покрыл `done`/`stop`,
но `finish_reason=None` оставался не покрыт.

---

## 1. Откуда берётся finish_reason=None (root cause)

### 1.1. Стрим DeepSeek может закончиться БЕЗ терминального чанка

`DialogCore._stream_response` собирает `finish_reason` ТОЛЬКО из `chunk.finish_reason`
(см. `dialog_core.py::_stream_response`):

```python
finish_reason: str | None = None
async for chunk in self._llm.stream(messages, tools=tools):
    ...
    if chunk.finish_reason:
        finish_reason = chunk.finish_reason
```

Если провайдер закрывает SSE-стрим без финального `finish_reason`-чанка
(обрыв соединения, пустой стрим, провайдер-сайд cut), агрегатор возвращает
`LLMResponse(finish_reason=None, content='', tool_calls=())`.

### 1.2. Tool-call дельты теряются при обрыве

Upstream `rob_box_llm/providers/deepseek.py::stream()` агрегирует tool-call
дельты в `pending`, но эмитит их ТОЛЬКО внутри `if finish:`:

```python
if finish:
    for idx in sorted(pending): ...   # tool_calls эмитятся только здесь
    yield LLMChunk(content_delta="", finish_reason=finish)
    return
```

Если стрим умирает до терминального чанка — частично собранные tool_calls
молча отбрасываются → наружу уходит `tools=() finish_reason=None content=''`.

### 1.3. Диагностика «пустоты» до фикса #1220

- 19 пустых ответов за 50 мин (прогон 14.08, до фикса #1231).
- Фикс #1220 (16ca6e35): corrective retry на пустой / bare-'done' ответ
  (ключ — пустой content + нет tool_calls). `finish_reason` при этом НЕ
  инспектировался.
- Фикс #1231 (c6a370dd): DJ-auto turn не доходил до LLM (preclassified_event) —
  часть «пустых» ответов была НЕ пустотой LLM, а тем, что LLM вообще не
  вызывалась.

### 1.4. Что осталось не покрыто после #1220/#1231

1. `finish_reason=None` с НЕпустым content (обрыв на полуслове) — сейчас
   трактуется как валидный ответ, corrective retry не срабатывает.
2. `finish_reason="insufficient_system_resource"` — DeepSeek документирует
   это значение (HTTP 200, генерация прервана из-за ресурсов провайдера).
   Не ретраится.
3. `finish_reason="length"` с пустым/коротким content — max_tokens достигнут
   до выдачи ответа. Не ретраится.
4. `finish_reason="content_filter"` — контент отфильтрован (пустой ответ).
   Не ретраится.
5. Болтовня при ошибке инструмента: инструмент вернул `is_error=True`,
   LLM ответила только словами без повторного вызова → робот озвучивает
   «дан», «бит не получился» вместо системного перехода.

---

## 2. Практики других систем (ресёрч 14.08)

### 2.1. DeepSeek API

- Документированные `finish_reason`: `stop`, `length`, `content_filter`,
  `tool_calls`, **`insufficient_system_resource`** (HTTP 200! генерация
  прервана из-за нехватки ресурсов провайдера).
  Источник: https://api-docs.deepseek.com/api/create-chat-completion/
- Рекомендация DeepSeek: проверять `finish_reason` ПЕРЕД парсингом ответа;
  `insufficient_system_resource` = прерванная генерация → retry.
- `enable_thinking` передаётся через `extra_body` (у нас уже сделано,
  см. `_build_kwargs` в upstream deepseek.py).
- Известный баг V4-Pro: tool-calls иногда приходят как raw text в `content`
  (deepseek-ai/DeepSeek-V3#1244) — харнесс должен уметь распознавать
  «content-shaped tool call».

### 2.2. vLLM

- **Bug #45736 (2026-06)**: vLLM возвращает HTTP 200 с `content: null`,
  `finish_reason="stop"`, `completion_tokens > 0`, молча отбрасывая
  сгенерированные токены. Урока: НЕЛЬЗЯ доверять только `finish_reason` —
  нужно проверять content + usage.
- Параметр `min_tokens`: запрещает EOS до выдачи N токенов — прямой
  аналог практики «min_length против пустого ответа».

### 2.3. llama.cpp / llama-cpp-python / LM Studio

- Пустой `stop` в списке stop-строк → пустой chat completion
  (abetlen/llama-cpp-python#1195).
- `n_predict=-2` может дать `content:"" finish_reason:"length"` (llama.cpp#12264).
- Практики: `min_keep` (минимум вариантов от семплера), контроль `stop`,
  проверка `predicted_n` (timings) — 0 предсказанных токенов = пустой ответ.

### 2.4. Агентные фреймворки

- **LangChain / LangGraph**: пустой AIMessage — известная проблема; recovery
  требует явного retry-узла (forum: «Return Node for Recovering from Empty
  AIMessage Outputs»). Tool errors → structured `tool_unavailable` сигнал +
  fallback, а не «пусть модель гадает»; deterministic degradation после
  исчерпания retry.
- **OpenAI Agents SDK**: fallback-хендлер для пустых structured responses
  (validate → fallback); replay-safety boundary для tool retries.
- **agno (#6910)**: truncated OpenAI responses молча превращаются в пустые
  assistant-сообщения → агент зацикливается/висит. Классифицировать
  truncation как retryable.

### 2.5. Сводка практик

| Практика | Кто использует | Наш статус |
|---|---|---|
| Retry на пустой/маркерный ответ (1 раз) | LangGraph, agno, OpenAI Agents | ✅ #1220 |
| Retry на `finish_reason=None` (обрыв стрима) | vLLM уроки, DeepSeek docs | ✅ этот PR |
| Retry на `insufficient_system_resource` | DeepSeek docs | ✅ этот PR |
| Проверка `length`/`content_filter` с пустым content | OpenAI-compatible общее | ✅ этот PR |
| `min_tokens` / `min_length` против EOS-до-выдачи | vLLM, llama.cpp | ⚠️ не применимо (API-провайдер) |
| Не доверять finish_reason, проверять content+usage | vLLM #45736 | ✅ `_is_silent_response` |
| Structured error + fallback для tool | LangChain | ✅ tool-errors идут в LLM как tool-сообщение |
| Фильтр болтовни после tool-error | LangChain (deterministic degradation) | ✅ этот PR |

---

## 3. Реализация в этом PR

### 3.1. dialog_core.py

1. `_is_silent_response`: дополнительно считать silent ответы с
   `finish_reason in ("insufficient_system_resource", "content_filter", "length")`
   даже при непустом content (прерванная/отфильтрованная генерация —
   DeepSeek документирует `insufficient_system_resource` как HTTP 200 с
   прерванной генерацией). Пустой content / done-маркер уже были silent.
   `finish_reason=None` с пустым content уже покрыт (#1231).
2. `_run_with_tools`: corrective retry теперь срабатывает и на
   `finish_reason=insufficient_system_resource/content_filter/length` с
   непустым content (обрыв на полуслове).
3. Фильтр болтовни при ошибке инструмента: если в ходе turn инструмент
   вернул `is_error=True`, а финальный ответ LLM — только маркер/пусто
   без повторного tool-вызова и без speak_text — вернуть пустой spoken
   (системный переход), чтобы dialogue_node НЕ озвучивал «дан» / «бит не
   получился».

### 3.2. dialogue_node.py

- Empty-response fallback («Принял.») не срабатывает, когда в turn была
  ошибка инструмента (spoken пуст, tools_called непуст) — робот молча
  переходит к следующему раунду, без болтовни.

---

## 4. Acceptance

- [ ] Пустых ответов 0 в прогоне 40 кейсов (e2e)
- [ ] При ошибке инструмента — системный переход, без «принял дан бит не получился»
- [ ] Документация практик (этот документ)
