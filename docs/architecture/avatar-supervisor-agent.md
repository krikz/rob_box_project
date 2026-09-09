# Avatar Supervisor Agent — wire-протокол (AV-21)

**Назначение.** Фиксирует контракт топиков `/avatar/command` и
`/avatar/command_result`, через которые супервизор-агент
(`AvatarSupervisor`) общается с входными клиентами (Quest STT,
Telegram-текст, будущие web/admin).

**Версия контракта.** `v1` (2026-09-02). Обратная совместимость по
JSON-полям: добавление новых полей — минор, удаление/переименование —
мажор.

**Связанные:**
- `docs/plans/2026-09-02-avatar-supervisor-agent-design.md` — дизайн и
  acceptance.
- `docs/adr/0028-avatar-supervisor.md` §4.3 — внешний API супервизора.
- `docs/plans/2026-08-27-quest-voice-passthrough-design.md` §1.1 — два
  агента, кто и зачем шлёт команды.

## 1. Тип носителя

Оба топика — `std_msgs/msg/String`, payload — UTF-8 JSON (см.
`AvatarSupervisor._publish_avatar_state`, тот же паттерн для JSON в
`std_msgs.String` использован в ноде). Это решение Phase 1 — не
плодим IDL-пакеты до AV-5.

## 2. `/avatar/command` — клиент → супервизор

### 2.1. Схема запроса (v1)

```json
{
  "source": "quest",
  "client_id": "quest-abc123",
  "text": "скажи привет",
  "ts_ms": 1788381040123
}
```

| Поле | Тип | Обязательно | Описание |
|---|---|---|---|
| `source` | `"quest" \| "telegram"` | да | откуда команда. В будущем — `"web"`, `"admin"`. |
| `client_id` | `string` | да | стабильный ID клиента (UUID, Telegram user_id, …). Используется для метрик и (в будущем) авторизации. |
| `text` | `string` | да | команда оператора на естественном языке. После wake-word strip. Не пустая. |
| `ts_ms` | `int \| string` | да | время отправки. `int` — Unix epoch в миллисекундах. `string` — ISO 8601 (`2026-09-02T22:30:40.123Z`). Принимаем оба. |

### 2.2. Поведение супервизора при приёме

1. Парсинг JSON. Невалидный JSON → WARN-лог + публикация в
   `/avatar/command_result` с `ok=false, summary="malformed_input"`.
2. **Гейт `agent_enabled`** (параметр супервизора, default `false`):
   - `false` → немедленная публикация результата
     `{ok: false, summary: "agent_disabled", tool_calls: []}`. LLM **не**
     вызван.
   - `true` → обработка через `OperatorHarness` (§3).
3. Свап `voice_input_mode` через `_voice_mode_swap()` (try/finally).
4. Вызов harness (`await harness.run(input_data)`).
5. Публикация результата в `/avatar/command_result` (§3).

### 2.3. Идемпотентность

Супервизор **не** дедуплицирует по `ts_ms` или `text`. Каждый
входящий `/avatar/command` обрабатывается один раз. Если клиент хочет
идемпотентность — это его ответственность (повтор команды не наш кейс).

## 3. `/avatar/command_result` — супервизор → клиент

### 3.1. Схема ответа (v1)

```json
{
  "request_id": "uuid-...",
  "ok": true,
  "summary": "ok",
  "tool_calls": [
    {"name": "say", "arguments": {"text": "Привет"}, "result": "ok"}
  ],
  "latency_ms": 1240
}
```

| Поле | Тип | Обязательно | Описание |
|---|---|---|---|
| `request_id` | `string` | да | UUID, прокинутый из входа (или сгенерированный, если клиент не передал). |
| `ok` | `bool` | да | `true` если команда выполнена (хотя бы один tool-call успешен); `false` если нет инструмента, ошибка LLM, `agent_disabled`, `malformed_input`. |
| `summary` | `string` | да | короткая строка-итог. Возможные значения: `"ok"`, `"agent_disabled"`, `"malformed_input"`, `"no_tool: <name>"`, `"llm_error: <type>"`, `"timeout"`. |
| `tool_calls` | `array` | да | список вызовов. Пустой массив `[]` если `ok=false` ИЛИ LLM не сделал ни одного вызова. |
| `latency_ms` | `int` | нет | сколько заняло end-to-end. Клиент может не использовать. |

### 3.2. Поля `tool_calls[i]`

| Поле | Тип | Описание |
|---|---|---|
| `name` | `string` | имя инструмента (`"say"`, `"play_animation"`, …). |
| `arguments` | `object` | JSON-объект аргументов (как их передал LLM). |
| `result` | `string \| null` | `string` — результат executor-а; `null` если executor не вернул (или упал). |

### 3.3. Когда `ok=false`

- `agent_disabled` — фича выключена.
- `malformed_input` — JSON не парсится, нет обязательных полей.
- `no_tool: <name>` — LLM попросил инструмент, которого **нет** в
  `rob_box_core.tool_catalog`. Это **жёсткий** случай (ADR-0018 в
  рантайме): supervisor **не** пытается выполнить действие, не
  выдумывает результат — публикует как есть и возвращает.
- `llm_error: <type>` — исключение из LLMProvider. Tool loop
  восстановлен, `tool_calls` пустой, `summary` содержит имя исключения.
- `timeout` — превышен `agent_command_timeout_s` (Phase 2; в этом
  PR без таймаута).

## 4. Семантика «личность молчит» (ADR-0028 S5)

Пока супервизор-агент обрабатывает команду, `dialogue_node._voice_input_mode`
выставляется в `agent_during_voice_mode` (default `"off"` —
W3-1, §3.5 спека: «диалог off, полное управление оператора»).
Свап через `_apply_voice_mode` + `_set_dialogue_param`; **try/finally**
гарантирует восстановление предыдущего значения даже если LLM
бросил исключение. См. `_voice_mode_swap()` в
`AvatarSupervisor`.

## 5. Пример end-to-end

```
[Telegram bot]                          [supervisor]              [dialogue_node]
    │                                        │                          │
    │ -- /avatar/command --                  │                          │
    │ {"source":"telegram","client_id":"u1", │                          │
    │  "text":"скажи привет","ts_ms":1234}   │                          │
    │ ─────────────────────────────────────►  │                          │
    │                                        │ -- voice_input_mode=off   │
    │                                        │ ───────────────────────► │
    │                                        │     (LLM completes       │
    │                                        │      tool_calls=[say])   │
    │                                        │ -- executor(say)         │
    │                                        │   → TTS                  │
    │                                        │ -- voice_input_mode=prev │
    │                                        │ ◄──────────────────────  │
    │ -- /avatar/command_result --           │                          │
    │ {"request_id":"r1","ok":true,           │                          │
    │  "summary":"ok","tool_calls":[...]}    │                          │
    │ ◄─────────────────────────────────────  │                          │
```

## 6. Совместимость

- Любые добавляемые поля — **минор** (клиенты должны игнорировать
  неизвестные ключи).
- Удаление/переименование — **мажор**, требует новой версии (`v2`).
- Wire-контракт Phase 1 — JSON в `std_msgs.String`. Phase 2
  (AV-5 / ADR-0028 §4.3) переведёт на кастомный IDL-пакет, тогда же
  будет версия `v2`.

## 7. Реализация

- Декларация топиков: `AvatarSupervisor.__init__` —
  `create_subscription` / `create_publisher` с `RosString`.
- Парсинг/сериализация: `_parse_command(data)`, `_serialize_result(...)`.
- Тесты: `test_avatar_agent.py::test_command_publishes_result`,
  `...::test_malformed_input_returns_ok_false`,
  `...::test_agent_disabled_skips_llm` и т.д.
