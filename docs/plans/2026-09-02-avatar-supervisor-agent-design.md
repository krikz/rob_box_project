# Avatar Supervisor Agent — каркас «мозга оператора»

| Поле | Значение |
|---|---|
| Статус | Approved (дизайн-фаза, реализация в AV-21) |
| Дата | 2026-09-02 |
| Issue | #1913 (AV-21) |
| Затрагивает | `rob_box_harness` (новый `harnesses/operator.py`), `rob_box_supervisor` (новые топики + `enabled` флаг + voice-mode swap), `rob_box_core.tool_catalog` (источник схем) |
| Родители | ADR-0028 §1.1 (два агента), `docs/plans/2026-08-27-quest-voice-passthrough-design.md` §1.1 (фаза P5), ADR-0021 (discipline — не плодим параллельные реализации), ADR-0013 (incremental — каркас + 2 инструмента за один PR) |

## 1. Цель

Заложить каркас супервизор-агента (в ADR-0028 §1.1 — «мозг оператора»), который
**на равных** обрабатывает голос из Quest и текст из Telegram. Это **не**
большой PR «полный агент»: только каркас, два первых инструмента, и
инфраструктура (топики, JSON-схемы, голосовой гейт, метрики), чтобы
последующие карточки добавляли инструменты инкрементально (по плану
`docs/plans/2026-08-27-quest-voice-passthrough-design.md` §1.1, P5).

## 2. Выбор архитектуры — (a) новый harness, **не** (b) внутренний модуль

Два честных варианта:

- **(a) Новый harness `rob_box_harness/harnesses/operator.py`**. Агент живёт
  как ещё одна реализация `Harness[OperatorState]`, supervisor его
  хостит. Плюсы: переиспользуем порты (LLM/Tools/Memory/Effects/Transport),
  lifecycle (`init/run/teardown`), snapshot/restore, тесты-инфраструктуру
  (FakeLLMProvider, FakeToolProvider, InMemoryStore), реестр harnesses;
  конфиг тот же `HarnessConfig` (YAML + env, тот же секретный pipeline);
  следующие инструменты добавляются одной строкой регистрации.
- **(b) Модуль внутри `rob_box_supervisor/agent/`** с прямым использованием
  `rob_box_llm` + `rob_box_core.tool_catalog`. Плюсы: меньше связей
  (supervisor не зависит от harness). Минусы: дублируем то, что уже
  построено в harness — повторяем lifecycle, snapshot, retry на
  провайдере, секретный pipeline YAML+env, мониторинг. По ADR-0021 R1
  запрещены параллельные реализации одной ответственности; harness
  уже ответственен за «диалог + tool loop + memory + snapshot».

**Выбираем (a)**, и вот почему — аргумент, а не умолчание:

1. **Harness — единственное место, где уже сходятся LLMProvider +
   ToolProvider + MemoryStore + SideEffectBus**. Любой код,
   обрабатывающий пользовательскую команду «от лица оператора», обязан
   иметь ту же пятёрку портов. Дублировать её в supervisor = ADR-0021 R1
   «запрет параллельных реализаций» нарушен.
2. **Идемпотентный lifecycle + snapshot/restore уже решены** для диалога
   и телеграма. Следующий продукт этой линейки (operator) должен ими
   пользоваться, а не изобретать заново.
3. **Конфиг-формат YAML+env уже есть**, и тот же harness-реестр
   подключит operator без правок в supervisor (registry знает «дай мне
   harness `operator` с конфигом X»). Альтернатива (b) вынудила бы
   supervisor знать и про YAML harness, и про какой-то свой формат.
4. **Тесты инфраструктуры уже есть**: FakeLLMProvider, FakeToolProvider,
   InMemoryStore, NoopBus. Это значит, unit-тесты supervisor-агента не
   требуют реального LLM — что и требует задача (acceptance: «Mock-LLM
   обязателен»).
5. **(b) бы выиграл только если supervisor не должен зависеть от
   harness**. Но supervisor **уже** зависит от harness концептуально
   (LockManager/ModeManager рядом с ToolProvider концептуально близки),
   и ADR-0028 §4.6 явно кладёт supervisor и harness в один daemon
   (Vision Pi). Зависимость supervisor→harness не новая — она уже
   есть через `core/locks.py` (LockManager — это и есть порт в стиле
   harness). Делать вид, что можно написать «внутренний модуль» без
   harness = самообман.

## 3. Границы (что в этом PR, что нет)

**В этом PR (AV-21, каркас):**

1. Каркас harness `OperatorHarness` (`Harness[OperatorState]`) с
   системным промптом из отдельного файла
   (`rob_box_harness/prompts/operator_system_prompt.txt`).
2. Два инструмента: `say(text)` и `play_animation(name)`. Схемы берутся
   **только** из `rob_box_core.tool_catalog` — никаких собственных
   объявлений.
3. Валидация при старте: имена инструментов, объявленных в harness,
   совпадают с именами в каталоге; их JSON-Schema — тоже из каталога.
4. ROS-узел `AvatarSupervisor`:
   - подписка на топик `/avatar/command` (`std_msgs/String` JSON);
   - публикация в топик `/avatar/command_result` (`std_msgs/String`
     JSON);
   - параметр `agent_enabled` (default **false**);
   - параметр `system_prompt_file` (default `operator_system_prompt.txt`);
   - при обработке команды — **свап `voice_input_mode`** через
     существующий `_apply_voice_mode` / `_set_dialogue_param` с
     `try/finally` (гарантия «молчит личность робота»);
   - метрики `avatar_agent_commands_total{source,result}`,
     `avatar_agent_tool_calls_total{tool}`,
     `avatar_agent_latency_seconds` (через
     `rob_box_voice/observability/metrics.py`, который умеет
     no-op если `prometheus_client` нет — идиома проекта).
5. Архитектурный документ `docs/architecture/avatar-supervisor-agent.md` —
   фиксирует JSON-схему `/avatar/command` и `/avatar/command_result`.
6. Тесты: 8 кейсов acceptance (см. ниже).

**Вне этого PR (карточки-после):**

- Остальные инструменты (`play_music`, `navigate_to_waypoint`,
  `stop_navigation`, …) — отдельные карточки, перечислены в `tasks.md`.
- Подключение входов (Quest STT, Telegram текст) — это AV-22 (Phase P6
  из плана).
- Активный режим супервизора (`mode=active`) — Phase 2, отдельные
  карточки. Эта карточка держит `agent_enabled` локально (только
  в `AvatarSupervisor`), независимо от monitor/active флага.

## 4. Каркас harness

```python
# src/rob_box_harness/rob_box_harness/harnesses/operator.py

class OperatorState(Mapping[str, Any]):
    enabled: bool           # gate
    last_command_text: str  # для snapshot
    last_summary: str

class OperatorHarness(Harness[OperatorState]):
    name = "operator"

    async def init(self):
        await super().init()
        # 1. Load system prompt from file (not a string in code)
        # 2. Build tool specs from rob_box_core.tool_catalog (NO local redeclaration)
        # 3. Validate tool names are in catalog
        self._tools = self._build_tool_specs_from_catalog(["say", "play_animation"])
        self._executor = FakeToolProvider({...})  # say/play_animation stub handlers

    async def step(self, input_data):
        # Coerce → LLM.complete (with tool_specs) → tool loop → summary
        ...
```

### 4.1. Откуда берутся схемы

```python
from rob_box_core.tool_catalog import get_tool, llm_visible_tools

# say и play_animation должны быть ОБЪЯВЛЕНЫ в rob_box_mcp_tools.
# Если их там нет — harness init() падает с ConfigError ("tool 'say' not in catalog").
def _specs_for(names: tuple[str, ...]) -> list[dict]:
    return [get_tool(n).to_openai_tool() for n in names]
```

Тест-инвариант (см. acceptance): в `operator.py` **нет** ни одного
объявления JSON-Schema (никаких `{"type": "object", "properties": {"text": ...}}`).
Если кто-то попытается добавить — `pytest` это ловит (grep-инвариант).

### 4.2. Системный промпт — отдельный файл

`src/rob_box_harness/rob_box_harness/prompts/operator_system_prompt.txt`:

```
Ты — мозг оператора робота. Ты действуешь от имени оператора, который
управляет роботом удалённо (через Quest-очки или текстом в Telegram).

Ты НЕ личность робота. Личность робота — это отдельный агент
(dialogue_node), который отвечает обычным людям рядом с роботом.
Когда оператор работает, личность молчит.

Твоя задача: получить команду и выполнить её через инструменты. Не
придумывай действия, для которых нет инструмента. Не веди светскую
беседу. Отвечай коротко — факт выполнения (одно-два слова).
```

Загрузка в `init()` через `_load_system_prompt(self.config.system_prompt_file)` —
как у `dialogue_node._load_system_prompt`. Если файла нет или он пустой —
`RuntimeError` (тест ловит).

## 5. ROS-узел супервизора

### 5.1. Топики (фиксируются в `docs/architecture/avatar-supervisor-agent.md`)

| Топик | Тип | Направление | Формат |
|---|---|---|---|
| `/avatar/command` | `std_msgs/String` | клиент → supervisor | JSON `{source, client_id, text, ts_ms}` |
| `/avatar/command_result` | `std_msgs/String` | supervisor → клиент | JSON `{request_id, ok, summary, tool_calls: [...]}` |

### 5.2. Параметры (declare_parameter в `__init__`)

| Параметр | Default | Когда применяется |
|---|---|---|
| `agent_enabled` | `false` | гейт всего агента (см. §5.4) |
| `system_prompt_file` | `operator_system_prompt.txt` | имя файла в `rob_box_harness/prompts/` |
| `agent_during_voice_mode` | `"off"` | какой `voice_input_mode` выставлять на `dialogue_node` во время обработки |

### 5.3. Поток обработки одной команды

```python
def _on_avatar_command(self, msg):
    if not self._agent_enabled:
        # Publish a deterministic "not_handled" result so callers don't hang
        self._publish_result(request_id=None, ok=False,
                             summary="agent_disabled", tool_calls=[])
        return
    payload = self._parse_command(msg.data)
    with self._voice_mode_swap():  # try/finally
        result = await self._harness.run(payload)   # async — TODO: schedule
        self._publish_result(...)
```

Важно: `voice_mode_swap` — **контекст-менеджер** с `try/finally`,
который восстанавливает прежний `voice_input_mode` даже если LLM
бросил исключение. Это гейт «пока оператор работает, личность молчит».

### 5.4. Гейт `agent_enabled = false`

При `false`:
- `/avatar/command` принимается, результат публикуется как
  `{ok: false, summary: "agent_disabled", tool_calls: []}` —
  вызывающий (Telegram bot) сразу понимает, что фича выключена, и не
  зависает в ожидании.
- harness **не** инстанцируется; LLM **не** дёргается; никакого
  `_voice_mode_swap` не происходит.
- Supervisor ведёт себя ровно как сегодня (`monitor`-режим,
  без нового влияния).

Тест: `agent_enabled = false` (default) + опубликованный
`/avatar/command` → ровно одна публикация в `/avatar/command_result`
с `ok=false, summary="agent_disabled"`; LLM **не** вызван (mock-LLM
не зафиксировал вызовов).

### 5.5. Метрики

Через `rob_box_voice.observability.metrics` (`get_metric` + ленивая
регистрация):

| Метрика | Тип | Лейблы |
|---|---|---|
| `avatar_agent_commands_total` | Counter | `{source, result}` (`source` ∈ quest/telegram, `result` ∈ ok/error/disabled) |
| `avatar_agent_tool_calls_total` | Counter | `{tool}` |
| `avatar_agent_latency_seconds` | Histogram | (без лейблов) |

Если `prometheus_client` нет (CI без него) — это no-op через
`HistogramValue`/`Counter`-заглушки в этом модуле (см. там же).

## 6. Acceptance criteria (1:1 с карточкой)

1. `design.md` (этот файл) содержит выбор (a) с аргументом, а не умолчание.
2. `/avatar/command` и `/avatar/command_result` объявлены; JSON-схема
   зафиксирована в `docs/architecture/avatar-supervisor-agent.md`.
3. Параметр `agent_enabled` (default `false`); при `false` нода ведёт
   себя ровно как сейчас — тест публикует в `/avatar/command` и
   проверяет, что LLM **не** вызван.
4. Системный промпт — отдельный файл; тест проверяет, что
   `_load_system_prompt` загружает файл и ругается на пустой/несуществующий.
5. Инструменты `say` и `play_animation` берут схемы из
   `rob_box_core.tool_catalog`; **тест-инвариант**: ни одного
   собственного объявления JSON-Schema в `operator.py` (grep-инвариант
   аналогично `_validate_tools_in_prompt` для master prompt).
6. Команда «скажи привет» → один tool-call `say` → результат в
   `/avatar/command_result` (`summary: "ok"`, `ok: true`,
   `tool_calls: [{name: "say", arguments: {text: "Привет"}}]`).
7. Команда, на которую нет инструмента → `ok: false`, `summary` —
   внятный (например `"no_tool: unknown_action"`), **без** выдумывания
   действия (ADR-0018 в рантайме — если LLM вернул `tool_calls: []`
   при отсутствии инструмента в каталоге, supervisor публикует
   `summary: "no_tool: <action>"` и не пытается выполнить ничего).
8. `voice_input_mode` свапается через `_apply_voice_mode` с `try/finally`,
   даже если LLM бросил исключение — тест на исключение в LLM.verify
   свап восстановился.
9. Метрики экспортируются; тест проверяет инкремент после одной
   успешной команды.
10. `black --check --line-length 120` + `flake8 --max-line-length 120` —
    чисто.

## 7. Тесты

```bash
pytest src/rob_box_supervisor/test -v        # supervisor-узел + handler
pytest src/rob_box_harness/test -v            # harness lifecycle + tools
pytest src/rob_box_core/test -v               # если в operator_test нужны
                                             # примеры tool_catalog entries
```

Все тесты — с Mock-LLM (FakeLLMProvider / DummyLLMProvider).
Реальных вызовов провайдера в тестах **не** должно быть (требование
карточки + AGENTS.md).

## 8. Файлы (новые/правки)

Новые:
- `src/rob_box_harness/rob_box_harness/harnesses/operator.py` — harness.
- `src/rob_box_harness/rob_box_harness/prompts/operator_system_prompt.txt` — системный промпт.
- `src/rob_box_harness/test/test_operator_harness.py` — тесты harness.
- `src/rob_box_supervisor/test/unit/test_avatar_agent.py` — тесты supervisor-агента.
- `docs/architecture/avatar-supervisor-agent.md` — JSON-схемы + поток.
- `docs/plans/2026-09-02-avatar-supervisor-agent-tasks.md` — список карточек-после.

Правки:
- `src/rob_box_harness/rob_box_harness/harnesses/__init__.py` — реэкспорт `OperatorHarness`.
- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` — добавить
  топики, `agent_enabled`, `_on_avatar_command`, `_publish_command_result`,
  `_voice_mode_swap()` контекст-менеджер, метрики.
- `src/rob_box_core/rob_box_core/_tool_catalog_data.py` — добавить `say` и
  `play_animation`, **если** их там нет (проверить в первую очередь).

## 9. Открытые вопросы (не блокеры для AV-21)

- **Q1**: реальные backend-ы для `say` и `play_animation` (TTS-канал и
  LED-канал) придут в карточках-после. В этом PR — заглушки executor,
  которые возвращают `ToolResult(content="ok")`.
- **Q2**: где supervisor получает `OperatorHarness`-инстанс — через
  harness-реестр (идиоматично) или создаёт напрямую (минимум связей).
  В этом PR — напрямую (`OperatorHarness(HarnessConfig(...))` через
  `__init__`), реестр подключим когда появятся 2+ harnesses в
  supervisor-е.
- **Q3**: формат `ts_ms` в `/avatar/command` — int (Unix-epoch ms) или
  str (ISO 8601). В этом PR принимаем оба (graceful парсинг).
