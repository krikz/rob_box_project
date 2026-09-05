# Дизайн: issue #1988 — шаг 4а, avatar_supervisor → ТАРС (AgentCore)

> **Автор:** Copilot (архитектор волны), сессия 2026-09-06.
> **База:** `develop` @ `d8a988dbb` (#1987 смержен; #1986 смержен; #1990 OPEN).
> **Метод:** разведка кода (субагент + чтение) + решения Шифу от 2026-09-06.
> **Честность (ADR-0018):** всё ниже — статический анализ. On-robot
> доказательство (docker logs, DoD «исполнил tool-call») — в e2e-ротацию,
> здесь не выполнимо.

## 1. Решения Шифу (2026-09-06, зафиксированы)

| # | решение |
|---|---|
| 1 | Скоуп = **ядро 4a + журнал operator**. Полное удаление harness-каркаса (`harness.py`, `registry`, `runner`, `harnesses/{dialog,echo,upper,persistent,telegram,_base}`, `transport/*`, их тесты, coverage 85%) — **отдельная карточка**, НЕ здесь. |
| 2 | `agent_enabled` по умолчанию → **true** (как в issue/§4a). |
| 3 | Подписка `/avatar/stt/result` — **вшить сейчас** (дремлющая; топик создаст #1990). |
| 4 | Прод-подписчики `/avatar/command_result`: **telegram_node И quest_node**. |

## 2. Факты кода (что есть сейчас)

- `supervisor_node.py` = 1182 LOC (порезан #1987). Ленивый операторский движок:
  `_build_agent_harness_sync` @825 (lazy import `OperatorHarness` @841-852),
  `_ensure_agent_harness` @860, `_run_harness_sync` @873, `_on_avatar_command` @983,
  `_handle_avatar_command_logic` @1093, `_publish_command_result` @951,
  `agent_enabled` default `False` @274. Топики `/avatar/command` (sub) + `/avatar/command_result` (pub).
- `OperatorHarness` (одношаговый LLM+tool, `FakeToolProvider`+stub, дефолт-`DummyLLMProvider`)
  — **единственный** прод-потребитель базового `Harness`. После замены становится мёртвым.
- Агентский движок: `AgentCore` (`rob_box_harness/core/agent_core.py` @469),
  kw-only `__init__` (llm/tools/memory/dsm обязательны; system_prompt, skill_prompts,
  narrow_tools_to_skill, history_trim_limit, llm_settings, on_prompt).
  `process_input(text, ...)` → `DialogResult` (`spoken_text`, `tools_called`, `error`, `new_state`).
- Прод-шаблон wiring — `dialogue_node.py` @474-498 + `_build_llm` (1631) +
  `_build_tool_provider` (1795) + `_build_memory` (1427) + `_load_skill_prompts` (1293).
- Переиспользуемые примитивы (импорты `dialogue_node.py:44-100`): `rob_box_harness.providers`
  (`build_deepseek_provider`, `LLM_PROVIDER_REGISTRY`, дефолты), `.config.LLMConfig`,
  `rob_box_llm.provider` (`LLMMessage`/`LLMSettings`), `core.agent_core`, `core.dialogue_state_machine`,
  `core.tool_registry.ToolRegistry`, `executors` (`ROSMCPToolProvider`, `adapt_tool_provider`),
  `memory` (`MemoryStore`, `SQLiteVoiceMemory`, `InMemoryStore`), `rob_box_core.tool_catalog`,
  `rob_box_core.prompt_sections`, `rob_box_mcp_tools.llm_adapter.LLMToolCallAdapter` (lazy).
- `LLMToolCallAdapter(node)` сам создаёт pub/sub `/mcp/execute` + `/mcp/result` и
  подписывает общим секретом `RequestAuthenticator.from_env(sender="dialogue_node")`.
  `ROSMCPToolProvider(bridge)` + `update_tools([...manifests...])` + `adapt_tool_provider()`.
- `SQLiteVoiceMemory(db_path)` — БЕЗ namespace; ключ памяти AgentCore — `user_id`.
  Настоящие namespace-ы — шаг 10 (#2000). Для оператора — ОТДЕЛЬНЫЙ db-файл.
- Продьюсеры `/avatar/command`: `dialogue_node` (quest_command @2707) и `telegram_node` (AV-22).
  Подписчиков `/avatar/command_result` в проде НЕТ.
- Промпт оператора сейчас в `rob_box_harness/prompts/operator_system_prompt.txt` (120 строк).
  Целевой путь — `rob_box_supervisor/prompts/operator_system_prompt.txt` (создать).
- Superisor-контейнер стартует `ros2 run rob_box_supervisor supervisor_node -p mode:=active`,
  без yaml-параметров агента; `/data` не смонтирован в supervisor.

## 3. Архитектура изменения

### 3.1 `supervisor_node.py` — замена движка на AgentCore

Ленивая сборка операторского агента (по шаблону `dialogue_node`), при `agent_enabled=true`
и первом входе:

```
_ensure_agent_core():
  1. llm = _build_operator_llm()      # провайдер(ы) из параметров; дефолт deepseek (env DEEPSEEK_API_KEY)
  2. tools = _build_operator_tools()  # LLMToolCallAdapter(self) → ROSMCPToolProvider
                                      #   → update_tools(манифесты ToolRegistry) → adapt_tool_provider()
                                      # backend "fake"/"none" → FakeToolProvider (CI/smoke)
  3. memory = SQLiteVoiceMemory(db_path=operator_db)   # InMemoryStore при сбое
  4. dsm = DialogueStateMachine()
  5. core = AgentCore(llm, tools, memory, dsm,
                     system_prompt=загруженный operator_system_prompt.txt,
                     skill_prompts=operator_skill_prompts (полный каталог + operator.speech/control),
                     narrow_tools_to_skill=False, history_trim_limit=..., llm_settings=...)
```

Каждый вход (`/avatar/command` v1; позже `/avatar/stt/result`) — один `process_input`:

- под `_voice_mode_swap()` (сохраняется: личность молчит, пока ТАРС работает);
- `journal_context` (свежие записи журнала) передаётся в `dynamic_system=`;
- маппинг `DialogResult` → `/avatar/command_result`:
  `ok = error is None`; `summary = spoken_text` (или внятный fail: `llm_error:<Type>` /
  `empty_input` / `no_tool` при отсутствии вызова); `tool_calls = tools_called` (имена).

Параметры ноды (новые, с дефолтами; часть — как в dialogue_node.yaml):
`llm_providers` ("deepseek"), `<name>.base_url/model/api_key/timeout_s`, `temperature`,
`max_tokens`, `llm_streaming` (false), `history_max_turns`, `tool_provider` ("ros_mcp"),
`sqlite_db_path` (operator db), `journal_path`, `skills_enabled` (true).
`agent_enabled` default → **true**. `system_prompt_file` — путь в `rob_box_supervisor/prompts/`.

Тест-силка `_handle_avatar_command_logic` сохраняется (через `agent_factory`/подмену core).

### 3.2 Промпт и срезы

- Создать `src/rob_box_supervisor/prompts/operator_system_prompt.txt` — идентичность ТАРС
  (оператор-супервизор робота), правила: только реальные действия, говорить «голосом робота»
  через `say`, сводка в `/avatar/command_result`. Контент — преемник текущего
  harness-промпта + требования целевой архитектуры §6 (никаких выдуманных действий).
- Срезы: `skill_prompts` оператора = **полный каталог личности** (фрагменты
  `rob_box_voice/prompts/skills/*.txt` — загрузка через share-директорию) **плюс**
  фрагменты `operator.speech` (say/set_voice/preview_voice: «заставить робота говорить
  вслух») и `operator.control` (управление паузой/floor — появятся с шагами 06/04б;
  пока текст-инструкция). `narrow_tools_to_skill=False` — полный каталог (сужение и
  проверка на транспорте = шаг 08, #1998).
- Тест-инвариант «нет локальных JSON-schema» (как `test_operator_no_local_schema`)
  переносится на supervisor-промпт/код.

### 3.3 Журнал ТАРС (namespace operator)

Интерпретация §5.2/§5.4: память AgentCore-оператора = `SQLiteVoiceMemory(operator_db)`
(факты, отдельно от личности), а **журнал — отдельный лог изменений**, пишет оболочка
(supervisor) и инжектит в `dynamic_system`.

- Новый чистый модуль журнала в `rob_box_harness` (без ROS, тестируемый):
  запись = `{ts_ms, action, outcome, count, last_ts_ms, detail}`;
  **схлопывание повторов** (§5.4): при вставке той же `action` в окне 1 ч →
  `count += 1`, обновить `last_ts_ms` (одна запись «перезапускал voice-assistant N раз»).
- Персист: JSONL-файл (или таблица sqlite) по `journal_path`; загрузка при старте.
- DoD-тест: две записи «перезапустил voice-assistant» за час → одна запись с `count=2`.

### 3.4 Подписчики `/avatar/command_result`

- **telegram_node**: subscribe `AVATAR_COMMAND_RESULT_TOPIC` (QoS `_RE`, группа `g`).
  `decode_command_result(msg.data)`; роут `request_id="telegram:<chat_id>:<ts_ms>"` →
  отправить `summary` оператору в тот чат через существующий `_response_queue`/
  `_chat_echo_worker`. Несводимый (uuid4 → malformed) — пропустить/логировать.
- **quest_node**: subscribe `AVATAR_COMMAND_RESULT_TOPIC` → `ws_server.broadcast_json_event`
  `{"type":"avatar_command_result", request_id, ok, summary, tool_calls, ts_ms}`
  (зеркало `_send_alert_event`). UX-ветка в `webxr_client` — поверхность шага 05b (#1993),
  здесь не трогаем клиент.

### 3.5 Дремлющая подписка `/avatar/stt/result`

Subscribe в supervisor; обработчик = та же воронка, что `/avatar/command` (payload v1:
source/client_id/text/ts_ms). До #1990 топик пуст — подписка инертна, безопасна.

## 4. Что удаляется / перестаёт использоваться

- `supervisor_node.py`: вся ветка `OperatorHarness` (импорт @841, init/build/run,
  `_run_harness_sync`), `agent_enabled` default false → true.
- Предложение (подтвердить): удалить `rob_box_harness/harnesses/operator.py` +
  `prompts/operator_system_prompt.txt` (harness) + `test/test_operator_harness.py` —
  последний прод-потребитель `OperatorHarness` исчез. Общий harness-каркас
  (`harness.py`, остальные harnesses, registry, runner, transport) — отдельная карточка.
- НЕ трогаем: `dialogue_node.py` (личность байт-в-байт), арбитр, остальной каркас.

## 5. DoD-чеклист и честный статус

| DoD | статус здесь | где доказывается |
|---|---|---|
| `/avatar/command` (v1) → агентский цикл; ответ в `/avatar/command_result` имеет подписчика в `src/` | сделано (telegram_node + quest_node подпишутся) | grep по `src/` |
| ТАРС исполнил tool-call через реальный `ROSMCPToolProvider` | код-путь есть; исполнение — на роботе | e2e, `docker logs` (ротация) |
| `FakeToolProvider` в src/ → только тесты | частично: путь supervisor чист; остаточные прод-хиты в `harness.py`/`dialogue_node` (fake-backend) умирают с каркасной карточкой | grep |
| `DummyLLMProvider` в rob_box_supervisor → пусто | уже пусто | grep |
| журнал: 2 записи за час → одна со счётчиком | тест | pytest |

## 6. Верификация (без робота)

- pytest по затронутым пакетам: `rob_box_supervisor/test/unit/test_avatar_agent.py`,
  `test_supervisor_node.py`, telegram `test_av22_handlers.py`/новые, journal-тесты,
  rob_box_quest unit.
- Линт: flake8/black/isort по изменённым файлам; CC-budget (ADR-0021) не превышен.
- DoD-грепы выше.
- On-robot e2e (docker logs, tool-call исполнение) — в e2e-ротацию, НЕ закрывать issue без неё.

## 7. Открытые пункты для Шифу

1. Удалять ли `operator.py`+его тесты в ЭТОЙ карточке (последний потребитель исчезает) —
   или оставить до каркасной карточки (риск дубля промпта).
2. Параметры LLM: дефолт single-provider `deepseek` из env — ок? (полный health-fallback
   chain — как у dialogue_node — добавлять в этой карточке или позже?)
3. Путь журнала/БД оператора по умолчанию и монтирование `/data` в supervisor-контейнер.

## 8. Итог реализации (2026-09-06) — решения и отклонения

Подтверждено Шифу («погнали»): удалять `operator.py`; single-provider deepseek;
монтировать `/data`. Реализовано на бранче
`1988-operator-agent-04a-avatar-supervisor-tars`:

- supervisor_node.py: `OperatorHarness` → `AgentCore` (+dsm+journal); agent_enabled default true;
  подписка `/avatar/stt/result` (дремлющая); новые параметры (llm_providers/temperature/max_tokens/
  llm_streaming/history_max_turns/tool_provider/operator_db_path/journal_path). Ветка
  `agent_unavailable` вместо `harness_unavailable`; `_handle_avatar_command_logic` (мёртвый seam) удалён.
- `src/rob_box_supervisor/prompts/operator_system_prompt.txt` + `prompts/skills/operator.{speech,control}.txt`;
  `setup.py` ставит их в share.
- **Отклонение от §3.3:** журнал `operator_journal.py` лежит в **`rob_box_supervisor`**
  (домен оператора), а не в `rob_box_harness` — переживёт чистку harness-каркаса и не
  тащит чистый модуль в пакет под удаление. Логика схлопывания та же (DoD-тест).
- rob_box_harness: удалены `harnesses/operator.py`, `prompts/operator_system_prompt.txt`,
  `test_operator_harness.py`; вычищен `harnesses/__init__.py`. Каркас (harness.py и пр.) — отдельная карточка.
- Подписчики `/avatar/command_result`: telegram_node (роут по request_id → чат) и quest_node
  (broadcast_json_event `avatar_command_result`; UX-ветка клиента = 05b).
- docker/vision compose: supervisor монтирует `/data`.

DoD-честность: «FakeToolProvider → только тесты» выполнен частично (остались prod-хиты в
`harness.py`/`tools.py`/`dialogue_node` fake-backend — каркасная карточка и карточка
dialogue_node). Tool-call через реальный ROSMCPToolProvider и схлопывание в логе docker —
требуют on-robot e2e (ротация), здесь не выполнимы.

