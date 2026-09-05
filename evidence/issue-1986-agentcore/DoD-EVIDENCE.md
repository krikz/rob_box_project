# DoD Evidence — issue #1986 (AgentCore: обобщение DialogCore)

Бранч: `1986-operator-agent-03-agentcore-dialogcore` (base `develop` @ `2ce3ad4eb`).
Метод: full rename `DialogCore` → `AgentCore` (`core/dialog_core.py` → `core/agent_core.py`),
полное сужение интерфейса 11→7 сейчас, минимальный fold (DoD only). Решение Шифу от 2026-09-05.

## DoD 1 — pytest личности зелёный + diff e2e байт-в-байт

- Прогон personality-пути (8 файлов, только fake-LLM, без сети):
  `pytest_personality_v.log` → **192 passed**.
- Diff e2e «ответ личности на фиксированный стимул до/после»:
  - до: `personality_pre.json` (класс `DialogCore`)
  - после: `personality_post.json` (класс `AgentCore`)
  - сценарии S1 (личность, реальный `master_prompt_compact.txt` + dynamic/speaker ctx),
    S2 (срез + активный скилл, реальные фрагменты из `prompts/skills/`),
    S3 (глубокая сессия, 20 ходов в окне + скилл).
  - `git diff --no-index pre post` → **различаются ТОЛЬКО 2 строки метаданных**
    (`module`/`class`), все `messages`/`result`/`state` идентичны побайтово.
  - Воспроизводимо: `scripts/verification/capture_agentcore_personality.py`.

## DoD 2 — у AgentCore 7 публичных методов, нет is_wake_word/handle_wake_word/handle_silence/check_timeout

- `src/rob_box_harness/test/test_agent_core.py:700` `test_agent_core_has_seven_method_interface`
  — assert: public surface ровно {process_input, discard_last_reply, clear_history,
  active_skill, set_active_skill, skill_load_counters, known_skills} и ∩ {4 удалённых} = ∅.
- Из ядра удалены `is_wake_word`/`handle_wake_word`/`handle_silence`/`check_timeout`
  (были `dialog_core.py:1796-1858`), параметр `inactivity_timeout` из конструктора.
  Таймаут переехал в оболочку: `dialogue_node.py` `_on_inactivity_check` теперь сам гонит
  `self._dsm.check_inactivity_timeout(self._dialogue_timeout_s)`.
- 6 фасадных тестов удалены, эквивалент перенесён на DSM (покрытие DSM живёт в
  `test_dialogue_state_machine.py`).

## DoD 3 — две конфигурации AgentCore дают разное поведение на одном движке

- `src/rob_box_harness/test/test_agent_core.py:733` `test_two_agent_configs_one_engine`
  — «личность» (system_prompt РОББОКС + срез `player`) vs «оператор» (system_prompt ТАРС +
  срез `operator.speech`) на одном классе/стимуле → разный состав промпта
  (системный промпт на [0], срез последним system-сообщением), разные `known_skills`.

## Полный harness-прогон (соседние пакеты не тронуты по логике)

`pytest src/rob_box_harness/test tests/unit/harness` (без coverage): **966 passed, 1 skipped, 5 failed**.
5 failed = MiniMax provider network-тесты (реальные HTTP к `x.invalid`, нет ключа/мока) —
не связаны с ренеймом, падают и на чистом develop (до изменений зафиксировано в baseline).
`tests/unit/harness/test_tool_provider_contract.py` — collection error (импортирует
несуществующий `MCPBridgeExecutor`), файл моим изменением не тронут (pre-existing).

## Pre-existing баги, исправленные попутно (в переименованном файле)

- `test_agent_core_with_acceptance.py` два теста были красными на develop baseline:
  `result` не был связан в области видимости (`NameError`); `len(llm.complete_calls)` где
  `complete_calls` — int (`TypeError`). Исправлено (захват `result`, `_FakeLLM` пишет `calls`).
  Baseline до правок: 194 passed / 2 failed; после ренейма+фикса: 192 passed (тесты
  фасадов удалены, добавлены DoD-тесты).
