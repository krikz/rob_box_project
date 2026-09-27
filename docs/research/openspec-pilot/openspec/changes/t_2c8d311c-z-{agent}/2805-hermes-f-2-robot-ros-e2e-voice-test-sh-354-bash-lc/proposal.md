## Why

`robot_ros()` в `.github/workflows/scripts/e2e_voice_test.sh:355-357` использует литерал `$*` в `bash -lc '...'`, из-за чего:

1. Подставляются позиционные параметры **всего скрипта**, а не функции (т.к. `$*` в single-quoted литерале остаётся `$*` до локального разворачивания bash на 249).
2. Любой апостроф или `$(...)` в аргументах → syntax error на роботе → `activate_e2e_*_db()` молча уходят в FATAL exit 2.
3. Сценарий едет по боевой `/data/speakers.db` мастерской — ровно то, что закрывали #2750.

PR #2816 был подготовлен, но закрыт без merge (CI был зелёный, но впоследствии процесс был приостановлен). Этот change — re-PR с фиксом по ADR-0129 (вариант B).

## What Changes

- `.github/workflows/scripts/e2e_voice_test.sh:355` — `robot_ros()` переписан с `$*` на `printf %q "$@"` + `eval "$cmd"` (вариант B по ADR-0129).
- `scripts/agent_flow/tests/test_e2e_voice_robot_ros_quoting.sh` — регресс-тест с 4 кейсами (простая команда; апостроф; `$(...)`; смесь кавычек), прогоняется без робота (через `ROBOT_SSH_OVERRIDE`-stub). До фикса — 0/4 PASS, после — 4/4 PASS.
- `docs/research/openspec-pilot/openspec/changes/t_2c8d311c-z-{agent}/2805-.../proposal.md` — этот файл.

## Capabilities

### New Capabilities

Нет новых capabilities — это bugfix существующего.

### Modified Capabilities

- `agent-flow.e2e_voice_test.ros_ssh` — фикс контракта robot_ros().

## Impact

- `.github/workflows/scripts/e2e_voice_test.sh` — изменён ТОЛЬКО body функции `robot_ros()` (строки 355-365). Сигнатура и контракт вызова НЕ изменились: `robot_ros "<команда>"` работает как раньше, но аргументы теперь корректно доезжают до робота.
- `scripts/agent_flow/tests/` — добавлен `test_e2e_voice_robot_ros_quoting.sh` (4 кейса, без зависимостей).

## Source

- Issue: #2805
- Title: `[hermes] F-2: robot_ros() в e2e_voice_test.sh:354 хрупко — $* в bash -lc '...' ломается на любом аргументе с одинарной кавычкой`
- ADR-0129 (принят, вариант B)
- PR #2816 (закрыт без merge — reference)
- Kanban: t_149ebebd
- Created: 2026-09-25 by developer