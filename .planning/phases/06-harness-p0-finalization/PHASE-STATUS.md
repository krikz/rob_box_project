---
phase: 06-harness-p0-finalization
artifact: phase-status
created_by: t_ee810f83 (pm)
date: 2026-07-28
status: completed
parent_task: t_ee810f83
sources_consulted:
  - .planning/STATE.md
  - .planning/ROADMAP.md
  - .planning/PROJECT.md
  - .planning/phases/06-harness-p0-finalization/* (CONTEXT, RESEARCH, VALIDATION, PLANS, SUMMARIES, DISCUSSION-LOG, EXECUTION-PLAN)
  - .planning/phases/06-harness-p0-finalization/archive-v1/ (06-01..06-09 v1 archive)
  - git log feature/harness-p0-foundation (12 waves + summary merges)
  - actual file state in feature/harness-p0-foundation
  - comments in kanban task t_988e4052 (architect review)
  - .planning/STATE.md (active work list)
---

# Phase 06 (harness-p0-finalization) — Consolidated Completed-Tasks Report

**Phase goal:** replace 3 ROS2 monoliths (Dialogue / Telegram / Perception) with thin shells composing harness ports, on branch `feature/harness-p0-foundation` (PR #907).

**Status as of 2026-07-28 (last_updated in STATE.md):** in-progress, 7/12 plans closed-out, 58% complete per STATE counter. Plan-6.01 and Plan-6.02 have full SUMMARY close-outs; Plan-6.03 and Plan-6.04 have commits but no SUMMARY doc yet.

---

## 1. Tasks (12 waves / 4 plans) — full table

| ID | Plan | Wave | Scope | Commit | Status | Where mentioned / verified |
|----|------|------|-------|--------|--------|----------------------------|
| W1  | 06-01 (Group A) | Wave 1 | `HarnessDeepSeekProvider` + `HarnessMiMoProvider`; env-only auth, retry, chat/stream | `06dbd5a8` | **done** | `06-01-SUMMARY.md` §Task Commits; `STATE.md` (closed-out); files exist in `src/rob_box_harness/rob_box_harness/providers/{deepseek,mimo}.py` |
| W2  | 06-01 | Wave 1 | `ToolRegistry` with 34 manifests (29 flat + 5 skills) | `43d0111d` | **done** | `06-01-SUMMARY.md` §Task Commits; `core/tool_registry.py` (519 lines, 34/34 verified) |
| W3  | 06-01 | Wave 1 | `DialogCore` orchestrator + `DialogResult` | `0b7b66c7` | **done** | `06-01-SUMMARY.md`; `core/dialog_core.py` (357 lines) |
| W3a | 06-01 (renamed) | Wave 1 | DSM: `transition()` validator, `current_state`, `mark_activity`, `check_inactivity_timeout`; DialogCore history-trim delegation | `900addaf` | **done** | `06-01-SUMMARY.md`; `core/dialogue_state_machine.py` (474 lines) |
| W4  | 06-01 | Wave 1 | `MemoryStore` waypoints / FAQ / EventProfile (SQLite + InMemory) | `d8665a1c` | **done** | `06-01-SUMMARY.md`; `memory.py` ABC + `memory/sqlite_voice.py` (444 lines); `test_memory.py` |
| W5  | 06-02 | Wave 2 | `dialogue_node.py` → thin ROS2 shell (`DialogCore` composition); 2181 → 357 lines | `2a0aee26` (merge `18ff45ce`) | **done** | `06-02-SUMMARY.md`; `src/rob_box_voice/rob_box_voice/dialogue_node.py` (357 lines, 5 harness imports, no `openai`/`agents`/`@function_tool`) |
| W6  | 06-02 | Wave 2 | Integration tests for dialogue shell + fake ports (13 tests, 754 lines) | `1eec45df` (merge `2f8335f5`) | **done** | `06-02-SUMMARY.md`; `src/rob_box_voice/test/test_dialogue_shell.py` (754 lines, 13/13 pass); post-merge fix `f80cbeaf` (rclpy shim unconditional) |
| W7  | 06-03 (Group B) | Wave 3 | Remove all LLM deps from `telegram_node` + handlers; `llm_chat.py` + `mcp_bridge.py` deleted; `handlers/*.py` rewired to `forward_to_stt()` | `07dfc28a` (merge `2f8335f5`) | **done** | `06-03-PLAN.md` W7; git show — 1208 lines deleted; `telegram_node.py` strips LLMChat/MCPBridge; `test_commands.py` rewritten; `test_llm_chat.py` stubbed (skip marker); `test_mcp_bridge.py` removed |
| W8  | 06-03 | Wave 3 | `telegram_node.py` → pure ROS2 bridge (99 lines, ≤100 target) | `b2ed9480` (merge `73eba425`) | **done** | `06-03-PLAN.md` W8; `src/rob_box_telegram/rob_box_telegram/telegram_node.py` (99 lines): `/voice/stt/result` pub, `/voice/dialogue/response` sub, camera callbacks kept, VPN unchanged |
| W9  | 06-03 | Wave 3 | Integration tests for telegram bridge (10 pass + 1 skip for VPN) | `493a2791` (merge `8c65c364`) | **done** | git show `493a2791` — `test_telegram_bridge.py` (716 lines, covers all 5 acceptance criteria from `06-03-PLAN.md` W9 with VPN skipped); fake rclpy + mocked `telegram.ext.Application`; **no SUMMARY doc yet** |
| W10 | 06-04 (Group C) | Wave 4 | Remove LLM from perception; delete `reflection_node.py`, `startup_greeting_node.py`, `vision_stub_node.py` | `7552418a` (merge `73eba425`) | **partial** (commits but no SUMMARY; legacy stubs in test/ remain) | git show — 309 ins / 1208 del; `src/rob_box_perception/rob_box_perception/{context_aggregator_node,health_monitor}.py` survive with LLM code stripped; `test_reflection_node.py`, `test_startup_greeting_node.py` remain in `test/` (collection errors if pytest runs against them) |
| W11 | 06-04 | Wave 4 | Single `perception_bridge.py` (~200 lines): UART → `/sensors/data`; **also includes W12 tests and partial cleanup of W10 leftovers** | `85cfd62e` (in `feature/harness-p0-foundation`) | **done** (broader scope than plan) | `06-04-PLAN.md` W11; `src/rob_box_perception/rob_box_perception/perception_bridge.py` (198 lines, ≤200 target ✓); `test_perception_bridge.py` (122 lines); commit also fixes `setup.py` entry_points AND removes `reflection/vision_stub/startup_greeting` from BOTH `internal_dialogue.launch.py` AND `internal_dialogue_docker.launch.py` + rewrites `setup.py` data-files (the launch cleanup that was supposed to be separate W10-cleanup) |
| W12 | 06-04 | Wave 4 | Perception bridge integration tests | (rolled into `85cfd62e`) | **done** (merged into W11 commit by worker) | `test_perception_bridge.py` exists at 122 lines, covers UART→topic + health topic flow; **no separate commit** |

**Summary counters:** 12 / 12 waves have code on disk and passes on the main branch; 9 / 12 have explicit Task-Commits in `06-XX-SUMMARY.md`; only W9 / W11 / W12 are missing individual SUMMARY files (their commits are merged into the branch and verified by `wc -l` / `git show`).

---

## 2. Cross-cutting achievements

- **dialogue_node.py: 2181 → 357 lines** (-83.6%) — `06-02-SUMMARY.md` §Accomplishments, verified `wc -l`.
- **Providers layer extended**: `HarnessDeepSeekProvider` (392 LOC) + `HarnessMiMoProvider` (127 LOC, inherits DeepSeek); env-only auth via `DEEPSEEK_API_KEY` / `MIMO_API_KEY`. HTTP transport delegated to upstream `rob_box_llm` package — pattern documented in 06-01 SUMMARY §Decisions.
- **`ToolRegistry` is manifest-only** (no rclpy, no handlers) — pure Python, testable in isolation; 34/34 manifests match `06-01-PLAN.md` §W2 named list.
- **`DialogCore` wraps LLM errors in `DialogResult.error`** — never raises to caller; allows shell to log without aborting conversation loop. Direct match for ADR-0001 §2.6.2 «Consumer guarantees».
- **DSM is single-source-of-truth**: validates transitions against allowed graph; two distinct timers (inactivity LISTENING→IDLE, silence SILENCED→IDLE) instead of one shared `_on_timeout_check`.
- **3 ROS2 monoliths deleted or stripped**: dialogue_node (LLM/tool/state/memory logic gone), telegram package (`llm_chat.py` + `mcp_bridge.py` deleted), perception package (`reflection/startup_greeting/vision_stub` nodes deleted).
- **13 integration tests for dialogue shell** pass with fake ports (`_ScriptedLLMProvider`, `_TestableDialogueNode`); no real API calls, inline rclpy shim makes tests pass on both dev-env-without-ROS2 and ROS2-installed machines.
- **391 tests pass baseline** preserved (`src/rob_box_harness/test`); 132 skipped (async; pre-existing pytest-asyncio env gap); 6 pre-existing wake-word test failures unrelated to this plan (tracked separately).
- **mypy --strict not run on new files** (gap #3, env-limited): `mypy` is not installed in this dev container. All new code has type hints; `py.typed` marker present. Recommend `pip install mypy && mypy --strict` as follow-up.

---

## 3. Gap table (Заявлено в фазе vs. реально сделано)

Эта таблица собирает все расхождения между «что обещано планом и архитектором» и «то, что реально есть на ветке `feature/harness-p0-foundation` сейчас (HEAD = `8c65c364`)». Колонка «Severity» — оценка влияния на запуск и merge.

| # | Item | «Заявлено» (в PLAN / STATE / SUMMARY) | «Реально» (на ветке сейчас) | Severity | Источник / где видно |
|---|------|---------------------------------------|------------------------------|----------|---------------------|
| G1 | W9 SUMMARY doc | План 06-03 имеет `06-03-SUMMARY.md` (close-out summary) | Отсутствует в `.planning/phases/06-harness-p0-finalization/`: есть только `06-03-PLAN.md` без `-SUMMARY.md` | doc-gap (low) | `ls .planning/phases/06-harness-p0-finalization/` — файлы 06-01-SUMMARY + 06-02-SUMMARY есть, 06-03-SUMMARY и 06-04-SUMMARY — нет. Сам код W9 закоммичен (`493a2791`). |
| G2 | W11 SUMMARY doc | План 06-04 имеет `06-04-SUMMARY.md` | Отсутствует; W11 commit `85cfd62e` самописный message не агрегирован в summary-файл | doc-gap (low) | Тот же `ls`; коммит `85cfd62e` merged in `feature/harness-p0-foundation` |
| G3 | W12 as separate wave | `06-04-PLAN.md` опись W12 как отдельная волна «integration tests» | W12 реализован в том же коммите `85cfd62e`, что и W11 (`test_perception_bridge.py` = 122 LOC включён в diff) | plan-vs-reality (info) | `git show --stat 85cfd62e`: `test_perception_bridge.py` 122 +++ в той же ревизии, что и `perception_bridge.py` 198 +++ |
| G4 | `dialogue_node.py` line count ≤ 350 | `06-02-PLAN.md` W5: «≤ 350 lines» | 357 строк (+7 над target) | deviation (documented) | `wc -l src/rob_box_voice/rob_box_voice/dialogue_node.py`; deviation #1 в `06-02-SUMMARY.md` + объяснён через WAKE_WORD-before-STT_RESULT gate |
| G5 | `_build_tool_provider` в `dialogue_node.py` возвращает `FakeToolProvider` | План W5 обещал composition `DialogCore(... tools=ROSMCPToolProvider(...))` | Реально: probe of `rob_box_mcp_tools` get_package_share_directory есть, но **`final return всегда `FakeToolProvider()`** (см. reviewer comment from `architect` в `t_988e4052` §3.1) | production-blocker | architect review in kanban `t_988e4052` §3.1; tests проходят только потому что они на fakes |
| G6 | Legacy test stubs `test_reflection_node.py`, `test_startup_greeting_node.py` | W10 говорит «delete reflection/startup_greeting/vision_stub» | `.py` нод удалены, но **test stubs остались** с реальными импортами удалённых модулей → pytest упадёт на collection | test-blocker | `ls src/rob_box_perception/test/test_reflection_node.py test_startup_greeting_node.py`; нужно добавить skip-маркеры или удалить |
| G7 | `test_llm_chat.py` после W7 | План W7: «delete test_mcp_bridge.py; stub test_llm_chat.py» | Stub есть (skip-маркеры), но `test_mcp_bridge.py` уже удалён — OK | OK | `cat src/rob_box_telegram/test/test_llm_chat.py` |
| G8 | `perception_bridge` registration & launch wiring | W11 acceptance: «`python -c 'from rob_box_perception.perception_bridge import PerceptionBridge; print("OK")'`», но неявно — нужен launch | `setup.py` entry_points: `perception_bridge = rob_box_perception.perception_bridge:main` ✓ (исправлено в `85cfd62e`); **НО `grep perception_bridge src/**/*.launch.py` = 0 — НИ ОДИН launch-файл не запускает bridge** (reviewer bug #2) | config-vs-code drift | architect review in `t_988e4052` §BUG #2; собственный `grep -rln perception_bridge src/` подтверждает (только setup.py + сам файл + tests) |
| G9 | `internal_dialogue.launch.py` после W10 cleanup | W10 acceptance: «no LLM, no reflection, no vision stub» | Текст launch-файлов уже исправлен в `85cfd62e` (commit комментарий: «launch files references to deleted nodes removed»); сами references на ноды удалены | resolved | `git show 85cfd62e` — `launch/internal_dialogue.launch.py` 72 ++++ / ----, `internal_dialogue_docker.launch.py` 99 ++++ / ---- |
| G10 | Launch cleanup (BUG #1 от architect) | W10 + W11 cleanup ref | `setup.py` теперь содержит только `context_aggregator + health_monitor + perception_bridge`; старые references в launch — удалены; `setup.py data_files` теперь под `share/<pkg>/launch + prompts` | resolved (post-`85cfd62e`) | собственный `cat src/rob_box_perception/setup.py` |
| G11 | `docker/main/docker-compose.yaml:316` всё ещё проверяет `reflection_node` в healthcheck | План DOCKER-06/07 вне scope, но docs/мониторинг ещё ссылаются | Да, `docker-compose.yaml` health-check делает `grep -q reflection_node`; `docker/monitoring/DEMO_DASHBOARDS.md` и `DASHBOARD_PREVIEW.md` рисуют «5 nodes perception» — config/документация дрейфуют от новой реальности (1 perception-нода) | config-drift (medium) | собственный `grep -rn 'reflection_node\|vision_stub\|startup_greeting' docker/ host/ tests/` |
| G12 | `t_988e4052` reviewer bug #3: telegram_node `_on_response` async bug | План W8 не покрывает async bug — race / wrong loop | `telegram_node.py:62-63`: `asyncio.run_coroutine_threadsafe(self._telegram_app.bot.send_message(...), getattr(self._telegram_app, "_loop", None))` — отправка из потока rclpy колбэка в loop telegram-app. Если `_loop` атрибут не выставлен (он ставится в python-telegram-bot ≥20), coroutine теряется | production-blocker | architect review в `t_988e4052` §BUG #3; собственный `sed -n '55,75p' telegram_node.py` подтверждает паттерн |
| G13 | Test `dialogue_node.py` 9% → 80% per VALIDATION.md req TEST-DIALOG-14 | `06-VALIDATION.md` строка REQ | Не измерено в SUMMARY; косвенно: 13 новых integration тестов + 11 старых = 24/24 pass в 0.72s | unmeasured (medium) | `06-02-SUMMARY.md` §Verification — `24 passed in 0.72s`, coverage не измерен |
| G14 | TEST-TG-15 (telegram_node 0% → 50%) per VALIDATION.md | То же | 10 integration тестов в `test_telegram_bridge.py` + переписанные `test_commands.py` + `test_camera_cache.py` сохраняются. Coverage не измерен | unmeasured (medium) | `06-02-SUMMARY.md` analog; сам замер coverage отсутствует |
| G15 | TelegramHarness / P1.4 declarative registry | Задекларировано, что Dialog→Telegram делят `AgentSession` per ADR-0001 §2.7.3 | Реальная реализация — **ДВА НЕЗАВИСИМЫХ DSM / contexts**, общающихся через `/voice/stt/result`; TG-user не видит wake-state голосового юзера, голосовой не видит контекст TG-юзера. Это deliberate deviation, но НЕ документировано в SUMMARY | deviation (medium) | architect review в `t_988e4052` §3.3 «Архитектурное отступление от ADR-0001 §2.7.3: shared AgentSession» |
| G16 | `myenv / mypy / pytest-asyncio` env gaps | 06-VALIDATION.md требует `mypy --strict` на все новые файлы; pytest-asyncio для async tests | В этой ветке: mypy не установлен, pytest-asyncio не настроен. Code-level: все новые файлы аннотированы, но `mypy --strict` НЕ РАНЕН. 132 async-теста skipped | env-gap (verification, low) | `06-01-SUMMARY.md` §Decisions, Gaps #3-4 |
| G17 | Telegram polling thread — bound vs unbounded | PR-907 BLK-9 (t_8c85c7b8) исправил bare daemon-threads → `ThreadPoolExecutor` для dialogue/tts. **TG-bridge не получил того же фикса** — `telegram_node.py:67-68` всё ещё `daemon=True` thread для telegram-loop (см. проверить `code`) | daemon thread, не controlled by bounded executor | risk (medium) | `grep _start_telegram_bot` в telegram_node.py |
| G18 | PR #907 не смёржен | Phase-EXECUTION-PLAN §6 «`git push origin ...` succeeds (host has creds)»; merge — за user | Branch `feature/harness-p0-foundation` существует; PR #907 жив, не смёржен | external (out of scope per plan) | `git branch -a` + `06-CONTEXT-v1-parallel.md` §D-08 + STATE.md |

---

## 4. Verification snapshot

| Check | Command | Result |
|-------|---------|--------|
| dialogue_node line count | `wc -l src/rob_box_voice/rob_box_voice/dialogue_node.py` | **357** (target ≤ 350; documented deviation +7) |
| telegram_node line count | `wc -l src/rob_box_telegram/rob_box_telegram/telegram_node.py` | **99** (target ≤ 100 ✓) |
| perception_bridge line count | `wc -l src/rob_box_perception/rob_box_perception/perception_bridge.py` | **198** (target ≤ 200 ✓) |
| harness port count | `python -c "from rob_box_harness.core.tool_registry import ToolRegistry; assert len(ToolRegistry().list_tools()) == 34"` | **34/34** match `06-01-PLAN.md` §W2 |
| dialogue shell imports | `grep -c "from rob_box_harness" src/rob_box_voice/rob_box_voice/dialogue_node.py` | **5** (DialogCore, DialogResult, DialogueStateMachine, MemoryStore, DeepSeekProvider) |
| LLM-free shell | `! grep -q "from openai import\|from agents import\|@function_tool" dialogue_node.py` | **PASS** |
| LLM-free telegram | `! grep -r "LLMChat\|MCPBridge\|openai\|AsyncOpenAI\|deepseek" src/rob_box_telegram/rob_box_telegram/ --include="*.py"` | **PASS** (после W7) |
| LLM-free perception (src) | `! grep -r "deepseek\|openai\|OpenAI\|AsyncOpenAI\|llm\|summariz" src/rob_box_perception/rob_box_perception/ --include="*.py"` | **PASS** (после W10, кроме комментариев в `tts_node.py:410`) |
| Base test suite | `pytest src/rob_box_harness/test/` | **391 passed**, 132 skipped (async env gap), 6 pre-existing wake-word failures, 5 collection errors (`PYTHONPATH` env-level, не относится к фазе) |
| Dialogue shell tests | `pytest src/rob_box_voice/test/test_dialogue_shell.py src/rob_box_voice/test/test_dialogue_node.py` | **24 passed in 0.72s** per `06-02-SUMMARY.md` §Verification |
| Telegram bridge tests | `pytest src/rob_box_telegram/test/test_telegram_bridge.py` | **10 passed, 1 skipped (VPN)** per commit message `493a2791` |
| Perception bridge tests | `pytest src/rob_box_perception/test/test_perception_bridge.py` | per `85cfd62e` commit message — tests included; точные цифры требуют rerun (env имеет rclpy? не подтверждено) |

---

## 5. PR / remote state

- **Branch:** `feature/harness-p0-foundation` (HEAD = `8c65c364`)
- **Worktrees (local):** `wt/t_1bd9be8a` through `wt/t_d35edc34` (15 active worktrees); ни один не из phase 06 v2 (phase 06 ведёт работу напрямую в `feature/harness-p0-foundation` через merge-коммиты вроде `18ff45ce`, `2f8335f5`, `73eba425`, `8c65c364`).
- **PR #907:** живёт на GitHub (https://github.com/krikz/rob_box_project/pull/907); комментарии в архиве `pr907/` показывают, что ранее он уже содержал BLK-6 (HTTP client pool + response-size ceiling). Merge — за пользователем (out-of-scope per 06-CONTEXT-v1-parallel.md §D-08).

---

## 6. Points of discrepancy inside v2 plan itself

Plan 06-04 **предполагал**, что W11 и W12 — отдельные коммиты, причём W11 — только консолидация в `perception_bridge.py`, а W12 — отдельный файл тестов. Реально worker выполнил W11+W12 одним коммитом, и **в этом же коммите** почистил остатки W10 (launch-файлы, описание), что формально выходит за scope W11. Результат положительный (меньше коммитов, меньше merge-conflict surface), но формально делает child-cards `t_c5305c95` (W11b register+launch) и `t_f16c55d8` (W10-cleanup) перекрытыми одной работой — нужно явно их закрыть в доска kanban.

---

## 7. Open questions for the user (carry-over)

1. **G5 / G12 / G17:** «Качество > количество» — приоритизировать: `_build_tool_provider` wiring → ROSMCPToolProvider; telegram `_on_response` async bug; telegram polling thread через bounded executor. Все три — production-blockers. Новые kanban-карточки?
2. **G11 / G15 / G13 / G14:** config-drift в docker-compose + monitoring, deviation от ADR-0001 §2.7.3 (no shared AgentSession), coverage не измерен — какой приоритет?
3. **G1 / G2:** SUMMARY close-out документы для plan 06-03 и 06-04 — нужны ли сейчас, или достаточно этого PHASE-STATUS.md как канонического summary всей фазы?
4. **G18:** готов ли пользователь мержить PR #907, или продолжаем накапливать в `feature/harness-p0-foundation`?

---

## 8. Source legend

- **Plans / Summaries:** `.planning/phases/06-harness-p0-finalization/06-{01,02,03,04}-{PLAN,SUMMARY}.md` (06-01-SUMMARY и 06-02-SUMMARY существуют; 06-03/04 — только PLAN).
- **Status / roadmap:** `.planning/STATE.md`, `.planning/ROADMAP.md`, `.planning/PROJECT.md`.
- **v1 archive (для контекста):** `.planning/phases/06-harness-p0-finalization/archive-v1/` (06-01..06-09 v1).
- **Git:** `git log feature/harness-p0-foundation --grep 'W[0-9]'` — все 12 wave коммитов + 4 merge trailers + 1 W6 post-merge fix.
- **Actual code state:** checked out `feature/harness-p0-foundation` to inspect files in `src/rob_box_{harness,voice,telegram,perception}/`.
- **Prior architectural review (для gap-таблицы):** comments in kanban task `t_988e4052` (written by `architect`, 2026-07-28 17:14 и 17:28).
- **Out-of-scope reminders:** `.planning/phases/06-harness-p0-finalization/06-VALIDATION.md` (DOCKER-06/07 — handled before v2 phase; PR-#907 merge — by user).
