---
phase: 06-harness-p0-finalization
artifact: execution-plan
created_by: t_7cffa9de (analyst)
date: 2026-07-28
status: decomposing-into-children
---

# Phase 6 v2 — Execution Plan & Task Graph (analyst consolidation)

## 1. Scope summary (re-derived from 06-CONTEXT.md v2)

Phase 6 finalizes branch `feature/harness-p0-foundation` by **REPLACING** three
ROS2 monoliths with thin shells that compose harness ports:

| Group | Node replaced | Current LOC | Target LOC | What stays |
|-------|---------------|-------------|------------|-----------|
| A. Dialogue  | `dialogue_node.py` | 2181 | ≤ 350 | ROS2 pub/sub, DJ mode, barge-in, async loop driver |
| B. Telegram  | `telegram_node.py` + handlers/ | 409 + 916 | ≤ 100 (node) | python-telegram-bot + camera callbacks; **NO LLM** |
| C. Perception | 5 nodes (`context_aggregator` 745, `reflection`, `startup_greeting`, `vision_stub`, `health_monitor`) | 3536 | ≤ 200 (1 new bridge) | UART → ROS2 topics; **NO LLM**, **NO micro-ROS** |

Harness ports (re-used, **not re-written**) per ADR-0001: `LLMProvider`,
`ToolProvider`, `MemoryStore`, `Transport`, `SideEffectBus`, `Clock`.

Harness adapters (re-written): `harnesses/dialog.py`, `harnesses/telegram.py`,
`harnesses/persistent.py` — they were "parallel wrappers" in v1, never wired.

## 2. Task graph (re-derived from 06-01..06-04 PLANs)

12 waves → 12 atomic tasks → 3 dependency chains:

```
Group A (Dialogue) — depends on existing harness ports:
  W1 ✅ done (commit 06dbd5a8)  DeepSeekProvider + MiMoProvider
  W2 ▶ WIP                       ToolRegistry (29 + 5 = 34 tools)
  W3                              DialogCore composing DSM + LLM + Tools + Memory
  W4                              MemoryStore extensions (waypoints / FAQ / EventProfile)
  W5                              Rewrite dialogue_node.py as thin shell (≤350 lines)
  W6                              Integration tests: dialogue shell + fake ports

Group B (Telegram) — independent of A:
  W7                              Remove LLM deps from telegram_node + handlers/
  W8                              Rewrite telegram_node.py as pure ROS2 bridge (≤100 lines)
  W9                              Integration tests: telegram bridge

Group C (Perception) — independent of A and B:
  W10                             Remove LLM deps from perception; delete reflection/startup_greeting/vision_stub
  W11                             Consolidate into perception_bridge.py (≤200 lines, UART → /sensors/data)
  W12                             Integration tests: perception bridge

Acceptance (every wave):
  - separate commit on feature/harness-p0-foundation
  - push to origin/feature/harness-p0-foundation
  - W6 / W9 / W12 only depend on prior waves in their own group
  - no merge into main (user does that)
```

## 3. State at 2026-07-28 10:25 (snapshot)

* W1 ✅ committed (`06dbd5a8` — `HarnessDeepSeekProvider + HarnessMiMoProvider`,
  14 tests pass, exports in `providers/__init__.py`, pushed to remote).
* W2 ⚠ in-flight by another worker (test file `test_tool_registry.py` is
  untracked but `core/tool_registry.py` source not yet written).
* W3-W12: not started.

Out-of-scope items (per 06-CONTEXT.md):
* Sensor-board firmware (separate project, not Python).
* Docker / build-system updates for harness (DOCKER-06 / DOCKER-07 in 06-VALIDATION.md).
* PR #907 merge (user does it).
* P1 features (capability filtering etc.).

## 4. Risk register (analyst view)

| ID | Risk | Observable signal | Mitigation |
|----|------|-------------------|-----------|
| R1 | Multi-package refactor of ~6000 LOC may exceed single-worker time budget | 5b618648 → no further commits in last 30 min | Decompose into 11 child tasks; cap each ≤ 2 hours of focused work |
| R2 | pytest-asyncio mode=auto not set in harness `pytest.ini`; 28 async tests already failing pre-existing | `28 failed, 441 passed` today | Plan-time decision: separate task to fix harness pytest.ini OR mark async tests with `@pytest.mark.asyncio` |
| R3 | GitHub push requires credentials not in this container | `fatal: could not read Username for 'https://github.com'` | Push from host where credentials exist; orchestrator confirms remote HEAD matches local |
| R4 | Concurrent writers on `feature/harness-p0-foundation` cause push conflicts | `git pull --rebase` required before each push | Each child task rebases on `origin/feature/harness-p0-foundation` before push |
| R5 | W2 test file exists without source — risk of orphan test if W2 worker aborts | `test_tool_registry.py` is untracked, source missing | Include "create source + verify import + run tests" in W2 acceptance |

## 5. Recommended child-task structure (for kanban-create)

Each child is a self-contained wave, assigned to `backend` profile, with:

* `parents=[t_7cffa9de]` (auto-promote when parent completes)
* explicit `dependants` via inline `kanban_comment` (sequential W2→W3→W4→W5→W6)
* `workspace_kind=worktree` reusing `feature/harness-p0-foundation`
* per-wave `verify` snippet copied from the plan file

Suggested split (12 cards):

```
A-2  W2 ToolRegistry                → backend, depends on nothing
A-3  W3 DialogCore + DSM integration → backend, block on A-2
A-4  W4 MemoryStore extensions       → backend, block on nothing (independent of W2/W3)
A-5  W5 dialogue_node shell rewrite  → backend, block on A-2, A-3, A-4
A-6  W6 dialogue shell tests         → backend, block on A-5
B-7  W7 Remove LLM from telegram     → backend, depends on nothing
B-8  W8 telegram ROS2 bridge         → backend, block on B-7
B-9  W9 telegram bridge tests        → backend, block on B-8
C-10 W10 Remove LLM from perception  → backend, depends on nothing
C-11 W11 perception_bridge.py        → backend, block on C-10
C-12 W12 perception bridge tests     → backend, block on C-11
+    ENV-MYPY-21 fix harness pytest.ini asyncio_mode=auto → backend, parallel
```

## 6. Acceptance criteria (mirrored from parent body)

For each child task:

1. All non-out-of-scope tasks done, each as a separate commit.
2. Each commit on `feature/harness-p0-foundation` branch in `feature/harness-p0-foundation`.
3. `git push origin feature/harness-p0-foundation` succeeds (host has creds).
4. Working tree clean at completion of each wave.
5. No merge into `main`.

## 7. Verification gates (per 06-VALIDATION.md)

* Per task commit: `python3 -m pytest src/rob_box_harness/test -x -q` (run only the
  touched test file — full harness suite has 28 pre-existing async failures).
* Final (after A-6, B-9, C-12): all 3 new tests files green +
  `mypy --strict src/rob_box_harness/rob_box_harness/providers/deepseek.py
  src/rob_box_harness/rob_box_harness/providers/mimo.py` clean.

## 8. Communication protocol

* Parent task t_7cffa9de stays in `running` until all 11 child waves reach `done`.
* Each child updates `.planning/phases/06-harness-p0-finalization/progress.md`
  with: wave id, commit SHA, push result, test summary.
* On any wave failure that requires human input, use `kanban_block(kind='needs_input')`.

---

*Analyst sign-off: scope fully understood, decomposed into 11 atomic child tasks,
W1 already shipped in 06dbd5a8, no further work from analyst required — parent
will complete with handoff metadata pointing to the child card ids.*
