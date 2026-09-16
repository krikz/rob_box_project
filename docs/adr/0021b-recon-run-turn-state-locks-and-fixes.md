# ADR-0021b — Recon: `_run_turn` shared state, locks, and live-incident `🔴 FIX` markers

**Дата:** 2026-09-16
**Статус:** reconnaissance (kanban t_cee156f5)
**Автор:** architect
**Branch:** `z-architect/2627-recon-run-turn-state-and-fixes`
**Связанные:** ADR-0021, ADR-0018, issue #2627, #935, #968, #992, #1204,
#1881, #2559, #2565, PR #2631 (issue #2631, уже merged → 6c7496bac),
PR #2640 (open), PR #2647 (post-turn finalizer ADR, open)

---

## 0. Контекст и предупреждение о stale-baseline

Тело issue #2627 ссылается на «`_run_turn` CC=59, 200-line `finally` block,
lines 3505–4004». Это **pre-merge snapshot**. PR #2631 (cc-architect, merged
в develop коммитом `6c7496bac` «_run_turn CC 59→15») уже разнёс бóльшую
часть того `finally` в helper'ы. Эта рекогносцировка описывает
**текущее** состояние кода (HEAD = `wt/t_cee156f5`, branch = `develop` +
unmerged, см. `git log --oneline -1`).

Radon-факт (`radon cc -s -n C -a src/rob_box_voice/rob_box_voice/dialogue_node.py`):

```
M 5921:4 DialogueNode._handle_result   - F (85)   ← главный долг ADR-0021 R1
M 5185:4 DialogueNode._apply_music_guard - C (14)
M 3374:4 DialogueNode._run_turn         - C (13)   ← после PR-2631
```

`_run_turn` сейчас 130 LOC (3374–3504 — prologue; 3504–3622 — body+finally).
Реальный «finally-блок, который проблема» — это **~95 строк**, 3529–3622,
большая часть которых уже диспатчит helper'ы. Конкретный inline-сайт
остаётся только для `_task_lock`-релиза (lines 3536–3538) и вызовов
helper'ов.

Все file:line ниже — `src/rob_box_voice/rob_box_voice/dialogue_node.py`,
если не указано иное.

---

## 1. Six pieces of shared mutable state — read/write map

### 1.1 `self._run_task: Optional[asyncio.Task]`

| Aspect | Site | Note |
| --- | --- | --- |
| Decl / init | `dialogue_node.py:481` (`__init__`: `self._run_task: Optional[asyncio.Task] = None`) | shared across recursion |
| Write (top of frame) | `dialogue_node.py:3388-3389` (`_run_turn`: `with self._task_lock: self._run_task = asyncio.current_task()`) | per-frame |
| Read (guard) | `dialogue_node.py:2616` (`_cancel_run`/`_is_active`: `run_task = self._run_task`) | shared (reads from any caller) |
| Read (predicate) | `dialogue_node.py:997-998` (`is_active=lambda: (self._run_task is not None and not self._run_task.done())`) | shared |
| Write (release in `finally`) | `dialogue_node.py:3536-3538` (`with self._task_lock: if self._run_task is asyncio.current_task(): self._run_task = None`) | per-frame; protected by lock |

**Classification:** SHARED across recursion (task slot is single-valued),
guarded by `self._task_lock`. Bug B fix (issue #992) — never wipe
a slot that another guard (`_apply_music_guard`) just replaced with a
fresh follow-up DJ coroutine. See inline comment lines 3530–3535.

### 1.2 `self._task_lock: threading.Lock`

| Aspect | Site | Note |
| --- | --- | --- |
| Decl / init | `dialogue_node.py:482` | shared across recursion |
| Read sites | `dialogue_node.py:3388` (`_run_turn` top), `3536` (`_run_turn` `finally`) | per-frame acquire, lock is shared |

**Classification:** SHARED infrastructure lock. The only two callers are
the two writes of `_run_task` (top + release). Per-frame usage — no
contention within one `_run_turn`, but if `_apply_music_guard` (in
`finally`) synchronously dispatches a new turn, the new turn's top-acquire
will queue behind the parent's release-acquire.

### 1.3 `self._synthetic_retries_left: int`

| Aspect | Site | Note |
| --- | --- | --- |
| Decl / init | `dialogue_node.py:980` (`DEFAULT_SYNTHETIC_RETRIES`) | shared |
| Reset (user-turn only) | `dialogue_node.py:4837` (`_reset_turn_retry_budgets: self._synthetic_retries_left = self.DEFAULT_SYNTHETIC_RETRIES`) | per-frame; gated by `is_synthetic` check at 4826 |
| Decrement (every guard) | `dialogue_node.py:4608` (`_consume_synthetic_retry: self._synthetic_retries_left -= 1`) | shared — guards in child frame can drain the parent's budget |
| Read (mirror to TurnState) | `dialogue_node.py:3968-3970` (`_check_babble_and_retry: self._synthetic_retries_left = decision.new_state.budget_left`) | shared |

**Classification:** SHARED across recursion (single integer; budget is
per user-initiated turn but visible to all guard frames). Per issue
#1881 — every guard decrements the SAME budget to prevent ping-pong.
**Re-entrancy invariant:** decrement is monotonically non-increasing
across one user-turn + its retries. See issue #1881 docs at 5556–5571.

### 1.4 `self._pending_music_cleanup: bool`

| Aspect | Site | Note |
| --- | --- | --- |
| Decl / init | `dialogue_node.py:845` (`self._pending_music_cleanup: bool = False`) | shared |
| Write (caller, was_idle path) | `dialogue_node.py:2756` (`_dispatch_turn: elif self._pending_music_cleanup and was_idle: self._pending_music_cleanup = False; self._publish_music_cleanup(...)`) | shared; `was_idle` gate prevents racing the parent's cleanup |
| Write (stop_music deferral) | `dialogue_node.py:4865-4874` (`_apply_stop_music_deferral: ... self._pending_music_cleanup = True`) | per-frame |
| Write (scheduling) | `dialogue_node.py:4901-4902` (backing mode → `True`), `4913` (LLM restarted → `False`), `4929` (default arming) | per-frame |
| Write (flush) | `dialogue_node.py:4951` (`_flush_music_cleanup_if_idle: self._pending_music_cleanup = False`) | per-frame |
| Read (predicate) | `dialogue_node.py:2624` (`_publish_music_cleanup_if_idle: if not self._pending_music_cleanup: return`), `3058` (analytics) | shared |

**Classification:** SHARED across recursion (single bool). Issue #992
Bug B contract: parent turn's `finally` may run while a child turn
already set `_pending_music_cleanup = True` via the same `_schedule_music_cleanup`
helper — read in `was_idle` arm (2748) is the **only** multi-frame race
site, and the `was_idle` flag is true only at the outermost dispatch
(no nested idle-starts).

### 1.5 `self._retry_dispatched_in_turn: bool`

| Aspect | Site | Note |
| --- | --- | --- |
| Decl / init | `dialogue_node.py:879` | shared |
| Reset (every frame, top) | `dialogue_node.py:3430` (`_run_turn: self._retry_dispatched_in_turn = False`) | per-frame |
| Set (any guard fires) | `dialogue_node.py:4581` (`_mark_retry_dispatched: self._retry_dispatched_in_turn = True`) | per-frame (only set within `_apply_*_guard`) |
| Read (`finally`) | `dialogue_node.py:3514` (`guard_retry_pending = bool(self._retry_dispatched_in_turn)`) | per-frame local |
| Read (music guard predicate) | `dialogue_node.py:5237-5242` (`_apply_music_guard: if self._retry_dispatched_in_turn: ... return False`) | shared — guards check before they act |

**Classification:** SEMI-SHARED. Reset per-frame at the top of every
`_run_turn`, but readable from any guard. The point is: two consecutive
guards in the SAME frame can both read `False` and both fire — so the
gating is **enforced at consumer level** by `_apply_tool_skipped_guard`'s
`other_retry_dispatched` parameter (5510), not by the flag alone. See
`_apply_music_guard`'s own check at 5237 for the alternate gate.

### 1.6 The five mode flags (`is_dj_auto`, `is_babble_retry`, `is_action_claim_retry`, `is_code_retry`, `is_synthetic`)

These are **parameters of `_run_turn` / `_dispatch_turn`** (3378–3387,
2717–2726), not `self.*` state. Each frame binds them as locals at
dispatch time. They flow through:

```
_dispatch_turn(...)                              # lines 2714-2780
  └─> asyncio.run_coroutine_threadsafe(
          self._run_turn(..., is_dj_auto=is_dj_auto, is_babble_retry=..., ...),
          self._loop,
      )
```

Inside `_run_turn`, only `was_dj_auto = is_dj_auto` (line 3395) is kept
as a per-frame local; the others are passed through to helpers:
`was_dj_auto` → `_finalize_music_cleanup_policy`, `_apply_music_guard`,
`_handle_llm_error`; `is_synthetic` → `_reset_turn_retry_budgets`.

**Classification:** PER-INVOCATION (not per-coroutine). Crucial invariant
(ADR-0021 already-implemented via `turn_kind` snapshot in PR-D, see
recent commits `e2a6c1af3`, `a693a04cf`): parent-turn flag set MUST NOT
leak into child-turn because the child gets its own `_dispatch_turn`
call with explicit params.

---

## 2. Every `🔴 FIX (...)` marker in the `finally` block (lines 3529–3621)

The remaining `🔴 FIX` markers AFTER PR-2631 (others were moved into
helpers and the comments now live next to their helper bodies):

| # | File:line | Marker | Branch / condition | Side-effect | Linked issue |
| --- | --- | --- | --- | --- | --- |
| F1 | `dialogue_node.py:3525` | `# (живой 12.08 FIX — НЕ падать, если логгер уронит RcutilsLogger)` | Inside `_handle_llm_error` try/except | Speak-direct + logger wrapped in separate try/except blocks | live 12.08, #1278 |
| F2 | `dialogue_node.py:3530-3535` | `# Issue #992 Bug B: ...` | `finally` slot-clear under `_task_lock` | Skip `_run_task = None` if a guard dispatched a replacement | #992 B |
| F3 | `dialogue_node.py:3539-3545` | `# S7 (scheduler-segments-merge, issue #968) — drain any user phrases ...` | After `_run_task` cleared | `_drain_pending_user_messages()` → may re-enter `_run_turn` | #968 (S7) |
| F4 | `dialogue_node.py:3547-3552` | `# Issue #935 v3: if LLM called stop_music(), defer cleanup until TTS finishes. # Issue #992: a second stop_music() call from a follow-up LLM turn ... must be ignored` | `_finalize_music_cleanup_policy` first arm | Arm / disarm `_pending_music_cleanup` | #935 v3, #992 |
| F5 | `dialogue_node.py:3561-3573` | `# 🔴 FIX (issue #1204, 13.08 DJ incident): guard вызывается ДО DIALOGUE_END. Сам guard переоткрывает DIALOGUE перед ретраем, поэтому закрывать диалог здесь нужно только если ретрай НЕ был задиспатчен` | `_apply_music_guard` invocation | Defer DIALOGUE_END if `music_retry_dispatched` | #1204 |
| F6 | `dialogue_node.py:3577-3584` | `# Issue #2565 — phantom-action deferral needs the LLM's reply text to detect «запустил/загрузил» claims before the FORCE_STOP branch silences the active track` | Pass `spoken=result.spoken_text` to `_apply_music_guard` | Phantom-action detector inside MusicGuard sees the actual reply | #2565 |
| F7 | `dialogue_node.py:3586-3603` | `# Issue #1777 / #1762 — Bug C retry для non-music tool-based запросов. ... защита от ping-pong: один ретрай на turn` | `_apply_tool_skipped_guard` invocation | Non-music tool skip → 1 critical retry | #1777, #1762 |
| F8 | `dialogue_node.py:3604-3615` | `# Issue #992 Bug D — defer the DIALOGUE_END transition when the babble detector scheduled a retry ... S7 — same reasoning for a drained pending-queue follow-up turn` | `_finalize_turn_dsm` predicate | Suppress DIALOGUE_END if `guard_retry_pending or music_retry_dispatched or pending_queue_dispatched or tool_retry_dispatched` | #992 D, #968 S7 |

### Additional `🔴 FIX` markers elsewhere in the helper bodies that the
finally block now transitively depends on (must be preserved 1:1 by
any refactor that moves the finalizer into its own class):

- `# Issue #1204 ...` in `_mark_retry_dispatched` docstring (4567–4581)
- `# Issue #1881 — общий budget декрементится здесь ...` in `_consume_synthetic_retry` (4602–4609)
- `# 🔴 FIX (live 30.08): юзер сказал «останови музыку», LLM ответила «Музыка выключена.» и не вызвала stop_music — трек продолжал играть` — `MusicGuardVerdictKind.FORCE_STOP` branch in `_apply_music_guard` (5331–5346)
- `# 🔴 FIX (live 30.08, e2e renardo_evolve rn02): ... ушло ДВА синтетических ретрая` — `_apply_music_guard` early-return on `_retry_dispatched_in_turn` (5229–5242)
- `# 🔴 FIX (issue #1101): НЕ используем формат «[Говорит <имя>]» ...` in `_apply_speaker_identity` (2799–2804)
- `# 🔴 FIX (live 12.08): защита от мусорных имён в БД ...` (2834–2840) and `# всегда добавляем имя спикера ...` (2845–2854)
- `# 🔴 FIX (issue 992 live 09:09): LLM часто делает speak_text(прелюдия)` (2609) — deferral hook still referenced by cleanup
- `# 🔴 FIX (issue 992 live 08:55): music_cleanup reason=new_dialogue ТОЛЬКО при новом диалоге из IDLE` in `_dispatch_turn` (2748–2757) — entry-point gate
- `# 🔴 FIX (live 02.09): «во время сочинения музыки LLM много говорит»` (6146) — referenced by batch_complete bookkeeping

**Net for refactor:** any finalizer must take a `TurnContext` snapshot
that includes `was_dj_auto`, `result`, `raw_user_command`, `user_input`,
`guard_retry_pending`, `music_retry_dispatched`,
`pending_queue_dispatched`, `tool_retry_dispatched`, `spoken`, and
preserve the **ordering**: slot-release → drain → music-policy →
music-guard → tool-guard → DSM-finalize. Reordering any of F1–F8 will
break a live-incident fix.

---

## 3. Five mode flags × 32 combinations

### 3.1 Read sites (where any of the 5 flags is consulted)

| Site | Reads | Note |
| --- | --- | --- |
| `_run_turn:3378-3387` | Declares all 5 as params | Default: all False |
| `_run_turn:3395` (`was_dj_auto = is_dj_auto`) | `is_dj_auto` | Per-frame local; threaded through helpers |
| `_run_turn:3413` (`_reset_turn_retry_budgets(is_synthetic=...)`) | `is_synthetic` | Gates the budget reset |
| `_run_turn:3445` (`if speaker_tag and not is_dj_auto:`) | `is_dj_auto` | Skip speaker bio in DJ transitions |
| `_run_turn:3514` (`guard_retry_pending = bool(self._retry_dispatched_in_turn)`) | (derived) | Not a mode flag — derived state |
| `_run_turn:3574-3585` (`_apply_music_guard(... was_dj_auto=was_dj_auto ...)`) | `was_dj_auto` | Passed as param to guard |
| `_run_turn:3599-3603` (`_apply_tool_skipped_guard(... raw_user_command=raw_user_command or user_input ...)`) | (none directly) | Tool guard reads `is_synthetic` via `raw_user_command`-derived path only |
| `_run_turn:3555-3560` (`_finalize_music_cleanup_policy(... was_dj_auto=was_dj_auto ...)`) | `was_dj_auto` | |
| `_run_turn:3616-3621` (`_finalize_turn_dsm(... music_retry_dispatched=music_retry_dispatched ...)`) | (none directly) | Reads guard verdicts, not mode flags |
| `_handle_result` (5921+) | (none of the 5 directly) | Mode flags are dispatched-time params, not consumed by `_handle_result` |
| `_dispatch_dj_turn` (called by `_apply_music_guard:5286`) | Sets `is_dj_auto=True` on retry prompt | DJ-transition specific |
| `_check_babble_and_retry:3984-3990` (`_dispatch_turn(..., is_babble_retry=True, is_synthetic=True, raw_user_command=user_input)`) | Sets `is_babble_retry` and `is_synthetic` | Babble retry dispatch |
| `_apply_music_guard:5318-5327` (`_dispatch_turn(verdict.prompt, was_idle=False, raw_user_command=user_input, is_synthetic=True)`) | Sets `is_synthetic=True` for music USER_RETRY | |
| `_apply_tool_skipped_guard:5541-5546` (`_dispatch_turn(retry_prompt, was_idle=False, raw_user_command=user_input, is_synthetic=True)`) | Sets `is_synthetic=True` for tool retry | |

### 3.2 The 32 = 2^5 combinations — which are reachable

Naming: `D=is_dj_auto, B=is_babble_retry, A=is_action_claim_retry,
C=is_code_retry, S=is_synthetic`. `1` = True.

| # | D | B | A | C | S | Reachable? | Site | Comment |
| --- | :-: | :-: | :-: | :-: | :-: | --- | --- | --- |
| 0 | 0 | 0 | 0 | 0 | 0 | ✅ | `_on_stt` / `_dispatch_turn` user path | Default user-initiated turn |
| 1 | 0 | 0 | 0 | 0 | 1 | ❌ | — | No code site sets ONLY `is_synthetic=True` without a sibling flag |
| 2 | 0 | 0 | 0 | 1 | 0 | ❌ | — | `is_code_retry=True` is wired through `*_retry` paths but never reached as standalone |
| 3 | 0 | 0 | 0 | 1 | 1 | ✅ | `_check_embedded_renardo_code_and_retry` (3993+) → `_dispatch_turn(is_code_retry=True, is_synthetic=True, ...)` | Code-speech retry |
| 4 | 0 | 0 | 1 | 0 | 0 | ❌ | — | `is_action_claim_retry` only used in conjunction with `is_synthetic=True` |
| 5 | 0 | 0 | 1 | 0 | 1 | ✅ | `_check_unbacked_action_claim_and_retry` (sibling of babble) → `_dispatch_turn(is_action_claim_retry=True, is_synthetic=True, ...)` | Unbacked claim retry |
| 6 | 0 | 0 | 1 | 1 | 0 | ❌ | — | A+C non-synthetic not constructed |
| 7 | 0 | 0 | 1 | 1 | 1 | ❌ | — | No code site fires both A+C |
| 8 | 0 | 1 | 0 | 0 | 0 | ❌ | — | `is_babble_retry=True` only used with `is_synthetic=True` |
| 9 | 0 | 1 | 0 | 0 | 1 | ✅ | `_check_babble_and_retry:3984-3990` | Babble retry — see comments at 3979–3983 |
| 10 | 0 | 1 | 0 | 1 | 0 | ❌ | — | B+C not constructed |
| 11 | 0 | 1 | 0 | 1 | 1 | ❌ | — | B+C+S not constructed |
| 12 | 0 | 1 | 1 | 0 | 0 | ❌ | — | |
| 13 | 0 | 1 | 1 | 0 | 1 | ❌ | — | |
| 14 | 0 | 1 | 1 | 1 | 0 | ❌ | — | |
| 15 | 0 | 1 | 1 | 1 | 1 | ❌ | — | |
| 16 | 1 | 0 | 0 | 0 | 0 | ✅ | DJ-tick → `_dispatch_dj_turn` | Fresh DJ transition |
| 17 | 1 | 0 | 0 | 0 | 1 | ❌ | — | DJ retries go through `_dispatch_dj_turn` (from_tick=False), which sets `is_dj_auto=True` without `is_synthetic=True` |
| 18 | 1 | 0 | 0 | 1 | 0 | ❌ | — | |
| 19 | 1 | 0 | 0 | 1 | 1 | ❌ | — | |
| 20 | 1 | 0 | 1 | 0 | 0 | ❌ | — | |
| 21 | 1 | 0 | 1 | 0 | 1 | ❌ | — | |
| 22 | 1 | 0 | 1 | 1 | 0 | ❌ | — | |
| 23 | 1 | 0 | 1 | 1 | 1 | ❌ | — | |
| 24 | 1 | 1 | 0 | 0 | 0 | ❌ | — | DJ + babble — BabbleGuard never sees `is_dj_auto=True` because the DJ dispatcher doesn't pass it; if it ever did, the user-budget reset would not fire (`is_synthetic=True` blocks reset at 4826) |
| 25 | 1 | 1 | 0 | 0 | 1 | ❌ | — | Same reasoning |
| 26–31 | | | | | | ❌ | — | All DJ-multi-flag combinations unreachable today |

**Reachable: 5 of 32.** Practical `TurnMode` enum (see ADR-0021a / PR
#2647) collapses to:

```
USER              (D=0,B=0,A=0,C=0,S=0)         — user-initiated turn
CODE_RETRY        (D=0,B=0,A=0,C=1,S=1)
ACTION_CLAIM      (D=0,B=0,A=1,C=0,S=1)
BABBLE_RETRY      (D=0,B=1,A=0,C=0,S=1)
DJ_TICK           (D=1,B=0,A=0,C=0,S=0)
```

5 reachable + 27 unreachable. Validation should reject every other
combination at construction time.

**Additional invariant (live-relevant):** `is_dj_auto=True` ⇒
`is_synthetic=False`. This is because DJ-tick dispatches happen from
`_dispatch_dj_turn` which never sets `is_synthetic=True` (a DJ-tick is
NOT a synthetic prompt — it's the next auto-cycle). Conversely, every
`is_synthetic=True` dispatch must come from a guard (`_check_*_and_retry`
or `_apply_*_guard`), never from `_on_stt`.

---

## 4. Re-entry path: `_dispatch_turn → _run_turn → guard → _dispatch_turn → _run_turn`

Concrete trace: **user says «спой рэп про роботов»**. The flow
`_dispatch_turn(user_input)` at 2743-ish → `_run_turn(user_input)` at
3388 → LLM returns `tools_called=[]`, `spoken_text="Зачитаю рэп про
роботов!"` → `_handle_result` → return to `_run_turn` → fall through to
`finally` → guards fire → `_apply_music_guard` matches Bug C
(`verdict.kind = MusicGuardVerdictKind.USER_RETRY`,
`tools_now ∩ MUSIC_STOP_TOOLS == ∅`, `_user_wants_music(user_input)` is
True) → guard calls `self._dispatch_turn(verdict.prompt,
was_idle=False, raw_user_command=user_input, is_synthetic=True)` at
5318-5327 → that schedules a fresh `_run_turn` on `self._loop` →
recursion into `_run_turn` frame #2.

### Frame #1 (parent) → Frame #2 (child) state mutation map

Between the moment `_apply_music_guard:5318` invokes `_dispatch_turn`
(child scheduled on `self._loop`) and the moment the child's `_run_turn`
runs to its own `finally`, the following `self.*` mutates **inside frame
#1's `finally` BEFORE the child starts**, AND **inside frame #2's body
BEFORE its own `finally`**:

**Inside parent (`_run_turn`) frame #1 `finally` — after guard fires:**

| State | Site | Before child starts | Effect on child |
| --- | --- | --- | --- |
| `self._run_task` | 3536-3538 | cleared to `None` (lock-protected; guarded by `if self._run_task is asyncio.current_task()` — currently True) | Child can acquire `_run_task` slot at its own 3388-3389 |
| `self._pending_user_messages` | `7062-7083` (`_drain_pending_user_messages`) | may dispatch another turn (S7); clear queue | If drained, ANOTHER child frame is scheduled |
| `self._pending_music_cleanup` | `_schedule_music_cleanup:4877+` (called inside `_finalize_music_cleanup_policy:4985`) | parent's `tools_called=[]` and `_track_mode_music_active=False` ⇒ arms `True` | Child sees the armed flag |
| `self._track_mode_music_active` | 4894 | `False` (no music tools called) | Child inherits `False` |
| `self._retry_dispatched_in_turn` | 4581 (`_mark_retry_dispatched`) | set to `True` by guard | Child starts with `_retry_dispatched_in_turn=False` (line 3430 resets per-frame) — leak closed |
| `self._synthetic_retries_left` | 4608 (`_consume_synthetic_retry:music_user`) | decremented 2 → 1 | Child reads `1` from its top |
| `self._babble_retry_used` | 3944 (mirror in babble shell — not for music) | n/a for music USER_RETRY | n/a |
| `self._tool_retry_used` | 4828+ (only on user-initiated, not in child) | unchanged | Child's body is synthetic ⇒ no reset at 4826 |
| `self._turn_state` | 4844-4846 (only on user-initiated) | unchanged | Child inherits parent's `TurnState` (no init) |
| `self._dsm.current_state` | `_reopen_dialogue_for_retry:4549-4565` | re-driven: IDLE → WAKE_WORD → STT_RESULT, ends in DIALOGUE | Child reads `DIALOGUE` at its own body (wake-word classifier sees DIALOGUE and short-circuits to STT_RESULT path) |

**Per-frame locals that DO NOT propagate** (per-invocation, ADR-0021 PR-D
`turn_kind` invariant):

- `was_dj_auto` (line 3395) — per-frame local; child does NOT inherit
  parent's `is_dj_auto=True` (music USER_RETRY sets
  `is_synthetic=True` only)
- `is_babble_retry / is_action_claim_retry / is_code_retry` — child
  carries `False` for all unless the child's own guard fires

### Cross-frame correctness invariants

1. **`_run_task` slot must be cleared BEFORE the child starts.** This is
   the slot-release at 3536-3538 — `with self._task_lock: if
   self._run_task is asyncio.current_task(): self._run_task = None`. If
   `_apply_stop_music_deferral` (or any later step) re-armed
   `_pending_music_cleanup` synchronously, the guard at 3537 ensures we
   don't wipe a freshly-set slot. Issue #992 Bug B.

2. **`_retry_dispatched_in_turn` MUST be re-armed by the parent's
   `_mark_retry_dispatched` (line 5275 / 5310 chain)** BEFORE the child
   starts; the child's own body resets it at 3430 (per-frame), but
   `_finalize_turn_dsm` reads the parent's frame-local
   `music_retry_dispatched` (not the flag), so DSM-correctness depends
   on the **return value** of `_apply_music_guard` and friends, not the
   flag. Issue #1204.

3. **`_synthetic_retries_left` decrements happen synchronously inside
   the parent's `finally`**, but `asyncio.run_coroutine_threadsafe`
   schedules the child on `self._loop` AFTER the current coroutine yields.
   Since the parent's `finally` is synchronous and finishes before any
   scheduled task resumes, the child sees the decremented value at its
   own top.

4. **`_pending_music_cleanup` is read in `_dispatch_turn` (line 2748)
   for the `was_idle` arm ONLY.** Child dispatches go through
   `_dispatch_turn(..., was_idle=False, ...)` (5318-5327, 5541-5546,
   3984-3990), so the `was_idle` arm doesn't fire for nested frames.
   The child's body then proceeds to `_finalize_music_cleanup_policy`
   on its own `result`, which may arm/disarm per its own tools_called.

5. **The reentry through `_drain_pending_user_messages` (line 3546) can
   schedule YET ANOTHER child** (issue #968 S7). This means the call
   stack can grow to 3 levels: parent → music guard child → drain child.
   Each level holds its own `result` and `was_dj_auto` local; shared
   state (`_run_task`, `_pending_music_cleanup`,
   `_synthetic_retries_left`) is the single point of truth.

---

## 5. Existing tests touching music / retry / finally cleanup — anchors

Group by incident (full path + representative test names; not exhaustive):

### #935 (stop_music cleanup deferral)

- `src/rob_box_voice/test/unit/core/test_dialogue_guards.py` — broader
  guards suite; many `def test_*` for stop_music behaviour
- `src/rob_box_voice/test/unit/test_issue_982_prompt_no_stop_music.py` —
  issue #982 / #935 cluster (prompt-side guard for stop_music)
- `src/rob_box_voice/test/test_issue_992_batch_cleanup.py`:
  - `test_chatty_track_turn_is_not_backing` (line 243)
  - `test_two_speak_text_batches_hold_cleanup_until_last` (line 310)
  - `test_single_speak_text_batch_fires_cleanup_normally` (line 400)
  - `test_multi_chunk_single_batch_fires_cleanup_after_last_chunk` (line 434)
  - `test_stop_music_when_already_pending_is_ignored` (line 468)
  - `test_backing_execute_music_code_with_lyrics_schedules_cleanup` (line 552)
  - `test_track_execute_music_code_single_accept_no_cleanup` (line 628)
  - `test_track_compose_music_single_accept_no_cleanup` (line 678)
  - `test_track_gen_play_from_library_single_accept_no_cleanup` (line 737)

### #992 (Bug B/C/D + dj_mode + babble + batch_cleanup)

- `src/rob_box_voice/test/test_issue_992_dj_mode.py`:
  - `test_dj_dispatch_does_not_publish_new_dialogue_cleanup` (line 123)
  - `test_dispatch_turn_user_path_still_publishes_new_dialogue_cleanup` (178)
  - `test_dj_auto_without_music_triggers_synchronous_retry` (234)
  - `test_dj_auto_retry_budget_caps_loop` (283)
  - `test_dj_auto_with_music_does_not_retry` (324)
  - `test_dj_flag_resets_after_run_turn` (360)
  - `test_user_rap_request_without_music_publishes_nudge` (396)
  - `test_user_normal_chat_without_music_is_silent` (431)
  - `test_user_music_request_with_music_code_no_nudge` (463)
  - `test_empty_response_on_music_request_publishes_fallback` (528)
  - `test_empty_response_on_plain_chat_no_fallback` (550)
  - `test_music_guard_retry_reaches_llm` (605)
  - `test_dj_phrase_does_not_trigger_stop_fallback` (629)
  - `test_real_stop_command_still_forces_cleanup` (666)
- `src/rob_box_voice/test/test_issue_992_babble_guard.py`:
  - `test_babble_response_triggers_one_retry` (201)
  - `test_no_babble_no_retry` (271)
  - `test_pure_promise_opener_retries_without_performance_keyword` (292)
  - `test_retry_only_fires_once_even_if_retry_also_babbles` (318)
  - `test_speak_text_call_suppresses_babble_retry` (350)
- `src/rob_box_voice/test/test_issue_992_batch_cleanup.py` (cross-list w/ #935)

### #968 (S7 scheduler-segments-merge, drain pending user messages)

- `src/rob_box_voice/test/unit/test_issue_968_segment_plan_prompt.py` —
  prompt-level scheduler side; not the `_drain_pending_user_messages`
  body itself but the prompt that triggers S7
- `src/rob_box_voice/test/unit/node/test_barge_in_policy.py`:
  - `test_pending_llm_with_live_task_queues_instead_of_dispatching` (219)
  - `test_pending_llm_with_finished_task_dispatches_normally` (239)

  These exercise the *enqueue* side; the `_drain_pending_user_messages`
  side is exercised by `test_drain_pending_user_messages`-style tests
  via the `dialogue_node_echo.py` suite (search needed in repo to
  confirm — out of scope of this recon, see "Open follow-ups" below).

### #1204 (DJ guard reopen + DIALOGUE_END deferral)

- `src/rob_box_voice/test/unit/node/test_dialogue_retry_flag_wiring.py` —
  integration tests for the `_retry_dispatched_in_turn` flag wiring
  (exercises 3430, 3514, 3616-3621 paths)

### #1881 (synthetic retry budget)

- `src/rob_box_voice/test/unit/core/test_issue_1881_synthetic_retry_budget.py`:
  - `test_initial_budget_is_default` (177)
  - `test_first_consume_returns_true` (181)
  - `test_second_consume_returns_true` (186)
  - `test_third_consume_returns_false_and_warns` (192)
  - `test_budget_exhausted_returns_false` (206)
  - `test_two_guards_share_the_same_budget` (231)
  - `test_no_ping_pong_when_budget_exhausted` (244)
- `src/rob_box_voice/test/unit/node/test_issue_1895_dj_retry_budget.py` —
  DJ-tick budget (sibling budget path)
- `src/rob_box_voice/test/unit/core/test_issue_1895_dj_retry_budget_chain.py` —
  DJ chain

### #2565 (phantom-action deferral)

- `src/rob_box_voice/test/unit/core/test_issue_2565_phantom_action_defers_stop.py`:
  - `test_live_oakenfold_phantom_action_defers_force_stop` (76)
  - `test_phantom_action_track_load_claim_with_stop_word` (110)
  - `test_real_stop_with_stop_music_tool_still_skips` (129)
  - `test_real_stop_without_phantom_still_force_stops` (149)
  - `test_no_spoken_param_does_not_crash_and_preserves_force_stop` (165)
  - `test_phantom_action_does_not_consume_user_retry_budget` (184)
  - `test_bug_c_still_reprompts_on_real_music_skip` (215)
  - `test_bug_b_dj_retry_unaffected` (231)

### #1777 / #1762 (non-music tool-skipped guard)

- `src/rob_box_voice/test/test_issue_1777_tool_skipped_guard.py` &
  `src/rob_box_voice/test/unit/node/test_issue_1777_tool_skipped_guard.py`:
  - `test_time_request_dispatches_retry` (166)
  - `test_pogoda_dispatches_search_web_retry` (219)
  - `test_retry_dispatched_in_turn_blocks_tool_retry` (357)
  - `test_other_retry_dispatched_param_blocks_tool_retry` (372)
  - `test_retry_reopens_dialogue` (390)
  - `test_kotoryy_chas_hallucination_from_issue` (428)

### #2536 / Occasion dispatch (occasion seam, PR-B etc.)

- `src/rob_box_voice/test/unit/node/test_occasion_dispatch.py`
- `src/rob_box_voice/test/unit/node/test_dialogue_node.py` — broad
  integration harness

### Helper-coverage / pure-method tests

- `src/rob_box_voice/test/unit/core/test_turn.py` — `begin_babble_retry`
  / `TurnState` / `TurnGuards` / `BabbleGuard` / `MusicGuard` (29+ tests)
- `src/rob_box_voice/test/unit/core/test_music_guard.py` — 30+ tests
  covering verdict matrix
- `src/rob_box_voice/test/unit/core/test_dialogue_guards.py` — keyword
  predicates (MUSIC_GUARD_KEYWORDS, BABBLE_BANNED_OPENERS, etc.)
- `src/rob_box_voice/test/unit/node/test_pure_methods.py` — pure
  helper coverage
- `src/rob_box_voice/test/unit/node/test_dialogue_node.py` — broad
  integration tests
- `src/rob_box_voice/test/test_dialogue_shell.py` — top-level
  exercise of the whole node

### Open follow-ups for test plan (t_e8b6676a)

1. Locate the test exercising `_drain_pending_user_messages` directly
   (likely in `test_dialogue_node.py` or via `test_barge_in_policy.py`'s
   `test_pending_llm_*` paths). Confirm the integration test asserts the
   **drain-after-finally** ordering (3546 runs AFTER 3536-3538 cleared
   the slot).
2. Locate the integration test that exercises `_apply_music_guard`
   re-entry — `test_dj_auto_without_music_triggers_synchronous_retry`
   (test_issue_992_dj_mode.py:234) covers the DJ_USER_RETRY path; the
   **music USER_RETRY** re-entry path (parent → music USER_RETRY child)
   should have an explicit test of the same shape (none found yet).
3. `test_dialogue_retry_flag_wiring.py` — verify which invariants it
   asserts. Likely covers `_retry_dispatched_in_turn` end-to-end (lines
   3430, 3514, 3616-3621).

---

## 6. References

- ADR-0021 (`docs/adr/0021-dialogue-node-decomposition-discipline.md`) —
  CC-budget R1, SSoT R2, per-bag workflow R3, lazy-import ceiling R4,
  issue-link R5
- ADR-0021-r1 (`docs/adr/0021-r1-cc-budget-ratchets-and-phantom-detection.md`) —
  R-1a..R-1g ratchets; refs issue #2626 for the broken-guard backstory
- ADR-0018 (`docs/adr/0018-agent-honesty-culture.md`) — культура честности
- PR #2631 (merged, commit `6c7496bac`) — actual extraction: reset
  budgets, music cleanup, DSM finalize, LLM telemetry, error handler,
  user-input prelude → `_run_turn` CC 59 → 13 (radon)
- PR #2640 (open) — follow-on micro-helpers (PR-A+B+C+D)
- PR #2647 (open, branch `z-architect/2627-adr-post-turn-music-finalizer`)
  — PostTurnMusicPolicy ADR (t_adc73325) which **supersedes** the
  `_MusicTurnFinalizer` class proposal with a dict-of-callables dispatch
  per `core/turn.py:783+`. This recon was written knowing that PR — the
  state-map and FIX-preservation table above are the data needed by
  PR-E1 / PR-E2 micro-helper extraction.
- Issue #2627 — root card
- Issue #1881 — synthetic retry budget ping-pong
- Issue #2266 — `TurnState` mirror / ADR-0084

