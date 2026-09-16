# ADR-0021c — Финализатор `_run_turn`: sequence-диаграммы (before/after) и risk register

**Дата:** 2026-09-16
**Статус:** design artefact (kanban t_51496162, parent t_cee156f5)
**Автор:** architect
**Branch:** `wt/t_51496162`
**Связанные:** ADR-0021 (decomposition discipline), ADR-0021b
(recon-note, post-PR-2631 state, 517 LOC, коммит `9fbbcac92`),
ADR-0018 (честность), ADR-0013 (incremental delivery),
issue #2627 (пост-turn finalizer), issue #935 (stop_music deferral),
issue #992 (Bug B/C/D), issue #968 (S7 drain), issue #1204 (DJ guard),
issue #2565 (phantom-action deferral), PR #2631 (already merged),
PR #2647 (post-turn finalizer ADR, open), PR #2687 (recon-note merged).

---

## 0. Что внутри и зачем

Этот документ — артефакт для PR #2647 / issue #2627: визуальная модель
текущего («before») и предлагаемого («after») устройства
`_run_turn.finally`, плюс явный risk register на восемь живых
`🔴 FIX`-веток (F1–F8) из ADR-0021b §2.

Никаких изменений production-кода. Только:

1. Две `mermaid sequenceDiagram` — до/после.
2. Risk register с оценкой likelihood × mitigation × top-3.

Цель — дать ревьюеру и Шифу возможность глазами увидеть, что именно
сохраняется после вынесения финализатора в отдельный класс/модуль
(`PostTurnFinalizer`, ADR-0021b §1.6 вкупе с уже извлечённым
`PostTurnMusicPolicy`, PR-C, коммит `25f6f3af7`).

---

## 1. Sequence diagram — BEFORE

Текущая (`HEAD = wt/t_cee156f5`, dialogue_node.py 3374–3622) картина:
reentry идёт через `_dispatch_turn → _run_turn → guard → _dispatch_turn →
_run_turn`, а 200-строчный `finally` (точнее, ~95 LOC после PR-2631,
3529–3621) держит ВСЮ cleanup-логику inline, включая восемь живых
`🔴 FIX`-патчей и пять mode-флагов.

```mermaid
sequenceDiagram
    autonumber
    actor STT as STT/caller
    participant DT as _dispatch_turn
    participant RT as _run_turn (parent frame)
    participant G as _apply_*_guard
    participant F as _run_turn.finally (inline)
    participant L as _task_lock
    participant DSM as _finalize_turn_dsm

    STT->>DT: _on_stt(text, was_idle=True)
    DT->>DT: bind mode flags (D=0,B=0,A=0,C=0,S=0) +<br/>was_idle gate; check _pending_music_cleanup
    DT->>RT: run_coroutine_threadsafe(_run_turn(...,is_synthetic=False),<br/>self._loop)
    activate RT
    RT->>RT: body: LLM call → tools → result.spoken_text
    Note over RT: body normal-exit ⇒ finally runs;<br/>body raises ⇒ finally STILL runs
    RT->>F: enter finally (line 3529+)
    activate F

    Note over F: 🔴 F1 (line 3525): try/except вокруг<br/>speak-direct + RcutilsLogger,<br/>issue #1278 live 12.08
    F->>L: with self._task_lock:
    L->>F: acquired
    Note over F: 🔴 F2 (lines 3530-3535):<br/>if _run_task is current_task:<br/>_run_task = None<br/>(issue #992 Bug B)
    F->>L: release
    Note over F: 🔴 F3 (lines 3539-3545):<br/>S7 drain — _drain_pending_user_messages()<br/>(issue #968)

    alt tools_called contains stop_music
        Note over F: 🔴 F4 (lines 3547-3552):<br/>_finalize_music_cleanup_policy<br/>arm/disarm _pending_music_cleanup<br/>(issue #935 v3 + #992)
    else was_dj_auto=True
        F->>F: arm _pending_music_cleanup=True (4901)
    end

    Note over F: 🔴 F6 (lines 3577-3584):<br/>pass spoken=result.spoken_text<br/>to _apply_music_guard<br/>(issue #2565 phantom-action)
    F->>G: _apply_music_guard(... was_dj_auto,<br/>spoken, raw_user_command)
    activate G
    alt MusicGuard ⇒ USER_RETRY (issue #1204 / live 30.08)
        G->>G: _mark_retry_dispatched(True) (4581)<br/>_consume_synthetic_retry (4608, #1881)
        Note over G: 🔴 F5 (lines 3561-3573):<br/>defer DIALOGUE_END if music_retry_dispatched<br/>(issue #1204 DJ incident 13.08)
        G-->>F: return music_retry_dispatched=True
        G->>DT: _dispatch_turn(prompt, was_idle=False,<br/>is_synthetic=True, raw_user_command=user_input)
        activate DT
        DT->>RT: schedule child _run_turn on self._loop
        deactivate DT
        Note over RT: child frame resets<br/>_retry_dispatched_in_turn=False (line 3430);<br/>inherits _run_task slot,<br/>_synthetic_retries_left decremented
    else ToolGuard fires
        Note over F: 🔴 F7 (lines 3586-3603):<br/>_apply_tool_skipped_guard<br/>1 critical retry<br/>(issue #1777 / #1762)
        F->>G: _apply_tool_skipped_guard(...)
        G-->>F: return tool_retry_dispatched=True
    else no guard fired
        G-->>F: return *_retry_dispatched=False
    end
    deactivate G

    Note over F: 🔴 F8 (lines 3604-3615):<br/>_finalize_turn_dsm predicate:<br/>DIALOGUE_END iff NOT<br/>(guard_retry_pending OR music_retry_dispatched<br/>OR pending_queue_dispatched OR tool_retry_dispatched)<br/>(issue #992 Bug D + #968 S7)
    F->>DSM: _finalize_turn_dsm(...)
    DSM-->>F: ok
    deactivate F
    deactivate RT
```

**Caption (читатель должен заметить).** Сейчас финализатор — это
один `try/finally`-блок с inline-сайдом на 95 строк, где 8 живых
incident-fixes (F1–F8) сидят в строгом порядке (slot-release → drain →
music-policy → music-guard → tool-guard → DSM-finalize). Любой reentry
через guard (USER_RETRY) или drain (S7) происходит **синхронно из
finally**: ребёнок планируется на `self._loop`, но `finally` ещё не
закончился → родительский фрейм уже отпустил `_run_task`-слот
(F2-гарантия) и уже декрементнул общий бюджет (`_synthetic_retries_left`,
#1881), но ребёнок стартует с **собственным** per-frame
`_retry_dispatched_in_turn=False` (line 3430) — то есть state leak по
mode-флагам закрыт ADR-0021 PR-D (`turn_kind`), а по `_run_task` —
только lock-ом. Видно, почему F2 важен: если в finally синхронно
переставить `_run_task` (через `_apply_stop_music_deferral`), защита
`if _run_task is asyncio.current_task()` не даёт затоптать свеже-
поставленное значение (issue #992 Bug B). Видно и почему порядок
F3 → F4 → F5-F7 → F8 нельзя менять: drain должен случиться до
music-policy (иначе pending фраза увидит неверный cleanup-arm), а
music-guard читает `result.spoken_text` (F6) — его нельзя отдавать
после того, как DSM уже закрыл DIALOGUE.

---

## 2. Sequence diagram — AFTER

Предложение (issue #2627, PR #2647): вынести всю
3529–3621-логику в `PostTurnFinalizer.finalize(TurnContext) →
list[CleanupAction]`, который получает **снимок** всех per-frame locals
+ shared state и **идемпотентно** возвращает actions; их выполняет
тонкий хвост в `dialogue_node._run_turn.finally`.

```mermaid
sequenceDiagram
    autonumber
    actor STT as STT/caller
    participant DT as _dispatch_turn
    participant RT as _run_turn (parent frame)
    participant TC as TurnContext (snapshot)
    participant FIN as PostTurnFinalizer
    participant MP as PostTurnMusicPolicy<br/>(PR-C, already extracted)
    participant MG as _apply_music_guard
    participant TG as _apply_tool_skipped_guard
    participant DSM as _finalize_turn_dsm

    STT->>DT: _on_stt(text, was_idle=True)
    DT->>DT: bind mode flags + turn_kind (PR-D)
    DT->>RT: run_coroutine_threadsafe(_run_turn(...,<br/>is_synthetic=False), self._loop)
    activate RT
    RT->>RT: body: LLM → tools → result
    Note over RT: finally ONLY:<br/>1) snapshot TurnContext<br/>2) delegate to finalizer<br/>3) execute returned actions in order

    RT->>TC: build(ctx): {result, raw_user_command,<br/>user_input, was_dj_auto, was_idle,<br/>guard_retry_pending (derived),<br/>spoken=result.spoken_text}
    TC-->>RT: immutable snapshot (per-frame)
    RT->>FIN: finalize(ctx) [single entry-point,<br/>idempotent: 2nd call ⇒ no-op]
    activate FIN

    FIN->>FIN: 🔴 F1 wrapper: try/except<br/>speak-direct + logger (preserve #1278)

    Note over FIN: F2 → F3 → F4 → F5-F7 → F8<br/>order preserved 1:1 vs BEFORE;<br/>each step is pure w.r.t. ctx

    FIN->>FIN: slot_release(ctx)<br/>with self._task_lock:<br/>if _run_task is current_task:<br/>_run_task = None (issue #992 Bug B)
    FIN->>FIN: drain_pending_user_messages(ctx)<br/>(issue #968 S7)
    FIN->>MP: decide(tools_called, mode, was_idle,<br/>_pending_music_cleanup) (issue #935 v3 + #992)
    MP-->>FIN: PostTurnActions(arm_cleanup, reason)

    Note over FIN: 🔴 F6: ctx.spoken forwarded<br/>(issue #2565 phantom-action)

    alt MusicGuard ⇒ USER_RETRY (#1204 / live 30.08)
        FIN->>MG: _apply_music_guard(ctx, was_dj_auto, ctx.spoken)
        activate MG
        MG->>MG: _mark_retry_dispatched (4581)<br/>_consume_synthetic_retry (4608, #1881)
        Note over FIN,MG: 🔴 F5: defer DIALOGUE_END<br/>if music_retry_dispatched (#1204)
        MG-->>FIN: {music_retry_dispatched=True}
        FIN->>DT: schedule_retry(prompt, ctx,<br/>is_synthetic=True)
        DT->>RT: schedule child _run_turn on self._loop
    else ToolGuard fires
        FIN->>TG: _apply_tool_skipped_guard(ctx)
        TG-->>FIN: {tool_retry_dispatched=True}
    else no guard fired
        FIN-->>FIN: all retry flags False
    end
    deactivate MG

    Note over FIN: 🔴 F8: DSM predicate unchanged<br/>(issue #992 Bug D + #968 S7):<br/>DIALOGUE_END iff NOT any retry_pending
    FIN->>DSM: _finalize_turn_dsm(ctx, retry_flags)
    DSM-->>FIN: ok

    FIN-->>RT: list[CleanupAction]<br/>(idempotent: re-finalize on same ctx ⇒ [])
    deactivate FIN
    Note over RT: thin finally: apply(actions) in returned order
    deactivate RT
```

**Caption (читатель должен заметить).** После вынесения:

- `TurnContext` — **immutable snapshot** per-frame locals и
  `result.spoken_text`. Никакой `_run_turn` не сможет «нечаянно»
  пере-записать поле после построения (важно: ребёнок строит свой
  собственный `TurnContext`, ADR-0021 PR-D `turn_kind` invariant
  сохраняется).
- `PostTurnFinalizer.finalize(ctx)` — **идемпотентен**: повторный
  вызов на том же `ctx` возвращает `[]`. Это закрывает риск двойного
  cleanup, если кто-то в будущем добавит ещё один `try/finally`-уровень
  (сейчас возможный сценарий — вложенный `try` внутри body, см. issue
  #2565 обсуждение).
- Порядок F1–F8 сохранён **байт-в-байт**: внутри `finalize` —
  `slot_release → drain → music_policy → music_guard → tool_guard →
  dsm_finalize`. Reviewer должен grep'нуть diff и убедиться, что
  ни одна строка не переставлена.
- Разделились «чистая функция решения» (`PostTurnMusicPolicy`,
  PR-C, pure, без ROS) и «применятель» (`PostTurnFinalizer`,
  impure — пишет в shared state и планирует reentry).
  Это и есть основной testability-выигрыш: политику можно
  unit-тестировать без `dialogue_node`, а finalize — интеграционно.

---

## 3. Risk register — F1–F8

Для каждого живого `🔴 FIX`-патча из ADR-0021b §2 (и пяти
дополнительных FIX-маркеров в helper-телах, которые финализатор
транзитивно использует) — оценка риска регрессии при вынесении.

### 3.1 Таблица рисков

| #    | Issue / marker | Site (before → after)                       | Likelihood | Why                                                                                                                | Guarding test / invariant                                                                                                                                                              | Mitigation if it breaks                                                                                                            |
| ---- | -------------- | ------------------------------------------- | :--------: | ------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| **F1** | #1278 live 12.08 | inline try/except → `finalize` F1 wrapper   |   **Low**  | Прямой copy-paste, добавляется только namespace (`finalize`.)                                                      | `test_dialogue_guards.py` (broad), плюс property-test: «RcutilsLogger raise → no exception leak»                                                                                       | Feature-flag `finalizer.extract_f1_wrapper=true`, shadow-run: parallel-call inline + `finalize`, compare outputs                  |
| **F2** | #992 Bug B     | slot-clear inline → `finalize.slot_release` |  **High**  | Самый много-сайтовый lock-acquire; легко потерять `if _run_task is asyncio.current_task()` guard                    | `test_issue_992_dj_mode.py::test_dj_flag_resets_after_run_turn` + новое property: «guard sync-set task → slot_release no-op»                                                          | Shadow-run must include concurrent `stop_music()` injection; если shadow diffs — **revert**, не fix-forward                        |
| **F3** | #968 S7        | inline drain → `finalize.drain_pending_user_messages` | **Med** | Drain сам по себе reentrant (может зашедулить ещё один child) — порядок в finalize критичен, но логика та же         | `test_barge_in_policy.py::test_pending_llm_with_finished_task_dispatches_normally` + новое: «3-level reentrancy не зацикливается»                                                      | Hard-cap на `finalize.recursion_depth=3` (логировать warn); при depth>3 — `kanban_comment` + safe-stop                              |
| **F4** | #935 v3 + #992 | inline arm/disarm → `PostTurnMusicPolicy.decide` | **Med** | Уже вынесено (PR-C `25f6f3af7`), но **зависит** от того, что finalize передаёт корректный `ctx.tools_called`       | `test_post_turn_music_policy.py` (387 LOC, pure) + integration `test_issue_992_batch_cleanup.py::test_stop_music_when_already_pending_is_ignored`                                       | `policy.decide()` pure → shadow compare inline vs new path; при diff — `finalize.use_inline_f4_legacy=true`                         |
| **F5** | #1204 DJ       | inline defer-DIALOGUE → `finalize.music_guard_retry`  | **High**  | Subtle: deferral зависит от возвращаемого значения guard, не от `_retry_dispatched_in_turn` (см. ADR-0021b §4 invariant #2) | `test_dialogue_retry_flag_wiring.py` + новое: «DJ guard fires ⇒ DSM sees `music_retry_dispatched=True` even if `_retry_dispatched_in_turn` reset»                                  | Shadow-run с recorded incident DJ 13.08 (e2e replay); **revert-on-first-diff**                                                     |
| **F6** | #2565 phantom  | inline `spoken=result.spoken_text` → `ctx.spoken` |   **Low**  | Snapshot механизм делает передачу тривиальной, но если `result` — `None` (exception body), `ctx.spoken` будет `None` | `test_issue_2565_phantom_action_defers_stop.py::test_real_stop_with_stop_music_tool_still_skips` + property: «ctx.spoken is None ⇒ guard sees spoken=None, не падает»                   | Default в finalize: `ctx.spoken = result.spoken_text if result else ""`; shadow compare                                                |
| **F7** | #1777 / #1762  | inline tool-guard → `finalize.tool_guard` |  **Med**  | Tool-guard читает `raw_user_command` derived, не сам mode-флаг; легко промахнуться с передачей `raw_user_command or user_input` (line 3601) | `test_dialogue_guards.py` (broad tool-skip) + новое: «empty user_input + raw_user_command set ⇒ guard sees raw»                                                                       | Shadow-run на recorded e2e rn02 (live 30.08, два синтетических ретрая); **revert-on-first-diff**                                   |
| **F8** | #992 D + #968 S7 | inline DSM predicate → `finalize.dsm_finalize` |  **High**  | DSM predicate — самая multi-source-input точка: 4 OR-условия (`guard_retry_pending`, `music_retry_dispatched`, `pending_queue_dispatched`, `tool_retry_dispatched`); легко потерять один флаг при передаче | `test_dialogue_retry_flag_wiring.py` (integration: все 4 комбинации) + новые property-тесты на cartesian (2^4 = 16 cases, минус unreachable из ADR-0021b §3.2 = ~10 reachable)                | Shadow-run с diff-tolerance=0; **никакого «fix-forward»** — bug в DIALOGUE_END-тайминге виден только на живом e2e, не на pytest    |
| **F-add-1** | live 30.08 phantom | inline `FORCE_STOP` ветка в `_apply_music_guard` (5331-5346) |  **Med**  | Финализатор вызывает guard, но не его внутренности; если guard изменится, **transitively** сломается finalize           | `test_issue_2565_phantom_action_defers_stop.py::test_live_oakenfold_phantom_action_defers_force_stop`                                                                                  | Pin guard версию в `finalize` docstring; при изменении guard — `kanban_comment` в issue #2627 перед merge                          |
| **F-add-2** | live 30.08 rn02 «два ретрая» | inline early-return в `_apply_music_guard` на `_retry_dispatched_in_turn` (5229-5242) |  **Med**  | Аналогично F-add-1 — внутренняя инвариантность guard, транзитивно видна из finalize                                  | `test_issue_992_dj_mode.py::test_dj_auto_without_music_triggers_synchronous_retry` + записанный rn02-реплей                                                                          | Тот же, что F-add-1                                                                                                                |
| **F-add-3** | #1101 «[Говорит имя]» | `_apply_speaker_identity` (2799-2804) |   **Low**  | Speaker identity вызывается ДО finally, финализатор не трогает                                                       | `test_dialogue_speaker.py` (broad)                                                                                                                                                    | n/a — out of refactor scope                                                                                                        |
| **F-add-4** | live 12.08 мусорные имена | speaker (2834-2840, 2845-2854) |   **Low**  | Тот же, что F-add-3                                                                                                  | `test_dialogue_speaker.py`                                                                                                                                                            | n/a                                                                                                                                |
| **F-add-5** | #992 live 09:09 «speak_text(прелюдия)» | `_dispatch_turn` (2609) deferral hook |   **Low**  | Entry-point, не finally                                                                                             | `test_issue_992_dj_mode.py::test_dispatch_turn_user_path_still_publishes_new_dialogue_cleanup`                                                                                       | n/a                                                                                                                                |
| **F-add-6** | #992 live 08:55 «reason=new_dialogue ТОЛЬКО при новом диалоге из IDLE» | `_dispatch_turn` (2748-2757) entry-point gate |   **Low**  | Тот же                                                                                                              | `test_issue_992_dj_mode.py::test_dj_dispatch_does_not_publish_new_dialogue_cleanup`                                                                                                  | n/a                                                                                                                                |
| **F-add-7** | live 02.09 «во время сочинения LLM говорит» | batch_complete bookkeeping (6146) |   **Low**  | Out of finally scope                                                                                                | (нет прямого теста, мониторинг через e2e)                                                                                                                                              | n/a                                                                                                                                |

### 3.2 Top-3 highest-risk branches

1. **F2 (slot-release, #992 Bug B) — High.** Самый locking-heavy
   участок, и его регрессия **не видна на pytest**: баг проявляется
   только при concurrent `_apply_stop_music_deferral`-вызове внутри
   finally. Mitigation: shadow-run обязателен, **revert-on-first-diff**,
   плюс property-test «guard sync-set task → slot_release no-op».

2. **F5 (DJ guard defer DIALOGUE_END, #1204) — High.** Subtle
   зависимость от возвращаемого значения `_apply_music_guard`, а не
   от `_retry_dispatched_in_turn` (ADR-0021b §4 invariant #2). Если
   finalize передаст только `ctx.retry_flags` и забудет
   `music_retry_dispatched` как отдельный return-value — DSM
   закроет DIALOGUE до retry. Mitigation: shadow e2e replay
   incident DJ 13.08, **revert-on-first-diff**.

3. **F8 (DSM predicate, #992 D + #968 S7) — High.** Самая
   multi-source-input точка — 4 OR-условия. Одна пропущенная
   переменная при передаче → DIALOGUE_END тайминг сломается, и это
   видно только на живом e2e (pytest с моками не ловит). Mitigation:
   cartesian property-test на 2^4 комбинаций (reachable = ~10),
   shadow-run с diff-tolerance=0.

### 3.3 Стратегия раскатки (issue #2627 ↔ PR #2647 ↔ этому ADR)

Поскольку #992 Bug B (F2), #1204 (F5) и #992 Bug D (F8) — это High,
а pytest их не ловит, рекомендую phased-rollout:

- **Phase 1 (PR-A, B):** извлечь `TurnContext` snapshot и переписать
  `finally` как «snapshot → finalize → apply», **не трогая** семантику
  F1–F8. Pure refactor. Diff должен быть 1:1 по side-effects. После
  merge — pytest + 1 e2e voice replay (любой свежий, не обязательно
  incident).
- **Phase 2 (PR-C, уже сделано `25f6f3af7`):** `PostTurnMusicPolicy`
  pure decision (387 LOC unit-тестов). Тут риск Low, тесты ловят.
- **Phase 3 (PR-D, уже сделано `e2a6c1af3`):** `TurnKind` enum +
  `_classify_turn_kind`. Тоже Low.
- **Phase 4 (issue #2627 PR, this ADR proposes):** полное вынесение
  `PostTurnFinalizer`. Тут High — **shadow-run обязателен** на
  rec-recorded incidents DJ 13.08 + rn02 live 30.08 + e2e от 02.09.
  Diff-tolerance=0; **revert-on-first-diff**, никакого fix-forward.
- **Phase 5 (post-merge):** property-тесты на cartesian для F8 +
  «guard sync-set task» для F2 + DJ-replay для F5. Это test-якоря
  для t_e8b6676a (test-plan).

---

## 4. Открытые вопросы

- **OQ-1:** Кто владеет `PostTurnFinalizer` — `dialogue_node.py`
  (расширяем существующий класс) или новый модуль
  `core/post_turn_finalizer.py`? Рекомендую **новый модуль**:
  mirroring `core/post_turn_music_policy.py`, pure-decision parts
  отделяем от impure-applier (testing).
- **OQ-2:** Нужен ли `FinalizerMetrics` (latency, action_count,
  reentry_depth) сразу, или отложим в Phase 5? Рекомендую
  **отложить** — KISS, добавим когда попросит observability.
- **OQ-3:** Идемпотентность `finalize(ctx)` — это контракт или
  свойство реализации? Рекомендую **контракт** (assert в debug),
  потому что без него Phase 4 shadow-run не сможет доказать «no
  double-cleanup». Шифу — на ваше усмотрение.

---

## 5. Артефакты

- Этот ADR: `docs/adr/0021c-finalizer-extraction-sequence-diagrams-and-risk.md`
- Входные данные: `docs/adr/0021b-recon-run-turn-state-locks-and-fixes.md`
  (recon-note, коммит `9fbbcac92`, merged PR #2687)
- Тестовые якоря: перечислены в §3.1 по каждой ветке; полный список —
  в ADR-0021b §5.