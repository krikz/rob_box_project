# ADR-0021b — `_MusicTurnFinalizer` API, `TurnMode` consolidation, state ownership, migration outline

**Дата:** 2026-09-16
**Статус:** proposed (раздел §0 требует разрешения конфликта с ADR-0021a, см. ниже)
**Автор:** architect (kanban `t_974eda35`)
**Родительский ADR:** ADR-0021, ADR-0021a (PR #2647, OPEN), ADR-0021-r1
**Связанные:** issue #2627, recon `docs/adr/0021b-recon-run-turn-state-locks-and-fixes.md` (kanban `t_cee156f5`, ✅ done), PR #2631 (merged), PR #2640 (open), `core/turn.py` (`TurnContext`/`TurnState`/`TurnGuards`), ADR-0013, ADR-0018

---

## §0. Конфликт с ADR-0021a (нужно решение товарища Шифу)

Карточка `t_974eda35` (этот ADR) была сгенерирована `auto-decomposer` 2026-09-16
11:23 из `t_adc73325` со старым текстом «sketch the `_MusicTurnFinalizer` class
proposal». Параллельная карточка `t_adc73325` уже **выполнена** воркером в
соседнем worktree — коммиты `dec87abf6` и `a5375d847` (wip-коммиты) и ADR
`docs/adr/0021a-post-turn-music-finalizer.md` (PR #2647, branch
`z-architect/2627-adr-post-turn-music-finalizer`, **OPEN**).

ADR-0021a **явно отвергает** класс `_MusicTurnFinalizer` и выбирает альтернативу:
`TurnKind` enum (PR-D, `e2a6c1af3`/`a693a04cf`) + `POLICY_BY_KIND` dict-of-callables
(прецедент `core/turn.py:783+` `TurnGuards.default_for_dialogue_node`).

Цитата ADR-0021a §"Альтернативы, которые мы НЕ выбрали / ❌ Класс `_MusicTurnFinalizer`":

> Body карточки t_adc73325 предлагает «`_MusicTurnFinalizer` класс (или
> эквивалент)». Я **отвергаю** это. Причины:
> 1. **Машина состояний уже вынесена** в `PostTurnMusicPolicy` (PR-C, PR #2640).
>    Финальный шаг — это **диспетчирование**, а не новая машина состояний.
> 2. **KISS.** Ещё один класс с `__init__(self, dialogue_node, turn_kind,
>    state_snapshot)` — это лишний dependency injection ceremony для 4-х
>    веток.
> 3. **Тестируемость не страдает.** `POLICY_BY_KIND` — это data, чисто
>    тестируется как `core/turn_kind.py::test_policy_table`.
> 4. **Прецедент.** `core/turn.py:783+` уже использует паттерн «dict-of-callables,
>    населённый в конструкторе». Новый класс — это лишняя сущность без выигрыша.

Этот документ **не** пытается пересмотреть ADR-0021a. Он **формально выполняет**
техзадание карточки `t_974eda35` (проектирует `_MusicTurnFinalizer` class, как
буквально сказано в body), но в §0 фиксирует конфликт и предлагает Шифу
**выбрать один из трёх сценариев** перед merge:

### Сценарий A (рекомендую) — закрыть `t_974eda35` как DUPLICATE, принять ADR-0021a

- `t_974eda35` archive, его текст перенесён в ADR-0021a §"Сценарии отвергнутые".
- ADR-0021a остаётся единственным дизайн-ADR для issue #2627.
- Воркеру на PR-E остаётся: реализовать `POLICY_BY_KIND`, расщепить 3 горячие
  точки в `_run_turn` (budget / exception-fallback / DIALOGUE_END-gate), вытащить
  helper'ы до CC≤15.
- **Плюс:** не плодим два конкурирующих дизайна; ADR-0021a уже detail-уровневый
  (658 строк), ничего из этого документа не теряется.
- **Минус:** §A-D этого документа остаются невостребованными (но они могут быть
  полезны как **отправная точка** для ADR-0021a, если Шифу захочет класс).

### Сценарий B (частичный merge) — оставить ADR-0021a, но добавить этот документ как ADR-0021c, описывающий `_MusicTurnFinalizer` для **другого слоя**

- ADR-0021a остаётся «как есть» (TurnKind enum + POLICY_BY_KIND для `_run_turn`
  CC≤15).
- Этот ADR-0021b становится дизайн-документом для **отдельного** класса
  `_MusicTurnFinalizer`, который **оборачивает** `PostTurnMusicPolicy` +
  `_apply_music_guard` + `_drain_pending_user_messages` как единое целое для
  тестов/диагностики/симуляций (но не для `_run_turn` — там остаётся
  inline-вызов).
- **Плюс:** оба дизайна живут; класс даёт testing seam (DI mock), а TurnKind
  даёт production efficiency.
- **Минус:** два дизайн-документа на одну задачу; для воркера на PR-E это
  источник несогласованности. **Слабо рекомендую** — увеличивает cognitive load.

### Сценарий C — переписать ADR-0021a под класс `_MusicTurnFinalizer`

- Этот документ становится ADR-0021a, класс `_MusicTurnFinalizer` —
  production-объект для `_run_turn`.
- ADR-0021a PR #2647 закрывается с комментарием «superseded by ADR-0021b»,
  воркеру на PR-E отправляется обновлённый дизайн.
- **Плюс:** один source of truth.
- **Минус:** ADR-0021a уже в процессе review (`status=proposed`, branch
  `z-architect/2627-adr-post-turn-music-finalizer`); откат = репутационный
  урон для уже-выполненной работы. KISS-аргумент ADR-0021a («класс — лишняя
  сущность для 4 веток») никуда не делся. **Не рекомендую.**

**Я (architect) рекомендую Сценарий A.** Решение за товарищем Шифу.

---

Нижеследующие разделы A-D — это **формальное выполнение** техзадания карточки
`t_974eda35` независимо от того, какой сценарий выберет Шифу. Если Сценарий A —
эти разделы идут в архив как evidence «мы рассмотрели класс, но отвергли» (см.
ADR-0021a §"Альтернативы"). Если B/C — они становятся дизайн-источником для
воркера на PR-E.

Все file:line ниже — `src/rob_box_voice/rob_box_voice/dialogue_node.py` (HEAD =
`develop` + PR #2631, post-merge `6c7496bac`), если не указано иное. Radon-факт
(`radon cc -s -n C -a src/rob_box_voice/rob_box_voice/dialogue_node.py`):

```
M 5921:4 DialogueNode._handle_result   - F (85)   ← главный долг ADR-0021 R1
M 5185:4 DialogueNode._apply_music_guard - C (14)
M 3374:4 DialogueNode._run_turn         - C (13)   ← после PR-2631
```

---

## A. `_MusicTurnFinalizer` class proposal

### A.1 Назначение и границы

Класс `_MusicTurnFinalizer` инкапсулирует **всю work, которую `_run_turn` выполняет
в finally-блоке** (lines 3529–3622): slot release → drain → music cleanup
policy → music guard → tool-skipped guard → DSM finalize → state publish. Это
**владелец финальной логики кадра**, не владелец музыкального state — последний
остаётся в `PostTurnMusicPolicy` (PR #2640, commit `6e41792b3`).

```
┌──────────────────────────────────────────────────────────────────┐
│                    _run_turn.finally (today)                     │
│                                                                  │
│  ┌──────────────────────────┐    ┌─────────────────────────┐     │
│  │ _MusicTurnFinalizer      │ ──►│ PostTurnMusicPolicy     │     │
│  │ (this proposal)          │    │ (PR #2640, existing)    │     │
│  │                          │    │                         │     │
│  │ • run_task release       │    │ • decide + execute      │     │
│  │ • drain pending          │    │ • arm/disarm            │     │
│  │ • music-guard            │    │   _pending_music_cleanup│     │
│  │ • tool-guard             │    └─────────────────────────┘     │
│  │ • DSM finalize           │                                    │
│  │ • publish state          │                                    │
│  └──────────────────────────┘                                    │
└──────────────────────────────────────────────────────────────────┘
```

Граница ответственности:
- `_MusicTurnFinalizer` **владеет** sequence/callsite finally-блока и
  orchestration guards' outputs (`music_retry_dispatched`, `tool_retry_dispatched`).
- `PostTurnMusicPolicy` **владеет** music state machine
  (`_pending_music_cleanup`, `_track_mode_music_active`, `_active_batches`).
- `TurnGuards` (`core/turn.py:248`) **владеет** guard order + budget (вынесено
  в core/ в issue #1881).

### A.2 Inputs: `TurnContext` snapshot (frozen dataclass)

Текущий `TurnContext` уже существует в `core/turn.py:106`:

```python
@dataclass(frozen=True)
class TurnContext:
    """Inputs that are NOT the LLM's reply."""
    user_input: str
    is_dj_auto: bool = False
    speech_id: Optional[str] = None
```

Для финализатора нужен **расширенный snapshot**. Предлагаю
**не ломать** существующий `TurnContext`, а ввести новый dataclass
`FinalizerContext` (или, проще, добавить optional поля в `TurnContext` через
`field(default_factory=…)` — обсудим в §A.2.3).

#### A.2.1 Финальная форма `FinalizerContext`

```python
@dataclass(frozen=True)
class FinalizerContext:
    """Immutable snapshot of everything _run_turn.finally needs.

    Required fields (no default):
      mode: TurnMode                 — classification result from §B
      retries_left: int              — copy of self._synthetic_retries_left
      run_task: Optional[asyncio.Task] — current task handle (for slot clear)
      task_lock: threading.Lock      — shared lock for _run_task mutation
      dialogue_node: DialogueNode    — back-reference for self.* helpers
                                       (publish_state, _dsm, etc.)

    Optional fields (have default):
      result: Optional[Reply]        — LLM result, None if cancelled/error
      raw_user_command: Optional[str] — pre-override user_input (DJ mode)
      was_dj_auto: bool = False      — mirror of is_dj_auto (per-frame local)
      speech_id: Optional[str] = None — for log correlation
      drained_user_messages: Tuple[str, ...] = ()  — populated by drain step
    """

    # Required
    mode: "TurnMode"
    retries_left: int
    run_task: Optional[asyncio.Task]
    task_lock: threading.Lock
    dialogue_node: "DialogueNode"

    # Optional (per-frame locals + post-drain data)
    result: Optional["Reply"] = None
    raw_user_command: Optional[str] = None
    was_dj_auto: bool = False
    speech_id: Optional[str] = None
    drained_user_messages: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        # ADR-0018 + R2 (State SSoT): validation at construction.
        if self.retries_left < 0:
            raise ValueError(
                f"FinalizerContext: retries_left must be >= 0, "
                f"got {self.retries_left!r}"
            )
        # The 5 flag → TurnMode collapse (§B) makes mode a single source;
        # construction-time check that mode matches retries_left invariants.
        if self.mode.requires_budget() and self.retries_left == 0:
            # Not a fatal — guards degrade RETRY→ACCEPT — but log a warning
            # so silent mode/budget mismatch is visible.
            pass  # soft check; non-raising
```

#### A.2.2 Кто конструирует `FinalizerContext`

`_run_turn` строит `FinalizerContext` ровно один раз — на входе в `finally`,
из уже-собранных per-frame locals и shared state:

```python
# In _run_turn.finally (line ~3529):
ctx = FinalizerContext(
    mode=turn_mode,                              # from §B.3 classification
    retries_left=self._synthetic_retries_left,   # current budget snapshot
    run_task=asyncio.current_task(),             # for slot-clear identity check
    task_lock=self._task_lock,                   # shared infrastructure lock
    dialogue_node=self,                          # back-reference
    result=result,                               # LLM Reply or None
    raw_user_command=raw_user_command,
    was_dj_auto=was_dj_auto,
    speech_id=getattr(result, "speech_id", None),
)
finalizer = _MusicTurnFinalizer(ctx)
finalizer.run()
```

**Один момент конструирования → ноль мутаций после.** ADR-0021 R2 (State SSoT)
требует, чтобы shared state читалось атомарным snapshot'ом, не через 5
отдельных `self._X` доступов в finally. `FinalizerContext` — это именно snapshot.

#### A.2.3 Альтернатива: расширить `TurnContext` вместо нового dataclass

Прецедент `core/turn.py:106` — `TurnContext` уже frozen dataclass с `user_input`,
`is_dj_auto`, `speech_id`. Можно добавить optional поля (`retries_left`,
`run_task`, etc.) прямо туда. **Минус**: `TurnContext` сейчас потребляется
`TurnGuards.evaluate(reply, turn, state)` (line 267) — добавление `run_task`
и `task_lock` в guard's TurnContext нарушит single-purpose separation
(`TurnContext` = про что LLM говорит; `FinalizerContext` = про cleanup).

**Рекомендация:** отдельный `FinalizerContext`. Это явный layering signal: guards
видят `TurnContext`, finalizer видит `FinalizerContext`, оба read-only.

### A.3 Конструктор и method surface

```python
class _MusicTurnFinalizer:
    """Owner of _run_turn.finally orchestration. See ADR-0021b §A.

    Construct once per _run_turn frame; call :meth:`run` exactly once.
    Methods are stateful in the sense that intermediate results are cached
    on the instance for logging/diagnostics, but the instance itself does
    NOT mutate ``dialogue_node`` state outside of the documented mutation
    sites. This is a class for orchestration, not state ownership.
    """

    def __init__(self, ctx: FinalizerContext) -> None:
        self._ctx = ctx                              # frozen snapshot
        self._node = ctx.dialogue_node                # shortcut
        self._logger = ctx.dialogue_node.get_logger()
        self._actions = _FinalizerActions.empty()     # result accumulator
        self._drained_messages: List[str] = []        # populated by drain
        self._music_retry_dispatched = False
        self._tool_retry_dispatched = False
        self._pending_queue_dispatched = False
        self._guard_retry_pending = False

    # Public surface ------------------------------------------------------

    def run(self) -> FinalizerResult:
        """Execute the full finally sequence. Idempotent (see §A.5)."""
        if self._actions.is_terminal():
            return self._actions.to_result()  # idempotency guard
        try:
            self._release_run_task_slot()       # §D line 1, F2 in recon
            self._drain_pending_user_messages()# §D line 2, F3 (issue #968 S7)
            self._apply_music_cleanup_policy()  # §D line 3, F4
            self._apply_music_guard()           # §D line 4, F5/F6
            self._apply_tool_skipped_guard()    # §D line 5, F7
            self._finalize_turn_dsm()           # §D line 6, F8
            self._publish_state()               # §D line 7
            self._actions.mark_complete()
        except asyncio.CancelledError:
            # CancelledError в finally — это **barge-in во время cleanup**.
            # Не падать (live 12.08 FIX, #1278), но и не продолжать —
            # оставшиеся шаги выполнятся в следующем кадре (slot уже
            # очищен, всё что после — best-effort).
            self._actions.mark_partial(reason="cancelled_in_finally")
            self._logger.warn(
                "⚠️ [finalizer] CancelledError in finally — slot released, "
                "remaining steps skipped (best-effort)"
            )
        except Exception as exc:                # noqa: BLE001 — см. §A.4
            self._actions.mark_partial(reason=f"exception: {exc!r}")
            self._logger.error(
                f"❌ [finalizer] exception in finally: {exc!r}",
                exc_info=True,
            )
            # ВАЖНО: НЕ swallow — log + mark partial. Caller (_run_turn)
            # завершается нормально, потому что мы в finally.
        return self._actions.to_result()

    # Private surface (one method per recon F1-F8) -------------------------

    def _release_run_task_slot(self) -> None:
        """F2 — Issue #992 Bug B. Lock-protected identity check."""
        with self._ctx.task_lock:
            if self._ctx.run_task is self._node._run_task:
                self._node._run_task = None
                self._actions.slot_released = True

    def _drain_pending_user_messages(self) -> None:
        """F3 — Issue #968 S7. Multiple queued phrases → ONE follow-up."""
        # Delegate to existing self._drain_pending_user_messages.
        # Capture the drained texts into _drained_messages for diagnostics.
        # Returns bool; we mirror it to _pending_queue_dispatched.
        ...

    def _apply_music_cleanup_policy(self) -> None:
        """F4 — Issue #935 v3, #992 Bug C. Delegate to existing
        _finalize_music_cleanup_policy(self, result, was_dj_auto, ...).
        Reads self._ctx.result and self._ctx.was_dj_auto.
        """
        ...

    def _apply_music_guard(self) -> None:
        """F5 + F6 — Issue #1204 + #2565. Captures music_retry_dispatched."""
        ...

    def _apply_tool_skipped_guard(self) -> None:
        """F7 — Issue #1777 / #1762. Captures tool_retry_dispatched."""
        ...

    def _finalize_turn_dsm(self) -> None:
        """F8 — Issue #992 Bug D, #968 S7. Suppress DIALOGUE_END if any
        retry is pending. Reads the 4 *retry_dispatched flags set above.
        """
        ...

    def _publish_state(self) -> None:
        """Issue #1160, #918 — always publish, even if everything else failed."""
        ...
```

### A.4 Outputs: `FinalizerResult`

```python
@dataclass(frozen=True)
class FinalizerResult:
    """What the finalizer did. Diagnostics + acceptance-test seam."""
    slot_released: bool
    drained_user_messages: Tuple[str, ...]
    music_retry_dispatched: bool
    tool_retry_dispatched: bool
    pending_queue_dispatched: bool
    guard_retry_pending: bool
    music_actions_taken: Tuple[str, ...]      # # ("arm_pending", "force_stop", …)
    dsm_finalized: bool                        # # whether DIALOGUE_END fired
    state_published: bool
    completion: Literal["complete", "partial", "skipped"]
    partial_reason: Optional[str] = None       # # populated if completion != complete
```

`completion ∈ {"complete", "partial", "skipped"}` — это явный ack для тестов:
"complete" = всё прошло; "partial" = что-то упало, но slot released; "skipped" =
двойной вызов (idempotency guard, см. §A.5).

### A.4.1 Diagnostics surface (опционально, для тестов)

`FinalizerResult` — это всё, что нужно юнит-тестам для проверки side-effects:
- `slot_released == True` после первого вызова;
- `drained_user_messages` содержит склеенный текст S7 (если drain сработал);
- `music_retry_dispatched` совпадает с ожиданием по result.tools_called;
- `completion == "partial"` после `pytest.mock`-inject CancelledError.

### A.5 Error semantics

#### A.5.1 Частичные отказы

`_run_turn.finally` обязан завершиться **всегда** — это контракт Python
`try/finally`. Finalizer **не raise'ит** (кроме programmer error из §A.5.2);
вместо этого маркирует `FinalizerResult.completion = "partial"` и пишет в
`self._logger`. Это **не** silent degradation (ADR-0018, capability-honest
TTS-extension-points): каждый partial имеет `partial_reason`, который попадает
в `docker logs` для post-mortem.

#### A.5.2 Что raise'ит (programmer errors)

- `FinalizerContext.__post_init__` raise'ит `ValueError` на negative
  `retries_left` — это invariant violation, должен быть виден на старте.
- Конструктор `_MusicTurnFinalizer.__init__` raise'ит `TypeError` если
  `ctx.dialogue_node` не имеет `get_logger()` / `_run_task` /
  `_task_lock` — programmer error в вызывающем коде, не runtime-condition.
- **Не** raise'ит на runtime-исключения внутри `run()` (см. §A.5.1).

#### A.5.3 Logging hooks

| Site | Level | Что пишем |
|---|---|---|
| `__init__` | DEBUG | `"[finalizer] ctx: mode={mode}, retries_left={n}, was_dj_auto={b}"` |
| `_release_run_task_slot` success | DEBUG | `"[finalizer] slot released"` |
| `_release_run_task_slot` skipped (identity miss) | DEBUG | `"[finalizer] slot NOT released (replaced by guard dispatch)"` |
| `_drain_pending_user_messages` success | INFO | `"📤 [S7] drained N messages"` (existing, see 7089) |
| `_apply_music_guard` retry dispatched | INFO | `"🔁 [music guard] retry dispatched, prompt={…}"` |
| `_apply_tool_skipped_guard` retry dispatched | INFO | `"🔁 [tool guard] retry dispatched"` |
| `_finalize_turn_dsm` DIALOGUE_END fired | DEBUG | `"[finalizer] DIALOGUE_END"` |
| `_finalize_turn_dsm` DIALOGUE_END suppressed | DEBUG | `"[finalizer] DIALOGUE_END suppressed (retry pending)"` |
| `run()` exception | ERROR | `"❌ [finalizer] exception in finally: {exc!r}"` (with traceback) |
| `run()` CancelledError | WARN | `"⚠️ [finalizer] cancelled in finally"` |
| `run()` complete | DEBUG | `"[finalizer] complete: {result}"` |

### A.6 Idempotency contract

`run()` **безопасен** для повторного вызова и **безопасен** в `finally`-блоке
(даже если предыдущая попытка упала посередине). Доказательство:

#### A.6.1 Idempotency: повторный вызов `run()` возвращает тот же `FinalizerResult`

```python
# Внутри _actions:
def is_terminal(self) -> bool:
    return self.completion != "in_progress"

def mark_complete(self) -> None:
    if self.completion == "in_progress":
        self.completion = "complete"

# В run():
if self._actions.is_terminal():
    return self._actions.to_result()  # ← idempotency guard
```

**Доказательство:** мутации каждого step'а либо:
1. **Защищены lock'ом** (`_run_task` slot release — `with self._ctx.task_lock`).
2. **Идемпотентны по природе** (`_publish_state` — pure read из self._dsm + write
   в ROS topic, повторный write идемпотентен).
3. **Опускают себя через guard'ы** (`_apply_music_guard` — внутри guard'а есть
   early-return при `_retry_dispatched_in_turn == True`, см. 5229–5242).

Три категории вместе = повторный `run()` не делает новой работы.

#### A.6.2 Idempotency: `finally`-безопасность

`_run_turn.finally` уже вызывает helper'ы (`_finalize_music_cleanup_policy`,
`_apply_music_guard`, …) которые **сами по себе** идемпотентны — это задокументировано
в issue #1881 (budget decrement), #992 Bug B (slot identity check). Finalizer
**не вводит новых неидемпотентных операций** — он только sequence'ит уже
существующие helper'ы.

Конкретный non-idempotent risk был бы в `_drain_pending_user_messages`:
если он сработает дважды, он отправит **два** follow-up turn'а вместо одного. **Защита:**
finalizer'ы **обнуляют** очередь в `self._node._pending_user_messages` после
drain (это уже делается в `_drain_pending_user_messages:7083`), и второй вызов
увидит пустую очередь.

#### A.6.3 Re-entry: два `_run_turn` frame'а завершаются out-of-order

**Scenario:** Frame #1 (parent) и frame #2 (child, dispatched by music guard в
`frame #1.finally`) одновременно входят в `finally`. Frame #2 первым завершает
свой finally и зовёт `finalizer #2.run()`; затем frame #1 доходит до своего
finally и зовёт `finalizer #1.run()`.

**Что может пойти не так:**

1. **Slot-release race** — frame #1 slot release стирает `self._run_task`,
   frame #2 (который ещё не начал finally) увидит `None`. **Защита есть:**
   `_release_run_task_slot` сравнивает `run_task is self._node._run_task` —
   frame #2 не матчит, потому что frame #1 уже стёр. Это **exactly** Bug B
   защита (issue #992). Finalizer её сохраняет.

2. **Drain double-dispatch** — frame #1 drain отправляет follow-up (который сам
   станет frame #3); frame #2 (если он в finally одновременно) тоже может drain'ить.
   **Защита:** `self._node._pending_user_messages` — общий deque, lock-free
   (`deque.append/popleft` atomic в CPython). Один из двух frame'ов увидит
   пустую очередь и no-op.

3. **DIALOGUE_END double-fire** — оба frame'а проверяют «все ли retry-dispatched
   False» и пытаются закрыть DIALOGUE. **Защита:** `_finalize_turn_dsm` уже
   проверяет `self._dsm.current_state == DialogueStateKind.DIALOGUE` (line
   3618); после первого `DIALOGUE_END` state становится `IDLE`, второй no-op.

4. **Music guard повторный fire на child result** — frame #2's `_apply_music_guard`
   может сработать на свой result и отправить frame #3. Это **нормальная
   семантика** recursion (issue #968 S7 не запрещает цепочку depth>2, см.
   recon §4.5 "Cross-frame correctness invariants #5").

**Вывод:** finalizer **сохраняет все существующие идемпотентностные защиты** +
добавляет одну новую (`_actions.is_terminal()` guard для повторного вызова).
Сохраняет семантику recursion.

---

## B. `TurnMode` consolidation

### B.1 5 booleans → enum

Текущие 5 параметров `_run_turn` (lines 3378–3387):

```python
is_dj_auto: bool = False
is_babble_retry: bool = False
is_action_claim_retry: bool = False
is_code_retry: bool = False
is_synthetic: bool = False
```

Из них 32 = 2⁵ комбинаций, **5 reachable** (recon §3.2):

| # | D | B | A | C | S | Reachable | Site |
|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 0 | 0 | ✅ | `_on_stt` / `_dispatch_turn` user path |
| 3 | 0 | 0 | 0 | 1 | 1 | ✅ | `_check_embedded_renardo_code_and_retry` |
| 5 | 0 | 0 | 1 | 0 | 1 | ✅ | `_check_unbacked_action_claim_and_retry` |
| 9 | 0 | 1 | 0 | 0 | 1 | ✅ | `_check_babble_and_retry:3984-3990` |
| 16 | 1 | 0 | 0 | 0 | 0 | ✅ | DJ-tick → `_dispatch_dj_turn` |
| — | — | — | — | — | — | ❌ | 27 unreachable |

### B.2 Предлагаемая форма: **enum + invariants**, не dataclass

```python
class TurnMode(str, Enum):
    """The 5 reachable turn modes after consolidating the 5 bool flags.

    Naming follows the recon's reachability analysis (0021b §3.2):
    out of 32 = 2⁵ combinations, only 5 are constructed by code today.
    Construction-time validation enforces reachability.
    """

    USER              = "user"             # D=0,B=0,A=0,C=0,S=0 — default
    CODE_RETRY        = "code_retry"       # D=0,B=0,A=0,C=1,S=1 — Renardo
    ACTION_CLAIM      = "action_claim"    # D=0,B=0,A=1,C=0,S=1 — unbacked claim
    BABBLE_RETRY      = "babble_retry"     # D=0,B=1,A=0,C=0,S=1
    DJ_AUTO           = "dj_auto"          # D=1,B=0,A=0,C=0,S=0

    @classmethod
    def from_flags(
        cls,
        *,
        is_dj_auto: bool,
        is_babble_retry: bool,
        is_action_claim_retry: bool,
        is_code_retry: bool,
        is_synthetic: bool,
    ) -> "TurnMode":
        """Classify the 5 flags into one mode. Raise on unreachable combo.

        Validation rules:
          1. Exactly one of (is_babble_retry, is_action_claim_retry,
             is_code_retry) may be True. Multiple = ValueError.
          2. is_dj_auto=True AND is_synthetic=True = ValueError (DJ ticks
             are never synthetic — see recon §3.2 #16).
          3. is_synthetic=True requires one of babble/action/code retry
             OR is_dj_auto=False (because S=1 without retry flag was
             #1 in recon — never constructed).
        """
        retry_count = sum([
            is_babble_retry,
            is_action_claim_retry,
            is_code_retry,
        ])
        if retry_count > 1:
            raise ValueError(
                f"TurnMode: multiple retry flags set "
                f"(babble={is_babble_retry}, "
                f"action_claim={is_action_claim_retry}, "
                f"code={is_code_retry}) — invalid"
            )
        if is_dj_auto and is_synthetic:
            raise ValueError(
                f"TurnMode: is_dj_auto=True AND is_synthetic=True — "
                f"DJ ticks are never synthetic (recon 0021b §3.2 #17)"
            )
        if is_dj_auto:
            return cls.DJ_AUTO
        if is_babble_retry:
            return cls.BABBLE_RETRY
        if is_action_claim_retry:
            return cls.ACTION_CLAIM
        if is_code_retry:
            return cls.CODE_RETRY
        if is_synthetic:
            # is_synthetic=True без retry-флага — не reachable сегодня,
            # но синтаксически возможно (recon §3.2 #1). По ADR-0018
            # мы raise'им на неизвестную комбинацию — никакого silent
            # default to USER (это бы сломало bug-class #1881).
            raise ValueError(
                f"TurnMode: is_synthetic=True without a retry flag — "
                f"unreachable today, would have caused #1881 ping-pong. "
                f"Pass one of babble_retry / action_claim / code_retry."
            )
        return cls.USER

    def requires_budget(self) -> bool:
        """True if this mode participates in the synthetic-retry budget.

        USER resets the budget (it gets a fresh N retries);
        others inherit parent's budget (no reset).
        """
        return self is TurnMode.USER

    def allows_music_guard(self) -> bool:
        """Issue #992 Bug B/C — DJ/USER/BABBLE/ACTION_CLAIM may need a
        music guard (e.g. user said 'сыграй Баха'); CODE_RETRY never
        (Renardo code has nothing to do with music).
        """
        return self in {TurnMode.USER, TurnMode.DJ_AUTO,
                        TurnMode.BABBLE_RETRY, TurnMode.ACTION_CLAIM}

    def allows_tool_skipped_guard(self) -> bool:
        """Issue #1777 / #1762 — only USER has non-music tool semantics
        (set_voice, get_current_time, etc.). DJ/RETRIES don't.
        """
        return self is TurnMode.USER

    def closes_dialogue_on_finish(self) -> bool:
        """USER transitions to IDLE; DJ stays in DIALOGUE for next tick;
        RETRIES defer (retry's _run_turn needs DIALOGUE state)."""
        return self is TurnMode.USER
```

### B.3 Альтернатива: dataclass

Если потребуется **per-mode policy data** (как в ADR-0021a `TurnKindPolicy`),
enum расширяется до dataclass:

```python
@dataclass(frozen=True)
class TurnModeSpec:
    """Per-mode policy data (e.g. for the 3 hot-spots in _run_turn)."""
    reset_retry_budgets: bool
    allow_music_guard: bool
    allow_tool_skipped_guard: bool
    say_degraded_on_error: bool
    close_dialogue_on_finish: bool

POLICY_BY_MODE: Mapping[TurnMode, TurnModeSpec] = { ... }
```

**Когда выбрать dataclass:** если `_run_turn` начинает роутить по ≥3
различным per-mode веткам. Пока (после PR #2631, CC=13) per-mode routing
ограничен 2 ветками (`if policy.allow_music_guard` / `if policy.allow_tool_skipped_guard`),
поэтому **enum с property-методами достаточен**.

Если Шифу выберет Сценарий A (§0), этот dataclass уже сделан в ADR-0021a
как `TurnKindPolicy` + `POLICY_BY_KIND`.

### B.4 Validation at construction time

```python
# На входе в _run_turn (line ~3388, после сбора флагов):
try:
    turn_mode = TurnMode.from_flags(
        is_dj_auto=is_dj_auto,
        is_babble_retry=is_babble_retry,
        is_action_claim_retry=is_action_claim_retry,
        is_code_retry=is_code_retry,
        is_synthetic=is_synthetic,
    )
except ValueError as exc:
    # ADR-0018: программная ошибка в dispatch-site (caller passed
    # невалидную комбинацию). Log + bail (return без finally).
    # Не raise — мы async, и raise здесь убьёт loop.
    self.get_logger().error(
        f"❌ [turn_mode] invalid flag combination: {exc}"
    )
    return
```

Тестовое покрытие: `test_turn_mode_classifier.py::test_*` × 27 невалидных
комбинаций (все, кроме 5 reachable) — каждая должна `raise ValueError`.

---

## C. State invariants and ownership boundaries

### C.1 Per-piece ownership

| Поле | Где живёт | Owns | Per-frame | Per-recursion | Shared | Обоснование |
|---|---|---|---|---|---|---|
| `self._run_task` | `DialogueNode.__init__:481` | **per-frame** (записывается на входе `_run_turn:3389`, стирается в `finally:3536`) | ✅ да | сохраняется через рекурсию | да (race на reentrancy) | Issue #992 Bug B; slot — single-valued, защищён `_task_lock`. Finalizer читает identity, не steal'ит. |
| `self._task_lock` | `DialogueNode.__init__:482` | **shared** infrastructure | — | — | да | Lock — глобальный для ноды. Finalizer берёт handle в `FinalizerContext`, не создаёт свой. |
| `self._synthetic_retries_left` | `DialogueNode.__init__:980` + `TurnState.budget_left` (mirror, `core/turn.py:151`) | **shared** | сбрасывается на USER | сохраняется | да (mirror критичен, см. #1881) | Issue #1881: budget — single integer; guards в child frame могут drain'ить parent's budget. Finalizer **читает** snapshot, **не пишет**. |
| `self._pending_music_cleanup` | `DialogueNode.__init__:845` | **shared** (через finalizer → PostTurnMusicPolicy) | читается в PostTurnMusicPolicy, пишется в `_execute_post_turn_music_actions` | сохраняется | да | Issue #992 Bug B contract: parent turn's finally может идти параллельно с child's set; single bool. Finalizer не пишет напрямую — delegate в `_apply_music_cleanup_policy`. |
| `self._track_mode_music_active` | `DialogueNode.__init__:863` | **shared** | то же | сохраняется | да | Тот же SSoT через `PostTurnMusicPolicy`. |
| `self._active_batches` | `DialogueNode.__init__:846` | **shared** (через `_publish_music_cleanup_if_idle:2599+`) | читается в PostTurnMusicPolicy | сохраняется | да | `dict[str, int]` — single-valued SSoT. |
| `self._retry_dispatched_in_turn` | `DialogueNode.__init__:879` | **per-frame** | сбрасывается на каждый вход (`_run_turn:3430`) | **не** наследуется через рекурсию | локально для кадра | Issue #992 Bug D: child **пере**сбрасывает на 3430 (per-frame), но DSM-correctness опирается на **return value** guard'ов, не на flag. Finalizer **не пишет** в этот flag — это работа guard'ов (`_mark_retry_dispatched:4581`). |
| `self._pending_user_messages` | `DialogueNode.__init__:491` (`deque`) | **shared** (single deque) | drain'ится в finally (S7) | сохраняется | да | Issue #968 S7: lock-free atomic ops в CPython; finalizer drain'ит, защита от double-dispatch через `queue.clear()` после consume. |
| `self._run_cancelled` | `DialogueNode.__init__` (lookup 3390) | **per-frame** | reset на входе `_run_turn:3390` | не наследуется | локально | Используется для barge-in detection; finalizer **не** касается (это concern `_handle_result`). |
| `turn_mode` (новое) | **локальная переменная** в `_run_turn` | **per-frame** (per-invocation) | вычисляется из флагов на входе (`TurnMode.from_flags`) | **не** передаётся в рекурсию — child классифицирует заново через `_dispatch_turn` | **нет** | ADR-0021 PR-D уже invariant: `del turn_kind` после use — child не наследует. Finalizer получает `turn_mode` через `FinalizerContext.mode`. |
| `self._dsm.current_state` | `DialogueNode._dsm` | **shared** | transition'ы в finally (DIALOGUE_END), guards' `_reopen_dialogue_for_retry:4549+` | сохраняется | да | Finalizer **читает** через `_finalize_turn_dsm`, **не пишет** напрямую (delegate'ит в `_dsm.on_event` helper'у). |
| `self._synthetic_retries_left` после finalizer | (см. выше) | **shared** | decrement'ы **внутри** guards (`_consume_synthetic_retry:4608`), не в finalizer | сохраняется | да | Finalizer **не трогает** budget — это контракт guards через `_mark_retry_dispatched`. |
| `was_dj_auto` | **локальная переменная** в `_run_turn` | **per-frame local** | `_run_turn:3395` | не наследуется | нет | Per-frame alias of `is_dj_auto`; finalizer получает через `FinalizerContext.was_dj_auto`. |
| `result` (Reply) | **локальная переменная** | **per-frame local** | `_run_turn:3437` | не наследуется | нет | LLM's reply; может быть None (CancelledError / Exception). Finalizer получает через `FinalizerContext.result`. |

### C.2 Ownership-классы

- **per-frame**: создаётся/умирает с `_run_turn` (slot, flags, locals).
- **finalizer**: живёт в `finally`, инициализируется из per-frame locals +
  shared snapshot; умирает с `run()` return.
- **shared**: долгоживущее состояние ноды; finalizer **читает** snapshot,
  **не пишет** напрямую (кроме slot release через lock-protected check).
- **caller** (caller of `_run_turn`): тот, кто передал 5 bool-флагов через
  `_dispatch_turn`; не имеет state на стороне finalizer.

### C.3 Запрещённые операции finalizer'а (security boundary)

Finalizer **НЕ** имеет права:

1. ❌ Писать в `self._synthetic_retries_left` (это делают guards).
2. ❌ Писать в `self._retry_dispatched_in_turn` (это делают guards).
3. ❌ Мутировать `self._pending_user_messages` (кроме как через
   `_drain_pending_user_messages` helper).
4. ❌ Писать в `self._dsm.current_state` напрямую (через
   `_finalize_turn_dsm` helper, который зовёт `self._dsm.on_event`).
5. ❌ Вызывать `self._dispatch_turn(...)` (это делают guards в `finally`
   **до** finalizer — финализируем то, что уже произошло).

Эти инварианты проверяются `pytest`-тестом `test_finalizer_invariants.py`:
mock'аем каждый из 5 self.* writer'ов и проверяем, что finalizer
вызывает только **read-only** site'ы + lock-protected slot release.

---

## D. Migration strategy

### D.1 Post-refactor `_run_turn` outline (псевдокод, ≤30 строк)

```python
async def _run_turn(self, user_input, *, is_dj_auto=False,
                    is_babble_retry=False, is_action_claim_retry=False,
                    is_code_retry=False, is_synthetic=False,
                    raw_user_command=None, speaker_tag=None,
                    speaker_duration_s=0.0, from_tg=False) -> None:
    # 1. CLASSIFY (CC=1, §B.4 validation)
    try:
        turn_mode = TurnMode.from_flags(is_dj_auto=is_dj_auto, ...)
    except ValueError as exc:
        self.get_logger().error(f"[turn_mode] {exc}"); return

    # 2. SLOT-ACQUIRE (CC=2, неизменяемо)
    with self._task_lock:
        self._run_task = asyncio.current_task()
    self._run_cancelled = False
    was_dj_auto = is_dj_auto  # per-frame local

    # 3. BUDGET-RESET (CC=2, через policy; см. ADR-0021a §D3 «3.»)
    if turn_mode.requires_budget():
        self._reset_turn_retry_budgets(
            is_synthetic=is_synthetic, was_dj_auto=was_dj_auto,
            user_input=user_input,
        )

    # 4. SPEAKER/CTX (CC=2, helper'ы из PR #2631)
    user_input, dynamic_system = await self._prepare_user_input_context(
        user_input=user_input, from_tg=from_tg, was_dj_auto=was_dj_auto,
        speaker_context=await self._handle_speaker_turn(...) if ... else None,
    )

    result = None
    try:
        # 5. LLM INVOKE (CC=1, helper)
        result = await self._invoke_llm_with_telemetry(...)
        self._handle_result(result, user_input=user_input,
                            is_dj_auto=was_dj_auto,
                            raw_user_command=raw_user_command)
    except asyncio.CancelledError:
        if is_metrics_enabled(): record_barge_in(); result = None
    except Exception as exc:                              # CC=2, см. ADR-0021a §D3
        self._handle_llm_error(exc, was_dj_auto, user_input, raw_user_command)
        result = None
    finally:
        # 6. FINALIZER (CC=0, всё в _MusicTurnFinalizer.run())
        ctx = FinalizerContext(
            mode=turn_mode,
            retries_left=self._synthetic_retries_left,
            run_task=asyncio.current_task(),
            task_lock=self._task_lock,
            dialogue_node=self,
            result=result,
            raw_user_command=raw_user_command,
            was_dj_auto=was_dj_auto,
            speech_id=getattr(result, "speech_id", None),
        )
        _MusicTurnFinalizer(ctx).run()
```

### D.2 Cyclomatic complexity per branch

| # | Блок | CC (radon) | Что даёт −CC |
|---|---|---|---|
| 1 | `TurnMode.from_flags` validation | CC=4 (5 if'ов + sum) | — |
| 2 | Slot-acquire | CC=2 (1 with + 1 assignment) | — |
| 3 | `if turn_mode.requires_budget()` → `_reset_turn_retry_budgets` | CC=1 | enum dispatch −2 vs old `if not was_dj_auto and not user_input.startswith(...)` |
| 4 | `_prepare_user_input_context` | CC=1 (helper) | helper extracted |
| 5 | `_invoke_llm_with_telemetry` | CC=1 (helper) | helper extracted |
| 5' | `try/except CancelledError/Exception` | CC=2 | ADR-0021a §D3 −3 |
| 6 | Finalizer `run()` (внутри отдельный класс) | CC=0 в `_run_turn` | **−6** (вся 95-строчная finally ушла в класс) |
| 7 | `_MusicTurnFinalizer.run()` body | CC=8 (7 steps + idempotency guard) | собственный CC-budget, цель ≤10 |
| | **ИТОГО `_run_turn`** | **CC=11–13** (target ≤15) ✅ | **−8 vs current CC=13** + ещё −3 в PR-E2 (см. ADR-0021a §D3) |

### D.3 `_dispatch_turn` и guard chain

`_dispatch_turn` (lines 2714–2780) **не меняется** в части 5-bool-сигнатуры —
это сознательное решение (ADR-0021a §D4: «внешний API `_run_turn` остаётся
5-flag-формой»). Что меняется:

1. **`_dispatch_turn` остаётся как есть.** Он только решает `was_idle` /
   `music_cleanup` (lines 2748–2757) и зовёт `asyncio.run_coroutine_threadsafe`.
2. **Guard'ы остаются как есть.** Каждый guard (`_check_babble_and_retry`,
   `_check_unbacked_action_claim_and_retry`, `_apply_music_guard`,
   `_apply_tool_skipped_guard`, etc.) по-прежнему вызывается из
   `_run_turn.finally` через `_MusicTurnFinalizer._apply_*_guard`.
3. **Связь между guard'ами и finalizer'ом:** finalizer **получает**
   `music_retry_dispatched` и `tool_retry_dispatched` из return value
   guard'ов (не из `self._retry_dispatched_in_turn` flag'а, который
   **остаётся** для guard'ов, но финалайзер его **не читает** для DSM —
   это per ADR-0021a §"Cross-frame correctness invariants #2").

### D.4 🔴 FIX → finalizer method map (1:1)

Каждый `🔴 FIX` маркер из recon §2 маппится 1:1 на finalizer method.
Preservation гарантируется тем, что **имена методов finalizer'а соответствуют
названиям комментариев**.

| Recon F# | File:line (до PR #2631) | Finalizer method | 🔴 FIX маркер | Issue |
|---|---|---|---|---|
| **F1** | `_handle_llm_error:3525` | `_handle_llm_error` (внутри except, не finalizer) | `# (живой 12.08 FIX …)` | live 12.08, #1278 |
| **F2** | `finally:3530-3535` | `_release_run_task_slot` | `# Issue #992 Bug B …` | #992 B |
| **F3** | `finally:3539-3545` | `_drain_pending_user_messages` | `# S7 (scheduler-segments-merge, issue #968) …` | #968 (S7) |
| **F4** | `finally:3547-3552` | `_apply_music_cleanup_policy` (delegate) | `# Issue #935 v3 … # Issue #992 …` | #935 v3, #992 |
| **F5** | `finally:3561-3573` | `_apply_music_guard` | `# 🔴 FIX (issue #1204 …)` | #1204 |
| **F6** | `finally:3577-3584` | `_apply_music_guard` (passes `spoken=`) | `# Issue #2565 …` | #2565 |
| **F7** | `finally:3586-3603` | `_apply_tool_skipped_guard` | `# Issue #1777 / #1762 — Bug C retry …` | #1777, #1762 |
| **F8** | `finally:3604-3615` | `_finalize_turn_dsm` | `# Issue #992 Bug D — defer the DIALOGUE_END … S7 …` | #992 D, #968 S7 |

Каждый маркер переезжает в docstring соответствующего метода finalizer'а.
**Сохранение 1:1** проверяется тестом: `test_finalizer_fix_preservation.py`
читает каждый docstring, грепает `🔴 FIX` маркер, и проверяет, что issue
reference соответствует recon §2 таблице.

### D.5 Guard chain (внутри finalizer.run())

```
MusicTurnFinalizer.run()
    │
    ├── _release_run_task_slot()      # F2
    │
    ├── _drain_pending_user_messages() # F3 → returns bool → _pending_queue_dispatched
    │
    ├── _apply_music_cleanup_policy()  # F4 (delegate to existing helper)
    │
    ├── _apply_music_guard()          # F5/F6 → returns bool → _music_retry_dispatched
    │   └── guard may dispatch a follow-up (sets _pending_user_messages for S7)
    │
    ├── _apply_tool_skipped_guard()   # F7 → returns bool → _tool_retry_dispatched
    │   └── guard may dispatch a follow-up
    │
    ├── _finalize_turn_dsm(           # F8 — reads 4 *retry_dispatched flags
    │       guard_retry_pending = bool(self._retry_dispatched_in_turn),
    │       music_retry_dispatched = _music_retry_dispatched,
    │       pending_queue_dispatched = _pending_queue_dispatched,
    │       tool_retry_dispatched = _tool_retry_dispatched,
    │   )
    │
    └── _publish_state()              # всегда последний (issue #918, #1160)
```

Это **сохраняет** существующий ordering (slot-release → drain → music-policy →
music-guard → tool-guard → DSM-finalize → publish), который был
задокументирован в recon §2 как «reordering any of F1–F8 will break a
live-incident fix».

### D.6 Risk: order dependency

Если Шифу выберет Сценарий B (§0), класс `_MusicTurnFinalizer` **обязан**
сохранять ordering строго. Тест `test_finalizer_order.py::test_order_invariant`
фиксирует: после `_release_run_task_slot`, `_pending_user_messages` уже
очищен (если был S7), `_pending_music_cleanup` уже armed (если F4 сработал).
Любое переупорядочивание ломает live-incident фиксы.

---

## §X. References

- **Recon (sibling task `t_cee156f5`, ✅ done):** `docs/adr/0021b-recon-run-turn-state-locks-and-fixes.md` — file:line карта всех 6 pieces of shared state + 8 🔴 FIX маркеров + 32 → 5 reachability analysis + reentry trace (parent → music USER_RETRY → drain child).
- **Sibling ADR `t_adc73325`:** `docs/adr/0021a-post-turn-music-finalizer.md` (PR #2647, branch `z-architect/2627-adr-post-turn-music-finalizer`, **OPEN**) — TurnKind enum + POLICY_BY_KIND dict-of-callables подход. **Конфликт см. §0.**
- **PR #2631 (merged, commit `6c7496bac`):** baseline — `_run_turn` CC 59→13, helpers вынесены.
- **PR #2640 (open):** follow-on micro-helpers + `TurnKind` enum (PR-D).
- **ADR-0021 R1:** `docs/adr/0021-dialogue-node-decomposition-discipline.md` — CC≤15 budget + R2 State SSoT + R5 issue-link.
- **ADR-0021-r1:** `docs/adr/0021-r1-cc-budget-ratch-and-phantom-detection.md` — R-1a..R-1g (frozen legacy, ratchet, phantom, verify-remote, CC>30 ADR, honesty message, nightly job).
- **ADR-0013:** incremental delivery (per-bag workflow).
- **ADR-0018:** честный FAIL > красивый PASS (capability-honest, raw-evidence, partial ≠ swallow).
- **`core/turn.py:106`:** `TurnContext` (frozen dataclass, расширяется optional полями **или** служит baseline для нового `FinalizerContext`).
- **`core/turn.py:125`:** `TurnState` (frozen dataclass, `budget_left` mirror of `self._synthetic_retries_left`).
- **`core/turn.py:248`:** `TurnGuards` (orchestrator, прецедент dict-of-callables pattern).
- **Live incidents:** #935 (stop_music deferral), #968 S7 (drain pending), #992 Bug B/C/D, #1160 (Prometheus), #1204 (DJ guard before DIALOGUE_END), #1278 (ProviderError), #2559, #2565 (phantom-action), #1777, #1762 (tool skip), #1881 (budget ping-pong), #2626 (CC-budget guard).

---

## §Y. Acceptance для ADR (он сам)

- [ ] **§0 conflict** с ADR-0021a явно зафиксирован, три сценария
      (A/B/C) с trade-off'ами, рекомендация = A.
- [ ] **Товарищ Шифу выбрал сценарий** в комментарии к этой карточке
      или в issue #2627.
- [ ] §A — `_MusicTurnFinalizer` API contract (inputs / outputs / error
      semantics / idempotency) детально расписан, привязан к F1–F8.
- [ ] §B — `TurnMode` enum + 5 reachable + validation на 27 unreachable.
- [ ] §C — таблица ownership для всех 6 pieces of shared state + 8
      per-frame locals + invariants.
- [ ] §D — post-refactor `_run_turn` pseudocode ≤30 строк + CC-budget
      per branch + 🔴 FIX 1:1 mapping + ordering invariant.

## §Z. Acceptance для имплементации (если Шифу выберет Сценарий B/C, для воркера на PR-E)

- [ ] `core/turn_mode.py` (или расширение `core/turn.py`):
      enum + `from_flags` с validation, тесты на 5 reachable + 27 unreachable.
- [ ] `dialogue_node/_music_turn_finalizer.py` (или
      `core/music_turn_finalizer.py`): `_MusicTurnFinalizer` + `FinalizerContext`
      + `FinalizerResult` + `FinalizerActions` + 7 private methods.
- [ ] `_run_turn` CC ≤ 15 (radon).
- [ ] Каждый `🔴 FIX` маркер переехал в docstring соответствующего
      finalizer method (1:1 preservation test).
- [ ] `test_finalizer_idempotency.py`: повторный `run()` возвращает
      тот же `FinalizerResult`.
- [ ] `test_finalizer_reentry.py`: parent + child `_run_turn` finally
      interleave — slot-release / drain / DIALOGUE_END защиты работают.
- [ ] `pytest -v src/rob_box_voice/test/unit/core/test_music_turn_finalizer.py` зелёный.
- [ ] e2e: «спой рэп про роботов» (music USER_RETRY), «который час»
      (tool-skipped retry), DJ сессия (#992 Bug C regression).
- [ ] `validate_honesty.sh` (ADR-0018) — pre-PR check проходит.