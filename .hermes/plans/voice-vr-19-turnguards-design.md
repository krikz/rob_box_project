# voice-vr 19 — TurnGuards design (ADR-0080 §2.4)

**Issue:** [#2241](https://github.com/krikz/rob_box_project/issues/2241)
**ADR:** `docs/adr/0080-voice-and-headset-control-eight-seams.md` §2.4
**Status:** design — committed before code so reviewers can sanity-check the seam shape

## What changed before (facts)

- Predicates live in `core/dialogue_guards.py` + `core/music_guard.py`. They are
  pure, covered by 1449 + 209 tests. **Orchestration** lived in
  `dialogue_node.py` and was the bug-class.
- Two orchestration sites (per the issue body):
  - `_run_turn.finally` (`:3920` `_apply_music_guard`, `:3938` `_apply_tool_skipped_guard`)
  - `_handle_result` (`:5485` regurge, `:5499` babble, `:5511` renardo_code,
    `:5521` action_claim)
- Six per-guard flags + one budget + one "in-turn retry" flag (13 state slots in
  `__init__`):
  - `_babble_retry_used`, `_action_claim_retry_used`, `_code_speech_retry_used`,
    `_tool_retry_used`, `_system_regurgitate_retry_used`
  - `_synthetic_retries_left` (`DEFAULT_SYNTHETIC_RETRIES`, the only budget)
  - `_retry_dispatched_in_turn`
- The pre-card comments at `:3534-3541` and `:5473-5479` literally admit two
  of the failure modes the refactor closes: "8 calls per phrase" and "seventh
  guard added because the sixth's budget was already spent".

## What we want (ADR-0080 §2.4 verbatim)

```
TurnGuards(order: Sequence[Guard], budget: RetryBudget)
  evaluate(reply: Reply, context: TurnContext, state: TurnState) -> Verdict

Verdict = Accept | Retry(prompt, guard_name) | Discard(reason)
```

- One list of guards, in one place.
- Budget and "one retry per turn" live INSIDE the module.
- `dialogue_node` only EXECUTES the verdict. No flags, no shared state.

## Module shape (`core/turn.py`)

```python
@dataclass(frozen=True)
class Reply:
    """The LLM's final response that the guards inspect."""
    spoken: str                 # raw text (after strip_markdown)
    tools_called: tuple         # tuple[str, ...], empty means no tool
    speak_text_real: int = 0    # count of speak_text calls that actually voiced

@dataclass(frozen=True)
class TurnContext:
    """Inputs that are NOT the LLM's reply."""
    user_input: str                              # original user command (for retry-prompt)
    is_dj_auto: bool = False                     # DJ transition, not user-initiated
    was_synthetic: bool = False                  # True on a retry turn (don't reset budget)

@dataclass(frozen=True)
class TurnState:
    """Mutated turn-scoped state. Budget counters live here."""
    budget_left: int                             # remaining synthetic retries

@dataclass(frozen=True)
class GuardContext:
    """Composite argument passed to each guard's evaluate."""
    reply: Reply
    turn: TurnContext
    state: TurnState

class VerdictKind(str, Enum):
    ACCEPT = "accept"
    RETRY = "retry"
    DISCARD = "discard"

@dataclass(frozen=True)
class Verdict:
    kind: VerdictKind
    guard_name: str                              # which guard decided; "" for ACCEPT
    prompt: Optional[str] = None                 # RETRY-only: synthetic prompt for next turn
    reason: Optional[str] = None                 # DISCARD-only: human-readable tag for logs

class Guard(Protocol):
    """One policy check. Pure: takes inputs + state, returns a Verdict or None to defer."""
    name: str
    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]: ...

class TurnGuards:
    """Orchestrator. Owns the order, the budget, and the 'one retry per turn' rule."""
    def __init__(
        self,
        guards: Sequence[Guard],
        *,
        max_retries: int = DEFAULT_MAX_SYNTHETIC_RETRIES,
    ): ...
    def evaluate(self, reply: Reply, turn: TurnContext, state: TurnState) -> Verdict: ...
    def with_state(self, **mutations: Any) -> TurnState: ...  # for tests / budget reset
```

### Verdict semantics

| kind      | meaning                                                                                                       | dialogue_node action                                                          |
|-----------|---------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|
| `ACCEPT`  | No guard raised. Reply is fine as-is.                                                                         | publish reply to TTS; close turn normally                                     |
| `RETRY`   | A guard wants a single-shot synthetic retry. The verdict carries `prompt` and `guard_name`.                   | dispatch the synthetic prompt as a new turn; budget--                          |
| `DISCARD` | A guard wants to SILENCE this reply (e.g. `planning_narration` hard-mute, future additions).                  | do NOT publish reply; close turn normally                                    |

`RETRY` always implies "publish NOTHING this turn — the retry turn will produce
the user-facing reply". That preserves the invariant the live bug-class depended
on: the user never hears the bad reply AND the retry answer.

### Budget + "one retry per turn"

- `evaluate` checks the budget in `state.budget_left`. If `0` and a guard
  returns `RETRY`, we **downgrade to ACCEPT** and log a warning ("budget
  exhausted, publishing as-is"). This replaces `_consume_synthetic_retry`.
- Within a single `evaluate` call, we collect guards in `order`; the FIRST one
  that returns a non-`None` verdict wins. Subsequent guards are NOT consulted.
  This replaces the contract "каждый `_check_*_and_retry` обязан позвать
  `_mark_retry_dispatched`" — it just falls out of the data flow.
- Within a single turn, multiple `evaluate` calls are possible (e.g. after a
  music retry, the music retry turn calls `evaluate` again). The budget
  counts across them; the caller is responsible for passing the SAME state
  object (mutated by the caller, not by TurnGuards, to keep it pure).

### Concrete guard list (matches ADR-0080 §1.5 inventory)

The orchestrator takes a `Sequence[Guard]`. The default ORDER for
`DialogueNode` (matches `:3934-3936`'s prose, hardened to a real list):

1. `SystemRegurgitateGuard` (issue #2175) — must fire BEFORE babble so
   regurgitated template doesn't get the babble CRITICAL pasted on top.
2. `MusicGuard` (re-exported from `core/music_guard.py`, the existing TD-2
   module).
3. `ToolSkippedGuard` (issue #1777 / #1762).
4. `BabbleGuard` (issue #992 Bug D).
5. `EmbeddedRenardoCodeGuard` (issue #992 Bug C').
6. `UnbackedActionClaimGuard` (issue #992 Bug E).
7. `PlanningNarrationHardMute` (issue #1882) — produces `DISCARD`, not `RETRY`.

All seven are pure: they take `GuardContext`, return `Optional[Verdict]`.
`MusicGuard` keeps its existing rich evaluation surface (DJ vs user vs
budget-exhausted nudge) and translates it into a `Verdict` at the boundary.

## Where it lives

- New file: `src/rob_box_voice/rob_box_voice/core/turn.py` (≈350 LOC).
- New tests: `src/rob_box_voice/test/unit/core/test_turn.py`.
- `dialogue_node.py` changes:
  - `_run_turn.finally` (`:3920` music + `:3938` tool) collapses into ONE call
    that consults the TurnGuards instance.
  - `_handle_result` (`:5485-5526`) collapses into ONE call against the same
    TurnGuards instance.
  - 5 per-guard flags + `_synthetic_retries_left` + `_retry_dispatched_in_turn`
    REMOVED. `git grep -n "_retry_used\|_synthetic_retries_left" .../dialogue_node.py`
    must return empty.
  - `_consume_synthetic_retry`, `_mark_retry_dispatched`,
    `_reopen_dialogue_for_retry` REMOVED (their only callers were the
    extracted guards).
  - `_apply_music_guard`, `_apply_tool_skipped_guard`,
    `_check_babble_and_retry`, `_check_embedded_renardo_code_and_retry`,
    `_check_unbacked_action_claim_and_retry`,
    `_check_system_template_regurgitate_and_retry` REMOVED.
  - `MusicGuard` instance kept (still owns DJ vs user budgets via TD-2). The
    TurnGuards instance HOLDS the music guard, just like the others.

## CC-budget impact

- `_run_turn` 57 → stays roughly the same (call sites collapse from 2 to 1
  guarded dispatch).
- `_handle_result` 65 → drops noticeably because 5 sequential `if spoken and
  self._check_*_and_retry(...): return` blocks (~50 LOC) collapse to one
  `verdict = self._turn_guards.evaluate(...)` + a small if/elif/else dispatch.
- Per ADR-0021 R1 the CC ceiling is 15. The reduction is required for
  `_handle_result`; the baseline gate will fail otherwise.

## What this does NOT do (Out of scope per card)

- Does NOT change predicate behavior byte-for-byte (`core/dialogue_guards.py`
  and `core/music_guard.py` are imports, not rewrites).
- Does NOT touch stend (voice-vr 20).
- Does NOT add new diagnostic fields to TTS or the operator panel.

## Tests (new file: `test/unit/core/test_turn.py`)

- `TestOrderFirstVerdictWins` — guard A returns RETRY, guard B is not consulted.
- `TestBudgetEnforced` — budget=0, guard returns RETRY → ACCEPT with warning.
- `TestOneRetryPerTurn` — multiple guards, only the first to fire counts.
- `TestBudgetResetOnNewUserTurn` — fresh TurnState resets the counter.
- `TestVerdictActionsAreCorrect` — RETRY carries prompt + guard_name,
  DISCARD carries reason, ACCEPT has empty guard_name.
- `TestMusicGuardIsFirstClassMember` — guards=[MusicGuardAdapter, …]
  translates existing MusicGuardVerdict into the unified Verdict shape.

## DoD mapping

| DoD                                                                  | How it's met                                                                 |
|----------------------------------------------------------------------|-------------------------------------------------------------------------------|
| `git grep` for `_retry_used\|_synthetic_retries_left` returns empty  | All 6 flags + budget removed from `dialogue_node.py` (move to `core/turn.py`)|
| Order/budget test values, not a node                                | Pure `TurnGuards` evaluated with dataclasses; no `DialogueNode` instance     |
| CC `_handle_result` and `_run_turn` reduced                         | Catch blocks collapse to 1-line `verdict = self._turn_guards.evaluate(...)`  |
| `test_issue_992_*`, `test_issue_1777_*`, `test_dialogue_guards.py` green | Same predicates imported; new wrapper tests stay green                         |
| Robot: no degradation, no "8 calls" log                             | One guard per turn; order is a list, not prose                                |
