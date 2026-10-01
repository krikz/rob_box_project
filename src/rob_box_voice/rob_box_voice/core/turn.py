"""turn.py — чистая политика babble-ретрая и бюджета синтетических ретраев.

Issue #2266 / ADR-0021 R2 — предикаты и шаг бюджета живут в ``core/``,
``DialogueNode._check_babble_and_retry`` — тонкая ROS-оболочка над
:func:`begin_babble_retry`.

ADR-0148 §2.4 / эпик #3312: параллельный оркестратор ``TurnGuards`` (и
гуарды-классы, которые использовал только он) удалён — он был выключен
(``_use_turn_guards=False``) с 09.09 и дублировал живой путь
``DialogueNode._handle_result`` / ``dialogue_guards``. Здесь осталось ровно
то, что вызывает прод: :class:`TurnState`, :func:`reset_budget`,
:func:`begin_babble_retry` и :class:`BabbleGuard`, на котором тот держится.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional, Tuple

from .dialogue_guards import (
    CLAIM_JUSTIFYING_TOOLS,
    build_babble_retry_prompt,
    is_metalanguage_babble,
    is_planning_narration,
    user_wants_performance,
)


#: Default ceiling for synthetic retries inside one user turn.
#:
#: Matches the legacy ``DialogueNode.DEFAULT_SYNTHETIC_RETRIES``. The value
#: was tuned live (issue #1881) to close the cross-guard ping-pong without
#: burning more than ~3 LLM round-trips per phrase on the worst-case music
#: requests.
DEFAULT_MAX_SYNTHETIC_RETRIES: int = 3


# ---------------------------------------------------------------------------
# Value types — what the guards see and what they return
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Reply:
    """The LLM's final response that the guards inspect.

    Attributes:
        spoken: Raw text after ``strip_markdown``. ``""`` means the LLM
            returned nothing (see ``RobBox Voice/TARS docs`` for the
            empty-response contract).
        tools_called: Tuple of tool names the LLM invoked this turn. ``()``
            means no tool was called — a frequent trigger for music / tool /
            action-claim retries.
        speak_text_real: Count of ``speak_text`` calls that ACTUALLY voiced
            non-empty text (phantom calls with empty payload do not count,
            see issue #1343 / #988).
    """

    spoken: str
    tools_called: Tuple[str, ...] = ()
    speak_text_real: int = 0


@dataclass(frozen=True)
class TurnContext:
    """Inputs that are NOT the LLM's reply.

    Attributes:
        user_input: Original user command. Used to build retry prompts that
            echo the request. On retry turns this is still the ORIGINAL
            command, not the synthetic CRITICAL prompt (issue #1204).
        is_dj_auto: ``True`` for DJ-mode tick transitions (Bug B path). The
            music guard uses it; everything else ignores it.
        has_error: ``True`` when ``DialogResult.error is not None`` for this
            turn. Issue #2549 — :class:`UniversalActionClaimGuard` must NOT
            retry on an already-errored turn (mirrors the legacy
            ``result.error is None`` gate at the ``_handle_result`` call
            site). Added alongside the guard itself (issue #2556) rather
            than threading a raw ``DialogResult`` through the orchestrator,
            which would leak a dialogue_node-specific type into ``core/``.
        speech_id: Optional speech-id for log correlation. Not interpreted by
            guards; carried for diagnostics.
        tool_error_occurred: ``True`` when at least one tool call THIS TURN
            returned ``is_error=True`` (issue #2949 — a tool being CALLED is
            not the same as it SUCCEEDING; a refused/errored
            ``save_arrangement_preset`` must not "back" a spoken claim of
            success, and a real action-fulfilling tool call that errored
            must not be mistaken for babble-suppression either). Mirrors
            ``DialogResult.tool_error_occurred`` (``rob_box_harness``).
    """

    user_input: str
    is_dj_auto: bool = False
    has_error: bool = False
    speech_id: Optional[str] = None
    tool_error_occurred: bool = False


@dataclass(frozen=True)
class TurnState:
    """Turn-scoped mutable state the orchestrator consults.

    The orchestrator does NOT mutate this; the caller (DialogueNode) threads
    a fresh ``TurnState`` between ``evaluate`` calls and resets the budget on
    a fresh user-initiated turn. Keeping the type frozen lets us pass it
    through pure functions without copy/clone ceremony.

    Attributes:
        budget_left: Remaining synthetic retries for the current turn.
            ``0`` means the budget is exhausted; the guard will
            downgrade any ``Retry`` verdict to ``Accept`` with a warning.
        babble_retry_consumed: ``True`` once :class:`BabbleGuard` has
            already fired a one-shot synthetic retry for this turn.
            Mirrors the legacy ``DialogueNode._babble_retry_used`` flag
            (issue #992 Bug D). Lives in :class:`TurnState` so the
            ``core/`` orchestrator can enforce the one-shot rule without
            the dialogue_node adapter keeping a per-guard boolean of its
            own — same pattern as ``budget_left`` for the cross-guard
            budget (issue #1881). Other per-guard flags
            (``_code_speech_retry_used`` / ``_action_claim_retry_used`` /
            ``_tool_retry_used`` / ``_system_regurgitate_retry_used``)
            stay in dialogue_node for now and migrate in voice-vr 23
            (ADR-0084 §"Что НЕ сделано в этой карточке").
    """

    budget_left: int = DEFAULT_MAX_SYNTHETIC_RETRIES
    babble_retry_consumed: bool = False


@dataclass(frozen=True)
class GuardContext:
    """Composite argument passed to each guard's :meth:`Guard.evaluate`."""

    reply: Reply
    turn: TurnContext
    state: TurnState


# ---------------------------------------------------------------------------
# Verdict — the unified output shape
# ---------------------------------------------------------------------------


class VerdictKind(str, Enum):
    """Action the dialogue_node adapter must take.

    * ``ACCEPT`` — no guard raised. Publish the reply, close the turn.
    * ``RETRY`` — a guard wants a single-shot synthetic retry. The verdict
      carries ``prompt`` and ``guard_name``. The dialogue_node must NOT
      publish the current reply to TTS (the retry turn will produce the
      user-facing answer). The orchestrator has already decremented the
      budget before returning this verdict.
    * ``DISCARD`` — a guard wants to silence this reply (no LLM retry).
      Used today by :class:`PlanningNarrationHardMute` (issue #1882); the
      door is open for future "this is just bad, drop it" additions.
    """

    ACCEPT = "accept"
    RETRY = "retry"
    DISCARD = "discard"


@dataclass(frozen=True)
class Verdict:
    """Result of one guard's :meth:`Guard.evaluate` (or the orchestrator).

    Attributes:
        kind: What the dialogue_node must do.
        guard_name: Which guard decided. ``""`` for ``ACCEPT`` (no guard
            raised).
        prompt: ``RETRY``-only: the synthetic prompt to dispatch as the next
            turn's ``user_input``.
        reason: ``DISCARD``-only: a short human-readable tag for the log
            line. Not interpreted programmatically.
    """

    kind: VerdictKind
    guard_name: str = ""
    prompt: Optional[str] = None
    reason: Optional[str] = None


#: Convenience constructor for "nothing raised".
ACCEPT = Verdict(kind=VerdictKind.ACCEPT)


# ---------------------------------------------------------------------------
# Convenience: budget-step helper
# ---------------------------------------------------------------------------


def consume_budget(state: TurnState) -> TurnState:
    """Return a new ``TurnState`` with ``budget_left`` decremented by 1.

    The dialogue_node adapter calls this after dispatching a ``Retry``
    verdict, so the next evaluation sees the decremented
    budget. Pure — no hidden mutation.
    """
    return replace(state, budget_left=max(0, state.budget_left - 1))


def consume_babble_retry(state: TurnState) -> TurnState:
    """Return a new ``TurnState`` with ``babble_retry_consumed=True``.

    Mirrors the legacy ``DialogueNode._babble_retry_used = True`` write
    that fires after a babble retry is dispatched (issue #992 Bug D).
    Lives in ``core/`` so :class:`BabbleGuard` can enforce the one-shot
    rule without dialogue_node keeping a per-guard boolean — same
    pattern as :func:`consume_budget` for the cross-guard budget.

    Idempotent: flipping the flag twice is harmless (the field is just
    ``True`` either way), so a misuse from a follow-up card can't
    accidentally regress behaviour.
    """
    return replace(state, babble_retry_consumed=True)


@dataclass(frozen=True)
class BabbleRetryDecision:
    """Pure result of :func:`begin_babble_retry` — what the adapter consumes.

    Issue #2266 / ADR-0021 R2 step 2 — the babble policy + the budget
    step + the one-shot flag all live in ``core/``. The adapter
    (:meth:`DialogueNode._check_babble_and_retry`) just performs the
    ROS-bound side effects (DSM transition, ``_dispatch_turn``) using
    the decision fields below.

    Attributes:
        prompt: The synthetic prompt to dispatch as the next turn's
            ``user_input``. Echoes the original request and adds the
            CRITICAL reminder — see
            :func:`rob_box_voice.core.dialogue_guards.build_babble_retry_prompt`.
        new_state: The :class:`TurnState` to install on the dialogue
            node — ``budget_left`` has been decremented by 1 and
            ``babble_retry_consumed=True``. Pure: a fresh ``TurnState``
            is returned (the input state is not mutated).
        reason: Short human-readable tag for the log line. Not
            interpreted programmatically; the adapter logs it via its
            own logger to keep ``core/`` log-free.
    """

    prompt: str
    new_state: TurnState
    reason: str


def begin_babble_retry(
    *,
    spoken: str,
    user_input: str,
    tools_called: tuple,
    speak_text_real: int,
    state: TurnState,
    tool_error_occurred: bool = False,
) -> Optional[BabbleRetryDecision]:
    """Issue #992 Bug D — decide whether to fire ONE babble retry, in pure.

    Same predicate chain as :class:`BabbleGuard` (speak-text-real /
    non-empty / metalanguage / planning / promise-only /
    user-wants-performance), but ADDS the two side-effecting state
    mutations that used to live inline in
    ``DialogueNode._check_babble_and_retry`` (issue #2266 /
    ADR-0021 R2 step 2):

    1. ``budget_left`` is decremented (mirrors the legacy
       ``self._consume_synthetic_retry(guard_name="babble")`` call).
    2. ``babble_retry_consumed=True`` is set (mirrors the legacy
       ``self._babble_retry_used = True`` write).

    Both happen as pure ``dataclasses.replace`` operations — the
    function takes a :class:`TurnState` and returns a new one, no
    mutation of the input. This keeps ``core/`` testable in isolation
    and lets the dialogue_node adapter stay a thin shell.

    Budget exhaustion is treated as "no retry" (matches the legacy
    semantics from ``_consume_synthetic_retry`` returning ``False``).
    The :class:`BabbleGuard` one-shot rule is enforced BEFORE this
    function consumes the budget, so a second babble reply in the same
    turn returns ``None`` without touching the budget — mirrors the
    legacy ``if self._babble_retry_used: return False`` short-circuit.

    Returns:
        ``None`` — no retry should fire (BabbleGuard deferred, or
        budget already exhausted). The caller publishes the original
        ``spoken`` text to TTS as-is.
        :class:`BabbleRetryDecision` — caller must (1) install
        ``new_state`` on the dialogue node, (2) perform the ROS-bound
        DSM transition, (3) call ``_dispatch_turn(prompt, ...)``.

    Pure: no ROS, no I/O, no logger access. Side effects live in the
    caller.
    """
    ctx = GuardContext(
        reply=Reply(
            spoken=spoken or "",
            tools_called=tuple(tools_called or ()),
            speak_text_real=int(speak_text_real or 0),
        ),
        turn=TurnContext(
            user_input=user_input or "",
            is_dj_auto=False,
            tool_error_occurred=bool(tool_error_occurred),
        ),
        state=state,
    )
    verdict = BabbleGuard().evaluate(ctx)
    if verdict is None:
        return None
    if verdict.kind is not VerdictKind.RETRY:
        # Future-proofing: BabbleGuard today only raises RETRY, but
        # if it ever raises DISCARD we honour it here as "no retry".
        return None
    if state.budget_left <= 0:
        # Mirrors ``_consume_synthetic_retry`` returning False: budget
        # exhausted → no retry, even if a guard asked. Already logged
        # by the caller in legacy code paths; the pure helper just
        # returns None and lets the caller log.
        return None
    # Build the post-mutation state: flag the one-shot AND decrement
    # the budget in one replace so the caller can't accidentally apply
    # only one of them.
    new_state = consume_budget(consume_babble_retry(state))
    prompt = verdict.prompt or build_babble_retry_prompt(user_input or "")
    return BabbleRetryDecision(
        prompt=prompt,
        new_state=new_state,
        reason="babble_guard",
    )


def reset_budget(max_retries: int = DEFAULT_MAX_SYNTHETIC_RETRIES) -> TurnState:
    """Return a fresh ``TurnState`` with the full budget.

    Called by the dialogue_node adapter on a NEW user-initiated turn (not
    on a synthetic retry, not on a DJ auto-transition).
    """
    return TurnState(budget_left=max_retries)


#: Pure-promise openers (issue #992 Bug D) — phrases that are NEVER
#: valid answers regardless of what the user asked for. The legacy
#: ``DialogueNode._check_babble_and_retry`` keeps this subset as a
#: fallback after ``user_wants_performance`` returns False. The
#: ``BabbleGuard`` wrapper below relies on the same constant.
#:
#: «Зачит», «погнали», «устроим», «переключ», «давай-ка» — these
#: openers come from live failures (see live 30.08 / 02.09 logs in
#: the legacy dialogue_node.py comments). The match is a substring on
#: the first 60 chars of the lower-cased reply.
BABBLE_PROMISE_ONLY_OPENERS: tuple = (
    "зачит",
    "погнали",
    "устроим",
    "переключ",
    "давай-ка",
)


def _is_promise_only_babble(spoken: str) -> bool:
    """``True`` iff the reply opens with one of the :data:`BABBLE_PROMISE_ONLY_OPENERS`."""
    head = spoken[:60].lower()
    return any(p in head for p in BABBLE_PROMISE_ONLY_OPENERS)


@dataclass(frozen=True)
class BabbleGuard:
    """Issue #992 Bug D — LLM answered with metalanguage instead of performing.

    Returns ``RETRY`` when one of these holds:

    1. :attr:`TurnState.babble_retry_consumed` is already ``True`` —
       the one-shot rule fired earlier this turn. ``None`` is returned
       so the second babble reply passes through to TTS verbatim (see
       ``test_issue_992_babble_guard.py::test_retry_only_fires_once_*``).
       The legacy equivalent was
       ``DialogueNode._babble_retry_used`` (now removed in voice-vr 22,
       issue #2266).
    2. The spoken text matches the *promise-only* opener subset
       (:data:`BABBLE_PROMISE_ONLY_OPENERS` — «зачит», «погнали»,
       «устроим», «переключ», «давай-ка»). These are NEVER valid
       answers, regardless of what the user asked for.
    3. The user explicitly asked for a performance
       (:func:`user_wants_performance`).
    4. The LLM is reading its own plan aloud (:func:`is_planning_narration`).

    See :func:`is_metalanguage_babble` for the full detector; this
    guard is a thin wrapper around it that produces the verdict
    shape.

    The caller (:class:`DialogueNode` adapter) is expected to apply
    :func:`consume_babble_retry` after a successful dispatch so the
    next :meth:`Guard.evaluate` call sees the consumed flag — mirrors
    :func:`consume_budget` for the cross-guard budget. Idempotency is
    not required at the consume site: the field is a boolean, and the
    orchestrator treats ``True`` as "already fired" regardless of how
    many times the write was attempted.
    """

    name: str = "babble"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if ctx.state.babble_retry_consumed:
            # One-shot rule (issue #992 Bug D): a second babble reply in
            # the same turn MUST pass through to TTS verbatim — the
            # alternative would be an unbounded LLM ping-pong. Mirrors
            # the legacy ``self._babble_retry_used`` short-circuit at
            # ``dialogue_node.py:4232-4233``.
            return None
        # Issue #2948 — a turn that ALREADY successfully ran the tool(s)
        # that fulfil the request (music-start / set_dj_mode / etc., see
        # :data:`CLAIM_JUSTIFYING_TOOLS`) is not babble: a short DJ-style
        # line NEXT TO a real action ("Слушай Still Dre, потом Next
        # Episode подхвачу!" with tools=['set_dj_mode', 'compose_music',
        # 'play_animation']) is flavour text, not an unfulfilled promise.
        # Retrying here re-dispatches the ORIGINAL user request and the
        # DJ set starts a second time (live 24.09, issue #2948). Mirrors
        # the same "tool called AND succeeded" bar as
        # :func:`detect_universal_action_claim` (issue #2949) — a called
        # tool that ERRORED does not count as fulfilling the request.
        if (
            set(ctx.reply.tools_called) & CLAIM_JUSTIFYING_TOOLS
            and not ctx.turn.tool_error_occurred
        ):
            return None
        if ctx.reply.speak_text_real > 0:
            return None
        if not ctx.reply.spoken:
            return None
        if not is_metalanguage_babble(ctx.reply.spoken):
            return None
        # 🔴 FIX (live 02.09): planning narration is NEVER a valid
        # answer regardless of user_input — matches legacy
        # ``DialogueNode._check_babble_and_retry`` (vision-pi 02.09:
        # user said «ебани ланудж», no perf keyword, robot read its
        # own plan aloud).
        if is_planning_narration(ctx.reply.spoken):
            prompt = build_babble_retry_prompt(ctx.turn.user_input or "")
            return Verdict(
                kind=VerdictKind.RETRY,
                guard_name=self.name,
                prompt=prompt,
            )
        if not user_wants_performance(ctx.turn.user_input or ""):
            # Promise-only subset always retries regardless of
            # user_input — these phrases are NEVER valid answers.
            # Mirrors the legacy fallback at
            # ``dialogue_node.py:4245-4251`` exactly.
            if not _is_promise_only_babble(ctx.reply.spoken):
                return None
        prompt = build_babble_retry_prompt(ctx.turn.user_input or "")
        return Verdict(
            kind=VerdictKind.RETRY,
            guard_name=self.name,
            prompt=prompt,
        )


__all__ = [
    "ACCEPT",
    "DEFAULT_MAX_SYNTHETIC_RETRIES",
    "BabbleGuard",
    "BabbleRetryDecision",
    "GuardContext",
    "Reply",
    "TurnContext",
    "TurnState",
    "Verdict",
    "VerdictKind",
    "begin_babble_retry",
    "consume_babble_retry",
    "consume_budget",
    "reset_budget",
]
