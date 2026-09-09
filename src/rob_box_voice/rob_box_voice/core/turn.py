"""turn.py — TurnGuards: one place for guard order + retry budget.

Issue #2241 / ADR-0080 §2.4 — extract the **orchestration** of dialogue
guards from :class:`DialogueNode` so the policy is unit-testable without
ROS2, and so the order + budget cannot drift between the two orchestration
sites that used to live in ``dialogue_node.py`` (one in ``_run_turn.finally``
and one in ``_handle_result``).

Scope (this module):

* The unified verdict shape ``Accept | Retry(prompt, guard_name) | Discard(reason)``.
* The :class:`Guard` protocol — one ``evaluate(ctx) -> Optional[Verdict]``
  per policy.
* :class:`TurnGuards` — owns the order list, the budget, and the
  "first verdict wins" rule.
* The default guard implementations:
  - :class:`SystemRegurgitateGuard` (issue #2175),
  - :class:`ToolSkippedGuard` (issue #1777 / #1762),
  - :class:`BabbleGuard` (issue #992 Bug D),
  - :class:`EmbeddedRenardoCodeGuard` (issue #992 Bug C'),
  - :class:`UnbackedActionClaimGuard` (issue #992 Bug E),
  - :class:`PlanningNarrationHardMute` (issue #1882).

The music guard (issue #992 Bug B/C) is NOT implemented here — it lives in
:mod:`rob_box_voice.core.music_guard` (TD-2 extraction, ADR-0021) and is
plugged in by the :class:`DialogueNode` adapter at construction time. See
:class:`TurnGuards.default_for_dialogue_node` for the canonical order
("regurgitate first, music second, the rest after" — see
``dialogue_node.py:3934-3936`` for the legacy prose).

Out of scope (per the issue body):

* Changing the predicate behavior byte-for-byte. Predicates live in
  :mod:`rob_box_voice.core.dialogue_guards` and are imported here.
* The stend (voice-vr 20).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import (
    Any,
    List,
    Optional,
    Protocol,
    Sequence,
    Tuple,
    runtime_checkable,
)

from .dialogue_guards import (
    ActionClaimRule,
    build_babble_retry_prompt,
    build_renardo_code_retry_prompt,
    build_system_regurgitate_retry_prompt,
    build_tool_retry_prompt,
    build_unbacked_action_retry_prompt,
    detect_required_tool,
    detect_unbacked_action_claim,
    extract_renardo_code_lines,
    is_metalanguage_babble,
    is_planning_narration,
    is_system_template_regurgitated,
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
        speech_id: Optional speech-id for log correlation. Not interpreted by
            guards; carried for diagnostics.
    """

    user_input: str
    is_dj_auto: bool = False
    speech_id: Optional[str] = None


@dataclass(frozen=True)
class TurnState:
    """Turn-scoped mutable state the orchestrator consults.

    The orchestrator does NOT mutate this; the caller (DialogueNode) threads
    a fresh ``TurnState`` between ``evaluate`` calls and resets the budget on
    a fresh user-initiated turn. Keeping the type frozen lets us pass it
    through pure functions without copy/clone ceremony.

    Attributes:
        budget_left: Remaining synthetic retries for the current turn.
            ``0`` means the budget is exhausted; :class:`TurnGuards` will
            downgrade any ``Retry`` verdict to ``Accept`` with a warning.
    """

    budget_left: int = DEFAULT_MAX_SYNTHETIC_RETRIES


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
# Guard protocol
# ---------------------------------------------------------------------------


@runtime_checkable
class Guard(Protocol):
    """One policy check.

    Each implementation takes the :class:`GuardContext`, decides whether to
    raise a verdict, and returns either a :class:`Verdict` or ``None`` to
    defer to the next guard.

    Implementations are PURE: they must not mutate the context, dispatch a
    retry turn, touch rclpy, or do I/O. Side effects (logging, counters
    that survive the call) live in :class:`TurnGuards` or in the
    :class:`DialogueNode` adapter.

    Note: ``name`` is read off the instance (``guard.name``) but is NOT a
    protocol member — Python's structural typing doesn't model class
    attributes as protocol members when implementations are
    ``@dataclass(frozen=True)`` (mutable ``name`` attribute against an
    immutable protocol member). The orchestrator reads ``name`` via
    :func:`getattr` defensively and falls back to the class name.
    """

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        ...


# ---------------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------------


@dataclass
class TurnGuards:
    """Owns the guard order, the retry budget, and the "first wins" rule.

    Construction takes the ordered list of guards and the budget ceiling.
    :meth:`evaluate` walks the list, returns the FIRST non-``None`` verdict,
    and decrements ``state.budget_left`` for ``RETRY`` verdicts.

    If the budget is exhausted (``state.budget_left == 0``) and a guard
    raises ``RETRY``, we return :data:`ACCEPT` and emit a warning — the
    model gets one more chance but no further retries. This replaces the
    legacy ``_consume_synthetic_retry`` path that lived inline in
    ``dialogue_node.py``.

    Usage::

        guards = TurnGuards(
            guards=[system_regurgitate, music, tool, babble, renardo, action],
            max_retries=DEFAULT_MAX_SYNTHETIC_RETRIES,
        )
        verdict = guards.evaluate(reply, turn, state)
        if verdict.kind is VerdictKind.RETRY:
            self._dispatch_turn(verdict.prompt, is_synthetic=True, ...)
            return
        if verdict.kind is VerdictKind.DISCARD:
            return  # do NOT publish spoken_text to TTS
        # ACCEPT: publish as-is.

    Args:
        guards: Ordered sequence. Iteration order = priority. The first guard
            that returns a non-``None`` verdict wins; the rest are not
            consulted for this turn.
        max_retries: Synthetic-retry ceiling per turn. Mirrors the legacy
            ``DialogueNode.DEFAULT_SYNTHETIC_RETRIES``. ``0`` disables
            retries entirely (all ``Retry`` verdicts degrade to ``Accept``
            with a warning).
        logger: Optional logger for diagnostic / warning lines. ``None``
            silences the orchestrator — handy for unit tests that don't
            care about log output.
    """

    guards: Sequence[Guard]
    max_retries: int = DEFAULT_MAX_SYNTHETIC_RETRIES
    logger: Optional[logging.Logger] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        # Defensive: empty order is a programmer error. Surface it here so
        # tests don't have to import a separate validator.
        if not self.guards:
            raise ValueError(
                "TurnGuards: guards must be a non-empty sequence "
                "(got empty — an empty order means 'always Accept')."
            )
        if self.max_retries < 0:
            raise ValueError(
                f"TurnGuards: max_retries must be >= 0, got {self.max_retries!r}"
            )

    def evaluate(
        self,
        reply: Reply,
        turn: TurnContext,
        state: TurnState,
    ) -> Verdict:
        """Run the ordered guards; return the first non-None verdict.

        Returns :data:`ACCEPT` if every guard defers (``None``) or if the
        only raised verdict is a ``RETRY`` whose budget has already been
        spent. Returns the verdict unchanged in every other case.
        """
        ctx = GuardContext(reply=reply, turn=turn, state=state)
        for guard in self.guards:
            # ``guard_name`` falls back to the class name when an instance
            # forgot to declare one — defensive against poorly-implemented
            # callers (the protocol doesn't require ``name``).
            guard_label = getattr(guard, "name", None) or type(guard).__name__
            raw = guard.evaluate(ctx)
            if raw is None:
                continue
            if raw.kind is VerdictKind.ACCEPT:
                # A guard that explicitly accepts early (rare). Treat as
                # the final word.
                return raw
            if raw.kind is VerdictKind.DISCARD:
                return raw
            # RETRY — check the budget.
            if state.budget_left <= 0:
                self._log_warning(
                    f"🚦 [turn-guards] budget exhausted; "
                    f"guard={guard_label!r} requested retry but turn "
                    f"already spent {self.max_retries} retries — publishing "
                    "as-is"
                )
                return ACCEPT
            # Budget available. Return the verdict; the caller will
            # mutate state (TurnState is frozen — they replace it with
            # ``replace(state, budget_left=state.budget_left - 1)``).
            return raw
        return ACCEPT

    # -- logging helpers ----------------------------------------------------

    def _log_warning(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.warning(msg)

    def _log_info(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.info(msg)

    def _log_debug(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.debug(msg)


# ---------------------------------------------------------------------------
# Convenience: budget-step helper
# ---------------------------------------------------------------------------


def consume_budget(state: TurnState) -> TurnState:
    """Return a new ``TurnState`` with ``budget_left`` decremented by 1.

    The dialogue_node adapter calls this after dispatching a ``Retry``
    verdict, so the next :meth:`TurnGuards.evaluate` sees the decremented
    budget. Kept separate from :class:`TurnGuards` to preserve
    "orchestrator is pure" — no hidden mutation.
    """
    return replace(state, budget_left=max(0, state.budget_left - 1))


def reset_budget(max_retries: int = DEFAULT_MAX_SYNTHETIC_RETRIES) -> TurnState:
    """Return a fresh ``TurnState`` with the full budget.

    Called by the dialogue_node adapter on a NEW user-initiated turn (not
    on a synthetic retry, not on a DJ auto-transition).
    """
    return TurnState(budget_left=max_retries)


# ===========================================================================
# Default guard implementations
# ===========================================================================


@dataclass(frozen=True)
class SystemRegurgitateGuard:
    """Issue #2175 — MiniMax-M3 regurgitates ``<system>...</system>`` template.

    Fires when ``spoken`` is the FULL regurgitated block (not a серединный
    reference — see :func:`is_system_template_regurgitated`). Returns
    ``RETRY`` with the critical-reminder prompt from
    :func:`build_system_regurgitate_retry_prompt`.

    Must run BEFORE :class:`BabbleGuard` so the regurgitated template does
    not get the babble CRITICAL pasted on top (issue #2175 follow-up).
    """

    name: str = "system_regurgitate"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if ctx.reply.speak_text_real > 0:
            return None
        if not ctx.reply.spoken:
            return None
        if not is_system_template_regurgitated(ctx.reply.spoken):
            return None
        prompt = build_system_regurgitate_retry_prompt(ctx.turn.user_input)
        return Verdict(
            kind=VerdictKind.RETRY,
            guard_name=self.name,
            prompt=prompt,
        )


@dataclass(frozen=True)
class ToolSkippedGuard:
    """Issue #1777 / #1762 — user asked for a specific tool, LLM skipped it.

    Matches :func:`detect_required_tool` patterns (``get_current_time``,
    ``search_web``, ``set_voice``, ``memory_search``, ``faq_search``).
    Returns ``RETRY`` with the critical-reminder prompt from
    :func:`build_tool_retry_prompt`, or ``None`` when the input is not a
    recognised tool request.
    """

    name: str = "tool_skipped"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if ctx.reply.tools_called:
            return None  # tool WAS called; nothing to retry.
        if not ctx.turn.user_input:
            return None
        tool_name = detect_required_tool(ctx.turn.user_input)
        if not tool_name:
            return None
        prompt = build_tool_retry_prompt(ctx.turn.user_input, tool_name)
        if not prompt:
            # Defence-in-depth: unknown tool in allow-list (prompt injection
            # vector). Silent skip.
            return None
        return Verdict(
            kind=VerdictKind.RETRY,
            guard_name=self.name,
            prompt=prompt,
        )


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

    1. The spoken text matches the *promise-only* opener subset
       (:data:`BABBLE_PROMISE_ONLY_OPENERS` — «зачит», «погнали»,
       «устроим», «переключ», «давай-ка»). These are NEVER valid
       answers, regardless of what the user asked for.
    2. The user explicitly asked for a performance
       (:func:`user_wants_performance`).
    3. The LLM is reading its own plan aloud (:func:`is_planning_narration`).

    See :func:`is_metalanguage_babble` for the full detector; this
    guard is a thin wrapper around it that produces the verdict
    shape. Bug fix (issue #2266 / voice-vr 22): the promise-only
    branch used to live inline in
    ``DialogueNode._check_babble_and_retry`` and was missed when
    ``BabbleGuard`` was extracted — without this fallback the guard
    regressed to "fires only on perf requests" and the well-known
    «Погнали!» case (live 30.08) silently passed through to TTS.
    """

    name: str = "babble"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
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


@dataclass(frozen=True)
class EmbeddedRenardoCodeGuard:
    """Issue #992 Bug C' — LLM pasted Renardo code into ``spoken`` instead
    of calling :func:`execute_music_code`.

    Matches :func:`extract_renardo_code_lines`. Returns ``RETRY`` carrying
    the same code back in the prompt so the LLM can call the tool with it.
    """

    name: str = "embedded_renardo_code"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if ctx.reply.tools_called:
            return None  # LLM already called the tool — no nudge.
        if not ctx.reply.spoken:
            return None
        code = extract_renardo_code_lines(ctx.reply.spoken)
        if not code:
            return None
        prompt = build_renardo_code_retry_prompt(code)
        return Verdict(
            kind=VerdictKind.RETRY,
            guard_name=self.name,
            prompt=prompt,
        )


@dataclass(frozen=True)
class UnbackedActionClaimGuard:
    """Issue #992 Bug E — LLM claimed a tool was called but ``tools_called`` is empty.

    Wraps :func:`detect_unbacked_action_claim`. Returns ``RETRY`` with the
    critical-reminder prompt from :func:`build_unbacked_action_retry_prompt`
    when the action-claim detector fires.
    """

    name: str = "unbacked_action_claim"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if not ctx.reply.spoken:
            return None
        rule: Optional[ActionClaimRule] = detect_unbacked_action_claim(
            user_input=ctx.turn.user_input or "",
            spoken=ctx.reply.spoken,
            tools_called=ctx.reply.tools_called,
        )
        if rule is None:
            return None
        prompt = build_unbacked_action_retry_prompt(
            user_input=ctx.turn.user_input or "",
            spoken=ctx.reply.spoken,
            rule=rule,
        )
        return Verdict(
            kind=VerdictKind.RETRY,
            guard_name=self.name,
            prompt=prompt,
        )


@dataclass(frozen=True)
class PlanningNarrationHardMute:
    """Issue #1882 — hard-mute the reply when the LLM reads its own plan aloud.

    Unlike the other guards this one returns ``DISCARD`` (no retry): the
    planning narration is never a valid user-facing reply, but we don't
    want to spend a budget retrying because the next turn will (a) be
    driven by the user's REAL request and (b) not contain planning
    narration. We just drop the reply and let the next legitimate user
    request recover.

    The detector (``is_planning_narration``) only fires when the LLM named
    a tool identifier in snake_case or opened with «юзер» / «пользователь»
    — see its docstring for the rationale.
    """

    name: str = "planning_narration_mute"

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        if not ctx.reply.spoken:
            return None
        if ctx.reply.speak_text_real > 0:
            return None
        if ctx.reply.tools_called:
            return None  # tool was called → not narration, even if it reads odd.
        if not is_planning_narration(ctx.reply.spoken):
            return None
        return Verdict(
            kind=VerdictKind.DISCARD,
            guard_name=self.name,
            reason="planning_narration",
        )


# ---------------------------------------------------------------------------
# Convenience: default order for the dialogue_node
# ---------------------------------------------------------------------------


def default_guards(
    *,
    music_guard: Optional[Any] = None,
    logger: Optional[logging.Logger] = None,
) -> List[Any]:
    """Return the canonical guard list for the dialogue_node.

    Order matches the legacy prose at ``dialogue_node.py:3934-3936``,
    hardened to a real list:

    1. :class:`SystemRegurgitateGuard` — must fire BEFORE babble so the
       regurgitated template doesn't get the babble CRITICAL pasted on top
       (issue #2175 follow-up).
    2. ``music_guard`` — TD-2 extraction from
       :mod:`rob_box_voice.core.music_guard`. Caller passes either a
       :class:`Guard` (already wrapped via :func:`music_guard_adapter`) or
       a raw ``MusicGuard`` instance (we wrap it transparently).
    3. :class:`ToolSkippedGuard` — non-music tool retry (issue #1777).
    4. :class:`BabbleGuard` — metalanguage / planning narration.
    5. :class:`EmbeddedRenardoCodeGuard` — Renardo code in text.
    6. :class:`UnbackedActionClaimGuard` — claimed but didn't call.
    7. :class:`PlanningNarrationHardMute` — hard-mute (DISCARD).

    The dialogue_node adapter passes its already-constructed
    :class:`MusicGuard` instance via ``music_guard=...``. If it is ``None``
    (e.g. in tests), the music slot is omitted — guards 3..7 still produce
    a coherent verdict stream for any non-music reply.
    """
    out: List[Any] = [SystemRegurgitateGuard()]
    if music_guard is not None:
        # If the caller passed a raw MusicGuard, wrap it for them so the
        # orchestrator never sees MusicGuard's surface. If they already
        # passed a Guard (e.g. ``music_guard_adapter(...)`` result), keep
        # it as-is — both are accepted.
        if not hasattr(music_guard, "name") or not callable(
            getattr(music_guard, "evaluate", None)
        ):
            raise TypeError(
                "default_guards(music_guard=...) expects a Guard-like "
                "object (with .name and .evaluate) — wrap raw MusicGuard "
                "instances with rob_box_voice.core.turn.music_guard_adapter"
            )
        out.append(music_guard)
    out.extend([
        ToolSkippedGuard(),
        BabbleGuard(),
        EmbeddedRenardoCodeGuard(),
        UnbackedActionClaimGuard(),
        PlanningNarrationHardMute(),
    ])
    return out


# ---------------------------------------------------------------------------
# MusicGuard adapter — translate the rich MusicGuardVerdict into Verdict
# ---------------------------------------------------------------------------


def music_guard_adapter(
    evaluate_fn: Any,
    *,
    name: str = "music",
) -> Any:
    """Wrap an existing music guard so its verdicts fit the unified
    ``Verdict`` shape.

    The underlying :class:`rob_box_voice.core.music_guard.MusicGuard` does
    NOT implement the :class:`Guard` protocol — its ``evaluate`` signature
    is keyword-only and takes ``dj_enabled`` / ``build_music_retry_prompt``
    / ``build_dj_retry_prompt`` that this layer cannot supply on its own.
    Rather than carry every MusicGuard-specific knob into the orchestrator,
    we let the dialogue_node adapter close over the music guard's instance
    state and pass a small ``Callable[[TurnContext, Reply], MusicGuardVerdict]``
    that the adapter calls with the right pre-bound arguments.

    Adapter mapping:

    * ``SKIP`` / ``SKIP_NOT_APPLICABLE`` — defer to the next guard.
    * ``DJ_RETRY`` / ``USER_RETRY`` — return ``RETRY`` carrying the prompt.
    * ``NUDGE`` — return ``ACCEPT`` (the dialogue_node adapter will speak
      the nudge separately; the orchestrator has no concept of a nudge).
    * ``FORCE_STOP`` — return ``ACCEPT`` (the dialogue_node adapter will
      publish the stop cleanup separately; same reasoning as NUDGE).

    Args:
        evaluate_fn: A callable ``(turn, reply) -> MusicGuardVerdict``
            that the dialogue_node adapter builds by closing over its
            ``MusicGuard`` instance and the prompt builders. Keeps the
            :class:`Guard` protocol orthogonal to MusicGuard's surface.
        name: The orchestrator-visible guard name. Defaults to ``"music"``.

    Example (in :class:`DialogueNode`)::

        self._turn_guards = TurnGuards(
            guards=default_guards(
                music_guard=music_guard_adapter(
                    lambda turn, reply: self._music_guard.evaluate(
                        was_dj_auto=turn.is_dj_auto,
                        user_input=turn.user_input,
                        tools_called=reply.tools_called,
                        dj_enabled=self._dj.state.enabled,
                        build_music_retry_prompt=self._build_music_retry_prompt,
                        build_dj_retry_prompt=self._build_dj_retry_prompt,
                    ),
                ),
            ),
        )
    """

    class _Adapter:
        # Class-level attribute read by the orchestrator (matches what
        # ``@dataclass(frozen=True)`` guards expose via ``name: str = "..."``).
        # ``name`` is captured via default-argument binding because Python's
        # nested-class scope rules do not see the enclosing function's
        # keyword-only argument directly (PEP 227 / late-binding).

        def __init__(self, *, _name: str = "") -> None:
            self._guard_name = _name

        def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
            verdict = evaluate_fn(ctx.turn, ctx.reply)
            kind_str = getattr(getattr(verdict, "kind", None), "value", None)
            if kind_str in ("skip", "skip_not_applicable"):
                return None  # defer
            if kind_str in ("dj_retry", "user_retry"):
                return Verdict(
                    kind=VerdictKind.RETRY,
                    guard_name=self._guard_name,
                    prompt=verdict.prompt,
                )
            # nudge, force_stop — handled by the dialogue_node adapter;
            # the orchestrator returns Accept so the turn flow continues
            # normally and the adapter gets a chance to publish the
            # nudge / stop separately.
            return ACCEPT

    # Expose ``name`` as a class attribute so the orchestrator's
    # ``getattr(guard, "name", type(guard).__name__)`` resolves it
    # BEFORE instantiation. Built once via ``type()`` to set the class
    # attribute without fighting nested-class scoping.
    Adapter = type(
        "_Adapter",
        (_Adapter,),
        {"name": name},
    )
    return Adapter(_name=name)


# ---------------------------------------------------------------------------
# Re-export the small surface used by dialogue_node tests
# ---------------------------------------------------------------------------


__all__ = [
    "ACCEPT",
    "DEFAULT_MAX_SYNTHETIC_RETRIES",
    "Guard",
    "GuardContext",
    "Reply",
    "ToolSkippedGuard",
    "BabbleGuard",
    "EmbeddedRenardoCodeGuard",
    "UnbackedActionClaimGuard",
    "PlanningNarrationHardMute",
    "SystemRegurgitateGuard",
    "TurnContext",
    "TurnGuards",
    "TurnState",
    "Verdict",
    "VerdictKind",
    "consume_budget",
    "default_guards",
    "music_guard_adapter",
    "reset_budget",
]
