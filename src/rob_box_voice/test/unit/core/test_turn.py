"""Unit tests for :mod:`rob_box_voice.core.turn` — babble-ретрай и бюджет.

ADR-0148 §2.4 / #3312: тесты оркестратора ``TurnGuards`` и гуардов, которые
использовал только он, удалены вместе с кодом. Остаётся живая поверхность:
``TurnState`` / ``reset_budget`` / ``consume_*`` / ``BabbleGuard`` /
``begin_babble_retry``.
"""

from __future__ import annotations

from rob_box_voice.core.turn import (
    DEFAULT_MAX_SYNTHETIC_RETRIES,
    BabbleGuard,
    BabbleRetryDecision,
    GuardContext,
    Reply,
    TurnContext,
    TurnState,
    VerdictKind,
    begin_babble_retry,
    consume_babble_retry,
    consume_budget,
    reset_budget,
)


# ---------------------------------------------------------------------------
# Test fixtures / fake guard
# ---------------------------------------------------------------------------


def _reply(
    spoken: str = "",
    tools_called: tuple = (),
    speak_text_real: int = 0,
) -> Reply:
    return Reply(
        spoken=spoken,
        tools_called=tools_called,
        speak_text_real=speak_text_real,
    )


def _turn(
    user_input: str = "",
    is_dj_auto: bool = False,
    tool_error_occurred: bool = False,
) -> TurnContext:
    return TurnContext(
        user_input=user_input,
        is_dj_auto=is_dj_auto,
        tool_error_occurred=tool_error_occurred,
    )


def _state(budget_left: int = DEFAULT_MAX_SYNTHETIC_RETRIES) -> TurnState:
    return TurnState(budget_left=budget_left)


# ---------------------------------------------------------------------------
# 1. Construction validation
# ---------------------------------------------------------------------------


class TestBudgetHelpers:
    def test_consume_budget_decrements(self) -> None:
        s0 = TurnState(budget_left=3)
        s1 = consume_budget(s0)
        assert s1.budget_left == 2
        assert s0.budget_left == 3  # original is frozen, untouched

    def test_consume_budget_clamps_at_zero(self) -> None:
        s0 = TurnState(budget_left=0)
        s1 = consume_budget(s0)
        assert s1.budget_left == 0

    def test_reset_budget_returns_fresh_state(self) -> None:
        s0 = reset_budget()
        assert s0.budget_left == DEFAULT_MAX_SYNTHETIC_RETRIES
        s1 = reset_budget(max_retries=5)
        assert s1.budget_left == 5


# ---------------------------------------------------------------------------
# 5. Real default guards — smoke tests on canned inputs
# ---------------------------------------------------------------------------


class TestBabbleGuard:
    def test_fires_on_babble_with_performance_intent(self) -> None:
        g = BabbleGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Зачитаю рэпчик про космос!"),
                turn=_turn(user_input="зачитай рэп про космос"),
                state=_state(),
            )
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "babble"

    def test_fires_on_planning_narration(self) -> None:
        g = BabbleGuard()
        # is_planning_narration opens with «юзер» / snake_case identifier
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="юзер хочет узнать время, надо вызвать get_current_time"),
                turn=_turn(user_input="сколько времени"),
                state=_state(),
            )
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY

    def test_defers_on_normal_text(self) -> None:
        g = BabbleGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Сейчас 12:34."),
                turn=_turn(user_input="сколько времени"),
                state=_state(),
            )
        )
        assert v is None

    # ----- issue #2948: successful fulfilling tools ⇒ not babble -----

    def test_defers_when_fulfilling_tools_succeeded(self) -> None:
        """Live 24.09.2026: ``tools=['set_dj_mode', 'compose_music',
        'play_animation']``, ``spoken='Слушай Still Dre, потом Next
        Episode подхвачу!'`` — the DJ request was ALREADY fulfilled by
        real tool calls. «Слушай » matches ``BABBLE_BANNED_OPENERS`` and
        the user's DJ-set request matches ``user_wants_performance``, so
        before the fix the guard fired unconditionally and re-dispatched
        the ORIGINAL request — starting the DJ set a second time. A
        short DJ-style line next to a completed action is not babble.
        """
        g = BabbleGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(
                    spoken="Слушай Still Dre, потом Next Episode подхвачу!",
                    tools_called=(
                        "set_dj_mode", "compose_music", "play_animation",
                    ),
                ),
                turn=_turn(
                    user_input=(
                        "Ты диджей Снупдог. Играй по очереди: Still Dre, "
                        "потом Next Episode."
                    ),
                ),
                state=_state(),
            )
        )
        assert v is None, (
            "babble guard must NOT retry a turn that already ran the "
            "tools fulfilling the request (issue #2948 — DJ set started "
            "twice live)"
        )

    def test_fires_when_fulfilling_tool_call_errored(self) -> None:
        """The bypass requires SUCCESS, not just a matching tool NAME
        (same bar as issue #2949): a called-but-errored fulfilling tool
        must not suppress the babble retry either.
        """
        g = BabbleGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(
                    spoken="Слушай Still Dre, потом Next Episode подхвачу!",
                    tools_called=("set_dj_mode", "compose_music"),
                ),
                turn=_turn(
                    user_input=(
                        "Ты диджей Снупдог. Играй по очереди: Still Dre, "
                        "потом Next Episode."
                    ),
                    tool_error_occurred=True,
                ),
                state=_state(),
            )
        )
        assert v is not None, (
            "a fulfilling tool that was called but ERRORED must not "
            "bypass the babble guard"
        )
        assert v.kind is VerdictKind.RETRY

    def test_still_fires_on_babble_with_no_tools_called(self) -> None:
        """No regression: the classic Bug D case (tools=()) still retries."""
        g = BabbleGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Слушай, сейчас устроим!"),
                turn=_turn(user_input="сыграй рэп"),
                state=_state(),
            )
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY


class TestBabbleRetryConsumedFlag:
    """``TurnState.babble_retry_consumed`` enforces the one-shot rule."""

    def test_fresh_state_has_flag_false(self) -> None:
        """Default ``TurnState`` must start with ``babble_retry_consumed=False``.

        Mirrors the legacy ``self._babble_retry_used = False`` initializer
        in ``DialogueNode.__init__``.
        """
        state = reset_budget()
        assert state.babble_retry_consumed is False

    def test_consume_babble_retry_flips_flag(self) -> None:
        """``consume_babble_retry`` is the canonical write path.

        Same shape as :func:`consume_budget` — returns a new
        ``TurnState``, original is unchanged (frozen dataclass).
        """
        state = reset_budget()
        consumed = consume_babble_retry(state)
        assert consumed.babble_retry_consumed is True
        assert state.babble_retry_consumed is False  # immutability

    def test_consume_babble_retry_is_idempotent(self) -> None:
        """Calling ``consume_babble_retry`` twice yields the same state.

        Not strictly required by the contract (the field is a boolean
        and the orchestrator treats ``True`` as "already fired"), but
        pinning the behaviour avoids accidental regressions in a
        follow-up card.
        """
        once = consume_babble_retry(reset_budget())
        twice = consume_babble_retry(once)
        assert once.babble_retry_consumed is True
        assert twice.babble_retry_consumed is True

    def test_reset_budget_clears_flag(self) -> None:
        """A fresh user-initiated turn MUST reset ``babble_retry_consumed``.

        Mirrors the legacy ``self._babble_retry_used = False`` write at
        the top of ``_run_turn``. Without this, a second babble reply
        on a NEW user turn would silently pass through to TTS even if
        the user gave a new performance command.
        """
        consumed = consume_babble_retry(reset_budget())
        assert consumed.babble_retry_consumed is True
        fresh = reset_budget()
        assert fresh.babble_retry_consumed is False

    def test_babble_guard_defers_when_already_consumed(self) -> None:
        """BabbleGuard returns ``None`` once the one-shot flag is set.

        Mirrors the legacy ``dialogue_node.py:4232-4233`` short-circuit
        that read ``self._babble_retry_used`` before doing any work.
        Without this, a second babble reply in the same turn would loop
        forever — see
        ``test_issue_992_babble_guard.py::test_retry_only_fires_once_*``.
        """
        guard = BabbleGuard()
        ctx = GuardContext(
            reply=_reply(spoken="Зачитаю рэпчик про космос!"),
            turn=_turn(user_input="зачитай рэп про космос"),
            state=TurnState(budget_left=3, babble_retry_consumed=True),
        )
        assert guard.evaluate(ctx) is None

    def test_babble_guard_fires_when_flag_is_false(self) -> None:
        """Sanity: BabbleGuard still raises RETRY on a fresh turn.

        The new ``babble_retry_consumed`` check MUST be additive — it
        does NOT replace the detector. A fresh ``TurnState`` with the
        default ``babble_retry_consumed=False`` must keep the existing
        behaviour byte-for-byte.
        """
        guard = BabbleGuard()
        ctx = GuardContext(
            reply=_reply(spoken="Зачитаю рэпчик про космос!"),
            turn=_turn(user_input="зачитай рэп про космос"),
            state=reset_budget(),  # babble_retry_consumed defaults to False
        )
        verdict = guard.evaluate(ctx)
        assert verdict is not None
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "babble"

    def test_full_lifecycle_via_babble_guard(self) -> None:
        """RETRY → consume → second babble → None (проходит как есть) (the bug-992 main path).

        End-to-end on the bare ``core/`` surface: a babble reply fires a
        RETRY, the adapter applies ``consume_babble_retry`` +
        ``consume_budget``, the retry turn babbles again — and now BOTH
        guards agree the second reply passes through. This is exactly
        what ``test_issue_992_babble_guard.py::test_retry_only_fires_once_*``
        asserts through the heavy harness; the ``core/`` version proves
        the policy doesn't depend on it.
        """
        guard = BabbleGuard()
        state = reset_budget()
        spoken = "Зачитаю рэпчик про космос!"
        user_input = "зачитай рэп про космос"

        # First turn — original babble. RETRY.
        v1 = guard.evaluate(
            GuardContext(_reply(spoken=spoken), _turn(user_input=user_input), state)
        )
        assert v1.kind is VerdictKind.RETRY
        state = consume_budget(state)
        state = consume_babble_retry(state)

        # Second turn — the retry also babbles. ACCEPT, no third LLM call.
        v2 = guard.evaluate(
            GuardContext(_reply(spoken=spoken), _turn(user_input=user_input), state)
        )
        assert v2 is None


# ---------------------------------------------------------------------------
# 11. Issue #2266 — ``begin_babble_retry`` is the bare ``core/`` surface
#     that owns the babble policy + the budget step + the one-shot flag.
#     The dialogue_node adapter just performs the ROS-bound side effects
#     using the returned ``BabbleRetryDecision``. These tests exercise the
#     pure helper in isolation — no DialogueNode, no harness, no rclpy.
# ---------------------------------------------------------------------------


class TestBeginBabbleRetry:
    """``begin_babble_retry`` — the pure adapter helper.

    Proves the DoD contract for issue #2266:

    * ``BabbleRetryDecision`` carries the synthetic prompt, the
      post-mutation :class:`TurnState` (budget decremented AND one-shot
      flag set), and a reason tag.
    * Returns ``None`` whenever BabbleGuard defers (clean reply,
      speak_text_real > 0, empty spoken, second babble in the same
      turn) — the caller publishes the original ``spoken`` as-is.
    * Returns ``None`` when the budget is already exhausted — the
      caller sees it as "no retry, no budget", mirrors the legacy
      ``_consume_synthetic_retry`` False return.
    * Pure: the input ``TurnState`` is never mutated.
    """

    def test_returns_none_on_normal_text(self) -> None:
        """Plain text + speak_text_real=0 + perf command — but no babble opener."""
        decision = begin_babble_retry(
            spoken="Сейчас 12:34.",
            user_input="сколько времени",
            tools_called=(),
            speak_text_real=0,
            state=_state(),
        )
        assert decision is None

    def test_returns_none_when_speak_text_real_positive(self) -> None:
        """``speak_text_real > 0`` ⇒ no retry (issue-988 anti-duplicate contract)."""
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=("speak_text",),
            speak_text_real=1,
            state=_state(),
        )
        assert decision is None

    def test_returns_none_when_spoken_empty(self) -> None:
        """Empty reply + perf command — no retry."""
        decision = begin_babble_retry(
            spoken="",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=_state(),
        )
        assert decision is None

    def test_returns_decision_for_babble_with_perf_intent(self) -> None:
        """«Зачитаю рэп» + perf command → RETRY decision."""
        state = _state()
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert isinstance(decision, BabbleRetryDecision)
        assert "зачитай рэп про космос" in decision.prompt
        assert decision.reason == "babble_guard"

    def test_decision_decrements_budget(self) -> None:
        """The returned ``new_state`` has ``budget_left - 1``."""
        state = _state(budget_left=3)
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert decision is not None
        assert decision.new_state.budget_left == 2

    def test_decision_sets_babble_retry_consumed(self) -> None:
        """The returned ``new_state`` has ``babble_retry_consumed=True``."""
        state = _state()
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert decision is not None
        assert decision.new_state.babble_retry_consumed is True

    def test_pure_does_not_mutate_input_state(self) -> None:
        """Input ``TurnState`` is never mutated — pure helper.

        ``TurnState`` is a frozen dataclass (immutable), so this is
        belt-and-braces: even if the helper tried to mutate, the
        dataclass would raise ``FrozenInstanceError``. The test pins
        the contract — if anyone refactors ``TurnState`` to be mutable,
        the helper MUST keep its purity.
        """
        state = _state(budget_left=2)
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert decision is not None
        # Input untouched.
        assert state.budget_left == 2
        assert state.babble_retry_consumed is False

    def test_returns_none_when_budget_exhausted(self) -> None:
        """``budget_left == 0`` ⇒ no retry, even if a guard asks.

        Mirrors the legacy ``_consume_synthetic_retry`` False return:
        budget is the cross-guard ceiling and babble respects it.
        """
        state = _state(budget_left=0)
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert decision is None

    def test_returns_none_when_one_shot_already_consumed(self) -> None:
        """One-shot rule: second babble in the same turn ⇒ no retry.

        The flag is checked by :class:`BabbleGuard` itself
        (``ctx.state.babble_retry_consumed`` short-circuit). Without
        this, the LLM and the guard ping-pong. The helper honours the
        guard's deferral without consuming the budget — ``state`` is
        untouched when no retry fires.
        """
        state = TurnState(budget_left=3, babble_retry_consumed=True)
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert decision is None
        # State untouched (no budget consume, no flag write).
        assert state.budget_left == 3
        assert state.babble_retry_consumed is True

    def test_planning_narration_retries_regardless_of_user_input(self) -> None:
        """Planning narration (live 02.09 «Юзер просит…, запускаю…») → RETRY.

        Even when ``user_input`` carries no performance keyword, the
        detector MUST retry — mirrors the live failure that drove the
        02.09 fix (``user_wants_perf=False`` + ``is_planning_narration``
        path in legacy ``_check_babble_and_retry``).
        """
        decision = begin_babble_retry(
            spoken="юзер хочет узнать время, вызываю get_current_time",
            user_input="сколько времени",
            tools_called=(),
            speak_text_real=0,
            state=_state(),
        )
        assert decision is not None
        assert decision.new_state.babble_retry_consumed is True

    def test_promise_only_opener_retries_without_perf_keyword(self) -> None:
        """«Погнали!» — pure promise — retries even on a non-perf turn.

        Mirrors the live 30.08 «Погнали!» bug — the user asked a
        non-perf question but the opener forces a retry.
        """
        decision = begin_babble_retry(
            spoken="Погнали!",
            user_input="расскажи про себя",
            tools_called=(),
            speak_text_real=0,
            state=_state(),
        )
        assert decision is not None
        assert decision.new_state.babble_retry_consumed is True

    def test_lifecycle_first_babble_retry_then_silence(self) -> None:
        """End-to-end on the bare ``core/`` surface.

        Two ``begin_babble_retry`` calls in a row, simulating the
        first user turn (babble) and the synthetic retry turn (also
        babbles). The second call MUST return ``None`` because the
        one-shot flag is set on the returned ``new_state``.

        Proves the cross-guard one-shot invariant that
        ``test_issue_992_babble_guard.py::test_retry_only_fires_once_*``
        checks via the heavy harness; the ``core/`` version proves the
        policy doesn't depend on rclpy / harness / DialogueNode.
        """
        spoken = "Зачитаю рэпчик про космос!"
        user_input = "зачитай рэп про космос"
        state = _state(budget_left=3)

        # First call — original babble. RETRY decision.
        d1 = begin_babble_retry(
            spoken=spoken,
            user_input=user_input,
            tools_called=(),
            speak_text_real=0,
            state=state,
        )
        assert d1 is not None
        assert d1.new_state.budget_left == 2
        assert d1.new_state.babble_retry_consumed is True

        # Second call — synthetic retry also babbles. None, no
        # third LLM call. State untouched (no budget consume, no flag
        # write — the guard defers BEFORE either happens).
        d2 = begin_babble_retry(
            spoken=spoken,
            user_input=user_input,
            tools_called=(),
            speak_text_real=0,
            state=d1.new_state,
        )
        assert d2 is None
        assert d1.new_state.budget_left == 2

    def test_prompt_is_synthetic_critical_with_user_input_echo(self) -> None:
        """The returned ``prompt`` is the synthetic CRITICAL reminder.

        Pin down the prompt shape — the legacy dialogue_node used
        ``build_babble_retry_prompt(user_input)`` and we preserve that.
        """
        decision = begin_babble_retry(
            spoken="Зачитаю рэпчик про космос!",
            user_input="зачитай рэп про космос",
            tools_called=(),
            speak_text_real=0,
            state=_state(),
        )
        assert decision is not None
        # The prompt must echo the original user command — that's how
        # the LLM knows what it was asked to do.
        assert "зачитай рэп про космос" in decision.prompt
        # And it must carry the CRITICAL reminder — that's how the
        # LLM is told "don't promise, perform".
        assert (
            "CRITICAL" in decision.prompt
            or "критич" in decision.prompt.lower()
            or "песн" in decision.prompt.lower()
            or "промпт" in decision.prompt.lower()
        )


# `TestBabbleRetryConsumedFlag` doesn't inherit `TestBabbleIntegrationViaTurnGuards`
# (which is a self-contained class) — pull `_reply`, `_turn`, `_state`
# locally so the new tests can use them too.
