"""Unit tests for :mod:`rob_box_voice.core.turn` — TurnGuards orchestrator.

Issue #2241 / ADR-0080 §2.4. These tests exercise the orchestration contract
without spinning up a ``DialogueNode``:

* ``TurnGuards.evaluate`` returns the FIRST non-None verdict and ignores
  the rest (the "one retry per turn" rule that used to require every
  ``_check_*_and_retry`` to call ``_mark_retry_dispatched``).
* Budget is enforced: ``state.budget_left == 0`` plus a ``Retry`` verdict
  degrades to ``Accept`` with a warning.
* Fresh ``TurnState`` resets the budget.
* Default guards (``SystemRegurgitateGuard``, ``ToolSkippedGuard``,
  ``BabbleGuard``, ``EmbeddedRenardoCodeGuard``,
  ``UnbackedActionClaimGuard``, ``PlanningNarrationHardMute``) produce
  the expected verdict shape on canned inputs.
* ``music_guard_adapter`` translates ``MusicGuardVerdict`` into the
  unified ``Verdict`` shape.

Pure-Python — no ROS2 node required.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional

import pytest

from rob_box_voice.core.turn import (
    ACCEPT,
    DEFAULT_MAX_SYNTHETIC_RETRIES,
    BabbleGuard,
    EmbeddedRenardoCodeGuard,
    Guard,
    GuardContext,
    PlanningNarrationHardMute,
    Reply,
    SystemRegurgitateGuard,
    ToolSkippedGuard,
    TurnContext,
    TurnGuards,
    TurnState,
    UnbackedActionClaimGuard,
    Verdict,
    VerdictKind,
    consume_budget,
    default_guards,
    music_guard_adapter,
    reset_budget,
)


# ---------------------------------------------------------------------------
# Test fixtures / fake guard
# ---------------------------------------------------------------------------


class _StaticGuard:
    """Returns a fixed verdict (or None). Used to test orchestrator rules.

    Implements the ``Guard`` protocol structurally — no inheritance needed.
    """

    def __init__(
        self,
        name: str,
        verdict: Optional[Verdict],
    ) -> None:
        self.name = name
        self._verdict = verdict

    def evaluate(self, ctx: GuardContext) -> Optional[Verdict]:
        return self._verdict


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
) -> TurnContext:
    return TurnContext(user_input=user_input, is_dj_auto=is_dj_auto)


def _state(budget_left: int = DEFAULT_MAX_SYNTHETIC_RETRIES) -> TurnState:
    return TurnState(budget_left=budget_left)


# ---------------------------------------------------------------------------
# 1. Construction validation
# ---------------------------------------------------------------------------


class TestConstruction:
    def test_empty_guards_sequence_raises(self) -> None:
        with pytest.raises(ValueError, match="non-empty"):
            TurnGuards(guards=[])

    def test_negative_max_retries_raises(self) -> None:
        with pytest.raises(ValueError, match=">= 0"):
            TurnGuards(guards=[_StaticGuard("a", None)], max_retries=-1)


# ---------------------------------------------------------------------------
# 2. Order: first verdict wins
# ---------------------------------------------------------------------------


class TestOrderFirstVerdictWins:
    def test_first_retry_wins_second_guard_not_consulted(self) -> None:
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="from-a"),
        )
        b = _StaticGuard(
            "b",
            Verdict(kind=VerdictKind.RETRY, guard_name="b", prompt="from-b"),
        )
        guards = TurnGuards(guards=[a, b])
        verdict = guards.evaluate(_reply(), _turn(), _state())
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "a"
        assert verdict.prompt == "from-a"

    def test_first_none_defers_to_second(self) -> None:
        a = _StaticGuard("a", None)
        b = _StaticGuard(
            "b",
            Verdict(kind=VerdictKind.DISCARD, guard_name="b", reason="x"),
        )
        guards = TurnGuards(guards=[a, b])
        verdict = guards.evaluate(_reply(), _turn(), _state())
        assert verdict.kind is VerdictKind.DISCARD
        assert verdict.guard_name == "b"
        assert verdict.reason == "x"

    def test_all_defer_returns_accept(self) -> None:
        guards = TurnGuards(
            guards=[_StaticGuard("a", None), _StaticGuard("b", None)],
        )
        verdict = guards.evaluate(_reply(), _turn(), _state())
        assert verdict is ACCEPT

    def test_discard_short_circuits_after_retry(self) -> None:
        """A RETRY followed by a DISCARD in order → RETRY wins (first wins)."""
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="p"),
        )
        b = _StaticGuard(
            "b",
            Verdict(kind=VerdictKind.DISCARD, guard_name="b", reason="x"),
        )
        guards = TurnGuards(guards=[a, b])
        verdict = guards.evaluate(_reply(), _turn(), _state())
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "a"

    def test_explicit_accept_short_circuits(self) -> None:
        a = _StaticGuard("a", ACCEPT)
        b = _StaticGuard(
            "b",
            Verdict(kind=VerdictKind.RETRY, guard_name="b", prompt="p"),
        )
        guards = TurnGuards(guards=[a, b])
        verdict = guards.evaluate(_reply(), _turn(), _state())
        assert verdict.kind is VerdictKind.ACCEPT


# ---------------------------------------------------------------------------
# 3. Budget enforcement
# ---------------------------------------------------------------------------


class TestBudgetEnforced:
    def test_zero_budget_with_retry_returns_accept(self) -> None:
        """Budget exhausted: guard wants retry, orchestrator downgrades."""
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="p"),
        )
        guards = TurnGuards(guards=[a])
        verdict = guards.evaluate(_reply(), _turn(), _state(budget_left=0))
        assert verdict.kind is VerdictKind.ACCEPT

    def test_budget_exhaustion_logs_warning(self, caplog) -> None:
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="p"),
        )
        logger = logging.getLogger("test.turn.budget")
        guards = TurnGuards(guards=[a], logger=logger)
        with caplog.at_level(logging.WARNING, logger="test.turn.budget"):
            guards.evaluate(_reply(), _turn(), _state(budget_left=0))
        assert any(
            "budget exhausted" in record.message for record in caplog.records
        ), f"expected budget-exhausted warning, got: {caplog.records}"

    def test_negative_budget_also_blocks_retry(self) -> None:
        """Defence-in-depth: negative budget (corrupt state) → Accept."""
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="p"),
        )
        guards = TurnGuards(guards=[a])
        verdict = guards.evaluate(_reply(), _turn(), _state(budget_left=-1))
        assert verdict.kind is VerdictKind.ACCEPT

    def test_full_budget_allows_retry(self) -> None:
        a = _StaticGuard(
            "a",
            Verdict(kind=VerdictKind.RETRY, guard_name="a", prompt="p"),
        )
        guards = TurnGuards(guards=[a])
        verdict = guards.evaluate(_reply(), _turn(), _state(budget_left=3))
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.prompt == "p"


# ---------------------------------------------------------------------------
# 4. Budget helpers
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


class TestSystemRegurgitateGuard:
    def test_fires_on_full_regurgitated_template(self) -> None:
        # An arbitrary-looking but structurally regurgitated template.
        # We rely on the existing predicate; if it fires for this input,
        # the guard wraps it correctly. The detailed predicate behavior
        # is tested in test_dialogue_guards.py.
        g = SystemRegurgitateGuard()
        # Use a known pattern that triggers the predicate (see issue #2175
        # in dialogue_guards.py for fixture examples).
        spoken = (
            "<system>Ты — робот, отвечай кратко. Сейчас "
            "выполни запрос пользователя.</system>"
        )
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken=spoken),
                turn=_turn(user_input="погнали"),
                state=_state(),
            )
        )
        # Bug fix (review, voice-vr 19): this was `if v is not None: assert
        # ...` — a phantom test that passes even if the guard never fires.
        # Assert unconditionally so a regression in
        # ``is_system_template_regurgitated`` actually fails this test.
        assert v is not None, "guard did not fire on a full regurgitated template"
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "system_regurgitate"
        assert v.prompt and "CRITICAL" in v.prompt

    def test_defers_on_normal_reply(self) -> None:
        g = SystemRegurgitateGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Привет, человек!"),
                turn=_turn(user_input="привет"),
                state=_state(),
            )
        )
        assert v is None

    def test_defers_when_speak_text_real_nonzero(self) -> None:
        """If the LLM already voiced real text, the regurgitate guard
        doesn't fire — there's nothing to retry."""
        g = SystemRegurgitateGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(
                    spoken="<system>...</system>",
                    speak_text_real=1,
                ),
                turn=_turn(user_input="x"),
                state=_state(),
            )
        )
        assert v is None


class TestToolSkippedGuard:
    def test_fires_when_user_asked_for_time(self) -> None:
        g = ToolSkippedGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Не знаю, который час."),
                turn=_turn(user_input="сколько времени?"),
                state=_state(),
            )
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "tool_skipped"

    def test_defers_when_tool_was_called(self) -> None:
        g = ToolSkippedGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Сейчас 12:34.", tools_called=("get_current_time",)),
                turn=_turn(user_input="сколько времени?"),
                state=_state(),
            )
        )
        assert v is None

    def test_defers_when_no_tool_request(self) -> None:
        g = ToolSkippedGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Привет!"),
                turn=_turn(user_input="расскажи анекдот"),
                state=_state(),
            )
        )
        assert v is None


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


class TestEmbeddedRenardoCodeGuard:
    def test_fires_on_renardo_code_in_text(self) -> None:
        g = EmbeddedRenardoCodeGuard()
        # Bug fix (review, voice-vr 19): the original fixture
        # (`a = Synth(...)`) never matched `_RENARDO_CODE_LINE_RE` in
        # dialogue_guards.py — that regex only recognises Renardo
        # player-line syntax (``p1 >> ...``), ``Clock.bpm =``, or
        # ``Scale.default`` / ``Root.default`` assignments. The old
        # fixture made ``extract_renardo_code_lines`` return None, and the
        # test's `if v is not None: assert ...` swallowed that silently —
        # a phantom test that passed even with the guard entirely broken.
        spoken = (
            "Вот код:\n"
            "p1 >> blip([0, 2, 4])\n"
            "Clock.bpm = 120\n"
            "Круто!"
        )
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken=spoken),
                turn=_turn(user_input="сыграй мелодию"),
                state=_state(),
            )
        )
        assert v is not None, "guard did not fire on embedded Renardo code"
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "embedded_renardo_code"
        assert v.prompt and "p1 >> blip" in v.prompt

    def test_defers_when_no_code(self) -> None:
        g = EmbeddedRenardoCodeGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Конечно, сыграю!"),
                turn=_turn(user_input="сыграй мелодию"),
                state=_state(),
            )
        )
        assert v is None


class TestUnbackedActionClaimGuard:
    def test_fires_on_unbacked_claim(self) -> None:
        g = UnbackedActionClaimGuard()
        # Bug fix (review, voice-vr 19): "сыграй мелодию" / "Вот, играю
        # мелодию!" matches none of the ``ACTION_CLAIM_RULES`` in
        # dialogue_guards.py (music isn't one of the covered categories —
        # waypoint_save, waypoint_delete, track_delete, library_search,
        # ...), so ``detect_unbacked_action_claim`` always returned None
        # here and the `if v is not None: assert ...` never executed —
        # a phantom test. Use the waypoint_save rule, which is covered.
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Точка сохранена!"),
                turn=_turn(user_input="запомни эту точку"),
                state=_state(),
            )
        )
        assert v is not None, "guard did not fire on an unbacked action claim"
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "unbacked_action_claim"

    def test_defers_when_tool_was_called(self) -> None:
        g = UnbackedActionClaimGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(
                    spoken="Вот, играю мелодию!",
                    tools_called=("execute_music_code",),
                ),
                turn=_turn(user_input="сыграй мелодию"),
                state=_state(),
            )
        )
        assert v is None


class TestPlanningNarrationHardMute:
    def test_returns_discard_on_planning_narration(self) -> None:
        g = PlanningNarrationHardMute()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="юзер хочет узнать время"),
                turn=_turn(user_input="сколько времени"),
                state=_state(),
            )
        )
        # Bug fix (review, voice-vr 19): tighten from the softened
        # `if v is not None: assert ...` (this one did fire, but the
        # conditional would silently mask a future regression).
        assert v is not None, "guard did not fire on planning narration"
        assert v.kind is VerdictKind.DISCARD
        assert v.guard_name == "planning_narration_mute"
        assert v.reason == "planning_narration"

    def test_defers_when_tool_called(self) -> None:
        """If a tool was actually called, it's NOT planning narration even
        if the text reads oddly."""
        g = PlanningNarrationHardMute()
        v = g.evaluate(
            GuardContext(
                reply=_reply(
                    spoken="юзер хочет узнать время, вызываю get_current_time",
                    tools_called=("get_current_time",),
                ),
                turn=_turn(user_input="сколько времени"),
                state=_state(),
            )
        )
        assert v is None


# ---------------------------------------------------------------------------
# 6. Music guard adapter
# ---------------------------------------------------------------------------


@dataclass
class _FakeMusicVerdict:
    """Shape-compatible stub for MusicGuard's verdict."""

    kind: "_FakeMusicKind"
    prompt: Optional[str] = None


@dataclass
class _FakeMusicKind:
    value: str


class TestMusicGuardAdapter:
    def test_skip_returns_none(self) -> None:
        """SKIP / SKIP_NOT_APPLICABLE → defer to next guard."""

        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="skip"),
                prompt=None,
            )

        adapter = music_guard_adapter(_fake_evaluate, name="music")
        v = adapter.evaluate(
            GuardContext(
                reply=_reply(),
                turn=_turn(),
                state=_state(),
            )
        )
        assert v is None

    def test_skip_not_applicable_returns_none(self) -> None:
        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="skip_not_applicable"),
                prompt=None,
            )

        adapter = music_guard_adapter(_fake_evaluate)
        v = adapter.evaluate(
            GuardContext(reply=_reply(), turn=_turn(), state=_state()),
        )
        assert v is None

    def test_dj_retry_translates_to_retry_verdict(self) -> None:
        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="dj_retry"),
                prompt="DJ prompt",
            )

        adapter = music_guard_adapter(_fake_evaluate)
        v = adapter.evaluate(
            GuardContext(reply=_reply(), turn=_turn(is_dj_auto=True), state=_state()),
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "music"
        assert v.prompt == "DJ prompt"

    def test_user_retry_translates_to_retry_verdict(self) -> None:
        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="user_retry"),
                prompt="user prompt",
            )

        adapter = music_guard_adapter(_fake_evaluate)
        v = adapter.evaluate(
            GuardContext(reply=_reply(), turn=_turn(), state=_state()),
        )
        assert v is not None
        assert v.kind is VerdictKind.RETRY
        assert v.prompt == "user prompt"

    def test_nudge_translates_to_accept(self) -> None:
        """Nudge is handled by the dialogue_node adapter; orchestrator says Accept."""

        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="nudge"),
                prompt="nudge prompt",
            )

        adapter = music_guard_adapter(_fake_evaluate)
        v = adapter.evaluate(
            GuardContext(reply=_reply(), turn=_turn(), state=_state()),
        )
        assert v is ACCEPT

    def test_force_stop_translates_to_accept(self) -> None:
        def _fake_evaluate(turn, reply):
            return _FakeMusicVerdict(
                kind=_FakeMusicKind(value="force_stop"),
                prompt=None,
            )

        adapter = music_guard_adapter(_fake_evaluate)
        v = adapter.evaluate(
            GuardContext(reply=_reply(), turn=_turn(), state=_state()),
        )
        assert v is ACCEPT


# ---------------------------------------------------------------------------
# 7. default_guards() factory
# ---------------------------------------------------------------------------


class TestDefaultGuardsFactory:
    def test_default_order_without_music(self) -> None:
        guards = default_guards()
        names = [getattr(g, "name", type(g).__name__) for g in guards]
        assert names[0] == "system_regurgitate"
        assert "tool_skipped" in names
        assert "babble" in names
        assert "embedded_renardo_code" in names
        assert "unbacked_action_claim" in names
        assert "planning_narration_mute" in names
        # Music is optional — must NOT appear when not passed.
        assert "music" not in names

    def test_default_order_with_music_inserts_after_system_regurgitate(self) -> None:
        # Caller must wrap a raw MusicGuard via music_guard_adapter —
        # default_guards() does not know the prompt-builder kwargs.
        def _fake_evaluate(turn, reply):
            return None  # any verdict; we only care about ordering here

        adapter = music_guard_adapter(_fake_evaluate)
        guards = default_guards(music_guard=adapter)
        names = [getattr(g, "name", type(g).__name__) for g in guards]
        assert names[0] == "system_regurgitate"
        assert names[1] == "music"
        # Music sits BEFORE babble (legacy ordering requirement).
        assert names.index("music") < names.index("babble")

    def test_raw_music_guard_without_name_raises(self) -> None:
        """Defence: caller passed a raw MusicGuard (not wrapped). Surface
        the configuration error instead of crashing later in evaluate()."""

        class _RawMusicGuard:
            # Mimics MusicGuard: no `.name`, no Guard-protocol `evaluate(ctx)`.
            def evaluate(self, was_dj_auto, user_input, tools_called):
                return None

        with pytest.raises(TypeError, match="music_guard_adapter"):
            default_guards(music_guard=_RawMusicGuard())

    def test_full_pipeline_with_all_guards(self) -> None:
        """Smoke: the full default pipeline runs without exceptions and
        returns a verdict of one of the three kinds."""

        def _stub_evaluate(turn, reply):
            return None  # defer; not a music reply

        guards = TurnGuards(
            guards=default_guards(music_guard=music_guard_adapter(_stub_evaluate)),
        )
        verdict = guards.evaluate(
            _reply(spoken="Привет, человек!"),
            _turn(user_input="привет"),
            _state(),
        )
        assert verdict.kind in {
            VerdictKind.ACCEPT,
            VerdictKind.RETRY,
            VerdictKind.DISCARD,
        }


# ---------------------------------------------------------------------------
# 9. Issue #2266 — babble retry semantics through the bare TurnGuards
#    surface (without _TestableDialogueNode).
#
# DoD #1 of issue #2266: «Логика DSM-перехода и бюджета покрыта тестом
# через голый интерфейс core/». These tests pin down the babble guard
# invariants — the same ones ``test_issue_992_babble_guard.py`` checks
# through the heavy ``_TestableDialogueNode`` harness — but using only
# the pure ``TurnGuards.evaluate`` surface and a hand-rolled budget
# driver, so the regression can be diagnosed without ROS2.
# ---------------------------------------------------------------------------


class TestBabbleIntegrationViaTurnGuards:
    """Babble guard invariants exposed through TurnGuards (issue #2266).

    The legacy ``_TestableDialogueNode`` harness in
    ``test/test_issue_992_babble_guard.py`` proves the end-to-end behaviour
    (LLM-provider round-trips, publishers, ``_babble_retry_used`` flag).
    This class proves the *policy contract* on the bare
    :class:`TurnGuards` surface — no ROS2, no harness, no flag plumbing.
    """

    def _guards(self) -> TurnGuards:
        # Babble-only pipeline. Order is irrelevant — there is only one
        # guard here — but going through ``default_guards`` would pull
        # in the music slot which we don't need and which would require
        # a fake callable. Build it directly.
        return TurnGuards(guards=[BabbleGuard()])

    def test_babble_opener_with_performance_intent_yields_retry(self) -> None:
        """«Зачитаю рэп» + perf command → RETRY (mirrors 992 Bug D main path)."""
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Зачитаю рэпчик про космос!"),
            _turn(user_input="зачитай рэп про космос"),
            _state(),
        )
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "babble"
        assert verdict.prompt and "зачитай рэп про космос" in verdict.prompt

    def test_pure_promise_opener_retries_without_perf_keyword(self) -> None:
        """«Погнали!» — pure promise — retries even on a non-perf turn.

        The legacy detector only defers when ``user_wants_performance`` is
        False AND the opener is NOT in the promise-only subset
        («зачит», «погнали», «устроим», «переключ», «давай-ка»).
        Verifying that the TurnGuards wrapper preserves this branch is
        the whole point of the issue — without it the guard would
        regress to "fires only on perf requests" and silence the
        well-known «Погнали!» bug.
        """
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Погнали!"),
            _turn(user_input="расскажи про себя"),
            _state(),
        )
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "babble"

    def test_planning_narration_retries_even_off_topic(self) -> None:
        """Snake_case tool name in the reply → RETRY (issue #992 Bug D fix)."""
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="юзер хочет узнать время, вызываю get_current_time"),
            _turn(user_input="сколько времени"),
            _state(),
        )
        assert verdict.kind is VerdictKind.RETRY
        assert verdict.guard_name == "babble"

    def test_normal_text_defers(self) -> None:
        """Normal answer → ACCEPT (no guard raises)."""
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Сейчас 12:34."),
            _turn(user_input="сколько времени"),
            _state(),
        )
        assert verdict is ACCEPT

    def test_speak_text_real_skips_babble(self) -> None:
        """If a real speak_text fired, the babble guard must NOT retry.

        Mirrors the issue-988 anti-duplicate contract — once the LLM
        voiced the answer, swallowing it to retry would just double
        the audio. The detector must defer.
        """
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Зачитаю рэпчик про космос!", speak_text_real=1),
            _turn(user_input="зачитай рэп про космос"),
            _state(),
        )
        assert verdict is ACCEPT

    def test_empty_spoken_defers(self) -> None:
        """Empty reply → ACCEPT (BabbleGuard guards nothing here)."""
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken=""),
            _turn(user_input="зачитай рэп про космос"),
            _state(),
        )
        assert verdict is ACCEPT

    def test_state_question_with_babble_opener_defers(self) -> None:
        """«Играет ли сейчас музыка?» + «сейчас » opener → ACCEPT.

        Live regression 30.08: «играет ли сейчас музыка» is a QUESTION,
        not a performance command. ``is_state_question`` short-circuits
        ``user_wants_performance`` to False, and ``сейчас `` is NOT in
        the promise-only subset, so the guard must defer.
        """
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Сейчас тишина — ничего не играет."),
            _turn(user_input="играет ли сейчас музыка"),
            _state(),
        )
        assert verdict is ACCEPT

    def test_budget_exhausted_degrades_retry_to_accept(self) -> None:
        """budget_left == 0 + RETRY → ACCEPT (no LLM ping-pong).

        Issue #1881 — when the per-turn budget is spent, the orchestrator
        MUST downgrade any further Retry to Accept and log a warning,
        instead of dispatching another round-trip that would just
        repeat the same guard verdict. This is the bare ``core/``
        equivalent of ``_consume_synthetic_retry(guard_name=...)``'s
        False branch.
        """
        guards = self._guards()
        verdict = guards.evaluate(
            _reply(spoken="Зачитаю рэпчик про космос!"),
            _turn(user_input="зачитай рэп про космос"),
            _state(budget_left=0),
        )
        assert verdict is ACCEPT

    def test_consume_budget_decrements_after_retry(self) -> None:
        """consume_budget(state) → state' with budget_left -= 1.

        The dialogue_node adapter applies ``consume_budget`` after
        dispatching a Retry verdict so the NEXT ``evaluate`` call sees
        the decremented budget. Verifying this contract on the bare
        ``core/`` surface means the regression test
        ``test_issue_1881_synthetic_retry_budget.py`` has a
        corresponding low-level check.
        """
        state = _state(budget_left=2)
        next_state = consume_budget(state)
        assert next_state.budget_left == 1
        # Original is unchanged — dataclass(frozen=True) immutability.
        assert state.budget_left == 2

    def test_reset_budget_restores_full_ceiling(self) -> None:
        """reset_budget() → fresh TurnState at DEFAULT_MAX_SYNTHETIC_RETRIES.

        Mirrors the legacy ``_synthetic_retries_left = DEFAULT`` reset
        at the top of a user-initiated turn. Verified on the bare
        surface so the dialogue_node adapter's ``_reset_turn_budget``
        has a direct unit counterpart.
        """
        fresh = reset_budget()
        assert fresh.budget_left == DEFAULT_MAX_SYNTHETIC_RETRIES
        # Explicit override works.
        custom = reset_budget(max_retries=5)
        assert custom.budget_left == 5

    def test_two_retry_waves_deplete_budget(self) -> None:
        """RETRY → consume → RETRY → consume → 0 → Accept.

        The "one-shot per turn" rule is enforced by the BUDGET, not by
        a per-guard boolean. Two successive RETRYs from BabbleGuard
        drain the budget to zero, and the third call must Accept even
        though the detector would otherwise fire — matching the legacy
        ``_babble_retry_used`` invariant in
        ``test_issue_992_babble_guard.py::test_retry_only_fires_once_*``.
        """
        guards = self._guards()
        state = reset_budget(max_retries=2)
        spoken = "Зачитаю рэпчик про космос!"
        user_input = "зачитай рэп про космос"

        v1 = guards.evaluate(_reply(spoken=spoken), _turn(user_input=user_input), state)
        assert v1.kind is VerdictKind.RETRY
        state = consume_budget(state)

        v2 = guards.evaluate(_reply(spoken=spoken), _turn(user_input=user_input), state)
        assert v2.kind is VerdictKind.RETRY
        state = consume_budget(state)

        v3 = guards.evaluate(_reply(spoken=spoken), _turn(user_input=user_input), state)
        assert v3 is ACCEPT
        assert state.budget_left == 0
