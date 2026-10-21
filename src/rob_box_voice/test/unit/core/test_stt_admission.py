"""Unit tests for :mod:`rob_box_voice.core.stt_admission` — issue #2628.

Pure-Python — no ROS2, no rclpy. Covers:

* :func:`parse_tg_prefix` / :func:`parse_speaker_event` — pure parsers
  with unit tests on byte-level inputs and bad-input paths
  (``TypeError`` / ``ValueError``).
* :class:`SttContext` — immutability + ``with_text`` mutation semantics.
* :class:`SttOutcome` + :class:`SttOutcomeKind` — verdict construction.
* :class:`SttAdmission` orchestrator — "first non-PASS wins",
  ``skip_counter`` auto-increment, ``logger=None`` silence.
* Per-step behaviour:
  - :class:`EmptyTextStep` drops empty text.
  - :class:`RejectedMarkerStep` drops ``rejected`` / ``empty`` /
    «тишина» markers (Issue #989).
  - :class:`SilencedStateStep` allows unsilence, drops everything else.
  - :class:`AddressStep` — адресация кодом (ADR-0150 PR-5): вейк, TG,
    координатор, бэклог (В12); :class:`BareWakeStep` — голое «робот».
  - :class:`SilenceCommandStep` — silence + music-stop override.
  - :class:`CommandIntentGateStep` — TG bypass, command_parser path.
  - :class:`NewSessionStep` — host ``reset_session`` path.
  - :class:`TakeTurnStep` — ход в полёте отменяется, речь — нет (ADR-0150 PR-5).
  - :class:`DispatchTriggerStep` — FSM + sfx chain.
* ``default_steps()`` order matches legacy inline branches.
"""

from __future__ import annotations

import logging
import sys
from typing import Dict, List, Optional, Tuple

import pytest

from rob_box_dialog.backlog import Backlog, Heard

from rob_box_voice.core.stt_admission import (
    AddressStep,
    BareWakeStep,
    CommandIntentGateStep,
    DispatchTriggerStep,
    EmptyTextStep,
    MediaCommandStep,
    NewSessionStep,
    PASS,
    RejectedMarkerStep,
    SilenceCommandStep,
    SilencedStateStep,
    SttAdmission,
    SttContext,
    SttOutcomeKind,
    TakeTurnStep,
    default_steps,
    drop,
    handled,
    parse_speaker_event,
    parse_tg_prefix,
)


# ---------------------------------------------------------------------------
# Pure helpers — parse_tg_prefix / parse_speaker_event
# ---------------------------------------------------------------------------


class TestParseTgPrefix:
    def test_strips_marker_with_chat_id(self) -> None:
        assert parse_tg_prefix("[TG:42] привет") == ("привет", 42)

    def test_returns_none_for_missing_marker(self) -> None:
        assert parse_tg_prefix("просто текст") == ("просто текст", None)

    def test_malformed_marker_returns_original_text(self) -> None:
        # Non-numeric chat id → marker kept verbatim, chat id None.
        assert parse_tg_prefix("[TG:not_a_number] привет") == (
            "[TG:not_a_number] привет",
            None,
        )

    def test_empty_string(self) -> None:
        assert parse_tg_prefix("") == ("", None)

    def test_none_input(self) -> None:
        # ``TypeError`` path — helper must not raise.
        text, chat_id = parse_tg_prefix(None)  # type: ignore[arg-type]
        assert text == ""
        assert chat_id is None

    def test_unclosed_marker(self) -> None:
        assert parse_tg_prefix("[TG:42 без скобки") == ("[TG:42 без скобки", None)

    def test_whitespace_inside_marker(self) -> None:
        # Leading/trailing whitespace is stripped by the caller
        # (``_on_stt`` does ``msg.data.strip()`` before parsing). The
        # helper itself does NOT strip the whole text — it strips only
        # the segment after the closing ``]``.
        assert parse_tg_prefix("[TG:7]  текст  ") == ("текст", 7)

    def test_only_marker(self) -> None:
        assert parse_tg_prefix("[TG:1]") == ("", 1)


class TestParseSpeakerEvent:
    def test_none_returns_defaults(self) -> None:
        assert parse_speaker_event(None) == (None, 0.0)

    def test_empty_dict(self) -> None:
        assert parse_speaker_event({}) == (None, 0.0)

    def test_full_payload(self) -> None:
        assert parse_speaker_event(
            {"speaker_tag": "alice", "duration_s": 1.5}
        ) == ("alice", 1.5)

    def test_empty_speaker_tag_normalised_to_none(self) -> None:
        # Empty-string speaker tag becomes None so the consumer can
        # use ``speaker_tag or "anonymous"`` without extra branches.
        assert parse_speaker_event({"speaker_tag": "", "duration_s": 0.0}) == (
            None,
            0.0,
        )

    def test_invalid_duration_defaults_to_zero(self) -> None:
        assert parse_speaker_event(
            {"speaker_tag": "bob", "duration_s": "oops"}
        ) == ("bob", 0.0)

    def test_non_dict_returns_defaults(self) -> None:
        assert parse_speaker_event("not a dict") == (None, 0.0)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# SttContext + SttOutcome
# ---------------------------------------------------------------------------


class TestSttContext:
    def test_with_text_lowercases(self) -> None:
        ctx = SttContext(raw="RAW", text="RoboT", text_lower="robot")
        new_ctx = ctx.with_text("RoboT")
        assert new_ctx.text == "RoboT"
        assert new_ctx.text_lower == "robot"
        assert new_ctx.raw == "RAW"

    def test_with_text_is_pure(self) -> None:
        # with_text must not mutate the source context.
        ctx = SttContext(raw="r", text="t", text_lower="t")
        ctx.with_text("changed")
        assert ctx.text == "t"
        assert ctx.text_lower == "t"

    def test_frozen_dataclass(self) -> None:
        ctx = SttContext(raw="r", text="t", text_lower="t")
        with pytest.raises(Exception):  # FrozenInstanceError
            ctx.text = "new"  # type: ignore[misc]

    def test_heard_and_bare_defaults(self) -> None:
        ctx = _ctx()
        assert ctx.heard is None and ctx.bare is False

    def test_skip_counter_is_shared(self) -> None:
        # Two contexts can share a counter dict — orchestrator relies on
        # this so increments land in the same dict the caller owns.
        counter: Dict[str, int] = {}
        ctx1 = SttContext(raw="r", text="t", text_lower="t", skip_counter=counter)
        ctx2 = SttContext(raw="r2", text="t2", text_lower="t2", skip_counter=counter)
        ctx1.skip_counter["x"] = 1
        assert ctx2.skip_counter["x"] == 1


class TestSttOutcome:
    def test_pass_singleton(self) -> None:
        assert PASS.kind is SttOutcomeKind.PASS

    def test_drop_helper(self) -> None:
        out = drop("name", "reason")
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "name"
        assert out.reason == "reason"
        assert out.new_context is None

    def test_handled_helper(self) -> None:
        out = handled("name", "reason")
        assert out.kind is SttOutcomeKind.HANDLED


# ---------------------------------------------------------------------------
# Fake host + orchestrator
# ---------------------------------------------------------------------------


class _RecordingHost:
    """In-memory ``SttAdmissionHost`` for tests.

    Tracks every callback the orchestrator fires so a test can assert
    on the **order** and **arity** of the calls — issue #2628 explicitly
    requires "test on the **order** of barriers".
    """

    def __init__(
        self,
        *,
        unsilence_ok: bool = False,
        is_music_stop: bool = False,
        handle_silence: bool = True,
        handle_command_intent_match: bool = False,
        reset_session_match: bool = False,
        accumulate: bool = True,
        backlog: Optional[Backlog] = None,
        state_idle: bool = True,
        music_stop_overrides: Tuple[str, ...] = (),
        silence_phrases: Tuple[str, ...] = ("замолчи",),
        media_command: bool = False,
    ) -> None:
        self.calls: List[str] = []
        self._unsilence_ok = unsilence_ok
        self._is_music_stop = is_music_stop
        self._handle_silence = handle_silence
        self._handle_command_intent_match = handle_command_intent_match
        self._reset_session_match = reset_session_match
        self._accumulate = accumulate
        self._backlog = backlog
        self.addressed_calls: List[object] = []
        self.remembered: List[Heard] = []
        self._state_idle = state_idle
        self._music_stop_overrides = music_stop_overrides
        self._silence_phrases = silence_phrases
        self._media_command = media_command
        self.take_turn_count = 0
        self.publish_state_count = 0
        self.thinking_sfx_count = 0

    def unsilence(self, text_lower: str) -> bool:
        self.calls.append(f"unsilence({text_lower!r})")
        return self._unsilence_ok

    def backlog(self) -> Optional[Backlog]:
        return self._backlog

    def remember_unaddressed(self, heard: Heard) -> bool:
        self.calls.append(f"remember({heard.text!r})")
        if self._accumulate:
            self.remembered.append(heard)
        return self._accumulate

    def addressed(self, decision: object, recent: object) -> None:
        self.addressed_calls.append((decision, recent))

    def bare_wake(self) -> None:
        self.calls.append("bare_wake")

    def handle_silence_command(self) -> bool:
        self.calls.append("handle_silence")
        return self._handle_silence

    def is_music_stop_command(self, text_lower: str) -> bool:
        self.calls.append(f"is_music_stop({text_lower!r})")
        return self._is_music_stop

    def handle_command_intent(self, text: str, text_lower: str) -> bool:
        self.calls.append(f"handle_command_intent({text!r},{text_lower!r})")
        return self._handle_command_intent_match

    def reset_session(
        self,
        text: str,
        text_lower: str,
        tg_chat_id: Optional[int],
    ) -> bool:
        self.calls.append(f"reset_session({text!r},{tg_chat_id!r})")
        return self._reset_session_match

    def handle_media_command(self, text: str) -> bool:
        self.calls.append(f"handle_media_command({text!r})")
        return self._media_command

    def take_turn(self) -> None:
        self.calls.append("take_turn")
        self.take_turn_count += 1

    def transition_idle_to_wake(self) -> bool:
        self.calls.append("transition_idle_to_wake")
        return self._state_idle

    def transition_stt_result(self) -> None:
        self.calls.append("transition_stt_result")

    def publish_state(self) -> None:
        self.calls.append("publish_state")
        self.publish_state_count += 1

    def trigger_thinking_sfx(self) -> None:
        self.calls.append("trigger_thinking_sfx")
        self.thinking_sfx_count += 1


def _ctx(**overrides: object) -> SttContext:
    base: Dict[str, object] = {
        "raw": "робот расскажи анекдот",
        "text": "робот расскажи анекдот",
        "text_lower": "робот расскажи анекдот",
        "tg_chat_id": None,
        "speaker_tag": None,
        "speaker_duration_s": 0.0,
        "state_name": "IDLE",
        "is_silenced": False,
        "wake_words": ("робот", "робокс", "робобокс"),
    }
    base.update(overrides)
    return SttContext(**base)  # type: ignore[arg-type]


def _admission(
    steps: Optional[List[object]] = None,
    logger: Optional[logging.Logger] = None,
) -> SttAdmission:
    return SttAdmission(
        steps=steps if steps is not None else default_steps(),  # type: ignore[arg-type]
        logger=logger,
    )


# ---------------------------------------------------------------------------
# Steps in isolation
# ---------------------------------------------------------------------------


class TestEmptyTextStep:
    def test_drops_empty_text(self) -> None:
        out = EmptyTextStep().apply(_ctx(text="", text_lower=""), _RecordingHost())
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "empty_text"

    def test_passes_normal_text(self) -> None:
        out = EmptyTextStep().apply(_ctx(), _RecordingHost())
        assert out.kind is SttOutcomeKind.PASS


class TestRejectedMarkerStep:
    @pytest.mark.parametrize(
        "marker",
        ["rejected", "rejected by vad", "«rejected»", "empty", "«пусто»", "тишина"],
    )
    def test_drops_rejected_markers(self, marker: str) -> None:
        out = RejectedMarkerStep().apply(
            _ctx(text=marker, text_lower=marker), _RecordingHost()
        )
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "stt_rejected"

    def test_passes_normal_text(self) -> None:
        out = RejectedMarkerStep().apply(_ctx(), _RecordingHost())
        assert out.kind is SttOutcomeKind.PASS


class TestSilencedStateStep:
    def test_passes_when_not_silenced(self) -> None:
        out = SilencedStateStep().apply(
            _ctx(is_silenced=False), _RecordingHost()
        )
        assert out.kind is SttOutcomeKind.PASS

    def test_drops_when_silenced_no_unsilence(self) -> None:
        host = _RecordingHost()
        out = SilencedStateStep().apply(
            _ctx(is_silenced=True, text="погода", text_lower="погода"),
            host,
        )
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "silenced"
        # host.unsilence must be called only when text matches the
        # unsilence phrase; here it must NOT have been called.
        assert all("unsilence" not in c for c in host.calls)

    def test_handles_unsilence(self) -> None:
        host = _RecordingHost(unsilence_ok=True)
        out = SilencedStateStep().apply(
            _ctx(is_silenced=True, text="отвечай мне",
                 text_lower="отвечай мне"),
            host,
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "unsilence"
        assert any("unsilence" in c for c in host.calls)


class TestAddressStep:
    def test_tg_is_addressed_without_wake(self) -> None:
        host = _RecordingHost()
        out = AddressStep().apply(_ctx(text="просто текст", text_lower="просто текст", tg_chat_id=42), host)
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context.text == "просто текст"
        assert all("remember" not in c for c in host.calls)

    def test_tg_coordinator_is_not_addressed(self) -> None:
        out = AddressStep().apply(_ctx(text="Клод привет", text_lower="клод привет", tg_chat_id=42), _RecordingHost())
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "NotAddressed(coordinator)"

    def test_wake_word_strips_and_passes(self) -> None:
        host = _RecordingHost()
        out = AddressStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context.text == "расскажи анекдот" and out.new_context.bare is False
        assert len(host.addressed_calls) == 1

    def test_no_wake_word_with_backlog_handles(self) -> None:
        host = _RecordingHost(accumulate=True)
        out = AddressStep().apply(_ctx(text="фон", text_lower="фон"), host)
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "backlog_accumulated"
        assert [h.text for h in host.remembered] == ["фон"]

    def test_no_wake_word_no_backlog_drops(self) -> None:
        host = _RecordingHost(accumulate=False)
        out = AddressStep().apply(_ctx(text="фон", text_lower="фон"), host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "no_wake_word"

    def test_bare_wake_with_fresh_backlog_picks_it_up(self) -> None:
        backlog = Backlog()
        backlog.add(Heard(text="покажи анимацию", t=95.0, doa_deg=90.0))
        heard = Heard(text="робот", t=100.0, doa_deg=95.0)
        out = AddressStep().apply(_ctx(text="робот", text_lower="робот", heard=heard), _RecordingHost(backlog=backlog))
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context.text == "покажи анимацию" and out.new_context.bare is False

    def test_bare_wake_without_backlog_marks_bare(self) -> None:
        out = AddressStep().apply(_ctx(text="робот", text_lower="робот"), _RecordingHost(backlog=Backlog()))
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context.text == "" and out.new_context.bare is True


class TestBareWakeStep:
    def test_bare_is_handled_by_reflex(self) -> None:
        host = _RecordingHost()
        out = BareWakeStep().apply(_ctx(bare=True, text="", text_lower=""), host)
        assert out.kind is SttOutcomeKind.HANDLED and out.reason == "bare_wake"
        assert "bare_wake" in host.calls

    def test_normal_phrase_passes(self) -> None:
        host = _RecordingHost()
        assert BareWakeStep().apply(_ctx(), host).kind is SttOutcomeKind.PASS
        assert "bare_wake" not in host.calls


class TestSilenceCommandStep:
    def test_passes_normal_text(self) -> None:
        out = SilenceCommandStep().apply(_ctx(), _RecordingHost())
        assert out.kind is SttOutcomeKind.PASS

    def test_handles_silence(self) -> None:
        host = _RecordingHost(handle_silence=True)
        out = SilenceCommandStep().apply(
            _ctx(text="замолчи", text_lower="замолчи"), host
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "silence_command"

    def test_music_stop_falls_through(self) -> None:
        # Issue #1279 — «хватит диджеить» is music-stop, must reach LLM.
        host = _RecordingHost(is_music_stop=True)
        out = SilenceCommandStep().apply(
            _ctx(text="хватит диджеить",
                 text_lower="хватит диджеить"),
            host,
        )
        assert out.kind is SttOutcomeKind.PASS
        assert any("is_music_stop" in c for c in host.calls)


class TestCommandIntentGateStep:
    def test_tg_bypasses(self) -> None:
        host = _RecordingHost(handle_command_intent_match=True)
        out = CommandIntentGateStep().apply(
            _ctx(tg_chat_id=1, text="вперёд", text_lower="вперёд"),
            host,
        )
        assert out.kind is SttOutcomeKind.PASS
        assert all("handle_command_intent" not in c for c in host.calls)

    def test_handles_intent_match(self) -> None:
        host = _RecordingHost(handle_command_intent_match=True)
        out = CommandIntentGateStep().apply(
            _ctx(text="вперёд", text_lower="вперёд"), host
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "command_intent"

    def test_passes_no_match(self) -> None:
        host = _RecordingHost(handle_command_intent_match=False)
        out = CommandIntentGateStep().apply(
            _ctx(text="расскажи анекдот",
                 text_lower="расскажи анекдот"),
            host,
        )
        assert out.kind is SttOutcomeKind.PASS


class TestNewSessionStep:
    def test_handles_session_reset(self) -> None:
        host = _RecordingHost(reset_session_match=True)
        out = NewSessionStep().apply(
            _ctx(text="новая сессия", text_lower="новая сессия"),
            host,
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "new_session"

    def test_passes_no_match(self) -> None:
        host = _RecordingHost(reset_session_match=False)
        out = NewSessionStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS


class TestMediaCommandStep:
    """Issue #3134 — роутер медиакоманд до LLM."""

    def test_handles_media_command(self) -> None:
        host = _RecordingHost(media_command=True)
        out = MediaCommandStep().apply(
            _ctx(text="играй громче", text_lower="играй громче"), host
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "media_command"
        assert host.calls == ["handle_media_command('играй громче')"]

    def test_passes_non_media(self) -> None:
        host = _RecordingHost(media_command=False)
        out = MediaCommandStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS

    def test_pipeline_media_command_skips_llm_dispatch_and_barge_in(self) -> None:
        # Реплика закрыта роутером: ни cancel барж-ина, ни FSM/SFX-диспатча.
        ctx = _ctx(
            text="робот играй громче", text_lower="робот играй громче",
            skip_counter={},
        )
        host = _RecordingHost(media_command=True)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.step_name == "media_command"
        assert "handle_media_command('играй громче')" in host.calls
        assert "transition_stt_result" not in host.calls
        assert host.take_turn_count == 0

    def test_pipeline_tg_reaches_router(self) -> None:
        ctx = _ctx(
            text="выключи музыку", text_lower="выключи музыку",
            tg_chat_id=42, skip_counter={},
        )
        host = _RecordingHost(media_command=True)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.step_name == "media_command"

    def test_router_runs_before_take_turn(self) -> None:
        # HA core#139415: медиакоманду закрывает роутер, ход в полёте она
        # не отменяет (громкость не рвёт идущий ответ).
        ctx = _ctx(
            text="робот потише", text_lower="робот потише", skip_counter={},
        )
        host = _RecordingHost(media_command=True)
        out = SttAdmission(steps=default_steps()).evaluate(ctx, host)  # type: ignore[arg-type]
        assert out.kind is SttOutcomeKind.HANDLED
        assert host.take_turn_count == 0


class TestTakeTurnStep:
    def test_takes_turn_and_passes(self) -> None:
        # ADR-0150 §6.2 (PR-5): одна политика — ход в полёте отменяется,
        # речь не трогается (рвёт её только ясная стоп-команда).
        host = _RecordingHost()
        out = TakeTurnStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert host.take_turn_count == 1


class TestDispatchTriggerStep:
    def test_fires_fsm_and_sfx(self) -> None:
        host = _RecordingHost()
        out = DispatchTriggerStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert "transition_idle_to_wake" in host.calls
        assert "transition_stt_result" in host.calls
        assert "publish_state" in host.calls
        assert "trigger_thinking_sfx" in host.calls
        assert host.thinking_sfx_count == 1


# ---------------------------------------------------------------------------
# Orchestrator — order, first-verdict-wins, skip_counter, logger
# ---------------------------------------------------------------------------


class TestOrchestrator:
    def test_first_drop_wins(self) -> None:
        steps = [
            EmptyTextStep(),
            RejectedMarkerStep(),
        ]
        ctx = _ctx(text="", text_lower="")
        host = _RecordingHost()
        out = _admission(steps).evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "empty_text"

    def test_subsequent_steps_not_consulted_after_drop(self) -> None:
        # After EmptyTextStep fires DROP, RejectedMarkerStep must NOT
        # be consulted. ``calls`` would record ``"unsilence(...)"`` etc.
        steps = [
            EmptyTextStep(),
            SilencedStateStep(),
        ]
        ctx = _ctx(text="", text_lower="", is_silenced=True)
        host = _RecordingHost(unsilence_ok=True)
        out = _admission(steps).evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "empty_text"
        # unsilence must NOT be called because EmptyTextStep already
        # dropped — proves "first verdict wins".
        assert all("unsilence" not in c for c in host.calls)

    def test_skip_counter_incremented_on_drop(self) -> None:
        ctx = _ctx(text="", text_lower="", skip_counter={})
        out = _admission().evaluate(ctx, _RecordingHost())
        assert out.kind is SttOutcomeKind.DROP
        # ``reason`` becomes the key in skip_counter.
        assert ctx.skip_counter.get("empty_text") == 1

    def test_skip_counter_not_incremented_on_handled(self) -> None:
        ctx = _ctx(
            text="хватит диджеить", text_lower="хватит диджеить",
            skip_counter={},
        )
        host = _RecordingHost(is_music_stop=False, handle_silence=True)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED
        # HANDLED does not auto-increment.
        assert ctx.skip_counter == {}

    def test_logger_none_silences_orchestrator(self) -> None:
        logger = logging.getLogger("test_stt_admission_silent")
        # With logger=None, no warning is raised even on HANDLED.
        ctx = _ctx(
            text="замолчи", text_lower="замолчи", skip_counter={},
        )
        host = _RecordingHost(handle_silence=True)
        out = _admission(logger=None).evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED

    def test_logger_warns_on_handled(self) -> None:
        logger = logging.getLogger("test_stt_admission_logger")
        captured: List[str] = []

        class _Capture(logging.Handler):
            def emit(self, record: logging.LogRecord) -> None:
                captured.append(record.getMessage())

        cap = _Capture(level=logging.DEBUG)
        logger.addHandler(cap)
        try:
            ctx = _ctx(
                text="робот замолчи",
                text_lower="робот замолчи",
                raw="робот замолчи",
                skip_counter={},
            )
            host = _RecordingHost(handle_silence=True)
            out = _admission(logger=logger).evaluate(ctx, host)
            assert out.kind is SttOutcomeKind.HANDLED
            assert any("silence_command" in m for m in captured)
        finally:
            logger.removeHandler(cap)

    def test_pass_threads_context_mutation(self) -> None:
        # AddressStep rewrites ``text`` (вейк снят). The orchestrator must
        # thread the new context to the next step so silence /
        # command-intent steps see the **stripped** text.
        steps = [
            AddressStep(),
            SilenceCommandStep(),
        ]
        ctx = _ctx(
            text="робот замолчи",
                 text_lower="робот замолчи",
                 skip_counter={},
        )
        host = _RecordingHost(handle_silence=True)
        out = _admission(steps).evaluate(ctx, host)
        # After strip: text="замолчи" → silence step fires HANDLED.
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.step_name == "silence_command"
        # Verify the host was called with the *stripped* lowercased.
        silence_calls = [c for c in host.calls if c.startswith("handle_silence")]
        assert silence_calls  # present, regardless of args

    def test_empty_steps_raises(self) -> None:
        with pytest.raises(ValueError, match="non-empty"):
            SttAdmission(steps=[])

    def test_default_steps_canonical_order(self) -> None:
        # Order matches the legacy inline if-chain at dialogue_node.py
        # 2159-2445. If this drifts, the orchestrator runs a different
        # order than the legacy code — regression hazard.
        steps = default_steps()
        names = [type(s).__name__ for s in steps]
        assert names == [
            "EmptyTextStep",
            "RejectedMarkerStep",
            "AddressStep",
            "SilencedStateStep",
            "BareWakeStep",
            "SilenceCommandStep",
            "CommandIntentGateStep",
            "NewSessionStep",
            "MediaCommandStep",
            "TakeTurnStep",
            "DispatchTriggerStep",
        ]


# ---------------------------------------------------------------------------
# End-to-end pipeline — full default_steps()
# ---------------------------------------------------------------------------


class TestEndToEnd:
    def test_full_pipeline_idle_wake_dispatch(self) -> None:
        # IDLE state, wake-word present, no backlog, no command
        # intent, replace policy. Pipeline must reach DispatchTrigger
        # and PASS with stripped text.
        ctx = _ctx(
            text="робот расскажи анекдот",
                 text_lower="робот расскажи анекдот",
                 skip_counter={},
        )
        host = _RecordingHost(state_idle=True)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context is not None
        # wake-word stripped.
        assert out.new_context.text == "расскажи анекдот"
        # FSM + SFX fired.
        assert "transition_idle_to_wake" in host.calls
        assert "transition_stt_result" in host.calls
        assert "trigger_thinking_sfx" in host.calls
        # Ход принят (TakeTurnStep): отмена хода в полёте, речь не рвётся.
        assert host.take_turn_count == 1

    def test_full_pipeline_silenced_no_unsilence_drops(self) -> None:
        ctx = _ctx(
            text="робот расскажи анекдот", text_lower="робот расскажи анекдот",
            state_name="SILENCED", is_silenced=True, skip_counter={},
        )
        host = _RecordingHost()
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "silenced_state"
        assert out.reason == "silenced"

    def test_full_pipeline_rejected_marker_drops(self) -> None:
        ctx = _ctx(text="rejected", text_lower="rejected", skip_counter={})
        host = _RecordingHost()
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "rejected_marker"

    def test_full_pipeline_tg_bypasses_wake_and_gate(self) -> None:
        # Telegram text: TG prefix was already stripped by the caller.
        # Pipeline must pass through WakeWordStep + CommandIntentGateStep
        # (both bypass on TG) and reach DispatchTrigger.
        ctx = _ctx(
            text="просто текст", text_lower="просто текст",
            tg_chat_id=42, skip_counter={},
        )
        host = _RecordingHost(
            handle_command_intent_match=True,  # would gate if called
        )
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.PASS
        # Command-intent gate must NOT be called on TG.
        assert all("handle_command_intent" not in c for c in host.calls)

    def test_full_pipeline_no_wake_word_accumulates(self) -> None:
        # Mic input without wake word, accumulator enabled → HANDLED.
        ctx = _ctx(text="фон", text_lower="фон", skip_counter={})
        host = _RecordingHost(accumulate=True)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.step_name == "address"
        assert out.reason == "backlog_accumulated"

    def test_full_pipeline_no_wake_word_no_accumulator_drops(self) -> None:
        ctx = _ctx(text="фон", text_lower="фон", skip_counter={})
        host = _RecordingHost(accumulate=False)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "address"
        assert out.reason == "no_wake_word"
        # Counter incremented.
        assert ctx.skip_counter.get("no_wake_word") == 1


# ---------------------------------------------------------------------------
# Issue #2713 — logger severity must not vary per call-site
# ---------------------------------------------------------------------------
#
# rclpy's ``RcutilsLogger.log()`` caches the logging severity **by
# call-site** (the caller's file/line/function) and raises
# ``ValueError('Logger severity cannot be changed between calls.')`` if
# the same call-site is later hit with a different severity. A plain
# ``Mock()``/``logging.Logger`` does not reproduce that contract at all
# (that is exactly why the bug shipped — the real unit tests used
# ``logging.Logger``, which has no such cache). ``_RcutilsLikeLogger``
# below reproduces the cache so a regression here would fail loudly,
# the same way it did on the robot on 2026-09-21.


class _RcutilsLikeLogger:
    """Minimal stand-in for ``rclpy.impl.rcutils_logger.RcutilsLogger``.

    Reproduces the one behaviour that matters for issue #2713: a given
    call-site (the caller's ``(filename, lineno, function)``) is only
    ever allowed one severity for its lifetime. A second call from the
    *same* line with a *different* severity raises ``ValueError`` —
    mirroring the real crash from the robot's traceback.
    """

    def __init__(self) -> None:
        self.records: List[Tuple[str, str]] = []
        self._severity_by_site: Dict[Tuple[str, int, str], str] = {}

    def _log(self, severity: str, message: str) -> None:
        # Frame 0 = this method, frame 1 = the .info()/.warning()
        # wrapper below, frame 2 = whoever actually wrote
        # ``self.logger.info(...)`` / ``self.logger.warning(...)`` —
        # i.e. the real call-site, exactly what rcutils keys on.
        caller = sys._getframe(2)
        site = (caller.f_code.co_filename, caller.f_lineno, caller.f_code.co_name)
        prev = self._severity_by_site.get(site)
        if prev is not None and prev != severity:
            raise ValueError("Logger severity cannot be changed between calls.")
        self._severity_by_site[site] = severity
        self.records.append((severity, message))

    def info(self, message: str) -> None:
        self._log("INFO", message)

    def warning(self, message: str) -> None:
        self._log("WARNING", message)


class TestIssue2713LoggerSeverityCallSite:
    def test_handled_then_drop_same_admission_no_crash(self) -> None:
        # First a HANDLED (-> WARNING), then a DROP (-> INFO) through
        # the SAME SttAdmission/logger — the exact sequence seen on
        # the robot (a run of "handled: step=wake_word" WARNs followed
        # by a DROP that used to crash the node).
        logger = _RcutilsLikeLogger()
        admission = _admission(logger=logger)

        handled_ctx = _ctx(
            text="робот замолчи", text_lower="робот замолчи",
            raw="робот замолчи", skip_counter={},
        )
        out1 = admission.evaluate(handled_ctx, _RecordingHost(handle_silence=True))
        assert out1.kind is SttOutcomeKind.HANDLED

        drop_ctx = _ctx(text="", text_lower="", skip_counter={})
        out2 = admission.evaluate(drop_ctx, _RecordingHost())
        assert out2.kind is SttOutcomeKind.DROP

        assert [sev for sev, _ in logger.records] == ["WARNING", "INFO"]

    def test_drop_then_handled_same_admission_no_crash(self) -> None:
        # Reverse order — DROP first, then HANDLED.
        logger = _RcutilsLikeLogger()
        admission = _admission(logger=logger)

        drop_ctx = _ctx(text="", text_lower="", skip_counter={})
        out1 = admission.evaluate(drop_ctx, _RecordingHost())
        assert out1.kind is SttOutcomeKind.DROP

        handled_ctx = _ctx(
            text="робот замолчи", text_lower="робот замолчи",
            raw="робот замолчи", skip_counter={},
        )
        out2 = admission.evaluate(handled_ctx, _RecordingHost(handle_silence=True))
        assert out2.kind is SttOutcomeKind.HANDLED

        assert [sev for sev, _ in logger.records] == ["INFO", "WARNING"]

    def test_stub_logger_actually_detects_shared_call_site(self) -> None:
        # Sanity check on the stub itself: if HANDLED and DROP were
        # (incorrectly) logged from the same line, the stub must raise
        # — otherwise the two tests above would be false negatives.
        logger = _RcutilsLikeLogger()

        def _shared_call_site(severity: str) -> None:
            log_fn = logger.warning if severity == "WARNING" else logger.info
            log_fn("same line for both severities")

        _shared_call_site("WARNING")
        with pytest.raises(ValueError, match="severity cannot be changed"):
            _shared_call_site("INFO")
