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
  - :class:`WakeWordStep` — TG bypass, wake-word pass, backlog
    accumulate-or-drop.
  - :class:`StripWakeWordStep` — strip wake-word, fallback to ``raw``
    when backlog pending.
  - :class:`SilenceCommandStep` — silence + music-stop override.
  - :class:`CommandIntentGateStep` — TG bypass, command_parser path.
  - :class:`NewSessionStep` — host ``reset_session`` path.
  - :class:`BacklogFlushStep` — host flag set + always PASS.
  - :class:`BargeInClassifyStep` — REPLACE/OFF vs ``classify`` policy.
  - :class:`DispatchTriggerStep` — FSM + sfx chain.
* ``default_steps()`` order matches legacy inline branches.
* ``SttContext.backlog_pending`` snapshot semantics.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Tuple

import pytest

from rob_box_voice.core.stt_admission import (
    DEFAULT_BARGE_IN_POLICY,
    BacklogFlushStep,
    BargeInClassifyStep,
    CommandIntentGateStep,
    DispatchTriggerStep,
    EmptyTextStep,
    NewSessionStep,
    PASS,
    RejectedMarkerStep,
    SilenceCommandStep,
    SilencedStateStep,
    SttAdmission,
    SttContext,
    SttOutcomeKind,
    StripWakeWordStep,
    WakeWordStep,
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

    def test_backlog_pending_default(self) -> None:
        ctx = SttContext(raw="r", text="t", text_lower="t")
        assert ctx.backlog_pending is False

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
        quick_decide: Tuple[str, bool, bool] = ("replace", False, False),
        live_task_alive: bool = False,
        state_idle: bool = True,
        music_stop_overrides: Tuple[str, ...] = (),
        silence_phrases: Tuple[str, ...] = ("замолчи",),
    ) -> None:
        self.calls: List[str] = []
        self._unsilence_ok = unsilence_ok
        self._is_music_stop = is_music_stop
        self._handle_silence = handle_silence
        self._handle_command_intent_match = handle_command_intent_match
        self._reset_session_match = reset_session_match
        self._accumulate = accumulate
        self._quick_decide = quick_decide
        self._live_task_alive = live_task_alive
        self._state_idle = state_idle
        self._music_stop_overrides = music_stop_overrides
        self._silence_phrases = silence_phrases
        self.enqueued: List[str] = []
        self.cancel_calls: List[bool] = []
        self.publish_state_count = 0
        self.thinking_sfx_count = 0

    def unsilence(self, text_lower: str) -> bool:
        self.calls.append(f"unsilence({text_lower!r})")
        return self._unsilence_ok

    def accumulate_without_wake(
        self, speaker_tag: Optional[str], text: str
    ) -> bool:
        self.calls.append(f"accumulate({speaker_tag!r},{text!r})")
        return self._accumulate

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

    def flush_pending_backlog(self) -> None:
        self.calls.append("flush_pending_backlog")

    def quick_decide_verdict(
        self, clean: str
    ) -> Tuple[str, bool, bool]:
        self.calls.append(f"quick_decide({clean!r})")
        return self._quick_decide

    def enqueue_pending(self, clean: str) -> bool:
        self.calls.append(f"enqueue_pending({clean!r})")
        if not self._live_task_alive:
            return False
        self.enqueued.append(clean)
        return True

    def cancel_inflight(self, stop_tts: bool) -> None:
        self.calls.append(f"cancel_inflight(stop_tts={stop_tts})")
        self.cancel_calls.append(stop_tts)

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
        "backlog_pending": False,
    }
    base.update(overrides)
    return SttContext(**base)  # type: ignore[arg-type]


def _admission(
    steps: Optional[List[object]] = None,
    logger: Optional[object] = None,
) -> SttAdmission:
    # ``logger`` is duck-typed: in production this is an
    # ``rclpy.impl.rcutils_logger.RcutilsLogger``, but the existing
    # ``SttAdmission.logger`` annotation is ``Optional[Any]``. Tests use
    # either a stdlib ``logging.Logger`` (old tests) or
    # ``_RclpyStyleLogger`` (issue #2713 regression).
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


class TestWakeWordStep:
    def test_tg_bypasses(self) -> None:
        host = _RecordingHost()
        out = WakeWordStep().apply(_ctx(tg_chat_id=42), host)
        assert out.kind is SttOutcomeKind.PASS
        # TG must not consult wake words or accumulator.
        assert all("accumulate" not in c for c in host.calls)

    def test_wake_word_passes(self) -> None:
        host = _RecordingHost()
        out = WakeWordStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS

    def test_no_wake_word_with_accumulator_handles(self) -> None:
        host = _RecordingHost(accumulate=True)
        out = WakeWordStep().apply(
            _ctx(text="фон", text_lower="фон"), host
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "backlog_accumulated"

    def test_no_wake_word_no_accumulator_drops(self) -> None:
        host = _RecordingHost(accumulate=False)
        out = WakeWordStep().apply(
            _ctx(text="фон", text_lower="фон"), host
        )
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "no_wake_word"


class TestStripWakeWordStep:
    def test_tg_bypasses(self) -> None:
        out = StripWakeWordStep().apply(_ctx(tg_chat_id=1), _RecordingHost())
        assert out.kind is SttOutcomeKind.PASS

    def test_strips_and_passes(self) -> None:
        out = StripWakeWordStep().apply(
            _ctx(text="робот расскажи анекдот",
                 text_lower="робот расскажи анекдот"),
            _RecordingHost(),
        )
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context is not None
        assert out.new_context.text == "расскажи анекдот"

    def test_empty_after_strip_with_backlog_falls_back_to_raw(self) -> None:
        # Issue #2346 — bare wake-word preserves ``raw`` when backlog
        # is pending (legacy L2288-2291 byte-for-byte).
        out = StripWakeWordStep().apply(
            _ctx(text="робот", text_lower="робот",
                 raw="робот",
                 backlog_pending=True),
            _RecordingHost(),
        )
        assert out.kind is SttOutcomeKind.PASS
        assert out.new_context is not None
        assert out.new_context.text == "робот"

    def test_empty_after_strip_without_backlog_drops(self) -> None:
        out = StripWakeWordStep().apply(
            _ctx(text="робот", text_lower="робот", backlog_pending=False),
            _RecordingHost(),
        )
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "empty_after_strip"


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


class TestBacklogFlushStep:
    def test_flushes_when_backlog_pending(self) -> None:
        host = _RecordingHost()
        out = BacklogFlushStep().apply(
            _ctx(backlog_pending=True), host
        )
        assert out.kind is SttOutcomeKind.PASS
        assert "flush_pending_backlog" in host.calls

    def test_noop_when_backlog_empty(self) -> None:
        # Issue #1766 — flag must NOT be set when backlog is empty
        # (legacy ``if backlog_pending: self._pending_backlog_flush = True``
        # at L2350-2351).
        host = _RecordingHost()
        out = BacklogFlushStep().apply(
            _ctx(backlog_pending=False), host
        )
        assert out.kind is SttOutcomeKind.PASS
        assert "flush_pending_backlog" not in host.calls


class TestBargeInClassifyStep:
    def test_replace_cancels_tts(self) -> None:
        # ``barge_in_policy != "classify"`` (default REPLACE) — cancel
        # in-flight turn with stop_tts=True unconditionally.
        host = _RecordingHost()
        out = BargeInClassifyStep().apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert host.cancel_calls == [True]

    def test_off_cancels_tts(self) -> None:
        host = _RecordingHost()
        out = BargeInClassifyStep(policy="off").apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert host.cancel_calls == [True]

    def test_classify_ignore_drops(self) -> None:
        host = _RecordingHost(quick_decide=("ignore", True, False))
        out = BargeInClassifyStep(policy="classify").apply(
            _ctx(text="фон", text_lower="фон"), host
        )
        assert out.kind is SttOutcomeKind.DROP
        assert out.reason == "quick_decide_ignore"

    def test_classify_pending_llm_enqueues(self) -> None:
        host = _RecordingHost(
            quick_decide=("pending_llm", False, True),
            live_task_alive=True,
        )
        out = BargeInClassifyStep(policy="classify").apply(
            _ctx(text="а ещё", text_lower="а ещё"), host
        )
        assert out.kind is SttOutcomeKind.HANDLED
        assert out.reason == "pending_llm"
        assert host.enqueued == ["а ещё"]

    def test_classify_pending_llm_no_live_task_cancels_no_tts(self) -> None:
        # No live task → orchestrator falls through to REPLACE-style
        # cancel; for PENDING_LLM stop_tts=False (do not cut a
        # playing segment). See legacy L2396-2398.
        host = _RecordingHost(
            quick_decide=("pending_llm", False, True),
            live_task_alive=False,
        )
        out = BargeInClassifyStep(policy="classify").apply(
            _ctx(text="а ещё", text_lower="а ещё"), host
        )
        assert out.kind is SttOutcomeKind.PASS
        assert host.cancel_calls == [False]

    def test_classify_replace_cancels_tts(self) -> None:
        host = _RecordingHost(quick_decide=("replace", False, False))
        out = BargeInClassifyStep(policy="classify").apply(_ctx(), host)
        assert out.kind is SttOutcomeKind.PASS
        assert host.cancel_calls == [True]


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
        # StripWakeWordStep rewrites ``text``. The orchestrator must
        # thread the new context to the next step so silence /
        # command-intent steps see the **stripped** text.
        # Manually compose a tiny pipeline: wake → strip → silence.

        steps = [
            WakeWordStep(),
            StripWakeWordStep(),
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
            "SilencedStateStep",
            "WakeWordStep",
            "StripWakeWordStep",
            "SilenceCommandStep",
            "CommandIntentGateStep",
            "NewSessionStep",
            "BacklogFlushStep",
            "BargeInClassifyStep",
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
        # Cancel fired (BargeInClassifyStep with REPLACE).
        assert host.cancel_calls == [True]

    def test_full_pipeline_silenced_no_unsilence_drops(self) -> None:
        ctx = _ctx(
            text="расскажи анекдот", text_lower="расскажи анекдот",
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
        assert out.step_name == "wake_word"
        assert out.reason == "backlog_accumulated"

    def test_full_pipeline_no_wake_word_no_accumulator_drops(self) -> None:
        ctx = _ctx(text="фон", text_lower="фон", skip_counter={})
        host = _RecordingHost(accumulate=False)
        out = _admission().evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.DROP
        assert out.step_name == "wake_word"
        assert out.reason == "no_wake_word"
        # Counter incremented.
        assert ctx.skip_counter.get("no_wake_word") == 1


# ---------------------------------------------------------------------------
# DEFAULT_BARGE_IN_POLICY smoke
# ---------------------------------------------------------------------------


def test_default_barge_in_policy_is_replace() -> None:
    # Mirrors the legacy ``getattr(self, "_barge_in_policy", "replace")``
    # default. A change here is a regression.
    assert DEFAULT_BARGE_IN_POLICY == "replace"


# ---------------------------------------------------------------------------
# Regression — issue #2713: rclpy caches per-call-site severity.
# ---------------------------------------------------------------------------


class _RclpyStyleLogger:
    """Fake rclpy ``RcutilsLogger`` for tests that reproduces issue #2713.

    rclpy's ``RcutilsLogger.log()`` keys its per-call-site cache by the
    caller frame (``file_path``/``line_number``/``function_name``) and
    raises ``ValueError("Logger severity cannot be changed between
    calls.")`` if the same call-site is invoked with a different
    severity. Stdlib ``logging.Logger`` does NOT do that, so the existing
    ``test_logger_warns_on_handled`` test missed the bug.

    We mirror the relevant contract: track the severity we last saw for
    a given ``(file, line)`` and raise on mismatch. Each call goes
    through a single ``info``/``warning``/``error``/``fatal``/``debug``
    shim that records into ``self.records``.
    """

    def __init__(self) -> None:
        self.records: List[Tuple[str, str]] = []
        self._seen_severity: Dict[Tuple[str, int], str] = {}

    def _emit(self, severity: str, message: str) -> None:
        # Mirror rclpy's rclpy.impl.rcutils_logger.RcutilsLogger.log: it
        # uses ``CallerId()`` which is the equivalent of
        # ``inspect.stack()[1]`` — i.e. the immediate caller of the
        # ``info``/``warning`` wrapper. The call chain is:
        #
        #   user_code   -> self.warning(msg)
        #   self.warning -> self._emit(sev, msg)
        #
        # so frame.f_back.f_back is the user's call-site. Each
        # ``self.logger.warning(...)`` / ``self.logger.info(...)`` in
        # user code lives on its own source line; if the user collapses
        # both onto one line via ``log_fn = ...; log_fn(msg)`` the key
        # is shared and we reproduce the rclpy crash.
        import inspect

        frame = inspect.currentframe()
        assert frame is not None and frame.f_back is not None
        caller = frame.f_back.f_back  # skip the info/warning wrapper
        assert caller is not None
        key = (caller.f_code.co_filename, caller.f_lineno)
        prev = self._seen_severity.get(key)
        if prev is not None and prev != severity:
            raise ValueError(
                f"Logger severity cannot be changed between calls. "
                f"({key[0]}:{key[1]} saw {prev!r}, now {severity!r})"
            )
        self._seen_severity[key] = severity
        self.records.append((severity, message))

    def debug(self, message: str) -> None:
        self._emit("DEBUG", message)

    def info(self, message: str) -> None:
        self._emit("INFO", message)

    def warning(self, message: str) -> None:
        self._emit("WARN", message)

    def error(self, message: str) -> None:
        self._emit("ERROR", message)

    def fatal(self, message: str) -> None:
        self._emit("FATAL", message)


class TestIssue2713LoggerCallSites:
    """Regression for issue #2713.

    ``SttAdmission.evaluate`` used to bind ``log_fn = self.logger.warning
    if HANDLED else self.logger.info`` and then call ``log_fn(msg)`` from
    a single source line. rclpy's per-call-site severity cache keyed on
    (file, line) collapsed both branches onto that line, so the very
    first time a DROP followed a HANDLED (or vice versa) the node raised
    ``ValueError: Logger severity cannot be changed between calls.``
    inside ``executor.spin()``, leaving dialogue_node as a zombie.

    These tests prove the fix: each severity now lives on its own source
    line, and the orchestrator can emit any interleaving of DROP/HANDLED
    against a real rclpy-style logger without raising.
    """

    def test_handled_then_drop_does_not_raise(self) -> None:
        # HANDLED followed by DROP must NOT raise
        # "Logger severity cannot be changed between calls."
        logger = _RclpyStyleLogger()
        host_silence = _RecordingHost(handle_silence=True)
        host_drop = _RecordingHost(accumulate=False)

        # HANDLED — silence_command.
        ctx_handled = _ctx(
            text="робот замолчи",
            text_lower="робот замолчи",
            raw="робот замолчи",
            skip_counter={},
        )
        out1 = _admission(logger=logger).evaluate(ctx_handled, host_silence)
        assert out1.kind is SttOutcomeKind.HANDLED
        assert out1.step_name == "silence_command"

        # DROP — wake_word missing.
        ctx_drop = _ctx(text="фон", text_lower="фон", skip_counter={})
        out2 = _admission(logger=logger).evaluate(ctx_drop, host_drop)
        assert out2.kind is SttOutcomeKind.DROP
        assert out2.step_name == "wake_word"

        severities = [s for s, _ in logger.records]
        assert severities == ["WARN", "INFO"]

    def test_drop_then_handled_does_not_raise(self) -> None:
        # Reverse order: DROP first, then HANDLED.
        logger = _RclpyStyleLogger()

        host_drop = _RecordingHost(accumulate=False)
        ctx_drop = _ctx(text="фон", text_lower="фон", skip_counter={})
        out1 = _admission(logger=logger).evaluate(ctx_drop, host_drop)
        assert out1.kind is SttOutcomeKind.DROP

        host_silence = _RecordingHost(handle_silence=True)
        ctx_handled = _ctx(
            text="робот замолчи",
            text_lower="робот замолчи",
            raw="робот замолчи",
            skip_counter={},
        )
        out2 = _admission(logger=logger).evaluate(ctx_handled, host_silence)
        assert out2.kind is SttOutcomeKind.HANDLED

        severities = [s for s, _ in logger.records]
        assert severities == ["INFO", "WARN"]

    def test_many_alternations_do_not_raise(self) -> None:
        # Stress: hammer the logger with alternating severities on the
        # same call-site. Old code raised on the 2nd call; new code must
        # never raise.
        logger = _RclpyStyleLogger()
        for i in range(10):
            if i % 2 == 0:
                host = _RecordingHost(handle_silence=True)
                ctx = _ctx(
                    text="робот замолчи",
                    text_lower="робот замолчи",
                    raw="робот замолчи",
                    skip_counter={},
                )
                out = _admission(logger=logger).evaluate(ctx, host)
                assert out.kind is SttOutcomeKind.HANDLED
            else:
                host = _RecordingHost(accumulate=False)
                ctx = _ctx(text="фон", text_lower="фон", skip_counter={})
                out = _admission(logger=logger).evaluate(ctx, host)
                assert out.kind is SttOutcomeKind.DROP

        severities = [s for s, _ in logger.records]
        assert severities == ["WARN", "INFO"] * 5

    def test_logger_none_still_silences_orchestrator(self) -> None:
        # The fix must not regress the ``logger=None`` test — silence
        # must still work.
        logger = _RclpyStyleLogger()  # never touched
        ctx = _ctx(
            text="робот замолчи",
            text_lower="робот замолчи",
            raw="робот замолчи",
            skip_counter={},
        )
        host = _RecordingHost(handle_silence=True)
        out = _admission(logger=None).evaluate(ctx, host)
        assert out.kind is SttOutcomeKind.HANDLED
        assert logger.records == []  # never called
