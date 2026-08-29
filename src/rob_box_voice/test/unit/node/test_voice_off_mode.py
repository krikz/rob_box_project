"""test_voice_off_mode.py — Unit-тесты ``voice_input_mode="off"`` (W3-1).

Карточка W3-1 (docs/plans/2026-08-29-dialogue-mode-wave2-plan.md, провал G1):
``voice_input_mode`` должен реально управлять диалоговой нодой, а не только
логировать смену (см. ``parameters_callback``). Добавляется режим "off":

  - блокирует ТОЛЬКО обычный ReSpeaker-микрофон (``/voice/stt/result`` без
    Telegram-маркера и без Quest-флага) — реплика не порождает LLM-турн;
  - НЕ трогает вход ОПЕРАТОРА: ни Telegram (маркер ``[TG:chat_id]``), ни
    Quest robot-voice (``_on_stt(from_quest=True)``, вызывается из
    ``_on_quest_stt`` при ``voice_input_mode=quest_stt``) — §3.5 спека
    docs/design/dialogue-mode-spec-2026-08-28.md: "off" = «диалог off,
    полное управление оператора», а не «робот оглох совсем».
  - ``voice_input_mode="respeaker"`` (default) — регресс: работает как раньше.

Образец фикстуры — ``test_barge_in_policy.py`` (``object.__new__(DialogueNode)``
+ ручные атрибуты, без реального ROS2, wake-word фраза "робот ...").
"""

from __future__ import annotations

from collections import deque
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.dialogue_node import DialogueNode


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
        "command_intent": 0,
        "quick_decide_ignore": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None

    n._run_task = None
    n._task_lock = MagicMock()
    n._task_lock.__enter__ = MagicMock(return_value=None)
    n._task_lock.__exit__ = MagicMock(return_value=False)
    n._loop = MagicMock()
    n._loop.call_soon_threadsafe = lambda fn, *a, **kw: fn(*a, **kw)
    n._run_cancelled = False
    n._effects = MagicMock()
    n._tts_control_pub = MagicMock()

    n._barge_in_policy = "replace"
    n._pending_user_messages = deque()

    # ADR-0027 §3.4 / W3-1 — режим, который кэширует parameters_callback.
    # По умолчанию — respeaker (как в _declare_params/YAML).
    n._voice_input_mode = "respeaker"
    return n


def _stt(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


class TestVoiceOffModeBlocksRespeaker:
    """voice_input_mode="off" — обычная реплика с ReSpeaker не идёт в LLM."""

    def test_off_mode_no_dispatch(self, node):
        node._voice_input_mode = "off"
        node._on_stt(_stt("робот спой про комара"))
        node._dispatch_turn.assert_not_called()

    def test_off_mode_no_stop_published(self, node):
        """Раз турн даже не стартует — STOP на TTS тоже не уходит."""
        node._voice_input_mode = "off"
        node._on_stt(_stt("робот спой про комара"))
        node._tts_control_pub.publish.assert_not_called()

    def test_off_mode_logs_ignored(self, node):
        node._voice_input_mode = "off"
        node._on_stt(_stt("робот спой про комара"))
        node.get_logger().info.assert_called()
        messages = [c.args[0] for c in node.get_logger().info.call_args_list]
        assert any("voice_input_mode=off" in m for m in messages)


class TestVoiceOffModeDoesNotBlockOperator:
    """§3.5 спека — "off" не трогает Telegram/Quest (вход оператора)."""

    def test_off_mode_still_dispatches_telegram(self, node):
        node._voice_input_mode = "off"
        node._on_stt(_stt("[TG:42] привет"))
        node._dispatch_turn.assert_called_once()

    def test_off_mode_still_dispatches_quest(self, node):
        node._voice_input_mode = "off"
        node._on_stt(_stt("расскажи что видишь"), from_quest=True)
        node._dispatch_turn.assert_called_once()


class TestVoiceRespeakerModeRegression:
    """voice_input_mode="respeaker" (default) — поведение как раньше."""

    def test_respeaker_mode_dispatches_turn(self, node):
        node._voice_input_mode = "respeaker"
        node._on_stt(_stt("робот спой про комара"))
        node._dispatch_turn.assert_called_once()

    def test_respeaker_mode_publishes_stop(self, node):
        node._voice_input_mode = "respeaker"
        node._on_stt(_stt("робот спой про комара"))
        node._tts_control_pub.publish.assert_called_once()
        published = node._tts_control_pub.publish.call_args.args[0]
        assert published.data == "STOP"

    def test_default_mode_field_is_respeaker(self, node):
        """Фикстура моделирует старт до прихода параметра от супервизора."""
        assert node._voice_input_mode == "respeaker"
