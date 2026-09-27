"""test_issue_2862_utterance_id_topic_race.py

Issue #2862 -- ``dialogue_node`` gets a phrase's ``utterance_id`` on
``/voice/stt/utterance`` and its text on ``/voice/stt/result``. DDS does not
order delivery across topics and both callbacks run in a
``ReentrantCallbackGroup``, so the id may be processed AFTER the text. The
old single "pending slot" then gave the turn ``None`` or the id of the
PREVIOUS phrase (robot evidence in the issue: backlog phrase
``1a2f742eb17c`` leaked into the next turn).

Drives the real ``_on_stt_utterance`` / ``_on_stt`` pair; SttAdmission is
stubbed (PASS / DROP-backlog) and ``_dispatch_cleaned`` records which id
each turn got. Node built via ``object.__new__`` (no ROS2 spin), like
``test_barge_in_policy.py``.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind  # noqa: E402
from rob_box_voice import dialogue_node as dn_mod  # noqa: E402
from rob_box_voice.core.stt_admission import PASS, drop  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

BACKLOG_TEXT = "Я не хочу, чтобы ты нас путал"
TURN_TEXT = "Робот, продолжаю говорить, чтобы ты привык к моему голосу."


def _msg(data: str):
    m = MagicMock()
    m.data = data
    return m


def _utt(uid: str, text: str):
    return _msg(json.dumps({"utterance_id": uid, "text": text}, ensure_ascii=False))


@pytest.fixture()
def node(monkeypatch):
    # Keep the "text before id" wait short so the timeout path is fast.
    monkeypatch.setattr(dn_mod, "_UTTERANCE_ID_WAIT_SEC", 0.3, raising=False)
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    # Attributes DialogueNode.__init__ sets for the utterance-id plumbing
    # (whatever this revision of dialogue_node uses).
    n._pending_utterance_id = None
    n._utterance_id_lock = threading.Lock()
    binder_cls = getattr(dn_mod, "UtteranceIdBinder", None)
    if binder_cls is not None:
        n._utterance_ids = binder_cls()
    n._speaker_by_text = {}
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._wake_words = ["робот"]
    n._llm_skipped_counter = {}
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None
    n._compute_backlog_pending = MagicMock(return_value=False)
    admission = MagicMock()
    # Backlog phrase (no wake word) is held, never becomes a turn.
    admission.evaluate.side_effect = lambda ctx, host: (
        drop("backlog", "no_wake_word") if ctx.text == BACKLOG_TEXT else PASS
    )
    n._stt_admission = admission
    n.dispatched = []
    n._dispatch_cleaned = lambda **kw: n.dispatched.append(
        (kw["clean"], kw.get("utterance_id"))
    )
    return n


def _later(delay: float, fn, *args):
    t = threading.Timer(delay, fn, args=args)
    t.start()
    return t


class TestTextBeforeId:
    def test_turn_gets_its_own_id_when_text_overtakes_id(self, node):
        t = _later(0.05, node._on_stt_utterance, _utt("a5d8a1212530", TURN_TEXT))
        node._on_stt(_msg(TURN_TEXT))
        t.join()
        assert node.dispatched == [(TURN_TEXT, "a5d8a1212530")]

    def test_id_first_still_works(self, node):
        node._on_stt_utterance(_utt("a5d8a1212530", TURN_TEXT))
        node._on_stt(_msg(TURN_TEXT))
        assert node.dispatched == [(TURN_TEXT, "a5d8a1212530")]


class TestIdOfPhraseNAfterTextOfNPlus1:
    def test_late_id_of_previous_phrase_never_reaches_next_turn(self, node):
        # Phrase N: text arrives, its id is late beyond the wait window.
        node._on_stt(_msg("Робот, привет"))
        # Id N finally lands, AFTER phrase N was handled.
        node._on_stt_utterance(_utt("1a2f742eb17c", "Робот, привет"))
        # Phrase N+1: text overtakes its own id again.
        t = _later(0.05, node._on_stt_utterance, _utt("a5d8a1212530", TURN_TEXT))
        node._on_stt(_msg(TURN_TEXT))
        t.join()
        ids = dict(node.dispatched)
        assert ids["Робот, привет"] is None  # honest unknown, not a guess
        assert ids[TURN_TEXT] == "a5d8a1212530"
        assert "1a2f742eb17c" not in ids.values()

    def test_id_never_bound_to_a_different_text(self, node):
        node._on_stt_utterance(_utt("1a2f742eb17c", "Робот, привет"))
        start = time.monotonic()
        node._on_stt(_msg(TURN_TEXT))
        assert node.dispatched == [(TURN_TEXT, None)]
        assert time.monotonic() - start < 1.0


class TestBacklogPhraseIdDoesNotLeak:
    def test_backlog_phrase_id_is_consumed_not_leaked(self, node):
        # Evidence case: backlog phrase text first, its id right after.
        t = _later(0.05, node._on_stt_utterance, _utt("1a2f742eb17c", BACKLOG_TEXT))
        node._on_stt(_msg(BACKLOG_TEXT))
        t.join()
        assert node.dispatched == []  # held in backlog, not a turn
        # Next phrase comes with NO id of its own (e.g. injected text).
        node._on_stt(_msg(TURN_TEXT))
        assert node.dispatched == [(TURN_TEXT, None)]

    def test_telegram_text_does_not_wait_or_steal_voice_id(self, node):
        node._on_stt_utterance(_utt("a5d8a1212530", TURN_TEXT))
        start = time.monotonic()
        node._on_stt(_msg("[TG:42] привет"))
        assert time.monotonic() - start < 0.2
        node._on_stt(_msg(TURN_TEXT))
        assert node.dispatched[-1] == (TURN_TEXT, "a5d8a1212530")
