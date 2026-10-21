"""Node-тесты адресации кодом и бэклога (ADR-0150 PR-5 #3694: §4.2, §7.3 В12; приёмка A15, A31).

Заменяют тесты удалённого ``SpeechAccumulator`` (блок-инструкция бэклога в LLM) и обёртки «Клод» (#3518): решение —
``rob_box_dialog.address.decide`` внутри приёма реплики, бэклог — ``rob_box_dialog.backlog.Backlog``.
"""

import threading
import time
from unittest.mock import MagicMock

import pytest

from rob_box_dialog.backlog import Backlog
from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.dialogue_node import DialogueNode
from ros_stubs import make_utterance  # noqa: E402


class _Doa:
    """DOA-трек: направление фразы задаёт тест."""

    def __init__(self):
        self.deg = None

    def during(self, start, end):
        return self.deg


class _Faces:
    """Кто в кадре во время фразы: тест держит перед роботом одно лицо (DOA засчитывается только при ком-то в кадре)."""

    def __init__(self):
        self.ids = frozenset({"face-1"})

    def during(self, start, end):
        return self.ids


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.IDLE
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._llm_skipped_counter = {k: 0 for k in (
        "no_wake_word", "silenced", "silence_command", "empty_after_strip", "stt_rejected", "music_stop",
        "command_intent")}
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None
    n._backlog = Backlog()
    n._accumulate_no_wake_enabled = True
    n._pending_unaddressed = None
    n._doa = _Doa()
    n._faces = _Faces()
    n._loop = None
    n.get_parameter = MagicMock(return_value=MagicMock(value="minimax"))
    return n


def _say(node, text, doa=None, **kw):
    """Реплика ReSpeaker, звучавшая только что, с направлением ``doa``."""
    msg = make_utterance(text, **kw)
    now = time.time()
    msg.started_at.sec, msg.ended_at.sec = int(now) - 2, int(now)
    node._doa.deg = doa
    node._on_stt(msg)


def _turn_texts(node):
    return [c.args[0] for c in node._dispatch_turn.call_args_list]


def _logged(node, needle):
    return [str(c.args[0]) for c in node.get_logger().info.call_args_list if needle in str(c.args[0])]


class TestNoWake:
    def test_phrase_without_wake_goes_to_backlog_not_llm(self, node):
        _say(node, "покажи анимацию ускорения", doa=90.0)
        node._dispatch_turn.assert_not_called()
        assert node._backlog.background_phrases(time.time()) == 1
        assert _logged(node, "[backlog] accumulated")

    def test_backlog_off_drops(self, node):
        node._accumulate_no_wake_enabled = False
        _say(node, "покажи анимацию ускорения", doa=90.0)
        assert node._llm_skipped_counter["no_wake_word"] == 1
        node._dispatch_turn.assert_not_called()


class TestBareWakeV12:
    def test_bare_wake_takes_fresh_phrase_of_the_same_direction(self, node):
        """Сценарий (а): «покажи анимацию ускорения» без вейка, через 5 с «Робот» — ход с этой фразой."""
        _say(node, "покажи анимацию ускорения", doa=90.0)
        _say(node, "Робот", doa=100.0)
        assert _turn_texts(node) == ["покажи анимацию ускорения"]
        assert _logged(node, "source=backlog_pickup")

    def test_nobody_in_frame_direction_alone_does_not_pick_up(self, node):
        """Шифу 21.10: «в кадре никого → только голос» — DOA один подхват не даёт."""
        node._faces.ids = frozenset()
        _say(node, "покажи анимацию ускорения", doa=90.0)
        _say(node, "Робот", doa=100.0)
        node._dispatch_turn.assert_not_called()
        assert _logged(node, "голое «робот»")

    def test_bare_wake_without_fresh_phrase_is_reflex_only_zero_llm(self, node):
        """Сценарий (а), вторая половина: «Робот» без свежей фразы — рефлекс, 0 ходов LLM."""
        _say(node, "девяносто пять градусов", doa=250.0)  # телевизор с другой стороны
        _say(node, "робот", doa=90.0)
        node._dispatch_turn.assert_not_called()
        assert _logged(node, "голое «робот»")
        assert node._backlog.background_phrases(time.time()) == 1

    def test_bare_wake_in_silence_is_dropped_silently(self, node):
        node._dsm.current_state = DialogueStateKind.SILENCED
        _say(node, "робот", doa=90.0)
        node._dispatch_turn.assert_not_called()
        assert node._llm_skipped_counter["silenced"] == 1


class TestRecentUnaddressedSnapshot:
    def test_tv_phrases_do_not_reach_llm_snapshot(self, node):
        """Сценарий (б): 5 фраз ТВ без вейка, потом «Робот, который час» — в снимке чужих фраз нет."""
        for i in range(5):
            _say(node, f"в эфире новости номер {i}", doa=250.0)
        _say(node, "робот который час", doa=80.0)
        assert _turn_texts(node) == ["который час"]
        ctx = node._build_dynamic_system_context()
        assert "recent_unaddressed" not in ctx and "новости" not in ctx
        assert _logged(node, "[backlog] recent_unaddressed entries=0 background_phrases=5")

    def test_one_fresh_same_direction_phrase_goes_as_structure(self, node):
        _say(node, "я хочу послушать джаз", doa=80.0)
        _say(node, "робот что посоветуешь", doa=85.0)
        ctx = node._build_dynamic_system_context()
        assert '<recent_unaddressed>{"text": "я хочу послушать джаз"' in ctx
        assert '"same_direction": true' in ctx
        assert "recent_unaddressed" not in node._build_dynamic_system_context(), "снимок — на один ход"


class TestTelegram:
    def test_coordinator_message_never_reaches_dialogue(self, node):
        """Сценарий (г): «Клод привет» из Telegram — NotAddressed(coordinator), робот молчит."""
        node._on_stt(make_utterance("[TG:42] Клод привет"))
        node._dispatch_turn.assert_not_called()
        node.get_logger().warning.assert_called()
        assert "NotAddressed(coordinator)" in str(node.get_logger().warning.call_args.args[0])

    def test_regular_telegram_message_is_addressed_without_wake(self, node):
        node._on_stt(make_utterance("[TG:42] включи музыку"))
        assert _turn_texts(node) == ["включи музыку"]
