"""Окно ответа после приветствия при встрече (#3776, решение Шифу 21.10, эпик #3683)."""

from __future__ import annotations

import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog import address  # noqa: E402
from rob_box_dialog.address import Addressed, NotAddressed, decide  # noqa: E402
from rob_box_dialog.backlog import Heard  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_voice.core import greeting_window as gw  # noqa: E402
from rob_box_voice.core.identity_question import publish  # noqa: E402
from voice_robot_wake import ROBOT_WAKE  # noqa: E402 — тестовый профиль робота

_FINISHED, _CANCELLED = 4, 5
FOREIGN = "поселения происходили давайте я устрою наше время"


class _Clock:
    t = 1000.0

    def __call__(self):
        return self.t


def _node(clock):
    node = types.SimpleNamespace(_session_pub=MagicMock())
    logger = MagicMock()
    node.get_logger = lambda: logger
    node._session = Session("personality", clock=clock)
    node._session.subscribe(lambda event: publish(node, event))
    return node


def _greeted(clock, name="Денис"):
    node = _node(clock)
    gw.open_window(node, "p1", name, "g1")
    gw.on_speech_event(node, _FINISHED, "g1", "personality")
    return node


def _decide(clock, node, text, after_s, faces=()):
    clock.t += after_s
    heard = Heard(text=text, t=clock.t, source="respeaker", doa_deg=10.0, faces=frozenset(faces))
    return decide(heard, None, ROBOT_WAKE, None, gw.window(node))


def _logged(node, needle):
    return any(needle in c.args[0] for c in node.get_logger().info.call_args_list)


def test_hello_5s_after_the_greeting_is_a_turn_without_wake():
    clock = _Clock()
    node = _greeted(clock)
    assert _decide(clock, node, "привет", 5.0) == Addressed("привет", address.GREETING_REPLY)


def test_4509_phrase_with_greeted_face_out_of_frame_is_a_turn_too():
    """#4509 (Шифу 06.11): лицо в кадре не условие — ответ без лица приветствованного («faces=0») тоже ход."""
    clock = _Clock()
    node = _greeted(clock)
    assert _decide(clock, node, FOREIGN, 5.0, faces=("ольга",)) == Addressed(FOREIGN, address.GREETING_REPLY)
    clock.t -= 5.0
    node = _greeted(clock)
    said = "Я не забыл про тебя, говорю."
    assert _decide(clock, node, said, 5.0) == Addressed(said, address.GREETING_REPLY)


def test_same_phrase_with_greeted_face_in_frame_is_a_turn():
    clock = _Clock()
    node = _greeted(clock)
    assert _decide(clock, node, FOREIGN, 5.0, faces=("денис",)) == Addressed(FOREIGN, address.GREETING_REPLY)


def test_after_20s_it_is_backlog_even_for_hello():
    clock = _Clock()
    node = _greeted(clock)
    assert _decide(clock, node, "привет", 20.0, faces=("денис",)) == NotAddressed(address.NO_WAKE)
    assert node._session.snapshot.top("GreetingReply") is None, "кадр истёк и снят"


def test_window_closed_while_greeting_is_still_speaking_and_open_from_its_end():
    """Пока приветствие звучит — окна нет. С его конца — есть, и в грейсе микрофона тоже (#4509: ответ сразу после
    приветствия терялся; эхо приветствия в грейсе отсеивает ``stt_node`` словами по событию окна ответа)."""
    clock = _Clock()
    node = _node(clock)
    gw.open_window(node, "p1", "Денис", "g1")
    assert _decide(clock, node, "привет", 3.0) == NotAddressed(address.NO_WAKE)  # речь не договорена
    gw.on_speech_event(node, _FINISHED, "g1", "personality")
    assert _decide(clock, node, "привет", 1.0) == Addressed("привет", address.GREETING_REPLY)


def test_first_addressed_phrase_takes_the_window_and_logs():
    clock = _Clock()
    node = _greeted(clock)
    heard = types.SimpleNamespace(source="respeaker", text="привет", t=clock.t + 5)
    gw.take_reply(node, address.GREETING_REPLY, heard)
    assert gw.window(node) is None and _logged(node, "принят")


def test_not_a_reply_is_logged_for_backlog():
    clock = _Clock()
    node = _greeted(clock)
    gw.not_a_reply(node, types.SimpleNamespace(text=FOREIGN, faces=frozenset()))
    assert _logged(node, "не ответ → бэклог")


def test_cancelled_greeting_leaves_no_window_and_no_speech_id_no_frame():
    clock = _Clock()
    node = _node(clock)
    gw.open_window(node, "p1", "Денис", "g1")
    gw.on_speech_event(node, _CANCELLED, "g1", "personality")
    gw.open_window(node, "p1", "Денис", "")
    assert node._session.snapshot.top("GreetingReply") is None


def test_flag_in_knowledge_turns_the_window_off(monkeypatch):
    from rob_box_dialog import knowledge as kn

    clock = _Clock()
    node = _greeted(clock)
    monkeypatch.setattr(kn, "GREETING_REPLY_WITHOUT_WAKE", False)
    assert _decide(clock, node, "привет", 5.0) == NotAddressed(address.NO_WAKE)


@pytest.mark.parametrize("text, ok", [
    ("привет", True), ("Привет!", True), ("и тебе привет", True), ("и тебе", True), ("доброе утро", True),
    ("как дела", True), ("ага", True), ("спасибо", True), ("привет как у тебя дела сегодня вообще", False),
    (FOREIGN, False), ("включи музыку", False),
])
def test_greeting_reply_grammar(text, ok):
    from rob_box_dialog.greeting_reply import is_greeting_reply

    assert is_greeting_reply(text) is ok
