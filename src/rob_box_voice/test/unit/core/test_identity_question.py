"""Кадр ``Identity(ask)`` и ``/dialog/session`` в v1 (ADR-0150 §7.1, §7.2, §7.5 п.2; #3695 ч.2c)."""

from __future__ import annotations

import json
import math
import time
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog.session import Session, SpeakerRef  # noqa: E402
from rob_box_dialog.speaker_adapter import AskCandidate  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402


_FINISHED, _CANCELLED, _REJECTED = 4, 5, 7


class _Clock:
    t = 1000.0

    def __call__(self):
        return self.t


def _timed_node(clock, azimuth=90.0):
    """Нода с сессией на управляемых часах: время идёт, когда тест скажет."""
    node = _node(azimuth)
    node._session = Session("personality", clock=clock)
    node._session.subscribe(lambda event: iq.publish(node, event))
    return node


def _node(azimuth=None):
    node = types.SimpleNamespace(_session_pub=MagicMock(), _speaker_ref=SpeakerRef(azimuth_deg=azimuth))
    logger = MagicMock()
    node.get_logger = lambda: logger
    return node


def _ask(node, cand=None):
    """Вопрос ушёл в речь и договорился (для тестов, где время не важно)."""
    iq.open_question(node, cand or AskCandidate("stranger", "unknown"), "q1")
    iq.on_speech_event(node, _FINISHED, "q1", "personality")


def _heard(t, doa=None, source="respeaker", text="меня зовут Гриша"):
    return types.SimpleNamespace(t=t, doa_deg=doa, source=source, text=text)


def test_question_opens_identity_frame_with_azimuth_and_publishes_session():
    node = _node(azimuth=80.0)
    iq.set_speaker(node, SpeakerRef(person_ref=None, azimuth_deg=80.0))
    _ask(node)
    frame = iq.question(node)
    assert (frame.kind, frame.set_by, frame.payload["kind"], frame.payload["azimuth_deg"]) == (
        "Identity", "code", "unknown", 80.0)
    msg = node._session_pub.publish.call_args.args[0]
    assert msg.speaker_azimuth_deg == 80.0 and msg.speaker_name_state == "unknown"
    (scene,) = msg.scenes
    assert scene.kind == "Identity" and json.loads(scene.payload_json)["kind"] == "unknown"
    assert scene.ttl_until.sec > 0


def test_session_state_without_speaker_azimuth_is_nan_and_name_only_from_ref():
    node = _node()
    iq.set_speaker(node, SpeakerRef(person_ref="p1", name="Саша", name_state="confirmed"))
    msg = node._session_pub.publish.call_args.args[0]
    assert math.isnan(msg.speaker_azimuth_deg)
    assert (msg.speaker_id, msg.speaker_name, msg.agent) == ("p1", "Саша", "personality")


def test_first_addressed_phrase_takes_the_question_and_keeps_it_for_the_turn():
    node = _node(azimuth=80.0)
    _ask(node)
    iq.take_answer(node, "identity_answer", _heard(time.time() + 3, doa=85.0))
    assert iq.question(node) is None
    assert node._identity_answer["kind"] == "unknown"
    iq.take_answer(node, "voice", _heard(time.time() + 5))  # следующая фраза — уже не ответ
    assert node._identity_answer is None


def test_telegram_message_does_not_take_the_voice_question():
    node = _node(azimuth=80.0)
    _ask(node)
    iq.take_answer(node, "telegram", _heard(time.time(), source="telegram"))
    assert iq.question(node) is not None and node._identity_answer is None


@pytest.mark.parametrize("kind, text, name", [
    ("unknown", "Гриша", "Гриша"),
    ("unknown", "Гриша.", "Гриша"),
    ("contested", "борис", "Борис"),
    ("unknown", "Гриша пришёл", None),      # не голое имя — решает обычное «меня зовут …» (#2925) или LLM
    ("unknown", "Кракозябра", None),        # не имя из словаря
    ("single", "Саша", None),               # на «Саша, это ты?» ответ — да/нет, не представление
])
def test_bare_name_is_an_introduction_only_as_answer_to_ask_name(kind, text, name):
    node = _node()
    node._identity_answer = {"kind": kind}
    assert iq.answer_name(node, text) == name


def _answer_accepted(clock, node, after_s, doa=95.0):
    """Фраза без вейка с той же стороны через ``after_s`` с после ТЕКУЩЕГО момента принята как ответ?"""
    from rob_box_dialog.speaker_adapter import answer_to_question

    clock.t += after_s
    iq.question(node)  # то же, что делает допуск: истёкшее снимается
    frame = node._session.snapshot.top("Identity")
    return answer_to_question(frame, _heard(clock.t, doa=doa)) is not None


def test_long_turn_answer_does_not_eat_the_window_answer_10s_after_question_end_is_taken():
    """#3771: ответ хода 12 с, вопрос звучит после него; окно 15 с идёт от конца ВОПРОСА."""
    clock = _Clock()
    node = _timed_node(clock)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")  # ход кончился, вопрос ушёл в речь
    clock.t += 12.0 + 1.5 + 3.0                                         # ответ хода + сам вопрос: без события — ждём
    assert iq.question(node) is None and node._session.snapshot.top("Identity") is not None
    iq.on_speech_event(node, _FINISHED, "q1", "personality")            # вопрос договорён
    assert _answer_accepted(clock, node, 10.0) is True


def test_answer_20s_after_question_end_is_too_late():
    clock = _Clock()
    node = _timed_node(clock)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    iq.on_speech_event(node, _FINISHED, "q1", "personality")
    assert _answer_accepted(clock, node, 20.0) is False
    assert node._session.snapshot.top("Identity") is None, "кадр истёк и снят"


def test_answer_during_mic_grace_after_question_is_not_an_answer():
    clock = _Clock()
    node = _timed_node(clock)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    iq.on_speech_event(node, _FINISHED, "q1", "personality")
    assert _answer_accepted(clock, node, 1.0) is False   # микрофон ещё глух (эхо собственной речи)
    assert _answer_accepted(clock, node, 2.0) is True    # 3.0 с после конца: окно открыто


def test_phrase_while_question_is_still_speaking_is_not_an_answer():
    node = _node(azimuth=80.0)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    iq.take_answer(node, "identity_answer", _heard(time.time() + 3, doa=85.0))
    assert node._identity_answer is None
    assert node._session.snapshot.top("Identity") is not None, "кадр ждёт, фразой не снят"


@pytest.mark.parametrize("kind", [_CANCELLED, _REJECTED])
def test_question_cancelled_or_rejected_leaves_no_frame(kind):
    node = _node(azimuth=80.0)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    iq.on_speech_event(node, kind, "q1", "personality")
    assert node._session.snapshot.top("Identity") is None


def test_events_of_other_speech_or_agent_do_not_touch_the_waiting_frame():
    node = _node(azimuth=80.0)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    iq.on_speech_event(node, _FINISHED, "turn-answer", "personality")  # ответ хода кончился раньше вопроса
    iq.on_speech_event(node, _CANCELLED, "q1", "telegram")
    assert iq.question(node) is None and node._session.snapshot.top("Identity") is not None


def test_frame_waiting_for_a_lost_event_is_dropped_by_the_safety_cap():
    from rob_box_dialog import knowledge as kn

    clock = _Clock()
    node = _timed_node(clock)
    iq.open_question(node, AskCandidate("stranger", "unknown"), "q1")
    clock.t += kn.IDENTITY_WAIT_QUESTION_S + 1
    assert iq.question(node) is None and node._session.snapshot.top("Identity") is None


def test_ask_planned_opens_frame_only_when_question_went_to_speech():
    from rob_box_voice.core.speaker_turn import ask_planned

    cand = AskCandidate("stranger", "unknown")
    node = _node(azimuth=80.0)
    node._speak_identity_question = MagicMock(return_value="")
    ask_planned(node, cand)
    assert node._session.snapshot.top("Identity") if hasattr(node, "_session") else True is True
    assert getattr(node, "_session", None) is None or node._session.snapshot.top("Identity") is None
    node._speak_identity_question = MagicMock(return_value="q9")
    ask_planned(node, cand)
    assert node._session.snapshot.top("Identity").payload["speech_id"] == "q9"


def test_bare_name_without_question_is_nothing():
    node = _node()
    node._identity_answer = None
    assert iq.answer_name(node, "Гриша") is None


@pytest.mark.parametrize("text, kind", [
    ("не скажу", "refusal"), ("Гриша", "name"), ("да", "yes"), ("нет", "no"), ("спой песню про ёжика", "other"),
])
def test_take_answer_logs_the_grammar_kind(text, kind):
    node = _node(azimuth=80.0)
    _ask(node)
    iq.take_answer(node, "identity_answer", _heard(time.time() + 3, doa=157.0, text=text))
    assert node._identity_answer["reply"] == kind
    assert f"kind={kind}" in node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize("text, name", [("зови меня Гриша", "Гриша"), ("не скажу", None), ("поселения происходили", None)])
def test_answer_name_registers_dictionary_names_by_code(text, name):
    node = _node()
    node._identity_answer = {"kind": "unknown"}
    assert iq.answer_name(node, text) == name
