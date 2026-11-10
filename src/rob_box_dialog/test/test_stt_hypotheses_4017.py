"""#4017 шаг C1: гипотезы распознавания доходят от ``Utterance`` до хода — без смены поведения."""

from __future__ import annotations

import types
from dataclasses import replace

from robot_wake_fixture import ROBOT_WAKE  # noqa: E402 — тестовый профиль робота

from rob_box_core.msg_stub import build_dialog_msgs_module
from rob_box_core.stt_hypotheses import NO_HYPOTHESES, SttHypotheses
from rob_box_dialog.address import BARE_WAKE, Addressed, decide, heard_from
from rob_box_dialog.backlog import Backlog, Heard

WAKES = list(ROBOT_WAKE)
M = build_dialog_msgs_module()


def _stamp(t):
    return types.SimpleNamespace(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def test_utterance_fields_reach_heard():
    msg = M.Utterance(
        text="робот всох в глуши",
        source="respeaker",
        confidence=0.4,
        alternatives=["робот всё глуши"],
        started_at=_stamp(10.0),
        ended_at=_stamp(12.0),
    )
    assert heard_from(msg, now=50.0).hypotheses == SttHypotheses(0.4, ("робот всё глуши",))


def test_utterance_without_fields_parses_as_before():
    old = types.SimpleNamespace(text=" робот ", source="respeaker", started_at=_stamp(10.0), ended_at=_stamp(12.0))
    heard = heard_from(old, now=50.0)
    assert (heard.text, heard.t, heard.hypotheses) == ("робот", 12.0, NO_HYPOTHESES)
    assert heard_from(M.Utterance(text="робот", source="telegram"), now=1.0).hypotheses == NO_HYPOTHESES


def test_turn_from_the_phrase_carries_its_hypotheses_and_the_decision_is_unchanged():
    hyp = SttHypotheses(0.3, ("робот включи рок",))
    with_hyp = decide(Heard("робот включи рак", 100.0, hypotheses=hyp), Backlog(), WAKES)
    without = decide(Heard("робот включи рак", 100.0), Backlog(), WAKES)
    assert isinstance(with_hyp, Addressed) and with_hyp.hypotheses == hyp
    assert without.hypotheses == NO_HYPOTHESES
    # шаг C1 — только данные: текст, источник и всё остальное в ходе те же
    assert replace(with_hyp, hypotheses=NO_HYPOTHESES) == without


def test_bare_wake_pickup_does_not_take_the_wake_phrase_hypotheses():
    """Текст подхвата — из бэклога, а не из фразы «робот»: её гипотезы к нему не относятся."""
    backlog = Backlog()
    backlog.add(Heard("включи что нибудь весёлое", 95.0))
    out = decide(Heard("робот", 100.0, hypotheses=SttHypotheses(0.9, ())), backlog, WAKES)
    assert isinstance(out, Addressed) and out.pickup and out.hypotheses == NO_HYPOTHESES
    bare = decide(Heard("робот", 100.0, hypotheses=SttHypotheses(0.9, ())), Backlog(), WAKES)
    assert bare.source == BARE_WAKE and bare.hypotheses == NO_HYPOTHESES
