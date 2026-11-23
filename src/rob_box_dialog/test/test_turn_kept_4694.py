"""Отметка хода строится из факта исполнения, не из обещания модели (ADR-0148, #4694 Я)."""

import asyncio

from rob_box_dialog.address import Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.turn import UNKEPT, Answer, TurnEngine, turn_kept
from test_turn import Ports, _engine


def test_promised_tool_not_called_is_not_success():
    assert turn_kept("finished", ("stop_music",), (), False, ()) == UNKEPT
    assert turn_kept("done", ("get_current_time",), (), False, ()) == UNKEPT
    assert turn_kept("finished", ("stop_music",), ("stop_music",), False, ()) == "finished"
    assert turn_kept("failed", ("stop_music",), (), False, ()) == "failed"


def test_promised_speech_without_start_is_not_success():
    assert turn_kept("finished", (), (), True, ()) == UNKEPT
    assert turn_kept("finished", (), (), True, ("sp1",)) == "finished"
    assert turn_kept("finished", (), (), False, ()) == "finished"


def test_turn_that_accepted_a_tool_and_never_called_it_is_unkept(monkeypatch):
    """Принятый тул не дошёл до вызова: агрегат не врёт ``finished``."""
    import rob_box_dialog.turn as turn_mod

    async def drop(_ports, actions, _snap, _now, turn_id, _log=None):
        assert actions and actions[0].tool == "memory_save"
        return []

    monkeypatch.setattr(turn_mod, "execute_for_speaker", drop)
    call = ToolCall("memory_save", {"fact": "любит чай", "kind": "preference"})
    ports = Ports(answers=[Answer(Command(actions=(call,)), "deepseek")])
    agg = asyncio.run(_engine(ports).run(Addressed("запомни что я люблю чай", "voice")))
    assert ports.executed == []
    assert agg.status == UNKEPT


def test_turn_that_asked_for_speech_and_never_started_it_is_unkept(tmp_path):
    """Речь в динамики запрошена, SpeechEvent started нет: отчёт хода не успех.

    Агрегат закрывается до события владельца (его пишет хост после ``run``). Отметку поэтому ставит отчёт
    по событиям e2e, а не статус агрегата на возврате из хода."""
    from rob_box_dialog.e2e_events import open_events

    events = open_events(str(tmp_path / "e2e.db"), "s1")
    ports = Ports(answers=[Answer(Command(speech="Привет!"), "deepseek")])
    engine = _engine(ports, events=events)
    agg = asyncio.run(engine.run(Addressed("привет", "voice")))
    events.close()
    assert ports.said and agg.status == "finished"
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "dialog"))
    import e2e_db

    turn = e2e_db.load(tmp_path / "e2e.db")[0]
    assert turn.awaits_start and turn.kept() is False
    assert turn_kept(agg.status, (), (), speech_owed=True, started=()) == UNKEPT


def test_chat_speech_does_not_wait_for_speech_started():
    """Текст в чат доставлен сразу: события старта нет, и это не ложь об успехе."""
    ports = Ports(answers=[Answer(Command(speech="Привет!"), "deepseek")])
    engine = TurnEngine(Session("telegram"), ports, new_id=lambda: "t1")
    agg = asyncio.run(engine.run(Addressed("привет", "telegram")))
    assert ports.said and agg.status == "finished"
