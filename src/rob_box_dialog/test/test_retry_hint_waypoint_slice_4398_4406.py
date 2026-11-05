"""#4398 и #4406 (М11 n109, HIL 05.11): подсказка повтора после запроса не спорит с шапкой; «запомни это место» -
в срезе с ``save_waypoint``; «Готово.» не звучит перед пояснением об обрезке."""

import asyncio
import re
from pathlib import Path

import pytest

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, requested_groups
from rob_box_dialog.turn import Answer, TurnEngine

from test_actions_cap_4391 import Ports, _deletes


# -- #4398 ---------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("reason", sorted(cm.AFTER_QUERY_HINTS))
def test_after_query_no_violation_line_asks_to_redo_actions(reason):
    note = cm.explain(Invalid(reason, "x"), after_query=True)
    assert "без actions" in note.splitlines()[0]
    assert "одно действие" not in note and "в speech" in note
    assert "Допустимо" not in note


def test_before_query_the_single_action_rule_is_unchanged():
    assert "одно действие за ход" in cm.explain(Invalid("single_action_class", "x"))


def test_the_live_n109_hint_has_no_contradiction():
    note = cm.explain(Invalid("single_action_class", "action.media: play_sound, stop_music"), after_query=True)
    assert "без actions" in note and "одно действие за ход" not in note


# -- #4406 ---------------------------------------------------------------------------------------------------------

def _slice(text):
    return [t.name for t in available_tools(Session("personality").snapshot, text=text)]


@pytest.mark.parametrize("text", [
    "Робот, запомни это место как хил-один",     # HIL 05.11 дословно
    "Робот, сохрани точку хил-два",
    "Робот, отметь здесь хил-три",
    "Робот, запомни тут хил-четыре",
    "Робот, запомни точку кухня",
])
def test_save_waypoint_is_in_the_slice_for_verb_plus_place(text):
    assert "navigation" in requested_groups(text)
    names = _slice(text)
    assert "save_waypoint" in names and len(names) <= kn.TOOLS_VISIBLE_MAX


@pytest.mark.parametrize("text", [
    "Робот, запомни, что я не ем лук",
    "Робот, запомни, что я люблю чай",
    "Робот, какое место ты занял?",
])
def test_plain_memory_requests_do_not_open_waypoints(text):
    assert "navigation" not in requested_groups(text)
    assert "save_waypoint" not in _slice(text)


def test_memory_save_still_in_slice_for_remember_a_fact():
    assert "memory_save" in _slice("Робот, запомни, что я не ем лук")


def test_the_phrase_list_waypoints_teaches_opens_the_slice_with_save_waypoint():
    """Подсказка тула и срез не расходятся: фраза из ответа list_waypoints открывает save_waypoint."""
    src = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools/rob_box_mcp_tools/tools/navigation.py"
    taught = re.findall(r"Скажи '([^']*?)\s*\[имя\]'", src.read_text(encoding="utf-8"))
    assert taught, "подсказка list_waypoints пропала - обновить тест"
    for phrase in taught:
        assert "save_waypoint" in _slice(f"Робот, {phrase} хил-один"), phrase


# -- «Готово.» + пояснение ----------------------------------------------------------------------------------------

def test_capped_turn_says_one_phrase_not_done_plus_explanation():
    ports = Ports([Answer(Command(actions=tuple(ToolCall(a["tool"], a["args"]) for a in _deletes())), "deepseek")])
    ids = iter(f"t{n}" for n in range(1, 100))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed("Робот, удали точки хил-вход, хил-кухня, хил-диван и хил-окно", VOICE)))
    assert phrases.text("local.done") not in ports.said
    assert ports.said == [phrases.text(kn.ACTIONS_CAPPED_NAMED_PHRASE, rest="хил-окно")]
