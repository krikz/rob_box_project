"""Смена темы под играющим сетом - не стоп (#4347, живой n604 04.11, ADR-0148: решает код).

Живая проверка 04.11 22:00Z (деплой 41907633834): «Робот, уводи в сторону рока, тётя Гриша под рок добреет.» под
club-сетом. Модель прислала в одной команде ``dj_set stop`` + ``dj_set start style=rock`` - валидатор отверг её
``single_action_class`` (два ``action.media``), повтор вернул ОДИН ``dj_set stop``, а ``theme_switch`` считал стоп
исполненной сменой (``dj_set`` в ``THEME_SWITCH_TOOLS``): музыка выключена, рока нет, «Выключил музыку.».

Здесь: пара stop+start одного тула - смена темы (стоп выброшен валидатором, ``knowledge.START_REPLACES_STOP``); один
стоп при просьбе сменить тему сменой не считается - код кладёт ``dj_set start`` с темой словом человека, стоп не
исполняется; стоп без просьбы сменить («выключи музыку») остаётся стопом.
"""

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, ToolCall, parse, validate
from rob_box_dialog.theme_switch import owed_switch, salvage_with_switch, with_switch

from test_theme_switch_4347 import _run

ALL = [t.name for t in TOOL_CATALOG]
#: n604 04.11 дословно (inject.sh).
N604_LIVE = "Робот, уводи в сторону рока, тётя Гриша под рок добреет."
STOP = ToolCall("dj_set", {"action": "stop"})
START = ToolCall("dj_set", {"action": "start", "theme": "ночной рок для тёти Гриши", "style": "rock",
                            "persona": "РОББОКС"})
CODE_SWITCH = ToolCall("dj_set", {"action": "start", "theme": "рока"})
#: Первая попытка модели в живом логе (``verdict=invalid:single_action_class``).
LIVE_FIRST = ('{"actions": [{"tool": "dj_set", "args": {"action": "stop"}}, {"tool": "dj_set", "args": {"action": '
              '"start", "theme": "ночной рок для тёти Гриши", "style": "rock", "persona": "РОББОКС"}}]}')


def _calls(ports):
    return [(tool, args.get("action")) for batch in ports.executed for tool, args in batch]


def test_the_live_stop_plus_start_is_a_valid_theme_switch_not_single_action_class():
    cmd = validate(parse(LIVE_FIRST), ALL)
    assert not isinstance(cmd, Invalid), cmd
    assert [(a.tool, a.args["action"]) for a in cmd.actions] == [("dj_set", "start")]
    assert cmd.actions[0].args["style"] == "rock"
    assert "actions[0].dj_set.stop" in cmd.dropped


def test_n604_live_stop_plus_start_plays_rock_and_stops_nothing_by_hand():
    """Сам n604 04.11: ответ модели через валидатор (его зовёт мост ``command_bridge``) и ход - исполнен один
    ``dj_set start style=rock``, отдельного стопа нет."""
    ports = _run(validate(parse(LIVE_FIRST), ALL), text=N604_LIVE)
    assert _calls(ports) == [("dj_set", "start")]
    assert ports.executed[0][0][1]["style"] == "rock"


def test_n604_live_retry_with_a_lone_stop_switches_the_theme_by_code():
    """Повтор живого лога (один ``dj_set stop``) под просьбой сменить тему: стоп не исполняется, смену кладёт код."""
    ports = _run(Command(actions=(STOP,)), text=N604_LIVE)
    assert ports.executed == [[("dj_set", {"action": "start", "theme": "рока"})]]


def test_a_lone_stop_without_a_switch_ask_stays_a_stop():
    """«Выключи музыку» под сетом - стоп, смены темы нет (Tier-1 music_off #4351 и обычный стоп не меняются)."""
    for text in ("Робот, выключи музыку", "хватит диджей, выключай", "останови сет"):
        assert owed_switch(True, text, (STOP,), ()) is None, text
        assert with_switch(True, text, Command(actions=(STOP,))) == (Command(actions=(STOP,)), "")
    ports = _run(Command(actions=(STOP,)), text="Робот, выключи музыку")
    assert ("dj_set", "stop") in _calls(ports) and ("dj_set", "start") not in _calls(ports)


def test_a_stop_already_done_in_the_turn_does_not_count_as_the_switch():
    fresh, done = with_switch(True, N604_LIVE, Command(speech="Сворачиваю в рок!"), (STOP,))
    assert done == "built" and fresh.actions == (CODE_SWITCH,)


def test_a_vague_switch_with_a_stop_does_not_stop_and_says_honestly():
    fresh, done = with_switch(True, "Робот, смени музыку", Command(actions=(STOP,)))
    assert done == "vague" and fresh.actions == ()


def test_stop_music_is_not_kept_next_to_the_code_switch():
    fresh, done = with_switch(True, N604_LIVE, Command(actions=(ToolCall("stop_music", {}),)))
    assert done == "built" and fresh.actions == (CODE_SWITCH,)


def test_salvaged_stop_yields_to_the_switch():
    assert salvage_with_switch(True, N604_LIVE, (STOP,)) == (CODE_SWITCH,)
    assert salvage_with_switch(True, "выключи музыку", (STOP,)) == (STOP,)


def test_only_a_stop_before_a_start_of_the_same_tool_is_dropped():
    start_then_stop = ('{"actions": [{"tool": "dj_set", "args": {"action": "start", "theme": "рок"}}, '
                       '{"tool": "dj_set", "args": {"action": "stop"}}]}')
    other_tool = ('{"actions": [{"tool": "stop_music", "args": {}}, '
                  '{"tool": "dj_set", "args": {"action": "start", "theme": "рок"}}]}')
    for raw in (start_then_stop, other_tool):
        cmd = validate(parse(raw), ALL)
        assert isinstance(cmd, Invalid) and cmd.reason == "single_action_class", raw


def test_stop_knowledge_lives_in_one_place():
    assert kn.stops_music("dj_set", {"action": "stop"}) and kn.stops_music("stop_music", {})
    assert not kn.stops_music("dj_set", {"action": "start"}) and not kn.stops_music("request_music", {})
    assert kn.replaces_stop("dj_set", {"action": "start"}) and not kn.replaces_stop("dj_set", {"action": "stop"})
