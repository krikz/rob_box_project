"""Действия сверх потолка хода и «сделал» без тула (#4391, HIL 04.11; ADR-0148: фразу об успехе строит код).

1. «Удали точки хил-вход, хил-кухня, хил-диван и хил-окно»: схема команды разрешала три действия, модель урезала
   просьбу сама, исполнено три, прозвучало «Готово.» - четвёртая точка осталась молча. Модель теперь шлёт всё
   (``ACTIONS_ACCEPT_MAX``), код исполняет первые ``ACTIONS_MAX`` и называет остальное (``turn.capped``).
2. «Удали ещё точку хил-окно»: «ещё» между «удали» и «точку» - группа ``waypoints_edit`` не открылась, ``delete_waypoint``
   не было в срезе, модель сказала «Готово, хил-окно удалена.» без тула. Теперь группу открывают глагол и предмет по
   отдельности, а речь о сделанном без тула - нарушение ``VIOLATION_DONE_CLAIM`` (повтор, затем фраза кода).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall, explain, parse, unkept_done_claims, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, requested_groups
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import done_claims

POINTS = ("хил-вход", "хил-кухня", "хил-диван", "хил-окно")
#: Речь модели из HIL 04.11 23:41 UTC дословно (verdict=speech actions=[]).
HIL_LIE = "Готово, хил-окно удалена."
TOOLS = ("delete_waypoint", "clear_waypoints", "list_waypoints", "save_waypoint", "play_animation", "memory_save")


def _deletes(names=POINTS):
    return [{"tool": "delete_waypoint", "args": {"name": n}} for n in names]


# ---------------------------------------------------------------------------------------------------------------
# 1. потолок действий: модель присылает всё, код исполняет ACTIONS_MAX и называет остальное
# ---------------------------------------------------------------------------------------------------------------


def test_the_schema_accepts_more_actions_than_the_turn_executes():
    schema = kn.command_schema("json")
    assert schema["properties"]["actions"]["maxItems"] == kn.ACTIONS_ACCEPT_MAX > kn.ACTIONS_MAX


def test_four_deletes_are_a_valid_command_with_all_four_actions():
    cmd = validate(parse({"actions": _deletes()}), TOOLS)
    assert isinstance(cmd, Command)
    assert [a.args["name"] for a in cmd.actions] == list(POINTS)


def test_more_than_the_accept_limit_is_still_a_violation():
    many = [{"tool": "delete_waypoint", "args": {"name": f"т{n}"}} for n in range(kn.ACTIONS_ACCEPT_MAX + 1)]
    result = validate(parse({"actions": many}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == "too_many_actions"


class Ports:
    """Транспорт хода: модель отвечает заготовками, исполнитель всё делает (``done``), речь - в ``said``."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.asked = []
        self.executed = []
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            self.executed.append((call.tool, dict(call.args)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, dict(call.args)))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text):
    ids = iter(f"t{n}" for n in range(1, 100))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return engine


def _command(actions):
    return Command(actions=tuple(ToolCall(a["tool"], a["args"]) for a in actions))


def test_hil_four_deletes_execute_three_and_the_code_names_the_fourth():
    """HIL 04.11: четыре удаления - исполнены первые три, а четвёртое не молча: фраза кода называет «хил-окно»."""
    ports = Ports([Answer(_command(_deletes()), "deepseek")])
    _run(ports, "Робот, удали точки хил-вход, хил-кухня, хил-диван и хил-окно")
    assert [args["name"] for _, args in ports.executed] == list(POINTS[:kn.ACTIONS_MAX])
    capped = phrases.text(kn.ACTIONS_CAPPED_NAMED_PHRASE, rest="хил-окно")
    assert ports.said[-1] == capped
    assert "хил-окно" in ports.said[-1]


def test_up_to_the_limit_nothing_extra_is_said():
    ports = Ports([Answer(_command(_deletes(POINTS[:kn.ACTIONS_MAX])), "deepseek")])
    _run(ports, "Робот, удали точки хил-вход, хил-кухня, хил-диван")
    assert len(ports.executed) == kn.ACTIONS_MAX
    assert not any("столько не делаю" in s for s in ports.said)


def test_a_capped_call_without_a_string_argument_gets_the_general_phrase():
    acts = [{"tool": "delete_waypoint", "args": {"name": n}} for n in POINTS[:kn.ACTIONS_MAX]]
    acts.append({"tool": "clear_waypoints", "args": {}})
    ports = Ports([Answer(_command(acts), "deepseek")])
    _run(ports, "Робот, удали точки и очисти все точки")
    assert ports.said[-1] == phrases.text(kn.ACTIONS_CAPPED_PHRASE)


def test_the_capped_phrases_are_in_the_catalog_and_claim_nothing():
    for key in (kn.ACTIONS_CAPPED_PHRASE, kn.ACTIONS_CAPPED_NAMED_PHRASE):
        assert key in kn.PHRASE_KEYS
        assert done_claims(phrases.text(key, rest="хил-окно")) == ()


# ---------------------------------------------------------------------------------------------------------------
# 2а. срез тулов: «удали ещё точку» открывает delete_waypoint
# ---------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text", [
    "Робот, удали ещё точку хил-окно",          # HIL 04.11 дословно
    "Робот, удали точку кухня",
    "Робот, удали все точки",
    "Робот, точку хил-окно тоже удали",
    "Робот, сотри эту точку",
])
def test_delete_waypoint_is_in_the_slice_when_the_human_asks_to_delete_a_point(text):
    assert "waypoints_edit" in requested_groups(text)
    names = [t.name for t in available_tools(Session("personality").snapshot, text=text)]
    assert "delete_waypoint" in names and len(names) <= kn.TOOLS_VISIBLE_MAX


@pytest.mark.parametrize("text", ["Робот, какие у тебя точки?", "Робот, удали трек из библиотеки"])
def test_without_verb_and_object_together_the_edit_group_stays_closed(text):
    assert "waypoints_edit" not in requested_groups(text)


# ---------------------------------------------------------------------------------------------------------------
# 2б. речь о сделанном без тула - нарушение, после повтора говорит код
# ---------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text, kinds", [
    (HIL_LIE, ("delete",)),
    ("Удалил точку кухня.", ("delete",)),
    ("Все точки стёрты.", ("delete",)),
    ("Сохранил точку у окна.", ("waypoint_save",)),
    ("Точку запомнил, называется кухня.", ("waypoint_save",)),
    ("Поехал на кухню!", ("motion",)),
    ("Выезжаю к окну.", ("motion",)),
])
def test_done_claim_words(text, kinds):
    assert done_claims(text) == kinds


@pytest.mark.parametrize("text", [
    "Точку хил-окно я уже удалил раньше.",       # было в прошлом ходе
    "Не удалил: такой точки нет.",
    "Удалить точку хил-окно?",
    "Запомнил, тебя зовут Саша.",                # знакомство - речь кода, не точка
    "Музыку выключил.",                          # после кодового стопа - правда (MUSIC_PROMISES)
    "Люблю вкусную еду.",                        # «еду» - не езда
])
def test_not_a_done_claim(text):
    assert done_claims(text) == ()


def test_hil_lie_without_a_tool_is_a_violation_with_a_hint_and_the_tools_that_would_do_it():
    result = validate(parse({"speech": HIL_LIE}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert "delete_waypoint" in result.all[0].allowed
    assert kn.DONE_CLAIM_HINT in explain(result)


def test_the_claim_with_its_tool_is_not_a_done_claim():
    cmd = parse({"actions": [{"tool": "delete_waypoint", "args": {"name": "хил-окно"}}]})
    assert unkept_done_claims(cmd) == ()
    cmd = Command(speech=HIL_LIE, actions=(ToolCall("delete_waypoint", {"name": "хил-окно"}),))
    assert unkept_done_claims(cmd) == ()


def test_after_the_exhausted_retry_the_code_says_nothing_was_done():
    """Повтор моста не помог - невалидная команда доходит до хода: ложь «удалена» не звучит, звучит фраза кода."""
    ports = Ports([Answer(Invalid(kn.VIOLATION_DONE_CLAIM, "speech утверждает delete без тула"), "deepseek")])
    _run(ports, "Робот, удали ещё точку хил-окно")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and HIL_LIE not in ports.said
    assert "delete_waypoint" in ports.asked[0]  # 2а: тул был в срезе хода
    assert done_claims(ports.said[0]) == ()
