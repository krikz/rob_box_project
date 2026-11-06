"""Речь о сделанном без тула (#4391 п.2, HIL 04.11; ADR-0148: фразу об успехе строит код по результату тула).

«Удали ещё точку хил-окно» → ``verdict=speech actions=[]``, модель сказала «Готово, хил-окно удалена.» без
``delete_waypoint``. Речь, утверждающая сделанное («удалил», «сохранил точку», «поехал»), без тула, что это делает, -
нарушение ``VIOLATION_DONE_CLAIM``: повтор моста с подсказкой, затем фраза кода. Тот же механизм, что
``music_promise``/``voice_promise``/``search_promise`` (таблица слов в ``knowledge``, сверка в ``words``). Ждёт решения
Шифу: продолжение ``*_promise`` или ``TEMP(ADR-0148)``.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall, explain, parse, unkept_done_claims, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import done_claims

#: Речь модели из HIL 04.11 23:41 UTC дословно (verdict=speech actions=[]).
HIL_LIE = "Готово, хил-окно удалена."
TOOLS = ("delete_waypoint", "clear_waypoints", "list_waypoints", "save_waypoint", "play_animation", "memory_save")


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
    ("Сделал звук глуше и тише.", ("volume",)),       # #4480, М14 n511: ни одного тула, музыка играла дальше
    ("Приглушил музыку.", ("volume",)),
    ("Убавил громкость.", ("volume",)),
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
    "Сделаю потише.",                            # обещание, его ловит music_promise, не done_claim
    "Звук я не сделал глуше: не просили.",
    "Сделал бутерброд.",
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
    _run(ports, "Робот, удали точку хил-окно")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and HIL_LIE not in ports.said
    assert done_claims(ports.said[0]) == ()


def test_the_done_claim_phrase_is_in_the_catalog():
    assert kn.DONE_CLAIM_PHRASE in kn.PHRASE_KEYS and kn.INVALID_COMMAND_PHRASES[kn.VIOLATION_DONE_CLAIM]


VOLUME_LIE = "Сделал звук глуше и тише."
VOLUME_TOOLS = (*TOOLS, "set_volume", "edit_music", "stop_music")


def test_4480_volume_claim_without_a_tool_is_a_violation_with_volume_tools_allowed():
    result = validate(parse({"speech": VOLUME_LIE}), VOLUME_TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert set(result.all[0].allowed) & set(kn.DONE_CLAIMS["volume"].tools)


def test_4480_volume_claim_with_its_tool_is_not_a_violation():
    for tool in kn.DONE_CLAIMS["volume"].tools:
        cmd = Command(speech=VOLUME_LIE, actions=(ToolCall(tool, {}),))
        assert unkept_done_claims(cmd) == ()


def test_4480_exhausted_retry_does_not_voice_the_volume_lie():
    ports = Ports([Answer(Invalid(kn.VIOLATION_DONE_CLAIM, "speech утверждает volume без тула"), "deepseek")])
    _run(ports, "Робот, что там со звуком")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and VOLUME_LIE not in ports.said
