"""#4106: ход тишины записан в окно LLM меткой кода «рот заклеен» (ADR-0148, ADR-0150 §7.3, решение Шифу 27.10 20:26).

Живой прогон 27.10: в тишине (#3982) реплика с вейком не доходит до LLM и ничего не звучит, но в окне модели она остаётся
репликой человека без ответа. Шифу: «не надо чинить это кодом, просто от него ход надо писать, что рот занят». После
реплики человека окно ставит реплику робота-метку (``TurnLog.mark``); то же у Tier-1 тишины (включить/снять) - метка-ответ
как у исходов действий (#4066). Фильтров окна и «отвечу позже» нет.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.turn_log import ASSISTANT, TOOL, USER, TurnLog, WindowEntry

MUTED = "[рот заклеен — молчу]"
SILENCE_ON = "[ответ действием: тишина включена, молчу]"
SILENCE_OFF = "[ответ действием: тишина снята, говорю]"


def _dialogue(window):
    """Что модель видит историей (``turn._messages``: ``tool`` уходит в снимок ``recent_actions``)."""
    return [(e.role, e.content) for e in window if e.role != TOOL]


class Ports:
    """Транспорт хода: ответы LLM по очереди, исполнение действий - ``done`` с аргументами вызова в данных."""

    def __init__(self, answers=()):
        self.answers, self.asked, self.said, self.executed = list(answers), [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool], "done",
                             None, dict(a.args)) for n, a in enumerate(actions)]

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


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text, source=VOICE):
    return asyncio.run(engine.run(Addressed(text, source)))


# -- модуль окна ---------------------------------------------------------------------------------------------------


def test_mark_is_a_robot_line_right_after_the_user_line():
    log = TurnLog()
    log.begin("t1", 0, "который час")
    assert log.mark("t1", MUTED)
    assert log.window() == (WindowEntry(USER, "который час"), WindowEntry(ASSISTANT, MUTED))


def test_mark_of_an_unknown_turn_or_an_empty_mark_writes_nothing():
    log = TurnLog()
    log.begin("t1", 0, "который час")
    assert not log.mark("t9", MUTED), "хода нет в журнале (выпал из окна, новая сессия)"
    assert not log.mark("t1", ""), "метки у вида Tier-1 нет - ничего не пишется"
    assert log.window() == (WindowEntry(USER, "который час"),)


def test_mark_is_counted_in_the_window_budget_and_dropped_by_reset():
    log = TurnLog(estimate=len)
    log.begin("t1", 0, "aaaa")
    log.mark("t1", MUTED)
    assert log.tokens() == len("aaaa") + len(MUTED)
    log.reset()
    assert log.window() == () and not log.mark("t1", MUTED)


def test_mark_after_an_action_keeps_the_action_in_recent_actions_and_does_not_double_the_reply():
    """Хвост «хватит, спой другую» в тишине: стоп музыки исполнен, остаток проглочен. Ответ робота в истории один."""
    log = TurnLog()
    log.begin("t1", 0, "хватит, спой другую")
    log.result("t1", ActionResult("t1/0", 0, "personality", "stop_music", kn.TOOL_CLASSES.get("stop_music", "query"), "done", None, {}))
    log.mark("t1", MUTED)
    window = log.window()
    assert [e.role for e in window] == [USER, TOOL, ASSISTANT]
    assert _dialogue(window) == [(USER, "хватит, спой другую"), (ASSISTANT, MUTED)]


def test_knowledge_marks_are_for_tier1_silence_kinds_only():
    assert set(kn.WINDOW_SILENCE_MARKS) == {"silence", "unsilence"} <= set(kn.TIER1_KINDS)
    assert kn.WINDOW_SILENCE_MARKS["silence"] == SILENCE_ON and kn.WINDOW_SILENCE_MARKS["unsilence"] == SILENCE_OFF
    assert kn.WINDOW_MUTED_MARK == MUTED


# -- ход целиком ---------------------------------------------------------------------------------------------------


def test_turn_in_silence_is_answered_by_the_muted_mark_not_left_hanging():
    """Сценарий карточки: «замолчи» -> «который час?» -> «говори»: после «который час?» стоит ход робота «рот занят»."""
    ports = Ports()
    engine = _engine(ports)
    for text in ("замолчи", "который час?", "говори"):
        _run(engine, text)
    assert ports.asked == [] and ports.said == [], "в тишине LLM не зовётся и речи нет"
    assert _dialogue(engine.log.window()) == [
        (USER, "замолчи"), (ASSISTANT, SILENCE_ON),
        (USER, "который час?"), (ASSISTANT, MUTED),
        (USER, "говори"), (ASSISTANT, SILENCE_OFF),
    ]


def test_wake_word_turn_in_silence_is_marked_too():
    """Вейк тишину не снимает (#3983): реплика с вейком в тишине - тот же ход «рот занят»."""
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "замолчи")
    _run(engine, "Робот, привет, как дела")
    assert _dialogue(engine.log.window())[-2:] == [(USER, "Робот, привет, как дела"), (ASSISTANT, MUTED)]
    assert ports.asked == [] and engine.session.snapshot.top("Silence") is not None


def test_stop_with_rest_in_silence_is_marked_and_the_stop_stays_a_recent_action():
    ports = Ports()
    engine = _engine(ports)
    _run(engine, "замолчи")
    _run(engine, "хватит, спой другую")
    assert _dialogue(engine.log.window())[-2:] == [(USER, "хватит, спой другую"), (ASSISTANT, MUTED)]
    assert ports.asked == [] and ports.said == []


def test_next_turn_after_unsilence_sees_the_marks_and_does_not_answer_the_old_question():
    """История модели после «говори»: каждая реплика человека в тишине отвечена меткой, а не висит без ответа."""
    ports = Ports([Answer(Command(speech="Спасибо, хорошо."), "deepseek")])
    engine = _engine(ports)
    for text in ("замолчи", "который час?", "говори"):
        _run(engine, text)
    _run(engine, "как дела")
    history = ports.asked[0]
    assert [m["role"] for m in history] == ["system", "user", "assistant", "user", "assistant", "user", "assistant",
                                            "user"], "реплики человека не идут подряд без ответа"
    said = [m["content"] for m in history[1:7]]  # реплики человека - с тегом «[говорит: …]» (#3881)
    assert [s.endswith(t) for s, t in zip(said[::2], ("замолчи", "который час?", "говори"))] == [True] * 3
    assert said[1::2] == [SILENCE_ON, MUTED, SILENCE_OFF]
    assert history[-1]["content"].endswith("как дела"), "реплика этого хода - последней, со снимком"
    assert ports.said == ["Спасибо, хорошо."]


def test_turn_outside_silence_has_no_mark():
    ports = Ports([Answer(Command(speech="Привет!"), "deepseek")])
    engine = _engine(ports)
    _run(engine, "привет")
    engine.speech_ended("sp1", 7)
    assert _dialogue(engine.log.window()) == [(USER, "привет"), (ASSISTANT, "Привет!")]


@pytest.mark.parametrize("text", ["замолчи", "помолчи"])
def test_repeated_silence_request_is_marked_without_a_second_scene(text):
    ports = Ports()
    engine = _engine(ports)
    _run(engine, text)
    _run(engine, text)
    assert [e.content for e in engine.log.window() if e.role == ASSISTANT] == [SILENCE_ON, SILENCE_ON]
    assert len(engine.session.snapshot.scenes) == 1
