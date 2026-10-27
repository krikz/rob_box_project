"""#4066: ход «только действие» закрыт в окне LLM репликой робота, которую строит код из исходов (ADR-0148, ADR-0150
§7.3, поправка #4066).

Живой робот 27.10 06:36 (образ dev-d2144c9): «покажи молча, как тебе грустно» → только ``play_animation(sad)``. В
истории модели реплика человека осталась без ответа, следующий ход («что у тебя в музыкальной библиотеке?») увидел две
реплики человека подряд и «догнал» первую — повторил ``play_animation sad`` рядом с ``music_library list``. Исходы
лежали в снимке ``recent_actions``, но окно выглядело как неотвеченный вопрос.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.turn_log import (ASSISTANT, PERFORM_DONE, PERFORM_INTERRUPTED, PERFORM_STARTED, PERFORM_STOPPED,
                                     PERFORM_TOOL, TOOL, USER, TurnLog, WindowEntry)

N_SAD = "покажи молча как тебе грустно"
SAD_CALL = ToolCall("play_animation", {"animation": "sad", "duration": 6})
N_LIBRARY = "что у тебя в музыкальной библиотеке"
N_SIREN = "включи сирену"
SAD_REPLY = "[ответ действием: play_animation sad — сделано]"


def _result(ref, tool="play_animation", status="done", reason=None, **data):
    return ActionResult(turn_id=ref, epoch=0, agent="personality", tool=tool, cls=kn.TOOL_CLASSES.get(tool, "query"),
                        status=status, reason=reason, data=data)


def _dialogue(window):
    """Что модель видит историей (``turn._messages``: ``tool`` уходит в снимок ``recent_actions``)."""
    return [(e.role, e.content) for e in window if e.role != TOOL]


# -- модуль окна ---------------------------------------------------------------------------------------------------


def test_action_only_turn_is_answered_by_a_code_reply_after_the_user_line():
    log = TurnLog()
    log.begin("t1", 0, N_SAD, "незнакомый голос")
    log.result("t1", _result("t1/0", animation="sad", duration=6))
    window = log.window()
    assert [e.role for e in window] == [USER, TOOL, ASSISTANT]
    assert window[-1] == WindowEntry(ASSISTANT, SAD_REPLY), "тул, ключевой аргумент и статус — словами кода"
    assert dict(window[1].content)["status"] == "done", "структурная запись исхода — как была (recent_actions)"


def test_turn_with_spoken_reply_is_unchanged():
    log = TurnLog()
    log.begin("t1", 0, N_SIREN)
    log.result("t1", _result("t1/0", animation="police_lights"))
    log.said("t1", "s1", "Включаю!")
    log.spoken("s1", 8)
    assert [(e.role, e.content) for e in log.window() if e.role != TOOL] == [(USER, N_SIREN), (ASSISTANT, "Включаю!")]


def test_two_action_only_turns_in_a_row_each_get_their_own_reply():
    """06:36:41 «включи сирену» видел две реплики человека подряд без ответа — теперь у каждой своя реплика робота."""
    log = TurnLog()
    log.begin("t1", 0, N_SAD)
    log.result("t1", _result("t1/0", animation="sad"))
    log.begin("t2", 0, N_LIBRARY)
    log.result("t2", _result("t2/0", animation="sad"))
    log.result("t2", _result("t2/1", tool=kn.LIBRARY_TOOL, action="list", count=213))
    assert _dialogue(log.window()) == [
        (USER, N_SAD), (ASSISTANT, SAD_REPLY),
        (USER, N_LIBRARY), (ASSISTANT, "[ответ действием: play_animation sad — сделано; music_library list — сделано]"),
    ]


def test_turn_without_actions_and_without_speech_gets_no_reply():
    """Ход ``passed`` (бэклог, отвергнутая команда #3145): ни действий, ни речи — новой записи нет."""
    log = TurnLog()
    log.begin("t1", 0, "сделай что-нибудь")
    assert log.window() == (WindowEntry(USER, "сделай что-нибудь"),)


def test_turn_without_user_line_gets_no_reply():
    """Ход без слов человека (реплика пуста) — отвечать не на что, записи кода нет."""
    log = TurnLog()
    log.begin("t1", 0, "")
    log.result("t1", _result("t1/0", animation="sad"))
    assert [e.role for e in log.window()] == [TOOL]


def test_reply_shows_the_refusal_and_its_reason():
    log = TurnLog()
    log.begin("t1", 0, N_SIREN)
    log.result("t1", _result("t1/0", tool="play_sound", status="rejected", reason="busy", sound="robot_alert"))
    assert log.window()[-1].content == "[ответ действием: play_sound robot_alert — отказ (busy)]"


def test_tool_without_key_argument_is_named_alone():
    log = TurnLog()
    log.begin("t1", 0, "выключи музыку")
    log.result("t1", _result("t1/0", tool="stop_music", was_playing=True))
    assert log.window()[-1].content == "[ответ действием: stop_music — сделано]"


def test_late_outcome_updates_the_reply():
    """Поздний итог (``TurnLog.late``) меняет запись исхода — реплика кода строится из неё при выдаче окна."""
    log = TurnLog()
    log.begin("t1", 0, "поставь ровный ритм")
    log.result("t1", _result("t1/0", tool="request_music", status="timeout", intent="track"))
    assert log.window()[-1].content == "[ответ действием: request_music track — итога нет в срок]"
    assert log.late(_result("t1/0", tool="request_music", intent="track"), lambda _r: None)
    assert log.window()[-1].content == "[ответ действием: request_music track — сделано]"


def test_reply_gives_way_to_the_spoken_line():
    """Реплика кода — только пока ответа словами в окне нет: прозвучала фраза хода — в окне она, а не запись кода."""
    log = TurnLog()
    log.begin("t1", 0, "включи музыку")
    log.result("t1", _result("t1/0", tool="request_music", intent="track"))
    log.said("t1", "s1", "Включаю музыку.")
    assert log.window()[-1].content == "[ответ действием: request_music track — сделано]", "речь ещё звучит"
    log.spoken("s1", 15)
    assert _dialogue(log.window()) == [(USER, "включи музыку"), (ASSISTANT, "Включаю музыку.")]


def test_reply_counts_in_the_token_budget_and_old_turns_drop_whole():
    log = TurnLog(max_tokens=60, estimate=len)
    log.begin("t1", 0, "aaaa")
    log.result("t1", _result("t1/0", animation="sad"))
    log.begin("t2", 0, "bbbb")
    log.said("t2", "s2", "cc")
    log.spoken("s2", 2)
    assert [e.content for e in log.window()] == ["bbbb", "cc"], "ход t1 с записью кода выпал целиком"
    assert log.tokens() == 6
    big = TurnLog(max_tokens=10_000, estimate=len)
    big.begin("t1", 0, "aaaa")
    big.result("t1", _result("t1/0", animation="sad"))
    assert big.tokens() == sum(len(e.text()) for e in big.window()) and len(SAD_REPLY) < big.tokens()


def test_reset_drops_the_reply_with_the_turn():
    log = TurnLog()
    log.begin("t1", 0, N_SAD)
    log.result("t1", _result("t1/0", animation="sad"))
    log.reset()
    assert log.window() == ()


def test_heard_backlog_turn_with_actions_is_answered_after_all_heard_lines():
    log = TurnLog()
    log.begin("t1", 0, "робот", heard=(("Саша", "покажи грусть"),))
    log.result("t1", _result("t1/0", animation="sad"))
    assert _dialogue(log.window()) == [(USER, "покажи грусть"), (ASSISTANT, SAD_REPLY)]


# -- знание: одна таблица ------------------------------------------------------------------------------------------


@pytest.mark.parametrize("status", [*kn.ACTION_STATUSES, PERFORM_STARTED, PERFORM_DONE, PERFORM_STOPPED,
                                    PERFORM_INTERRUPTED])
def test_every_status_has_words(status):
    assert kn.WINDOW_ACTION_STATUS[status], "статус исполнителя или выступления без слов — сырой код в окне"


def test_key_arguments_are_for_known_tools():
    assert set(kn.WINDOW_ACTION_ARG) - set(kn.TOOL_CLASSES) == {PERFORM_TOOL}


# -- ход целиком: что уходит модели (живой лог 27.10 06:36) ----------------------------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия — аргументы вызова, данные тула поверх них."""

    def __init__(self, answers):
        self.answers, self.asked, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
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


def test_next_turn_sees_the_action_only_turns_answered_not_user_lines_in_a_row():
    """Два хода «только анимация» по прямой просьбе подряд, затем вопрос: в истории модели у каждой просьбы — своя
    реплика робота из исходов, реплики человека не идут подряд без ответа."""
    happy = ToolCall("play_animation", {"animation": "happy"})
    ports = Ports([Answer(Command(actions=(SAD_CALL,)), "deepseek"), Answer(Command(actions=(happy,)), "deepseek"),
                   Answer(Command(speech="Двести тринадцать треков."), "deepseek")])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    for text in (N_SAD, "покажи как ты радуешься", N_LIBRARY):
        asyncio.run(engine.run(Addressed(text, VOICE)))
    assert ports.said == ["Двести тринадцать треков."], "обе анимации — молча, по прямой просьбе"
    history = ports.asked[2]
    assert [m["role"] for m in history] == ["system", "user", "assistant", "user", "assistant", "user"]
    assert history[2]["content"] == SAD_REPLY
    assert history[4]["content"] == "[ответ действием: play_animation happy — сделано]"
    assert history[-1]["content"].endswith(N_LIBRARY), "реплика этого хода — последней, со снимком"
