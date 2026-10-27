"""#4073: ход «ответ одним лицом» (``command.expression`` без действий и без речи) отмечен в окне LLM (ADR-0148, ADR-0150
§7.3, продолжение #4066).

Марафон 27.10 (dev-d68a784), акт 9: n904 «покажи, как тебе грустно…» → ``command.expression sad``, действий нет, лицо
показано (``лицо команды на done: emotion=sad``). На n905 в окне стояла реплика человека без ответа, модель «догнала»
её и показала ``sad`` вместо победной анимации. Метка #4066 ``[ответ действием: …]`` у такого хода не ставилась.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.expression import face_label
from rob_box_dialog.face_scene import FaceScene
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.turn_log import ASSISTANT, TOOL, USER, TurnLog, WindowEntry

N_SAD = "покажи как тебе грустно"
N_VICTORY = "а теперь покажи победную анимацию"
SAD_FACE = "[ответ лицом: sad — показано]"
SAD = Expression(emotion="sad")


def _dialogue(window):
    return [(e.role, e.content) for e in window if e.role != TOOL]


# -- журнал: запись о показанном лице ------------------------------------------------------------------------------


def test_turn_answered_by_a_face_alone_gets_a_code_reply_after_the_user_line():
    log = TurnLog()
    log.begin("t1", 0, N_SAD, "незнакомый голос")
    assert log.faced("t1", "sad")
    assert log.window() == (WindowEntry(USER, N_SAD, "незнакомый голос"), WindowEntry(ASSISTANT, SAD_FACE))


def test_the_reply_follows_the_user_line_in_order_not_user_lines_in_a_row():
    log = TurnLog()
    log.begin("t1", 0, N_SAD)
    log.faced("t1", "sad")
    log.begin("t2", 0, N_VICTORY)
    assert _dialogue(log.window()) == [(USER, N_SAD), (ASSISTANT, SAD_FACE), (USER, N_VICTORY)]


def test_spoken_reply_wins_over_the_face_reply():
    log = TurnLog()
    log.begin("t1", 0, "как дела")
    log.faced("t1", "happy")
    log.said("t1", "s1", "Отлично!")
    log.spoken("s1", 8)
    assert _dialogue(log.window()) == [(USER, "как дела"), (ASSISTANT, "Отлично!")]


def test_action_reply_wins_over_the_face_reply_one_answer_per_turn():
    """Лицо рядом с действием отдельной метки не получает: ход уже отвечен меткой #4066."""
    log = TurnLog()
    log.begin("t1", 0, "включи сирену")
    log.faced("t1", "surprised")
    log.result("t1", ActionResult("t1/0", 0, "personality", "play_sound", "action.media", "done", None,
                                  {"sound": "robot_alert"}))
    assert _dialogue(log.window())[-1] == (ASSISTANT, "[ответ действием: play_sound robot_alert — сделано]")


def test_nothing_shown_nothing_written():
    log = TurnLog()
    log.begin("t1", 0, "что-нибудь")
    assert not log.faced("t1", "") and not log.faced("нет такого", "sad")
    assert log.window() == (WindowEntry(USER, "что-нибудь"),)


def test_turn_without_user_line_gets_no_face_reply():
    log = TurnLog()
    log.begin("t1", 0, "")
    log.faced("t1", "sad")
    assert log.window() == ()


def test_face_reply_goes_with_the_turn_on_reset_and_counts_in_the_budget():
    log = TurnLog(max_tokens=10_000, estimate=len)
    log.begin("t1", 0, N_SAD)
    log.faced("t1", "sad")
    assert log.tokens() == len(N_SAD) + len(SAD_FACE)
    log.reset()
    assert log.window() == () and log.tokens() == 0


def test_heard_backlog_turn_is_answered_after_all_heard_lines():
    log = TurnLog()
    log.begin("t1", 0, "робот", heard=(("Саша", "покажи грусть"),))
    log.faced("t1", "sad")
    assert _dialogue(log.window()) == [(USER, "покажи грусть"), (ASSISTANT, SAD_FACE)]


# -- слова метки: одна таблица -------------------------------------------------------------------------------------


@pytest.mark.parametrize("expression, want", [
    (SAD, "sad"),
    (Expression(emotion="sleepy"), "sleep"),                          # имя сцены лица, как её видит face_node
    (Expression(emotion="neutral", gesture="wink"), "wink"),
    (Expression(emotion="happy", gesture="wink"), "happy, wink"),
    (Expression(emotion="victory", gesture="victory"), "victory"),     # эмоция и жест с одним именем - один раз
    (Expression(emotion="neutral", face=FaceScene(eyes="wide")), kn.WINDOW_FACE_SCENE),
    (Expression(emotion="sad", face=FaceScene(eyes="wide")), f"sad, {kn.WINDOW_FACE_SCENE}"),
    (Expression(emotion="neutral"), ""),                               # нейтраль - показывать нечего
    (Expression(emotion="sad", gesture="no_such_gesture"), "sad"),     # жест вне таблицы лицо не красит
])
def test_face_label_names_what_the_face_shows(expression, want):
    assert face_label(expression.emotion, expression.gesture, expression.face) == want


# -- ход целиком: что уходит модели на следующем ходу (живой лог 27.10, акт 9) -----------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия - аргументы вызова."""

    def __init__(self, answers):
        self.answers, self.asked, self.said, self.shown = list(answers), [], [], []
        self.gate = None  # событие: ход ждёт ответа LLM, пока его не поставят

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        if self.gate is not None:
            await self.gate.wait()
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool], "done",
                             None, dict(a.args)) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        if stage == "done" and expression is not None:
            self.shown.append(expression.emotion)

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


def test_next_turn_sees_the_face_only_answer_in_the_window():
    """n904 → n905: ответ лицом без слов и действий; на следующем ходу окно модели - реплика, ответ лицом, новая реплика,
    а не две реплики человека подряд."""
    ports = Ports([Answer(Command(expression=SAD), "deepseek"),
                   Answer(Command(actions=(ToolCall("play_animation", {"animation": "victory"}),)), "deepseek")])
    engine = _engine(ports)
    asyncio.run(engine.run(Addressed(N_SAD, VOICE)))
    assert ports.said == [] and ports.shown == ["sad"], "лицо показано молча, по просьбе показать"
    asyncio.run(engine.run(Addressed(N_VICTORY, VOICE)))
    history = ports.asked[1]
    assert [m["role"] for m in history] == ["system", "user", "assistant", "user"]
    assert history[2]["content"] == SAD_FACE
    assert history[-1]["content"].endswith(N_VICTORY)


def test_face_that_was_not_shown_is_not_written_to_the_window():
    """Лицо не показано (в кадрах появилась тишина: заклеенный рот, ``done_expression`` пуст) - в окне ответа лицом нет,
    запись не врёт «показано»."""
    ports = Ports([Answer(Command(expression=SAD), "deepseek")])
    ports.gate = asyncio.Event()
    engine = _engine(ports)

    async def scenario():
        turn = asyncio.ensure_future(engine.run(Addressed(N_SAD, VOICE)))
        await asyncio.sleep(0)
        engine.session.push_scene("Silence", "user")
        ports.gate.set()
        return await turn

    asyncio.run(scenario())
    assert ports.shown == [] and ports.said == []
    assert _dialogue(engine.log.window()) == [(USER, N_SAD)]


def test_finish_stage_hands_back_the_face_the_reflex_saw():
    """Окно пишет то лицо, которое увидел итог хода, а не своё второе решение: отменённый ход его не имеет."""
    from rob_box_dialog.deadline import TurnDeadline
    from rob_box_dialog.turn import _Turn, finish_stage

    ports = Ports([])
    turn = _Turn("t1", 0, N_SAD, VOICE, TurnDeadline(30.0), stage="done", expression=SAD)
    snap = Session("personality").snapshot
    assert finish_stage(ports, turn, snap) == SAD and ports.shown == ["sad"]
    turn.status = "cancelled"
    assert finish_stage(ports, turn, snap) is None and ports.shown == ["sad"]
