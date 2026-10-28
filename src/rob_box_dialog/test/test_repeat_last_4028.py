"""#4028: «повтори / я не расслышал» — команда кода (Tier-1), а не правка выступления LLM (ADR-0148).

Марафон 23.10, n808: «повтори последнее ещё раз, я не расслышал» шло в LLM; во время выступления она правкой
(``edit_pending``) заменила сегменты 3–11. Теперь закрытая грамматика (``knowledge.REPEAT``) узнаёт просьбу, а ход
кодом говорит последнюю ОЗВУЧЕННУЮ реплику робота из журнала (``TurnLog.last_spoken``): 0 вызовов LLM, выступление
не трогается. Метки кода в окне («[ответ действием: …]») — не слова робота и не повторяются.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Performance, ToolCall
from rob_box_dialog.grammar import match, repeat_asked
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.turn_log import TurnLog

#: Сценарий марафона после #4077: n808 идёт сразу после n803, где последний ответ робота - «семью восемь».
N803 = "Робот, стоп, лучше скажи, сколько будет семью восемь."
N808 = "Робот, повтори последнее ещё раз, я не расслышал."
FIFTY_SIX = "Семью восемь — пятьдесят шесть."
NOTHING = phrases.text(kn.REPEAT_NOTHING_PHRASE)


class Ports:
    """Транспорт хода: ответы LLM по очереди, всё сказанное и отданное владельцу выступления записано."""

    def __init__(self, answers=(), perform=None):
        self.answers, self.perform_outcome = list(answers), perform
        self.asked, self.executed, self.said, self.stages, self.performed, self.stopped = [], [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.call_class(a.tool, a.args),
                             "done", None, dict(a.args)) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", **kwargs):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        self.stopped.append(how)
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        self.performed.append((perf.kind, edit))
        return self.perform_outcome


def _engine(ports, agent="personality"):
    return TurnEngine(Session(agent), ports, system_prompt="Ты робот.")


def _turn(engine, text, spoken=True):
    """Ход целиком; каждая реплика, отданная в этом ходе, отзвучала полностью (``SpeechEvent finished``)."""
    before = len(engine._ports.said)
    agg = asyncio.run(engine.run(Addressed(text, VOICE)))
    if spoken:
        for n, line in enumerate(engine._ports.said[before:], before + 1):
            engine.speech_ended(f"sp{n}", len(line))
    return agg


# -- грамматика: таблица слов в knowledge ----------------------------------------------------------------------------


@pytest.mark.parametrize("text", [
    "повтори", N808, "робот повтори последнее еще раз я не расслышал", "Я не расслышал.", "Скажи ещё раз",
    "повтори-ка", "Что ты сказал?", "извини, я не понял, повтори пожалуйста", "не расслышала, повторите",
    "можешь повторить?",
])
def test_repeat_request_is_tier1(text):
    command = match(text, Session("personality").snapshot)
    assert command is not None and command.kind == kn.REPEAT_KIND and not command.disputed


@pytest.mark.parametrize("text", [
    "ещё раз", "сыграй ещё раз", "повтори песню", "повтори за мной: раз, два, три", "я не понял",
    "я не понял, как это работает", "не повторяй", "покажи ещё раз", "повтори последнюю песню",
])
def test_other_phrases_are_not_the_repeat_command(text):
    """Лишнее слово или «ещё раз» без просьбы о словах - не Tier-1: решает модель (повтор действия, песни)."""
    assert not repeat_asked(text)
    command = match(text, Session("personality").snapshot)
    assert command is None or command.kind != kn.REPEAT_KIND


def test_repeat_is_not_heard_in_silence_and_only_the_personality_has_it():
    """Повтор - речь: в тишине Tier-1 его не отдаёт (ход молчит, как вся тишина); у чата и оператора его нет."""
    session = Session("personality")
    session.push_scene("Silence", "user")
    assert match("повтори", session.snapshot) is None
    assert kn.TIER1_KINDS[kn.REPEAT_KIND] is True


# -- ход: последняя озвученная реплика кодом, без LLM ----------------------------------------------------------------


def test_n808_repeats_the_last_spoken_answer_without_llm():
    """n803 → «пятьдесят шесть» прозвучало; n808 → код говорит его дословно, LLM не зовётся."""
    ports = Ports([Answer(Command(speech=FIFTY_SIX), "deepseek")])
    engine = _engine(ports)
    _turn(engine, N803)
    agg = _turn(engine, N808)
    assert len(ports.asked) == 1, "второй ход - 0 вызовов LLM"
    assert ports.said == [FIFTY_SIX, FIFTY_SIX], "повтор дословный"
    assert (agg.stage, agg.llm_calls) == ("done", 0)


def test_repeat_says_all_spoken_lines_of_the_last_answer_in_order():
    """Ответ из двух реплик - фраза кода об исходе (провал тихого действия) и слова модели рядом с ним: повторяются
    обе, по порядку."""
    disco = ToolCall("play_animation", {"animation": "disco"})
    ports = Ports([Answer(Command(speech="Смотри, как я танцую!", actions=(disco,)), "deepseek")])

    async def refuse(actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/0", snapshot.epoch, snapshot.agent, "play_animation", "action.local",
                             "rejected", None, {"animation": "disco"})]

    ports.execute = refuse
    engine = _engine(ports)
    _turn(engine, "Робот, станцуй диско")
    _turn(engine, "повтори")
    first = [phrases.text("local.rejected"), "Смотри, как я танцую!"]
    assert ports.said == [*first, " ".join(first)]


def test_only_the_spoken_part_of_an_interrupted_line_is_repeated():
    """Реплику перебили на середине: человек слышал только начало - его и повторяем («озвученная» реплика)."""
    ports = Ports([Answer(Command(speech="Раз, два, три, четыре, пять."), "deepseek")])
    engine = _engine(ports)
    _turn(engine, "Робот, посчитай до пяти", spoken=False)
    engine.speech_ended("sp1", len("Раз, два, три,"))
    _turn(engine, "Робот, я не расслышал, повтори")
    assert ports.said[-1] == "Раз, два, три,"


def test_code_marks_in_the_window_are_not_repeated():
    """Ход ответил только действием: в окне метка кода «[ответ действием: …]» - это не слова робота. Повтор берёт
    последнюю реплику, которая прозвучала."""
    sad = ToolCall("play_animation", {"animation": "sad"})
    ports = Ports([Answer(Command(speech=FIFTY_SIX), "deepseek"), Answer(Command(actions=(sad,)), "deepseek")])
    engine = _engine(ports)
    _turn(engine, N803)
    _turn(engine, "Робот, покажи молча, как тебе грустно")
    assert any("[ответ действием:" in str(e.content) for e in engine.log.window())
    _turn(engine, "повтори")
    assert ports.said == [FIFTY_SIX, FIFTY_SIX]


def test_nothing_spoken_yet_is_an_honest_code_phrase():
    ports = Ports()
    engine = _engine(ports)
    agg = _turn(engine, N808)
    assert ports.said == [NOTHING] and ports.asked == [] and agg.stage == "done"


def test_new_session_forgets_what_to_repeat():
    ports = Ports([Answer(Command(speech=FIFTY_SIX), "deepseek")])
    engine = _engine(ports)
    _turn(engine, N803)
    engine.log.reset()
    _turn(engine, "повтори")
    assert ports.said[-1] == NOTHING


def test_repeat_during_a_performance_does_not_edit_or_stop_it():
    """n808 на деле: идёт выступление - «повтори…» не уходит модели, и сегменты никто не правит (было: ``edit_pending``
    заменил сегменты 3–11); выступление не останавливается, а повтор - последняя прозвучавшая реплика хода."""
    poem = Performance("poem", ("Шаг — один, шаг — два.", "Шаг — три, разгоняюсь."))
    ports = Ports([Answer(Command(speech=FIFTY_SIX), "deepseek"), Answer(Command(perform=poem), "deepseek")],
                  perform={"status": "started"})
    engine = _engine(ports)
    _turn(engine, N803)
    _turn(engine, "Робот, прочитай стих про шаги")
    assert engine.log.performing() and ports.performed == [("poem", False)]
    stopped = list(ports.stopped)  # «стоп» в n803 - стоп прошлого ответа, до выступления
    _turn(engine, N808)
    assert len(ports.asked) == 2, "повтор - без LLM"
    assert ports.performed == [("poem", False)] and ports.stopped == stopped, "выступление не правится и не гасится"
    assert ports.said[-1] == FIFTY_SIX and engine.log.performing()


def test_text_channel_agent_has_no_repeat_command():
    """Телеграм: «повтори» - слова для его LLM (переписка видна в чате), Tier-1 повтора у агента нет."""
    ports = Ports([Answer(Command(speech="Повторяю: 56."), "deepseek")])
    _turn(_engine(ports, kn.TEXT_CHANNEL_AGENT), "повтори")
    assert len(ports.asked) == 1


# -- журнал: что значит «озвученная» -----------------------------------------------------------------------------


def test_last_spoken_skips_superseded_phrases_and_the_asking_turn():
    """Фраза о таймауте, которую опроверг поздний итог (#3960), не повторяется: музыка всё же заиграла."""
    log = TurnLog()
    log.begin("t1", 0, "включи музыку")
    timeout = ActionResult("t1/0", 0, "personality", "request_music", "action.media", "timeout", None,
                           {"intent": "track"})
    log.result("t1", timeout)
    log.said("t1", "s1", "Музыка пока не заиграла. Но я попробую ещё.")
    log.spoken("s1", 100)
    assert log.last_spoken() == "Музыка пока не заиграла. Но я попробую ещё."
    done = ActionResult("t1/0", 0, "personality", "request_music", "action.media", "done", None, {"intent": "track"})
    log.late(done, lambda r: "Музыка пока не заиграла." if r.status == "timeout" else None)
    assert log.last_spoken() == "Но я попробую ещё."
    log.begin("t2", 0, "повтори")
    log.said("t2", "s2", "Но я попробую ещё.")
    log.spoken("s2", 100)
    assert log.last_spoken(exclude="t2") == "Но я попробую ещё."


def test_marks_and_pending_speech_are_not_spoken():
    log = TurnLog()
    log.begin("t1", 0, "который час")
    log.mark("t1", kn.WINDOW_MUTED_MARK)
    log.said("t1", "s1", "Ещё звучит.")
    assert log.last_spoken() == "", "метка кода и неотзвучавшая реплика - не озвученное"
    log.said("t1", "s2", "Текст в чат.", delivered=True)
    assert log.last_spoken() == "Текст в чат.", "текстовый канал: доставлено - значит сказано"


def test_phrase_keys_of_the_repeat_exist():
    assert kn.REPEAT_NOTHING_PHRASE in kn.PHRASE_KEYS and NOTHING
    assert all(" " not in word for word in kn.REPEAT.filler)
    assert all(word == word.lower().replace("ё", "е") for ask in kn.REPEAT.asks for word in ask)
