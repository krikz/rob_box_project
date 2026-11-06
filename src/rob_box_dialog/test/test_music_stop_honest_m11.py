"""М11 (05.11, разбор #4251 comment 6648562305; Refs #4236, #4030): стоп музыки - кодом и честно.

* n712 («Робот, всё, глуши.»): STT 02:22:46 «робот заглушил». Tier-1 формы не знал, LLM без тула сказала «Тишина.
  Борису больше не мешаем.», а музыка играла дальше (req45347) - ложь о действии. Теперь (а) «глуши/заглуши/заглушил»
  под музыкой - Tier-1 ``music_off``; (б) речь «тишина / музыку выключил», пока музыка ИГРАЕТ по снимку плеера и стопа в
  ходе не было, - нарушение ``music_promise``: повтор, после повтора - фраза кода по состоянию.
* n608 («Робот, хватит диджеить, выключай.»): Tier-1 ``stop_all`` погасил музыку, а остаток «диджей выключай» уходил
  модели (``get_music_state`` → повтор ``dj_set stop`` → запасная фраза по потолку). Остаток из слов того же стопа - ход
  закрыт кодом, LLM не зовётся.
* n511 («Робот, всё, глуши музыку, у меня уши в трубочку.»): STT «врубай все глуши музыку» - вейк потерян под
  музыкой, фраза ушла в бэклог. Пока музыка играет, фраза без вейка с формой стопа рядом с объектом музыки - ход ровно
  из этого стопа (узкое исключение из «только через вейк»).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import MUSIC_STOP_UNWOKEN, NO_WAKE, VOICE, Addressed, NotAddressed, decide
from rob_box_dialog.backlog import Heard
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session, SessionSnapshot
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import music_stop_claimed, music_stop_unwoken, same_stop_rest

WAKE = ("робот",)
OK = phrases.text("music.stop_ok")

#: Варианты фразы Tier-1 стопа музыки (решение Шифу 05.11: вариант выбирает код, без повтора подряд).
SAID = frozenset(phrases.catalog()[kn.MUSIC_STOP_SAID])
PLAYING_PHRASE = phrases.text(kn.MUSIC_PROMISE_PHRASES["playing"])
N712_STT = "робот заглушил"  # STT М11 n712, 02:22:46
N712_LLM = "Тишина. Борису больше не мешаем."  # ответ LLM М11 n712 без тула
N608_STT = "робот хватит диджей выключай"
N511_STT = "врубай все глуши музыку у меня уши в трубочку"  # STT М11 n511, вейк потерян


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


PLAYING = _snap(music=MusicStateRef("playing", track_id="req45347:01:A"))
IDLE = _snap()


class Ports:
    """Транспорт хода: стоп музыки гасит плеер (``was_playing``); LLM считает вызовы; что прозвучало."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.playing = True
        self.executed, self.said, self.asked = [], [], 0

    async def ask(self, messages, tools, timeout_s):
        self.asked += 1
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = dict(call.args)
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"], self.playing = self.playing, False
            self.executed.append((call.tool, dict(call.args)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, data))
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


def _run(ports, addressed, music="playing", dj=False):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef(music, track_id="req45347:01:A" if music == "playing" else None))
    if dj:
        session.push_scene("DJ", "user")
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    return asyncio.run(engine.run(addressed))


def _say(text):
    return Answer(Command(speech=text), "deepseek")


# --- n712 (б): «заглуши/заглушил/глуши» под музыкой - Tier-1 ------------------------------------------------------


@pytest.mark.parametrize("text", ["заглушил", "заглуши", "всё, заглушил", "заглушите", "заглуши музыку",
                                  "заглуши её"])
def test_mute_forms_under_music_are_music_off(text):
    assert match(text, PLAYING) == Tier1Command(kn.MUSIC_OFF_KIND), text


@pytest.mark.parametrize("text", ["заглушил", "заглуши", "заглуши музыку"])
def test_mute_forms_without_music_are_not_tier1(text):
    assert match(text, IDLE) is None, text


@pytest.mark.parametrize("text", ["заглуши мотор", "не заглушай музыку", "заглушил двигатель и уехал"])
def test_mute_with_a_foreign_object_or_negation_is_not_music_off(text):
    command = match(text, PLAYING)
    assert command is None or command.kind != kn.MUSIC_OFF_KIND, text


def test_n712_stt_stops_music_by_code_without_llm():
    ports = Ports([_say(N712_LLM)])
    agg = _run(ports, Addressed("заглушил", VOICE))
    assert ports.executed == [("stop_music", {})]
    assert ports.asked == 0, "стоп музыки решает код"
    assert len(ports.said) == 1 and ports.said[0] in SAID, ports.said
    assert (agg.stage, agg.status) == ("done", "finished")


# --- n712 (а): «Тишина» без стоп-тула под музыкой - нарушение music_promise ----------------------------------------


@pytest.mark.parametrize("text", [N712_LLM, "Тишина, Борису больше не мешаем.", "Музыку выключил.", "Заглушил.",
                                  "Трек остановлен, отдыхай."])
def test_stop_claims_are_recognised(text):
    assert music_stop_claimed(text), text


@pytest.mark.parametrize("text", ["Тишина?", "Если хочешь, будет тишина.", "Музыку не выключил.", "Играет рок.",
                                  "Могу выключить музыку.", "Скажи - и станет тихо."])
def test_questions_offers_and_negations_are_not_stop_claims(text):
    assert not music_stop_claimed(text), text


def test_n712_false_silence_under_music_is_retried_and_then_the_code_says_the_truth():
    """Лог n712: модель дважды говорит «Тишина» без стоп-тула - ложь не звучит, звучит фраза кода по состоянию."""
    ports = Ports([_say(N712_LLM), _say(N712_LLM), _say(N712_LLM)])
    agg = _run(ports, Addressed("что-то там", VOICE))
    assert N712_LLM not in ports.said, ports.said
    assert ports.said == [PLAYING_PHRASE], ports.said
    assert ports.executed == [], "музыку код не трогает: её никто не просил выключать словами Tier-1"
    assert ports.asked == 1 + kn.COMMAND_RETRIES_MAX
    assert agg.stage == "done"


def test_retry_with_a_stop_tool_is_kept():
    """После нарушения модель вызывает стоп - исполняется, о стопе говорит код."""
    ports = Ports([_say(N712_LLM), Answer(Command(actions=(ToolCall("stop_music", {}),)), "deepseek")])
    _run(ports, Addressed("что-то там", VOICE))
    assert ports.executed == [("stop_music", {})]
    assert ports.said == [OK]


@pytest.mark.parametrize("music", ["idle", "stopped"])
def test_silence_claim_without_music_is_the_truth(music):
    ports = Ports([_say(N712_LLM)])
    _run(ports, Addressed("что-то там", VOICE), music=music)
    assert ports.asked == 1 and ports.said == [N712_LLM]


# --- n608: остаток после Tier-1 стопа из того же стопа - ход закрыт кодом -----------------------------------------


@pytest.mark.parametrize("rest", ["диджей выключай", "выключай", "музыку стоп", "заглуши её"])
def test_same_stop_rest(rest):
    assert same_stop_rest(rest), rest


@pytest.mark.parametrize("rest", ["ещё песню", "не выключай", "спой другую", "давай", "выключи свет", ""])
def test_new_request_rest_is_not_the_same_stop(rest):
    assert not same_stop_rest(rest), rest


@pytest.mark.parametrize("text", [N608_STT, "Робот, хватит диджеить, выключай.", "робот стоп, музыку стоп"])
def test_n608_rest_of_the_same_stop_does_not_reach_the_llm(text):
    ports = Ports([Answer(Command(actions=(ToolCall("get_music_state", {}),)), "deepseek")])
    agg = _run(ports, Addressed(text, VOICE), dj=True)
    assert ports.asked == 0, "остаток - тот же стоп, модели нечего решать"
    assert ports.executed == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    assert len(ports.said) == 1 and ports.said[0] in SAID, ports.said
    assert (agg.stage, agg.status) == ("done", "finished")


def test_a_new_request_after_the_stop_still_goes_to_the_model():
    ports = Ports([_say("Сейчас спою.")])
    _run(ports, Addressed("робот хватит, спой другую", VOICE), dj=True)
    assert ports.asked == 1


# --- n511: стоп музыки без вейка, пока музыка играет --------------------------------------------------------------


def _decide(text, music_playing):
    return decide(Heard(text=text, t=100.0, source="respeaker"), None, WAKE, music_playing=music_playing)


def test_n511_unwoken_stop_under_music_is_a_turn_of_only_that_stop():
    decision = _decide(N511_STT, True)
    assert decision == Addressed("глуши музыку", MUSIC_STOP_UNWOKEN)


@pytest.mark.parametrize("text", ["выключи музыку", "выключай музыку пожалуйста", "стоп музыка", "музыку выключи"])
def test_exact_music_stop_forms_without_wake_are_addressed_under_music(text):
    decision = _decide(text, True)
    assert isinstance(decision, Addressed) and decision.source == MUSIC_STOP_UNWOKEN, text


@pytest.mark.parametrize("text", ["выключи свет", "стоп, машина", "не выключай музыку", "музыка классная",
                                  "глуши мотор", "расскажи сказку"])
def test_other_unwoken_phrases_under_music_stay_unaddressed(text):
    assert _decide(text, True) == NotAddressed(NO_WAKE), text


@pytest.mark.parametrize("text", [N511_STT, "выключи музыку", "стоп музыка"])
def test_without_music_wake_word_only_still_holds(text):
    assert _decide(text, False) == NotAddressed(NO_WAKE), text


def test_unwoken_stop_text_parses_as_a_music_stop():
    assert music_stop_unwoken(N511_STT) == "глуши музыку"
    assert music_stop_unwoken("музыку стоп") == "стоп музыку"


def test_n511_turn_stops_music_by_code_and_nothing_else_reaches_the_llm():
    ports = Ports([_say("Уши в трубочку - бывает.")])
    agg = _run(ports, _decide(N511_STT, True))
    assert ports.executed == [("stop_music", {})]
    assert ports.asked == 0, "остаток фразы без вейка модели не уходит"
    assert len(ports.said) == 1 and ports.said[0] in SAID, "«глуши музыку» без вейка - стоп кодом и фраза кода (Шифу 05.11)"
    assert (agg.stage, agg.status) == ("done", "finished")
