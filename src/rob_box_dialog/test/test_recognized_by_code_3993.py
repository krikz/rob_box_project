"""#3993 (б): на «ты меня узнал?» отвечает КОД по собеседнику снимка, а не модель (решение Шифу 26.10, ADR-0148).

Марафон 23.10, акт 2b: голос узнан (0.944), имя скрыто спором - модель сказала «Голос я пока не запомнил, но имя
твоё знаю — Саша»: неправда и о голосе, и об имени. Вопрос «ты Саша?» в v2 не задаём (решение Шифу 26.10). Теперь
реплика целиком «ты меня узнал?» закрывается фразой кода по ``name_state`` снимка: узнан с именем / знакомый, не уверен
чей (без имени кандидата) / профиль без имени / не узнаю. LLM не зовётся.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import asks_recognized, recognition_reply, recognition_state, speaker_ref
from rob_box_dialog.turn import Answer, TurnEngine

#: Акт 2b, 21:30:14: «Саша» 0.944, конкурент «Борис» - имя скрыто спором.
CONTESTED = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.944, "epithet": "Спутник",
             "tentative_conf": 0.944, "tentative_kind": "contested"}
TENTATIVE = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.78,
             "tentative_name": "Саша", "tentative_kind": "single"}
CONFIRMED = {"is_known": True, "speaker_id": "spk-sasha", "name": "Саша", "confidence": 0.944}
NAMELESS = {"is_known": True, "speaker_id": "spk-guest-3", "name": None, "epithet": "Гость"}
UNKNOWN = {"is_known": False}

YES_SASHA = "Да, Саша, узнал."
UNSURE = "Голос знакомый, но не уверен, чей."


def _ref(result):
    return speaker_ref(result, now=1.0)


# -- вопрос целиком ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text", ["ты меня узнал?", "Ты меня узнаёшь?", "узнал меня?", "ну что, узнаёшь меня?",
                                  "ты меня не узнал?", "ты узнал мой голос?", "узнал, кто я?",
                                  "ты меня по голосу узнал?", "а ты меня узнаешь ли?"])
def test_whole_recognition_question(text):
    assert asks_recognized(text)


@pytest.mark.parametrize("text", ["ты меня узнал? включи музыку", "узнай погоду", "что ты обо мне помнишь?",
                                  "ты узнал, что Саша любит чай?", "как меня зовут?", "узнал", "ты меня слышишь?",
                                  "узнал узнал меня", ""])
def test_not_the_recognition_question(text):
    assert not asks_recognized(text)


# -- ответ по собеседнику снимка -------------------------------------------------------------------------------


def test_reply_by_name_state():
    assert recognition_reply("ты меня узнал?", _ref(CONFIRMED)) == ("confirmed", YES_SASHA)
    assert recognition_reply("ты меня узнал?", _ref(CONTESTED)) == ("contested", UNSURE)
    assert recognition_reply("ты меня узнал?", _ref(TENTATIVE)) == ("tentative", UNSURE)
    assert recognition_reply("ты меня узнал?", _ref(NAMELESS)) == ("nameless", "Голос твой помню, а имени не знаю.")
    assert recognition_reply("ты меня узнал?", _ref(UNKNOWN)) == ("unknown", "Нет, по голосу пока не узнаю.")
    assert recognition_reply("ты меня узнал?", None)[0] == "unknown"
    assert recognition_reply("включи музыку", _ref(CONFIRMED)) is None


@pytest.mark.parametrize("result", [CONTESTED, TENTATIVE])
def test_unconfirmed_voice_gets_no_candidate_name_and_no_identity_question(result):
    """Решение Шифу 26.10: «ты Саша?» не спрашиваем; имя кандидата вслух - утечка (n210 must_not_say)."""
    _state, said = recognition_reply("ты меня узнал?", _ref(result))
    assert "Саша" not in said and "?" not in said
    assert "не запомнил" not in said, "голос узнан - «не запомнил» неправда (акт 2b 23.10)"


def test_confirmed_without_name_is_not_a_hollow_yes():
    """Битый снимок «confirmed» без имени не даёт «Да, , узнал.»."""
    assert recognition_state(SpeakerRef(person_ref="spk-x", name=None, name_state="confirmed")) == "nameless"
    assert recognition_state(SpeakerRef(person_ref=None, name=None, name_state="confirmed")) == "unknown"


def test_every_name_state_has_a_reply_phrase():
    for state in (*kn.VOICE_UNCONFIRMED_STATES, "confirmed", "nameless", "unknown"):
        assert phrases.text(kn.RECOGNIZED_REPLIES[state], name="Саша"), state
    assert set(kn.RECOGNIZED_REPLIES.values()) <= set(kn.PHRASE_KEYS)


# -- ход целиком -------------------------------------------------------------------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди (пусто - вызывать модель нельзя)."""

    def __init__(self, answers=()):
        self.answers, self.asked, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

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


def _engine(result, answers=(), agent="personality"):
    session = Session(agent)
    if result is not None:
        session.set_speaker(_ref(result))
    ports = Ports(answers)
    return TurnEngine(session, ports, system_prompt="Ты робот."), ports, session


@pytest.mark.parametrize("result, said", [(CONTESTED, UNSURE), (CONFIRMED, YES_SASHA)])
def test_recognition_question_is_answered_by_code_without_llm(result, said):
    engine, ports, _ = _engine(result)
    agg = asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    assert ports.asked == [], "модель не зовётся"
    assert ports.said == [said]
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "finished", 0)


def test_question_inside_another_request_goes_to_the_model():
    engine, ports, _ = _engine(CONTESTED, [Answer(Command(speech="Включаю."), "deepseek")])
    asyncio.run(engine.run(Addressed("ты меня узнал? включи музыку", VOICE)))
    assert len(ports.asked) == 1 and ports.said == ["Включаю."]


def test_silence_swallows_the_question_too():
    engine, ports, session = _engine(CONFIRMED)
    session.push_scene("Silence", "user")
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    assert ports.asked == [] and ports.said == []


def test_backlog_pickup_is_not_answered_by_the_callers_voice():
    """Подхват бэклога: вопрос мог задать не тот, кто сказал «робот» - отвечает модель по меткам фраз."""
    engine, ports, _ = _engine(CONFIRMED, [Answer(Command(speech="Кто спрашивал?"), "deepseek")])
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE, pickup=True)))
    assert len(ports.asked) == 1 and YES_SASHA not in ports.said


def test_telegram_chat_has_no_voice_and_asks_the_model():
    engine, ports, _ = _engine(None, [Answer(Command(speech="Я вижу только текст."), "deepseek")], agent="telegram")
    asyncio.run(engine.run(Addressed("ты меня узнал?", "telegram")))
    assert len(ports.asked) == 1


def test_code_reply_lands_in_the_window_as_robot_speech():
    """Ответ кода - реплика робота в окне (после ``SpeechEvent``), как любая речь хода."""
    engine, ports, _ = _engine(CONTESTED)
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    engine.speech_ended("sp1", len(UNSURE))
    assert [(e.role, e.content) for e in engine.log.window()][-1] == ("assistant", UNSURE)
