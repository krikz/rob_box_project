"""#4184 (марафон-3 28.10, n210): модель назвала неизвестного Гришу «Саша» - имя из истории разговора (ADR-0148).

``name_state=unknown``, а ответ «Понял, Саша.»: Саша и Борис говорили до Гриши, их имена модель видела в окне. Решает
код: обращение к неподтверждённому собеседнику по имени, которого он сам не называл, - нарушение
``VIOLATION_FOREIGN_NAME`` (повтор с подсказкой), после повтора имя вычищает код. Имя О третьем лице не трогается.
"""

import asyncio

import pytest

from rob_box_dialog import foreign_name as fn
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Ask, Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.turn import Answer, TurnEngine

SASHA = SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed")
BORIS = SpeakerRef(person_ref="spk-boris", name="Борис", name_state="confirmed")
NEW = SpeakerRef(person_ref="spk-new", name=None, name_state="unknown")
TENTATIVE = SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="tentative")
KEYS = fn.name_keys(["Саша", "Борис"])


# -- одно знание: что считается обращением -----------------------------------------------------------------------


@pytest.mark.parametrize("text,called", [
    ("Понял, Саша.", ("Саша",)),
    ("Привет, Борис, как дела?", ("Борис",)),
    ("Саша, понял.", ("Саша",)),
    ("Привет Борис!", ("Борис",)),
    ("Нет, Саша?", ("Саша",)),
    ("Ладно, Сашей меня не зови. Понял, Борис.", ("Борис",)),
    ("Саша просил передать привет.", ()),
    ("Это сказал Борис.", ()),
    ("Борис - молодец.", ()),
    ("Я передам Саше, что ты звонил.", ()),
    ("Понял, что Саша ушёл.", ()),
    ("Никаких обид.", ()),
])
def test_only_the_address_is_an_address(text, called):
    assert fn.addressed(text, KEYS) == called


@pytest.mark.parametrize("text,clean", [
    ("Понял, Саша.", "Понял."),
    ("Привет, Борис, как дела?", "Привет, как дела?"),
    ("Саша, понял.", "Понял."),
    ("Привет Борис!", "Привет!"),
    ("Хорошо. Борис, держи.", "Хорошо. Держи."),
    ("Саша, Борис, привет!", "Привет!"),
    ("Саша просил передать привет.", "Саша просил передать привет."),
    ("Саша.", "Саша."),
    ("Саша!", ""),
])
def test_the_stripped_phrase_stays_grammatical(text, clean):
    assert fn.without_address(text, KEYS) == clean


def test_names_compare_by_word_and_case_not_by_substring():
    keys = fn.name_keys(["Борис", "Иван Петров", kn.UNKNOWN_SPEAKER_LABEL, ""])
    assert keys == {"борис", "иван", "петров"}, "метка без имени и пустая - не имена; составная - по словам"
    assert fn.addressed("Понял, Бориса.", keys) == ("Бориса",), "падеж: один и тот же человек"
    assert fn.addressed("Борисовна, понял.", keys) == (), "подстрока - не имя"


def test_only_the_unconfirmed_speaker_is_protected():
    assert fn.unconfirmed(NEW) and fn.unconfirmed(TENTATIVE)
    assert fn.unconfirmed(SpeakerRef(name_state="contested"))
    assert not fn.unconfirmed(SASHA)
    assert not fn.unconfirmed(None), "снимка собеседника нет - код ничего не утверждает"


def test_unnamed_cleans_speech_and_ask_and_nothing_else():
    cmd = Command(speech="Понял, Саша.", ask=Ask("free", "Привет, Борис, чего хочешь?"),
                  actions=(ToolCall("play_animation", {"animation": "happy"}),))
    clean = fn.unnamed(cmd, KEYS)
    assert (clean.speech, clean.ask.text) == ("Понял.", "Привет, чего хочешь?")
    assert clean.actions == cmd.actions and fn.offending(clean, KEYS) == ()
    assert fn.offending(cmd, KEYS) == ("Саша", "Борис")
    assert fn.unnamed(Command(speech="Саша!"), KEYS).speech is None, "от речи ничего не осталось"


# -- ход целиком: что услышит человек и что получит модель --------------------------------------------------------


class Ports:
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


def _talk(answers, speaker, text="робот я не нашла и на свою тебя называть не буду"):
    """Саша и Борис говорили до собеседника хода (имена в окне модели), затем говорит ``speaker``."""
    session = Session("personality")
    ports = Ports([Answer(Command(speech="Привет."), "deepseek")] * 2 + list(answers))
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    for who, phrase in ((SASHA, "робот привет"), (BORIS, "робот привет")):
        session.set_speaker(who)
        asyncio.run(engine.run(Addressed(phrase, VOICE)))
    ports.said.clear()
    session.set_speaker(speaker)
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports


def test_unknown_gets_a_retry_with_the_violation_and_the_fixed_answer_is_spoken():
    """n210: «Понял, Саша.» неизвестному - нарушение; второй ответ модели без имени звучит как есть."""
    ports = _talk([Answer(Command(speech="Понял, Саша. Никаких обид."), "deepseek"),
                   Answer(Command(speech="Понял. Никаких обид."), "deepseek")], NEW)
    assert len(ports.asked) == 4 and kn.VIOLATION_FOREIGN_NAME in ports.asked[3][-1]["content"]
    assert ports.said == ["Понял. Никаких обид."]


def test_after_the_retry_the_code_strips_the_name_and_the_phrase_stays_grammatical():
    wrong = Answer(Command(speech="Понял, Саша. Никаких обид - железке не доверять разумно."), "deepseek")
    ports = _talk([wrong, wrong], NEW)
    assert len(ports.asked) == 4, "повтор один (COMMAND_RETRIES_MAX)"
    assert ports.said == ["Понял. Никаких обид - железке не доверять разумно."]


def test_tentative_voice_is_protected_the_same_way():
    wrong = Answer(Command(speech="Привет, Борис, слушаю."), "deepseek")
    ports = _talk([wrong, wrong], TENTATIVE)
    assert ports.said == ["Привет, слушаю."]


def test_confirmed_sasha_is_called_sasha():
    ports = _talk([Answer(Command(speech="Понял, Саша."), "deepseek")], SASHA)
    assert len(ports.asked) == 3 and ports.said == ["Понял, Саша."]


def test_third_person_name_is_not_touched_for_unknown():
    ports = _talk([Answer(Command(speech="Саша просил передать, что скоро будет."), "deepseek")], NEW)
    assert len(ports.asked) == 3 and ports.said == ["Саша просил передать, что скоро будет."]


def test_unknown_who_gave_his_own_name_is_called_by_it():
    """«Меня зовут Гриша» - «Понял, Гриша.» честно: имя назвал он сам. Борис из окна - по-прежнему чужой."""
    ports = _talk([Answer(Command(speech="Понял, Гриша."), "deepseek")], NEW, text="меня зовут Гриша")
    assert len(ports.asked) == 3 and ports.said == ["Понял, Гриша."]
    ports = _talk([Answer(Command(speech="Понял, Борис."), "deepseek")] * 2, NEW, text="меня зовут Гриша")
    assert ports.said == ["Понял."]


def test_unknown_with_a_name_in_the_registration_may_be_called_by_it():
    reg = Command(speech="Приятно познакомиться, Гриша.",
                  actions=(ToolCall("register_speaker", {"name": "Гриша"}),))
    ports = _talk([Answer(reg, "deepseek")], NEW, text="запомни меня")
    assert len(ports.asked) == 3 and "Гриша" in ports.said[-1]
