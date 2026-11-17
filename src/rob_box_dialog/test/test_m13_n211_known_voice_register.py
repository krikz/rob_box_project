"""М13 n211 (05.11, run 42034287725, 13:24:29 UTC; Refs #4236): «Робот, перечисли всех, кого ты сегодня запомнил по
голосу» голосом Саши (биометрия 'Саша' 0.959, ``confirmed``). Модель ответила одним ``register_speaker(name="Саша")``
без слов; владелец голосов склеил фразу с профилем Саши, код сказал «Запомнил тебя, Саша.» - вопрос без ответа, Бориса
никто не назвал (акцепт n211: ``expected keywords missing: ['Борис']``).

Решает код (ADR-0148, п.2 - валидатор с понятной ошибкой): имя собеседника подтверждено, модель регистрирует ЭТО ЖЕ
имя, а себя человек в реплике не называл - записывать нечего. Нарушение ``known_voice_register``: модели повтор с
подсказкой; повтора нет - такая регистрация не исполняется, говорит код.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import known_voice_registers
from rob_box_dialog.turn import TurnEngine, _Turn, owed_reason

from test_sound_answer_after_query_n109 import Ports, _answer

N211 = "робот перечисли всех кого ты сегодня запомнил по голосу"  # STT М13 n211, 13:24:29
SASHA = SpeakerRef(person_ref="f47c189a", name="Саша", name_state="confirmed")
REGISTER_SASHA = ToolCall("register_speaker", {"name": "Саша"})
#: ответ после повтора - так модель отвечала на n211 в М12 (05.11 04:51, шаг зелёный): по истории разговора
N211_ANSWER = "Сегодня запомнил по голосу двоих: тебя, Саша, и Бориса."


def _session(speaker=SASHA):
    session = Session("personality")
    session.set_speaker(speaker)
    return session


def _run(ports, text=N211, speaker=SASHA):
    engine = TurnEngine(_session(speaker), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _registers(ports):
    return [call for batch in ports.executed for call in batch if call[0] == "register_speaker"]


@pytest.mark.parametrize("text, speaker, calls, needless", [
    # живой n211: подтверждённый голос, своё же имя, не представлялся
    (N211, SASHA, (REGISTER_SASHA,), True),
    ("робот запомни мой голос", SASHA, (ToolCall("register_speaker", {"name": "саша"}),), True),
    # представился в реплике (акт 2b n701) - законно: владелец голосов ответит «уже знаю»
    ("робот здравствуй я саша чиню технику сегодня перебираю колеса", SASHA, (REGISTER_SASHA,), False),
    ("робот меня зовут саша", SASHA, (REGISTER_SASHA,), False),
    # другое имя - не этот валидатор (гейт регистрации, not_self_introduction / представление)
    ("робот добрый вечер меня зовут борис", SASHA, (ToolCall("register_speaker", {"name": "Борис"}),), False),
    # имя не подтверждено - регистрация может быть единственным способом записать голос
    (N211, SpeakerRef(person_ref="f47c189a", name="Саша", name_state="contested"), (REGISTER_SASHA,), False),
    (N211, None, (REGISTER_SASHA,), False),
    # без регистрации нарушения нет
    (N211, SASHA, (ToolCall("memory_context", {}),), False),
])
def test_known_voice_registers(text, speaker, calls, needless):
    assert bool(known_voice_registers(speaker, text, calls)) is needless


def test_n211_register_of_known_voice_is_a_violation():
    snap = _session().snapshot
    turn = _Turn("t1", 0, N211, VOICE, deadline=None)
    assert owed_reason(snap, turn, Command(actions=(REGISTER_SASHA,))) == kn.VIOLATION_KNOWN_VOICE_REGISTER
    assert owed_reason(snap, turn, Command(speech=N211_ANSWER)) == ""


def test_n211_model_is_returned_the_violation_and_answers_in_words():
    """Живой n211: вместо ответа - регистрация своего же голоса. Она не исполняется, модели уходит подсказка, ответ -
    словами по истории разговора (как в М12)."""
    ports = Ports([_answer(actions=(REGISTER_SASHA,)), _answer(speech=N211_ANSWER)])
    _run(ports)
    assert ports.said == [N211_ANSWER]
    assert _registers(ports) == []
    hint = ports.asked[1][-1]["content"]
    assert kn.VIOLATION_KNOWN_VOICE_REGISTER in hint and kn.KNOWN_VOICE_REGISTER_HINT in hint
    assert phrases.text("identity.done", name="Саша") not in ports.said


def test_n211_model_insists_registration_is_dropped_and_code_speaks():
    """Модель и после повторов регистрирует свой же голос: регистрация не исполняется («Запомнил тебя» - неправда),
    говорит код, ход не молчит."""
    ports = Ports([_answer(actions=(REGISTER_SASHA,))] * (kn.COMMAND_RETRIES_MAX + 1))
    _run(ports)
    assert _registers(ports) == []
    assert ports.said == [phrases.text(kn.INVALID_COMMAND_PHRASE)]


def test_self_introduction_of_known_voice_still_registers():
    """Акт 2b n701: «я Саша, чиню технику» знакомым голосом - регистрация исполняется как раньше."""
    text = "робот здравствуй я саша чиню технику сегодня перебираю колеса"
    ports = Ports([_answer(actions=(REGISTER_SASHA,), speech="Привет, Саша!")])
    _run(ports, text=text)
    assert _registers(ports) == [("register_speaker", {"name": "Саша"})]
    assert len(ports.asked) == 1
