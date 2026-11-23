"""#5104: вопрос о каталоге звуков, а ответ — одно лицо. Каталог перевешивает общий ``silent_answer``.

Живой ход ``b558da1ac9a0`` (реплика синтетическая): «а какие звуки ты умеешь играть» → речь без тула → повтор за
каталог; второй ответ — только ``expression:thinking``. ``owed_reason`` брал ``VIOLATION_SILENT_ANSWER`` раньше
``catalog_owed``, а у молчания в ``_OWED`` нет ``reads``: сальваж исполнял лицо и говорил «Ага, я на связи.», каталог
не читал. Решает код (ADR-0148): вопрос о каталоге — нарушение каталога, и после повтора код сам читает
``get_sound_info`` и говорит по его данным (``query_fallback``). Обычная адресованная реплика без каталожного вопроса
по-прежнему ``silent_answer``.
"""

import asyncio

from test_sound_answer_after_query_n109 import SOUND_DATA, SOUNDS_PHRASE, Ports, _answer
from test_sound_catalog_without_data_n109 import FROM_MEMORY, N109, _turn

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression
from rob_box_dialog.session import Session
from rob_box_dialog.turn import TurnEngine, owed_reason

THINKING = Expression(emotion="thinking")
ASKED = "а какие звуки ты умеешь играть"
PLAIN = "ты меня слышишь"


def _run(ports, text):
    return asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _reads(ports):
    return [call for batch in ports.executed for call in batch if call[0] == "get_sound_info"]


def test_catalog_question_with_only_an_expression_reads_the_catalog_after_the_retry():
    """Сценарий лога: речь без тула → повтор за каталог; только ``expression`` → код читает ``get_sound_info`` и
    говорит по данным, не «Ага, я на связи.»."""
    ports = Ports([_answer(speech=FROM_MEMORY), _answer(expression=THINKING)], {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports, ASKED)
    assert len(ports.asked) == kn.COMMAND_RETRIES_MAX + 1
    hint = ports.asked[1][-1]["content"]
    assert kn.VIOLATION_SOUND_CATALOG in hint and kn.SOUND_CATALOG_HINT in hint
    assert kn.VIOLATION_SILENT_ANSWER not in hint
    assert _reads(ports) == [("get_sound_info", {})]
    assert ports.said == [SOUNDS_PHRASE]
    assert not set(ports.said) & set(phrases.catalog()[kn.SILENT_ANSWER_PHRASE])


def test_plain_addressed_reply_with_only_an_expression_stays_silent_answer():
    """Контроль: адресованная реплика без каталожного вопроса + голая ``expression`` — по-прежнему ``silent_answer``."""
    snap = Session("personality").snapshot
    assert owed_reason(snap, _turn(text=PLAIN), Command(expression=THINKING)) == kn.VIOLATION_SILENT_ANSWER
    ports = Ports([_answer(expression=THINKING), _answer(speech="Слышу тебя.")])
    _run(ports, PLAIN)
    assert len(ports.asked) == 2 and kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert kn.VIOLATION_SOUND_CATALOG not in ports.asked[1][-1]["content"]
    assert _reads(ports) == [] and ports.said == ["Слышу тебя."]


def test_owed_reason_names_the_catalog_before_the_silent_answer():
    """Вопрос о каталоге + голая ``expression`` — нарушение каталога, не ``silent_answer`` (порядок #5104)."""
    snap = Session("personality").snapshot
    assert owed_reason(snap, _turn(text=ASKED), Command(expression=THINKING)) == kn.VIOLATION_SOUND_CATALOG
    assert owed_reason(snap, _turn(text=N109), Command(expression=THINKING)) == kn.VIOLATION_SOUND_CATALOG
    assert owed_reason(snap, _turn(text=PLAIN), Command(expression=THINKING)) == kn.VIOLATION_SILENT_ANSWER
