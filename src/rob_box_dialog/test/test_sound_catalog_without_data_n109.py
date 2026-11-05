"""М11 n109 (#4236, засчитанный прогон 01:12:19): «какие звуки ты умеешь?» → модель ответила списком по памяти
(«Умею подтверждать… глючить… капать нотой… пикать») без ``get_sound_info``.

PR #4397 ловит звук без слов ПОСЛЕ запроса; здесь запроса не было вовсе. Решает код (ADR-0148): вопрос о каталоге
звуков (``tools_view.sound_catalog_asked`` - слова человека) без данных каталога в ходе и без ``get_sound_info`` в
команде - нарушение ``VIOLATION_SOUND_CATALOG`` (``turn.catalog_owed``): модели повтор с подсказкой, повтора нет -
каталог читает код сам и говорит по его данным (``query.sounds_summary``), слова модели по памяти не звучат.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Ask, Command, Performance, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import sound_catalog_asked
from rob_box_dialog.turn import TurnEngine, _Turn, catalog_owed, owed_reason

from test_sound_answer_after_query_n109 import HAPPY, SOUND_DATA, SOUNDS, SOUNDS_PHRASE, Ports, _answer, _played

N109 = "Робот, какие звуки ты умеешь?"
FROM_MEMORY = "Умею подтверждать, глючить, капать нотой и пикать."


def _run(ports, text=N109):
    return asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _reads(ports):
    return [call for batch in ports.executed for call in batch if call[0] == "get_sound_info"]


# -- ход ---------------------------------------------------------------------------------------------------------------


def test_n109_list_from_memory_is_returned_and_the_answer_comes_from_the_catalog():
    """Живой n109: список звуков словами без запроса. Слова по памяти не звучат - модели нарушение с подсказкой
    «положи get_sound_info»; она читает каталог и отвечает по данным."""
    ports = Ports([_answer(speech=FROM_MEMORY), _answer(actions=(SOUNDS,)),
                   _answer(speech="Звуков у меня полсотни: эмоции, интерфейс и эффекты.")],
                  {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports)
    assert ports.said == ["Звуков у меня полсотни: эмоции, интерфейс и эффекты."]
    assert FROM_MEMORY not in ports.said
    hint = ports.asked[1][-1]["content"]
    assert kn.VIOLATION_SOUND_CATALOG in hint and kn.SOUND_CATALOG_HINT in hint
    assert _reads(ports) == [("get_sound_info", {})]


def test_n109_model_insists_on_memory_the_code_reads_the_catalog_and_speaks():
    """Модель и после повторов отвечает по памяти: ход не говорит её список - код сам читает ``get_sound_info`` и
    называет категории по данным; звук вместо ответа не исполняется."""
    memory = _answer(speech=FROM_MEMORY, actions=(HAPPY,))
    ports = Ports([memory] * (kn.COMMAND_RETRIES_MAX + 1), {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports)
    assert ports.said == [SOUNDS_PHRASE]
    assert len(ports.asked) == kn.COMMAND_RETRIES_MAX + 1 and ports.answers == []
    assert _reads(ports) == [("get_sound_info", {})] and _played(ports) == []


def test_catalog_read_in_the_command_is_not_a_violation():
    """Модель сразу читает каталог - нарушения нет, ответ по данным."""
    ports = Ports([_answer(actions=(SOUNDS,)), _answer(speech="Звуков полсотни.")],
                  {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports)
    assert ports.said == ["Звуков полсотни."] and len(ports.asked) == 2
    assert all(kn.VIOLATION_SOUND_CATALOG not in str(m) for batch in ports.asked for m in batch)


def test_request_for_a_sound_stays_a_sound():
    """Просьба «включи какой-нибудь звук» - не вопрос о каталоге: звук исполняется, повтора нет."""
    ports = Ports([_answer(actions=(HAPPY,))])
    _run(ports, "Робот, включи какой-нибудь звук")
    assert _played(ports) == [("play_sound", {"sound": "robot_happy"})] and len(ports.asked) == 1


# -- правило -----------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text, expected", [
    ("Робот, какие звуки ты умеешь?", True),
    ("Робот, какие звуки ты вообще умеешь издавать?", True),
    ("Робот, сколько у тебя звуков?", True),
    ("перечисли свои звуки", True),
    ("Робот, включи какой-нибудь звук", False),
    ("Робот, какие-нибудь звуки издай", False),
    ("Робот, какие песни ты знаешь?", False),
    ("Робот, звук погромче", False),
])
def test_sound_catalog_asked(text, expected):
    assert sound_catalog_asked(text) is expected


def _turn(text=N109, queried=False):
    done = (ActionResult("t1/0", 0, "personality", "get_sound_info", "query", "done", None, SOUND_DATA),)
    return _Turn("t1", 0, text, VOICE, deadline=None, queried=done if queried else ())


@pytest.mark.parametrize("cmd, kw, expected", [
    (Command(speech=FROM_MEMORY), {}, True),
    (Command(speech=FROM_MEMORY, actions=(HAPPY,)), {}, True),
    (Command(actions=(SOUNDS,)), {}, False),                                    # читает каталог
    (Command(speech="Звуков полсотни."), {"queried": True}, False),             # каталог уже прочитан в ходе
    (Command(speech=FROM_MEMORY), {"text": "Робот, включи звук"}, False),       # просьба, не каталог
    (Command(ask=Ask(text="Какие именно: весёлые или служебные?")), {}, False),  # встречный вопрос
    (Command(passed="not_addressed"), {}, False),
    (Command(perform=Performance(kind="song", segments=("Пи-пи",))), {}, False),
])
def test_catalog_owed(cmd, kw, expected):
    assert catalog_owed(Session("personality").snapshot, _turn(**kw), cmd) is expected


def test_owed_reason_names_the_sound_catalog():
    snap = Session("personality").snapshot
    assert owed_reason(snap, _turn(), Command(speech=FROM_MEMORY)) == kn.VIOLATION_SOUND_CATALOG
    assert owed_reason(snap, _turn(queried=True), Command(speech="Звуков полсотни.")) == ""
    assert owed_reason(snap, _turn(), Command(actions=(ToolCall("get_sound_info"),))) == ""
