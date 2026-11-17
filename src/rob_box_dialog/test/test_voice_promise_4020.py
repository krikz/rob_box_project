"""Речь обещает сменить голос, темп или тон, а тула нет (#4020 К1, ADR-0148): слова без действия - ложь о своей речи.

Тот же класс, что ``music_promise`` (test_music_promise_4020.py): знание - одна таблица ``knowledge.VOICE_PROMISES``,
валидатор команды судит по словам, о провале говорит код (``command.voice_promise``).
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Invalid, ToolCall, explain, parse, unkept_voice_promises, validate
from rob_box_dialog.words import voice_promises

TOOLS = ("set_speed", "set_pitch", "set_voice", "set_tts_provider", "set_volume", "play_animation")


@pytest.mark.parametrize("text, kinds", [
    ("Хорошо, заговорю помедленнее.", ("speed",)),
    ("Буду говорить быстрее, как просил.", ("speed",)),
    ("Замедлюсь, чтобы ты успел записать.", ("speed",)),
    ("Ускоряюсь!", ("speed",)),
    ("Понижу тон, будет басом.", ("pitch",)),
    ("Дальше буду говорить басом.", ("pitch",)),
    ("Переключусь на голос Алёны.", ("voice",)),
    ("Меняю голос на женский.", ("voice",)),
    ("Включаю голос Джейн.", ("voice",)),
])
def test_promises_are_found_by_words(text, kinds):
    assert set(voice_promises(text)) == set(kinds)


@pytest.mark.parametrize("text", [
    "Говорю медленнее.",                                   # настоящее - правда после Tier-1 шага голоса
    "Сейчас я говорю через MiniMax, голос Qingse (legacy).",
    "Скажу голосом Алёны: привет!",                        # речь голосом, а не смена голоса
    "Включаю музыку.",
    "Голос не менял, прежний.",
    "Я быстро соображаю.",
    "",
])
def test_not_a_promise(text):
    assert voice_promises(text) == ()


def test_table_is_one_knowledge_and_tools_exist():
    assert set(kn.VOICE_PROMISES) == {"speed", "pitch", "voice"}
    assert kn.VOICE_PROMISES["speed"].tools == ("set_speed",)
    assert kn.VOICE_PROMISES["pitch"].tools == ("set_pitch",)
    assert set(kn.VOICE_PROMISES["voice"].tools) == {"set_voice", "set_tts_provider"}


def test_promise_without_tool_is_invalid_with_hint():
    cmd = parse({"speech": "Хорошо, заговорю помедленнее."})
    assert unkept_voice_promises(cmd) == ("speed",)
    result = validate(cmd, TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_VOICE_PROMISE
    assert kn.VOICE_PROMISE_HINT in explain(result)


def test_promise_with_the_tool_is_fine():
    cmd = Command(speech="Заговорю помедленнее.", actions=(ToolCall("set_speed", {"action": "slower"}),))
    assert unkept_voice_promises(cmd) == ()
    result = validate(cmd, TOOLS)
    assert not (isinstance(result, Invalid) and kn.VIOLATION_VOICE_PROMISE in {v.reason for v in result.all})


def test_wrong_tool_does_not_keep_the_promise():
    cmd = Command(speech="Переключусь на голос Алёны.", actions=(ToolCall("set_speed", {"action": "slower"}),))
    assert unkept_voice_promises(cmd) == ("voice",)


def test_code_phrase_exists_in_catalog():
    assert kn.VOICE_PROMISE_PHRASE in kn.PHRASE_KEYS
    assert phrases.text(kn.VOICE_PROMISE_PHRASE)
