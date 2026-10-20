"""ADR-0157 PR-1 (#3558): слот инструмента и правило режима «пьеса без слова сет» — решает грамматика, не LLM.

Прогон 09.10 (§1.1): из 14 просьб «сыграй …» ни одна не выполнена; инструмент учтён 0 из 10, а LLM на хвосте
грамматики трижды увела «сыграй … на <инструменте>» в диджей-сет. Здесь проверяется поведение модулей:

* R1 — режим по фразам §1.1: без слова «сет/сэт/диджей/вечеринк» и без стиля — пьеса, со словом — сет;
* R2 — инструмент из таблицы попадает в слот, а не в название для поиска (промаха «слово вне архива» нет);
* фраза об инструменте — от кода по результату ``request_music`` (честная деградация «нет пака»).
"""

from __future__ import annotations

import asyncio
from typing import Any, Dict, List, Tuple

import pytest

from rob_box_voice.core.media_command_grammar import (
    MediaIntent,
    asks_for_set,
    instrument_in,
    parse_media_command,
)
from rob_box_voice.core.media_phrases import instrument_text, with_instrument
from rob_box_voice.core.music_turn import MusicTurn
from rob_box_voice.core.named_play import NamedPlayStatus, outcome_text, run_named_play


#: §1.1: реплика товарища Шифу → (режим, инструмент). #3 — «какой нахуй сэт» — отказ от сета, не просьба.
RUN_0910 = [
    ("Сыграй на балалайке", "piece", "balalaika"),
    ("сфграй баха на балалайке", "piece", "balalaika"),
    ("не еби голову сыграй Баха на балалайке какой нахуй сэт", "piece", "balalaika"),
    ("ну что баля будет Бах на балалайке?", "piece", "balalaika"),
    ("сыграй Баха на Балалайке", "piece", "balalaika"),
    ("свграй Баха на арфе", "piece", "harp"),
    ("сыграй баха на трамбоне", "piece", "trombone"),
    ("я попросил просто сыграть баха на инструменте трамбон!!!", "piece", "trombone"),
    ("Сыграй баха На трамбоне", "piece", "trombone"),
    ("у тебя что другого нет произведения баха? почему всегда одно и то же", "piece", ""),
    ("Сыграй Токкато на кселофоне", "piece", "xylophone"),
    ("Сыграй Токката ре минор на кселофоне", "piece", "xylophone"),
    ("Сыграй Токката ре минор", "piece", ""),
    ("Ставь диджей сэт Токката ре минор", "set", ""),
    ("Ставь синтвейв сэт Токката ре минор", "set", ""),
]

#: Набор живой проверки PR-1 (6 фраз) и соседние формы.
ROBOT_SIX = [
    ("Робот, сыграй Баха на балалайке", "piece", "balalaika"),
    ("Робот, сыграй Баха на трамбоне", "piece", "trombone"),
    ("Робот, сыграй Токкату на кселофоне", "piece", "xylophone"),
    ("Робот, сыграй что-нибудь на арфе", "piece", "harp"),
    ("Робот, включи диджей сет на тему Бах", "set", ""),
    ("Робот, сыграй на пианино", "piece", "piano"),
    ("включи рейв", "set", ""),
    ("ты диджей Снупдог", "set", ""),
    ("замути сэт на 30 минут", "set", ""),
    ("музыку для вечеринки", "set", ""),
    ("сыграй без сета, просто Баха", "piece", ""),
]


@pytest.mark.parametrize("text,mode,instrument", RUN_0910 + ROBOT_SIX)
def test_mode_and_instrument_slot(text, mode, instrument):
    """R1 + R2: режим по словам реплики, инструмент — ключ таблицы ``INSTRUMENTS``."""
    assert ("set" if asks_for_set(text) else "piece") == mode
    assert instrument_in(text) == instrument


@pytest.mark.parametrize("text,instrument", [
    ("Робот, сыграй джаз на ситаре", "sitar"), ("сыграй на ситаре", "sitar"), ("сыграй что-нибудь на гитаре", "guitar"),
    ("Робот, сыграй джаз на ситаре!!!", "sitar"), ("сыграй на ситоре", "sitar"), ("сыграй на банджо", ""),
])
def test_sitar_is_its_own_instrument_not_a_typo_of_guitar(text, instrument):
    """#3592: «ситаре» играл гитару (одна правка от «гитаре»); теперь ситар — строка таблицы, гитара остаётся гитарой."""
    assert instrument_in(text) == instrument


def test_sitar_phrase_is_built_by_code_from_the_result():
    """Фраза об инструменте — от кода по ``instrument`` результата: «Играю на ситаре.»"""
    assert instrument_text({"instrument": "sitar", "instrument_played": "sitar"}) == "Играю на ситаре."


@pytest.mark.parametrize("text,name,instrument", [
    ("сыграй Баха на Балалайке", "баха", "balalaika"),
    ("сфграй баха на балалайке", "баха", "balalaika"),
    ("свграй Баха на арфе", "баха", "harp"),
    ("Сыграй баха На трамбоне", "баха", "trombone"),
    ("Сыграй Токкато на кселофоне", "токкато", "xylophone"),
    ("Сыграй Токката ре минор на кселофоне", "токката ре минор", "xylophone"),
    ("Робот, сыграй Баха на балалайке", "баха", "balalaika"),
])
def test_named_order_keeps_the_instrument_out_of_the_title(text, name, instrument):
    """Название для поиска — без слова инструмента: ``lookup_melody`` больше не промахивается на «балалайке»."""
    command = parse_media_command(text)
    assert command.intent is MediaIntent.PLAY_NAMED and command.closed
    assert (command.name, command.instrument) == (name, instrument)


@pytest.mark.parametrize("text,instrument", [
    ("Сыграй на балалайке", "balalaika"),
    ("Робот, сыграй что-нибудь на арфе", "harp"),
    ("Робот, сыграй на пианино", "piano"),
])
def test_order_without_title_is_one_track_not_a_set(text, instrument):
    """«Сыграй на арфе» — одиночный заказ (``request_music``), не сет и не LLM."""
    command = parse_media_command(text)
    assert command.intent is MediaIntent.REQUEST_MUSIC and command.closed and command.instrument == instrument


def test_set_orders_are_not_changed():
    assert parse_media_command("Робот, включи диджей сет на тему Бах").intent is MediaIntent.DJ
    assert parse_media_command("включи рейв").intent is MediaIntent.DJ
    assert parse_media_command("поставь к элизе").name == "к элизе"
    assert parse_media_command("ты сыграл баха?").intent is MediaIntent.NONE  # «сыграл» — не опечатка «сыграй»
    assert parse_media_command("выключи музыку и включи Oakenfold").intent is MediaIntent.NONE  # не «включи»


def test_honest_degradation_phrase_is_built_from_the_table():
    """Решение Шифу В2: пака нет — «могу сыграть на <соседи из таблицы>» и играет первым соседом."""
    assert instrument_text({"instrument": "balalaika", "instrument_played": "guitar"}) == (
        "Для балалайки у меня нет пака, могу сыграть на гитаре или на арфе или на ситаре — играю на гитаре.")
    assert instrument_text({"instrument": "trombone", "instrument_played": "trombone"}) == "Играю на тромбоне."
    assert instrument_text({"instrument": "flute", "instrument_played": ""}) == (
        "Для флейты у меня нет пака — играю своим звуком.")
    assert instrument_text({}) == "" and with_instrument("Ставлю «Баха».", {}) == "Ставлю «Баха»."


def test_named_play_phrase_comes_from_the_request_music_result():
    """Роутер: ``lookup_melody`` по названию без инструмента → ``request_music`` → фраза из его ``data``."""
    calls: List[Tuple[str, Dict[str, Any]]] = []

    async def execute(name: str, args: Dict[str, Any]) -> Tuple[bool, str]:
        calls.append((name, args))
        if name == "lookup_melody":
            return True, repr({"name": "j_s_bach", "title": "Bach", "match": {"unmatched": [], "ignored": []}})
        return True, "ok\n" + repr({"title": "Bach", "instrument": "balalaika", "instrument_played": "guitar"})

    outcome = asyncio.run(run_named_play(execute, parse_media_command("сыграй Баха на балалайке").name))
    assert outcome.status is NamedPlayStatus.PLAYED and calls[0] == ("lookup_melody", {"name": "баха"})
    assert outcome_text(outcome, "Баха") == (
        "Ставлю «Баха». Для балалайки у меня нет пака, могу сыграть на гитаре или на арфе или на ситаре — играю на гитаре.")


def test_llm_turn_phrase_about_the_instrument_is_code():
    """Ход LLM: фраза об инструменте — из результата тула, а не текст модели («играю на тромбоне, сет идёт»)."""
    turn = MusicTurn()
    content = repr({"ok": True, "started": True, "title": "Bach", "instrument": "trombone",
                    "instrument_played": "trombone"})
    turn.record("request_music", {"intent": "melody", "text": "Бах"}, is_error=False, content=content)
    assert turn.phrase() == "Ставлю «Bach». Играю на тромбоне."
