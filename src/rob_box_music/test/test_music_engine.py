"""#3960 — движок музыки, названный по имени («сыграй на Ренардо»): слот ``engine`` решает код, а не кэш.

Живой прогон 23.10, акт 5: STT дал «робот сыграй на индардо спокойный ночной бит что нибудь под пайку», путь выбрал
трек кэша «STAR WARS (RoBBox cover)» — не Ренардо. Имя движка и его искажения STT живут в ``knowledge.MUSIC_ENGINES``.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, DeterministicChooser
from rob_box_music.source import (CacheTrack, SourceAsk, SourceContext, ask_engine, choose_route, decline_text,
                                  engine_route, route_note, set_candidates, source_ask, source_candidates,
                                  with_engine)

#: Кэш как на роботе: свежий инструментал, который без имени движка выиграл бы у синтеза.
CACHE = (CacheTrack("k", "STAR WARS (RoBBox cover)", "cover", "club", True, rating=0, group="g1"),)
RICH = SourceContext(cache=CACHE, gen_ready=True, balance=900.0, day_left=120.0)


class Boom:
    def choose(self, question, *, deadline_ms=0):
        return Choice("generate:song:V6", 1.0, "llm")


@pytest.mark.parametrize("text", [
    "робот сыграй на индардо спокойный ночной бит что нибудь под пайку",  # дословно из лога 23.10
    "Робот, сыграй на рендардо спокойный ночной бит, что-нибудь под пайку.",  # текст шага n502
    "сыграй на Ренардо",
    "поставь бит в ренардо",
    "давай на рейнардо что-нибудь",
    "сыграй на ре нардо",  # STT разрезал имя
    "play on Renardo",
])
def test_engine_named_in_speech_and_its_stt_garbles(text):
    ask = source_ask(text)
    assert ask.engine == "renardo" and engine_route(ask.engine) == "synth"
    assert "engine=renardo" in ask.describe()


@pytest.mark.parametrize("text", [
    "включи музыку", "поставь Леонардо да Винчи", "сгенерируй рок", "сыграй на ренессансе", "",
])
def test_no_engine_when_not_named(text):
    assert ask_engine(text) == "" and source_ask(text).engine == "" and "engine" not in source_ask(text).describe()


def test_named_engine_beats_cache_that_would_win_otherwise():
    """Без имени движка свежий трек кэша выигрывает; с именем — только синтез (Ренардо), кэш и генерация не кандидаты."""
    plain = choose_route(source_ask("поставь ночной бит"), RICH, DeterministicChooser(Boom()))
    named = choose_route(source_ask("сыграй на индардо спокойный ночной бит"), RICH, DeterministicChooser(Boom()))
    assert plain.kind == "cache"
    assert (named.kind, named.id) == ("synth", "synth:club")
    options, reason = source_candidates(source_ask("сыграй на индардо ночной бит"), RICH)
    assert [o.id for o in options] == ["synth:club"] and reason == ""
    assert route_note(named) == ""  # просили именно это: оправданий нет


def test_named_engine_plays_even_style_that_synth_is_poor_at():
    """Рок синтез «не играет» (``synth_fit`` 0) — но человек назвал Ренардо: играет движок со стилем по умолчанию."""
    route = choose_route(source_ask("сыграй рок на ренардо"), RICH, DeterministicChooser(Boom()))
    assert route.id == f"synth:{kn.DEFAULT_STYLE}"


def test_named_engine_cannot_sing_declines_honestly():
    ask = source_ask("спой на ренардо")
    route = choose_route(ask, RICH, DeterministicChooser(Boom()))
    assert route.id == "decline:engine_cannot" and route.kind == "decline"
    assert "названный движок так не играет" in decline_text(route)


def test_set_of_named_engine_is_synth_only():
    options, _reason = set_candidates(source_ask("сет на ренардо"), RICH)
    assert [o.id for o in options] == ["synth:club"]


def test_slot_value_from_schema_extends_words():
    """Слот ``engine`` схемы (его заполнила LLM, поняв искажённое имя) - ключ таблицы; сверка с таблицей, не со словами."""
    assert list(kn.MUSIC_ENGINES) == ["renardo"] and kn.ENGINE_AUTO == "auto"
    assert SourceAsk(engine="renardo").describe().endswith("engine=renardo")
    assert engine_route("auto") == "" and engine_route("") == "" and engine_route("renardo") == "synth"


def test_with_engine_fills_slot_only_when_words_missed_it():
    """STT исказил имя дальше основ («на виндарда»): слот схемы дополняет; ``auto``/чужое/``None`` ничего не меняют."""
    garbled = source_ask("поставь ровный ритм на виндарда")
    assert garbled.engine == ""
    assert with_engine(garbled, "renardo").engine == "renardo"
    for slot in ("auto", "leonardo", "", None):
        assert with_engine(garbled, slot) == garbled
    named = source_ask("сыграй на ренардо")
    assert with_engine(named, "auto") is named
