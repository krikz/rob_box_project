"""#3960 — движок музыки, названный по имени («сыграй на Ренардо»): слот ``engine`` решает код, а не кэш.

Живой прогон 23.10, акт 5: STT дал «робот сыграй на индардо спокойный ночной бит что нибудь под пайку», путь выбрал
трек кэша «STAR WARS (RoBBox cover)» — не Ренардо. Имя движка и его искажения STT живут в ``knowledge.MUSIC_ENGINES``.
"""

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, DeterministicChooser
from rob_box_music.source import (CacheTrack, SourceAsk, SourceContext, ask_engine, blank_engine, choose_route,
                                  decline_text, engine_route, engine_words, route_note, set_candidates, settle_engine,
                                  source_ask, source_candidates)

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
    # ядро названия (#4013): искажение дальше основ — живой марафон 23.10, #3960
    "Робот, поставь фоном ровный ритм наррендарда только не громко",  # n809
    "Робот, сыграй нарен дарда спокойный ночной бит",  # n502: имя разрезано внутри
    "поставь ровный ритм на виндарда",  # тест #3960
    "сыграй на индардо",
    "нарен дарда",
    "наррендарда",
])
def test_engine_named_in_speech_and_its_stt_garbles(text):
    ask = source_ask(text)
    assert ask.engine == "renardo" and engine_route(ask.engine) == "synth"
    assert "engine=renardo" in ask.describe()


@pytest.mark.parametrize("text", [
    "включи музыку", "поставь Леонардо да Винчи", "сгенерируй рок", "сыграй на ренессансе", "",
    # #4013: инструмент, жанр, пьеса — не движок
    "сыграй что-нибудь на балалайке", "сыграй рок", "сыграй Running Up That Hill на балалайке и гуслях",
    "Робот, сыграй Running Up That Hill на балалайке в рок стиле", "сгенерируй Running Up That Hill",
    # слова-двойники ядра: «нардо» в «Леонардо»/«Бернардо», «нардов» — нарды
    "включи Бернардо", "сыграй с Леонардо да Винчи", "я проиграл три партии нардов", "сыграй стандарт",
])
def test_no_engine_when_not_named(text):
    assert ask_engine(text) == "" and source_ask(text).engine == "" and "engine" not in source_ask(text).describe()


def test_engine_words_give_the_exact_spans_so_the_name_leaves_the_theme():
    """Слова-названия движка (в том числе пара, разрезанная STT) вырезаются из темы трека, соседей не задевают."""
    text = "Робот, поставь фоном ровный ритм на нарен дарда только не громко"
    assert [(key, text[start:end]) for key, start, end in engine_words(text)] == [("renardo", "нарен дарда")]
    assert blank_engine(text).split() == "Робот, поставь фоном ровный ритм на только не громко".split()
    assert blank_engine("сыграй на ре нардо джаз").split() == ["сыграй", "на", "джаз"]
    assert blank_engine("поставь Леонардо") == "поставь Леонардо" and blank_engine("") == ""


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


def test_slot_without_support_in_the_words_is_dropped_with_a_reason():
    """#4013: LLM ставит ``engine=renardo`` на «сыграй X на балалайке» — слов про Ренардо нет, слот отбрасывается с
    причиной в строке, и путь не сужается до синтеза (кэш и генерация остаются кандидатами)."""
    for text in ("сыграй что-нибудь на балалайке", "сыграй рок", "Робот, сыграй Running Up That Hill на балалайке"):
        ask, note = settle_engine(source_ask(text), "renardo")
        assert ask.engine == "" and "без опоры в словах" in note and "engine=renardo" in note
    ask = settle_engine(source_ask("сыграй что-нибудь на балалайке"), "renardo")[0]
    options, _reason = source_candidates(ask, RICH)
    assert {o.id.split(":")[0] for o in options} == {"synth", "generate", "cache"}  # путь не сужен
    narrowed, _reason = source_candidates(replace(ask, engine="renardo"), RICH)
    assert [o.id for o in narrowed] == ["synth:club"]  # так было до #4013: принятый слот оставлял один синтез


def test_slot_with_support_in_the_words_is_the_engine():
    """Слова называют движок (в том числе искажённо) — он и есть; слот ничего не меняет и лога не просит."""
    for text in ("поставь ровный ритм на виндарда", "нарен дарда", "наррендарда", "сыграй на индардо"):
        ask, note = settle_engine(source_ask(text), "renardo")
        assert ask.engine == "renardo" and note == ""
        assert settle_engine(source_ask(text), None)[0].engine == "renardo"


def test_unknown_or_auto_slot_changes_nothing():
    plain = source_ask("сыграй рок")
    for slot in ("auto", "leonardo", "", None):
        assert settle_engine(plain, slot) == (plain, "")
    named = source_ask("сыграй на ренардо")
    assert settle_engine(named, "auto")[0] is named


@pytest.mark.parametrize("text", [
    "Робот, и ускорь темп, а то мы тут засыпаем оба.",  # n506
    "Робот, продолжай развивать эту тему и добавь баса пожирнее.",  # n504
    "Робот, сделай атмосферу помрачнее, как в киберпанке.",  # n505
])
def test_playing_engine_is_inherited_by_a_plain_request(text):
    """n504–n506: «ускорь», «добавь баса» под Ренардо — движок тот же, его наследует код по снимку плеера."""
    ask, note = settle_engine(source_ask(text), None, playing="renardo")
    assert ask.engine == "renardo" and "наследуется от играющего: renardo" in note
    assert engine_route(ask.engine) == "synth"
    assert settle_engine(source_ask(text), "renardo", playing="renardo")[0].engine == "renardo"  # слот не мешает


@pytest.mark.parametrize("text,own_subject", [
    ("сгенерируй новый трек", False),  # просьба называет путь: «сгенерируй»
    ("спой что-нибудь", False),
    ("сыграй рок", False),  # жанр, которого синтез не играет
    ("поставь новый трек", False),
    ("сыграй что-нибудь на балалайке", True),  # свой предмет: инструмент реплики
    ("поставь Калинку", True),  # свой предмет: пьеса по названию
])
def test_playing_engine_is_not_inherited_by_a_request_with_its_own_subject_or_path(text, own_subject):
    ask, note = settle_engine(source_ask(text), None, playing="renardo", own_subject=own_subject)
    assert ask.engine == "" and note == ""


def test_nothing_is_inherited_when_nothing_plays_or_the_engine_is_unknown():
    ask = source_ask("ускорь темп")
    assert settle_engine(ask, None, playing="") == (ask, "")
    assert settle_engine(ask, None, playing="leonardo") == (ask, "")
