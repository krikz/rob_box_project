"""#3960 — движок музыки, названный по имени («сыграй на Ренардо»): слот ``engine`` решает код, а не кэш.

Живой прогон 23.10, акт 5: STT дал «робот сыграй на индардо спокойный ночной бит что нибудь под пайку», путь выбрал
трек кэша «STAR WARS (RoBBox cover)» — не Ренардо. Имя движка и его искажения STT живут в ``knowledge.MUSIC_ENGINES``.

#4070 — тот же путь зовут по смыслу («на синтезаторе», «на синте», «вживую», «живьём»): живой прогон 27.10, STT дал
«на рем дарда» — движок не узнан, заиграл трек библиотеки. Ключ движка — ``synth`` (путь синтеза, не генерация KIE).
"""

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, DeterministicChooser
from rob_box_music.source import (CacheTrack, SourceAsk, SourceContext, ask_engine, ask_tweak, blank_engine,
                                  choose_route, decline_text, engine_route, engine_words, route_note, set_candidates,
                                  settle_engine, source_ask, source_candidates)

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
    assert ask.engine == "synth" and engine_route(ask.engine) == "synth"
    assert "engine=synth" in ask.describe()


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
    assert [(key, text[start:end]) for key, start, end in engine_words(text)] == [("synth", "нарен дарда")]
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


@pytest.mark.parametrize("text,style", [
    ("сыграй рок на ренардо", "breaks"), ("сыграй метал на Ренардо", "breaks"), ("сыграй блюз на синтезаторе", "breaks"),
    ("сыграй шансон на ренардо", "synthwave"), ("сыграй на ренардо что-нибудь оркестровое", "synthwave"),
])
def test_named_engine_plays_a_genre_it_is_poor_at_in_the_nearest_style_not_a_decline(text, style):
    """Вилка Шифу 27.10 (2×2, без отказа): синт назван, а жанр он не играет (``synth_fit`` < ``SYNTH_FIT_MIN``) — играет
    ближайший сильный стиль (``GEN_BRIDGE_STYLES``), фраза кода говорит какой: ни отказа, ни тихой подмены."""
    ask = source_ask(text)
    assert kn.GEN_BRIDGE_STYLES[ask.genre] == style
    route = choose_route(ask, RICH, DeterministicChooser(Boom()))
    assert (route.kind, route.id) == ("synth", f"synth:{style}")
    assert route_note(route) == (f"{kn.GEN_STYLES[ask.genre].name.capitalize()} на синте играю ближайшим стилем — "
                                 f"{kn.GEN_STYLES[style].name}.")


def test_named_engine_plays_even_in_the_generate_mode_and_says_nothing_for_a_genre_it_plays():
    """Режим ``generate`` (без синтеза) — не отговорка для названного синта; жанр, который синт играет, — без фразы."""
    route = choose_route(source_ask("сыграй на ренардо"), replace(RICH, mode="generate"), DeterministicChooser(Boom()))
    assert route.id == "synth:club" and route_note(route) == ""
    assert route_note(choose_route(source_ask("сыграй синтвейв на синтезаторе"), RICH, DeterministicChooser(Boom()))) == ""


def test_named_engine_with_a_request_to_sing_is_live_singing():
    """#4079: «спой на Ренардо» — робот поёт сам под синт (``live:song``), а не отказ «движок так не играет»."""
    ask = source_ask("спой на ренардо")
    route = choose_route(ask, RICH, DeterministicChooser(Boom()))
    assert ask.engine == "synth" and ask.live == "song" and (route.id, route.kind) == ("live:song", "live")


def test_set_of_named_engine_is_synth_only():
    options, _reason = set_candidates(source_ask("сет на ренардо"), RICH)
    assert [o.id for o in options] == ["synth:club"]


def test_slot_value_from_schema_extends_words():
    """Слот ``engine`` схемы (его заполнила LLM, поняв искажённое имя) - ключ таблицы; сверка с таблицей, не со словами."""
    assert list(kn.MUSIC_ENGINES) == ["synth"] and kn.ENGINE_AUTO == "auto"
    assert SourceAsk(engine="synth").describe().endswith("engine=synth")
    assert engine_route("auto") == "" and engine_route("") == "" and engine_route("synth") == "synth"


def test_slot_without_support_in_the_words_is_dropped_with_a_reason():
    """#4013: LLM ставит ``engine=synth`` на «сыграй X на балалайке» — слов про Ренардо нет, слот отбрасывается с
    причиной в строке, и путь не сужается до синтеза (кэш и генерация остаются кандидатами)."""
    for text in ("сыграй что-нибудь на балалайке", "сыграй рок", "Робот, сыграй Running Up That Hill на балалайке"):
        ask, note = settle_engine(source_ask(text), "synth")
        assert ask.engine == "" and "без опоры в словах" in note and "engine=synth" in note
    ask = settle_engine(source_ask("сыграй что-нибудь на балалайке"), "synth")[0]
    options, _reason = source_candidates(ask, RICH)
    assert {o.id.split(":")[0] for o in options} == {"synth", "generate", "cache"}  # путь не сужен
    narrowed, _reason = source_candidates(replace(ask, engine="synth"), RICH)
    assert [o.id for o in narrowed] == ["synth:club"]  # так было до #4013: принятый слот оставлял один синтез


def test_slot_with_support_in_the_words_is_the_engine():
    """Слова называют движок (в том числе искажённо) — он и есть; слот ничего не меняет и лога не просит."""
    for text in ("поставь ровный ритм на виндарда", "нарен дарда", "наррендарда", "сыграй на индардо"):
        ask, note = settle_engine(source_ask(text), "synth")
        assert ask.engine == "synth" and note == ""
        assert settle_engine(source_ask(text), None)[0].engine == "synth"


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
    "ускорь темп", "замедли", "быстрее", "помедленнее",  # темп
    "добавь баса", "басы погромче", "жирнее", "добавь тембр",  # бас и тембр
    "развивай", "продолжи этот бит", "ещё такой же", "в том же духе", "давай в таком же духе",  # развитие
])
def test_playing_engine_is_inherited_by_a_request_that_tweaks_what_plays(text):
    """n504–n506: «ускорь», «добавь баса», «развивай» под Ренардо — движок тот же, его наследует код по снимку плеера."""
    assert source_ask(text).tweak
    ask, note = settle_engine(source_ask(text), None, playing="synth")
    assert ask.engine == "synth" and "наследуется от играющего: synth" in note
    assert engine_route(ask.engine) == "synth"
    assert settle_engine(source_ask(text), "synth", playing="synth")[0].engine == "synth"  # слот не мешает


@pytest.mark.parametrize("text", [
    # список ревьюера PR #4037: Ренардо держался до стопа для любой «чистой» просьбы
    "сыграй ещё", "поставь трек про космос", "включи музыку", "поставь Нирвану", "включи что-нибудь другое",
    "сделай потише",
    "Робот, сыграй ещё", "Робот, включи другую", "поставь Metallica", "сделай погромче", "поставь медленную музыку",
    "включи светлую музыку", "включи что-нибудь про баскетбол", "сыграй что-нибудь жёсткое",
])
def test_playing_engine_is_not_inherited_by_a_request_that_does_not_tweak_it(text):
    """Названное произведение или исполнитель, тема, «другое», «включи музыку», громкость — не правка играющего: путь
    решается заново, Ренардо до стопа не липнет (кавер и кэш остаются кандидатами)."""
    assert not source_ask(text).tweak
    assert settle_engine(source_ask(text), None, playing="synth") == (source_ask(text), "")
    assert settle_engine(source_ask(text), "synth", playing="synth")[0].engine == ""  # и слот без опоры снят


def test_tweak_words_are_one_table_and_a_phrase_must_stand_in_a_row():
    """Единственная таблица ``ENGINE_TWEAK_WORDS``: все виды правки в ней; одно слово — основа, несколько — подряд."""
    assert set(kn.ENGINE_TWEAK_WORDS) == {"tempo", "bass", "timbre", "develop"}
    assert ask_tweak("в том же духе") and not ask_tweak("в том городе же")
    assert ask_tweak("ускорь") and not ask_tweak("")


@pytest.mark.parametrize("text,own_subject", [
    ("ускорь и сгенерируй новый трек", False),  # просьба называет путь: «сгенерируй»
    ("добавь баса и спой что-нибудь", False),
    ("ускорь, сыграй рок", False),  # жанр, которого синтез не играет
    ("развивай, поставь новый трек", False),
    ("добавь баса на балалайке", True),  # свой предмет: инструмент реплики
    ("продолжи, поставь Калинку", True),  # свой предмет: пьеса по названию
])
def test_playing_engine_is_not_inherited_by_a_request_with_its_own_subject_or_path(text, own_subject):
    """Даже правка не наследует движок, если называет путь или свой предмет."""
    assert source_ask(text).tweak
    ask, note = settle_engine(source_ask(text), None, playing="synth", own_subject=own_subject)
    assert ask.engine == "" and note == ""


def test_nothing_is_inherited_when_nothing_plays_or_the_engine_is_unknown():
    ask = source_ask("ускорь темп")
    assert settle_engine(ask, None, playing="") == (ask, "")
    assert settle_engine(ask, None, playing="leonardo") == (ask, "")


# ── #4070: путь синтеза по смыслу, а не только по имени «Ренардо» ────────────────────────────────────────────────

#: Слова со смыслом «играй сам, синтезом» (предложение Шифу 27.10) и живая реплика, на которой имя не узнали.
SYNTH_BY_MEANING = [
    "сыграй на синтезаторе",
    "Робот, сыграй на синтезаторе спокойный ночной бит, что-нибудь под пайку.",
    "сыграй синтезатором", "включи синтезатор", "давай с синтезатора", "поставь синтезатору бит",  # падежи
    "сыграй на синте", "переключись на синт", "давай на синту", "сыграй синтом",
    "сыграй вживую", "сыграй вживую на синте", "Робот, сыграй живьём", "сыграй живьем что-нибудь",
    # живой прогон 27.10 (образ dev-d68a784): STT разрезал «Ренардо» на «рем дарда» — ядро ``мдард``
    "робот сыграй на рем дарда спокойный ночной бит что нибудь под пайку",
    "сыграй на ремдарда",
]


@pytest.mark.parametrize("text", SYNTH_BY_MEANING)
def test_synth_path_is_named_by_meaning_and_by_the_garbled_name(text):
    ask = source_ask(text)
    assert ask.engine == "synth" and engine_route(ask.engine) == "synth" and "engine=synth" in ask.describe()


@pytest.mark.parametrize("text", [
    # стили синтеза — слова стиля, а не путь: «синт» как основа зацепила бы все
    "поставь синтвейв", "сыграй синтвэйв", "включи synthwave", "поставь ретровейв",
    "сыграй что-нибудь в стиле синтвейв", "поставь синти-поп", "сыграй синт-поп", "поставь синт вейв",
    "включи синтпоп", "добавь синт бас",
    # составной стиль из двух слов и с предлогом «на»: «на синт» перед второй половиной стиля — не путь (ревью #4071)
    "сыграй на синт вейв", "сыграй на синт-поп", "поставь на синт-вейве", "включи на синт-вейв", "сыграй на синт бас",
    # множественное число — правка тембра («добавь синты»), не путь; прилагательное — окраска звука
    "добавь синты", "синты погромче", "включи что-нибудь с синтами", "сыграй синтезаторную музыку",
    "что-то синтетическое",
    # «живой звук» — просьба настоящих инструментов, обратная синтезу
    "включи живую музыку", "хочу живой звук", "включи живое исполнение", "сыграй живой концерт",
    # инструмент и жанр реплики (#4013) — не движок
    "сыграй на балалайке", "сыграй Running Up That Hill на балалайке", "сыграй на гитаре", "включи рок",
])
def test_styles_and_other_words_are_not_the_synth_path(text):
    ask = source_ask(text)
    assert ask.engine == "" and ask_engine(text) == "" and blank_engine(text) == text


def test_no_style_or_genre_word_of_the_tables_names_the_engine():
    """Слова стилей и жанров из таблиц знания (``STYLE_WORDS``, ``WINDOW_WORDS``, ``GEN_GENRE_WORDS``, имена
    ``GEN_STYLES``) движком не считаются — ни голыми, ни после предлога «на»; новый стиль не сломает путь, и слово
    движка не окажется в стиле."""
    words = {*kn.STYLE_WORDS, *kn.WINDOW_WORDS, *kn.GEN_GENRE_WORDS, *(style.name for style in kn.GEN_STYLES.values())}
    assert len(words) > 30
    for word in sorted(words):
        for text in (word, f"поставь {word}", f"поставь на {word}", f"сыграй {word} бит"):
            assert ask_engine(text) == "", text


def test_every_style_tail_of_the_table_cancels_the_bare_synt_and_only_it():
    """«На синт» перед второй половиной составного стиля (``SYNTH_STYLE_TAILS``) — стиль, а не путь. Отмена держится
    за запись «на синт»: «на синте вейв» и «вживую поп» хвостов не знают, путь на них остаётся."""
    assert kn.SYNTH_STYLE_TAILS and kn.MUSIC_ENGINES["synth"].words["на синт"] == kn.SYNTH_STYLE_TAILS
    for tail in kn.SYNTH_STYLE_TAILS:
        assert ask_engine(f"сыграй на синт {tail}") == "", tail
        assert blank_engine(f"сыграй на синт {tail}") == f"сыграй на синт {tail}", tail
    assert ask_engine("сыграй на синте вейв") == "synth" and ask_engine("сыграй вживую поп") == "synth"
    assert ask_engine("добавь на синт баса") == "synth"  # «баса» — не «бас»: хвосты точные слова, это «на синт» + бас
    assert ask_engine("сыграй на синт и поп") == "synth"  # хвост — слово сразу после, не где-то дальше


def test_every_entry_of_the_engine_table_names_the_engine():
    """Таблица не врёт сама себе: каждая основа, слово и ядро узнаётся в речи, записи строчные и без «ё»."""
    for engine in kn.MUSIC_ENGINES.values():
        assert engine.words and engine.stems and engine.cores
        for entry in (*engine.stems, *engine.words):
            assert ask_engine(f"сыграй {entry}") == engine.key, entry
        for core in engine.cores:  # слово не начинается ни с одной основы — узнано ядром
            assert ask_engine(f"сыграй на во{core}а") == engine.key, core
        assert all(word == word.lower().replace("ё", "е") for word in (*engine.stems, *engine.words, *engine.cores))


def test_bare_synt_needs_the_preposition_because_it_is_the_first_half_of_a_style():
    """«синт» голым — первая половина «синт вейв», «синт-поп», «синт бас»; «на синт/синте/синту» — путь."""
    assert ask_engine("поставь синт") == "" and ask_engine("сыграй синт") == ""
    assert ask_engine("сыграй на синт") == ask_engine("сыграй на синте") == ask_engine("сыграй на синту") == "synth"
    assert ask_engine("сыграй синтом") == "synth"  # творительный не бывает частью стиля


def test_engine_word_by_meaning_leaves_the_theme_and_only_it():
    """Слова по смыслу вырезаются из темы трека, как имя: «на синте» — вместе с предлогом, соседей не задевают."""
    text = "Робот, сыграй на синте спокойный ночной бит"
    assert [(key, text[start:end]) for key, start, end in engine_words(text)] == [("synth", "на синте")]
    assert blank_engine(text).split() == ["Робот,", "сыграй", "спокойный", "ночной", "бит"]
    assert blank_engine("сыграй вживую джаз").split() == ["сыграй", "джаз"]
    assert blank_engine("Сыграй живьём рок").split() == ["Сыграй", "рок"]
    assert blank_engine("играй синтезатором").split() == ["играй"]
    live = "робот сыграй на рем дарда спокойный ночной бит что нибудь под пайку"  # живой прогон 27.10
    assert blank_engine(live).split() == "робот сыграй на спокойный ночной бит что нибудь под пайку".split()
    two = "сыграй вживую на синте"  # два слова по смыслу в одной реплике — оба вырезаны, движок один
    assert [two[start:end] for _key, start, end in engine_words(two)] == ["вживую", "на синте"]
    assert ask_engine(two) == "synth"


#: Просьбы иного пути со словами по смыслу «вживую»/«на синтезаторе» (ревью #4071, блокирующее): человек не просил
#: движок — «спой вживую» это спеть, «сгенерируй вживую» это генерация. Раньше: ``engine=synth`` → «Спеть сейчас не
#: могу: названный движок так не играет» (движок не назван) и молчаливое сужение генерации до ``synth:club``.
OTHER_PATH_WITH_MEANING_WORDS = [
    "спой песню вживую", "спой вживую", "Робот, спой живьём", "спой песню на синтезаторе", "спой на синте",
    "сыграй песню своим голосом вживую", "сгенерируй трек вживую", "сгенерируй вживую", "сгенерируй на синтезаторе",
    "сочини вживую", "сочини что-нибудь на синтезаторе", "сыграй новую вживую", "поставь новый трек на синте",
]


@pytest.mark.parametrize("text", OTHER_PATH_WITH_MEANING_WORDS)
def test_meaning_words_do_not_name_the_engine_when_another_path_is_asked(text):
    ask = source_ask(text)
    assert ask.asks_path and ask.engine == "" and ask_engine(text) == "" and engine_words(text) == []
    assert blank_engine(text) == text  # слово остаётся в реплике: движок не назван, вырезать нечего


@pytest.mark.parametrize("text", ["спой на ренардо", "сгенерируй на индардо", "спой на рем дарда", "сочини на ренардо",
                                  "сыграй новую на нарен дарда"])
def test_the_name_of_the_engine_names_it_whatever_path_is_asked(text):
    """Имя «Ренардо» (и искажения STT) — прямое слово про движок: оно называет его и в просьбе иного пути."""
    ask = source_ask(text)
    assert ask.asks_path and ask.engine == "synth"


def test_by_meaning_override_is_for_the_caller_that_knows_the_path():
    """Тул «сгенерируй» сам просит иной путь, даже когда слов про него в реплике нет: ``by_meaning=False``."""
    assert ask_engine("сыграй вживую") == "synth" and ask_engine("сыграй вживую", by_meaning=False) == ""
    assert ask_engine("сыграй на ренардо", by_meaning=False) == "synth"
    assert ask_engine("спой вживую", by_meaning=True) == "synth"
    assert engine_words("сыграй на синте", by_meaning=False) == []


def test_sing_live_is_live_singing_not_a_generation_and_not_a_false_decline():
    """#4079 (решение Шифу 27.10, поправка ревью #4071): «спой вживую», «спой на синтезаторе», «спой на Ренардо» — робот
    поёт сам (``live:song``: ``perform`` диалога под синт-подложку), а не генерация KIE и не отказ «названный движок так
    не играет». Кэш и генерация не кандидаты, модель чойсера не зовётся (``Boom`` выбрал бы генерацию)."""
    for text in ("спой песню вживую", "спой вживую", "спой песню на синтезаторе", "сыграй песню своим голосом вживую",
                 "спой на ренардо", "спой на рем дарда", "Робот, спой живьём"):
        options, reason = source_candidates(source_ask(text), RICH)
        assert [o.id for o in options] == ["live:song"] and reason == "", text
        route = choose_route(source_ask(text), RICH, DeterministicChooser(Boom()))
        assert (route.kind, route.id, route.provider) == ("live", "live:song", "code"), text


@pytest.mark.parametrize("text,route_id", [
    ("сгенерируй песню на ренардо", "live:song"), ("сочини песню на синтезаторе", "live:song"),
    ("запиши рэп на Ренардо", "live:rap"), ("сгенерируй трек на ренардо", "synth:club"),
    ("сгенерируй метал на ренардо", "synth:breaks"),
])
def test_generation_on_the_named_engine_is_a_cell_of_the_fork_not_a_decline(text, route_id):
    """Решение Шифу 27.10: «он должен на «Сгенерируй песню на Ренардо» сделать перформ» — названный синт с пением —
    живое пение робота, без пения — синтез; отказа «названный движок так не играет» нет ни на одном пути."""
    route = choose_route(source_ask(text), RICH, DeterministicChooser(Boom()))
    assert route.id == route_id and route.kind != "decline"
    assert "движок так не играет" not in decline_text(route) + route_note(route)


def test_named_engine_with_a_song_kind_from_the_generation_tool_is_live_singing():
    """Тул генерации отдал пение (``vocal`` из ``kind=song``), слова назвали синт — та же клетка вилки: живое пение."""
    ask = replace(source_ask("сгенерируй на ренардо"), generated=True, vocal=True)
    options, reason = source_candidates(ask, RICH)
    assert [o.id for o in options] == ["live:song"] and reason == ""


def test_generate_live_keeps_every_path_open_instead_of_narrowing_to_the_synth():
    """«Сгенерируй вживую» и «сочини вживую» не сужаются молча до ``synth:club``: кэш и генерация остаются кандидатами."""
    for text in ("сгенерируй трек вживую", "сгенерируй вживую", "сочини вживую", "сыграй новую вживую"):
        options, _reason = source_candidates(source_ask(text), RICH)
        assert {o.id.split(":")[0] for o in options} >= {"synth", "generate", "cache"}, text
        route = choose_route(source_ask(text), RICH, DeterministicChooser(Boom()))
        assert route.kind == "generate", text


def test_slot_synth_without_the_engine_in_the_words_of_another_path_is_dropped():
    """LLM поставила ``engine=synth`` на «спой песню вживую»: слов про движок нет — слот отброшен с причиной."""
    for text in ("спой песню вживую", "сгенерируй трек вживую"):
        ask, note = settle_engine(source_ask(text), "synth")
        assert ask.engine == "" and "engine=synth без опоры в словах" in note, text


@pytest.mark.parametrize("text", ["сыграй на рем дарда", "сыграй на ремдарда", "поставь бит на рен дарда"])
def test_rem_darda_garble_of_the_name_is_caught_by_the_core(text):
    assert ask_engine(text) == "synth"


@pytest.mark.parametrize("text", ["ремонт дарда", "сыграй рем и дарда", "поставь рэм дорожку", "сыграй на дарда рем"])
def test_core_of_the_garble_needs_the_seam_of_the_name(text):
    """Ядро ``мдард`` стоит на стыке «рем|дарда»: те же слова не рядом или в другом порядке имя не дают."""
    assert ask_engine(text) == ""


def test_synth_path_beats_the_cache_that_would_win_otherwise():
    """Без слова свежий трек кэша выигрывает; с «на синтезаторе»/«вживую»/«на рем дарда» — только синтез, кэш и
    генерация KIE не кандидаты (инцидент 27.10: «на рем дарда» → трек библиотеки)."""
    assert choose_route(source_ask("поставь ночной бит"), RICH, DeterministicChooser(Boom())).kind == "cache"
    for text in ("сыграй на синтезаторе спокойный ночной бит", "сыграй вживую спокойный ночной бит",
                 "сыграй на рем дарда спокойный ночной бит что нибудь под пайку", "сыграй живьём ночной бит"):
        route = choose_route(source_ask(text), RICH, DeterministicChooser(Boom()))
        assert (route.kind, route.id) == ("synth", "synth:club"), text
        options, reason = source_candidates(source_ask(text), RICH)
        assert [o.id for o in options] == ["synth:club"] and reason == ""


def test_style_of_synth_words_keeps_all_paths_open():
    """«Синтвейв» — стиль: путь не сужен до синтеза, генерация остаётся кандидатом (движком слово не названо)."""
    options, _reason = source_candidates(source_ask("поставь синтвейв"), RICH)
    assert {o.id.split(":")[0] for o in options} >= {"synth", "generate"}


def test_slot_synth_is_backed_by_the_words_of_the_meaning():
    """Слот ``engine=synth`` от LLM с опорой в словах («на синтезаторе») принимается без записи в лог; без опоры —
    отбрасывается: «синтвейв», балалайка, «живая музыка»."""
    for text in ("сыграй на синтезаторе", "сыграй вживую", "сыграй на рем дарда"):
        ask, note = settle_engine(source_ask(text), "synth")
        assert ask.engine == "synth" and note == "", text
    for text in ("поставь синтвейв", "сыграй что-нибудь на балалайке", "включи живую музыку"):
        ask, note = settle_engine(source_ask(text), "synth")
        assert ask.engine == "" and "engine=synth без опоры в словах" in note, text
    # слот прежнего ключа ничего не прибавляет и не ломает: решают слова (схема его теперь отклоняет по enum)
    assert settle_engine(source_ask("сыграй на синтезаторе"), "renardo") == (source_ask("сыграй на синтезаторе"), "")
    assert settle_engine(source_ask("сыграй рок"), "renardo") == (source_ask("сыграй рок"), "")
