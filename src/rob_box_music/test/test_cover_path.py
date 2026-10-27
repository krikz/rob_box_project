"""ADR-0158 PR-4 (G10 офлайн) — кавер пьесы по названию: кандидаты кода, деньги кодом, кэш того же ключа, фразы.

Решение Шифу В7 (13.10): кавер нашей мелодии — основной путь пьесы по названию; «валькирии на ситаре» → кавер с
ситаром; играть потом из кэша. Наш синтез — мост, фолбэк и единственный путь без денег. В14 (14.10): гейт кавера —
есть наш материал мелодии (RTTTL/партитура), а не «общественное достояние»: «главная тема Звёздных войн» — кавер.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser, score
from rob_box_music.source import (CacheTrack, CoverWork, SourceAsk, SourceContext, choose_route, cover_failed_text,
                                  cover_note, cover_ready_text, cover_style_key, route_note, source_ask,
                                  source_candidates)
VALKYRIES = CoverWork("Ride Of The Valkyries", "rtttl:rideofth", spoken="Полёт валькирий")
SITAR = CoverWork("Ride Of The Valkyries", "rtttl:rideofth", "sitar", "Полёт валькирий")
#: Проба координатора 14.10: партитура PDMX «STAR WARS» arr. S. Hermand (мотив RTTTL starwars_6/7/9 с такта 15).
STARWARS = CoverWork("STAR WARS", "pdmx:QmRmXQhpQ2qesBm9oFVP58yuByZcZRVfvPYAbHRAYbLvsZ", spoken="Звёздные войны")
NO_MELODY = CoverWork("Never Gonna Give You Up", "")
#: Импорт PR-1 на роботе: оба варианта пробы 13.10 (``task_id`` NULL — одна группа по названию).
IMPORTED = (CacheTrack("v0", "Ride of the Valkyries (RoBBox cover)", "cover", "orchestral", True,
                       group="cover|valk", work_id="Ride Of The Valkyries", played_ago_h=1.0),
            CacheTrack("v1", "Ride of the Valkyries (RoBBox cover)", "cover", "orchestral", True,
                       group="cover|valk", work_id="Ride Of The Valkyries"))
RICH = SourceContext(cache=IMPORTED, cover_ready=True, balance=900.0, day_left=120.0)
DET = DeterministicChooser()


def ids(text, ctx, work):
    return {o.id for o in source_candidates(source_ask(text), ctx, True, work)[0]}


def gaps(text, ctx, work):
    """Разрывы очков между соседями по убыванию (``choice.score`` с весами пути)."""
    options = source_candidates(source_ask(text), ctx, True, work)[0]
    points = sorted((score(o.features, kn.SOURCE_WEIGHTS) for o in options), reverse=True)
    return [a - b for a, b in zip(points, points[1:])]


def test_cover_tables_are_data_next_to_instruments():
    assert set(kn.COVER_STYLES) <= set(kn.INSTRUMENTS) and "sitar" in kn.COVER_STYLES
    assert kn.COVER_DEFAULT_STYLE in kn.GEN_STYLES and kn.COVER_MODEL in kn.GEN_MODELS
    assert kn.SOURCE_WEIGHTS["live"] == kn.COVER_LIVE_WEIGHT


@pytest.mark.parametrize("text,work,style", [
    ("включи полёт валькирий", VALKYRIES, "orchestral"),
    ("сыграй полёт валькирий на ситаре", SITAR, ""),
    ("сыграй полёт валькирий роком", VALKYRIES, "rock"),
])
def test_cover_style_key(text, work, style):
    assert cover_style_key(work, source_ask(text)) == style


def test_valkyries_play_from_the_imported_cache_least_recently_played_variant():
    """«включи полёт валькирий» — кэш того же ключа (work_id + instrument + style) побеждает и генерацию, и синтез."""
    assert ids("включи полёт валькирий", RICH, VALKYRIES) == {"synth:piece", "cache:v1", "generate:cover:V6"}
    route = choose_route(source_ask("включи полёт валькирий"), RICH, DET, piece=True, work=VALKYRIES)
    assert route.id == "cache:v1" and route.track.id == "v1"
    assert gaps("включи полёт валькирий", RICH, VALKYRIES)[0] >= kn.CHOOSER_CLEAR_GAP  # однозначно: без модели


def test_sitar_is_another_key_so_it_generates_and_bridges_with_our_synth():
    assert ids("сыграй полёт валькирий на ситаре", RICH, SITAR) == {"synth:piece", "generate:cover:V6"}
    route = choose_route(source_ask("сыграй полёт валькирий на ситаре"), RICH, DET, piece=True, work=SITAR)
    assert route.id == "generate:cover:V6"
    assert gaps("сыграй полёт валькирий на ситаре", RICH, SITAR)[0] >= kn.CHOOSER_CLEAR_GAP  # кавер > синтез кодом
    assert route_note(route) == "Живую версию на ситаре готовлю — секунд через 50, а пока своя."


def test_new_one_asked_skips_the_cache():
    route = choose_route(source_ask("включи новую версию полёта валькирий"), RICH, DET, piece=True, work=VALKYRIES)
    assert route.id == "generate:cover:V6"


@pytest.mark.parametrize("ctx,reason", [
    (SourceContext(cover_ready=True, balance=0.0, day_left=120.0), "no_credits"),
    (SourceContext(cover_ready=True, balance=900.0, day_left=0.0), "budget_exhausted"),
    (SourceContext(cover_ready=True, balance=900.0, day_left=120.0, provider_ok=False), "provider_down"),
    (SourceContext(cover_ready=False, balance=900.0, day_left=120.0), "not_wired"),
])
def test_no_money_no_cover_candidate_and_the_synth_says_why(ctx, reason):
    """G2/G10: денег, лимита или сервиса нет — кандидата ``generate:cover`` нет кодом; играет наш синтез с фразой."""
    options, why = source_candidates(source_ask("сыграй полёт валькирий на ситаре"), ctx, True, SITAR)
    assert {o.id for o in options} == {"synth:piece"} and why == reason
    route = choose_route(source_ask("сыграй полёт валькирий на ситаре"), ctx, DET, piece=True, work=SITAR)
    assert route_note(route) == f"Живую версию на ситаре сейчас не сделаю: {kn_words(reason)}."


def kn_words(reason):
    from rob_box_music.source import REASON_WORDS
    return REASON_WORDS[reason]


def test_star_wars_theme_is_a_cover_of_our_material_not_gated_by_public_domain():
    """В14: «если у нас есть материал — идём через ветку кавер»; узнаваемость в приоритете — кавер выше синтеза."""
    text = "сыграй главную тему звёздных войн"
    assert ids(text, RICH, STARWARS) == {"synth:piece", "generate:cover:V6"}
    route = choose_route(source_ask(text), RICH, DET, piece=True, work=STARWARS)
    assert route.id == "generate:cover:V6" and gaps(text, RICH, STARWARS)[0] >= kn.CHOOSER_CLEAR_GAP
    assert route_note(route) == "Живую версию в стиле «оркестр» готовлю — секунд через 50, а пока своя."


def test_star_wars_without_money_plays_our_synth_and_says_why():
    poor = SourceContext(cover_ready=True, balance=0.0, day_left=120.0)
    route = choose_route(source_ask("сыграй главную тему звёздных войн"), poor, DET, piece=True, work=STARWARS)
    assert route.id == "synth:piece"
    assert route_note(route) == "Живую версию в стиле «оркестр» сейчас не сделаю: на счёте генерации нет кредитов."


def test_no_melody_of_ours_no_cover_and_no_excuse():
    """Лид не собрался из нашего материала — кавера нет (``no_material``), фраза молчит: играет то, что собралось."""
    options, why = source_candidates(source_ask("сыграй never gonna give you up"), RICH, True, NO_MELODY)
    assert {o.id for o in options} == {"synth:piece"} and why == "no_material"
    route = choose_route(source_ask("сыграй never gonna give you up"), RICH, DET, piece=True, work=NO_MELODY)
    assert route.id == "synth:piece" and route_note(route) == ""


def test_disliked_cover_is_not_a_candidate():
    disliked = SourceContext(cache=(CacheTrack("v0", "x", "cover", "orchestral", True, rating=-1,
                                               work_id="Ride Of The Valkyries"),), cover_ready=True, balance=9.0e2,
                             day_left=120.0)
    assert ids("включи полёт валькирий", disliked, VALKYRIES) == {"synth:piece", "generate:cover:V6"}


def test_vocal_ask_keeps_the_old_path():
    """«спой калинку» — не кавер (кавер инструментальный): поведение PR-3 — синтез-трека в пути тула нет (спеть вживую
    под синт — путь пения диалога, #4079), генерации песни нет."""
    assert ids("спой полёт валькирий", RICH, VALKYRIES) == {"decline:not_wired"}


def test_phrases_are_built_by_code():
    ask = SourceAsk()
    assert cover_failed_text(VALKYRIES, "timeout") == "Живой версии «Полёт валькирий» не будет: генерация не " \
                                                      "уложилась по времени."
    assert cover_ready_text(SITAR, ask) == "А вот живая версия «Полёт валькирий» на ситаре."
    assert cover_ready_text(VALKYRIES, ask) == "А вот живая версия «Полёт валькирий» в стиле «оркестр»."
    assert cover_note(choose_route(ask, RICH, DET, piece=True, work=VALKYRIES)) == ""  # из кэша — фраза кэша
