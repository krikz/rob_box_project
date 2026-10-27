"""ADR-0158 PR-3 — путь музыки: слоты просьбы, кандидаты кода с жёсткими фильтрами, выбор, фразы (G1–G3).

Живой прогон 13.10: «Робот, сгенерируй пиздатый музон своим голосом» → синтез и «Включаю музыку.» — тихая подмена.
"""

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, ChoiceQuestion, DeterministicChooser
from rob_box_music.source import (CacheTrack, SourceAsk, SourceContext, cache_text, choose_route,
                                  choose_with_fallback, decline_text, gen_block, route_note, source_ask,
                                  source_candidates)

#: Кэш как на роботе 13.10 (``/data/music_library``): варианты одной задачи — одна группа.
CACHE = (CacheTrack("a", "Я робот (РОББОКС)", "robot_song", "grunge", False, group="t1"),
         CacheTrack("b", "Я робот (РОББОКС)", "robot_song", "grunge", False, group="t1"),
         CacheTrack("c", "Robbox Grunge Test", "instrumental", "grunge", True, group="t2"),
         CacheTrack("d", "Ride of the Valkyries", "cover", "orchestral", True, group="t3"),
         CacheTrack("e", "Весна пришла", "song", "", False, group="t4"))
#: Генерация подключена, деньги есть.
RICH = SourceContext(gen_ready=True, balance=900.0, day_left=120.0)
DET = DeterministicChooser()


class Boom:
    """Модель, которая отвечает вне перечня: выбор остаётся за кодом."""

    def choose(self, question, *, deadline_ms=0):
        return Choice("generate:song:V6", 1.0, "llm")


def ids(text, ctx, piece=False):
    return {o.id for o in source_candidates(source_ask(text), ctx, piece)[0]}


@pytest.mark.parametrize("text,ask", [
    ("Робот, сгенерируй пиздатый музон своим голосом", SourceAsk(True, True, True)),
    ("спой песню", SourceAsk(vocal=True)),
    ("поставь песню калинка", SourceAsk()),  # «песня» без «сгенерируй»/«сочини» — не просьба петь
    ("сочини песню про кота", SourceAsk(vocal=True, own=True)),
    ("сгенерируй новую музыку", SourceAsk(generated=True, fresh=True)),
    ("включи гранж", SourceAsk(genre="grunge")),
    ("сыграй рок", SourceAsk(genre="rock")),
    ("включи метал", SourceAsk(genre="metal")),
    ("поставь синтвейв", SourceAsk(genre="synthwave")),
])
def test_ask_slots_are_parsed_by_code(text, ask):
    assert source_ask(text) == ask


def test_style_synth_fit_is_data():
    assert {k for k, st in kn.STYLES.items() if st.synth_fit >= kn.SYNTH_FIT_MIN} == {
        "club", "rave", "synthwave", "chiptune", "breaks", "dnb"}
    assert kn.STYLES["rock"].synth_fit == 0.0
    assert all(st.synth is None or st.synth in kn.STYLES for st in kn.GEN_STYLES.values())


@pytest.mark.parametrize("ctx", [
    SourceContext(cache=CACHE),                                          # исполнителя генерации нет (PR-3)
    SourceContext(gen_ready=True, balance=0.0, day_left=120.0),           # денег нет
    SourceContext(gen_ready=True, balance=900.0, day_left=6.0),           # дневной лимит
    SourceContext(gen_ready=True, balance=900.0, day_left=120.0, set_left=0.0),  # лимит сета
    SourceContext(gen_ready=True, balance=900.0, day_left=120.0, mode="synth"),  # режим
    SourceContext(gen_ready=True, balance=900.0, day_left=120.0, has_key=False),
    SourceContext(gen_ready=True, balance=900.0, day_left=120.0, provider_ok=False),
])
def test_no_generate_candidates_without_money_limit_mode_or_executor(ctx):
    """G2: кандидатов ``generate:*`` нет — кодом, а не верой в модель; модель вне списка — ответ кода."""
    for text in ("сгенерируй музыку", "спой песню своим голосом", "включи метал"):
        assert not any(i.startswith("generate:") for i in ids(text, ctx))
        route = choose_route(source_ask(text), ctx, DeterministicChooser(Boom()))
        assert route.kind != "generate"


def test_reasons_of_blocked_generation():
    cost = kn.GEN_MODELS[kn.DEFAULT_GEN_MODEL].credits_per_task
    assert gen_block(SourceContext(), cost) == "not_wired"
    assert gen_block(SourceContext(gen_ready=True, balance=0.0, day_left=120.0), cost) == "no_credits"
    assert gen_block(SourceContext(gen_ready=True, day_left=0.0), cost) == "budget_exhausted"
    assert gen_block(RICH, cost) == ""  # баланс и лимиты ≥ цены — можно
    assert gen_block(SourceContext(gen_ready=True, day_left=120.0), cost) == ""  # баланс неизвестен: 402 → заново


def test_candidate_sets_g1():
    """G1: набор кандидатов по фразе. Синтез не поёт и не играет рок; кэш — по жанру и голосу; варианты — один."""
    ctx = SourceContext(cache=CACHE)
    assert ids("сгенерируй музон своим голосом", ctx) == {"cache:a"}
    assert ids("спой песню", ctx) == {"cache:a", "cache:e"}
    assert ids("включи гранж", ctx) == {"cache:a", "cache:c"}
    assert ids("сыграй рок", ctx) == {"cache:a", "cache:c"}  # гранж — окно рока: тема 0.5
    assert ids("поставь клубный трек", ctx) == {"synth:club"}
    assert ids("включи метал", ctx) == {"decline:not_wired"}
    assert ids("сгенерируй музыку", SourceContext()) == {"synth:club"}
    assert ids("спой калинку", ctx, piece=True) == {"decline:not_wired"}
    assert ids("сыграй калинку", ctx, piece=True) == {"synth:piece"}
    assert "generate:robot_song:V6" in ids("сгенерируй музон своим голосом", RICH)


@pytest.mark.parametrize("text,ctx,winner", [
    ("поставь клубный трек", SourceContext(cache=CACHE), "synth:club"),
    ("включи музон", SourceContext(cache=CACHE), "cache:c"),  # сочинённое раньше выше своего синтеза (В13)
    ("сгенерируй музыку", SourceContext(cache=CACHE), "cache:c"),  # сгенерированное раньше, инструментал
    ("сгенерируй музыку", RICH, "generate:instrumental:V6"),
    ("сгенерируй музыку", SourceContext(), "synth:club"),
    ("сгенерируй музон своим голосом", SourceContext(cache=CACHE), "cache:a"),
    ("включи метал", RICH, "generate:instrumental:V6"),
])
def test_deterministic_winner(text, ctx, winner):
    assert choose_route(source_ask(text), ctx, DET).id == winner


def test_generation_failure_chooses_again_without_generate():
    """В3: генерация упала после выбора — выбор заново без ``generate:*``, второго круга нет, фраза — от кода."""
    ctx = SourceContext(cache=CACHE, gen_ready=True, balance=900.0, day_left=120.0)
    tried = []
    route = choose_with_fallback(source_ask("сгенерируй музон своим голосом"), ctx, DET,
                                 attempt=lambda r: tried.append(r.id) or "provider_down")
    assert tried == ["generate:robot_song:V6"] and route.id == "cache:a" and route.failed
    assert cache_text(route) == ("Сгенерировать не вышло: сервис генерации не отвечает. "
                                 "Ставлю спетое раньше своим голосом: «Я робот (РОББОКС)».")
    ok = choose_with_fallback(source_ask("сгенерируй музыку"), RICH, DET, attempt=lambda r: "")
    assert ok.kind == "generate" and not ok.failed


def test_phrases_name_the_ask_and_the_reason_g3():
    """G3: играет не то, что просили — фраза кода называет просьбу и причину; отказ — что могу вместо: песню — спеть
    самому вживую под синт (другая клетка вилки Шифу 27.10; «синтез не поёт» — неправда, робот поёт под синт)."""
    live = choose_route(source_ask("Робот, сгенерируй пиздатый музон своим голосом"), SourceContext(cache=CACHE), DET)
    assert cache_text(live) == ("Сгенерировать песню своим голосом сейчас не могу: генерация у меня ещё не подключена. "
                                "Ставлю спетое раньше своим голосом: «Я робот (РОББОКС)».")
    empty = choose_route(source_ask("сгенерируй музон своим голосом"), SourceContext(), DET)
    assert decline_text(empty) == ("Сгенерировать песню своим голосом сейчас не могу: генерация у меня ещё не "
                                   "подключена. Могу спеть сам вживую под синт.")
    metal = choose_route(source_ask("включи метал"), SourceContext(), DET)
    assert decline_text(metal).startswith("Метал синтезом у меня не выходит, а генерация у меня ещё не подключена.")
    synth = choose_route(source_ask("сгенерируй музыку"), SourceContext(), DET)
    assert route_note(synth) == "Сгенерировать сейчас не могу: генерация у меня ещё не подключена, играю своё."
    assert route_note(choose_route(source_ask("поставь клубный трек"), SourceContext(), DET)) == ""


# ── Решение Шифу 13–14.10 (ADR-0158 §8, В13): кэш → генерация → синтез ВЕЗДЕ; таблица «фраза → путь» ────────────
#: Кэш: рок давно игранный (не свежий), рок «нравится», клуб, гранж-инструментал и песня (как на роботе).
ROCK_OLD = CacheTrack("r", "Rock Old", "instrumental", "rock", True, played_ago_h=1.0, group="t5")
ROCK_LIKED = CacheTrack("rl", "Rock Liked", "instrumental", "rock", True, rating=kn.RATING_LIKE, group="t6")
CLUB = CacheTrack("k", "Club Cached", "instrumental", "club", True, group="t7")
SHELF = (ROCK_OLD, CLUB) + CACHE
POOR = SourceContext(cache=SHELF, gen_ready=True, balance=0.0, day_left=120.0)  # денег нет
SPENT = SourceContext(gen_ready=True, balance=900.0, day_left=0.0)  # лимит дня вышел, кэша нет


def rich(cache=()):
    return SourceContext(cache=cache, gen_ready=True, balance=900.0, day_left=120.0)


#: фраза, контекст → путь. Явное «сгенерируй» при деньгах — генерация (даже с треком «нравится» по теме);
#: «включи <жанр>» — кэш по теме (и давно не свежий), иначе генерация — и для жанров, которые синтез играет хорошо
#: (``synth_fit`` 1.0: клуб, джангл, рейв, синтвейв — только фильтр допуска синтеза); синтез — только когда нет
#: ни кэша, ни генерации (денег, лимита, провайдера).
TABLE = [
    ("Робот, сгенерируй рок", rich(SHELF), "generate:instrumental:V6"),
    ("сгенерируй рок", rich((ROCK_LIKED,)), "generate:instrumental:V6"),
    ("сгенерируй клубный трек", rich((CLUB,)), "generate:instrumental:V6"),
    ("сгенерируй новый рок", rich(SHELF), "generate:instrumental:V6"),
    ("включи рок", rich(SHELF), "cache:r"),  # трек кэша игран час назад — всё равно кэш, без траты
    ("Робот, включи гранж", rich(SHELF), "cache:c"),
    ("включи метал", rich(SHELF), "generate:instrumental:V6"),  # кэша метала нет, синтез метал не играет
    ("поставь клубный трек", rich(SHELF), "cache:k"),
    ("поставь клубный трек", rich(), "generate:instrumental:V6"),  # synth_fit 1.0 — не причина синтеза (14.10)
    ("включи джангл", rich(), "generate:instrumental:V6"),
    ("поставь синтвейв", rich(), "generate:instrumental:V6"),
    ("включи рейв", rich((CLUB,)), "generate:instrumental:V6"),  # кэш чужого стиля — не «по теме»
    ("включи музон", rich(SHELF), "cache:k"),  # жанра нет — любое сочинённое раньше (свежее), не синтез
    ("включи музон", rich(), "generate:instrumental:V6"),
    ("сгенерируй рок", POOR, "cache:r"),  # денег нет — сочинённое раньше
    ("поставь клубный трек", POOR, "cache:k"),
    ("включи джангл", POOR, "synth:dnb"),  # денег нет, кэша нет — синтез запасом
    ("включи джангл", SPENT, "synth:dnb"),  # лимит дня вышел
    ("включи джангл", replace(rich(), provider_ok=False), "synth:dnb"),  # провайдер лежит
    ("сгенерируй клубный трек", SourceContext(gen_ready=True, balance=0.0, day_left=120.0), "synth:club"),
    ("сгенерируй рок", SPENT, "decline:budget_exhausted"),
]


@pytest.mark.parametrize("text,ctx,winner", TABLE)
def test_source_weights_table(text, ctx, winner):
    """Путь победителя — с разрывом ≥ ``CHOOSER_CLEAR_GAP`` над лучшим кандидатом другого пути: код решает сам
    (треки кэша между собой — ничья, её решает модель среди кэша)."""
    route = choose_route(source_ask(text), ctx, DeterministicChooser(Boom()))
    assert route.id == winner
    options, _reason = source_candidates(source_ask(text), ctx)
    ranked = DET.rank(ChoiceQuestion("путь", options, weights=kn.SOURCE_WEIGHTS))
    other = [score for score, o in ranked if o.id.split(":")[0] != winner.split(":")[0]]
    if other:
        assert ranked[0][0] - other[0] >= kn.CHOOSER_CLEAR_GAP, ranked


def test_synth_fit_is_a_filter_not_points():
    """В13: годность синтеза — допуск кандидата, не очки: у кандидата синтеза нет признака ``synth_fit``."""
    assert "synth_fit" not in kn.SOURCE_WEIGHTS and kn.SOURCE_WEIGHTS["fallback"] < 0
    synth = next(o for o in source_candidates(source_ask("включи джангл"), rich())[0] if o.id.startswith("synth:"))
    assert "synth_fit" not in synth.features and synth.features["fallback"] == 1.0
    assert not any(o.id.startswith("synth:") for o in source_candidates(source_ask("включи джаз"), rich())[0])


def test_synth_fallback_phrase_names_why():
    """Синтез запасом при деньгах/лимите/провайдере — фраза кода говорит почему; генерации нет совсем — молчит."""
    poor = choose_route(source_ask("включи джангл"), SourceContext(gen_ready=True, balance=0.0, day_left=120.0), DET)
    assert route_note(poor) == "Сгенерировать сейчас не могу: на счёте генерации нет кредитов, играю своё."
    assert route_note(choose_route(source_ask("включи джангл"), SPENT, DET)) == (
        "Сгенерировать сейчас не могу: лимит генерации на сегодня вышел, играю своё.")
    assert route_note(choose_route(source_ask("включи джангл"), SourceContext(), DET)) == ""


def test_cache_freshness_still_orders_cache_tracks():
    """Свежесть кэша не сломана: из двух треков темы давно не игранный выше на ``fresh``."""
    fresh = CacheTrack("f", "Rock Fresh", "instrumental", "rock", True, played_ago_h=48.0, group="t8")
    ctx = SourceContext(cache=(ROCK_OLD, fresh))
    options = {o.id: o for o in source_candidates(source_ask("включи рок"), ctx)[0]}
    assert options["cache:f"].features["fresh"] == 1.0 and options["cache:r"].features["fresh"] == 0.0
    assert choose_route(source_ask("включи рок"), ctx, DET).id == "cache:f"
