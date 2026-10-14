"""ADR-0158 В13 (решение Шифу 13–14.10): любой сет — путь каждого трека кэш → генерация → синтез-запас.

Живой провал 14.10 06:14 UTC: «Ты диджей Кукорек Сыграй патриотический сэт в стилях клуб джангл рэйв синтвейв и
джаз, хочу 10 минут музыки» → ``dj_set(theme='патриотический', style='auto')`` ушёл мимо чойсера в синтез-сет
``dnb`` на 8 треков: ни одной строки ``[source]``, стили и «10 минут» потеряны. Без сети: фейковый KIE, настоящий
кэш ``GeneratedMusicLibrary``.
"""

from __future__ import annotations

import logging

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import set_genres, theme_tags
from rob_box_voice.core.set_length_words import heard_set_length, heard_set_seconds

from .test_adr0158_pr8_gen_set import Kie, _dj, _lib

pytestmark = pytest.mark.unit

PHRASE = ("Ты диджей Кукорек Сыграй патриотический сэт в стилях клуб джангл рэйв синтвейв и джаз, "
          "хочу 10 минут музыки")
FIVE = ("club", "dnb", "rave", "synthwave", "jazz")
GEN = "generate:instrumental:V6"


def _plan(dj, **kw):
    return dj.gen_sets.plan("патриотический", "dnb", PHRASE, heard_set_length(PHRASE), **kw)


def _maker(lib, kie, balance=lambda: None):
    return gs.GenSetMaker(kie, lambda: lib, DeterministicChooser(), balance=balance)


def test_slots_of_shifu_phrase_are_code():
    """Слоты — код (ADR-0148): 5 стилей по порядку слов, «10 минут» — время, а не 8 треков синтеза."""
    assert set_genres(PHRASE) == (FIVE, ())
    assert heard_set_seconds(PHRASE) == 600.0 and heard_set_length(PHRASE) == 8  # 8 — по 75 с трека синтеза
    assert heard_set_seconds("сет на 3 трека") is None and heard_set_seconds("на полчаса") == 1800.0
    assert set_genres("сет в стиле клуб и полька") == (("club",), ("полька",))
    assert theme_tags("патриотический").startswith("patriotic")


def test_shifu_phrase_routes_every_track_cache_or_generate(tmp_path, caplog):
    """Пустой кэш, деньги есть: 5 треков (10 мин / 150 с = 4, но стилей 5), каждый — генерация своего стиля,
    строка ``[source]`` на каждый трек; задач 5 = 60 кр. ≤ лимита сета."""
    dj = _dj(_lib(tmp_path), Kie())
    with caplog.at_level(logging.INFO):
        plan = _plan(dj)
    assert plan.genres == FIVE and [r.id for r in plan.routes] == [GEN] * 5
    lines = [r.message for r in caplog.records if "[source] сет" in r.message]
    assert len(lines) == 5 and all(f"genre={g}" in line for g, line in zip(FIVE, lines))
    assert plan.theme_tags == theme_tags("патриотический") and plan.synth_style() == "club"


def test_shifu_phrase_through_dj_set_starts_generated_set(tmp_path):
    """Тот же ход, что 14.10 (LLM: theme, persona, style=auto; ход: heard_text, heard_tracks=8): сет из генерации на
    5 треков по стилям, мост — синтез первого стиля, пока пишется первый трек; фраза — от кода."""
    dj = _dj(_lib(tmp_path), Kie())
    out = dj.execute(action="start", theme="патриотический", persona="Кукорек", style="auto", heard_text=PHRASE,
                     heard_tracks=8)
    assert out.success, out.error
    assert out.data["source"] == "generated" and out.data["styles"] == list(FIVE) and out.data["tracks"] == 5
    assert out.data["routes"] == [GEN] * 5 and dj.running
    assert out.data["source_message"] == "Пока я дописываю клуб — вот вам клуб, секунд 30."  # мост синтезом


def test_cache_first_per_track_style_and_theme(tmp_path):
    """Кэш стиля и темы — первым; трек кэша другой темы (тема с тегами) не берётся; генерация — остальным."""
    lib = _lib(tmp_path, cached=0)
    tags = theme_tags("патриотический")
    for key, prompt in (("club", f"{kn.GEN_STYLES['club'].tags}, {tags}"), ("jazz", f"{kn.GEN_STYLES['jazz'].tags}, {tags}"),
                        ("rave", kn.GEN_STYLES["rave"].tags)):  # рейв — без темы: не патриотический
        lib.save_track(b"MP3:150.0", prompt=prompt, title=f"{key} old", is_instrumental=True, kind="instrumental",
                       style_key=key, genre=key, task_id=f"OLD-{key}", variant=0, duration_ms=150000)
    dj = _dj(lib, Kie())
    routes = [r.id.split(":")[0] for r in _plan(dj).routes]
    assert routes == ["cache", "generate", "generate", "generate", "cache"]


def test_no_money_synth_set_is_fallback_with_reason(tmp_path):
    """Фейковое «нет денег», кэша нет: синтез-сет запасом, фраза кода называет причину; стиль — первый, который
    синтез играет (клуб), а не случайный «драм-н-бейс» по слову «джангл»."""
    lib = _lib(tmp_path)
    dj = _dj(lib, Kie())
    dj.gen_sets = _maker(lib, Kie(), balance=lambda: 0.0)
    plan = _plan(dj)
    assert {r.kind for r in plan.routes} <= {"synth", "decline"} and plan.gen_reason() == "no_credits"
    out = dj.execute(action="start", theme="патриотический", persona="Кукорек", style="auto", heard_text=PHRASE,
                     heard_tracks=8)
    assert out.success, out.error
    assert out.data["source_note"] == ("Сгенерировать сет сейчас не могу: на счёте генерации нет кредитов — "
                                       "играю своё: клуб.")
    assert out.data["style"] == "club"


def test_day_limit_is_a_hard_filter(tmp_path):
    """Дневной лимит выбран — генерации нет ни у одного трека (жёсткий фильтр ``budget``), синтез-запас."""
    lib = _lib(tmp_path)
    lib.ledger.record_spend("T0", 120.0, kind="instrumental", request_key="k", model="V6", gen_provider="kie_suno")
    plan = _plan(_dj(lib, Kie()))
    assert not plan.played() and plan.gen_reason() == "budget_exhausted"


def test_set_limit_caps_tasks_and_spare_variant_is_free(tmp_path):
    """Лимит сета: задач не больше ``set_credits`` / цены; второй вариант задачи закрывает трек того же стиля."""
    dj = _dj(_lib(tmp_path), Kie())
    plan = dj.gen_sets.plan("", "club", "включи клубный сет на 4 трека", 4, named=True)
    assert len(plan.routes) == 4 and all(r.id == GEN for r in plan.routes)  # 2 задачи по 2 варианта
    many = dj.gen_sets.plan("", "club", "сет в стилях клуб рейв синтвейв джаз джангл чиптюн брейкбит рок метал блюз "
                            "шансон", 11, named=True)
    gens = [r for r in many.routes if r.kind == "generate"]
    assert len(gens) == int(120.0 // 12.0) and many.routes[-1].gen_reason == "budget_exhausted"


def test_poor_with_partial_cache_plays_cache_and_names_dropped(tmp_path, monkeypatch):
    """Денег нет, в кэше клуб и джаз: сет из кэша (2 трека), выпавшие стили и причина — во фразе кода."""
    from .test_adr0158_pr8_gen_set import _fake_audio_track

    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib = _lib(tmp_path)
    for key in ("club", "jazz"):
        lib.save_track(b"MP3:150.0", prompt=f"{kn.GEN_STYLES[key].tags}, {theme_tags('патриотический')}",
                       title=f"{key} old", is_instrumental=True, kind="instrumental", style_key=key, genre=key,
                       task_id=f"OLD-{key}", variant=0, duration_ms=150000)
    dj = _dj(lib, Kie())
    dj.gen_sets = _maker(lib, Kie(), balance=lambda: 0.0)
    out = dj.execute(action="start", theme="патриотический", persona="Кукорек", style="auto", heard_text=PHRASE)
    assert out.success, out.error
    assert out.data["tracks"] == 2 and out.data["styles"] == ["club", "jazz"]
    assert out.data["source_message"] == (
        "Сет «клуб, джаз» из сочинённого раньше: 2 трека. Без «драм-н-бейс»: на счёте генерации нет кредитов. "
        "Без «рейв»: на счёте генерации нет кредитов. Без «синтвейв»: на счёте генерации нет кредитов.")


def test_multi_style_gen_set_orders_slot_styles_with_theme_tags(tmp_path):
    """Сет по стилям: задачи — по стилю ближайшего трека, теги — стиль + тема; трек слота — своего стиля."""
    lib, kie = _lib(tmp_path), Kie()
    gen = gs.GenSet(provider=kie, library=lib, ask=set_ask(), genre="club", genres=("club", "dnb", "jazz"),
                    theme="патриотический", theme_tags=theme_tags("патриотический"), set_id="g", bpm=120, tracks=3,
                    cache=(), spawn=lambda fn: fn(), run=None)
    gen._audio = lambda item, how, no, deck: (item, how)  # noqa: SLF001 — звук не нужен: важен выбор трека
    picked = [gen.next_track(no, "AB"[no % 2]) for no in (1, 2, 3)]
    assert [j.style_key for j in kie.jobs] == ["club", "dnb", "jazz"]
    assert all(j.style_tags.endswith(theme_tags("патриотический")) for j in kie.jobs)
    assert [item.genre for item, _how in picked] == ["club", "dnb", "jazz"]


def set_ask():
    from rob_box_music.source import SourceAsk

    return SourceAsk()


def test_launch_text_multi_style_head():
    assert gs.set_head(("club", "jazz")) == "Сет «клуб, джаз»" and gs.set_head("rock") == "Рок-сет"
    assert gs.set_notes(("полька",), ()) == "Стиль «полька» не знаю."
    assert gs.synth_set_note("not_wired", "club") == ""
