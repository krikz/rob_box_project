"""Issue #3775 — тема, названная человеком, доходит до заказа Suno (ADR-0158 В13, ADR-0148: тему решает код).

Живой прогон 21.10 ~11:20Z (Sound Pi, образ 042ed87df):

* «Ты диджей Воландеморт и у нас сегодня сэт для пленников прибывших в Эскобар» → ``[gen set] … тема-теги=—``:
  у темы нет строки ``GEN_THEME_TAGS`` → теги пустые → кэш «любой темы» (``"" in prompt``) → 10 треков из кэша club;
* «сочини новые треки под тему …» → ``generate_music(theme=…)`` → ``instrumental_job(genre, model)`` без темы →
  KIE получил обычный «RoBBox club».

Без сети и денег: фейковый KIE (``Kie`` из PR-8), настоящий кэш ``GeneratedMusicLibrary``.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.instrumental import (InstrumentalMaker, InstrumentalResult, instrumental_job,
                                                       ready_text, wait_text)
from rob_box_mcp_tools.engine.theme_grounding import order_theme
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import SourceAsk, theme_tags, theme_title

from .test_adr0158_pr8_gen_set import Kie, _dj, _lib

pytestmark = pytest.mark.unit

DJ_PHRASE = "Ты диджей Воландеморт и у нас сегодня сэт для пленников прибывших в Эскобар"
GEN_PHRASE = "[TG] сочини новый трек под тему Ты Воландеморт и сэт для пленников прибывших в Эскобар"
THEME = "пленников прибывших эскобар"
GEN = "generate:instrumental:V6"


def _club_cache(lib, n=4, prompt=None):
    """Кэш club без темы — то, что 21.10 сыграл сет «для пленников» вместо генерации."""
    for i in range(n):
        lib.save_track(b"MP3:150.0", prompt=prompt or kn.GEN_STYLES["club"].tags, title=f"club {i}",
                       is_instrumental=True, kind="instrumental", style_key="club", genre="club", task_id=f"OLD{i}",
                       variant=0, duration_ms=150000)


def test_free_theme_is_its_words_table_theme_is_en_tags():
    assert theme_tags(THEME) == THEME
    assert theme_tags("патриотический").startswith("patriotic")  # строка таблицы — как было
    assert theme_tags("") == "" and theme_title("") == ""
    assert theme_title(THEME) == "Пленников прибывших эскобар"
    long = " ".join(["слово"] * 30)
    assert len(theme_tags(long)) <= kn.GEN_THEME_MAX_CHARS and len(theme_title(long)) <= kn.GEN_THEME_MAX_CHARS


def test_dj_phrase_theme_goes_to_set_plan_and_off_theme_cache_is_not_taken(tmp_path, caplog):
    """Деньги есть, в кэше club без темы: каждый трек — генерация, теги темы — слова темы; строка лога с темой."""
    lib = _lib(tmp_path)
    _club_cache(lib)
    dj = _dj(lib, Kie())
    with caplog.at_level("INFO"):
        plan = dj.gen_sets.plan(THEME, "club", DJ_PHRASE, 3, named=True)
    assert plan.theme_tags == THEME
    assert [r.id for r in plan.routes] == [GEN] * 3
    line = next(r.message for r in caplog.records if "[gen set] слоты" in r.message)
    assert f"тема='{THEME}'" in line and f"тема-теги={THEME}" in line


def test_themed_cache_of_the_same_theme_is_taken_first(tmp_path):
    """Повтор сета на ту же тему — из кэша этой темы (метка темы в prompt заказа)."""
    lib = _lib(tmp_path)
    _club_cache(lib, n=2, prompt=f"{kn.GEN_STYLES['club'].tags}, {THEME}")
    _club_cache(lib, n=2)
    plan = _dj(lib, Kie()).gen_sets.plan(THEME, "club", DJ_PHRASE, 3, named=True)
    kinds = [r.kind for r in plan.routes]
    assert kinds[:2] == ["cache", "cache"] and kinds[2] == "generate"
    assert all(THEME in r.track.prompt for r in plan.routes[:2])


def test_no_money_off_theme_style_cache_beats_synth(tmp_path):
    """Денег нет, кэша темы нет: кэш стиля без темы лучше синтеза (В13: синтез — когда нечего включить)."""
    lib = _lib(tmp_path)
    _club_cache(lib, n=2)
    dj = _dj(lib, Kie())
    dj.gen_sets = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), balance=lambda: 0.0)
    plan = dj.gen_sets.plan(THEME, "club", DJ_PHRASE, 2, named=True)
    assert [r.kind for r in plan.routes] == ["cache", "cache"]


def test_dj_set_from_phrase_carries_theme_to_result(tmp_path):
    """Путь роутера (тема явно) и путь скрытого ``heard_text`` дают одну тему, сет — генерация."""
    for theme in (THEME, None):
        dj = _dj(_lib(tmp_path / str(theme)), Kie())
        out = dj.execute(action="start", theme=theme, persona="диджей Воландеморт", heard_text=DJ_PHRASE, tracks=2)
        assert out.success, out.error
        assert out.data["theme"] == THEME and out.data["source"] == "generated"
        assert set(out.data["routes"]) == {GEN}


def test_set_job_carries_theme_in_style_and_title(tmp_path):
    lib, kie = _lib(tmp_path), Kie()
    gen = gs.GenSet(provider=kie, library=lib, ask=SourceAsk(), genre="club", theme=THEME,
                    theme_tags=theme_tags(THEME), set_id="g", bpm=120, tracks=1, cache=(), spawn=lambda fn: fn(),
                    run=None)
    gen._audio = lambda item, how, no, deck: (item, how)  # noqa: SLF001 — звук не нужен: важен заказ
    gen.next_track(1, "A")
    job = kie.jobs[0]
    assert job.style_tags == f"{kn.GEN_STYLES['club'].tags}, {THEME}"
    assert job.title == "Пленников прибывших эскобар"


def test_order_theme_is_the_theme_words_of_the_request():
    text = "ты воландеморт и сэт для пленников прибывших в эскобар"  # theme= от грамматики
    assert order_theme(text, GEN_PHRASE) == "воландеморт и пленников прибывших эскобар"
    assert order_theme("сгенерируй рок", "сгенерируй рок") == ""  # жанр — не тема
    assert order_theme("космос", "сделай что-нибудь") == "космос"  # тема LLM, в реплике её нет
    assert order_theme(None, None) == ""


def test_instrumental_job_carries_theme_and_untitled_stays_robbox():
    job = instrumental_job("club", "V6", "воландеморт и пленников прибывших эскобар")
    assert job.title == "Воландеморт и пленников прибывших эскобар"
    assert job.style_tags == f"{kn.GEN_STYLES['club'].tags}, воландеморт и пленников прибывших эскобар"
    plain = instrumental_job("club", "V6")
    assert plain.title == "RoBBox club" and plain.style_tags == kn.GEN_STYLES["club"].tags


def test_instrumental_maker_sends_theme_to_kie_and_phrase_names_it(tmp_path):
    """Фейковый KIE: заказ с темой в style/title; фраза готовности — из результата, с темой."""
    lib, kie = _lib(tmp_path), Kie()
    maker = InstrumentalMaker(kie, lambda: lib, balance=lambda: 900.0, spawn=lambda fn: fn())
    result = maker.make("club", "V6", THEME)
    assert not result.reason and result.theme == THEME
    assert kie.jobs[0].title == "Пленников прибывших эскобар" and kie.jobs[0].style_tags.endswith(THEME)
    named = InstrumentalResult("t", "Rock T1.0", 149.0, theme=THEME)
    assert ready_text(named, "club") == ("А вот новый трек «Rock T1.0» на тему «пленников прибывших эскобар», "
                                         "клуб, 2 минуты 29 секунд.")
    titled = InstrumentalResult("t", "Пленников прибывших эскобар", 149.0, theme=THEME)
    assert ready_text(titled, "club") == "А вот новый трек «Пленников прибывших эскобар», клуб, 2 минуты 29 секунд."
    assert wait_text("club", "V6", "", THEME).startswith("Сочиняю новый трек на тему «пленников прибывших эскобар»")
