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
from rob_box_music.model import AudioTrack, TrackError
from rob_box_music.render.audio import render_audio
from rob_box_music.source import CacheTrack, SourceAsk, theme_fit, theme_tags, theme_title

from .test_adr0158_pr8_gen_set import Kie, _dj, _fake_audio_track, _lib

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


@pytest.fixture
def decision_2110(monkeypatch):
    """Ручка решения 21.10: по теме в кэше ничего — фон и заказ всех треков темы, кэш стиля не берётся
    (``SET_THEME_MISS_STYLE_CACHE=False``; по умолчанию 29.10 — сначала кэш стиля,
    ``test_theme_chooser_known_themes``)."""
    monkeypatch.setattr(kn, "SET_THEME_MISS_STYLE_CACHE", False)


def test_dj_phrase_theme_goes_to_set_plan_and_off_theme_cache_is_not_taken(tmp_path, caplog, decision_2110):
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


# ── Решение Шифу 21.10: «надо сет 10 треков по теме, есть 2 — дозаказываем недостающие и сразу ставим что есть; если
# нечего поставить — фоном тихо что-то включить и сказать, что готовим сет» ──────────────────────────────────────────


def _near_cache(lib, n, title="Ночь в Эскобаре"):
    """Кэш club, близкий к теме по названию (метки темы в prompt нет — заказан раньше под другие слова)."""
    for i in range(n):
        lib.save_track(b"MP3:150.0", prompt=kn.GEN_STYLES["club"].tags, title=f"{title} {i}", is_instrumental=True,
                       kind="instrumental", style_key="club", genre="club", task_id=f"NEAR{i}", variant=0,
                       duration_ms=150000)


def _slots_line(caplog):
    return next(r.message for r in caplog.records if "[gen set] слоты" in r.message)


def test_theme_fit_is_share_of_theme_words_in_track():
    near = CacheTrack("a", "Ночь в Эскобаре", style_key="club", prompt=kn.GEN_STYLES["club"].tags)
    off = CacheTrack("b", "RoBBox club set", style_key="club", prompt=kn.GEN_STYLES["club"].tags)
    labelled = CacheTrack("c", "x", style_key="club", prompt=f"club house, {THEME}")
    assert theme_fit(THEME, near) == pytest.approx(1 / 3) and theme_fit(THEME, near) >= kn.THEME_FIT_MIN
    assert theme_fit(THEME, off) == 0.0 and theme_fit("", near) == 0.0
    assert theme_fit(THEME, labelled) == 1.0  # повтор темы — метка заказа
    assert theme_fit("музыка для сета", off) == 0.0  # служебные слова темы не делают


def test_theme_cache_covers_whole_set_no_orders(tmp_path, caplog):
    """K = N: всё по теме из кэша (близкое по словам, не только метка) — ни одного заказа."""
    lib = _lib(tmp_path)
    _near_cache(lib, 3)
    _club_cache(lib, 3)
    with caplog.at_level("INFO"):
        plan = _dj(lib, Kie()).gen_sets.plan(THEME, "club", DJ_PHRASE, 3, named=True)
    assert [r.kind for r in plan.routes] == ["cache"] * 3
    assert all("Эскобаре" in r.track.title for r in plan.routes)
    assert plan.gen_reason() == "theme_cache"  # сет ничего не дозаказывает
    line = _slots_line(caplog)
    assert "по теме из кэша=3 заказ=0" in line and "фон=нет" in line and "хватило кэша" in line


def test_two_of_ten_on_theme_order_eight_and_play_two_at_once(tmp_path, caplog, monkeypatch):
    """K = 2 из N = 10: 2 трека темы — сразу (первым файл темы, без моста), 8 — заказ (4 задачи по 2 варианта)."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib = _lib(tmp_path)
    _near_cache(lib, 2)
    _club_cache(lib, 4)  # кэш стиля без темы не берётся, пока можно заказать
    dj = _dj(lib, Kie())
    with caplog.at_level("INFO"):
        plan = dj.gen_sets.plan(THEME, "club", DJ_PHRASE, 10, named=True)
        out = dj.execute(action="start", theme=THEME, heard_text=DJ_PHRASE, tracks=10)
    assert [r.kind for r in plan.routes] == ["cache"] * 2 + ["generate"] * 8
    assert "по теме из кэша=2 заказ=8 (задач ≈4) фон=нет — дозаказ" in _slots_line(caplog)
    assert out.success, out.error
    first = dj._owner.played[0].code  # noqa: SLF001
    assert "loop(" in first and "amp=var([1, 0]" in first  # трек темы, полная громкость, без моста
    assert out.data["cached"] == 2
    assert out.data["source_message"].endswith("2 из сочинённого раньше, остальные допишу по ходу.")


def test_nothing_on_theme_quiet_background_phrase_and_ten_orders(tmp_path, caplog, monkeypatch, decision_2110):
    """K = 0: тихий фон из кэша стиля, фраза кода «Готовлю сет на тему …», все 10 треков — заказ."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib = _lib(tmp_path)
    _club_cache(lib, 2)
    dj = _dj(lib, Kie())
    with caplog.at_level("INFO"):
        plan = dj.gen_sets.plan(THEME, "club", DJ_PHRASE, 10, named=True)
        out = dj.execute(action="start", theme=THEME, heard_text=DJ_PHRASE, tracks=10)
    assert [r.id for r in plan.routes] == [GEN] * 10 and plan.background is not None
    assert "по теме из кэша=0 заказ=10 (задач ≈5) фон=да" in _slots_line(caplog)
    assert out.success, out.error
    assert out.data["source_message"] == f"Готовлю сет на тему «{THEME}», пока немного фона."
    first = dj._owner.played[0].code  # noqa: SLF001
    assert f"amp=var([{kn.SET_BACKGROUND_GAIN:g}, 0]" in first and "loop(" in first
    assert f"sus={kn.SET_BACKGROUND_S * kn.AUDIO_DEFAULT_BPM / 60:g}," in first  # проход фона не длиннее 60 с
    assert any("мост=фон" in r.message for r in caplog.records)


def test_nothing_on_theme_and_no_style_cache_bridge_by_synth(tmp_path, caplog):
    """K = 0 и кэша стиля нет — как раньше: мост синтезом, фона нет."""
    with caplog.at_level("INFO"):
        plan = _dj(_lib(tmp_path), Kie()).gen_sets.plan(THEME, "club", DJ_PHRASE, 3, named=True)
    assert plan.background is None and "фон=нет" in _slots_line(caplog)


def test_audio_gain_scales_note_amp_and_default_keeps_old_code(tmp_path):
    path = str(tmp_path / "t.wav")
    loud = render_audio(AudioTrack("a", path, 150.0), "A", xfade_s=1.5, once=False)
    quiet = render_audio(AudioTrack("a", path, 150.0, gain=0.35), "A", xfade_s=1.5, once=False)
    assert "amp=var([1, 0]" in loud.code and "amp=var([0.35, 0]" in quiet.code
    with pytest.raises(TrackError):
        AudioTrack("a", path, 150.0, gain=0.0)
