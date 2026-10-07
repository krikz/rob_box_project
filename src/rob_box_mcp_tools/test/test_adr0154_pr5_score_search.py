"""ADR-0154 PR-5: поиск материала партитуры по теме сета, ``TrackPlan.material`` из плана, ``material_id`` в логе.

Библиотека — синтетическая (наши ноты, не чужая партитура; ADR-0154 В1): каталог ``score_index.db`` + JSON, как пишет
``scripts/music/score_import.py``. Приоритет: название в индексе партитур > RTTTL > строка ``THEMES``; библиотеки нет
— сет как до PR-5, причина — строкой лога.
"""

from __future__ import annotations

import logging
import sqlite3
from typing import Sequence

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music import works
from rob_box_music.model import Key, PitchEvent
from rob_box_music.set_plan import plan_materials, seeded_plan
from rob_box_music.theme import seeded_profile

from rob_box_mcp_tools.engine.score_library import DEFAULT_DIR, ENV, INDEX_FILE, ScoreLibrary, library_dir
from rob_box_mcp_tools.engine.search import ThemeHits, score_search
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool

from .test_engine_session import _rig, _started

pytestmark = pytest.mark.unit

#: Такт на трезвучии ступени до мажора (четверти), в коридоре лида — как в ``test_harmony_material``.
_BAR = {0: (72, 76, 79, 76), 3: (77, 81, 84, 81), 4: (71, 74, 79, 74), 5: (69, 72, 76, 72)}


def synthetic(material_id: str, title: str, degrees: Sequence[int] = (0, 0, 5, 5, 3, 3, 4, 4),
              license: str = "PD") -> mt.ScoreMaterial:
    melody, spans = [], []
    for i, d in enumerate(degrees):
        for k, midi in enumerate(_BAR[d]):
            melody.append(PitchEvent(midi, i * 4 + k, 1.0, 3 if k == 0 else 2))
        spans.append(mt.ChordSpan(i * 4, 4, kn.SCALES["major"][d], "min" if d == 5 else "maj", d))
    return mt.ScoreMaterial(material_id=material_id, title=title, composer="test", source="synthetic",
                            license=license, meter=(4, 4), bpm=None, key=Key(0, "major"), melody=tuple(melody),
                            chords=tuple(spans), phrases=(mt.Phrase(0, 8, "new", 1),))


def make_library(path, materials, ratings=None):
    """Каталог как у ``score_import.py``: JSON по ``file_name_of`` и ``score_index`` в ``score_index.db``."""
    path.mkdir(parents=True, exist_ok=True)
    rows = []
    for m in materials:
        name = m.material_id.replace(":", "_") + ".json"
        (path / name).write_text(mt.to_json(m), encoding="utf-8")
        row = works.score_index_row(m, bars=8, file=name)
        rows.append(row if ratings is None else works.ScoreIndexRow(**{**row.__dict__, "rating": ratings.get(
            m.material_id)}))
    conn = sqlite3.connect(str(path / INDEX_FILE))
    works.write_score_index(conn, rows)
    conn.close()
    return ScoreLibrary(str(path))


ROWS = (
    {"material_id": "pdmx:a", "title": "Interstellar Main Theme", "rating": 4.9, "n_ratings": 8},
    {"material_id": "pdmx:b", "title": "First Step - Interstellar", "rating": 4.72, "n_ratings": 22},
    {"material_id": "local:c", "title": "Interstellar", "rating": None, "n_ratings": None},
    {"material_id": "pdmx:d", "title": "Super Mario Land 2 Ending Theme", "rating": 4.95, "n_ratings": 73},
    {"material_id": "pdmx:e", "title": "Tetris", "rating": 4.88, "n_ratings": 6},
    {"material_id": "pdmx:f", "title": "Space Oddity", "rating": 4.0, "n_ratings": 1},
)


# ── поиск по названию (search.score_search) ─────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("theme", ["интерстеллар", "Interstellar", "на тему интерстеллара", "INTERSTELLAR сет"])
def test_interstellar_finds_scores_exact_title_first_then_by_rating(theme):
    assert score_search(ROWS, theme) == ("local:c", "pdmx:a", "pdmx:b")


@pytest.mark.parametrize("theme, expected", [
    ("марио", ("pdmx:d",)),  # «марио» → mario целиком, а не основа «мари»
    ("тетрис", ("pdmx:e",)),
    ("кино", ()),  # жанр каталога — не название: хуки из RTTTL, как раньше
    ("космос", ()),  # понятие THEME_CONCEPTS («космос» → space) к партитурам не применяется: только название
    ("интерстеллар марио", ()),  # все значимые слова темы — в одном названии
    ("сет", ()),  # одни служебные слова
    ("", ()),
])
def test_score_search_needs_every_content_word_of_the_theme_in_the_title(theme, expected):
    assert score_search(ROWS, theme) == expected


# ── библиотека на устройстве (engine.score_library) ─────────────────────────────────────────────────────────

def test_default_library_dir_is_the_host_resource_pack_and_env_overrides_it(monkeypatch):
    monkeypatch.delenv(ENV, raising=False)
    assert library_dir() == DEFAULT_DIR == "/opt/rob_box/scores"
    monkeypatch.setenv(ENV, "/tmp/scores_pr5")
    assert library_dir() == "/tmp/scores_pr5" and str(ScoreLibrary().path).replace("\\", "/") == "/tmp/scores_pr5"


def test_missing_library_gives_no_rows_and_an_honest_state(tmp_path):
    lib = ScoreLibrary(str(tmp_path / "nope"))
    assert lib.rows() == () and "нет" in lib.state and "RTTTL" in lib.state
    assert lib.load(["pdmx:a"])[0] == {}


def test_library_loads_index_and_validated_materials_and_skips_broken_json(tmp_path):
    lib = make_library(tmp_path / "lib", [synthetic("pdmx:i1", "Interstellar"), synthetic("pdmx:t1", "Tetris")])
    assert {r["material_id"] for r in lib.rows()} == {"pdmx:i1", "pdmx:t1"} and "партитур 2" in lib.state
    found, skipped = lib.load(["pdmx:i1", "pdmx:zz"])
    assert set(found) == {"pdmx:i1"} and found["pdmx:i1"].title == "Interstellar"
    assert len(skipped) == 1 and skipped[0].startswith("pdmx:zz")
    (tmp_path / "lib" / "pdmx_t1.json").write_text("{битый", encoding="utf-8")
    found, skipped = lib.load(["pdmx:t1"])
    assert found == {} and "MaterialError" in skipped[0]


# ── план: материал №1 — трек 1, свежесть как у хуков (#3495) ────────────────────────────────────────────────

def test_plan_gives_material_one_to_track_one_and_the_rest_in_order():
    assert plan_materials(("m1", "m2"), 4) == ("m1", "m2", None, None)
    assert plan_materials((), 2) == (None, None)
    plan = seeded_plan(seeded_profile("интерстеллар", materials=("m1", "m2")), 7, n_tracks=3)
    assert [t.material for t in plan.tracks] == ["m1", "m2", None]
    assert plan.profile.row == "space"  # #3463: тема (темп/лад/строка THEMES) остаётся «космос»


def test_material_that_opened_the_last_set_goes_last_like_a_hook():
    history = [{"set_id": "old", "melody_name": "m1"}]  # прошлый сет открыл материал №1
    assert plan_materials(("m1", "m2", "m3"), 3, history, "new") == ("m2", "m3", "m1")


def test_theme_without_scores_plans_no_material():
    """Тема без партитур — ни одного ``material`` в плане: компоновка идёт путём хуков, как до PR-5."""
    plain = seeded_plan(seeded_profile("космос"), 11, n_tracks=2)
    assert all(t.material is None for t in plain.tracks)


# ── путь DjSetTool.execute: тема → материал → started с material_id (M6) ───────────────────────────────────

def _dj(rig, lib, found=()):
    return DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(tuple(found)),
                     seed=lambda: 4242, scores=lib, lines=False)


def test_interstellar_set_plays_score_material_and_logs_material_id(tmp_path, caplog):
    lib = make_library(tmp_path / "lib", [synthetic("local:i1", "Interstellar"),
                                          synthetic("pdmx:i2", "Interstellar Main Theme", (0, 0, 3, 3, 4, 4, 0, 0))],
                       ratings={"pdmx:i2": 4.9})
    rig = _rig()
    dj = _dj(rig, lib)
    with caplog.at_level(logging.INFO):
        data = dj.execute(action="start", theme="интерстеллар", tracks=3).data
        rig.clock.run_until(rig.clock.beat + 2)
    assert data["ok"] and data["theme_source"] == "theme"
    plan_line = next(r.getMessage() for r in caplog.records if "тема «интерстеллар»" in r.getMessage())
    assert "материалы=['local:i1', 'pdmx:i2']" in plan_line and "row=space" in plan_line
    assert any("№1 local:i1 загружен" in r.getMessage() for r in caplog.records)
    assert _started(rig)[0]["track_id"] == data["track_id"]
    started = next(r.getMessage() for r in caplog.records if " started track_id=" in r.getMessage())
    assert "material_id=local:i1" in started and "source=theme" in started


def test_no_library_on_device_plays_rtttl_hooks_with_an_honest_line(tmp_path, caplog):
    rig = _rig()
    dj = _dj(rig, ScoreLibrary(str(tmp_path / "absent")), found=("popcorn",))
    with caplog.at_level(logging.INFO):
        data = dj.execute(action="start", theme="интерстеллар", tracks=2).data
        rig.clock.run_until(rig.clock.beat + 2)
    assert data["ok"]
    line = next(r.getMessage() for r in caplog.records if "тема «интерстеллар»" in r.getMessage())
    assert "материалы=[]" in line and "хуки RTTTL" in line and "'popcorn'" in line
    started = next(r.getMessage() for r in caplog.records if " started track_id=" in r.getMessage())
    assert "material_id=-" in started
    assert not [r for r in caplog.records if r.levelno >= logging.ERROR]
