"""ADR-0157 PR-3 (#3560): партитуры-кандидаты произведения по полю ``composer`` и словам названия, выбор ``Chooser``.

Прогон 12.10: «Токката ре минор» без «Баха» не поднимала BWV 565 в топ-8 поиска партитур (выше по рейтингу — чужие
токкаты) и играла RTTTL; «сыграй Баха» находил «Mundian To Bach Ke» (Panjabi MC). Здесь — индекс партитур
(``ScoreIndex``) и путь ``request_music(melody)`` на синтетической библиотеке (ADR-0154 В1: наши ноты).
"""

from __future__ import annotations

from dataclasses import replace

from rob_box_music.diversity import MusicHistory
from rob_box_music.works import TITLE_EXACT, TITLE_PARTIAL, WorkRef, WorkWord

from rob_box_mcp_tools.engine.score_library import ScoreIndex
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool

from .test_adr0154_pr5_score_search import make_library, synthetic
from .test_engine_session import _rig
from .test_piece_fail_open import _hits

BACH, TOCCATA = WorkWord(("bach",)), WorkWord(("toccata",))

#: Чужие токкаты выше BWV 565 по рейтингу (как в PDMX: топ-8 «toccata» без «Баха» — не Бах).
ROWS = [{"material_id": f"pdmx:t{i}", "title": f"Toccata No. {i}", "composer": f"Composer {i}", "rating": 4.9}
        for i in range(10)] + [
    {"material_id": "pdmx:bwv565", "title": "Toccata and Fugue in D Minor", "composer": "J. S. Bach", "rating": 4.1},
    {"material_id": "pdmx:air", "title": "Air on the G String", "composer": "Johann Sebastian Bach", "rating": 4.8},
    {"material_id": "pdmx:minuet", "title": "Minuet in G", "composer": "J.S. Bach", "rating": 4.7},
    {"material_id": "pdmx:mundian", "title": "Mundian To Bach Ke", "composer": "Panjabi MC", "rating": 5.0},
    {"material_id": "pdmx:bachelor", "title": "Bachelor Blues", "composer": "Bachelors", "rating": 5.0},
    {"material_id": "pdmx:minority", "title": "Minority", "composer": "Green Day", "rating": 5.0},
]


def test_toccata_in_d_minor_finds_bwv565_above_better_rated_toccatas():
    """«Токката ре минор» (без «Баха»): тональность-слот совпала в названии — BWV 565 первым, хотя рейтинг ниже."""
    found = ScoreIndex(ROWS).work_candidates(WorkRef(title=(TOCCATA,), key="D minor"))
    assert found[0].id == "pdmx:bwv565" and found[0].title_match == TITLE_EXACT
    assert {c.title_match for c in found[1:]} == {TITLE_PARTIAL}
    assert "pdmx:minority" not in {c.id for c in found}


def test_bach_is_the_composer_field_never_a_title_word_or_a_prefix():
    found = [c.id for c in ScoreIndex(ROWS).work_candidates(WorkRef(composer=(BACH,)))]
    assert set(found) == {"pdmx:bwv565", "pdmx:air", "pdmx:minuet"}
    assert found[0] == "pdmx:air"  # без названия — по рейтингу


def test_toccata_of_bach_puts_the_work_first_then_the_nearest_of_the_composer():
    found = ScoreIndex(ROWS).work_candidates(WorkRef(composer=(BACH,), title=(TOCCATA,)))
    assert [c.id for c in found] == ["pdmx:bwv565", "pdmx:air", "pdmx:minuet"]
    assert [c.title_match for c in found] == [TITLE_EXACT, 0.0, 0.0]


def test_composer_role_comes_from_the_composer_field_share():
    index = ScoreIndex(ROWS)
    assert index.composer_role(BACH) is True
    assert index.composer_role(TOCCATA) is False
    assert index.composer_role(WorkWord(("zzzz",))) is None


def _library(tmp_path):
    def score(mid, title, composer, rating):
        return replace(synthetic(mid, title), composer=composer), rating
    items = [score(r["material_id"], r["title"], r["composer"], r["rating"]) for r in ROWS]
    return make_library(tmp_path / "lib", [m for m, _r in items], {m.material_id: r for m, r in items})


def _request(rig, library, hits, history=None):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: hits, seed=lambda: 4242,
                   scores=library)
    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, history=history,
                            records=lambda ids: {}, words=lambda word: ThemeHits())


def test_toccata_in_d_minor_plays_the_bwv565_score_and_logs_the_choice(tmp_path, caplog):
    caplog.set_level("INFO")
    lib = _library(tmp_path)
    data = _request(_rig(), lib, _hits([], "токкату", "toccata")).execute(intent="melody",
                                                                          text="сыграй токкату ре минор").data
    assert data["ok"] and data["melody_id"] == "pdmx:bwv565" and data["played"]["source"] == "score"
    assert data["played"]["replacement_for"] is None
    line = next(r.getMessage() for r in caplog.records if "выбор «токкату»" in r.getMessage())
    assert "тональность=D minor → pdmx:bwv565 (deterministic" in line


def test_bach_three_times_plays_three_bach_scores_and_never_mundian(tmp_path):
    """R3/R4: «сыграй Баха» ×3 — три разные партитуры Баха (история ``music_history``), ни одной Panjabi MC."""
    lib, history = _library(tmp_path), MusicHistory(":memory:")
    played = [_request(_rig(), lib, _hits([], "баха", "bach"), history).execute(
        intent="melody", text="сыграй баха").data["melody_id"] for _ in range(3)]
    assert sorted(played) == ["pdmx:air", "pdmx:bwv565", "pdmx:minuet"]


def test_set_theme_toccata_in_d_minor_has_no_minority_hook(tmp_path):
    """Приёмка #3560: «ставь диджей сет Токката ре минор» — тональность не слова поиска темы сета (прогон 09.10 #19:
    в сете играл «Minority», Green Day)."""
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    from rob_box_mcp_tools.engine.tools_v2 import theme_finder

    library = RtttlLibrary(db_path=str(tmp_path / "set_key.db"))
    names = theme_finder(lambda: library)("Токката ре минор").names
    assert names and not [n for n in names if "minor" in n.lower()]
    assert any("toccata" in str(library.get(n).get("title")).lower() for n in names)
