"""#4012 — тематический сет: источник кавера — лучшая по годности партитура, а не первая по поиску; задачи одного
произведения берут разные источники.

Живой сет ``gen64903`` (24.10, «сет в стиле Stranger Things»): годность партитур темы к эталону RTTTL — 0.96 (барабанная
партия, первая в поиске), 1.00, 1.00, 0.96, 1.00; ``pick_score`` брал первую подошедшую (0.96), срез
``SET_COVER_MATERIALS`` резал материалы до ранжирования, все 10 слотов сета — каверы одной партитуры. Материалы здесь —
синтетические (``test_harmony_material.synthetic``): годность считает настоящий ``hook.theme_fit``.
"""

from __future__ import annotations

import logging
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine import piece_cover
from rob_box_mcp_tools.engine.classic import pick_score, rank_scores, score_songs
from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.cover import CoverResult
from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange.hook import theme_fit
from rob_box_music.choice import DeterministicChooser
from rob_box_music.model import PitchEvent
from rob_box_music.source import CoverWork, SourceAsk, cover_slot

from .test_adr0158_pr8_gen_set import Kie, _lib, fake_ffmpeg

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rob_box_music" / "test"))
from test_harmony_material import synthetic  # noqa: E402

pytestmark = pytest.mark.unit

DEGREES = (0, 0, 5, 5, 3, 3, 4, 4, 5, 5, 3, 3, 0, 0, 4, 4)
SECTIONS = (mt.ScoreSection("A", 0, 8, "text"), mt.ScoreSection("B", 8, 8, "text"))
_NAMES = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")
ST = CoverWork("Stranger Things Intro Theme", "pdmx:QmbSEvE33N", spoken="Stranger Things Intro Theme")


def _material(mid, off=()):
    """Партитура с главным мотивом в тактах 12–14; ``off`` — ноты мотива, сдвинутые на полутон (годность ниже 1)."""
    m = synthetic(DEGREES)
    bass = tuple(PitchEvent(c.root_pc + 36, c.beat, c.dur_beats, 2) for c in m.chords)
    melody = list(m.melody)
    for k in off:
        melody[48 + k] = replace(melody[48 + k], midi=melody[48 + k].midi + 1)
    return replace(m, sections=SECTIONS, bass=bass, bpm=120, material_id=mid, title=mid, melody=tuple(melody))


EXACT = _material("local:exact")
REF = "ref:d=4,o=5,b=120:" + ",".join(f"{_NAMES[e.midi % 12]}{e.midi // 12 - 1}" for e in EXACT.melody[48:60])


def _drumline(mid):
    """«Барабанная партия»: мотив с тремя нотами мимо — годность между порогом пьесы и 1."""
    return _material(mid, off=(2, 5, 8))


def test_partial_material_is_above_the_piece_threshold_but_below_exact():
    fit = theme_fit(_drumline("local:drum"), [REF])
    assert kn.CLASSIC_REF_MATCH_MIN <= fit < 1.0 and theme_fit(EXACT, [REF]) == 1.0


def test_pick_score_takes_the_best_fit_not_the_first_in_search_order():
    """Порядок поиска [0.91, 1.00, 1.00]: песня — из первой партитуры с 1.00 (прежде — из первой подошедшей 0.91);
    без эталонов годности нет — порядок поиска, как раньше."""
    mats = {"local:drum": _drumline("local:drum"), "local:exact": EXACT,
            "local:exact2": replace(EXACT, material_id="local:exact2", title="local:exact2")}
    ids = ["local:drum", "local:exact", "local:exact2"]
    pick = pick_score(mats, ids, "q", seed=1, references=[REF])
    assert pick.found and pick.melody_id == "local:exact" and pick.title == "local:exact"
    assert pick_score(mats, ids, "q", seed=1).melody_id == "local:drum"


def test_rank_scores_best_first_ties_keep_search_order_missing_last():
    mats = {"local:drum": _drumline("local:drum"), "local:exact": EXACT,
            "local:exact2": replace(EXACT, material_id="local:exact2")}
    ranked = rank_scores(mats, ["local:drum", "local:gone", "local:exact", "local:exact2"], [REF])
    assert [mid for mid, _fit in ranked] == ["local:exact", "local:exact2", "local:drum", "local:gone"]
    assert ranked[0][1] == 1.0 and ranked[-1][1] == 0.0
    assert [mid for mid, _f in rank_scores(mats, ["local:drum", "local:exact"])] == ["local:drum", "local:exact"]
    built, reasons = score_songs(mats, ["local:gone", "local:exact", "local:exact2"], seed=1, references=[REF],
                                 limit=1)
    assert [mid for mid, _t in built] == ["local:exact"] and len(reasons) == 1 and "local:gone" in reasons[0]


def test_theme_covers_rank_all_materials_before_the_limit_and_log_every_fit(monkeypatch, caplog):
    """Живой порядок поиска «Stranger Things» (0.91, 1.00, 1.00, 0.91, 1.00): мелодии кавера — три партитуры 1.00
    (прежде срез первых трёх до ранжирования и первая подошедшая — барабанная партия), затем RTTTL хука; лог называет
    годность всех пяти и сколько вне лимита."""
    ids = ("pdmx:drum1", "pdmx:theme", "pdmx:main", "pdmx:drum2", "pdmx:bells")
    loaded = {mid: (_drumline(mid) if "drum" in mid else replace(EXACT, material_id=mid, title=mid)) for mid in ids}
    profile = SimpleNamespace(materials=ids, theme_hooks=("user_st",), found=("user_st",), row="")
    dj = SimpleNamespace(theme_profile=lambda theme, find=None: profile, quick_find=None, _scores=None)
    records = lambda hooks: {h: ({"name": h, "rtttl": REF}, ST.title) for h in hooks}  # noqa: E731
    monkeypatch.setattr(piece_cover, "PlanMaterials", lambda lib, mids, log: {m: loaded[m] for m in mids})
    monkeypatch.setattr(piece_cover, "pick_record", lambda rec, title, q, seed: SimpleNamespace(
        found=True, track=f"T-{rec['name']}"))
    with caplog.at_level(logging.INFO):
        got = piece_cover.theme_covers(dj, "stranger things", 7, records=records)
    assert len(got) == 1
    work, sources = got[0]
    assert [label for label, _t in sources] == ["pdmx:theme", "pdmx:main", "pdmx:bells", "rtttl:user_st"]
    assert work.material == "pdmx:theme" and kn.SET_COVER_MATERIALS == 3
    line = next(r.message for r in caplog.records if "[cover] тема" in r.message)
    assert f"«{ST.title}» ← pdmx:theme 1.00" in line
    assert "pdmx:drum1 0.91" in line and "pdmx:drum2 0.91" in line and "вне лимита 2" in line


class Maker:
    """``CoverMaker`` без сети: запоминает мелодию первой попытки, 2 варианта — в настоящий кэш."""

    def __init__(self, lib):
        self.lib, self.tracks = lib, []

    def ready(self):
        return True

    def make(self, track, work, ask, set_spent=None, early=True, attempt=None):
        self.tracks.append(track)
        task = f"T{len(self.tracks)}"
        ids = tuple(self.lib.save_track(b"MP3:90.0", prompt="cover", title=f"{work.title} {task}.{v}",
                                        is_instrumental=True, kind="cover", style_key=ask.genre, task_id=task,
                                        variant=v, duration_ms=90000, work_id=work.title)["id"] for v in range(2))
        return CoverResult(ids[0], "", task, 12.0, track_ids=ids)


SOURCES = (("pdmx:theme", "M-theme"), ("pdmx:main", "M-main"), ("pdmx:bells", "M-bells"), ("rtttl:user_st", "M-hook"))


def test_turn_sources_rotate_scores_and_keep_the_rest_as_retries():
    """Вращаются партитуры; RTTTL хука - всегда запас в хвосте (ревью #4119: четвёртая задача не начинает с хука)."""
    assert gs.turn_sources(SOURCES, 0) == SOURCES
    assert [s for s, _t in gs.turn_sources(SOURCES, 1)] == ["pdmx:main", "pdmx:bells", "pdmx:theme", "rtttl:user_st"]
    assert gs.turn_sources(SOURCES, 3) == SOURCES and gs.turn_sources((), 3) == ()


def test_one_piece_set_gives_each_task_another_source_and_neighbours_differ(tmp_path, caplog):
    """Тема с одним произведением на 6 слотов: 3 задачи — три разные партитуры (1-я, 2-я, 3-я по годности), соседние
    треки — разные задачи, значит и разные источники (прежде — все задачи из одной партитуры)."""
    lib = _lib(tmp_path)
    maker = Maker(lib)
    key = cover_slot("synthwave", ST.title)
    plan = gs.SetRoutes(("synthwave",) * 6, (), theme_covers=((ST, SOURCES),))
    run = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), cover=maker)._set_cover(plan)  # noqa: SLF001
    gen = gs.GenSet(provider=Kie(), library=lib, ask=SourceAsk(genre="synthwave"), genre="synthwave", theme="",
                    set_id="gen64903", bpm=120, tracks=6, cache=[], keys=[key] * 6,
                    covers=gs.SetCovers({key: ST}, run, (ST,)), spawn=lambda fn: fn(), run=fake_ffmpeg)
    with caplog.at_level(logging.INFO):
        tracks = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in range(1, 7)]
    assert maker.tracks == ["M-theme", "M-main", "M-bells"]
    source = dict(zip(gen.task_ids, maker.tracks))
    played = [source[gen._items[t.track_id][0].group] for t in tracks]  # noqa: SLF001
    assert len(set(played)) == 3 and all(a != b for a, b in zip(played, played[1:]))
    lines = [r.message for r in caplog.records if "задача" in r.message and "[cover]" in r.message]
    assert [line.split("мелодия ")[1].split(" ")[0] for line in lines] == ["pdmx:theme", "pdmx:main", "pdmx:bells"]
