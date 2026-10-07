"""#3531 (аудит Ф3): затакт материала — одна фаза у мелодии, гармонии и баса.

Хук материала не срезает начальную паузу первого такта (``hook._onsets(bars=True)``): сильные доли автора падают на
доли 1/3 клуба, а затакт честно стоит перед сильной долей. У RTTTL (тактов нет) пауза срезается, как раньше.
Материал — синтетический (``test_harmony_material.synthetic``): первая четверть такта 1 — пауза (как в Super Mario Bros.).
"""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import harmony
from rob_box_music.arrange import hook as hooks
from test_harmony_material import synthetic

THEME = (0, 0, 5, 5, 3, 3, 4, 4)


def _hook(lead_in: float):
    m = synthetic(THEME, lead_in=lead_in)
    hook, _key = hooks.from_material(m, 120, 0, "major")
    return m, hook


def _author_downbeats(m, hook, first_bar=0):
    """Ноты хука, что в материале стоят на доле 1 такта автора (индексы совпадают: хук не теряет нот)."""
    notes = [e for e in m.melody if e.beat < len(hook.notes) * 4]
    pairs = list(zip(notes, hook.notes))
    return [(a, h) for a, h in pairs if a.beat % 4 == 0]


@pytest.mark.parametrize("lead_in", [0.0, 1.0, 2.0])
def test_author_strong_beats_land_on_club_beats_1_and_3(lead_in):
    m, hook = _hook(lead_in)
    down = _author_downbeats(m, hook)
    assert down, "в хуке нет нот на доле 1 автора"
    assert all(h.beat % 4 == 0 for _a, h in down), [(a.beat, h.beat) for a, h in down]
    strong = [(a, h) for a, h in zip(m.melody, hook.notes) if a.beat % 2 == 0]
    assert all(h.beat % 2 == 0 for _a, h in strong)


def test_pickup_rest_is_kept_before_the_downbeat():
    _m, hook = _hook(1.0)
    assert hook.notes[0].beat == 1.0


def test_no_pickup_starts_at_zero():
    _m, hook = _hook(0.0)
    assert hook.notes[0].beat == 0.0


def test_melody_and_harmony_share_one_phase():
    """Доля трека 0 — начало такта автора и для хука, и для гармонии/баса (``material_beat``)."""
    m = synthetic(THEME, lead_in=1.0)
    at = harmony.material_beat(m, mt.Phrase(0, 8, "new", 1))
    assert at(0.0) == 0.0 and at(1.0) == 1.0 and at(4.0) == 4.0
    hook, _ = hooks.from_material(m, 120, 0, "major")
    first_author = min(e.beat for e in m.melody)
    assert at(hook.notes[0].beat) == first_author


def test_rtttl_still_strips_leading_rest():
    onsets = hooks._onsets([(None, 1.0), (60, 1.0), (62, 1.0)], 1.0)
    assert onsets[0][0] == 0.0
    kept = hooks._onsets([(None, 1.0), (60, 1.0), (62, 1.0)], 1.0, bars=True)
    assert kept[0][0] == 1.0
