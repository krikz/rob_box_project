"""#3517: перевод размеров материала в 4/4 клуба — режимы ``knowledge.METER_MODES`` как данные.

Сильные доли материала — на сильных долях 4/4 (1 и 3), фраза хука — целое число тактов, ноты на сетке 16-х (кроме
растяжения ×4/3 — там квантование, считается честно), отображение обратимо (гармония и бас читают материал по
доле трека)."""

from __future__ import annotations

import pytest
from test_material import make_material

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import hook as hooks
from rob_box_music.model import BEATS_PER_BAR, PitchEvent

ROWS = [(mode, meter) for mode, table in kn.METER_MODES.items() for meter in table]
GRID = 0.25
TRIPLE = [(3, 4), (6, 8), (3, 8), (12, 8)]


def _on_grid(x: float, step: float = GRID) -> bool:
    return abs(x / step - round(x / step)) < 1e-9


@pytest.mark.parametrize("mode, meter", ROWS)
def test_knots_run_from_zero_to_bar_end_and_grow(mode, meter):
    mm = mt.meter_map(meter, mode)
    assert mm.knots[0] == (0, 0) and mm.knots[-1][0] == mt.bar_beats(meter)
    assert 0 < mm.knots[-1][1] <= mm.club
    assert all(x1 > x0 and y1 > y0 for (x0, y0), (x1, y1) in zip(mm.knots, mm.knots[1:]))


@pytest.mark.parametrize("mode, meter", ROWS)
def test_downbeats_land_on_strong_club_beats(mode, meter):
    """Такт материала начинается на 1 или 3 такта 4/4 (такт в одну долю клуба, 3/8 — на доле)."""
    mm = mt.meter_map(meter, mode)
    for k in range(16):
        club = mm.to_club(k * mm.bar)
        assert _on_grid(club, 1.0)
        if mm.club >= 2:
            assert club % BEATS_PER_BAR in (0, 2)


@pytest.mark.parametrize("mode, meter", ROWS)
@pytest.mark.parametrize("bars", hooks.HOOK_BARS)
def test_hook_phrase_is_whole_club_bars(mode, meter, bars):
    assert mt.meter_map(meter, mode).to_club(bars * mt.bar_beats(meter)) % BEATS_PER_BAR == 0


@pytest.mark.parametrize("mode", ["pause", "long", "lift"])
@pytest.mark.parametrize("meter", [(3, 4), (2, 4), (4, 4)])
def test_simple_meter_sixteenths_stay_on_the_grid(mode, meter):
    mm = mt.meter_map(meter, mode)
    if mm is None:
        pytest.skip(f"{meter} не в режиме {mode}")
    assert all(_on_grid(mm.to_club(k * GRID)) for k in range(int(4 * mm.bar / GRID)))


@pytest.mark.parametrize("mode", ["long", "lift"])
@pytest.mark.parametrize("meter", [(6, 8), (3, 8), (12, 8)])
def test_compound_eighths_stay_on_the_grid(mode, meter):
    mm = mt.meter_map(meter, mode)
    assert all(_on_grid(mm.to_club(k * 0.5)) for k in range(int(4 * mm.bar / 0.5)))


@pytest.mark.parametrize("mode, meter", ROWS)
def test_club_beat_reads_back_the_material_beat(mode, meter):
    mm = mt.meter_map(meter, mode)
    for k in range(int(3 * mm.bar / GRID)):
        assert mm.from_club(mm.to_club(k * GRID)) == pytest.approx(k * GRID)


def test_pause_mode_has_silence_on_the_fourth_beat():
    mm = mt.meter_map((3, 4), "pause")
    assert mm.from_club(3.0) is None and mm.from_club(3.5) is None and mm.from_club(4.0) == 3.0


def test_stretch_quantizes_three_four_eighths_unevenly():
    """Растяжение ×4/3: восьмые 3/4 встают на 0, ⅔, 1⅓ … — после квантования к 16-м шаги 3-3-2 (триольный бег)."""
    mm = mt.meter_map((3, 4), "stretch")
    steps = [round(mm.to_club(k * 0.5) / GRID) for k in range(7)]
    assert steps == [0, 3, 5, 8, 11, 13, 16]


def _waltz(meter, bars=8):
    """Мелодия по долям (четверть 3/4 или восьмая сложных размеров), сильная доля такта — нота 72."""
    step = 1.0 if meter[1] == 4 else 0.5
    per_bar = round(mt.bar_beats(meter) / step)
    scale = [60, 62, 64, 65, 67, 69, 71]
    melody = tuple(PitchEvent(72 if k % per_bar == 0 else scale[k % 7], k * step, step, 3 if k % per_bar == 0 else 2)
                   for k in range(bars * per_bar))
    return make_material(meter=meter, melody=melody, phrases=(mt.Phrase(0, bars, "new", 1),))


@pytest.mark.parametrize("mode", ["stretch", "long", "lift"])
@pytest.mark.parametrize("meter", TRIPLE)
def test_hook_puts_material_downbeats_on_club_downbeats(monkeypatch, mode, meter):
    """Хук из 3-дольного материала: все ноты на сетке 16-х, сильные доли материала — на 1 или 3, такты целые."""
    monkeypatch.setattr(kn, "TRIPLE_METER_MODE", mode)
    m = _waltz(meter)
    hook, _key = hooks.from_material(m, 128, 0, "major")
    assert all(_on_grid(e.beat) and _on_grid(e.dur_beats) for e in hook.notes)
    mm = mt.meter_map(meter, mode)
    scale = hooks.material_scale(m, 128)
    downbeats = {round(mm.to_club(k * mm.bar) * scale, 6) for k in range(8)}
    starts = {round(e.beat, 6) for e in hook.notes}
    span = hook.bars * BEATS_PER_BAR
    expected = {b for b in downbeats if b < span}
    assert expected <= starts
    if mm.club >= 2:
        assert all(b % BEATS_PER_BAR in (0, 2) for b in expected)


def test_two_four_glues_two_bars_into_one():
    """2/4 — тот же пульс: два такта 2/4 = такт 4/4, переводить нечего (годен во всех режимах)."""
    mm = mt.meter_map((2, 4), "unfit")
    assert [mm.to_club(k * 2.0) for k in range(4)] == [0, 2, 4, 6]
    hook, _key = hooks.from_material(_waltz((2, 4), bars=16), 120, 0, "major")
    assert hook.bars in hooks.HOOK_BARS and all(_on_grid(e.beat) for e in hook.notes)


@pytest.mark.parametrize("mode, scale", [("pause", 2.0), ("stretch", 1.0), ("long", 1.0)])
def test_tempo_is_measured_in_club_beats(monkeypatch, mode, scale):
    """Requiem Recordare: 3/4, 72 под 131 — пауза оставляет ×2 (6 долей музыки + 2 тишины на 2 такта), растяжение и
    долгая доля (музыка ×4/3 в долях клуба, 96) — ×1."""
    monkeypatch.setattr(kn, "TRIPLE_METER_MODE", mode)
    assert hooks.material_scale(make_material(meter=(3, 4), bpm=72), 131) == scale
