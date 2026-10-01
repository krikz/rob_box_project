"""knowledge — копия значений старого кода; пока старый код жив, расхождение ловится здесь (ADR-0149 §2.2).

Старые модули лежат в ``rob_box_mcp_tools`` и ``rob_box_voice``: тест подставляет их пути сам и НЕ пропускается,
если каталоги репо на месте. Когда старый путь удалят (PR-13…15), тест удаляется вместе с ним.
"""

import pathlib
import sys

import pytest

from rob_box_music import knowledge as kn

SRC = pathlib.Path(__file__).resolve().parents[2]
for pkg in ("rob_box_mcp_tools", "rob_box_voice"):
    if (SRC / pkg).is_dir() and str(SRC / pkg) not in sys.path:
        sys.path.append(str(SRC / pkg))

if not (SRC / "rob_box_mcp_tools").is_dir():  # пакет поставлен отдельно от монорепо
    pytest.skip("нет src/rob_box_mcp_tools рядом с пакетом", allow_module_level=True)

from rob_box_mcp_tools.core import club_energy, synth_traits  # noqa: E402
from rob_box_mcp_tools.core.club_arranger import KICK_PATTERNS, ROLE_PALETTE  # noqa: E402
from rob_box_mcp_tools.core.harmonize import BASS_MIDI_FLOOR  # noqa: E402
from rob_box_mcp_tools.core.arranger import BPM_RANGE, SCALE_INTERVALS, VALID_ROOTS  # noqa: E402


def test_scales_and_roots_match_legacy():
    # renardo_events теперь реэкспорт из knowledge (PR-1b), сверяем с независимой таблицей аранжировщика.
    scales = {k: v for k, v in kn.SCALES.items() if k != kn.CHROMATIC}
    assert scales == {k: tuple(v) for k, v in SCALE_INTERVALS.items()}
    assert kn.ROOTS == tuple(VALID_ROOTS)


def test_synth_traits_match_legacy():
    legacy = {k: (v.tail, v.register, v.tail_note) for k, v in synth_traits.SYNTH_TRAITS.items()}
    assert {k: (v.tail, v.register, v.tail_note) for k, v in kn.SYNTH_TRAITS.items()} == legacy


def test_palette_kicks_energy_and_bass_floor_match_legacy():
    assert dict(kn.SYNTH_PALETTE) == dict(ROLE_PALETTE)
    assert dict(kn.KICK_PATTERNS) == KICK_PATTERNS
    assert club_energy.ENERGY_TRIM_DB is kn.ENERGY_TRIM_DB, "PR-3b: старый club_energy импортирует таблицу"
    assert kn.REGISTERS["bass"][0] == BASS_MIDI_FLOOR


def test_loudness_model_and_pan_tables_are_one_object():
    """PR-3c: модель громкости (``club_loudness``) и таблицы панорамы (``club_stereo``) живут в knowledge."""
    from rob_box_mcp_tools.core import club_arranger, club_loudness, club_stereo
    from rob_box_music.arrange.mix import layer_db

    assert club_loudness.LANE_DB_AT_UNIT is kn.LANE_DB_AT_UNIT and club_loudness.AMP_EXPONENT is kn.AMP_EXPONENT
    assert club_loudness._MEASURED_DB is kn.LAYER_MEASURED_DB and club_loudness.MODEL_SOURCE is kn.LOUDNESS_SOURCE
    assert club_loudness.layer_db is layer_db and club_arranger.MAX_LAYER_AMP is kn.MAX_LAYER_AMP
    assert (club_stereo.PAN_HATS, club_stereo.PAN_PAD_WIDTH, club_stereo.PAD_PAN_BEATS) == (
        kn.PAN_HATS, kn.PAN_PAD_WIDTH, kn.PAD_PAN_BEATS)
    from rob_box_music.arrange.mix import alternate_pan
    assert club_stereo.alternate_pan is alternate_pan, "PR-9: смена стороны по ударам — одна реализация"
    assert tuple(float(x) for x in kn.BPM_RANGE) == BPM_RANGE


def test_dave_catalog_and_diversity_have_one_implementation():
    """PR-3d: каталог DJ_Dave и ``weighted_pick``/``MusicHistory`` живут в ``rob_box_music``; старое — вид/реэкспорт."""
    from rob_box_mcp_tools.core import club_samples, music_diversity, sample_dave
    from rob_box_music import diversity

    assert music_diversity.weighted_pick is diversity.weighted_pick
    assert music_diversity.MusicHistory is diversity.MusicHistory
    view = sample_dave.sample_catalog()
    assert set(view) == set(kn.SAMPLE_CATALOG) and len(view) == 123
    assert all(view[n].path == i.loop_arg and view[n].channels == i.channels for n, i in kn.SAMPLE_CATALOG.items())
    assert sample_dave.group_descriptions() == dict(kn.SAMPLE_GROUPS)
    kinds = {"perc": "step", "loop": "stretch", "vox": "chop"}
    for layer in club_samples.SAMPLE_LAYERS.values():
        info = kn.SAMPLE_CATALOG[layer.sample]
        assert layer.kind == kinds[info.role] and layer.beats == (info.beats or 1)
    assert club_samples.SAMPLE_LAYERS["spilltab_chop"].roots == ("A#",)
