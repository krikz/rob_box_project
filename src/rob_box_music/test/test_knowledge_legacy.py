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
    assert dict(kn.ENERGY_TRIM_DB) == club_energy.ENERGY_TRIM_DB
    assert kn.REGISTERS["bass"][0] == BASS_MIDI_FLOOR
    assert tuple(float(x) for x in kn.BPM_RANGE) == BPM_RANGE
