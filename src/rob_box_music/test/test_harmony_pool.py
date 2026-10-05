"""Пул прогрессий club (ADR-0149 A13): достаточно широкий и целиком пригодный для пэда."""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.harmony import (PROGRESSION_CAP, PROGRESSION_WINDOW, PROGRESSIONS, pad_chords,
                                           progression_name)
from rob_box_music.model import Key


def test_pool_is_wide_enough_for_the_cap():
    """При потолке 3 из 10 четырёх прогрессий мало: выбор по хуку упирается в потолок почти сразу."""
    names = [progression_name(p) for p in PROGRESSIONS]
    assert len(set(names)) == len(names) >= 8
    assert len(names) * PROGRESSION_CAP >= 2 * PROGRESSION_WINDOW
    assert all(p[0] == 0 and len(p) == 4 and all(0 <= d <= 6 for d in p) for p in PROGRESSIONS)


@pytest.mark.parametrize("degrees", PROGRESSIONS, ids=progression_name)
def test_every_progression_fits_the_pad_register_in_every_key(degrees):
    for mode in kn.SCALES:
        for root in range(12):
            chords = pad_chords(Key(root, mode), degrees, kn.REGISTERS["pad"])
            assert [c.degree for c in chords] == list(degrees)
