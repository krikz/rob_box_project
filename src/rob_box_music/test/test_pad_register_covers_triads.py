"""Регистр пэда под низким лидом вмещает любое диатоническое трезвучие лада (баг #3487, найден воркером PR-3).

Верх пэда = низ лида − ``PAD_GAP``; при низе лида 62 это окно 50..59, а в нём нет ни C, ни C#: трезвучия с C
(C, Am, F, ...) не ставились ни в одном обращении, ``compose`` отказывал хуку («пэд не помещается»), а в RTTTL-пути
молча брал следующую мелодию. Обычное окно трека не меняется — пэд опускается (до коридора стиля) только когда
трезвучие иначе не помещается; остальные треки побайтно прежние (``test_style_same_tracks``).
"""

from __future__ import annotations


import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import harmony
from rob_box_music.model import Hook, Key, PitchEvent

CLUB = kn.STYLES["club"]
LEAD_LOW = cp.hook_register(CLUB)[0]
TOP = LEAD_LOW - cp.PAD_GAP
USUAL = (CLUB.registers["pad"][0] + kn.PAD_WIDEN, TOP)
DEGREES = tuple(range(7))


def test_defect_usual_window_has_no_voicing_of_a_c_triad():
    """Доказательство дефекта: в окне 50..59 трезвучие C-dur не ставится ни в одном обращении."""
    assert LEAD_LOW == 62 and USUAL == (50, 59)
    assert harmony.voicings((0, 4, 7), USUAL) == []
    with pytest.raises(ValueError, match="не помещается в регистр"):
        harmony.pad_chords(CLUB, Key(0, "major"), (0, 5, 3, 4), USUAL)


@pytest.mark.parametrize("mode", sorted(kn.SCALES))
@pytest.mark.parametrize("root", range(12))
def test_every_diatonic_triad_gets_a_voicing_under_the_lowest_lead(root, mode):
    key = Key(root, mode)
    for degree in DEGREES:
        register, chords = cp._pad_chords(CLUB, key, (degree,) * 4, TOP)
        pcs = set(harmony.chord_pcs(CLUB, key, degree))
        lo, hi = register
        assert CLUB.registers["pad"][0] <= lo <= USUAL[0] and hi == TOP
        assert all(lo <= m <= hi for c in chords for m in c.voicing)
        assert all({m % 12 for m in c.voicing} == pcs for c in chords)


def test_usual_window_is_kept_when_the_chords_fit_it():
    """Лид выше — окно 50..верх, как раньше: регистр и аккорды те же, что даёт ``harmony.voice_chain``."""
    key = Key(0, "major")
    for top in (60, 62, 66, CLUB.registers["pad"][1]):
        usual = (50, top)
        try:
            expected = harmony.voice_chain(CLUB, key, (0, 5, 3, 4), usual)
        except ValueError:
            continue
        assert cp._pad_chords(CLUB, key, (0, 5, 3, 4), top) == (usual, expected)


def test_widening_is_minimal():
    """Низ опускается на полутон, пока не поместится."""
    register, _chords = cp._pad_chords(CLUB, Key(0, "major"), (0, 0, 0, 0), TOP)
    assert register == (48, TOP)  # C-dur: ни в 50..59, ни в 49..59 нет C — только C=48
    register, _chords = cp._pad_chords(CLUB, Key(1, "major"), (0, 0, 0, 0), TOP)
    assert register == (49, TOP)  # C#-dur: C#=49 хватает, до 48 не опускаемся


def _hook() -> Hook:
    """4 такта четвертей лида C-dur: D E F G A G F E (дважды); самая низкая нота — D4 = 62 = ``hook_register``."""
    midi = [62, 64, 65, 67, 69, 67, 65, 64] * 2
    return Hook(tuple(PitchEvent(m, float(i), 1.0, 0) for i, m in enumerate(midi)), 4, None)


def test_hook_at_the_lowest_lead_register_is_arranged_with_c_chords():
    """Хук с низом ровно 62 (``hook_register``) и гармония из C, Am, F, G: ``_arrange`` не отказывает, пэд помещается."""
    style, key = CLUB, Key(0, "major")
    motif = _hook()
    assert min(e.midi for e in motif.notes) == LEAD_LOW
    spec = cp.form_spec(style, style.opening_form)
    fixed = cp.Harmonizer(lambda _notes, _slots: (0, 5, 3, 4), lambda _notes, _slots: ())
    arranged = cp._arrange(style, spec, motif, key, "blip", fixed)
    assert arranged.loop == (0, 5, 3, 4)
    lowest_lead = min(e.midi for e in arranged.lead.pitches)
    assert arranged.register[1] <= lowest_lead - cp.PAD_GAP
    assert all(arranged.register[0] <= m <= arranged.register[1] for c in arranged.chords.values() for m in c.voicing)
