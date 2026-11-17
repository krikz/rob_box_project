"""ADR-0154 PR-1: хук из нот (общий путь) и из материала партитуры; выбор фразы — первое проведение (PR-7, отзыв
Шифу 07.10: самая повторяемая фраза ≠ главный мотив) или место контура RTTTL-эталона (``hook.for_theme``)."""

from __future__ import annotations

from dataclasses import replace

import pytest
from test_material import make_material

from melodies import LONG, SLOW
from rob_box_music import material as mt
from rob_box_music.arrange import hook as hooks
from rob_box_music.model import Key, PitchEvent
from rob_box_music.rtttl import parse_rtttl


@pytest.mark.parametrize("rtttl, bpm, root, mode", [(LONG, 132, 9, "minor"), (SLOW, 130, 2, "major")])
def test_from_notes_is_the_path_from_rtttl_takes(rtttl, bpm, root, mode):
    _name, melody_bpm, notes = parse_rtttl(rtttl)
    assert hooks.from_notes(notes, melody_bpm, "x", bpm, root, mode) == hooks.from_rtttl(rtttl, "x", bpm, root, mode)


def test_broken_rtttl_is_refused_with_reason():
    with pytest.raises(hooks.HookError, match="RTTTL не разбирается"):
        hooks.from_rtttl("not rtttl", "x", 120, 0, "major")


def _phrase_material(phrases, sections=()):
    """Четыре «фразы» по 4 такта, каждая — своя мелодия (по высоте узнаётся, откуда хук)."""
    shapes = [[0, 2, 4, 5, 7, 5, 4, 2], [7, 9, 11, 12, 14, 12, 11, 9], [4, 5, 7, 9, 11, 9, 7, 5],
              [2, 4, 5, 7, 9, 7, 5, 4]]
    melody = tuple(PitchEvent(60 + shapes[b // 4][(b * 2 + k) % 8], b * 4.0 + k * 2.0, 2.0, 2)
                   for b in range(16) for k in range(2))
    return make_material(melody=melody, phrases=tuple(phrases), sections=tuple(sections), key=Key(0, "major"))


def _first_midi(material):
    hook, _key = hooks.from_material(material, 120, 0, "major")
    return hook, hook.notes[0].midi


def test_first_statement_beats_most_repeats():
    """Без эталона — первое проведение, а не самая повторяемая фраза (Григ QmT5K8df…: repeats вёл на такт 12)."""
    m = _phrase_material([mt.Phrase(0, 4, "new", 1), mt.Phrase(4, 4, "new", 3), mt.Phrase(8, 4, "new", 2)])
    assert hooks.pick_phrase(m).bar == 0
    m = _phrase_material([mt.Phrase(8, 4, "new", 2), mt.Phrase(4, 4, "new", 2), mt.Phrase(12, 4, "new", 2)])
    assert hooks.pick_phrase(m).bar == 4


def test_only_hook_length_phrases_are_candidates():
    m = _phrase_material([mt.Phrase(0, 2, "new", 9), mt.Phrase(4, 4, "new", 1)])
    assert hooks.pick_phrase(m).bar == 4


def test_theme_section_beats_repeats():
    m = _phrase_material([mt.Phrase(0, 4, "new", 1), mt.Phrase(8, 4, "new", 5)],
                         sections=[mt.ScoreSection("Theme", 0, 4, "text")])
    assert hooks.pick_phrase(m).bar == 0
    guess = _phrase_material([mt.Phrase(0, 4, "new", 1), mt.Phrase(8, 4, "new", 5)],
                             sections=[mt.ScoreSection("theme", 8, 4, "repeat")])
    assert hooks.pick_phrase(guess).bar == 0  # секция «по повторам» — не метка автора


def test_no_phrases_falls_back_to_melody_start():
    m = _phrase_material([])
    assert hooks.pick_phrase(m) == mt.Phrase(0, hooks.HOOK_BARS[0], "new", 1)


def test_hook_is_cut_from_the_picked_phrase_and_carries_material_id():
    m = _phrase_material([mt.Phrase(4, 4, "new", 1), mt.Phrase(8, 4, "new", 3)])  # первое проведение — такт 4
    hook, first = _first_midi(m)
    start = hooks.from_material(replace(m, phrases=(mt.Phrase(0, 4, "new", 1),)), 120, 0, "major")[0]
    assert hook.source == "local:ab12cd34" == start.source
    assert hook.notes != start.notes  # другая фраза — другие ноты
    assert hook.key_fit is not None and hook.key_fit >= 0.6


def test_hook_uses_material_key_not_detected_one():
    m = _phrase_material([mt.Phrase(0, 4, "new", 1)])
    _hook, key = hooks.from_material(m, 120, 5, "major")
    assert key == Key(5, "major")  # тоника плана, лад материала


def test_three_four_is_unfit_until_accepted_by_ear():
    """#3517: пока перевод 3/4 → 4/4 не принят на слух, материал в 3/4 негоден — с причиной, не хромает под бочку."""
    melody = tuple(PitchEvent(n, i * 1.0, 1.0, 2) for i, n in enumerate([60, 62, 64, 65, 67, 65, 64, 62] * 3))
    m = make_material(meter=(3, 4), melody=melody, phrases=(mt.Phrase(0, 8, "new", 1),))
    assert hooks.material_unfit(m, 120, 0, "major") == "размер 3/4: перевод в 4/4 на приёмке (#3517)"


def test_three_four_gets_a_rest_on_the_fourth_beat(monkeypatch):
    monkeypatch.setattr(hooks.kn, "TRIPLE_METER_MODE", "pause")
    # 3/4: восьмерки в 4 такта = 12 четвертей. Акцент: после переноса каждая нота 1-й доли встаёт на начало такта 4/4.
    notes = [60, 62, 64, 65, 67, 65, 64, 62, 60, 62, 64, 62]
    melody = tuple(PitchEvent(n, i * 1.0, 1.0, 2) for i, n in enumerate(notes * 2))  # 8 тактов 3/4
    m = make_material(meter=(3, 4), melody=melody, phrases=(mt.Phrase(0, 8, "new", 1),))
    hook, _key = hooks.from_material(m, 120, 0, "major")
    beats = {round(e.beat % 4, 3) for e in hook.notes}
    assert beats == {0.0, 1.0, 2.0}  # 4-я доля (3.0) пуста: такт 3/4 дополнен паузой
    assert hook.bars == 8


@pytest.mark.parametrize("meter", [(5, 4), (6, 4), (9, 8)])
def test_unsupported_meter_is_refused_with_reason(meter):
    with pytest.raises(hooks.HookError, match="не переводится в 4/4"):
        hooks.from_material(make_material(meter=meter), 120, 0, "major")


@pytest.mark.parametrize("meter", [(3, 4), (6, 8), (3, 8), (12, 8)])
def test_triple_meters_wait_for_acceptance_in_unfit_mode(meter):
    with pytest.raises(hooks.HookError, match=r"перевод в 4/4 на приёмке \(#3517\)"):
        hooks.from_material(make_material(meter=meter), 120, 0, "major")


def test_invalid_material_is_not_turned_into_a_hook():
    with pytest.raises(mt.MaterialError):
        hooks.from_material(make_material(license="unknown"), 120, 0, "major")


# ── главный мотив по RTTTL-эталону (PR-7) ─────────────────────────────────────────────────────────────────────

#: Эталон — третья «фраза» (такты 8–11) _phrase_material и начало четвёртой: 4 5 7 9 11 9 7 5 2 4 5 7.
REF = "ref:d=4,o=4,b=120:e,f,g,a,b,a,g,f,d,e,f,g"


def test_reference_contour_picks_the_phrase_and_the_voice():
    m = _phrase_material([mt.Phrase(b, 4, "new", 5 if b == 4 else 1) for b in range(0, 16, 4)])
    themed, anchor = hooks.for_theme(m, [REF])
    assert anchor == 8 and themed is m and hooks.pick_phrase(themed, anchor).bar == 8
    hook, _key = hooks.from_material(themed, 120, 0, "major", anchor=anchor)
    steps = [b - a for a, b in zip([e.midi for e in hook.notes], [e.midi for e in hook.notes][1:])]
    assert steps[:6] == [1, 2, 2, 2, -2, -2]
    # тема в басу: мелодия — гаммы, бас — мотив эталона с такта 4 → бас становится мелодией
    low = tuple(replace(e, midi=e.midi - 24, beat=e.beat - 16.0) for e in m.melody if 32.0 <= e.beat < 56.0)
    scales = tuple(PitchEvent(72 + (i % 2), i * 2.0, 2.0, 2) for i in range(32))
    bassy = replace(m, melody=scales, bass=low)
    themed, anchor = hooks.for_theme(bassy, [REF])
    assert anchor == 4 and themed.melody == hooks.voices(bassy)["bass"]


def test_reference_without_match_refuses_the_material_and_no_reference_keeps_it():
    m = _phrase_material([mt.Phrase(b, 4, "new") for b in range(0, 16, 4)])
    with pytest.raises(hooks.HookError, match="главного мотива"):
        hooks.for_theme(m, ["other:d=4,o=4,b=120:c,c,c,c,c,c,c,c,c,c,c,c"])
    assert hooks.for_theme(m, []) == (m, None)
