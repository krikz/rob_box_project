"""В15 (14.10, #3644): переход между файлами сета — ``GEN_XFADE_S`` секунд, а не 4 такта.

Шифу: «трек заканчивается вровень, другой пусть будет 1–2 сек этот кроссфейд». Плеер встаёт только на такт, поэтому
проход считается целыми тактами, а остаток такта входящий отрабатывает ``delay`` (``Program.carry_beats``): звук
входящего встаёт ровно за кроссфейд до конца уходящего — при любом темпе, в том числе у кэша без номинала (60).
"""

from __future__ import annotations

import pytest
from rob_box_music import knowledge as kn
from rob_box_music.model import BEATS_PER_BAR, AudioTrack
from rob_box_music.render.audio import entry_blend, render_audio, set_track_seconds, xfade_beats

ALL = [(60, 90.0), (60, 149.6), (124, 90.0), (124, 150.0), (90, 63.3)]


def _chain(bpm: int, durations: list) -> list:
    """Сет из файлов как его играет дека: ``(начало звука, конец звука)`` каждого трека, секунды."""
    spb, lead, boundary, spans = 60.0 / bpm, 0.0, 0.0, []
    for i, seconds in enumerate(durations):
        last = i == len(durations) - 1
        program = render_audio(AudioTrack(f"t{i}", f"/abs/{i}.wav", seconds, bpm_nominal=bpm), "A",
                               xfade_s=kn.GEN_XFADE_S, once=last, lead_beats=lead)
        start = (boundary + lead) * spb
        spans.append((start, start + seconds))
        boundary, lead = boundary + program.form_beats, program.carry_beats
    return spans


@pytest.mark.parametrize("bpm", [60, 124, 90])
def test_overlap_is_the_constant_in_seconds_at_any_tempo(bpm):
    spans = _chain(bpm, [90.0, 73.7, 150.0, 41.2])
    for (_, end), (start, _) in zip(spans, spans[1:]):
        assert end - start == pytest.approx(kn.GEN_XFADE_S, abs=0.01)
    assert kn.GEN_XFADE_S <= 2.0


@pytest.mark.parametrize("bpm", [60, 124])
def test_a_90s_file_takes_88_5s_of_the_set_and_the_last_one_whole(bpm):
    assert set_track_seconds(90.0, last=False) == pytest.approx(88.5)
    assert set_track_seconds(90.0, last=True) == 90.0
    spans = _chain(bpm, [90.0, 90.0, 90.0])
    assert spans[1][0] - spans[0][0] == pytest.approx(88.5, abs=0.01)  # следующий встал на 88.5 с
    assert spans[2][1] - spans[0][0] == pytest.approx(88.5 * 2 + 90.0, abs=0.02)  # сумма = план длины сета
    last = render_audio(AudioTrack("l", "/abs/l.wav", 90.0, bpm_nominal=bpm), "A", xfade_s=1.5, once=True)
    assert f"sus={90.0 * bpm / 60.0:g}" in last.code  # последний трек звучит целиком, не срезан до такта


@pytest.mark.parametrize("bpm,seconds", ALL)
def test_rounding_to_a_bar_never_eats_the_tail(bpm, seconds):
    track = AudioTrack("t", "/abs/t.wav", seconds, bpm_nominal=bpm)
    program = render_audio(track, "A", xfade_s=kn.GEN_XFADE_S, once=False)
    assert f"sus={seconds * bpm / 60.0:g}" in program.code  # нота звучит весь файл, форма на неё не режет
    assert program.form_beats % BEATS_PER_BAR == 0 and 0 <= program.carry_beats < BEATS_PER_BAR
    assert (program.form_beats + program.tail_beats) * 60.0 / bpm == pytest.approx(seconds, abs=0.002)


def test_entry_after_synth_lands_exactly_a_crossfade_before_the_form_end():
    for bpm in (60, 124):
        blend, delay = entry_blend(bpm)
        assert blend % BEATS_PER_BAR == 0 and 0 <= delay < BEATS_PER_BAR  # плеер на такте
        assert (blend - delay) * 60.0 / bpm == pytest.approx(kn.GEN_XFADE_S)  # звук — за кроссфейд до границы
        assert xfade_beats(bpm) == pytest.approx(kn.GEN_XFADE_S * bpm / 60.0)


def test_delay_is_in_the_program_only_when_there_is_a_remainder():
    track = AudioTrack("t", "/abs/t.wav", 90.0, bpm_nominal=60)
    assert "delay=" not in render_audio(track, "A", xfade_s=1.5, once=False).code
    assert "delay=2.5," in render_audio(track, "A", xfade_s=1.5, once=False, lead_beats=2.5).code


def test_old_bar_constant_is_gone():
    assert not hasattr(kn, "GEN_XFADE_BARS")
