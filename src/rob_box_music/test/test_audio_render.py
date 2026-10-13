"""``AudioTrack`` и ``render_audio`` (ADR-0158 §2.4, PR-2): трек кэша — программа одной деки."""

import os

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.model import AudioTrack, TrackError
from rob_box_music.render.audio import render_audio
from rob_box_music.render.renardo import RenderError

WAV = os.path.abspath("music_library/abc/track.16k.wav")


def test_audio_track_is_a_file_with_positive_length():
    with pytest.raises(TrackError, match="path"):
        AudioTrack("t", "relative/track.wav", 10.0)
    with pytest.raises(TrackError, match="duration_s"):
        AudioTrack("t", WAV, 0.0)
    assert AudioTrack("t", WAV, 1.5).source == "generated"


def test_render_plays_the_whole_file_once_in_the_loop_slot():
    program = render_audio(AudioTrack("gen_1", WAV, 89.5), "B")
    slot = kn.DECK_SLOTS["B"][8]
    assert program.slots == {"audio": slot} and program.deck == "B"
    assert program.bpm == kn.AUDIO_DEFAULT_BPM and program.form_beats == 89.5  # 60 BPM: доля — секунда
    assert (f"{slot} >> loop({WAV!r}, dur=89.5, sus=89.5, atk=0.02, rel=1, "
            "amp=var([1, 0], [89.5, 89.5], start=Clock.next_bar()))") in program.code
    assert program.sample_files == frozenset({WAV}) and not program.synths and not program.samples
    assert render_audio(AudioTrack("gen_1", WAV, 89.5), "B") == program  # детерминирована


def test_nominal_bpm_sets_the_deck_tempo_and_form_length():
    program = render_audio(AudioTrack("gen_2", WAV, 60.0, bpm_nominal=120), "A")
    assert program.bpm == 120 and program.form_beats == 120.0
    assert program.form_beats * 60.0 / program.bpm == 60.0  # длина в секундах — та же


def test_unknown_deck_is_a_loud_error():
    with pytest.raises(RenderError, match="деки"):
        render_audio(AudioTrack("gen_3", WAV, 5.0), "Z")
