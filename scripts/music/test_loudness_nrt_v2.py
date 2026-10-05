"""Юнит-тесты харнесса ``loudness_nrt_v2.py`` без SuperCollider (issue #3422, ADR-0152 §7 PR-2).

``python -m pytest scripts/music/test_loudness_nrt_v2.py -v`` (нужны numpy и ``src/rob_box_music`` в пути —
харнесс добавляет его сам).
"""

from __future__ import annotations

import random
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import loudness_nrt_v2 as nrt  # noqa: E402

SR = nrt.SAMPLE_RATE


def _events(role: str = "pad", root: str = "A#"):
    return nrt.program_events(nrt.load_frame()["programs"][role][root])


def test_score_is_deterministic_and_independent_of_event_order():
    program, events = _events("lead")
    shuffled = list(events)
    random.Random(3).shuffle(shuffled)
    a = nrt.score(events, program.bpm, "defs", {}, {}, 70.0)
    b = nrt.score(shuffled, program.bpm, "defs", {}, {}, 70.0)
    assert a == b
    assert nrt.encode_score(a) == nrt.encode_score(b)


def test_score_timestamps_and_note_parameters_follow_program_events():
    program, events = _events("pad")
    bundles = nrt.score(events, program.bpm, "defs", {}, {}, 70.0)
    notes = bundles[3:-1]
    assert len(notes) == len(events) == 48  # аккорд из 3 нот каждые 8 долей, форма 128 долей
    beat_dur = 60.0 / 124.0
    first = sorted(events, key=lambda e: (e.beat, e.midi))[0]
    time, msgs = notes[0]
    assert time == pytest.approx(first.beat * beat_dur + nrt.NOTE_OFFSET_S)
    synth = msgs[2]
    assert synth[:2] == ("/s_new", "sinepad")
    args = dict(zip(synth[5::2], synth[6::2]))
    assert args["amp"] == pytest.approx(0.11)
    assert args["sus"] == pytest.approx(8 * beat_dur)  # ``sus`` не задан → ``dur`` (Players.py)
    assert args["freq"] == pytest.approx(nrt.midi_hz(first.midi))
    names = [m[1] for m in msgs[1:]]
    assert names == ["startSound", "sinepad", "reverb", "makeSound"]
    assert notes[-1][0] == pytest.approx(max(e.beat for e in events) * beat_dur + nrt.NOTE_OFFSET_S)


def test_score_head_loads_defs_buffers_and_master_without_dynamics():
    program, events = _events("kick")
    bundles = nrt.score(events, program.bpm, "/d", {"X0": (1, 1)}, {"X0": "/s/x.wav"}, 66.0)
    assert bundles[0] == (0.0, [("/d_loadDir", "/d"), ("/b_allocRead", 1, "/s/x.wav")])
    master = bundles[2][1][0]
    assert master[:3] == ("/s_new", "masterfilter", nrt.MASTER_NODE)
    assert dict(zip(master[5::2], master[6::2])) == {"dyn": 0.0, "gain": 0.5}
    hit = bundles[3][1]
    assert hit[2][1] == "play1" and dict(zip(hit[2][5::2], hit[2][6::2]))["buf"] == 1
    assert bundles[-1] == (66.0, [("/c_set", 0, 0.0)])


def test_effects_follow_renardo_order_and_skip_zero():
    program, events = _events("lead")
    ev = next(e for e in events if e.fx.get("hpf"))
    names = [name for name, _args in nrt._effects(ev, 0.5)]
    assert names == ["highPassFilter", "lowPassFilter", "reverb"]
    quiet = next(e for e in events if not e.fx.get("hpf"))
    assert "highPassFilter" not in [name for name, _args in nrt._effects(quiet, 0.5)]


def test_encode_score_frames_bundles_with_length_and_ntp_time():
    raw = nrt.encode_score([(1.5, [("/c_set", 0, 0.0)])])
    size = struct.unpack(">i", raw[:4])[0]
    assert size == len(raw) - 4
    assert raw[4:12] == b"#bundle\0"
    assert struct.unpack(">II", raw[12:20]) == (1, 1 << 31)


def test_frame_code_swaps_synth_and_constant_amp():
    code = nrt.load_frame()["programs"]["pad"]["D"]
    out = nrt.frame_code(code, "ambi", 0.055)
    assert "p3 >> ambi(" in out and "sinepad" not in out
    assert "amp=0.055" in out and "amp=0.11" not in out
    program, events = nrt.program_events(out)
    assert {e.synth for e in events} == {"ambi"}


def _tone(*freqs: float, seconds: float = 2.0):
    t = np.arange(int(seconds * SR)) / SR
    return sum(np.sin(2 * np.pi * f * t) for f in freqs)


@pytest.mark.parametrize("freq,band", [(80.0, 0), (1000.0, 1), (4000.0, 2)])
def test_band_shares_put_a_sine_into_its_band(freq, band):
    shares = nrt.band_shares(_tone(freq))
    assert shares[band] > 0.99
    assert sum(shares) == pytest.approx(1.0, abs=1e-9)


def test_band_shares_split_equal_tones_evenly():
    low, mid, high = nrt.band_shares(_tone(100.0, 1000.0, 3000.0))
    assert (low, mid, high) == pytest.approx((1 / 3, 1 / 3, 1 / 3), abs=0.01)


def test_band_shares_reject_silence_and_rms_of_full_scale_sine():
    with pytest.raises(ValueError):
        nrt.band_shares(np.zeros(SR))
    assert nrt.rms_db(_tone(440.0)) == pytest.approx(-3.01, abs=0.01)
    assert nrt.rms_db(np.zeros(10)) == -200.0
