"""Issue #3432: renardo ``loop`` synth envelope fits inside ``sus``.

Upstream renardo_lib 0.9.13 ``LoopPygenSynthDef`` (``loop.scd``):
``Env([0,1,1,0],[0.05, sus-0.05, 0.05])`` — 50 ms fade-in and a 50 ms tail
AFTER ``sus``. A psr hit on a 16th (109 ms at 138 BPM) swelled to mid-step and
overlapped the next hit; a file shorter than ``sus`` + 50 ms restarted its head
inside the release (``PlayBuf(loop: 1)``). The patch takes ``atk``/``rel``
(seconds) and ends exactly at ``sus``.
"""

from __future__ import annotations

import re
from pathlib import Path

from rob_box_voice.core.renardo_synthdef_patches import (
    LOOP_SYNTHDEF,
    PYGEN_PATCHED_SYNTHS,
    apply_renardo_synthdef_patches,
    patch_loop_scd_content,
)

UPSTREAM_LOOP = """SynthDef.new(\\loop,
{|amp=1, sus=1, pan=0, freq=0, vib=0, fmod=0, rate=1.0, bus=0, blur=1, beat_dur=1, atk=0.01, decay=0.01, \
rel=0.01, peak=1, level=0.8, buf=0, pos=0, room=0.1, sample=0, spack=0, beat_stretch=0|
var osc, env;
sus = sus * blur;
rate = In.kr(bus, 1);
rate = (rate * (1-(beat_stretch>0))) + ((BufDur.kr(buf) / sus) * (beat_stretch>0));
osc = PlayBuf.ar(2, buf, BufRateScale.kr(buf) * rate, startPos: BufSampleRate.kr(buf) * pos, loop: 1.0);
osc = osc * EnvGen.ar(Env([0,1,1,0],[0.05, sus-0.05, 0.05]));
osc=(osc * amp);
osc = Mix(osc) * 0.5;
osc = Pan2.ar(osc, pan);
\tReplaceOut.ar(bus, osc)}).add;
"""


def _args(code: str) -> list:
    header = code[code.index("|") + 1:code.index("|", code.index("|") + 1)]
    return [a.split("=")[0].strip() for a in header.replace("\n", " ").split(",")]


def test_patched_envelope_ends_at_sus_with_atk_rel_edges():
    env = re.search(r"Env\(\[0, 1, 1, 0\], \[(.*?)\]\)", LOOP_SYNTHDEF)
    assert env, "envelope must stay a 4-point gate"
    assert [t.strip() for t in env.group(1).split(",")] == ["edgeA", "sus - edgeA - edgeR", "edgeR"]
    assert "edgeA = atk.clip(0.001, sus * 0.5);" in LOOP_SYNTHDEF
    assert "edgeR = rel.clip(0.001, sus * 0.5);" in LOOP_SYNTHDEF
    assert "0.05" not in LOOP_SYNTHDEF


def test_patch_keeps_upstream_arguments_and_playback():
    assert _args(LOOP_SYNTHDEF) == _args(UPSTREAM_LOOP)
    for line in ("rate = In.kr(bus, 1);", "loop: 1.0);", "osc = Mix(osc) * 0.5;", "ReplaceOut.ar(bus, osc)"):
        assert line in LOOP_SYNTHDEF


def test_patch_loop_scd_content_replaces_only_loop():
    assert patch_loop_scd_content(UPSTREAM_LOOP) == LOOP_SYNTHDEF
    assert patch_loop_scd_content(LOOP_SYNTHDEF) == LOOP_SYNTHDEF
    other = "SynthDef.new(\\pluck, {|amp=1| Out.ar(0, amp)}).add;\n"
    assert patch_loop_scd_content(other) == other


def test_apply_patches_both_file_and_pygen_copies(tmp_path: Path):
    for sub in ("scsynth", "tmp_code/scsynth"):
        (tmp_path / sub).mkdir(parents=True)
        (tmp_path / sub / "loop.scd").write_text(UPSTREAM_LOOP, encoding="utf-8")
    patched = apply_renardo_synthdef_patches(tmp_path)
    assert patched.count("loop.scd") == 2
    for sub in ("scsynth", "tmp_code/scsynth"):
        assert (tmp_path / sub / "loop.scd").read_text(encoding="utf-8") == LOOP_SYNTHDEF


def test_loop_is_pygen_redirected():
    """``loop`` is ``LoopPygenSynthDef``: its ``add()`` rewrites tmp_code/loop.scd on every music start."""
    assert "loop" in PYGEN_PATCHED_SYNTHS
