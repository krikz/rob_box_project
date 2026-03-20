"""Helpers for repairing broken Renardo SuperCollider SynthDef sources."""

from __future__ import annotations

import re
from pathlib import Path

CONFLICT_BLOCK_RE = re.compile(
    r"^<<<<<<<.*?\n(.*?)^=======\s*$\n(.*?)^>>>>>>>.*?$",
    re.MULTILINE | re.DOTALL,
)

BRASS_SYNTHDEF = """SynthDef.new(\\brass, {
        |vib=0, rate=0.3, sus=1, fmod=0, bus=0, atk=0.01, amp=1, freq=0, pan=0|
        var osc, env;
        freq = In.kr(bus, 1);
        freq = [freq, freq + fmod];
        rate = Lag.kr(freq, rate);
        osc = (Saw.ar(rate, 0.4) + Saw.ar((rate + LFNoise2.ar(1).range(0.5, 1.1)), 0.4));
        osc = (osc + Resonz.ar(osc, (freq * XLine.ar(1, 5, 0.13)), 1));
        osc = BPF.ar(osc, (freq * 2.5), 0.3);
        osc = RLPF.ar(osc, 1300, 0.78);
        env = EnvGen.ar(Env.perc(atk, sus, amp, 0), doneAction: 0);
        osc = (osc * env);
        osc = Mix(osc) * 0.5;
        osc = Pan2.ar(osc, pan);
        ReplaceOut.ar(bus, osc)
}).add;
"""

ORGAN_SYNTHDEF = """SynthDef.new(\\organ, {
        |vib=0, rate=0.3, sus=1, fmod=0, bus=0, atk=0.02, rel=0.18, amp=1, freq=0, pan=0|
        var osc, env, baseFreq, gate, width;
        baseFreq = Lag.kr(In.kr(bus, 1).max(20), 0.03);
        width = Lag.kr(0.48 + (vib * 0.02), 0.05).clip(0.15, 0.85);
        osc = Mix([
            VarSaw.ar(baseFreq, 0, width, 0.16),
            VarSaw.ar(baseFreq * 1.995, 0, width * 0.92, 0.1),
            Pulse.ar(baseFreq * 0.5, 0.5, 0.05),
            SinOsc.ar(baseFreq * 1.01, 0, 0.04)
        ]);
        osc = LPF.ar(osc, Lag.kr((baseFreq * 6).clip(500, 4200), 0.08));
        osc = HPF.ar(LeakDC.ar(osc), 35);
        gate = EnvGen.kr(Env([0, 1, 1, 0], [atk.max(0.01), sus.max(0.25), rel.max(0.08)], curve: -4));
        env = EnvGen.ar(Env.asr(atk.max(0.01), amp, rel.max(0.08), curve: -4), gate: gate, doneAction: 0);
        osc = Pan2.ar(osc * env * 0.55, pan);
        ReplaceOut.ar(bus, osc)
}).add;
"""


def resolve_conflicted_scd_content(content: str) -> str:
    """Resolve all git conflict blocks by keeping the bottom/theirs variant."""

    resolved = content
    while True:
        match = CONFLICT_BLOCK_RE.search(resolved)
        if match is None:
            return resolved

        theirs = match.group(2).strip("\n")
        resolved = resolved[: match.start()] + theirs + resolved[match.end() :]


def patch_brass_scd_content(content: str) -> str:
    """Replace broken brass.scd content with a known-good upstream variant."""

    if "\\brass" not in content and "brass" not in content:
        return content

    if content == BRASS_SYNTHDEF or "<<<<<<<" not in content:
        return content

    return BRASS_SYNTHDEF


def patch_organ_scd_content(content: str) -> str:
    """Replace organ.scd content with a stable anti-click variant."""

    if "\\organ" not in content and "organ" not in content:
        return content

    if content == ORGAN_SYNTHDEF:
        return content

    return ORGAN_SYNTHDEF


def apply_renardo_synthdef_patches(sclang_dir: Path) -> list[str]:
    """Patch broken Renardo .scd files in place and return modified file names."""

    patched_files: list[str] = []
    for scd_file in sorted(sclang_dir.rglob("*.scd")):
        original = scd_file.read_text(encoding="utf-8", errors="replace")
        updated = original
        had_conflict_markers = "<<<<<<<" in original

        if had_conflict_markers:
            updated = resolve_conflicted_scd_content(updated)

        if scd_file.name == "brass.scd" and had_conflict_markers:
            updated = patch_brass_scd_content(updated)

        if scd_file.name == "organ.scd":
            updated = patch_organ_scd_content(updated)

        if updated != original:
            scd_file.write_text(updated, encoding="utf-8")
            patched_files.append(scd_file.name)

    return patched_files