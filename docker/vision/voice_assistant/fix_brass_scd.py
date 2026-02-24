#!/usr/bin/env python3
"""Patch brass.scd in renardo_lib: resolve git merge conflict markers.

The published renardo_lib pip package contains unresolved git merge conflict
markers in brass.scd, which cause 'syntax error, unexpected BINOP' in sclang
when compiling the SynthDef.

We keep the newer version (03b34654) with the 'atk' parameter, which only uses
standard SuperCollider UGens (no sc3-plugins required for this specific synth).
"""

import pathlib

path = pathlib.Path(
    "/usr/local/lib/python3.10/dist-packages/renardo_lib"
    "/SynthDefManagement/sclang_code/scsynth/brass.scd"
)

content = path.read_text()

if "<<<<<<<" not in content:
    print("brass.scd: OK, no merge conflict markers found")
else:
    fixed = (
        "SynthDef.new(\\brass, {\n"
        "        |vib=0, rate=0.3, sus=1, fmod=0, bus=0, atk=0.01, amp=1, freq=0, pan=0|\n"
        "        var osc, env;\n"
        "        freq = In.kr(bus, 1);\n"
        "        freq = [freq, freq + fmod];\n"
        "        rate = Lag.kr(freq, rate);\n"
        "        osc = (Saw.ar(rate, 0.4) + Saw.ar((rate + LFNoise2.ar(1).range(0.5, 1.1)), 0.4));\n"
        "        osc = (osc + Resonz.ar(osc, (freq * XLine.ar(1, 5, 0.13)), 1));\n"
        "        osc = BPF.ar(osc, (freq * 2.5), 0.3);\n"
        "        osc = RLPF.ar(osc, 1300, 0.78);\n"
        "        env = EnvGen.ar(Env.perc(atk, sus, amp, 0), doneAction: 0);\n"
        "        osc = (osc * env);\n"
        "        osc = Mix(osc) * 0.5;\n"
        "        osc = Pan2.ar(osc, pan);\n"
        "        ReplaceOut.ar(bus, osc)\n"
        "}).add;\n"
    )
    path.write_text(fixed)
    print("brass.scd: merge conflict markers removed, fixes applied")
