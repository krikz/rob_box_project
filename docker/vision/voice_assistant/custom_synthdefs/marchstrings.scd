SynthDef.new(\marchstrings, {
        |vib=0.02, rate=3.2, sus=1.2, bus=0, atk=0.02, amp=1, freq=0, pan=0, lpf=1500|
        var baseFreq, detune, osc, env, cutoff, body;
        baseFreq = In.kr(bus, 1);
        detune = [0.997, 1.0, 1.003, 0.5];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.42, mul: [0.16, 0.22, 0.16, 0.08]));
        body = SinOsc.ar(baseFreq * 0.5, mul: 0.08) + Pulse.ar(baseFreq, 0.48, 0.06);
        cutoff = Lag.kr(lpf.max(baseFreq * 3), 0.14);
        osc = RLPF.ar(osc + body, cutoff, 0.22);
        osc = LPF.ar(osc, cutoff + 250);
        env = EnvGen.ar(Env.adsr(atk, 0.18, 0.66, sus * 0.45, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.78, pan);
        ReplaceOut.ar(bus, osc)
}).add;