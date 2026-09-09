SynthDef.new(\supersawlead, {
        |vib=0.03, rate=5, sus=1, bus=0, atk=0.01, amp=1, freq=0, pan=0, lpf=2200|
        var baseFreq, detune, osc, env, cutoff;
        baseFreq = In.kr(bus, 1);
        detune = [0.985, 0.993, 1.0, 1.007, 1.015];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.82, mul: 0.16));
        osc = osc + Pulse.ar(baseFreq * 2, 0.5, 0.05);
        cutoff = Lag.kr(lpf.max(baseFreq * 4), 0.12);
        osc = RLPF.ar(osc, cutoff, 0.18);
        osc = LeakDC.ar(osc);
        env = EnvGen.ar(Env.adsr(atk, 0.12, 0.75, sus * 0.6, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.8, pan);
        ReplaceOut.ar(bus, osc)
}).add;