SynthDef.new(\warmpad, {
        |vib=0.12, rate=0.25, sus=4, bus=0, atk=0.08, amp=1, freq=0, pan=0, lpf=1800|
        var baseFreq, detune, osc, env, cutoff;
        baseFreq = In.kr(bus, 1);
        detune = [0.997, 1.0, 1.004, 2.0];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.35, mul: [0.18, 0.24, 0.18, 0.08]));
        osc = osc + SinOsc.ar(baseFreq * 0.5, mul: 0.08);
        cutoff = Lag.kr(lpf.max(baseFreq * 3), 0.4);
        osc = RLPF.ar(osc, cutoff, 0.22);
        osc = LPF.ar(osc, cutoff + 400);
        osc = osc * (1 + (SinOsc.kr(rate, 0, vib, 0)));
        env = EnvGen.ar(Env.linen(atk, sus, 1.2, amp), doneAction: 0);
        osc = Pan2.ar(osc * env * 0.7, pan);
        ReplaceOut.ar(bus, osc)
}).add;