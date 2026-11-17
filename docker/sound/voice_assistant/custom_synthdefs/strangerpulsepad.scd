SynthDef.new(\strangerpulsepad, {
        |vib=0.1, rate=0.22, sus=6, bus=0, atk=0.18, amp=1, freq=0, pan=0, lpf=1200|
        var baseFreq, detune, osc, env, cutoff;
        baseFreq = In.kr(bus, 1);
        detune = [0.996, 1.0, 1.003, 2.0];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.42, mul: [0.18, 0.22, 0.18, 0.06]));
        osc = osc + Pulse.ar(baseFreq * 0.5, 0.5, 0.08);
        cutoff = Lag.kr(lpf.max(baseFreq * 2.8), 0.45);
        osc = RLPF.ar(osc, cutoff, 0.2);
        osc = LPF.ar(osc, cutoff + 280);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        env = EnvGen.ar(Env.linen(atk, sus, 1.6, amp), doneAction: 0);
        osc = Pan2.ar(osc * env * 0.72, pan);
        ReplaceOut.ar(bus, osc)
}).add;