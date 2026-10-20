SynthDef.new(\retrobass, {
        |sus=1, bus=0, atk=0.01, amp=1, freq=0, pan=0, lpf=900, rate=0.04|
        var baseFreq, osc, env, cutoff;
        baseFreq = In.kr(bus, 1);
        osc = Mix([
            Saw.ar(baseFreq, 0.22),
            Pulse.ar(baseFreq * 0.5, 0.48, 0.18),
            SinOsc.ar(baseFreq, mul: 0.12)
        ]);
        cutoff = Lag.kr(lpf.max(baseFreq * 2), rate);
        osc = RLPF.ar(osc, cutoff, 0.28);
        env = EnvGen.ar(Env.perc(atk, sus, amp, curve: -4), doneAction: 0);
        osc = Pan2.ar(osc * env * 0.9, pan);
        ReplaceOut.ar(bus, osc)
}).add;