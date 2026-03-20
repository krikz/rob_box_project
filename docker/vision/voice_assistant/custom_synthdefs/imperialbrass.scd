SynthDef.new(\imperialbrass, {
        |vib=0.015, rate=4.5, sus=0.6, bus=0, atk=0.012, amp=1, freq=0, pan=0, lpf=2400|
        var baseFreq, detune, osc, env, cutoff, bite;
        baseFreq = In.kr(bus, 1);
        detune = [0.992, 1.0, 1.006, 2.0];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.18, mul: [0.22, 0.26, 0.2, 0.06]));
        osc = osc + Mix(Pulse.ar(baseFreq * [1.0, 1.5], [0.42, 0.34], [0.1, 0.05]));
        bite = EnvGen.ar(Env.perc(0.001, 0.08, 1.0, curve: -6), doneAction: 0);
        cutoff = Lag.kr(lpf.max(baseFreq * 4), 0.06);
        osc = BLowPass4.ar(osc, cutoff, 0.26);
        osc = osc + HPF.ar(osc * bite, baseFreq * 2.4, 0.1);
        osc = LeakDC.ar(osc);
        env = EnvGen.ar(Env.adsr(atk, 0.12, 0.72, sus * 0.35, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.82, pan);
        ReplaceOut.ar(bus, osc)
}).add;