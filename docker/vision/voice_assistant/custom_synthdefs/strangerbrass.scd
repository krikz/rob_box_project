SynthDef.new(\strangerbrass, {
        |vib=0.018, rate=4.0, sus=0.8, bus=0, atk=0.016, amp=1, freq=0, pan=0, lpf=1900|
        var baseFreq, detune, osc, env, cutoff, bite;
        baseFreq = In.kr(bus, 1);
        detune = [0.994, 1.0, 1.004, 1.5];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.24, mul: [0.2, 0.24, 0.18, 0.08]));
        osc = osc + Mix(Pulse.ar(baseFreq * [1.0, 2.0], [0.38, 0.32], [0.08, 0.04]));
        bite = EnvGen.ar(Env.perc(0.001, 0.05, 0.8, curve: -5), doneAction: 0);
        cutoff = Lag.kr(lpf.max(baseFreq * 3.2), 0.08);
        osc = BLowPass4.ar(osc, cutoff, 0.24);
        osc = osc + HPF.ar(osc * bite, baseFreq * 1.8, 0.08);
        osc = LeakDC.ar(osc);
        env = EnvGen.ar(Env.adsr(atk, 0.1, 0.72, sus * 0.4, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.8, pan);
        ReplaceOut.ar(bus, osc)
}).add;