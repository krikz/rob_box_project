SynthDef.new(\imperialbrass, {
        |vib=0.01, rate=4.0, sus=0.7, bus=0, atk=0.018, amp=1, freq=0, pan=0, lpf=2100|
        var baseFreq, detune, osc, env, cutoff, bite;
        baseFreq = In.kr(bus, 1);
        detune = [0.994, 1.0, 1.004, 2.0];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.22, mul: [0.2, 0.24, 0.18, 0.04]));
        osc = osc + Mix(Pulse.ar(baseFreq * [1.0, 2.0], [0.34, 0.28], [0.05, 0.02]));
        bite = EnvGen.ar(Env.perc(0.003, 0.045, 0.35, curve: -4), doneAction: 0);
        cutoff = Lag.kr(lpf.max(baseFreq * 3.2), 0.1);
        osc = BLowPass4.ar(osc, cutoff, 0.22);
        osc = osc + HPF.ar(osc * bite, baseFreq * 1.5, 0.04);
        osc = LeakDC.ar(osc);
        env = EnvGen.ar(Env.adsr(atk, 0.16, 0.76, sus * 0.45, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.74, pan);
        ReplaceOut.ar(bus, osc)
}).add;