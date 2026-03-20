SynthDef.new(\strangerarp, {
        |vib=0.02, rate=5.5, sus=0.45, bus=0, atk=0.01, amp=1, freq=0, pan=0, lpf=1600|
        var baseFreq, detune, osc, env, cutoff, drive;
        baseFreq = In.kr(bus, 1);
        detune = [0.991, 0.997, 1.0, 1.006];
        osc = Mix(VarSaw.ar(baseFreq * detune, width: 0.76, mul: [0.18, 0.16, 0.18, 0.14]));
        osc = osc + Pulse.ar(baseFreq * 2, 0.44, 0.05);
        drive = (osc * 2.2).tanh;
        cutoff = Lag.kr(lpf.max(baseFreq * 3.5), 0.1);
        osc = RLPF.ar(drive, cutoff, 0.16);
        osc = LeakDC.ar(osc);
        env = EnvGen.ar(Env.adsr(atk, 0.08, 0.68, sus * 0.55, amp), doneAction: 0);
        osc = osc * (1 + SinOsc.kr(rate, 0, vib, 0));
        osc = Pan2.ar(osc * env * 0.78, pan);
        ReplaceOut.ar(bus, osc)
}).add;