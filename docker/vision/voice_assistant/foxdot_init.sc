// foxdot_init.sc — FoxDot/Renardo SynthDef compiler for Rob Box
//
// This script runs inside the voice-assistant container alongside scsynth
// (which runs in the supercollider container).
//
// Pipeline:
//   Python renardo: sdef.add() → write .scd file → OSC /foxdot to sclang:57120
//   This sclang: receives /foxdot → path.load → compiles .scd → /d_recv → scsynth:57110
//
// scsynth runs at localhost:57110 (network_mode: host).

// Connect to running scsynth via alive thread
Server.default.startAliveThread(0.5);

// Wait 3s for alive thread to detect scsynth, then register OSCdef
SystemClock.sched(3.0, {
    OSCdef.new(
        \foxdot,
        {|msg, time, addr, recvPort|
            var path = msg[1].asString;
            path.load;
        },
        '/foxdot'
    );
    "FoxDot OSCdef registered. Ready to compile SynthDefs.".postln;
    ("Server running: " ++ Server.default.serverRunning).postln;
    nil;
});

"sclang started. Waiting 3s for scsynth connection...".postln;
