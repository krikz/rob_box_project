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

var renardoSynthDir = "__RENARDO_SCLANG_DIR__";
var renardoSynthDirPlaceholder = "__RENARDO_SCLANG_DIR_PLACEHOLDER__";
var startupSynths = ["strings", "wobblebass", "brass", "organ", "tb303", "pianovel"];
var customSynthDir = "/ws/custom_synthdefs";
var customSynths = ["warmpad", "retrobass", "supersawlead", "imperialbrass", "marchstrings", "strangerpulsepad", "strangerarp", "strangerbrass"];

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
    if(renardoSynthDir.isNil || { renardoSynthDir == renardoSynthDirPlaceholder } || { renardoSynthDir.isEmpty }) {
        "RENARDO_SCLANG_DIR is not set; skipping startup SynthDef preload.".postln;
    } {
        startupSynths.do({ |name|
            var path = renardoSynthDir ++ "/" ++ name ++ ".scd";
            ("Preloading SynthDef: " ++ name).postln;
            path.load;
            ("SynthDef preload ok: " ++ name).postln;
        });
    };
    customSynths.do({ |name|
        var path = customSynthDir ++ "/" ++ name ++ ".scd";
        ("Preloading custom SynthDef: " ++ name).postln;
        path.load;
        ("SynthDef preload ok: " ++ name).postln;
    });
    nil;
});

"sclang started. Waiting 3s for scsynth connection...".postln;
