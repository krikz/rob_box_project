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
// live 12.08: расширенный прелоад — renardo-init залпом шлёт 188 /foxdot,
// UDP-буфер sclang рвётся (drops >500), часть SynthDef-ов (pads, bass, karp...)
// не доезжает до scsynth → "SynthDef not found" → тишина. Прелоад на старте
// идёт последовательно внутри sclang — потерь нет.
var startupSynths = [
    "strings", "wobblebass", "brass", "organ", "tb303", "pianovel",
    "pads", "bass", "bell", "blip", "fuzz", "gong", "karp",
    "dub", "pluck", "space", "epiano", "saw", "varsaw", "square",
    "ambi", "faim", "marimba", "sitar", "viola", "noise",
    "scatter", "orient", "creep"
];
var customSynthDir = "/ws/custom_synthdefs";
var customSynths = ["warmpad", "retrobass", "supersawlead", "imperialbrass", "marchstrings", "strangerpulsepad", "strangerarp", "strangerbrass", "masterlimiter"];

// Мастер-лимитер: фиксированный node ID НИЖЕ 1000. renardo раздаёт ID
// начиная с 1001 и только вверх (ServerManager.nextnodeID: self.node = 1000),
// поэтому 999 не будет переиспользован ни при каком сценарии.
var masterLimiterNode = 999;

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

    // ── Мастер-лимитер в хвост RootNode ──────────────────────────────────
    // /s_new <def> <id> <addAction=1:addToTail> <target=0:RootNode>
    // Renardo играет в группе 1 (она создаётся addToHead node 0), значит
    // хвост node 0 — всегда ПОСЛЕ всей музыки. `/g_freeAll 1` в
    // Clock.clear() / stop_all() эту ноду не трогает.
    //
    // Одноразовая установка (без периодического re-arm): рестарт scsynth
    // и так стирает ВСЕ SynthDef-ы, преложенные этим sclang, поэтому он
    // уже требует перезапуска voice-assistant. Отдельный watchdog именно
    // для лимитера ничего бы не спас.
    SystemClock.sched(2.0, {
        Server.default.sendMsg("/s_new", "masterlimiter", masterLimiterNode, 1, 0);
        ("Master limiter armed at tail of RootNode (node "
            ++ masterLimiterNode ++ ")").postln;
        nil;
    });

    nil;
});

"sclang started. Waiting 3s for scsynth connection...".postln;
