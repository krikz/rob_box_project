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
// 🔴 FIX (live 30.08): список должен покрывать ВСЮ палитру, которую промпт
// предлагает модели. Было 29 имён, промпт предлагал 48 — и модель регулярно
// выбирала те, которых тут нет. Живой лог scsynth за один вечер:
//     745  *** ERROR: SynthDef rhpiano not found
//      11  *** ERROR: SynthDef subbass not found
//     760  FAILURE IN SERVER /s_new SynthDef not found
// «Сыграй тему на рояле» → модель берёт rhpiano → /s_new отбит → в цепочке
// ноты остаётся только обвязка (startSound/lowPassFilter/volume/makeSound)
// без инструмента → ТИШИНА. При этом ноды считаются, CPU тратится, и
// execute_music_code рапортует «успешно»: exec() Python-кода прошёл, а про
// отказ scsynth он не знает.
// Инвариант «палитра промпта ⊆ этот список» закреплён тестом
// test_music_runtime_assets.py::test_foxdot_init_preloads_every_synth_the_prompts_advertise.
var startupSynths = [
    "ambi", "arpy", "bass", "bell", "blip", "brass",
    "creep", "cs80lead", "dirt", "donk", "dub", "ecello",
    "eoboe", "epiano", "faim", "feel", "flute", "fuzz",
    "gong", "hoover", "jbass", "kalimba", "karp", "keys",
    "marimba", "mhpad", "moogbass", "noise", "organ", "organ2",
    "orient", "pads", "pianovel", "pluck", "pulse", "quin",
    "rave", "rhpiano", "saw", "scatter", "sinepad", "sitar",
    "soprano", "space", "square", "steeldrum", "strings", "subbass",
    "tb303", "tubularbell", "varsaw", "viola", "wobblebass"
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
    // Прелоад идёт в Routine, и после КАЖДОГО файла ждём Server.sync.
    //
    // 🔴 FIX (live 30.08): раньше «SynthDef preload ok: X» печаталось сразу
    // после path.load — то есть подтверждало, что sclang СКОМПИЛИРОВАЛ
    // определение, а вовсе не что scsynth его принял. /d_recv уходит по UDP
    // без подтверждения, и при залпе часть просто терялась (тот же приём,
    // что описан выше про 188 /foxdot). Отсюда брались «SynthDef not found»
    // при формально успешном прелоаде, и validate_music_stack.py, который
    // грепает этот лог, давал ложный PASS.
    //
    // Server.sync возвращается только после того, как scsynth обработал всё
    // отправленное. Теперь «ok» означает «есть в scsynth», а не «отправлено».
    // Это же снимает риск от вдвое выросшего списка: залпа больше нет.
    fork {
        if(renardoSynthDir.isNil || { renardoSynthDir == renardoSynthDirPlaceholder } || { renardoSynthDir.isEmpty }) {
            "RENARDO_SCLANG_DIR is not set; skipping startup SynthDef preload.".postln;
        } {
            startupSynths.do({ |name|
                var path = renardoSynthDir ++ "/" ++ name ++ ".scd";
                path.load;
                Server.default.sync;
                ("SynthDef in scsynth: " ++ name).postln;
            });
        };
        customSynths.do({ |name|
            var path = customSynthDir ++ "/" ++ name ++ ".scd";
            path.load;
            Server.default.sync;
            ("SynthDef in scsynth: " ++ name).postln;
        });
        ("SynthDef preload finished: "
            ++ (startupSynths.size + customSynths.size) ++ " defs").postln;

        // ── Мастер-лимитер в хвост RootNode ──────────────────────────────
        // Ставим ПОСЛЕ прелоада, а не по фиксированной задержке в 2 с:
        // masterlimiter сам лежит в customSynths, и до Server.sync его в
        // scsynth могло ещё не быть.
        Server.default.sendMsg("/s_new", "masterlimiter", masterLimiterNode, 1, 0);
        ("Master limiter armed at tail of RootNode (node "
            ++ masterLimiterNode ++ ")").postln;
    };

    nil;
});

"sclang started. Waiting 3s for scsynth connection...".postln;
