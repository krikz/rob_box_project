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
//
// 🔴 FIX (#1807): sclang теперь следит за serverRunning и САМ перезаливает
// всю палитру, если scsynth перезапустят отдельно от него — см. блок про
// loadedSynthPaths / watch-routine ниже, после регистрации OSCdef.

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
var customSynths = ["warmpad", "retrobass", "supersawlead", "imperialbrass", "marchstrings", "strangerpulsepad", "strangerarp", "strangerbrass", "masterlimiter", "masterfilter"];

// Мастер-лимитер: фиксированный node ID НИЖЕ 1000. renardo раздаёт ID
// начиная с 1001 и только вверх (ServerManager.nextnodeID: self.node = 1000),
// поэтому 999 не будет переиспользован ни при каком сценарии.
var masterLimiterNode = 999;

// ── #1807: история загрузок для само-восстановления палитры ─────────────────
//
// Живой инцидент: перезапуск ТОЛЬКО контейнера supercollider (docker restart
// supercollider, без voice-assistant) поднимает scsynth с пустой памятью —
// ни одного SynthDef. sclang при этом НЕ перезапускался, свой прелоад уже
// отработал один раз при собственном старте и второй раз запускать некому:
// сам sclang не замечает, что адресат на другом конце OSC-порта — уже другой
// процесс с нулевым состоянием. Симптом на роботе — makeSound, startSound,
// volume, play1, play2 (и вся остальная палитра) отсутствуют в scsynth, хотя
// оба контейнера healthy, инструменты-тулы рапортуют успех, а в логах чисто:
// FoxDot ОТПРАВЛЯЕТ ноты, просто их некому проиграть.
//
// Та же природа аварии, что и с группой 1 (commit 8c4079a7, «scsynth не
// создавал группу 1»), только не на одном /g_new, а на ~300 SynthDef сразу.
// Решение по тому же принципу: сторона, которая ПЕРЕЖИВАЕТ рестарт партнёра
// (здесь — sclang, там — сам scsynth), обязана сама восстановить состояние.
//
// loadedSynthPaths копит путь к КАЖДОМУ .scd, который sclang когда-либо
// грузил за время своей жизни — и стартовый прелоад (53 имени), и всё, что
// Python renardo досылает позже через /foxdot по ходу работы (полная
// палитра вырастает примерно до ~300 SynthDef — issue #1809). Это и есть
// материал для повторной заливки при обнаружении рестарта scsynth.
var loadedSynthPaths = Set.new;

// Перезаливает один .scd и ждёт подтверждения (Server.sync) — используется
// ТОЛЬКО при авто-реставре после рестарта scsynth (см. вотчер ниже).
// Стартовый прелоад ниже по файлу использует те же два вызова инлайново
// (это отдельные литеральные вызовы, не через эту функцию) — так исторически
// устроено с live-фикса 30.08, и это же закрыто отдельным тестом
// (test_synth_palette_is_preloaded.py::test_preload_waits_for_scsynth_after_each_load),
// который проверяет per-цикл наличие path.load/Server.default.sync в тексте
// файла. Дублирование тела небольшое (две строки) — цена за то, чтобы не
// ломать существующий инвариант лишним слоем косвенности.
var reloadKnownPath = { |path|
    path.load;
    Server.default.sync;
};

// Вооружает мастер-шину заново — после рестарта scsynth старый node 999
// исчез вместе со всем остальным состоянием сервера.
var armMasterFilter = {
    Server.default.sendMsg("/s_new", "masterfilter", masterLimiterNode, 1, 0);
    ("Master filter armed at tail of RootNode (node "
        ++ masterLimiterNode ++ ")").postln;
};

// serverRunning-вотчер (#1807): startAliveThread(0.5) ниже уже пингует
// scsynth раз в 0.5с и держит Server.default.serverRunning в актуальном
// состоянии. Флаги ниже нужны, чтобы отличить «сервер только что поднялся
// первый раз» (обычный холодный старт — прелоад и так случится штатно, ниже
// по файлу) от «сервер ПРОПАДАЛ и вернулся» (перезапуск контейнера — вот
// тут нужно восстановление palette).
var serverWasRunning = false;
var serverDropped = false;

// Connect to running scsynth via alive thread
Server.default.startAliveThread(0.5);

// Вотчер живёт всё время работы sclang, независимо от прелоада ниже.
// Порядок переходов false→true при холодном старте НЕ считается «дропом»
// (serverDropped стартует false и не выставляется, пока сервер не был уже
// живым хотя бы раз) — поэтому обычный старт не запускает лишнюю перезаливку.
fork {
    loop {
        var running = Server.default.serverRunning;
        if(serverWasRunning && running.not) {
            serverDropped = true;
            "[#1807] scsynth пропал — жду возврата, чтобы перезалить палитру.".postln;
        };
        if(serverDropped && running) {
            serverDropped = false;
            ("[#1807] scsynth вернулся — перезаливаю " ++ loadedSynthPaths.size
                ++ " SynthDef из истории сессии.").postln;
            // Копия множества: если Python пришлёт новый /foxdot прямо во
            // время перезаливки, do не должен споткнуться об изменение
            // коллекции, по которой итерируется.
            fork {
                loadedSynthPaths.copy.do({ |path| reloadKnownPath.value(path) });
                armMasterFilter.value;
                "[#1807] Палитра и masterfilter восстановлены после рестарта scsynth.".postln;
            };
        };
        serverWasRunning = running;
        0.5.wait;
    };
};

// Wait 3s for alive thread to detect scsynth, then register OSCdef
SystemClock.sched(3.0, {
    OSCdef.new(
        \foxdot,
        {|msg, time, addr, recvPort|
            var path = msg[1].asString;
            // #1807: запоминаем путь, чтобы при авто-реставре после
            // рестарта scsynth перезалить и то, что прислали не мы, а
            // Python renardo во время работы (сэмплы, доп. синты и т.п.).
            loadedSynthPaths.add(path);
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
                loadedSynthPaths.add(path);  // #1807: запоминаем для авто-реставра
                ("SynthDef in scsynth: " ++ name).postln;
            });
        };
        customSynths.do({ |name|
            var path = customSynthDir ++ "/" ++ name ++ ".scd";
            path.load;
            Server.default.sync;
            loadedSynthPaths.add(path);  // #1807: запоминаем для авто-реставра
            ("SynthDef in scsynth: " ++ name).postln;
        });
        ("SynthDef preload finished: "
            ++ (startupSynths.size + customSynths.size) ++ " defs").postln;

        // ── Мастер-шина: masterfilter (сменил masterlimiter 31.08) ───────
        // Лимитер глушил ВЕСЬ выход. Разбор на роботе: детектор
        // CheckBadValues до него видел чистый сигнал, детектор сразу после —
        // NaN. Порченые сэмплы уходили в ReplaceOut и убивали мастер-шину.
        // На чистой синусоиде он при этом работал, поэтому и продержался
        // так долго незамеченным — ломался только на живой музыке.
        //
        // masterfilter решает те же задачи (анти-алиасинг + потолок суммы),
        // но устроен так, что повторить ту аварию не может: NaN чистится на
        // входе, а вместо Compander/Limiter стоит tanh — функция без
        // состояния, которой нечему защёлкнуться. Подробности в
        // custom_synthdefs/masterfilter.scd.
        //
        // Лимитер оставлен в прелоаде намеренно: SynthDef на месте, если
        // понадобится сравнить поведение вживую.
        armMasterFilter.value;
    };

    nil;
});

"sclang started. Waiting 3s for scsynth connection...".postln;
