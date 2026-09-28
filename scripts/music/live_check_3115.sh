#!/usr/bin/env bash
# live_check_3115.sh — живая проверка музыки на роботе одной командой (issue #3115).
#
# Что делает (по пунктам #3115):
#   1. fuzz-редирект #3008: короткая fuzz-басовая программа (oct=0) через
#      execute_music_code, запись ~10 с, grep «[#3008] /foxdot» в /tmp/sclang.log;
#   2. dur у play(): compose_music(name=tetris) — запись 15 с, в сыгранном
#      коде у d1/d2 dur=0.25;
#   3. режим мелодии: тот же трек и та же запись — amplify=[...] списком у
#      p-плеера, в логе supercollider нет ошибок;
#   4. compose_music(style="club") — запись 30 с, в логе supercollider нет
#      «not found» / «FAILURE IN SERVER»;
#   5. эталон core/reference_tracks/by_design_dj_dave.foxdot через
#      execute_music_code — запись 30 с.
#
# Всё складывается в ./live_check_3115_<UTC-время>/ : wav, коды, ответы MCP,
# docker logs --tail 50 обоих контейнеров, /tmp/sclang.log, summary.md.
#
# КАК ДОСТУЧАТЬСЯ ДО МУЗЫКИ БЕЗ LLM: подписанный вызов /mcp/execute изнутри
# контейнера voice-assistant (scripts/music/live_check_mcp_call.py, sender
# «harness», секрет /data/.mcp_token — см. rob_box_mcp_tools/mcp_auth.py).
# Это тот же путь, по которому ходит LLM, минус сама LLM.
#
# КАК ПИШЕТСЯ ЗВУК: jack_rec (пакет jackd2, уже стоит в образе supercollider,
# docker/vision/supercollider/Dockerfile) внутри контейнера supercollider
# снимает порты jack:out_1/jack:out_2 — ровно то, что scsynth отдаёт в
# system:playback (docker/vision/scripts/supercollider/start_supercollider.sh).
# Это цифровая копия выхода scsynth, не микрофон. Ничего на робот не ставится:
# если jack_rec/jack_capture в образе нет — запись честно FAIL, пункт не закрыт.
#
# ЧЕСТНОСТЬ (AGENTS.md): PASS в summary.md — только МЕХАНИЧЕСКИЕ проверки
# (строки в логах, dur/amplify в коде, «не тишина» в wav). На слух скрипт не
# решает ничего: в каждом пункте стоит «НА СЛУХ: требуется вердикт Шифу».
#
# Запуск:
#   на Vision Pi:        bash scripts/music/live_check_3115.sh
#   с другой машины:     SSHPASS=... bash scripts/music/live_check_3115.sh --host 10.1.1.21
#   посмотреть команды:  bash scripts/music/live_check_3115.sh --dry-run
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ── Параметры (дефолты — из репо) ──────────────────────────────────────────
# Vision Pi: 10.1.1.21, пользователь ros2 (L-Architecture Audit.yml,
# .github/workflows/scripts/e2e_voice_test.sh). Пусто = работаем локально.
HOST="${LIVE_CHECK_HOST:-}"
SSH_USER="${LIVE_CHECK_USER:-ros2}"
# container_name из docker/vision/docker-compose.yaml.
VA_CONTAINER="${VA_CONTAINER:-voice-assistant}"
SC_CONTAINER="${SC_CONTAINER:-supercollider}"
# start_voice_assistant.sh: sclang -i none ... > /tmp/sclang.log (ADR-0129 §8).
SCLANG_LOG="${SCLANG_LOG:-/tmp/sclang.log}"
# Порты scsynth в JACK (start_supercollider.sh: jack_connect jack:out_1 ...).
JACK_PORTS="${JACK_PORTS:-jack:out_1 jack:out_2}"
ROS_SETUP="${ROS_SETUP:-source /opt/ros/humble/setup.bash; source /ws/install/setup.bash}"
MELODY_NAME="${MELODY_NAME:-tetris}"
REFERENCE_FILE="${REFERENCE_FILE:-${REPO_ROOT}/src/rob_box_mcp_tools/rob_box_mcp_tools/core/reference_tracks/by_design_dj_dave.foxdot}"
FUZZ_CODE="${FUZZ_CODE:-Clock.bpm = 110
p1 >> fuzz([40, 40, 43, 45, 40, 40, 47, 45], dur=1/2, oct=0, root=0, scale=Scale.chromatic, amp=0.6)}"
REC_FUZZ="${REC_FUZZ:-10}"
REC_CLASSIC="${REC_CLASSIC:-15}"
REC_CLUB="${REC_CLUB:-30}"
REC_REFERENCE="${REC_REFERENCE:-30}"
# Запас записи: MCP-вызов (особенно compose_music) стартует звук не мгновенно.
REC_PAD="${REC_PAD:-5}"
MCP_TIMEOUT="${MCP_TIMEOUT:-90}"
DRY_RUN=0
LOCAL_ONLY=0
ONLY=""
TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT=""

HELPER="${SCRIPT_DIR}/live_check_mcp_call.py"
ASSERT="${SCRIPT_DIR}/live_check_3115_assert.py"

usage() {
    cat <<EOF
Usage: $0 [options]
  --host HOST        Vision Pi по SSH (напр. 10.1.1.21); без него — локально на Pi
  --user USER        SSH-пользователь [${SSH_USER}] (пароль: env SSHPASS, если задан)
  --va NAME          контейнер voice-assistant [${VA_CONTAINER}]
  --sc NAME          контейнер supercollider [${SC_CONTAINER}]
  --sclang-log PATH  лог sclang внутри voice-assistant [${SCLANG_LOG}]
  --melody NAME      мелодия для пунктов 2-3 [${MELODY_NAME}]
  --reference FILE   эталон для пункта 5 [${REFERENCE_FILE#"${REPO_ROOT}"/}]
  --only N[,N]       только эти пункты (1..5; 2 и 3 идут одним прогоном)
  --out DIR          каталог доказательств [./live_check_3115_<UTC>]
  --dry-run          только напечатать команды, ничего не выполнять
  --local            (служебное) выполнить на этой машине, даже если задан --host
EOF
}

PASSTHRU=()
while [ $# -gt 0 ]; do
    case "$1" in
        --host) HOST="$2"; shift 2 ;;
        --user) SSH_USER="$2"; shift 2 ;;
        --va) VA_CONTAINER="$2"; PASSTHRU+=(--va "$2"); shift 2 ;;
        --sc) SC_CONTAINER="$2"; PASSTHRU+=(--sc "$2"); shift 2 ;;
        --sclang-log) SCLANG_LOG="$2"; PASSTHRU+=(--sclang-log "$2"); shift 2 ;;
        --melody) MELODY_NAME="$2"; PASSTHRU+=(--melody "$2"); shift 2 ;;
        --reference) REFERENCE_FILE="$2"; shift 2 ;;
        --only) ONLY="$2"; PASSTHRU+=(--only "$2"); shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        --local) LOCAL_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Неизвестный аргумент: $1" >&2; usage >&2; exit 64 ;;
    esac
done
OUT="${OUT:-./live_check_3115_${TS}}"

want() {  # want N — выбран ли пункт N
    [ -z "$ONLY" ] && return 0
    case ",${ONLY}," in *",$1,"*) return 0 ;; esac
    return 1
}

# ── Режим SSH: везём бандл на Pi, запускаем там, забираем каталог ──────────
if [ -n "$HOST" ] && [ "$LOCAL_ONLY" = "0" ]; then
    SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10)
    if [ -n "${SSHPASS:-}" ]; then SSHP=(sshpass -e); else SSHP=(); fi
    REMOTE_DIR="/tmp/live_check_3115_bundle"
    REMOTE_OUT="${REMOTE_DIR}/$(basename "$OUT")"
    TARGET="${SSH_USER}@${HOST}"
    DRY_FLAG=(); [ "$DRY_RUN" = "1" ] && DRY_FLAG=(--dry-run)
    remote_steps=(
        "${SSHP[*]} ssh ${SSH_OPTS[*]} ${TARGET} mkdir -p ${REMOTE_DIR}"
        "${SSHP[*]} scp ${SSH_OPTS[*]} $0 ${HELPER} ${ASSERT} ${REFERENCE_FILE} ${TARGET}:${REMOTE_DIR}/"
        "${SSHP[*]} ssh ${SSH_OPTS[*]} ${TARGET} bash ${REMOTE_DIR}/$(basename "$0") --local --out ${REMOTE_OUT} --reference ${REMOTE_DIR}/$(basename "$REFERENCE_FILE") ${PASSTHRU[*]} ${DRY_FLAG[*]}"
        "${SSHP[*]} scp -r ${SSH_OPTS[*]} ${TARGET}:${REMOTE_OUT} $(dirname "$OUT")/"
    )
    if [ "$DRY_RUN" = "1" ]; then
        echo "# SSH-режим: ${TARGET}"
        for s in "${remote_steps[@]}"; do echo "DRY: $s"; done
        echo "# --- план на Vision Pi (то же, что выполнит удалённый --local) ---"
        exec bash "$0" --local --dry-run --out "$REMOTE_OUT" --reference "$REFERENCE_FILE" "${PASSTHRU[@]}"
    fi
    rc=0
    for s in "${remote_steps[@]}"; do
        echo "+ $s" >&2
        # Шаги собраны из наших же параметров; eval нужен, чтобы пустой SSHP не давал ''.
        eval "$s" || { rc=$?; echo "Шаг упал (rc=$rc): $s" >&2; [ "$s" = "${remote_steps[2]}" ] || exit "$rc"; }
    done
    echo "Доказательства: $(dirname "$OUT")/$(basename "$OUT")"
    exit "$rc"
fi

# ── Локальный режим (на Vision Pi) ──────────────────────────────────────────
if [ "$DRY_RUN" = "0" ]; then
    mkdir -p "$OUT" || exit 1
    OUT="$(cd "$OUT" && pwd)"
fi
CMDLOG="${OUT}/commands.log"
SUMMARY_ROWS=()
FAILS=0
PASSES=0

run() {  # run 'shell command' — всё, что трогает робота, идёт через неё
    if [ "$DRY_RUN" = "1" ]; then
        echo "DRY: $1"
        return 0
    fi
    printf '%s + %s\n' "$(date -u +%H:%M:%S)" "$1" >> "$CMDLOG"
    bash -c "$1"
}

check_assert() {  # check_assert <пункт> <название> <assert-подкоманда> args...
    local item="$1" name="$2"; shift 2
    if [ "$DRY_RUN" = "1" ]; then
        echo "DRY: [assert п.${item}] ${name}: python3 ${ASSERT} $*"
        return 0
    fi
    local line
    line="$(python3 "$ASSERT" "$@" 2>&1 | tail -1)"
    case "$line" in
        PASS:*) PASSES=$((PASSES + 1)) ;;
        *) FAILS=$((FAILS + 1)) ;;
    esac
    SUMMARY_ROWS+=("| ${item} | ${name} | ${line//|/\\|} |")
    echo "[п.${item}] ${name}: ${line}"
}

note_fail() {  # note_fail <пункт> <название> <причина> — FAIL без assert-скрипта
    FAILS=$((FAILS + 1))
    SUMMARY_ROWS+=("| $1 | $2 | FAIL: ${3//|/\\|} |")
    echo "[п.$1] $2: FAIL: $3"
}

b64() { printf '%s' "$1" | base64 | tr -d '\n'; }
json_params() {  # json_params key value [key value ...] — значения строками/числами как есть в JSON
    python3 -c 'import json,sys; a=sys.argv[1:]; print(json.dumps({a[i]: json.loads(a[i+1]) for i in range(0,len(a),2)}, ensure_ascii=False))' "$@"
}

# mcp_call <tool> <params_json> <out_prefix>: пишет <out>.request.json, <out>.json, <out>.stderr
mcp_call() {
    local tool="$1" params="$2" out="$3"
    if [ "$DRY_RUN" = "0" ]; then printf '%s\n' "$params" > "${out}.request.json"; fi
    run "docker exec -i ${VA_CONTAINER} bash -lc '${ROS_SETUP}; python3 - ${tool} $(b64 "$params") ${MCP_TIMEOUT}' < '${HELPER}' > '${out}.json' 2> '${out}.stderr'"
}

REC_TOOL=""
REC_PID=""
REC_NAME=""
record_start() {  # record_start <секунды> <имя.wav>
    local secs=$(( $1 + REC_PAD )) name="$2"
    REC_NAME="$name"
    local cmd
    case "$REC_TOOL" in
        jack_rec) cmd="docker exec ${SC_CONTAINER} jack_rec -f /tmp/${name} -d ${secs} -b 16 ${JACK_PORTS}" ;;
        jack_capture)
            local ports="" p
            for p in $JACK_PORTS; do ports="$ports -p $p"; done
            cmd="docker exec ${SC_CONTAINER} jack_capture -d ${secs} -b 16${ports} -f wav /tmp/${name}" ;;
        *) REC_PID=""; return 1 ;;
    esac
    if [ "$DRY_RUN" = "1" ]; then echo "DRY: ${cmd} &"; REC_PID=""; return 0; fi
    printf '%s + %s &\n' "$(date -u +%H:%M:%S)" "$cmd" >> "$CMDLOG"
    bash -c "$cmd" > "${OUT}/.rec_${name}.log" 2>&1 &
    REC_PID=$!
}
record_finish() {  # record_finish <каталог>
    local dir="$1"
    [ -n "$REC_TOOL" ] || return 1
    if [ -n "$REC_PID" ]; then wait "$REC_PID"; fi
    run "docker cp ${SC_CONTAINER}:/tmp/${REC_NAME} '${dir}/${REC_NAME}'"
    run "docker exec ${SC_CONTAINER} rm -f /tmp/${REC_NAME}"
    if [ "$DRY_RUN" = "0" ] && [ -f "${OUT}/.rec_${REC_NAME}.log" ]; then
        mv "${OUT}/.rec_${REC_NAME}.log" "${dir}/${REC_NAME}.recorder.log"
    fi
}

collect_logs() {  # collect_logs <каталог> <since RFC3339>
    local dir="$1" since="$2" c
    for c in "$VA_CONTAINER" "$SC_CONTAINER"; do
        run "docker logs --tail 50 ${c} > '${dir}/${c}.tail50.log' 2>&1"
        run "docker logs --since '${since}' ${c} > '${dir}/${c}.since_check.log' 2>&1"
    done
    run "docker cp ${VA_CONTAINER}:${SCLANG_LOG} '${dir}/sclang.log'"
}

stop_music() {  # stop_music <out_prefix>
    mcp_call stop_music '{}' "$1"
}

now_utc() {
    if [ "$DRY_RUN" = "1" ]; then echo '<UTC>'; else date -u +%Y-%m-%dT%H:%M:%SZ; fi
}

# play_and_record <каталог> <tool> <params_json> <секунды> <wav>
play_and_record() {
    local dir="$1" tool="$2" params="$3" secs="$4" wav="$5"
    run "mkdir -p '${dir}'"
    stop_music "${dir}/stop_before"
    SINCE="$(now_utc)"
    run "sleep 2"
    record_start "$secs" "$wav"
    mcp_call "$tool" "$params" "${dir}/mcp_result"
    # Даём доиграть до конца записи (запись идёт в фоне).
    record_finish "$dir"
    collect_logs "$dir" "$SINCE"
    stop_music "${dir}/stop_after"
}

wav_assert() {  # wav_assert <пункт> <каталог> <wav>
    if [ -z "$REC_TOOL" ]; then
        note_fail "$1" "запись звука" "нет jack_rec/jack_capture в контейнере ${SC_CONTAINER} — пункт НЕ закрыт (AGENTS.md: без записи не закрываем)"
    else
        check_assert "$1" "запись не тишина" wav "$2/$3"
    fi
}

# ── Предпроверки + окружение ────────────────────────────────────────────────
echo "# live_check_3115: out=${OUT}"
if [ "$DRY_RUN" = "0" ]; then
    for f in "$HELPER" "$ASSERT"; do [ -f "$f" ] || { echo "нет $f" >&2; exit 1; }; done
    {
        echo "utc_start: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "host: $(hostname)"
        echo "script_git: $(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo 'нет git (бандл по SSH)')"
    } > "${OUT}/env.txt"
fi
run "docker inspect -f '{{.Name}} image={{.Config.Image}} id={{.Image}} started={{.State.StartedAt}} status={{.State.Status}}' ${VA_CONTAINER} ${SC_CONTAINER} >> '${OUT}/env.txt' 2>&1"
run "docker exec ${SC_CONTAINER} jack_lsp -c > '${OUT}/jack_lsp.txt' 2>&1"
run "docker exec ${VA_CONTAINER} grep -c 'SynthDef preload finished' ${SCLANG_LOG} > '${OUT}/sclang_preload_count.txt' 2>&1"

if [ "$DRY_RUN" = "1" ]; then
    echo "DRY: docker exec ${SC_CONTAINER} sh -c 'command -v jack_rec || command -v jack_capture'"
    REC_TOOL="jack_rec"
else
    probe="$(docker exec "$SC_CONTAINER" sh -c 'command -v jack_rec || command -v jack_capture' 2>/dev/null | head -1)"
    REC_TOOL="$(basename "${probe:-}" 2>/dev/null)"
    echo "recorder: ${REC_TOOL:-НЕТ}" | tee -a "${OUT}/env.txt"
    if [ -n "$REC_TOOL" ]; then
        for p in $JACK_PORTS; do
            grep -q "^${p}$" "${OUT}/jack_lsp.txt" || echo "WARN: JACK-порт ${p} не найден (см. jack_lsp.txt) — запись может быть пустой" | tee -a "${OUT}/env.txt"
        done
    fi
fi

# ── 1. fuzz-редирект #3008 ──────────────────────────────────────────────────
if want 1; then
    D="${OUT}/check1_fuzz"
    echo "## п.1 fuzz-редирект #3008 (запись ${REC_FUZZ} с)"
    play_and_record "$D" execute_music_code "$(json_params code "$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$FUZZ_CODE")" pattern_name '"p1"' segments 8)" "$REC_FUZZ" fuzz_oct0.wav
    check_assert 1 "sclang стартовал (прелоад)" grep "SynthDef preload finished" "$D/sclang.log"
    check_assert 1 "sclang без синтакс-ошибок" absent "ERROR: syntax error" "$D/sclang.log"
    check_assert 1 "редирект [#3008] в sclang.log" grep "[#3008] /foxdot" "$D/sclang.log"
    check_assert 1 "execute_music_code ok, код" code "$D/mcp_result.json" "$D/code.foxdot"
    wav_assert 1 "$D" fuzz_oct0.wav
fi

# ── 2+3. dur у play() и режим мелодии: один трек, одна запись ───────────────
if want 2 || want 3; then
    D="${OUT}/check2_3_classic_${MELODY_NAME}"
    echo "## п.2-3 compose_music(name=${MELODY_NAME}) (запись ${REC_CLASSIC} с)"
    play_and_record "$D" compose_music "$(json_params name "\"${MELODY_NAME}\"")" "$REC_CLASSIC" "classic_${MELODY_NAME}.wav"
    check_assert 2 "compose_music ok, код" code "$D/mcp_result.json" "$D/code.foxdot"
    want 2 && check_assert 2 "d1/d2 dur=0.25" drums "$D/code.foxdot"
    want 3 && check_assert 3 "amplify=[...] у лида" melody "$D/code.foxdot"
    want 3 && check_assert 3 "лог supercollider без ошибок" logerr "$D/${SC_CONTAINER}.since_check.log"
    wav_assert 2 "$D" "classic_${MELODY_NAME}.wav"
fi

# ── 4. style="club" ─────────────────────────────────────────────────────────
if want 4; then
    D="${OUT}/check4_club"
    echo "## п.4 compose_music(style=club) (запись ${REC_CLUB} с)"
    play_and_record "$D" compose_music '{"style": "club"}' "$REC_CLUB" club.wav
    check_assert 4 "compose_music(club) ok, код" code "$D/mcp_result.json" "$D/code.foxdot"
    check_assert 4 "supercollider: нет not found/FAILURE" logerr "$D/${SC_CONTAINER}.since_check.log"
    wav_assert 4 "$D" club.wav
fi

# ── 5. эталон by_design_dj_dave через execute_music_code ────────────────────
if want 5; then
    D="${OUT}/check5_reference"
    echo "## п.5 эталон $(basename "$REFERENCE_FILE") (запись ${REC_REFERENCE} с)"
    if [ "$DRY_RUN" = "0" ] && [ ! -f "$REFERENCE_FILE" ]; then
        note_fail 5 "эталон" "нет файла ${REFERENCE_FILE}"
    else
        ref_code='""'
        [ -f "$REFERENCE_FILE" ] && ref_code="$(python3 -c 'import json,sys; print(json.dumps(open(sys.argv[1],encoding="utf-8").read()))' "$REFERENCE_FILE")"
        play_and_record "$D" execute_music_code "$(json_params code "$ref_code" pattern_name '"composition"')" "$REC_REFERENCE" reference_by_design.wav
        check_assert 5 "execute_music_code ok, код" code "$D/mcp_result.json" "$D/code.foxdot"
        check_assert 5 "supercollider: нет not found/FAILURE" logerr "$D/${SC_CONTAINER}.since_check.log"
        wav_assert 5 "$D" reference_by_design.wav
    fi
fi

if [ "$DRY_RUN" = "1" ]; then
    echo "# Каталог доказательств (план):"
    echo "#   ${OUT}/summary.md, commands.log, env.txt, jack_lsp.txt"
    want 1 && echo "#   ${OUT}/check1_fuzz/{fuzz_oct0.wav,code.foxdot,mcp_result.json,sclang.log,${VA_CONTAINER}.tail50.log,${SC_CONTAINER}.tail50.log}"
    { want 2 || want 3; } && echo "#   ${OUT}/check2_3_classic_${MELODY_NAME}/{classic_${MELODY_NAME}.wav,code.foxdot,mcp_result.json,sclang.log,${VA_CONTAINER}.tail50.log,${SC_CONTAINER}.tail50.log}"
    want 4 && echo "#   ${OUT}/check4_club/{club.wav,code.foxdot,mcp_result.json,sclang.log,${VA_CONTAINER}.tail50.log,${SC_CONTAINER}.tail50.log}"
    want 5 && echo "#   ${OUT}/check5_reference/{reference_by_design.wav,code.foxdot,mcp_result.json,sclang.log,${VA_CONTAINER}.tail50.log,${SC_CONTAINER}.tail50.log}"
    exit 0
fi

# ── summary.md ──────────────────────────────────────────────────────────────
{
    echo "# Живая проверка #3115 — $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo
    echo "Скрипт: \`scripts/music/live_check_3115.sh\`. Окружение: \`env.txt\`, команды: \`commands.log\`."
    echo
    echo "**Механика:** ${PASSES} PASS / ${FAILS} FAIL. Это только логи/код/«не тишина» — НЕ качество звука."
    echo
    echo "| п. | проверка | результат |"
    echo "|---|---|---|"
    for r in "${SUMMARY_ROWS[@]}"; do echo "$r"; done
    echo
    echo "## На слух"
    want 1 && echo "- п.1 fuzz oct=0 без «пердения» — \`check1_fuzz/fuzz_oct0.wav\` — **НА СЛУХ: требуется вердикт Шифу**"
    want 2 && echo "- п.2 ударные 16-ми, дакинг баса с бочкой — \`check2_3_classic_${MELODY_NAME}/classic_${MELODY_NAME}.wav\` — **НА СЛУХ: требуется вердикт Шифу**"
    want 3 && echo "- п.3 акценты/легато/лид громче всех — та же запись — **НА СЛУХ: требуется вердикт Шифу**"
    want 4 && echo "- п.4 club: пампинг, hpf-подъём перед дропом — \`check4_club/club.wav\` — **НА СЛУХ: требуется вердикт Шифу**"
    want 5 && echo "- п.5 эталон vs club — \`check5_reference/reference_by_design.wav\` против \`check4_club/club.wav\` — **НА СЛУХ: требуется вердикт Шифу**"
    echo
    echo "Запись — цифровой выход scsynth (JACK-порты ${JACK_PORTS}, ${REC_TOOL:-рекордера нет}), 16 кГц; динамик и комнату она не включает."
} > "${OUT}/summary.md"

echo
cat "${OUT}/summary.md"
echo
echo "Доказательства: ${OUT}"
[ "$FAILS" -eq 0 ]
