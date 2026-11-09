#!/usr/bin/env bash
# Регресс-тест scripts/hil/hil.sh и замка HIL в деплое (эпик #4371).
#
# Ни робота, ни docker не нужно: ssh и docker подменены фейками в PATH.
# Фейковый ssh выполняет «удалённую» команду локально (в том же фейковом docker),
# поэтому проверяется настоящий путь hil.sh → ssh → remote_pi.sh → docker.
#
# Что проверяется:
#   1. bash -n всех скриптов HIL;
#   2. hil.sh не трогает боевые пути: HIL_MAPS_DIR = ./maps Main Pi, /maps, /data,
#      относительный путь — отказ ДО первого ssh; в командах `on` нет /data и
#      боевого ./maps, карта rtabmap — HIL_MAPS_DIR;
#   3. on: замок → stop ros2-control/lslidar → stop oak-d/ceiling-camera →
#      пересоздание шести сервисов с compose.hil.yaml и --no-deps → сим; ID робота
#      из .env Main Pi; Zenoh-endpoint'ы с #iface;
#   4. on при существующем замке — отказ, моторы не трогаются;
#   5. on, если в графе два /controller_manager — сим гасится, выход 1;
#   6. off: сим гаснет ДО возврата ros2-control; замок снимается последним;
#   7. scripts/deploy/check_hil_lock.sh: свободно / замок / нет ответа;
#   8. реестр карт и точек (#4392): on снимает снапшот боевого maps и пересоздаёт
#      voice-assistant на Sound Pi с WAYPOINT_DB_PATH=/data/hil/…; off возвращает боевой
#      и сверяет maps со снапшотом (изменился — выход 3 и diff); плохой HIL_WAYPOINT_DB —
#      отказ до ssh; HIL_PROD_TOOLS_DB = rob_box_core.tools_db.TOOLS_DB_DEFAULT;
#      шаг замка в деплое стоит до первой остановки контейнеров, update_and_restart.sh
#      проверяет замок до `down`.
#
# Запуск: bash scripts/hil/tests/test_hil.sh

# check «имя» '<выражение>' вычисляет выражение через eval в момент проверки:
# одинарные кавычки и «неиспользуемые» L_*/W_* здесь намеренные.
# shellcheck disable=SC2016,SC2034
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HIL="$REPO_ROOT/scripts/hil/hil.sh"
LOCKCHK="$REPO_ROOT/scripts/deploy/check_hil_lock.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }
check() { if eval "$2"; then ok "$1"; else ko "$1"; fi; }

# ── 1. синтаксис ──────────────────────────────────────────────────────────
for f in scripts/hil/hil.sh scripts/hil/remote_pi.sh scripts/hil/hil.conf \
         scripts/hil/video/fix_crlf.sh scripts/hil/video/runvideo.sh scripts/hil/video/runcoverage.sh scripts/hil/video/sandbox_prep.sh \
         scripts/hil/building/run_building.sh scripts/hil/building/building_remote.sh \
         scripts/deploy/check_hil_lock.sh docker/sim/hil_run.sh docker/main/update_and_restart.sh; do
    if bash -n "$REPO_ROOT/$f"; then ok "bash -n $f"; else ko "bash -n $f"; fi
done

# ── фейки ─────────────────────────────────────────────────────────────────
mkdir -p "$WORK/bin" "$WORK/state"
cat > "$WORK/bin/ssh" <<'EOF'
#!/usr/bin/env bash
# Фейковый ssh: пропускает -o опции, пишет host|cmd в лог и выполняет cmd локально.
while [ "$#" -gt 0 ]; do
    case "$1" in -o) shift 2 ;; *) break ;; esac
done
host="$1"; shift
echo "$host|$*" >> "$FAKE_SSH_LOG"
[ -n "${FAKE_SSH_DOWN:-}" ] && [ "$host" = "$FAKE_SSH_DOWN" ] && exit 255
exec bash -c "$*"
EOF
cat > "$WORK/bin/docker" <<'EOF'
#!/usr/bin/env bash
# Фейковый docker: состояние контейнеров — файлы $FAKE_STATE/<имя> (status) и <имя>.simt.
echo "docker $*" >> "$FAKE_DOCKER_LOG"
S="$FAKE_STATE"
st() { cat "$S/$1" 2>/dev/null || echo "${FAKE_DEFAULT_STATUS:-running}"; }
last() { local a; for a; do :; done; printf '%s' "$a"; }
case "$1" in
    stop) echo exited > "$S/$(last "$@")" ;;
    inspect)
        name=$(last "$@")
        case "$*" in
            *"State.Status}} {{.State.ExitCode"*)
                [ -f "$S/$name" ] || { [ -n "${FAKE_INSPECT_BLANK:-}" ] && echo; exit 1; }; echo "$(cat "$S/$name") 0" ;;
            *"State.Status"*) echo "$(st "$name")" ;;
            *"State.Running"*) [ "$(st "$name")" = running ] && echo true || echo false ;;
            *"Config.Env"*) [ -f "$S/$name.simt" ] && echo "USE_SIM_TIME=$(cat "$S/$name.simt")"
                            [ -f "$S/$name.wpdb" ] && echo "WAYPOINT_DB_PATH=$(cat "$S/$name.wpdb")"; echo PATH=/x ;;
        esac ;;
    image) exit 0 ;;
    logs) echo "fake logs" ;;
    exec)
        case "$*" in
            *"python3 -c"*)  # maps-dump: боевой maps — файл $FAKE_STATE/prod_maps; нет файла — sqlite падает
                [ "$(last "$@")" = "${FAKE_PROD_DB:-/data/voice_memory.db}" ] || exit 1
                [ -f "$S/prod_maps" ] || exit 1
                cat "$S/prod_maps" ;;
            *) [ -n "${FAKE_ZENOH_LOG:-}" ] && echo "INFO zenoh::net::runtime: Using ZID: 1a2b3c"; echo "${FAKE_CM:-1}"; echo /controller_manager ;;
        esac ;;
    ps)
        case "$*" in
            *working_dir=*) for s in ros2-control lslidar robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2 oak-d ceiling-camera voice-assistant; do
                                echo "$s ghcr.io/krikz/rob_box:$s-humble-dev-abc"; done ;;
        esac ;;
    compose)
        shift
        hil=0; sim=0; snd=0; verb=""; svcs=()
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -f) case "$2" in *main.hil.yaml) hil=1 ;; *sound.hil.yaml) snd=1 ;; *compose.hil.yaml) sim=1 ;; esac; shift 2 ;;
                -p) shift 2 ;;
                --pull) shift 2 ;;
                up | down) verb="$1"; shift ;;
                -*) shift ;;
                *) svcs+=("$1"); shift ;;
            esac
        done
        if [ "$sim" = 1 ]; then
            if [ "$verb" = up ]; then echo running > "$S/${SIM_PREFIX}-gazebo"; else rm -f "$S/${SIM_PREFIX}-gazebo"; fi
        elif [ "$verb" = up ]; then
            for s in "${svcs[@]}"; do
                if [ -f "$S/$s.broken" ]; then echo restarting > "$S/$s"; else echo running > "$S/$s"; fi
                if [ "$hil" = 1 ]; then echo true > "$S/$s.simt"; else rm -f "$S/$s.simt"; fi
                if [ "$snd" = 1 ]; then printf '%s\n' "${HIL_WAYPOINT_DB:?compose.hil.yaml Sound без HIL_WAYPOINT_DB}" > "$S/$s.wpdb"; else rm -f "$S/$s.wpdb"; fi
            done
        fi ;;
esac
exit 0
EOF
chmod +x "$WORK/bin/ssh" "$WORK/bin/docker"

mkdir -p "$WORK/main" "$WORK/vision" "$WORK/sound" "$WORK/sbx/docker/sim"
echo "ROBOT_ID=RBXTEST01" > "$WORK/main/.env"
touch "$WORK/sbx/docker/sim/compose.hil.yaml"

PROD_MAPS='1|aaaa-1111|мастерская|1793000000.0|1
2|bbbb-2222|хил-офис|1793100000.0|0'
reset_state() {
    rm -rf "$WORK/state" "$WORK/remote" "$WORK/tmp" "$WORK/sndstate"
    mkdir -p "$WORK/state" "$WORK/tmp"
    printf '%s\n' "$PROD_MAPS" > "$WORK/state/prod_maps"
    : > "$WORK/ssh.log"; : > "$WORK/docker.log"
}

run_hil() {  # run_hil "<cmd> [аргументы]" [VAR=val ...]
    local cmd=()
    read -r -a cmd <<<"$1"  # «restart rtabmap» — команда и аргументы
    shift
    env PATH="$WORK/bin:$PATH" FAKE_SSH_LOG="$WORK/ssh.log" FAKE_DOCKER_LOG="$WORK/docker.log" \
        FAKE_STATE="$WORK/state" SIM_PREFIX=t-hil \
        HIL_MAIN_HOST=main HIL_VISION_HOST=vision HIL_SOUND_HOST=sound HIL_SANDBOX_HOST=sbx \
        HIL_MAIN_DIR="$WORK/main" HIL_VISION_DIR="$WORK/vision" HIL_SOUND_DIR="$WORK/sound" HIL_SANDBOX_REPO="$WORK/sbx" \
        HIL_MAPS_SNAPSHOT="$WORK/sndstate/prod_maps.before.txt" \
        HIL_LOCK="$WORK/tmp/hil.lock" HIL_REMOTE_DIR="$WORK/remote" HIL_MAPS_DIR="$WORK/hilmaps" \
        HIL_SIM_PREFIX=t-hil HIL_WAIT_CM_S=5 \
        "$@" bash "$HIL" "${cmd[@]}" > "$WORK/out.log" 2>&1
}
line_of() { grep -n -m1 -- "$1" "$2" | cut -d: -f1; }

# ── 2. боевые пути — отказ до ssh ─────────────────────────────────────────
for bad in "$WORK/main/maps" "$WORK/main/maps/sub" /maps /data/hil relative/maps "$WORK/x/../main/maps"; do
    reset_state
    run_hil on HIL_MAPS_DIR="$bad"; rc=$?
    check "on отказывает при HIL_MAPS_DIR=$bad (rc=$rc), ssh не вызывался" \
        '[ "$rc" -ne 0 ] && [ ! -s "$WORK/ssh.log" ]'
done

# ── 3. on — порядок и содержимое ──────────────────────────────────────────
reset_state
run_hil on; rc=$?
check "on: выход 0 (rc=$rc)" '[ "$rc" -eq 0 ]' || cat "$WORK/out.log"
check "on: замок взят" '[ -f "$WORK/tmp/hil.lock" ] && grep -q "HIL on since" "$WORK/tmp/hil.lock"'
L_LOCK=$(line_of "lock-take" "$WORK/ssh.log")
L_STOPSSH=$(line_of "remote_pi.sh 'stop'" "$WORK/ssh.log")
L_STOPM=$(line_of "docker stop --time 20 ros2-control" "$WORK/docker.log")
check "on: ros2-control и lslidar остановлены" 'grep -q "stop --time 20 ros2-control" "$WORK/docker.log" && grep -q "stop --time 20 lslidar" "$WORK/docker.log"'
check "on: oak-d и ceiling-camera остановлены" 'grep -q "stop --time 20 oak-d" "$WORK/docker.log" && grep -q "stop --time 20 ceiling-camera" "$WORK/docker.log"'
L_SIMTIME=$(line_of "main.hil.yaml up -d --no-deps --pull never robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2" "$WORK/docker.log")
L_SIMUP=$(line_of "compose.hil.yaml up -d --no-build zenoh-router gazebo" "$WORK/docker.log")
check "on: шесть сервисов пересозданы с compose.hil.yaml и --no-deps" '[ -n "$L_SIMTIME" ]'
check "on: порядок замок → stop моторов → sim time → сим" \
    '[ -n "$L_LOCK" ] && [ -n "$L_STOPM" ] && [ -n "$L_SIMUP" ] && [ "$L_STOPM" -lt "$L_SIMTIME" ] && [ "$L_SIMTIME" -lt "$L_SIMUP" ] && [ "$L_LOCK" -lt "$L_STOPSSH" ]'
check "on: USE_SIM_TIME=true у nav2 и rtabmap" '[ "$(cat "$WORK/state/nav2.simt")" = true ] && [ "$(cat "$WORK/state/rtabmap.simt")" = true ]'
check "on: ros2-control не поднимался" '! grep -E "up .*ros2-control" "$WORK/docker.log" && [ "$(cat "$WORK/state/ros2-control")" = exited ]'
EXPECT_SIM_ENV="HIL_ROBOT_ID='RBXTEST01' HIL_SANDBOX_IP='10.1.1.30' HIL_ZENOH_CONNECT='tcp/10.1.1.10:7447#iface=enp2s0,tcp/10.1.1.11:7447#iface=enp2s0,tcp/10.1.1.12:7447#iface=enp2s0'"
check "on: ROBOT_ID робота из .env Main Pi и провод с #iface" \
    'grep -qF "$EXPECT_SIM_ENV" "$WORK/ssh.log"'
EXPECT_MAPS="HIL_MAPS_DIR='$WORK/hilmaps'"
check "on: карта rtabmap — HIL_MAPS_DIR" 'grep -qF "$EXPECT_MAPS" "$WORK/ssh.log" && [ -d "$WORK/hilmaps" ]'
check "on: ни одна команда Main/Vision/песочницы не трогает /data и боевой ./maps" \
    '! grep -v "^sound|" "$WORK/ssh.log" | grep -E "/data|$WORK/main/maps|rtabmap\.db"'
check "on: Sound видит /data только как HIL-реестр и чтение боевого maps (maps-dump)" \
    '! grep "^sound|" "$WORK/ssh.log" | grep "/data" | grep -vE "maps-dump.*/data/voice_memory\.db|HIL_WAYPOINT_DB=./data/hil/waypoints\.db."'
# Оверлей, который применяется на Main Pi, тоже не ссылается на боевые пути.
check "compose.hil.yaml Main: /maps только из HIL_MAPS_DIR" \
    '! grep -E "^\s*- \./maps|/data" "$REPO_ROOT/docker/main/compose.hil.yaml" && [ "$(grep -c "HIL_MAPS_DIR:?" "$REPO_ROOT/docker/main/compose.hil.yaml")" -eq 2 ]'

# ── 4. замок уже есть ─────────────────────────────────────────────────────
reset_state
echo "HIL on since earlier" > "$WORK/tmp/hil.lock"
run_hil on; rc=$?
check "on при замке: отказ (rc=$rc), моторы и сим не тронуты" \
    '[ "$rc" -ne 0 ] && ! grep -q "stop" "$WORK/docker.log" && ! grep -q "compose" "$WORK/docker.log"'

# ── 5. два контроллера ────────────────────────────────────────────────────
reset_state
run_hil on FAKE_CM=2; rc=$?
check "on при двух /controller_manager: выход 1, сим погашен" \
    '[ "$rc" -eq 1 ] && grep -q "compose.hil.yaml down" "$WORK/docker.log" && [ ! -f "$WORK/state/t-hil-gazebo" ]'

# ── 6. off ────────────────────────────────────────────────────────────────
reset_state
run_hil on >/dev/null
: > "$WORK/docker.log"; : > "$WORK/ssh.log"
run_hil off; rc=$?
check "off: выход 0 (rc=$rc)" '[ "$rc" -eq 0 ]' || cat "$WORK/out.log"
L_DOWN=$(line_of "compose.hil.yaml down" "$WORK/docker.log")
L_BACK=$(line_of "up -d --no-deps --pull never robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2 ros2-control lslidar" "$WORK/docker.log")
check "off: сим гаснет до возврата ros2-control" '[ -n "$L_DOWN" ] && [ -n "$L_BACK" ] && [ "$L_DOWN" -lt "$L_BACK" ]'
check "off: возврат без compose.hil.yaml, sim time снят" \
    '! grep -E "main.hil.yaml up" "$WORK/docker.log" && [ ! -f "$WORK/state/nav2.simt" ] && [ "$(cat "$WORK/state/ros2-control")" = running ]'
check "off: oak-d и ceiling-camera подняты" '[ "$(cat "$WORK/state/oak-d")" = running ] && [ "$(cat "$WORK/state/ceiling-camera")" = running ]'
check "off: замок снят последним" '[ ! -e "$WORK/tmp/hil.lock" ] && [ "$(tail -n 1 "$WORK/ssh.log" | grep -c lock-release)" -eq 1 ]'

# ── 6b. status: лог zenoh в stdout перед числом; docker inspect без контейнера ──
reset_state
run_hil on >/dev/null
run_hil status FAKE_ZENOH_LOG=1; rc=$?
check "status: число /controller_manager при zenoh-логе перед ним (rc=$rc)" \
    'grep -qE "controller_manager в графе.*: 1 \(норма" "$WORK/out.log"'
reset_state
run_hil on FAKE_ZENOH_LOG=1 FAKE_CM=2 >/dev/null; rc=$?
check "on: два /controller_manager ловится и при zenoh-логе (rc=$rc)" '[ "$rc" -eq 1 ]'
reset_state
run_hil on FAKE_ZENOH_LOG=1 >/dev/null; rc=$?
check "on: zenoh-лог в stdout счёта не ломает (rc=$rc)" '[ "$rc" -eq 0 ]'
reset_state
run_hil status FAKE_INSPECT_BLANK=1; rc=$?
check "status: контейнера gazebo нет (docker inspect печатает пустую строку) -> absent -" \
    'grep -qx "── gazebo: absent -" "$WORK/out.log"'
# off после упавшего on: gazebo нет, замок есть — робот должен вернуться.
reset_state
echo "HIL on since earlier" > "$WORK/tmp/hil.lock"
echo exited > "$WORK/state/ros2-control"
run_hil off FAKE_INSPECT_BLANK=1; rc=$?
check "off после упавшего on (gazebo нет, пустой inspect): ros2-control вернулся, замок снят (rc=$rc)" \
    '[ "$rc" -eq 0 ] && [ "$(cat "$WORK/state/ros2-control")" = running ] && [ ! -e "$WORK/tmp/hil.lock" ]'
# status: helper не доставлен != нет ssh.
reset_state
run_hil status; rc=$?
check "status сам доставляет helper (rc=$rc)" '[ -f "$WORK/remote/remote_pi.sh" ] && ! grep -q "нет ssh" "$WORK/out.log"'
reset_state
run_hil status FAKE_SSH_DOWN=vision; rc=$?
check "status: ssh до Vision лежит -> нет ssh до vision" 'grep -q "нет ssh до vision" "$WORK/out.log"'
check "vision ssh в hil.conf 10.1.1.21, zenoh-роутеры не тронуты" \
    'grep -q "HIL_VISION_HOST:=ros2@10.1.1.21}" "$REPO_ROOT/scripts/hil/hil.conf" && grep -q "HIL_ROBOT_ROUTERS:=10.1.1.10 10.1.1.11 10.1.1.12}" "$REPO_ROOT/scripts/hil/hil.conf"'

# ── 8. реестр карт и точек на Sound Pi (#4392) ────────────────────────────
for bad in /data/voice_memory.db /data/hil.db /data/hil/../voice_memory.db /data/hil/ relative/hil.db /data/hil/x.txt /maps/hil/x.db; do
    for c in on off; do
        reset_state
        run_hil "$c" HIL_WAYPOINT_DB="$bad"; rc=$?
        check "$c отказывает при HIL_WAYPOINT_DB=$bad (rc=$rc), ssh не вызывался" \
            '[ "$rc" -ne 0 ] && [ ! -s "$WORK/ssh.log" ]'
    done
done
reset_state
run_hil on HIL_PROD_TOOLS_DB=/data/hil/waypoints.db; rc=$?
check "on: HIL_WAYPOINT_DB = HIL_PROD_TOOLS_DB — отказ до ssh (rc=$rc)" '[ "$rc" -ne 0 ] && [ ! -s "$WORK/ssh.log" ]'

reset_state
run_hil on; rc=$?
check "on (реестр): выход 0 (rc=$rc)" '[ "$rc" -eq 0 ]' || cat "$WORK/out.log"
check "on: voice-assistant пересоздан с sound.hil.yaml и --no-deps" \
    'grep -q "sound.hil.yaml up -d --no-deps --pull never voice-assistant" "$WORK/docker.log"'
check "on: у voice-assistant WAYPOINT_DB_PATH=/data/hil/waypoints.db" \
    '[ "$(cat "$WORK/state/voice-assistant.wpdb")" = /data/hil/waypoints.db ]'
check "on: снапшот боевого maps снят и равен SELECT * FROM maps" \
    '[ "$(cat "$WORK/sndstate/prod_maps.before.txt")" = "$PROD_MAPS" ] && [ ! -e "$WORK/sndstate/prod_maps.before.txt.part" ]'
L_SNAP=$(line_of "maps-dump" "$WORK/ssh.log")
L_SWITCH=$(line_of "sound.hil.yaml' up -d" "$WORK/ssh.log")
L_LOCK=$(line_of "lock-take" "$WORK/ssh.log")
L_SIMUP=$(line_of "compose.hil.yaml up -d --no-build" "$WORK/docker.log")
L_SNDUP=$(line_of "sound.hil.yaml up" "$WORK/docker.log")
check "on: порядок замок → снапшот → переключение реестра → сим" \
    '[ -n "$L_SNAP" ] && [ -n "$L_SWITCH" ] && [ "$L_LOCK" -lt "$L_SNAP" ] && [ "$L_SNAP" -lt "$L_SWITCH" ] && [ "$L_SNDUP" -lt "$L_SIMUP" ]'
check "on: на Sound пересоздан только voice-assistant" \
    '[ "$(grep "^sound|" "$WORK/ssh.log" | grep -c "docker compose")" -eq 1 ] && ! grep "^sound|" "$WORK/ssh.log" | grep -E "docker (stop|rm)|supercollider"'
check "compose.hil.yaml Sound: только WAYPOINT_DB_PATH из обязательного HIL_WAYPOINT_DB" \
    'grep -q "WAYPOINT_DB_PATH=\${HIL_WAYPOINT_DB:?" "$REPO_ROOT/docker/sound/compose.hil.yaml" && [ "$(grep -cE "^\s+- " "$REPO_ROOT/docker/sound/compose.hil.yaml")" -eq 1 ] && ! grep -E "VOICE_MEMORY_DB_PATH|volumes:" "$REPO_ROOT/docker/sound/compose.hil.yaml"'
run_hil status; rc=$?
check "status: Sound показывает WAYPOINT_DB_PATH в HIL" 'grep -q "WAYPOINT_DB_PATH=/data/hil/waypoints.db" "$WORK/out.log"'

: > "$WORK/docker.log"; : > "$WORK/ssh.log"
run_hil off; rc=$?
check "off (реестр): выход 0 (rc=$rc)" '[ "$rc" -eq 0 ]' || cat "$WORK/out.log"
check "off: voice-assistant без sound.hil.yaml, WAYPOINT_DB_PATH снят" \
    'grep -q "up -d --no-deps --pull never voice-assistant" "$WORK/docker.log" && ! grep -q "sound.hil.yaml up" "$WORK/docker.log" && [ ! -f "$WORK/state/voice-assistant.wpdb" ]'
check "off: в логе сверка «реестр не изменился» и строки до/после" \
    'grep -q "боевой реестр карт (SELECT \* FROM maps) за HIL не изменился" "$WORK/out.log" && grep -q "до    1|aaaa-1111|мастерская" "$WORK/out.log" && grep -q "после 2|bbbb-2222|хил-офис" "$WORK/out.log"'
check "off: снапшот после успешной сверки убран, замок снят последним" \
    '[ ! -e "$WORK/sndstate/prod_maps.before.txt" ] && [ ! -e "$WORK/tmp/hil.lock" ] && [ "$(tail -n 1 "$WORK/ssh.log" | grep -c lock-release)" -eq 1 ]'

# Боевой maps изменился за HIL (кто-то переключил is_active) — off возвращает робота, но выход 3 и diff.
reset_state
run_hil on >/dev/null
printf '%s\n' '1|aaaa-1111|мастерская|1793000000.0|0' '2|bbbb-2222|хил-офис|1793100000.0|1' > "$WORK/state/prod_maps"
run_hil off; rc=$?
check "off при изменённом боевом maps: выход 3 (rc=$rc), робот вернулся, замок снят" \
    '[ "$rc" -eq 3 ] && [ "$(cat "$WORK/state/ros2-control")" = running ] && [ ! -f "$WORK/state/voice-assistant.wpdb" ] && [ ! -e "$WORK/tmp/hil.lock" ]'
check "off при изменённом maps: diff в логе, снапшот оставлен" \
    'grep -q "ИЗМЕНИЛСЯ" "$WORK/out.log" && grep -qE "^ +< 1\|aaaa-1111\|мастерская\|1793000000.0\|1" "$WORK/out.log" && grep -qE "^ +> 1\|aaaa-1111\|мастерская\|1793000000.0\|0" "$WORK/out.log" && [ -f "$WORK/sndstate/prod_maps.before.txt" ]'

# Боевой maps не читается — on не переключает реестр.
reset_state
rm -f "$WORK/state/prod_maps"
run_hil on; rc=$?
check "on: боевой maps не прочитан — выход ≠0 (rc=$rc), voice-assistant не пересоздан, сим не поднят" \
    '[ "$rc" -ne 0 ] && ! grep -q "sound.hil.yaml up" "$WORK/docker.log" && [ ! -f "$WORK/state/t-hil-gazebo" ]'

# off после on, упавшего до Sound (снапшота нет): робот возвращается, сверка честно «не сверен».
reset_state
echo "HIL on since earlier" > "$WORK/tmp/hil.lock"
run_hil off; rc=$?
check "off без снапшота: выход 0 (rc=$rc), в логе «НЕ сверен», замок снят" \
    '[ "$rc" -eq 0 ] && grep -q "снапшота .* нет" "$WORK/out.log" && grep -q "НЕ сверен" "$WORK/out.log" && [ ! -e "$WORK/tmp/hil.lock" ]'

# off, а voice-assistant остался на HIL-реестре (compose не снял env) — замок не снимается.
reset_state
run_hil on >/dev/null
mkdir -p "$WORK/bin2"
cat > "$WORK/bin2/docker" <<'EOF'
#!/usr/bin/env bash
# Фейк поверх фейка: compose up на Sound без оверлея «забывает» снять WAYPOINT_DB_PATH.
real="$(dirname "$0")/../bin/docker"
case "$*" in
    *"sound.images.yaml up"*) echo "docker $*" >> "$FAKE_DOCKER_LOG"; echo running > "$FAKE_STATE/voice-assistant"; exit 0 ;;
esac
exec "$real" "$@"
EOF
chmod +x "$WORK/bin2/docker"
run_hil off PATH="$WORK/bin2:$WORK/bin:$PATH"; rc=$?
check "off: voice-assistant остался с WAYPOINT_DB_PATH — отказ (rc=$rc), замок оставлен" \
    '[ "$rc" -ne 0 ] && [ -e "$WORK/tmp/hil.lock" ] && grep -q "WAYPOINT_DB_PATH=/data/hil/waypoints.db" "$WORK/out.log"'

TOOLS_DEFAULT=$(sed -n 's/^TOOLS_DB_DEFAULT = "\(.*\)"$/\1/p' "$REPO_ROOT/src/rob_box_core/rob_box_core/tools_db.py" | tr -d '\r')
check "hil.conf: HIL_PROD_TOOLS_DB = TOOLS_DB_DEFAULT из rob_box_core.tools_db ($TOOLS_DEFAULT)" \
    '[ -n "$TOOLS_DEFAULT" ] && grep -qF "HIL_PROD_TOOLS_DB:=$TOOLS_DEFAULT}" "$REPO_ROOT/scripts/hil/hil.conf"'
check "hil.conf: Sound Pi ssh ros2@10.1.1.22, реестр HIL под /data/hil/" \
    'grep -q "HIL_SOUND_HOST:=ros2@10.1.1.22}" "$REPO_ROOT/scripts/hil/hil.conf" && grep -q "HIL_WAYPOINT_DB:=/data/hil/waypoints.db}" "$REPO_ROOT/scripts/hil/hil.conf"'
check "env WAYPOINT_DB_PATH — то же имя, что rob_box_core.tools_db.WAYPOINT_DB_ENV" \
    'grep -q "^WAYPOINT_DB_ENV = \"WAYPOINT_DB_PATH\"" "$REPO_ROOT/src/rob_box_core/rob_box_core/tools_db.py"'

# ── 9. off: сервис сломан ещё до on (followpath-watchdog в Restarting) ────────
broken_before() {  # broken_before <сервис>: Restarting и до on, и после compose up
    echo restarting > "$WORK/state/$1"; : > "$WORK/state/$1.broken"
}
reset_state
broken_before followpath-watchdog
run_hil on; rc=$?
check "on со сломанным followpath-watchdog: выход 0 (rc=$rc), состояние «до» записано" \
    '[ "$rc" -eq 0 ] && grep -qx "followpath-watchdog restarting" "$WORK/remote/pre_on_state.txt" && grep -qx "nav2 running" "$WORK/remote/pre_on_state.txt" && grep -qx "voice-assistant running" "$WORK/remote/pre_on_state.txt" && grep -qx "oak-d running" "$WORK/remote/pre_on_state.txt"'
run_hil off; rc=$?
check "off: сервис сломан и до on — предупреждение, выход 0 (rc=$rc), замок снят, реестр сверен" \
    '[ "$rc" -eq 0 ] && grep -q "followpath-watchdog restarting.*⚠️.*до on было: restarting" "$WORK/out.log" && grep -q "нерабочих и до on: 1" "$WORK/out.log" && [ ! -e "$WORK/tmp/hil.lock" ] && grep -q "SELECT \* FROM maps) за HIL не изменился" "$WORK/out.log" && [ ! -e "$WORK/remote/pre_on_state.txt" ]'
# сервис работал до on, после off не работает — провал, замок остаётся, реестр всё равно сверен
reset_state
run_hil on >/dev/null
: > "$WORK/state/nav2.broken"
run_hil off; rc=$?
check "off: nav2 работал до on, после off Restarting — провал (rc=$rc), замок оставлен, реестр сверен" \
    '[ "$rc" -ne 0 ] && grep -q "nav2 restarting.*❌" "$WORK/out.log" && [ -e "$WORK/tmp/hil.lock" ] && grep -q "SELECT \* FROM maps) за HIL не изменился" "$WORK/out.log"'
# сломан до on и поправился сам — это ok
reset_state
broken_before followpath-watchdog
run_hil on >/dev/null
rm -f "$WORK/state/followpath-watchdog.broken"
run_hil off; rc=$?
check "off: сервис, сломанный до on, снова работает — ok без предупреждения (rc=$rc)" \
    '[ "$rc" -eq 0 ] && ! grep -q "нерабочих и до on" "$WORK/out.log" && [ ! -e "$WORK/tmp/hil.lock" ]'
# изменение боевого maps при сломанном-до-on сервисе: выход 3, замок снят
reset_state
broken_before followpath-watchdog
run_hil on >/dev/null
printf '%s\n' '1|aaaa-1111|мастерская|1793000000.0|0' > "$WORK/state/prod_maps"
run_hil off; rc=$?
check "off: предупреждение + изменённый maps → выход 3 (rc=$rc), замок снят" '[ "$rc" -eq 3 ] && [ ! -e "$WORK/tmp/hil.lock" ]'

# ── 9b. restart: рестарт стека в работающем HIL (#4468) ──────────────────
reset_state
run_hil "restart rtabmap" HIL_RTABMAP_MODE=localization; rc=$?
check "restart без HIL on: отказ (rc=$rc), ничего не остановлено и не поднято" \
    '[ "$rc" -ne 0 ] && ! grep -q "stop" "$WORK/docker.log" && ! grep -q "compose" "$WORK/docker.log"'
reset_state
run_hil on >/dev/null
: > "$WORK/docker.log"; : > "$WORK/ssh.log"
run_hil "restart rtabmap" HIL_RTABMAP_MODE=localization HIL_MAPS_DIR="$WORK/other_maps"; rc=$?
L_RSTOP=$(line_of "stop --time 60 rtabmap" "$WORK/docker.log")
L_RUP=$(line_of "main.hil.yaml up -d --no-deps --pull never --force-recreate rtabmap" "$WORK/docker.log")
check "restart rtabmap: выход 0 (rc=$rc), сначала docker stop --time 60, потом up --force-recreate только rtabmap" \
    '[ "$rc" -eq 0 ] && [ -n "$L_RSTOP" ] && [ -n "$L_RUP" ] && [ "$L_RSTOP" -lt "$L_RUP" ] && ! grep -q "force-recreate rtabmap ." "$WORK/docker.log"' \
    || cat "$WORK/out.log"
check "restart: карта из замка (HIL_MAPS_DIR окружения не берётся), режим localization" \
    'grep -qF "HIL_MAPS_DIR='"'"'$WORK/hilmaps'"'"'" "$WORK/ssh.log" && ! grep -qF "$WORK/other_maps" "$WORK/ssh.log" && grep -qF "HIL_RTABMAP_MODE='"'"'localization'"'"'" "$WORK/ssh.log"'
check "restart: замок HIL на месте, ros2-control не поднимался" \
    '[ -f "$WORK/tmp/hil.lock" ] && ! grep -E "up .*ros2-control" "$WORK/docker.log"'
: > "$WORK/docker.log"
run_hil "restart ros2-control" HIL_RTABMAP_MODE=localization; rc=$?
check "restart ros2-control: отказ (не из контура HIL, rc=$rc), docker не вызывался" \
    '[ "$rc" -ne 0 ] && [ ! -s "$WORK/docker.log" ]'
run_hil "restart rtabmap" HIL_RTABMAP_MODE=bogus; rc=$?
check "restart с HIL_RTABMAP_MODE=bogus: отказ (rc=$rc)" '[ "$rc" -ne 0 ] && [ ! -s "$WORK/docker.log" ]'
run_hil "restart rtabmap"; rc=$?
check "restart rtabmap в mapping (умолчание hil.conf): отказ — стёр бы карту (rc=$rc), docker не вызывался" \
    '[ "$rc" -ne 0 ] && [ ! -s "$WORK/docker.log" ] && grep -q "delete_db_on_start" "$WORK/out.log"'
run_hil off >/dev/null

# ── 10. sandbox_prep: fix_crlf.sh не заходит в out*/fuel и не падает на чужих файлах ──
FIX="$REPO_ROOT/scripts/hil/video/fix_crlf.sh"
CR=$'\r'
has_cr() { [ "$(tr -d '\r' < "$1" | wc -c)" -ne "$(wc -c < "$1")" ]; }
T="$WORK/crlf"; rm -rf "$T"; mkdir -p "$T/out/shisyun_s1/bag" "$T/fuel/Depot" "$T/sub"
printf 'a%s\nb%s\n' "$CR" "$CR" > "$T/run.sh"
printf 'a%s\n' "$CR" > "$T/sub/x.py"
printf 'a%s\n' "$CR" > "$T/out/shisyun_s1/bag/other.sh"
printf 'a%s\n' "$CR" > "$T/fuel/Depot/m.sdf"
chmod 444 "$T/out/shisyun_s1/bag/other.sh"; chmod 555 "$T/out/shisyun_s1/bag" "$T/out/shisyun_s1" "$T/out"
bash "$FIX" "$T"; rc=$?
check "fix_crlf: выход 0 при недоступном out/ (rc=$rc)" '[ "$rc" -eq 0 ]'
check "fix_crlf: CRLF в run.sh и sub/x.py убраны" '! has_cr "$T/run.sh" && ! has_cr "$T/sub/x.py"'
check "fix_crlf: out/ и fuel/ не тронуты" 'has_cr "$T/out/shisyun_s1/bag/other.sh" && has_cr "$T/fuel/Depot/m.sdf"'
chmod -R u+rwx "$T"
if [ "$(id -u)" != 0 ]; then
    printf 'a%s\n' "$CR" > "$T/sub/ro.sh"; chmod 444 "$T/sub/ro.sh"; chmod 555 "$T/sub"
    bash "$FIX" "$T"; rc=$?
    check "fix_crlf: файл в каталоге без записи не роняет прогон (rc=$rc)" '[ "$rc" -eq 0 ]'
    chmod -R u+rwx "$T"
else
    echo "SKIP: fix_crlf с недоступным каталогом (root)"
fi
check "sandbox_prep.sh вызывает fix_crlf.sh, не голый find" \
    'grep -q "fix_crlf.sh" "$REPO_ROOT/scripts/hil/video/sandbox_prep.sh" && ! grep -q "find \. -path \./fuel" "$REPO_ROOT/scripts/hil/video/sandbox_prep.sh"'

# ── 11. CAMERAS у runvideo.sh / runcoverage.sh ────────────────────────────
mkdir -p "$WORK/vbin" "$WORK/vdir/out"
cat > "$WORK/vbin/docker" <<'EOF'
#!/usr/bin/env bash
echo "docker $*" >> "$VLOG"
exit 0
EOF
chmod +x "$WORK/vbin/docker"
run_video() {  # run_video <скрипт> [VAR=val ...]; лог docker — $WORK/vlog.txt
    local sc="$1"; shift
    : > "$WORK/vlog.txt"
    env PATH="$WORK/vbin:$PATH" VLOG="$WORK/vlog.txt" VIDEO_DIR="$WORK/vdir" SETTLE_S=0 STOP_S=0 "$@" \
        bash "$REPO_ROOT/scripts/hil/video/$sc" > "$WORK/vout.txt" 2>&1
}
run_video runvideo.sh; rc=$?
check "runvideo.sh по умолчанию CAMERAS=1: подписчики запущены, в конце убиты по ign-transport-topic (rc=$rc)" \
    '[ "$rc" -eq 0 ] && grep -q "ign topic -e -t /sim/video/video_top" "$WORK/vlog.txt" && grep -q "pkill -f ign-transport-topic" "$WORK/vlog.txt" && grep -qx "CAMERAS=1" "$WORK/vout.txt"'
run_video runvideo.sh CAMERAS=0; rc=$?
check "runvideo.sh CAMERAS=0: подписчиков камер нет (rc=$rc)" \
    '[ "$rc" -eq 0 ] && ! grep -q "ign topic" "$WORK/vlog.txt" && grep -qx "CAMERAS=0" "$WORK/vout.txt"'
run_video runcoverage.sh; rc=$?
check "runcoverage.sh по умолчанию CAMERAS=0: подписчиков нет, старые гасятся (rc=$rc)" \
    '[ "$rc" -eq 0 ] && ! grep -q "ign topic" "$WORK/vlog.txt" && grep -q "pkill -f ign-transport-topic" "$WORK/vlog.txt" && grep -qx "CAMERAS=0" "$WORK/vout.txt"'
run_video runcoverage.sh CAMERAS=1; rc=$?
check "runcoverage.sh CAMERAS=1: подписчики запущены (rc=$rc)" \
    '[ "$rc" -eq 0 ] && grep -q "ign topic -e -t /sim/video/video_chase" "$WORK/vlog.txt"'
run_video runvideo.sh CAMERAS=2; rc=$?
check "runvideo.sh CAMERAS=2: отказ, docker не вызывался (rc=$rc)" '[ "$rc" -eq 2 ] && [ ! -s "$WORK/vlog.txt" ]'

# ── 7. замок в деплое ─────────────────────────────────────────────────────
fake_free() { echo HIL_FREE; }
mk_probe() { printf '#!/usr/bin/env bash\n%s\n' "$1" > "$WORK/bin/probe"; chmod +x "$WORK/bin/probe"; }
mk_probe 'echo HIL_FREE'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: замка нет → 0" '[ "$rc" -eq 0 ]'
mk_probe 'printf "HIL_LOCKED\nHIL on since 2026-11-04 by shifu\n"'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: замок → 1 и текст про hil.sh off" '[ "$rc" -eq 1 ] && grep -q "hil.sh off" <<<"$out" && grep -q "by shifu" <<<"$out"'
mk_probe 'echo "ssh: connect to host 10.1.1.10 port 22: No route"; exit 255'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: нет ответа → 2" '[ "$rc" -eq 2 ]'
mk_probe 'echo "Warning: Permanently added x (ECDSA) to the list of known hosts." >&2; echo HIL_FREE'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: stderr-предупреждение known hosts + HIL_FREE → 0" '[ "$rc" -eq 0 ]'
mk_probe 'echo "Warning: Permanently added x (ECDSA) to the list of known hosts."; echo HIL_FREE'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: предупреждение строкой в stdout + HIL_FREE → 0" '[ "$rc" -eq 0 ]'
mk_probe 'echo "Warning: Permanently added x (ECDSA)." >&2; printf "HIL_LOCKED\nby shifu\n"'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: предупреждение + HIL_LOCKED → 1" '[ "$rc" -eq 1 ] && grep -q "by shifu" <<<"$out"'
mk_probe 'exit 0'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: пустой вывод → 2" '[ "$rc" -eq 2 ]'
mk_probe 'echo "Warning: only stderr" >&2'
out=$(bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: только stderr без маркера → 2" '[ "$rc" -eq 2 ]'
# Настоящая проверка на «Pi»: команда — bash -c.
mk_probe 'exec bash -c "$1"'
out=$(HIL_LOCK="$WORK/tmp/hl" bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: живой bash, файла нет → 0" '[ "$rc" -eq 0 ]'
echo "x" > "$WORK/tmp/hl"
out=$(HIL_LOCK="$WORK/tmp/hl" bash "$LOCKCHK" "$WORK/bin/probe"); rc=$?
check "check_hil_lock: живой bash, файл есть → 1" '[ "$rc" -eq 1 ]'

W_LOCK=$(grep -n -m1 'name: "\[Main Pi\] HIL lock' "$WORKFLOW" | cut -d: -f1)
W_FIRST_STOP=$(grep -n -m1 'name: ".*Stop Containers"' "$WORKFLOW" | cut -d: -f1)
W_FIRST_UPDATE=$(grep -n -m1 'name: ".*Update Code"' "$WORKFLOW" | cut -d: -f1)
check "деплой: шаг замка HIL до первого Update Code и Stop Containers" \
    '[ -n "$W_LOCK" ] && [ "$W_LOCK" -lt "$W_FIRST_STOP" ] && [ "$W_LOCK" -lt "$W_FIRST_UPDATE" ]'
check "деплой: шаг замка вызывает check_hil_lock.sh и не маскирован" \
    'sed -n "${W_LOCK},$((W_LOCK + 6))p" "$WORKFLOW" | grep -q "check_hil_lock.sh" && ! sed -n "${W_LOCK},$((W_LOCK + 6))p" "$WORKFLOW" | grep -qE "continue-on-error|\|\| *(true|echo)"'
U="$REPO_ROOT/docker/main/update_and_restart.sh"
check "update_and_restart.sh: замок проверяется до down" \
    '[ "$(line_of "/tmp/hil.lock" "$U")" -lt "$(line_of "docker-compose down" "$U")" ]'

echo
echo "итог: $pass PASS, $failures FAIL"
[ "$failures" -eq 0 ]
