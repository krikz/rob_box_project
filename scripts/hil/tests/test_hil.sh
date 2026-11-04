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
#      пересоздание пяти сервисов с compose.hil.yaml и --no-deps → сим; ID робота
#      из .env Main Pi; Zenoh-endpoint'ы с #iface;
#   4. on при существующем замке — отказ, моторы не трогаются;
#   5. on, если в графе два /controller_manager — сим гасится, выход 1;
#   6. off: сим гаснет ДО возврата ros2-control; замок снимается последним;
#   7. scripts/deploy/check_hil_lock.sh: свободно / замок / нет ответа;
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
            *"Config.Env"*) [ -f "$S/$name.simt" ] && echo "USE_SIM_TIME=$(cat "$S/$name.simt")"; echo PATH=/x ;;
        esac ;;
    image) exit 0 ;;
    logs) echo "fake logs" ;;
    exec) [ -n "${FAKE_ZENOH_LOG:-}" ] && echo "INFO zenoh::net::runtime: Using ZID: 1a2b3c"; echo "${FAKE_CM:-1}"; echo /controller_manager ;;
    ps)
        case "$*" in
            *working_dir=*) for s in ros2-control lslidar robot-state-publisher twist-mux rtabmap imu-filter nav2 oak-d ceiling-camera; do
                                echo "$s ghcr.io/krikz/rob_box:$s-humble-dev-abc"; done ;;
        esac ;;
    compose)
        shift
        hil=0; sim=0; verb=""; svcs=()
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -f) case "$2" in *main.hil.yaml) hil=1 ;; *compose.hil.yaml) sim=1 ;; esac; shift 2 ;;
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
                echo running > "$S/$s"
                if [ "$hil" = 1 ]; then echo true > "$S/$s.simt"; else rm -f "$S/$s.simt"; fi
            done
        fi ;;
esac
exit 0
EOF
chmod +x "$WORK/bin/ssh" "$WORK/bin/docker"

mkdir -p "$WORK/main" "$WORK/vision" "$WORK/sbx/docker/sim"
echo "ROBOT_ID=RBXTEST01" > "$WORK/main/.env"
touch "$WORK/sbx/docker/sim/compose.hil.yaml"

reset_state() { rm -rf "$WORK/state" "$WORK/remote" "$WORK/tmp"; mkdir -p "$WORK/state" "$WORK/tmp"; : > "$WORK/ssh.log"; : > "$WORK/docker.log"; }

run_hil() {  # run_hil <cmd> [VAR=val ...]
    local cmd="$1"; shift
    env PATH="$WORK/bin:$PATH" FAKE_SSH_LOG="$WORK/ssh.log" FAKE_DOCKER_LOG="$WORK/docker.log" \
        FAKE_STATE="$WORK/state" SIM_PREFIX=t-hil \
        HIL_MAIN_HOST=main HIL_VISION_HOST=vision HIL_SANDBOX_HOST=sbx \
        HIL_MAIN_DIR="$WORK/main" HIL_VISION_DIR="$WORK/vision" HIL_SANDBOX_REPO="$WORK/sbx" \
        HIL_LOCK="$WORK/tmp/hil.lock" HIL_REMOTE_DIR="$WORK/remote" HIL_MAPS_DIR="$WORK/hilmaps" \
        HIL_SIM_PREFIX=t-hil HIL_WAIT_CM_S=5 \
        "$@" bash "$HIL" "$cmd" > "$WORK/out.log" 2>&1
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
L_SIMTIME=$(line_of "main.hil.yaml up -d --no-deps --pull never robot-state-publisher twist-mux rtabmap imu-filter nav2" "$WORK/docker.log")
L_SIMUP=$(line_of "compose.hil.yaml up -d --no-build zenoh-router gazebo" "$WORK/docker.log")
check "on: пять сервисов пересозданы с compose.hil.yaml и --no-deps" '[ -n "$L_SIMTIME" ]'
check "on: порядок замок → stop моторов → sim time → сим" \
    '[ -n "$L_LOCK" ] && [ -n "$L_STOPM" ] && [ -n "$L_SIMUP" ] && [ "$L_STOPM" -lt "$L_SIMTIME" ] && [ "$L_SIMTIME" -lt "$L_SIMUP" ] && [ "$L_LOCK" -lt "$L_STOPSSH" ]'
check "on: USE_SIM_TIME=true у nav2 и rtabmap" '[ "$(cat "$WORK/state/nav2.simt")" = true ] && [ "$(cat "$WORK/state/rtabmap.simt")" = true ]'
check "on: ros2-control не поднимался" '! grep -E "up .*ros2-control" "$WORK/docker.log" && [ "$(cat "$WORK/state/ros2-control")" = exited ]'
EXPECT_SIM_ENV="HIL_ROBOT_ID='RBXTEST01' HIL_SANDBOX_IP='10.1.1.30' HIL_ZENOH_CONNECT='tcp/10.1.1.10:7447#iface=enp2s0,tcp/10.1.1.11:7447#iface=enp2s0,tcp/10.1.1.12:7447#iface=enp2s0'"
check "on: ROBOT_ID робота из .env Main Pi и провод с #iface" \
    'grep -qF "$EXPECT_SIM_ENV" "$WORK/ssh.log"'
EXPECT_MAPS="HIL_MAPS_DIR='$WORK/hilmaps'"
check "on: карта rtabmap — HIL_MAPS_DIR" 'grep -qF "$EXPECT_MAPS" "$WORK/ssh.log" && [ -d "$WORK/hilmaps" ]'
check "on: ни одна команда не трогает /data и боевой ./maps" \
    '! grep -E "/data|$WORK/main/maps|rtabmap\.db" "$WORK/ssh.log"'
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
L_BACK=$(line_of "up -d --no-deps --pull never robot-state-publisher twist-mux rtabmap imu-filter nav2 ros2-control lslidar" "$WORK/docker.log")
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
