#!/usr/bin/env bash
# HIL-проверки группы А (#3789, #4435/#4458, #4470, #4365, #4770): один запуск → raw-файлы и строки вердикта кода.
# Runbook: docs/runbooks/hil-checks.md. HIL включает и выключает не этот скрипт, а scripts/hil/hil.sh.
#
#   HIL_CHECKS_OWNER=<первое слово stand.lock> scripts/hil/checks/run_checks.sh [сценарий ...]   (без аргументов — all)
#
# Сценарии (all — по порядку):
#   orphan          #4458  сирота нарочно: своя цель FollowPath после отменённой NavigateToPose — гасит сторож
#   zone            #4365 + #4470(on)  явно HIL_GUARD_SOUND=on, телеоп в препятствие: стоп, писк, «Осторожно!», маячок
#   guard_off       #4470(off)  умолчание/явный off: тот же наезд без писка, затем назад off (умолчание HIL)
#   corner          #4631  угол корпуса на пути: стенка перекрывает только левый угол; стоп до 0.20 м внутри коридора
#   doorway         #4365  регресс проёма 0.90 м (мир polygon, x=2): подъезд по оси, в проёме нет стопа
#   nav_aborted     #3789 + #4435  «поезжай на …» в недостижимую точку → фраза по ABORTED; после провала mux = 0
#   stop_idle       #3789  «отмени навигацию» без цели → «Я и так стою.»
#   stop_active     #3789 + #4435  то же во время поездки из скрипта → «Останавливаюсь.»; после отмены mux = 0
#   orphan_natural  #4435  цель NavigateToPose из скрипта (3b из #4365): нужен ORPHAN_GOAL="x,y,yaw°" (карта мира);
#                   ORPHAN_LOAD_S=120 — нагрузка Main Pi (4 пустых цикла с timeout). Без ORPHAN_GOAL — пропуск
#   controller_abort #4770  цель в свободную сторону, FollowPath принят, корпус удержан нулём на /cmd_vel_web
#                   (priority 50 > Nav2). Progress checker абортит за movement_time_allowance (30 с, не крутим).
#                   Не nav_aborted: без принятого FollowPath вердикт NOT_TRIGGERED.
#   client_kill     #4435  цель принята, процесс пробы убит (SIGTERM) без cancel_goal. Робот должен встать.
#   host_cancel     #4435  ros2 action cancel /navigate_to_pose с хоста (нулевой GoalInfo), не наш тул.
#   move_ahead      #5160  «Робот, вперёд на метр»: цель создаёт move_direction, проба только мерит TRACK.
#                   До цели — isolation.py: только Gazebo (один /controller_manager),
#                   боевой ros2-control не running, командных кадров VESC 0. Иначе выход 2.
#
# Предусловия проверяет сам и честно отказывает (выход 3): stand.lock начинается с HIL_CHECKS_OWNER (замок берёт тот,
# кто запускает, скрипт его не трогает), hil.sh status = HIL on и один /controller_manager, нужные контейнеры running,
# здоровы и живут ≥ 15 с (followpath-watchdog в Restarting — отказ), часы Pi и песочницы расходятся ≤ 5 с.
# Итог: каталог HIL_CHECKS_OUT (по умолчанию $TMPDIR/hil_checks/<UTC>), в нём verdicts.txt и raw по сценариям.
# Пароли не используются: ssh -o BatchMode=yes, ключ в authorized_keys.
set -euo pipefail
export MSYS_NO_PATHCONV=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
if [ -n "${HIL_CONF:-}" ]; then
    # shellcheck source=/dev/null
    . "$HIL_CONF"
fi
# shellcheck source=scripts/hil/hil.conf
. "$REPO_ROOT/scripts/hil/hil.conf"

: "${HIL_CHECKS_OWNER:=}"
: "${STAND_LOCK:=C:/Users/krikz/AppData/Local/Temp/claude/a31bd7f7b8b42df0/3f238318-a4ed-402f-88a9-a5003be4590b/scratchpad/stand.lock}"
: "${HIL_CHECKS_OUT:=${TMPDIR:-/tmp}/hil_checks/$(date -u +%Y%m%dT%H%M%SZ)}"
: "${PY:=python}"
# Недостижимая точка: план не строится, Nav2 после восстановлений отдаёт ABORTED (#3789).
: "${ABORT_PHRASE:=Робот, поезжай на координаты икс сорок, игрек сорок.}"
: "${ABORT_WAIT_S:=180}"
# Фразы для stop_navigation через модель (стоп-слова «стой/стоп/останови» уходят в Tier-1 stop_all, тул не зовётся).
: "${STOP_PHRASES:=Робот, отмени навигацию.|Робот, отмени текущую цель навигации.|Робот, вызови тул stop_navigation.}"
: "${STOP_AHEAD_M:=3.0}"
: "${ORPHAN_LEN_M:=1.5}"
: "${ORPHAN_GOAL:=}"
: "${ORPHAN_LOAD_S:=0}"
# sim-секунд после провала цели, за которые FollowPath должен погаснуть и mux обнулиться (orphan_timeout_s 1.5 + такт).
: "${ORPHAN_GRACE_S:=2.0}"
: "${WALL_SPEED:=0.1}"
# Регресс проёма 0.90 м: цель за проёмом x=2 мира polygon, участок проёма по x правды Gazebo.
: "${DOOR_GOAL:=3,0,0}"
: "${DOOR_OPENING:=1.2,2.8}"
# #4770: боевой movement_time_allowance (nav2_params.yaml). Сценарий его не укорачивает.
: "${PROGRESS_ALLOWANCE_S:=30}"
: "${HOLD_AHEAD_M:=1.5}"
# #5160: реплика, которая должна вызвать move_direction, не цель пробы.
: "${MOVE_AHEAD_PHRASE:=Робот, вперёд на метр.}"
: "${MOVE_AHEAD_WAIT_S:=90}"

OUT="$HIL_CHECKS_OUT"
GZ="$HIL_SIM_PREFIX-gazebo"
REMOTE_PROBE=/tmp/hil_checks/hil_probe.py
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
ALL=(orphan zone guard_off corner doorway nav_aborted stop_idle stop_active orphan_natural controller_abort client_kill host_cancel move_ahead)
NAV_TOOLS_RX='navigate_to_coordinates|navigate_to_waypoint|move_direction'
GUARD_SOUND_CHANGED=0

log() { printf '[checks %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"; }
die() { printf '❌ %s\n' "$*" >&2; exit 3; }
rsh() {  # rsh <host> <команда>
    local host="$1"; shift
    ssh "${SSH_OPTS[@]}" "$host" "$*" </dev/null
}
iso_now() { date -u +%Y-%m-%dT%H:%M:%SZ; }
dlogs() {  # dlogs <host> <контейнер> <since ISO> <файл>: логи с метками UTC (ROS пишет в stderr)
    rsh "$1" "docker logs --timestamps --since $3 $2 2>&1" >"$4" || true
}
# Путь для python: в Git Bash с MSYS_NO_PATHCONV=1 «/d/...» и «/tmp» Windows-python не поймёт.
pypath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
verdict() {  # verdict <сценарий> <каталог> [аргументы verdicts.py]: строку и код решает код
    local sc="$1" d="$2"; shift 2
    "$PY" -X utf8 "$(pypath "$HERE/verdicts.py")" "$sc" "$(pypath "$d")" "$@" || true
}
probe() {  # probe <файл> <аргументы probe.py>: проба в контейнере gazebo песочницы, шум zenoh отрезан
    local f="$1"; shift
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ /entrypoint.sh python3 -u /hil_probe.py $*" 2>&1 \
        | grep --line-buffered -v -i -E 'zenoh|^\s*$' >"$f" || true
}
probe_stop() { rsh "$HIL_SANDBOX_HOST" "docker exec $GZ touch /tmp/hil_probe.stop" || true; }
push_probe() {
    tr -d '\r' <"$HERE/probe.py" | ssh "${SSH_OPTS[@]}" "$HIL_SANDBOX_HOST" \
        "mkdir -p $(dirname "$REMOTE_PROBE") && cat > $REMOTE_PROBE && docker cp $REMOTE_PROBE $GZ:/hil_probe.py"
}
inject() {  # inject <фраза> <файл>: реплика в диалог v2 (/voice/stt/utterance) на Sound Pi
    ROBOT="$HIL_SOUND_HOST" bash "$REPO_ROOT/scripts/robot_dev/inject.sh" "$1" >>"$2" 2>&1 || true
}
wait_log() {  # wait_log <host> <контейнер> <since ISO> <регекс> <секунд>: ждать строку в логах контейнера
    local t_end=$((SECONDS + $5)) text
    while [ "$SECONDS" -lt "$t_end" ]; do
        text=$(rsh "$1" "docker logs --since $3 $2 2>&1" || true)
        if printf '%s\n' "$text" | grep -q -E "$4"; then return 0; fi
        sleep 5
    done
    return 1
}
wait_file() {  # wait_file <файл> <регекс> <секунд>
    local t_end=$((SECONDS + $3))
    while [ "$SECONDS" -lt "$t_end" ]; do
        if grep -q -E "$2" "$1" 2>/dev/null; then return 0; fi
        sleep 1
    done
    return 1
}
skip() {  # skip <карточка> <сценарий> <причина>: честная строка «не проверено»
    echo "VERDICT $1 $2 NOT_TRIGGERED: $3" | tee -a "$OUT/verdicts.txt"
}

# ── предусловия ─────────────────────────────────────────────────────────────────────────────

needs_for() {  # контейнеры, без которых выбранные сценарии не идут
    local sc need=(nav2 twist-mux proximity-guard "$GZ")
    for sc in "$@"; do
        case "$sc" in
            orphan | orphan_natural | nav_aborted | stop_active | controller_abort | client_kill | host_cancel) \
                need+=(followpath-watchdog) ;;
        esac
        case "$sc" in
            zone | guard_off | nav_aborted | stop_idle | stop_active | move_ahead) need+=(voice-assistant) ;;
        esac
    done
    printf '%s\n' "${need[@]}" | awk '!seen[$0]++' | tr '\n' ' '
}

preflight() {
    local d="$OUT/preflight" need fmt host
    mkdir -p "$d"
    need=$(needs_for "$@")
    if [ -f "$STAND_LOCK" ]; then cat "$STAND_LOCK" >"$d/stand_lock.txt"; else : >"$d/stand_lock.txt"; fi
    bash "$REPO_ROOT/scripts/hil/hil.sh" status >"$d/hil_status.txt" 2>&1 || true
    fmt='{{.Name}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} {{.RestartCount}} {{.State.StartedAt}}'
    {
        rsh "$HIL_MAIN_HOST" "docker inspect -f '$fmt' nav2 twist-mux proximity-guard followpath-watchdog 2>&1"
        rsh "$HIL_SOUND_HOST" "docker inspect -f '$fmt' voice-assistant 2>&1"
        rsh "$HIL_SANDBOX_HOST" "docker inspect -f '$fmt' $GZ 2>&1"
    } >"$d/inspect.txt" || true
    {
        echo "local $(date +%s.%N)"
        for host in "$HIL_MAIN_HOST" "$HIL_SOUND_HOST" "$HIL_SANDBOX_HOST"; do
            echo "$host $(rsh "$host" 'date +%s.%N' || echo nan)"
        done
    } >"$d/clock.txt"
    local line
    line=$(verdict preflight "$d" --owner "$HIL_CHECKS_OWNER" --need "$need")
    echo "$line"
    case "$line" in
        *" preflight PASS:"*) ;;
        *) die "предусловия не выполнены — raw в $d (hil_status.txt, inspect.txt, stand_lock.txt)" ;;
    esac
}

# ── защита по лидару: наезд на препятствие ────────────────────────────────────────────────

guard_env() {
    rsh "$HIL_MAIN_HOST" "docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' proximity-guard" \
        | grep -E '^(HIL_GUARD_SOUND|USE_SIM_TIME)=' || true
}
expr_capture() {  # expr_capture <файл> <секунд>: запросы мигалки (ExpressionRequest) глазами Main Pi
    rsh "$HIL_MAIN_HOST" "docker exec -e PYTHONUNBUFFERED=1 proximity-guard bash -c 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; timeout $2 ros2 topic echo /voice/expression/request' 2>&1" >"$1" || true
}
zone_run() {  # zone_run <каталог>: наезд + raw (общая часть zone и guard_off)
    local d="$1" t0 epid
    mkdir -p "$d"
    t0=$(iso_now)
    guard_env >"$d/env.txt"
    expr_capture "$d/expr.log" 90 &
    epid=$!
    probe "$d/probe.log" wall --speed "$WALL_SPEED"
    sleep 5
    dlogs "$HIL_MAIN_HOST" proximity-guard "$t0" "$d/guard.log"
    dlogs "$HIL_SOUND_HOST" voice-assistant "$t0" "$d/voice.log"
    wait "$epid" || true
}
guard_recreate() {  # guard_recreate on|off: proximity-guard в HIL с HIL_GUARD_SOUND (команда hil.sh on, один сервис)
    local mode="$1" since maps rtab
    maps=$(rsh "$HIL_MAIN_HOST" "cat $HIL_LOCK" | sed -n 's/.* maps=\([^ ]*\).*/\1/p')
    rtab=$(rsh "$HIL_MAIN_HOST" "docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' rtabmap" \
        | sed -n 's/^RTABMAP_START_MODE=//p' | tail -n 1)
    [ -n "$maps" ] || { log "❌ в замке HIL нет maps= — proximity-guard не пересоздаю"; return 1; }
    since=$(iso_now)
    rsh "$HIL_MAIN_HOST" "cd $HIL_MAIN_DIR && HIL_MAPS_DIR=$maps HIL_RTABMAP_MODE=${rtab:-mapping} HIL_GUARD_SOUND=$mode docker compose -f docker-compose.yaml -f $HIL_REMOTE_DIR/main.images.yaml -f $HIL_REMOTE_DIR/main.hil.yaml up -d --no-deps --pull never proximity-guard" || return 1
    wait_log "$HIL_MAIN_HOST" proximity-guard "$since" 'зона под контролем' 90
}
restore_guard() {
    # Умолчание HIL — off (#4470). После явного on (сценарий zone) возвращаем off.
    if [ "${GUARD_SOUND_CHANGED:-0}" = 1 ]; then
        log "возвращаю proximity-guard в HIL_GUARD_SOUND=off (умолчание HIL)"
        mkdir -p "$OUT/zone"
        guard_recreate off >>"$OUT/zone/recreate_off.txt" 2>&1 || log "❌ proximity-guard не вернулся в off — hil.sh status"
        GUARD_SOUND_CHANGED=0
    fi
}

sc_zone() {
    # #4365 и #4470(on) требуют писк — явно on, умолчание HIL теперь off.
    local d="$OUT/zone"
    mkdir -p "$d"
    GUARD_SOUND_CHANGED=1
    if ! guard_recreate on >"$d/recreate_on.txt" 2>&1; then
        skip "#4470" guard_sound_on "proximity-guard с HIL_GUARD_SOUND=on не поднялся (recreate_on.txt)"
        restore_guard
        return 0
    fi
    zone_run "$d"
    verdict zone "$d"
    verdict guard_sound "$d" --mode on
    restore_guard
}
sc_guard_off() {
    local d="$OUT/guard_off"
    mkdir -p "$d"
    probe "$d/backoff_before.log" backoff --secs 5
    # Умолчание уже off; recreate фиксирует off на случай, если до этого zone оставил on.
    if ! guard_recreate off >"$d/recreate_off.txt" 2>&1; then
        skip "#4470" guard_sound_off "proximity-guard с HIL_GUARD_SOUND=off не поднялся (recreate_off.txt)"
        return 0
    fi
    zone_run "$d"
    verdict guard_sound "$d" --mode off
    probe "$d/backoff_after.log" backoff --secs 5
}
sc_corner() {
    # #4631: стенка в 1.5 м от носа, сдвинутая влево на 0.25 м — перекрывает только левый угол.
    # Вердикт читает зазор и точку из строки стопа: внутри коридора не меньше 0.19, вне коридора стопа быть не должно.
    local d="$OUT/corner" t0
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" corner --speed "$WALL_SPEED" --shift 0.25 --ahead 1.5
    sleep 5
    dlogs "$HIL_MAIN_HOST" proximity-guard "$t0" "$d/guard.log"
    verdict corner "$d"
}

# ── фразы тулов движения ────────────────────────────────────────────────────────────────

sc_doorway() {
    # #4365: проём 0.90 м мира polygon (x=2, коробки pose 2.000 ±1.725, размер 0.100×2.550).
    # Подъезд по оси к цели за проёмом. PASS — доехала и в проёме нет входа в stop.
    local d="$OUT/doorway" t0 lo hi
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" nav --goal "$DOOR_GOAL" --timeout 180
    sleep 2
    dlogs "$HIL_MAIN_HOST" proximity-guard "$t0" "$d/guard.log"
    lo=${DOOR_OPENING%,*}
    hi=${DOOR_OPENING#*,}
    verdict doorway "$d" --opening-lo "$lo" --opening-hi "$hi"
}
cm_count() {  # число /controller_manager глазами сторожа в gazebo; нет числа — «-», не ноль.
    local out rc=0
    out=$(rsh "$HIL_SANDBOX_HOST" "timeout ${HIL_PROBE_TIMEOUT_S:-30} docker exec $GZ /entrypoint.sh python3 -m rob_box_sim.hil count --settle 3 2>/dev/null") || rc=$?
    if [ "$rc" -eq 124 ]; then echo timeout; return 0; fi
    printf '%s\n' "$out" | grep -E '^[0-9]+$' | tail -n 1 | grep . || echo -
}
isolation_ok() {  # isolation_ok <каталог>: код решает до цели. Отказ — exit 2, цель не шлётся.
    local d="$1" line
    {
        rsh "$HIL_MAIN_HOST" "docker inspect -f '{{.Name}} {{.State.Status}}' ros2-control 2>&1 || true"
        rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{.Name}} {{.State.Status}}' $GZ 2>&1 || true"
    } >"$d/inspect.txt"
    {
        echo "── Main Pi"
        rsh "$HIL_MAIN_HOST" "docker inspect -f '{{.Name}} {{.State.Status}}' ros2-control 2>&1 || true"
        echo "── /controller_manager в графе (сторож в gazebo): $(cm_count) (норма HIL — 1)"
    } >"$d/hil_status.txt"
    rsh "$HIL_MAIN_HOST" "cat /tmp/hil/can_speed.json 2>/dev/null || true" >"$d/can_watch.json"
    line=$("$PY" -X utf8 "$(pypath "$HERE/isolation.py")" "$(pypath "$d")" --gazebo-prefix "$HIL_SIM_PREFIX")
    printf '%s\n' "$line" | tee -a "$OUT/verdicts.txt"
    case "$line" in
        "ISOLATION PASS:"*) return 0 ;;
    esac
    printf '❌ предохранитель #5160 отказал — цель не отправлена (%s)\n' "$d" >&2
    exit 2
}
sc_move_ahead() {
    # #5160: реплика в диалог, цель создаёт move_direction. Проба только слушает TRACK
    # (watch не шлёт NavigateToPose). Вердикт читает лог тула и сдвиг правды Gazebo.
    # До реплики — предохранитель: только Gazebo, боевой ros2-control стоит,
    # командных кадров VESC нет. Иначе exit 2, inject не вызывается.
    local d="$OUT/move_ahead" t0 ppid
    mkdir -p "$d"
    isolation_ok "$d"
    t0=$(iso_now)
    probe "$d/probe.log" watch --secs $((MOVE_AHEAD_WAIT_S + 30)) &
    ppid=$!
    sleep 5
    inject "$MOVE_AHEAD_PHRASE" "$d/inject.txt"
    wait_log "$HIL_SOUND_HOST" voice-assistant "$t0" \
        "Инструмент move_direction (завершился|выполнен успешно)|✅ Движение " \
        "$MOVE_AHEAD_WAIT_S" || log "⚠️  move_direction не завершился за $MOVE_AHEAD_WAIT_S с"
    sleep 5
    probe_stop
    wait "$ppid" || true
    dlogs "$HIL_SOUND_HOST" voice-assistant "$t0" "$d/voice.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict move_ahead "$d"
}

sc_nav_aborted() {
    local d="$OUT/nav_aborted" t0 ppid
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" watch --secs $((ABORT_WAIT_S + 60)) &
    ppid=$!
    sleep 5
    inject "$ABORT_PHRASE" "$d/inject.txt"
    wait_log "$HIL_SOUND_HOST" voice-assistant "$t0" "Инструмент ($NAV_TOOLS_RX) завершился|✅ (Навигация|Движение) " \
        "$ABORT_WAIT_S" || log "⚠️  тул навигации не завершился за $ABORT_WAIT_S с"
    sleep 15
    probe_stop
    wait "$ppid" || true
    dlogs "$HIL_SOUND_HOST" voice-assistant "$t0" "$d/voice.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict nav_aborted "$d"
    verdict tool_halt "$d" --grace "$ORPHAN_GRACE_S"
}

stop_by_voice() {  # stop_by_voice <каталог>: фразы по очереди, пока модель не позовёт stop_navigation
    local d="$1" p ti phrases
    IFS='|' read -r -a phrases <<<"$STOP_PHRASES"
    for p in "${phrases[@]}"; do
        ti=$(iso_now)
        inject "$p" "$d/inject.txt"
        if wait_log "$HIL_SOUND_HOST" voice-assistant "$ti" '\[stop_navigation\] Остановка навигации' 30; then
            return 0
        fi
    done
    return 1
}
sc_stop_idle() {
    local d="$OUT/stop_idle" t0
    mkdir -p "$d"
    t0=$(iso_now)
    stop_by_voice "$d" || log "⚠️  stop_navigation не вызван ни одной фразой"
    sleep 10
    dlogs "$HIL_SOUND_HOST" voice-assistant "$t0" "$d/voice.log"
    verdict stop_idle "$d"
}
sc_stop_active() {
    local d="$OUT/stop_active" t0 ppid
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" nav --ahead "$STOP_AHEAD_M" --timeout 90 &
    ppid=$!
    if wait_file "$d/probe.log" 'NAV_STATUS .*status=EXECUTING' 40; then
        sleep 3
        stop_by_voice "$d" || log "⚠️  stop_navigation не вызван ни одной фразой"
    else
        log "⚠️  цель из скрипта не пошла в EXECUTING за 40 с"
    fi
    wait "$ppid" || true
    sleep 5
    dlogs "$HIL_SOUND_HOST" voice-assistant "$t0" "$d/voice.log"
    verdict stop_active "$d"
    verdict tool_halt "$d" --grace "$ORPHAN_GRACE_S"
}

# ── сирота FollowPath ─────────────────────────────────────────────────────────────────

sc_orphan() {
    local d="$OUT/orphan" t0
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" orphan --len "$ORPHAN_LEN_M"
    sleep 2
    dlogs "$HIL_MAIN_HOST" followpath-watchdog "$t0" "$d/watchdog.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict orphan "$d" --grace "$ORPHAN_GRACE_S"
}
sc_controller_abort() {
    # #4770: FollowPath уже принят, корпус удержан — abort progress checker, не «план не построен».
    local d="$OUT/controller_abort" t0
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/probe.log" hold --ahead "$HOLD_AHEAD_M" --allowance "$PROGRESS_ALLOWANCE_S" --grace "$ORPHAN_GRACE_S"
    sleep 2
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    dlogs "$HIL_MAIN_HOST" followpath-watchdog "$t0" "$d/watchdog.log"
    verdict controller_abort "$d" --grace "$ORPHAN_GRACE_S" --allowance "$PROGRESS_ALLOWANCE_S"
}
sc_client_kill() {
    # #4435: клиент цели убит без cancel_goal. Слушатель — отдельная проба watch.
    local d="$OUT/client_kill" t0 ppid pid
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/watch.log" watch --secs 90 &
    ppid=$!
    sleep 3
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ rm -f /tmp/hil_probe.killed /tmp/hil_probe.host_cancel" || true
    rsh "$HIL_SANDBOX_HOST" "docker exec -d $GZ /entrypoint.sh python3 -u /hil_probe.py leave --ahead $HOLD_AHEAD_M --after 30" \
        >"$d/leave_start.txt" 2>&1 || true
    if ! wait_file "$d/watch.log" "CLIENT_PID" 40; then
        skip "#4435" client_kill "CLIENT_PID нет за 40 с (FollowPath не принят или проба не стартовала)"
        probe_stop
        wait "$ppid" || true
        return 0
    fi
    pid=$(grep -o -E "pid=[0-9]+" "$d/watch.log" | head -1 | cut -d= -f2)
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ sh -c \"printf %s $pid > /tmp/hil_probe.killed; kill -TERM $pid\"" \
        >"$d/kill.txt" 2>&1 || true
    sleep $((ORPHAN_GRACE_S + 8))
    probe_stop
    wait "$ppid" || true
    cat "$d/watch.log" >"$d/probe.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict client_kill "$d" --grace "$ORPHAN_GRACE_S"
}
sc_host_cancel() {
    # #4435: ros2 action cancel с хоста (все цели, нулевой GoalInfo). Не наш тул.
    local d="$OUT/host_cancel" t0 ppid
    mkdir -p "$d"
    t0=$(iso_now)
    probe "$d/watch.log" watch --secs 90 &
    ppid=$!
    sleep 3
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ rm -f /tmp/hil_probe.killed /tmp/hil_probe.host_cancel" || true
    rsh "$HIL_SANDBOX_HOST" "docker exec -d $GZ /entrypoint.sh python3 -u /hil_probe.py leave --ahead $HOLD_AHEAD_M --after 30" \
        >"$d/leave_start.txt" 2>&1 || true
    if ! wait_file "$d/watch.log" "FP_STATUS .*status=EXECUTING" 40; then
        skip "#4435" host_cancel "FollowPath не принят за 40 с — отмена с хоста не проверена"
        probe_stop
        wait "$ppid" || true
        return 0
    fi
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ /entrypoint.sh bash -lc \"source /opt/ros/humble/setup.bash; ros2 action cancel /navigate_to_pose; touch /tmp/hil_probe.host_cancel\"" \
        >"$d/cancel.txt" 2>&1 || true
    sleep $((ORPHAN_GRACE_S + 8))
    probe_stop
    wait "$ppid" || true
    cat "$d/watch.log" >"$d/probe.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict host_cancel "$d" --grace "$ORPHAN_GRACE_S"
}
sc_orphan_natural() {
    local d="$OUT/orphan_natural" t0
    if [ -z "$ORPHAN_GOAL" ]; then
        skip "#4435" orphan_natural "ORPHAN_GOAL не задан (цель зависит от мира, см. runbook)"
        return 0
    fi
    mkdir -p "$d"
    t0=$(iso_now)
    rsh "$HIL_MAIN_HOST" "cat /proc/loadavg" >"$d/load_before.txt" || true
    if [ "$ORPHAN_LOAD_S" -gt 0 ]; then
        rsh "$HIL_MAIN_HOST" "setsid nohup bash -c 'for i in 1 2 3 4; do timeout $ORPHAN_LOAD_S sh -c \"while :; do :; done\" & done; wait' </dev/null >/dev/null 2>&1 &"
    fi
    probe "$d/probe.log" nav --goal "$ORPHAN_GOAL" --timeout 180
    rsh "$HIL_MAIN_HOST" "cat /proc/loadavg" >"$d/load_after.txt" || true
    dlogs "$HIL_MAIN_HOST" followpath-watchdog "$t0" "$d/watchdog.log"
    dlogs "$HIL_MAIN_HOST" nav2 "$t0" "$d/nav2.log"
    verdict orphan_natural "$d" --grace "$ORPHAN_GRACE_S"
}

# ── запуск ──────────────────────────────────────────────────────────────────────────────

main() {
    local sel=("$@") sc
    if [ ${#sel[@]} -eq 0 ] || [ "${sel[0]}" = all ]; then sel=("${ALL[@]}"); fi
    case "${sel[0]}" in -h | --help) sed -n '2,22p' "$0"; exit 0 ;; esac
    for sc in "${sel[@]}"; do
        printf '%s\n' "${ALL[@]}" | grep -qx "$sc" || die "нет сценария '$sc'; есть: ${ALL[*]}"
    done
    mkdir -p "$OUT"
    log "raw: $OUT; сценарии: ${sel[*]}"
    preflight "${sel[@]}"
    push_probe || die "проба не доставлена в $GZ на $HIL_SANDBOX_HOST"
    trap restore_guard EXIT
    for sc in "${sel[@]}"; do
        log "── $sc"
        "sc_$sc"
    done
    log "итог ($OUT/verdicts.txt):"
    cat "$OUT/verdicts.txt"
}

main "$@"
