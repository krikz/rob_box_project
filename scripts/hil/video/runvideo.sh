#!/bin/bash
# На песочнице: подписчики камер + рекордер карты + маршрут внутри rob-box-hil-gazebo.
# Код выхода — код route.py, не grep: без pipefail пайп отдавал 0, хотя маршрут
# падал до первой цели (пустые цели, assert, traceback). Нет GOAL_START в логе —
# «ERROR before GOAL_START» и ненулевой код, даже если route.py вышел 0.
set -o pipefail
G=rob-box-hil-gazebo
D=${VIDEO_DIR:-/home/ros2/rob_box_hilvideo}
# CAMERAS=1: подписчики камер video_top/video_chase (сенсор рендерит, на программном рендере песочницы
# RTF ~0.3); CAMERAS=0: без них RTF ~1.0, кадров камер нет. SETTLE_S/STOP_S — паузы (в тестах 0).
CAMERAS=${CAMERAS:-1}
SETTLE_S=${SETTLE_S:-20}
STOP_S=${STOP_S:-8}
case "$CAMERAS" in 0 | 1) ;; *) echo "CAMERAS='$CAMERAS' — нужно 0 или 1" >&2; exit 2 ;; esac
echo "CAMERAS=$CAMERAS"
docker cp $D/route.py $G:/route.py
if [ "$CAMERAS" = 0 ]; then
    # подписчики от прошлого прогона (имя процесса в контейнере — ign-transport-topic) держат рендер камер
    docker exec $G pkill -f ign-transport-topic || true
fi
if [ "$CAMERAS" = 1 ]; then
    docker exec -d $G /entrypoint.sh bash -c 'ign topic -e -t /sim/video/video_top >/dev/null & ign topic -e -t /sim/video/video_chase >/dev/null & wait'
fi
docker exec -d $G /entrypoint.sh python3 -m rob_box_sim.map_recorder --out /sim_video/maprec --rate 10
sleep "$SETTLE_S"
date -u +%FT%TZ > $D/out/run_start.txt
# Гейт twist_mux — только по явному opt-in (HIL_MUX_GATE=1). Без флага поведение не меняется.
# Включён: до первой цели вколоть vx=0.1 и сверить выход mux. rc 1/2 — маршрут не слать.
# Скрипт сам отказывает вне контейнера HIL с живым замком /tmp/hil.lock (скорость не публикует).
if [ "${HIL_MUX_GATE:-0}" = 1 ]; then
    MUX_GATE_DIR="${HIL_MUX_GATE_DIR:-$D/out/mux_gate}"
    mkdir -p "$MUX_GATE_DIR"
    docker exec $G mkdir -p /mux_gate_out /hil_checks
    docker cp "$D/checks/mux_gate.py" $G:/hil_checks/mux_gate.py
    docker cp "$D/mux_gate.py" $G:/mux_gate.py
    docker exec -e HIL_LOCK_FILE="${HIL_LOCK_FILE:-/tmp/hil.lock}" -e HIL_MUX_GATE_PY=/hil_checks/mux_gate.py $G \
        /entrypoint.sh python3 -u /mux_gate.py run /mux_gate_out > "$MUX_GATE_DIR/gate.log" 2>&1
    gate_rc=$?
    docker cp $G:/mux_gate_out/capture.txt "$MUX_GATE_DIR/capture.txt" 2>/dev/null || true
    docker cp $G:/mux_gate_out/raw.txt "$MUX_GATE_DIR/raw.txt" 2>/dev/null || true
    if [ "$gate_rc" -ne 0 ]; then
        echo "MUX_GATE: rc=${gate_rc}, маршрут не послан (capture: $MUX_GATE_DIR)" >&2
        tail -n 20 "$MUX_GATE_DIR/gate.log" >&2 || true
        exit "$gate_rc"
    fi
    echo "MUX_GATE: PASS"
fi
# ROUTE_GOALS_FILE (путь в контейнере, напр. /sim_worlds/building.goals.yaml) — цели из файла точек мира.
# ROUTE_LOG — файл этого run_id. Пусто — прежний общий out/route.log (и он же, если задан ROUTE_LOG:
# вызывающий читает оба; m1repeat копирует общий лог только если он не старше старта).
ROUTE_LOG_PATH="${ROUTE_LOG:-$D/out/route.log}"
mkdir -p "$(dirname "$ROUTE_LOG_PATH")" "$D/out"
# pipefail: код пайпа — первый ненулевой. docker exec (route.py) стоит первым, grep — вторым,
# поэтому сбой route.py не маскируется успешным grep. zenoh и пустые строки в лог не пишем.
docker exec ${ROUTE_GOALS_FILE:+-e ROUTE_GOALS_FILE="$ROUTE_GOALS_FILE"} $G /entrypoint.sh python3 -u /route.py "$@" 2>&1 \
    | grep -v -i -E "zenoh|^\s*$" > "$ROUTE_LOG_PATH"
# PIPESTATUS живёт только до следующей команды: сохранить сразу, до cp.
route_rc=${PIPESTATUS[0]}
if [ -n "${ROUTE_LOG:-}" ] && [ "$ROUTE_LOG_PATH" != "$D/out/route.log" ]; then
    cp -f "$ROUTE_LOG_PATH" "$D/out/route.log" || true
fi
date -u +%FT%TZ > $D/out/run_end.txt
docker exec $G pkill -INT -f rob_box_sim.map_recorder || true
sleep "$STOP_S"
if [ "$CAMERAS" = 1 ]; then docker exec $G pkill -f ign-transport-topic || true; fi
# Координатор смотрит код обёртки. Нет GOAL_START — маршрут умер до первой цели
# (ошибка, пустой список, падение route.py): это не успешный прогон.
if ! grep -q "GOAL_START" "$ROUTE_LOG_PATH" 2>/dev/null; then
    reason=$(grep -E "Error|error|Exception|Traceback|AssertionError|assert|нет целей" "$ROUTE_LOG_PATH" 2>/dev/null | tail -n 1)
    [ -n "$reason" ] || reason="route.py rc=${route_rc}, лог без GOAL_START"
    echo "ERROR before GOAL_START: ${reason}" >&2
    [ "$route_rc" -ne 0 ] && exit "$route_rc"
    exit 1
fi
[ "$route_rc" -eq 0 ] || exit "$route_rc"
echo done
