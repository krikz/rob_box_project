#!/bin/bash
# Стенд локализации после перезапуска в Gazebo (#4346, вариант D; #4468 «Локализация»).
#
# Две команды:
#   map <метка> <мир>
#       картирование мира с нуля (rtabmap mapping, маршрут scenarios/mapping.yaml через Nav2);
#       после остановки rtabmap база → $SIM_LOC_ROOT/dbs/<метка>.db, истинная поза в конце
#       маршрута → dbs/<метка>.end («x y yaw»), сводка базы (узлы, ориентиры) → dbs/<метка>.dbinfo.
#   loc <метка> <мир> <база> <сценарий>
#       «перезапуск»: свежая копия dbs/<база>.db в /maps, спавн по сценарию
#       scenarios/localization.yaml (spawn_offset от dbs/<база>.end или spawn_abs), стек без Nav2
#       (gazebo, rsp, twist-mux, imu-filter, rtabmap в localization, при SIM_CAMERA=true — apriltag),
#       стенд rob_box_sim.loc_bench; «похищение» — ign service set_pose по запросу стенда.
#       Результаты: $SIM_LOC_ROOT/runs/<метка>/results/<метка>.json, _poses.csv, логи.
#
# Камера и теги: SIM_CAMERA=true (+ мир polygon_tags) включает камеру и apriltag_node.
# SIM_RELOC=true (только loc, с камерой) — прототип варианта B, сервис tagreloc.
# Остальные переменные compose (SIM_TAG_LINEAR_VARIANCE, SIM_APRILTAG_SIZE, SIM_CAMERA_YAW_ERROR)
# берутся из окружения как есть.
#
# Из корня копии репо на песочнице, образ собран:
#   SIM_IMAGE=shidi-loc-rob-box-sim:4346 SIM_DRI_CARD=/dev/dri/card1 \
#     docker/sim/run_loc_bench.sh map m_notag polygon
#   ... docker/sim/run_loc_bench.sh loc a1 polygon m_notag restart_same
#
#   SIM_LOC_ROOT   каталог баз и прогонов   (docker/sim/loc)
#   SIM_PROJECT    compose-проект           (rob_box_sim_loc)
#   SIM_PREFIX     префикс контейнеров      (rob-box-sim-loc)
#   SIM_UP_TIMEOUT сколько ждать nav2, с    (240)
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
root="${SIM_LOC_ROOT:-$here/loc}"
project="${SIM_PROJECT:-rob_box_sim_loc}"
export SIM_PREFIX="${SIM_PREFIX:-rob-box-sim-loc}"
export SIM_UID="${SIM_UID:-$(id -u)}" SIM_GID="${SIM_GID:-$(id -g)}"
scenarios="$repo/src/rob_box_sim/scenarios/localization.yaml"
mkdir -p "$root/dbs" "$root/runs"

# bench/locbench/tagreloc (профиль bench) запускаются явно по имени; в `up` без имён их нет.
profiles=()
log_services=(gazebo rtabmap)
if [ "${SIM_CAMERA:-false}" = true ]; then
    profiles+=(--profile tags)
    log_services+=(apriltag)
fi
# Прототип варианта B (тег → /rtabmap/initialpose), только с камерой: SIM_RELOC=true
if [ "${SIM_RELOC:-false}" = true ]; then
    [ "${SIM_CAMERA:-false}" = true ] || { echo "❌ SIM_RELOC=true без SIM_CAMERA=true" >&2; exit 1; }
    profiles+=(--profile reloc)
    log_services+=(tagreloc)
fi

dc() { docker compose -p "$project" -f "$here/compose.yaml" -f "$here/compose.loc.yaml" "${profiles[@]}" "$@"; }
# гасит весь свой проект, включая сервисы профилей (bench, tags, reloc)
dc_down() {
    docker compose -p "$project" -f "$here/compose.yaml" -f "$here/compose.loc.yaml" --profile bench --profile tags --profile reloc down -v --remove-orphans > /dev/null 2>&1
}

die() { echo "❌ $*" >&2; exit 1; }

wait_healthy() {  # <контейнер>
    for _ in $(seq 1 "$(( ${SIM_UP_TIMEOUT:-240} / 5 ))"); do
        [ "$(docker inspect -f '{{.State.Health.Status}}' "$1" 2>/dev/null)" = healthy ] && return 0
        sleep 5
    done
    return 1
}

db_info() {  # <rtabmap.db> → узлы, ориентиры (узлы с id < 0), связи-ориентиры (Link.type = 11)
    python3 - "$1" <<'PY'
import sqlite3, sys
c = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
q = lambda s: c.execute(s).fetchone()[0]
print(f"nodes={q('select count(*) from Node')} landmarks={q('select count(distinct to_id) from Link where to_id < 0')} "
      f"landmark_links={q('select count(*) from Link where type = 11')}")
PY
}

spawn_for() {  # <сценарий> <файл .end> → «x y yaw»
    python3 - "$scenarios" "$1" "$2" <<'PY'
import math, sys, yaml
doc = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
sc = doc["scenarios"][sys.argv[2]]
if "spawn_abs" in sc:
    x, y, yaw = sc["spawn_abs"]
else:
    ex, ey, eyaw = (float(v) for v in open(sys.argv[3]).read().split())
    dx, dy, dyaw = sc["spawn_offset"]
    x, y, yaw = ex + dx, ey + dy, eyaw + dyaw
print(f"{x:.4f} {y:.4f} {math.atan2(math.sin(yaw), math.cos(yaw)):.4f}")
PY
}

teleport() {  # <мир> <x> <y> <yaw>
    local qz qw
    qz=$(python3 -c "import math; print(f'{math.sin($4 / 2):.6f}')")
    qw=$(python3 -c "import math; print(f'{math.cos($4 / 2):.6f}')")
    docker exec "$SIM_PREFIX-gazebo" bash -c "source /opt/ros/humble/setup.bash && ign service -s /world/$1/set_pose \
        --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 \
        --req 'name: \"rob_box\", position: {x: $2, y: $3, z: 0.05}, orientation: {z: $qz, w: $qw}'"
}

cmd_map() {  # <метка> <мир>
    local label="$1" world="$2" run="$root/runs/$1"
    mkdir -p "$run/maps" "$run/results"
    export SIM_WORLD="$world" SIM_MAPS_DIR="$run/maps" SIM_RESULTS_DIR="$run/results" SIM_RTABMAP_MODE=mapping
    export SIM_SPAWN_X=0.0 SIM_SPAWN_Y=0.0 SIM_SPAWN_YAW=0.0
    echo "=== map $label: $world ($(date -u +%FT%TZ)) camera=${SIM_CAMERA:-false} var=${SIM_TAG_LINEAR_VARIANCE:-default}"
    dc up -d --no-build
    wait_healthy "$SIM_PREFIX-nav2" || { dc ps -a; dc_down; die "$label: nav2 не стал healthy"; }
    dc run --rm --no-deps bench /ros_scripts/ros_with_namespace.sh python3 -m rob_box_sim.nav_bench \
        --world "$world" --scenarios /ws/install/rob_box_sim/share/rob_box_sim/scenarios/mapping.yaml \
        --label "$label" > "$run/bench.log" 2>&1 || echo "⚠️  $label: nav_bench rc≠0 ($run/bench.log)"
    grep -E '^\[bench\] (\{|MAP [0-9])' "$run/bench.log" || true
    # rtabmap пишет базу (и последнюю позу) при остановке
    docker stop -t 60 "$SIM_PREFIX-rtabmap" > /dev/null
    dc logs --no-color --tail 600 "${log_services[@]}" > "$run/stack.log" 2>&1 || true
    dc_down
    [ -s "$run/maps/rtabmap.db" ] || die "$label: rtabmap.db не записана"
    cp "$run/maps/rtabmap.db" "$root/dbs/$label.db"
    local truth="$run/results/${world}_home_truth.csv"
    [ -s "$truth" ] || die "$label: нет $truth (маршрут не доехал домой)"
    tail -1 "$truth" | awk -F, '{print $2, $3, $4}' > "$root/dbs/$label.end"
    db_info "$root/dbs/$label.db" | tee "$root/dbs/$label.dbinfo"
    echo "end pose (truth): $(cat "$root/dbs/$label.end")"
}

cmd_loc() {  # <метка> <мир> <база> <сценарий>
    local label="$1" world="$2" db="$3" scenario="$4" run="$root/runs/$1"
    [ -s "$root/dbs/$db.db" ] || die "нет базы $root/dbs/$db.db"
    mkdir -p "$run/maps" "$run/results"
    cp "$root/dbs/$db.db" "$run/maps/rtabmap.db"
    read -r sx sy syaw < <(spawn_for "$scenario" "$root/dbs/$db.end")
    export SIM_WORLD="$world" SIM_MAPS_DIR="$run/maps" SIM_RESULTS_DIR="$run/results" SIM_RTABMAP_MODE=localization
    export SIM_SPAWN_X="$sx" SIM_SPAWN_Y="$sy" SIM_SPAWN_YAW="$syaw" SIM_LOC_SCENARIO="$scenario" SIM_LOC_LABEL="$label"
    echo "=== loc $label: $world db=$db $scenario spawn=($sx, $sy, $syaw) ($(date -u +%FT%TZ)) camera=${SIM_CAMERA:-false} reloc=${SIM_RELOC:-false}"
    local services=(gazebo robot-state-publisher twist-mux imu-filter rtabmap)
    [ "${SIM_CAMERA:-false}" = true ] && services+=(apriltag)
    [ "${SIM_RELOC:-false}" = true ] && services+=(tagreloc)
    dc up -d --no-build "${services[@]}"
    dc run --rm --no-deps locbench > "$run/locbench.log" 2>&1 &
    local pid=$! kid="$run/results/$label.kidnap"
    while kill -0 "$pid" 2>/dev/null; do
        if [ -f "$kid" ]; then
            read -r kx ky kyaw < "$kid"
            echo "teleport → ($kx, $ky, $kyaw)"
            teleport "$world" "$kx" "$ky" "$kyaw" || echo "⚠️  teleport rc≠0"
            mv "$kid" "$kid.done"
        fi
        sleep 1
    done
    wait "$pid" || echo "⚠️  $label: loc_bench rc≠0 ($run/locbench.log)"
    grep '^\[loc\] RESULT' "$run/locbench.log" | cut -c1-400 || true
    dc logs --no-color --tail 600 "${log_services[@]}" > "$run/stack.log" 2>&1 || true
    grep -E 'last localization|correction|Rtabmap.cpp.*(Localization|rejected)' "$run/stack.log" | head -5 || true
    dc_down
}

case "${1:-}" in
    map) [ "$#" -eq 3 ] || die "usage: $0 map <метка> <мир>"; cmd_map "$2" "$3" ;;
    loc) [ "$#" -eq 5 ] || die "usage: $0 loc <метка> <мир> <база> <сценарий>"; cmd_loc "$2" "$3" "$4" "$5" ;;
    *) die "usage: $0 map <метка> <мир> | loc <метка> <мир> <база> <сценарий>" ;;
esac
