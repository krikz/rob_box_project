#!/bin/bash
# Прогон стенда навигации по мирам: на каждый мир — чистый стек
# (up → ждём nav2 healthy → bench → down -v), чтобы карта rtabmap и состояние
# Nav2 не переходили из мира в мир.
#
# План: docs/research/2026-11-04-gazebo-sim-navigation.md §6 шаг 4.
#
# Из корня репо, образ уже собран (docker build -f docker/sim/Dockerfile ...):
#   SIM_IMAGE=rob_box_sim:local SIM_DRI_CARD=/dev/dri/card1 \
#     docker/sim/run_bench.sh polygon turn90 corridor low_obstacle local/robot_map
#
# Переменные: SIM_IMAGE, SIM_PREFIX, SIM_DRI_CARD, SIM_DRI_RENDER (см. compose.yaml),
#   SIM_PROJECT   имя compose-проекта         (rob_box_sim)
#   SIM_RESULTS_DIR  каталог результатов      (docker/sim/results)
#   SIM_BENCH_LABEL  метка прогона в JSON     (baseline)
#   SIM_UP_TIMEOUT   сколько ждать nav2, с    (240)
# Шаг 6 (сравнение контроллеров, оверлеи src/rob_box_sim/config/nav2/): SIM_CONTROLLER,
#   SIM_PLANNER, SIM_FOOTPRINT, SIM_GLOBAL_MAP, SIM_SMOOTH_CMD_VEL — см. compose.yaml;
#   повторы одной конфигурации — метка вида dwb-navfn@3 (rob_box_sim.bench_report).
set -euo pipefail

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <мир> [мир ...]   (polygon turn90 corridor low_obstacle local/robot_map)" >&2
    exit 2
fi

here="$(cd "$(dirname "$0")" && pwd)"
compose_file="$here/compose.yaml"
project="${SIM_PROJECT:-rob_box_sim}"
prefix="${SIM_PREFIX:-rob-box-sim}"
export SIM_RESULTS_DIR="${SIM_RESULTS_DIR:-$here/results}"
export SIM_UID="${SIM_UID:-$(id -u)}"
export SIM_GID="${SIM_GID:-$(id -g)}"
mkdir -p "$SIM_RESULTS_DIR"

dc() { docker compose -p "$project" -f "$compose_file" "$@"; }

rc=0
for world in "$@"; do
    log="$SIM_RESULTS_DIR/$(echo "$world" | tr '/' '_').log"
    echo "=== мир $world ($(date -u +%FT%TZ)) ==="
    export SIM_WORLD="$world"
    dc up -d --no-build
    ok=0
    for _ in $(seq 1 "$(( ${SIM_UP_TIMEOUT:-240} / 5 ))"); do
        if [ "$(docker inspect -f '{{.State.Health.Status}}' "$prefix-nav2" 2>/dev/null)" = healthy ]; then
            ok=1
            break
        fi
        sleep 5
    done
    if [ "$ok" -ne 1 ]; then
        echo "❌ $world: nav2 не стал healthy за ${SIM_UP_TIMEOUT:-240} с"
        dc ps -a
        rc=1
    elif ! dc --profile bench run --rm --no-deps bench 2>&1 | tee "$log"; then
        echo "❌ $world: стенд упал (лог $log)"
        rc=1
    fi
    dc logs --no-color --tail 400 gazebo nav2 rtabmap > "$SIM_RESULTS_DIR/$(echo "$world" | tr '/' '_').stack.log" 2>&1 || true
    dc --profile bench down -v
done
exit "$rc"
