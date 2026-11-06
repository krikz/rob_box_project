#!/bin/bash
# Видео картографии в Gazebo: rtabmap строит карту мира с нуля, пока робот
# объезжает мир через боевой Nav2; на видео сверху — стены мира, растущая карта
# /rtabmap/map полупрозрачным слоем, истинная поза (зелёная) и оценка
# map→base_footprint (красная) со следами, подпись: время, ошибка позы, IoU карты.
#
#   * rtabmap — SIM_RTABMAP_MODE=mapping (start_rtabmap.sh: localization:=false, пустая база);
#   * маршрут — scenarios/mapping.yaml, едет стенд nav_bench (compose.mapping.yaml);
#   * запись — rob_box_sim.map_recorder (сервис maprec), видео — rob_box_sim.map_video.
# Миры из карты квартиры (local/...) не снимаются: приватно.
#
# Из корня репо, образ уже собран (docker build -f docker/sim/Dockerfile ...):
#   SIM_IMAGE=rob_box_sim:local SIM_DRI_CARD=/dev/dri/card1 \
#     docker/sim/record_mapping.sh polygon corridor
#
# Переменные (кроме SIM_IMAGE/SIM_PREFIX/SIM_DRI_* из compose.yaml):
#   SIM_PROJECT       имя compose-проекта           (rob_box_sim_mapping)
#   SIM_MAP_OUT       куда класть видео и записи    (docker/sim/mapping)
#   SIM_CONTROLLER    контроллер Nav2               (rpp: ровнее всего, шаг 6)
#   SIM_FOOTPRINT     габарит Nav2                  (urdf: проходит все проёмы полигона)
#   SIM_MAP_STEP_S    шаг кадров, с sim time        (0.5)
#   SIM_MAP_FPS       кадров в секунду видео        (10)
#   SIM_MAP_MAX_MB    предел mp4 и gif, МБ          (10)
#   SIM_UP_TIMEOUT    сколько ждать nav2, с         (240)
set -euo pipefail

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <мир> [мир ...]   (polygon turn90 corridor)" >&2
    exit 2
fi

here="$(cd "$(dirname "$0")" && pwd)"
project="${SIM_PROJECT:-rob_box_sim_mapping}"
prefix="${SIM_PREFIX:-rob-box-sim-mapping}"
out_root="${SIM_MAP_OUT:-$here/mapping}"
export SIM_PREFIX="$prefix"
export SIM_RTABMAP_MODE=mapping
export SIM_CONTROLLER="${SIM_CONTROLLER:-rpp}"
export SIM_FOOTPRINT="${SIM_FOOTPRINT:-urdf}"
export SIM_UID="${SIM_UID:-$(id -u)}"
export SIM_GID="${SIM_GID:-$(id -g)}"

dc() { docker compose -p "$project" -f "$here/compose.yaml" -f "$here/compose.mapping.yaml" "$@"; }

rc=0
for world in "$@"; do
    case "$world" in
        local/* | local)
            echo "❌ $world: карту квартиры не снимаем" >&2
            rc=1
            continue
            ;;
    esac
    out="$out_root/$world"
    mkdir -p "$out/results"
    export SIM_WORLD="$world" SIM_RESULTS_DIR="$out/results"
    echo "=== картография $world ($(date -u +%FT%TZ)) → $out ==="
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
        dc --profile bench down -v
        rc=1
        continue
    fi

    dc --profile bench up -d --no-deps maprec
    bench_rc=0
    dc --profile bench run --rm --no-deps bench > "$out/bench.log" 2>&1 || bench_rc=$?
    grep -E '^\[bench\] (\{|MAP [0-9]|Nav2)' "$out/bench.log" || true
    [ "$bench_rc" -eq 0 ] || { echo "❌ $world: стенд упал ($out/bench.log)"; rc=1; }
    # SIGTERM: рекордер дописывает meta.json (RTF) и выходит.
    docker stop -t 30 "$prefix-maprec" > /dev/null
    docker logs "$prefix-maprec" > "$out/maprec.log" 2>&1 || true
    grep '^\[maprec\]' "$out/maprec.log" || true

    # Коридор 36 м: на видео — только участок езды ± 3 м, иначе он в одну полоску.
    crop=""
    [ "$world" = corridor ] && crop=3
    if [ -f "$out/results/maprec/meta.json" ]; then
        dc --profile bench run --rm --no-deps maprec /ros_scripts/ros_with_namespace.sh \
            python3 -m rob_box_sim.map_video --rec /results/maprec --world "/sim_worlds/$world.sdf" \
            --out "/results/${world}_mapping" --title "$world, rtabmap mapping" \
            --step "${SIM_MAP_STEP_S:-0.5}" --fps "${SIM_MAP_FPS:-10}" --max-mb "${SIM_MAP_MAX_MB:-10}"             ${crop:+--crop "$crop"} \
            || { echo "❌ $world: видео не собралось"; rc=1; }
    else
        echo "❌ $world: рекордер не записал meta.json ($out/maprec.log)"
        rc=1
    fi
    dc logs --no-color --tail 400 gazebo rtabmap nav2 > "$out/stack.log" 2>&1 || true
    dc --profile bench down -v
    ls -l "$out/results"
done
exit "$rc"
