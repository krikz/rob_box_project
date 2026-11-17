#!/bin/bash
# На песочнице: исполнитель шагов прогона здания (#4468) в rob-box-hil-gazebo. Зовёт run_building.sh по ssh.
#
#   building_remote.sh push                     route.py, building_step.py и pylib/rob_box_sim → в контейнер
#   building_remote.sh start <метка> <арг…>     шаг building_step.py в фоне: лог out/building/steps/<метка>.log,
#                                               код выхода — out/building/steps/<метка>.rc (по нему ждёт оркестр)
#   building_remote.sh maprec_start <каталог> [карта: 1|0]   рекордер поз (и снимков /rtabmap/map при 1)
#   building_remote.sh maprec_stop              SIGINT рекордеру: дописывает poses.csv и meta.json
#   building_remote.sh pack                     tar в stdout: out/building (шаги) + /tmp/building из контейнера
#
# Переменные: BUILDING_DIR (каталог копии, по умолчанию — где лежит скрипт), G (rob-box-hil-gazebo).
# Шум zenoh из логов отрезается, как в runvideo.sh. Модули rob_box_sim — из pylib копии (PYTHONPATH=/cov):
# образ сима собран раньше, чем появились building_floor и building_run.
set -uo pipefail
D="${BUILDING_DIR:-$(cd "$(dirname "$0")" && pwd)}"
G="${G:-rob-box-hil-gazebo}"
GOALS="${ROUTE_GOALS_FILE:-/sim_worlds/building.goals.yaml}"
STEPS="$D/out/building/steps"

case "${1:-}" in
    push)
        docker cp "$D/route.py" "$G:/route.py" \
            && docker cp "$D/building_step.py" "$G:/building_step.py" \
            && docker exec "$G" mkdir -p /cov /tmp/building \
            && docker cp "$D/pylib/rob_box_sim" "$G:/cov/" || exit 1
        docker exec "$G" pkill -f ign-transport-topic || true  # CAMERAS=0: подписчики камер держат рендер
        docker exec "$G" test -f /cov/rob_box_sim/building_run.py
        ;;
    run)  # run <метка> <арг…> — синхронно (его зовёт start в фоне)
        label="$2"
        shift 2
        mkdir -p "$STEPS"
        # shellcheck disable=SC2016
        docker exec -e ROUTE_GOALS_FILE="$GOALS" "$G" /entrypoint.sh bash -c \
            'PYTHONPATH=/cov:$PYTHONPATH exec python3 -u /building_step.py "$@"' _ "$@" 2>&1 \
            | grep --line-buffered -v -i -E "zenoh|^\s*$" > "$STEPS/$label.log"
        echo "${PIPESTATUS[0]}" > "$STEPS/$label.rc"
        ;;
    start)
        mkdir -p "$STEPS"
        rm -f "$STEPS/$2.rc"
        nohup bash "$0" run "${@:2}" > /dev/null 2>&1 < /dev/null &
        echo "started $2 pid=$!"
        ;;
    maprec_start)  # уже пишет (блоки по одному) — второй не нужен: два рекордера дали бы два ряда поз
        if docker exec "$G" pgrep -f rob_box_sim.map_recorder > /dev/null; then
            echo "maprec уже пишет"
            exit 0
        fi
        dir="/tmp/building/${2:?каталог}"
        topic=/rtabmap/map
        [ "${3:-1}" = 1 ] || topic=/building/no_map  # только позы: снимки карты здания за часы — сотни МБ
        # shellcheck disable=SC2016
        docker exec -d "$G" /entrypoint.sh bash -c \
            'PYTHONPATH=/cov:$PYTHONPATH exec python3 -m rob_box_sim.map_recorder --out "$1" --rate 10 --map-topic "$2"' \
            _ "$dir" "$topic"
        ;;
    maprec_stop)
        docker exec "$G" pkill -INT -f rob_box_sim.map_recorder || true
        sleep 8
        ;;
    pack)
        tmp="$D/out/building/container"
        rm -rf "$tmp" && mkdir -p "$tmp"
        docker cp "$G:/tmp/building/." "$tmp/" >/dev/null 2>&1 || echo "pack: /tmp/building в контейнере нет" >&2
        tar cf - -C "$D/out/building" .
        ;;
    *)
        sed -n '2,10p' "$0"
        exit 2
        ;;
esac
