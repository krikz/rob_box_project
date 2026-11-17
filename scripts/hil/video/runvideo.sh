#!/bin/bash
# На песочнице: подписчики камер + рекордер карты + маршрут внутри rob-box-hil-gazebo.
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
# ROUTE_GOALS_FILE (путь в контейнере, напр. /sim_worlds/building.goals.yaml) — цели из файла точек мира
docker exec ${ROUTE_GOALS_FILE:+-e ROUTE_GOALS_FILE="$ROUTE_GOALS_FILE"} $G /entrypoint.sh python3 -u /route.py "$@" 2>&1 | grep -v -i -E "zenoh|^\s*$" > $D/out/route.log
date -u +%FT%TZ > $D/out/run_end.txt
docker exec $G pkill -INT -f rob_box_sim.map_recorder
sleep "$STOP_S"
if [ "$CAMERAS" = 1 ]; then docker exec $G pkill -f ign-transport-topic; fi
echo done
