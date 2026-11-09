#!/bin/bash
# На песочнице: подписчики камер + рекордер карты + маршрут внутри rob-box-hil-gazebo.
G=rob-box-hil-gazebo
D=/home/ros2/rob_box_hilvideo
docker cp $D/route.py $G:/route.py
docker exec -d $G /entrypoint.sh bash -c 'ign topic -e -t /sim/video/video_top >/dev/null & ign topic -e -t /sim/video/video_chase >/dev/null & wait'
docker exec -d $G /entrypoint.sh python3 -m rob_box_sim.map_recorder --out /sim_video/maprec --rate 10
sleep 20
date -u +%FT%TZ > $D/out/run_start.txt
# ROUTE_GOALS_FILE (путь в контейнере, напр. /sim_worlds/building.goals.yaml) — цели из файла точек мира
docker exec ${ROUTE_GOALS_FILE:+-e ROUTE_GOALS_FILE="$ROUTE_GOALS_FILE"} $G /entrypoint.sh python3 -u /route.py "$@" 2>&1 | grep -v -i -E "zenoh|^\s*$" > $D/out/route.log
date -u +%FT%TZ > $D/out/run_end.txt
docker exec $G pkill -INT -f rob_box_sim.map_recorder
sleep 8
docker exec $G pkill -f "ign topic"
echo done
