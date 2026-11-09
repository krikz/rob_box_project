#!/bin/bash
# На песочнице: обход покрытием (#4560) внутри rob-box-hil-gazebo — как runvideo.sh, но вместо route.py
# coverage_run.py. Модули плана и метрики (rob_box_sim.coverage_*) идут из копии репо $D/pylib
# (sandbox_prep.sh), а не из образа: образ собран раньше, чем они появились.
#   ./runcoverage.sh [аргументы coverage_run.py, например --plan-only или --max-poses 20]
# Итог: out/coverage.log (PLAN, UNREACHABLE, GOAL_*, TRACK, EM_*, COVERAGE_DONE) и out/coverage/*.png|json.
G=rob-box-hil-gazebo
D=/home/ros2/rob_box_hilvideo
docker cp $D/route.py $G:/route.py
docker cp $D/coverage_run.py $G:/coverage_run.py
docker exec $G mkdir -p /cov
docker cp $D/pylib/rob_box_sim $G:/cov/
docker exec -d $G /entrypoint.sh bash -c 'ign topic -e -t /sim/video/video_top >/dev/null & ign topic -e -t /sim/video/video_chase >/dev/null & wait'
docker exec -d $G /entrypoint.sh python3 -m rob_box_sim.map_recorder --out /sim_video/maprec --rate 10
sleep 20
date -u +%FT%TZ > $D/out/run_start.txt
# PYTHONPATH — после entrypoint: setup.bash /ws ставит старый rob_box_sim из образа впереди.
# shellcheck disable=SC2016
docker exec $G /entrypoint.sh bash -c \
    'PYTHONPATH=/cov:$PYTHONPATH exec python3 -u /coverage_run.py --out /sim_video/coverage "$@"' _ "$@" 2>&1 \
    | grep --line-buffered -v -i -E "zenoh|^\s*$" > $D/out/coverage.log
date -u +%FT%TZ > $D/out/run_end.txt
docker exec $G pkill -INT -f rob_box_sim.map_recorder
sleep 8
docker exec $G pkill -f "ign topic"
echo done
