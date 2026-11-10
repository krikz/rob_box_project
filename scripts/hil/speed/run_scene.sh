#!/usr/bin/env bash
# Набор «предельная скорость» (решение Шифу 09.11): один прогон одной сцены на одной скорости в HIL.
#
#   scripts/hil/speed/run_scene.sh <сцена> <скорость|base> <повтор>
#     сцена: polygon (проёмы 0.90–1.10 туда и обратно), turn90 (Г-поворот), depot (внезапная стенка + прямая 2×≈20 м),
#            dead_end_out (спавн → разворот к выходу), dead_end_back (из тупика назад)
#     base — боевой nav2_params.yaml без оверлея скорости (точка отсчёта)
#
# Дополнительные оверлеи и зоны скорости (решение Шифу 10.11):
#   SPEED_EXTRA_OVERLAYS='a.yaml b.yaml' — локальные yaml → /tmp/hil/nav2tune/, кладутся в Nav2 после speed.yaml;
#   SPEED_ZONES=1 — включить зоны: маска SpeedFilter (world-режим из sdf сцены, либо SPEED_MASK_LOCAL/ SPEED_MASK_NPZ),
#                 оверлей берётся из docker/main/config/nav2/speed_zones.yaml репо, публикаторы маски — docker exec
#                 в контейнер nav2 (боевой checkout на Main Pi старый, штатный блок start_nav2_direct.sh недоступен).
#   SPEED_MASK_LOCAL=<yaml> — готовая маска (pgm рядом), SPEED_MASK_NPZ=<npz> — снимок карты для depot (world-кадр).
#   Для зон speed — базовая скорость зала (0.35): маска только понижает её у стен/в тесноте.
#
# HIL уже включён (scripts/hil/hil.sh on с HIL_SANDBOX_REPO=$SPEED_SANDBOX_DIR после sandbox_prep.sh).
# Сброс позы: gazebo пересоздаётся со спавном сцены, на Main Pi пересоздаются сервисы с sim time
# (свежая карта rtabmap в своём каталоге, свежие costmap, оверлей Nav2 со скоростью).
# Пишет $SPEED_OUT/<метка>/: result.json, track.csv, pose_err.csv, bench.out, guard.log, nav2.log,
# nav2_missed.txt, cpu.log, main.json, verdict.txt; строку вердикта — в $SPEED_OUT/ledger.jsonl.
# Только ключи (BatchMode), пароли не используются.
set -uo pipefail
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8

scene="$1"; speed="$2"; rep="$3"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
# Windows-python не понимает /d/… при MSYS_NO_PATHCONV=1
PYHERE="$(cygpath -m "$HERE" 2>/dev/null || echo "$HERE")"
: "${SPEED_OUT:?SPEED_OUT — каталог результатов на машине оператора}"
: "${SPEED_MAIN:=ros2@10.1.1.10}"
: "${SPEED_SANDBOX:=ros2@10.1.1.250}"
: "${SPEED_SANDBOX_DIR:=/home/ros2/rob_box_hilvideo}"
: "${SPEED_ROBOT_ID:=RBXU100001}"
: "${SPEED_MAPS_ROOT:=/home/ros2/rob_box_hil/maps_speed}"
: "${SPEED_BENCH_TIMEOUT:=1800}"
G=rob-box-hil-gazebo
label="${scene}_v${speed}_r${rep}_$(date -u +%H%M%S)"
RES="$SPEED_OUT/$label"
mkdir -p "$RES"
SSHO=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=6)
log() { printf '[speed %s %s] %s\n' "$label" "$(date -u +%H:%M:%SZ)" "$*" | tee -a "$RES/run.log"; }
sb() { ssh "${SSHO[@]}" "$SPEED_SANDBOX" "$*" </dev/null; }
mn() { ssh "${SSHO[@]}" "$SPEED_MAIN" "$*" </dev/null; }

# ── сцена: мир сима, спавн (мир), задачи speed_bench ──
case "$scene" in
    polygon)
        world=polygon; spawn="0 0 0"
        tasks="g:E090:3,0,0@150;g:E095:5,0,0@150;g:E100:7,0,0@150;g:E110:9,0,0@150"
        tasks="$tasks;g:W110:7,0,3.1416@100;g:W100:5,0,3.1416@100;g:W095:3,0,3.1416@100;g:W090:1,0,3.1416@100" ;;
    turn90)
        world=turn90; spawn="0 0 0"; tasks="g:T90:3.6,4.0,1.5708@150" ;;
    depot)
        # полоса y=1.3 свободна от x −3 до 23 (карта покрытия 09.11: ±0.75 м без занятых клеток)
        world=depot; spawn="-2.5 1.3 0"
        tasks="o:OBST:22,1.3,0@200,3.0,1.5;g:DE:22,1.3,0@240;g:DW:-2,1.3,3.1416@300" ;;
    dead_end_out)
        world=dead_end; spawn="0 0 0"; tasks="g:turn_out:-2.5,0,3.1416@100" ;;
    dead_end_back)
        world=dead_end; spawn="-2.5 0 3.1416"; tasks="g:back_in:0,0,0@100" ;;
    *) echo "сцена '$scene': polygon | turn90 | depot | dead_end_out | dead_end_back" >&2; exit 2 ;;
esac
read -r SX SY SYAW <<<"$spawn"
# Таймауты целей рассчитаны на 0.15 м/с и выше; ниже — растягиваются в 0.15/v раз, чтобы медленный робот не
# падал по таймауту набора (Depot: 20 м на 0.1 м/с — 200 с хода при таймауте 240).
if [ "$speed" != base ]; then
    tasks=$(python -c "
import re, sys
k = max(1.0, 0.15 / float(sys.argv[2]))
print(re.sub(r'@(\d+(?:\.\d+)?)', lambda m: '@%d' % round(float(m.group(1)) * k), sys.argv[1]))" "$tasks" "$speed")
fi
if [ "$world" = depot ]; then
    sim_cmd="python3 /mkdepot.py && exec /hil_run.sh world:=/sim_video/world.sdf x:=$SX y:=$SY yaw:=$SYAW"
else
    sim_cmd="exec /hil_run.sh world:=/sim_worlds/$world.sdf x:=$SX y:=$SY yaw:=$SYAW"
fi

# ── оверлей Nav2: скорость и ускорения пропорционально (разгон до max_vel_x за 1 с) ──
overlay=""
vmax=0.15
if [ "$speed" != base ]; then
    vmax="$speed"
    acc=$(python -c "print(round(min(max($speed, 0.15), 1.0), 3))")
    cat >"$RES/speed.yaml" <<EOF
# scripts/hil/speed: только HIL, боевой файл не меняется
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: $speed
      max_speed_xy: $speed
      acc_lim_x: $acc
      decel_lim_x: -$acc
EOF
    ssh "${SSHO[@]}" "$SPEED_MAIN" "mkdir -p /tmp/hil/nav2tune && cat > /tmp/hil/nav2tune/speed.yaml" <"$RES/speed.yaml"
    overlay="/nav2tune/speed.yaml"
fi
# Дополнительные оверлеи (Г1, зоны): локальные файлы → /tmp/hil/nav2tune/ → Nav2 после speed.yaml.
for f in ${SPEED_EXTRA_OVERLAYS:-}; do
    base=$(basename "$f")
    ssh "${SSHO[@]}" "$SPEED_MAIN" "mkdir -p /tmp/hil/nav2tune && cat > /tmp/hil/nav2tune/$base" <"$f"
    overlay="$overlay /nav2tune/$base"
    log "доп. оверлей: $f → /nav2tune/$base"
done
overlay="${overlay# }"
# Зоны скорости: маска + оверлей конфигурации + публикаторы в контейнер nav2 (после его пересоздания).
ZONES=0
if [ "${SPEED_ZONES:-0}" = 1 ]; then
    ZONES=1
    mask_dir="$RES/mask"
    mkdir -p "$mask_dir"
    if [ -n "${SPEED_MASK_LOCAL:-}" ]; then
        cp "$SPEED_MASK_LOCAL" "$mask_dir/speed_mask.yaml"
        cp "$(dirname "$SPEED_MASK_LOCAL")/$(grep -m1 '^image:' "$SPEED_MASK_LOCAL" | awk '{print $2}')" "$mask_dir/"
    else
        map_pose="$SX $SY $SYAW"
        case "$scene" in
            depot)
                npz="${SPEED_MASK_NPZ:-$SPEED_SANDBOX_DIR/out/coverage_plan_full/plan_map.npz}"
                [ -f "$npz" ] || sb "cat $npz" >"$RES/plan_map.npz"
                [ -f "$RES/plan_map.npz" ] || { log "❌ нет снимка карты $npz (SPEED_MASK_NPZ)"; exit 2; }
                npz="$RES/plan_map.npz" ;;
            *) npz="" ;;
        esac
        case "$scene" in
            depot)
                ( cd "$REPO_ROOT" && PYTHONPATH="$REPO_ROOT/src/rob_box_sim" python -m rob_box_sim.speed_zones map "$npz" \
                    --map-pose $map_pose --out "$mask_dir/speed_mask" ) >>"$RES/run.log" 2>&1 ;;
            *)
                sdf="$REPO_ROOT/src/rob_box_sim/worlds/$world.sdf"
                [ -f "$sdf" ] || { log "❌ нет sdf $sdf для маски зон"; exit 2; }
                ( cd "$REPO_ROOT" && PYTHONPATH="$REPO_ROOT/src/rob_box_sim" python -m rob_box_sim.speed_zones world "$sdf" \
                    --map-pose $map_pose --out "$mask_dir/speed_mask" ) >>"$RES/run.log" 2>&1 ;;
        esac
        [ -f "$mask_dir/speed_mask.yaml" ] || { log "❌ маска зон не создана (run.log)"; exit 2; }
    fi
    ssh "${SSHO[@]}" "$SPEED_MAIN" "mkdir -p /tmp/hil/nav2tune/mask && cat > /tmp/hil/nav2tune/mask/speed_mask.yaml" <"$mask_dir/speed_mask.yaml"
    ssh "${SSHO[@]}" "$SPEED_MAIN" "cat > /tmp/hil/nav2tune/mask/speed_mask.pgm" <"$mask_dir/speed_mask.pgm"
    ssh "${SSHO[@]}" "$SPEED_MAIN" "mkdir -p /tmp/hil/nav2tune && cat > /tmp/hil/nav2tune/zones.yaml" \
        <"$REPO_ROOT/docker/main/config/nav2/speed_zones.yaml"
    overlay="$overlay /nav2tune/zones.yaml"
    overlay="${overlay# }"
    log "зоны скорости: маска $mask_dir → /tmp/hil/nav2tune/mask, оверлей zones.yaml"
fi
# CRLF из чекаута на Windows срезается на той стороне (tr -d '\r')
ssh "${SSHO[@]}" "$SPEED_MAIN" "tr -d '\r' > /tmp/hil/nav2tune.compose.yaml" <"$HERE/nav2tune.compose.yaml"
ssh "${SSHO[@]}" "$SPEED_SANDBOX" "mkdir -p /home/ros2/shidi-speed && tr -d '\r' > /home/ros2/shidi-speed/sim_spawn.compose.yaml" \
    <"$HERE/sim_spawn.compose.yaml"

mn "test -f /tmp/hil.lock" || { log "❌ HIL не включён (/tmp/hil.lock нет)"; exit 2; }
log "старт: сцена $scene мир $world спавн ($spawn) скорость $speed оверлей '${overlay:-—}'"

SIMENV="HIL_ROBOT_ID=$SPEED_ROBOT_ID HIL_SANDBOX_IP=10.1.1.30 HIL_ZENOH_CONNECT='tcp/10.1.1.10:7447#iface=enp2s0,tcp/10.1.1.11:7447#iface=enp2s0,tcp/10.1.1.12:7447#iface=enp2s0' SIM_IMAGE=rob_box_sim:hil SIM_PREFIX=rob-box-hil SIM_WORLD=polygon SIM_DRI_CARD=/dev/dri/card1 SIM_DRI_RENDER=/dev/dri/renderD128 SPEED_SIM_CMD='$sim_cmd'"
SIMDC="cd $SPEED_SANDBOX_DIR && $SIMENV docker compose -p rob_box_hil -f docker/sim/compose.yaml -f docker/sim/compose.hil.yaml -f /home/ros2/shidi-speed/sim_spawn.compose.yaml"
MAINSVC="robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2"
maps="$SPEED_MAPS_ROOT/$label"

# 1. сим вниз
sb "$SIMDC rm -sf gazebo" >>"$RES/run.log" 2>&1
# 2. Main: свежий стек с sim time и оверлеем
st=$(mn "docker inspect -f '{{.State.Status}}' ros2-control 2>/dev/null || echo absent")
case "$st" in running*) log "❌ ros2-control работает ($st) — стоп"; exit 3 ;; esac
extra=""
[ -n "$overlay" ] && extra="-f /tmp/hil/nav2tune.compose.yaml"
T0=$(mn "date -u +%Y-%m-%dT%H:%M:%SZ")
mn "mkdir -p $maps && cd ~/rob_box_project/docker/main && HIL_MAPS_DIR=$maps HIL_RTABMAP_MODE=mapping HIL_GUARD_SOUND=\${HIL_GUARD_SOUND:-on} NAV2TUNE_OVERLAYS='$overlay' docker compose -f docker-compose.yaml -f /tmp/hil/main.images.yaml -f /tmp/hil/main.hil.yaml $extra up -d --no-deps --pull never --force-recreate $MAINSVC" >>"$RES/run.log" 2>&1 \
    || { log "❌ Main: пересоздание не прошло"; exit 4; }
mn "docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' nav2 | grep -E 'NAV2_PARAMS_OVERLAYS|USE_SIM_TIME'" >>"$RES/run.log" 2>&1
# 2b. зоны скорости: публикаторы маски внутри контейнера nav2 (образ содержит nav2_map_server;
#     на боевом checkout Main Pi штатного блока start_nav2_direct.sh ещё нет). Detached: живут
#     до пересоздания nav2. lifecycle_manager (autostart) поднимает оба узла; маска и
#     filter_info — latched: nav2 заберёт их в любой момент после активации.
if [ "$ZONES" = 1 ]; then
    mn "docker exec -d nav2 bash -c 'source /opt/ros/humble/setup.bash; ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=true -r __node:=speed_mask_server -r map:=speed_filter_mask -p yaml_filename:=/nav2tune/mask/speed_mask.yaml & ros2 run nav2_map_server costmap_filter_info_server --ros-args -p use_sim_time:=true -r __node:=costmap_filter_info_server -p type:=2 -p base:=0.0 -p multiplier:=0.01 -p mask_topic:=/speed_filter_mask -p filter_info_topic:=/costmap_filter_info & ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p use_sim_time:=true -r __node:=lifecycle_manager_speed_mask -p autostart:=true -p node_names:=[speed_mask_server,costmap_filter_info_server] & wait'" >>"$RES/run.log" 2>&1 \
        || { log "❌ публикаторы маски не запустились"; exit 4; }
    mask_ok=""
    for _ in $(seq 1 12); do
        mask_ok=$(mn "docker exec nav2 bash -lc 'source /opt/ros/humble/setup.bash && timeout 15 ros2 topic echo --once /speed_filter_mask >/dev/null 2>&1 && echo OK'" 2>/dev/null)
        [ "$mask_ok" = OK ] && break
        sleep 5
    done
    [ "$mask_ok" = OK ] || { log "❌ маска зон не опубликовалась за ~3 мин (docker logs nav2)"; exit 4; }
    log "маска зон опубликована (latched): /speed_filter_mask + /costmap_filter_info"
fi
# 3–4. сим вверх со спавном сцены, ровно один /controller_manager. Сторож gazebo отказывает, пока не видит
# /robot_state_publisher робота (Main только что пересоздан) — тогда gazebo выходит, и пробуем ещё раз.
# Оба контроллера сима активны. 09.11 в Depot joint_state_broadcaster падал «Switch controller timed out»:
# odom→base_footprint на Main не появлялся, Nav2 не поднимался 7 мин — такой сим пересоздаём сразу.
sim_controllers_ok() {
    local ok bad
    for _ in $(seq 1 12); do
        ok=$(sb "docker logs $G 2>&1 | grep -c 'Configured and activated'")
        bad=$(sb "docker logs $G 2>&1 | grep -cE 'Failed to activate controller|spawner.*process has died'")
        [ "${bad:-0}" -gt 0 ] && { log "контроллер сима не активировался (spawner) — пересоздаю сим"; return 1; }
        [ "${ok:-0}" -ge 2 ] && return 0
        sleep 5
    done
    log "контроллеры сима не активны за 60 с — пересоздаю сим"; return 1
}
sim_up() {
    local st gz cm
    st=$(mn "docker inspect -f '{{.State.Status}}' ros2-control 2>/dev/null || echo absent")
    case "$st" in running*) log "❌ ros2-control работает ($st) — сим не поднимаю"; exit 3 ;; esac
    sb "$SIMDC up -d --no-build gazebo" >>"$RES/run.log" 2>&1 || { log "❌ gazebo up"; exit 5; }
    for _ in $(seq 1 36); do
        gz=$(sb "docker inspect -f '{{.State.Status}}' $G 2>/dev/null || echo absent")
        case "$gz" in running*) ;; *) log "gazebo не работает ($gz)"; sb "docker logs --tail 5 $G" >>"$RES/run.log" 2>&1; return 1 ;; esac
        cm=$(sb "docker exec $G /entrypoint.sh python3 -m rob_box_sim.hil count --settle 3 2>/dev/null | grep -E '^[0-9]+$' | tail -n 1")
        [ "$cm" = 1 ] && { sim_controllers_ok; return $?; }
        if [ -n "$cm" ] && [ "$cm" != 0 ]; then log "❌ /controller_manager=$cm — гашу сим"; sb "$SIMDC rm -sf gazebo"; exit 7; fi
        sleep 5
    done
    log "❌ контроллер Gazebo не появился"; sb "$SIMDC rm -sf gazebo"; exit 8
}
sleep 15
up=0
for attempt in 1 2 3 4; do
    if sim_up; then up=1; break; fi
    log "повтор подъёма сима ($attempt)"; sb "$SIMDC rm -sf gazebo" >>"$RES/run.log" 2>&1; sleep 10
done
[ "$up" = 1 ] || { log "❌ сим не поднялся"; exit 6; }
log "сим поднят, /controller_manager=1"

# 5. замер Main Pi (load и docker stats раз в ~2 с) на всё время прогона
stopf="/tmp/hil/speed_stop_$label"
mn "rm -f $stopf; nohup bash -c 'end=\$((SECONDS+$SPEED_BENCH_TIMEOUT+120)); while [ ! -f $stopf ] && [ \$SECONDS -lt \$end ]; do echo \"T \$(date +%s)\"; echo \"L \$(cut -d\" \" -f1 /proc/loadavg)\"; docker stats --no-stream --format \"S {{.Name}} {{.CPUPerc}}\" nav2 rtabmap proximity-guard; done' > /tmp/hil/speed_cpu_$label.log 2>&1 </dev/null &"

# 6. прогон
sb "docker exec $G mkdir -p /tmp/speed" >>"$RES/run.log" 2>&1
for f in speed_bench.py speed_verdict.py; do
    ssh "${SSHO[@]}" "$SPEED_SANDBOX" "docker exec -i $G sh -c 'tr -d \"\\\\r\" > /tmp/speed/$f'" <"$HERE/$f"
done
sb "docker exec $G /entrypoint.sh timeout $SPEED_BENCH_TIMEOUT python3 -u /tmp/speed/speed_bench.py --world $world --spawn $SX $SY $SYAW --vmax $vmax --tasks '$tasks' --label $label --out /tmp/sp/$label" >"$RES/bench.out" 2>&1
log "bench rc=$?"
mn "touch $stopf"
for f in result.json track.csv pose_err.csv; do
    sb "docker exec $G cat /tmp/sp/$label/$f" >"$RES/$f" 2>/dev/null
done
# Ни одной задачи — сбой стенда (бенч не дождался Nav2 и т.п.), а не провал скорости: в журнал не пишем,
# sweep.sh повторит тот же прогон.
if ! grep -q '"name"' "$RES/result.json" 2>/dev/null; then
    log "❌ стенд: в result.json нет задач — прогон не засчитан (bench.out)"
    exit 9
fi

# 7. логи и замер Main
sleep 3
mn "cat /tmp/hil/speed_cpu_$label.log" >"$RES/cpu.log"
mn "docker logs --since $T0 proximity-guard 2>&1" >"$RES/guard.log"
mn "docker logs --since $T0 nav2 2>&1 | grep -vE 'missed its desired rate'" >"$RES/nav2.log"
missed=$(mn "docker logs --since $T0 nav2 2>&1 | grep -cE 'missed its desired rate'")
mn "docker logs --since $T0 rtabmap 2>&1 | tail -100" >"$RES/rtabmap.log"
echo "$missed" >"$RES/nav2_missed.txt"
python "$PYHERE/speed_verdict.py" cpu "$RES/cpu.log" >"$RES/cpu.json"
python - "$RES/cpu.json" "$missed" "$RES/main.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1], encoding="utf-8"))
d["nav2_missed"] = int(sys.argv[2] or 0)
json.dump(d, open(sys.argv[3], "w", encoding="utf-8"), ensure_ascii=False)
EOF
python "$PYHERE/speed_verdict.py" run "$RES/result.json" "$RES/main.json" --speed "$([ "$speed" = base ] && echo 0.15 || echo "$speed")" \
    --scene "$scene" --rep "$rep" --label "$label" --ledger "$SPEED_OUT/ledger${SPEED_LEDGER_SUFFIX:-}.jsonl" | tee "$RES/verdict.txt" | tee -a "$RES/run.log"
log "стопов защиты (→ stop): $(grep -c '→ stop' "$RES/guard.log"); missed rate: $missed"
