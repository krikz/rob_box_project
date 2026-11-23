#!/usr/bin/env bash
# Контрольный прогон стенда P0 (#5180): полный изолированный сим, N стартов.
# Ступень A — все сервисы из amd64-образа rob_box_sim:local.
# Ступень B — nav2 из боевого arm64-образа (NAV2_IMAGE=тег@digest).
# Скрипт не ходит на робота, не вызывает hil.sh, dockerd не трогает.
set -u

STAGE="${1:?stage A|B}"
N="${2:-3}"
OUT="${3:-/tmp/stand5180/runs}"
PREFIX="${SIM_PREFIX:-stand5180}"
PROJECT="${SIM_PROJECT:-stand5180}"
DRI="${SIM_DRI_CARD:-/dev/dri/card1}"
ROOT="$(CDPATH='' cd -- "$(dirname -- "$0")/../../.." && pwd)"

COMPOSE=(docker compose -p "$PROJECT" -f "$ROOT/docker/sim/compose.yaml")
if [ "$STAGE" = "B" ]; then
    : "${NAV2_IMAGE:?NAV2_IMAGE=tag@sha256 required for stage B}"
    COMPOSE+=(-f "$ROOT/docker/sim/compose.prod-nav2.yaml")
fi
export SIM_PREFIX="$PREFIX" SIM_DRI_CARD="$DRI" NAV2_IMAGE="${NAV2_IMAGE:-}"

mkdir -p "$OUT"
echo "stage=$STAGE n=$N prefix=$PREFIX $(date -u +%Y-%m-%dT%H:%M:%SZ)"

for i in $(seq 1 "$N"); do
    DIR="$OUT/${STAGE}-${i}"
    mkdir -p "$DIR"
    echo "=== start $i $(date -u +%H:%M:%SZ) ==="
    SINCE=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    "${COMPOSE[@]}" up -d --no-build \
        zenoh-router gazebo robot-state-publisher twist-mux rtabmap imu-filter nav2 \
        >"$DIR/up.txt" 2>&1
    echo "$?" >"$DIR/up.rc"
    echo "up rc=$(cat "$DIR/up.rc") since=$SINCE"

    deadline=$(( $(date +%s) + 180 ))
    outcome="timeout-180s"
    while [ "$(date +%s)" -lt "$deadline" ]; do
        sleep 10
        if docker logs --since "$SINCE" "${PREFIX}-nav2" 2>&1 \
            | grep -q "Managed nodes are active"; then
            outcome="active"
            break
        fi
        if ! docker inspect -f '{{.State.Running}}' "${PREFIX}-nav2" 2>/dev/null | grep -q true; then
            outcome="nav2-not-running"
            break
        fi
    done
    echo "$outcome" >"$DIR/outcome"
    echo "outcome=$outcome $(date -u +%H:%M:%SZ)"
    docker logs --since "$SINCE" --tail 50 "${PREFIX}-nav2" >"$DIR/nav2-tail50.txt" 2>&1 || true
    docker ps -a --filter "name=${PREFIX}" --format '{{.Names}} {{.Status}}' >"$DIR/ps.txt" 2>&1 || true
    docker inspect -f 'Health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' \
        "${PREFIX}-nav2" >"$DIR/inspect-health.txt" 2>&1 || true

    for h in 1 2 3; do
        docker exec "${PREFIX}-nav2" bash /scripts/healthcheck.sh >"$DIR/hc${h}.txt" 2>&1
        echo "$?" >"$DIR/hc${h}.rc"
        echo "hc$h rc=$(cat "$DIR/hc${h}.rc")"
    done

    echo "=== down $i $(date -u +%H:%M:%SZ) ==="
    "${COMPOSE[@]}" down >"$DIR/down.txt" 2>&1
    echo "$?" >"$DIR/down.rc"
    echo "down rc=$(cat "$DIR/down.rc")"
    docker ps -a --filter "name=${PREFIX}" --format '{{.ID}} {{.Names}} {{.Status}}' >"$DIR/ps-after.txt" 2>&1 || true
    {
        echo -n "removing="; docker ps -a --filter status=removing --filter "name=${PREFIX}" -q | wc -l
        echo -n "dead="; docker ps -a --filter status=dead --filter "name=${PREFIX}" -q | wc -l
        echo -n "created="; docker ps -a --filter status=created --filter "name=${PREFIX}" -q | wc -l
    } >"$DIR/leftovers.txt"
    docker volume rm "${PROJECT}_sim-maps" >"$DIR/vol-rm.txt" 2>&1
    echo "$?" >"$DIR/vol-rm.rc"
done
echo "done $(date -u +%Y-%m-%dT%H:%M:%SZ)"
