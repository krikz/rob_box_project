#!/usr/bin/env bash
# Штатный сценарий Depot (#4694): сам выставляет HIL_EXPECT_WORLD=depot
# и не шлёт первую цель, пока код не подтвердит мир.
# Обычный Polygon этим скриптом не запускается.
#
#   scripts/hil/video/run_depot.sh on
#   scripts/hil/video/run_depot.sh goals [M1…]
#
# Переменные: HIL_CMD, HIL_SANDBOX_REPO, VIDEO_DIR, HIL_WORLD_CHECK_DIR.
# Живой HIL этот файл сам не поднимает.
set -euo pipefail
export MSYS_NO_PATHCONV=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
CHECKS="$ROOT/scripts/hil/checks/world_check.py"
HIL_CMD="${HIL_CMD:-$ROOT/scripts/hil/hil.sh}"
VIDEO_DIR="${VIDEO_DIR:-${HIL_SANDBOX_REPO:-/home/ros2/rob_box_hilvideo}}"
RUNVIDEO="${RUNVIDEO:-$VIDEO_DIR/runvideo.sh}"

die() { printf '❌ %s\n' "$*" >&2; exit 1; }

# Код решает HIL_EXPECT_WORLD. Не из текста оператора.
expect_world() {
    python3 -X utf8 "$CHECKS" --scenario depot
}

# Факты мира уже в HIL_WORLD_CHECK_DIR. Нет PASS — цель не уходит.
assert_world_passed() {
    local dir="${HIL_WORLD_CHECK_DIR:-}" line
    [ -n "$dir" ] || die "WORLD_MISMATCH: нет HIL_WORLD_CHECK_DIR, цель не шлю"
    [ -f "$dir/verdict.txt" ] || die "WORLD_MISMATCH: нет $dir/verdict.txt, цель не шлю"
    line=$(head -n 1 "$dir/verdict.txt")
    case "$line" in
        PASS:*) return 0 ;;
    esac
    die "WORLD_MISMATCH: $line — первая цель не отправлена"
}

cmd_on() {
    local expect kind compose
    expect=$(expect_world)
    [ "$expect" = depot ] || die "сценарий Depot не выставил HIL_EXPECT_WORLD=depot (код: '${expect:-пусто}')"
    export HIL_EXPECT_WORLD="$expect"
    export HIL_SANDBOX_REPO="${HIL_SANDBOX_REPO:-$VIDEO_DIR}"
    # До hil.sh on: копия Compose должна быть Depot, не обычный Polygon.
    # Потеря overlay (full13) — WORLD_MISMATCH здесь, цель ещё не слалась.
    compose="$HIL_SANDBOX_REPO/docker/sim/compose.hil.yaml"
    [ -f "$compose" ] || die "WORLD_MISMATCH: нет $compose, Depot overlay не готов, цель не шлю"
    kind=$(python3 -X utf8 "$CHECKS" --compose-file "$compose")
    [ "$kind" = depot ] || die "WORLD_MISMATCH: $compose не Depot (код: ${kind:-пусто}), цель не шлю"
    if [ -z "${HIL_WORLD_CHECK_DIR:-}" ]; then
        export HIL_WORLD_CHECK_DIR="${HIL_DEPOT_ARTIFACT_DIR:-$ROOT/.tmp/hil-world-check}"
    fi
    mkdir -p "$HIL_WORLD_CHECK_DIR"
    printf '%s\n' "$HIL_SANDBOX_REPO" >"$HIL_WORLD_CHECK_DIR/compose_dir.txt"
    printf 'HIL_EXPECT_WORLD=%s\n' "$HIL_EXPECT_WORLD" >"$HIL_WORLD_CHECK_DIR/expect_world.txt"
    # hil.sh on: гейт до «HIL включён». Отказ — sim_down, цель ещё не слалась.
    bash "$HIL_CMD" on
}

cmd_goals() {
    assert_world_passed
    [ -f "$RUNVIDEO" ] || die "нет $RUNVIDEO (сначала sandbox_prep.sh)"
    # Первая цель уходит только отсюда, после PASS.
    bash "$RUNVIDEO" "$@"
}

usage() {
    echo "usage: run_depot.sh on | goals [цель...]" >&2
    exit 2
}

[ "$#" -ge 1 ] || usage
case "$1" in
    on) shift; cmd_on "$@" ;;
    goals) shift; cmd_goals "$@" ;;
    *) usage ;;
esac
