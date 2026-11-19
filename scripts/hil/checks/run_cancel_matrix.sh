#!/usr/bin/env bash
# Матрица отмен FollowPath (#4435/#4458/#3789/#4770/#4647).
# HIL_CHECKS_OWNER=<имя> scripts/hil/checks/run_cancel_matrix.sh
# Замок — hil.sh on/off (HIL_LOCK=/tmp/hil.lock). Занятый stand.lock — выход 3, hil.sh on не вызывается.
# NOT_TRIGGERED — не PASS. Робот на кирпичах. Параллельный HIL — отказ.
set -euo pipefail
export MSYS_NO_PATHCONV=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
. "$REPO_ROOT/scripts/hil/hil.conf"
: "${HIL_CHECKS_OWNER:=}"
: "${STAND_LOCK:=C:/Users/krikz/AppData/Local/Temp/claude/a31bd7f7b8b42df0/3f238318-a4ed-402f-88a9-a5003be4590b/scratchpad/stand.lock}"
: "${HIL_CHECKS_OUT:=${TMPDIR:-/tmp}/hil_checks/cancel_matrix_$(date -u +%Y%m%dT%H%M%SZ)}"
MATRIX=(orphan orphan_natural nav_aborted stop_idle stop_active controller_abort client_kill host_cancel)
log() { printf "[matrix %s] %s\n" "$(date -u +%H:%M:%SZ)" "$*"; }
die() { printf "FAIL %s\n" "$*" >&2; exit 3; }
stand_owner() {
    if [ ! -f "$STAND_LOCK" ]; then printf "%s\n" ""; return 0; fi
    awk '{print $1; exit}' "$STAND_LOCK"
}
hil_is_on() {
    local status
    status=$(bash "$REPO_ROOT/scripts/hil/hil.sh" status 2>&1 || true)
    printf "%s\n" "$status" | grep -q "HIL on"
}
preflight_matrix() {
    local owner
    [ -n "$HIL_CHECKS_OWNER" ] || die "HIL_CHECKS_OWNER пуст — замок не беру"
    owner=$(stand_owner)
    if [ -n "$owner" ] && [ "$owner" != "$HIL_CHECKS_OWNER" ]; then
        die "stand.lock занят ($owner), не $HIL_CHECKS_OWNER — выход без hil.sh on"
    fi
    if hil_is_on; then
        die "HIL уже on ($HIL_LOCK). Параллельный прогон не стартую. hil.sh status"
    fi
}
take_lock() {
    log "hil.sh on (замок $HIL_LOCK, владелец $HIL_CHECKS_OWNER)"
    bash "$REPO_ROOT/scripts/hil/hil.sh" on
}
release_lock() {
    log "hil.sh off (снять $HIL_LOCK)"
    bash "$REPO_ROOT/scripts/hil/hil.sh" off || log "hil.sh off вернул ошибку — замок мог остаться"
}
summarize() {
    local f="$HIL_CHECKS_OUT/verdicts.txt" line worst=0
    [ -f "$f" ] || die "нет verdicts.txt"
    log "итог $f"
    cat "$f"
    while IFS= read -r line; do
        case "$line" in
            VERDICT*" PASS:"*) ;;
            VERDICT*" FAIL:"*) worst=1 ;;
            VERDICT*" NOT_TRIGGERED:"*|VERDICT*" NO_DATA:"*)
                if [ "$worst" -eq 0 ]; then worst=2; fi ;;
            *) ;;
        esac
    done <"$f"
    if [ "$worst" -ne 0 ]; then log "NOT_TRIGGERED не PASS. Код $worst"; fi
    return "$worst"
}
main() {
    case "${1:-}" in -h|--help) sed -n "2,6p" "$0"; exit 0 ;; esac
    preflight_matrix
    mkdir -p "$HIL_CHECKS_OUT"
    printf "%s %s\n" "$HIL_CHECKS_OWNER" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >"$STAND_LOCK"
    take_lock
    trap release_lock EXIT
    log "сценарии: ${MATRIX[*]}"
    HIL_CHECKS_OWNER="$HIL_CHECKS_OWNER" HIL_CHECKS_OUT="$HIL_CHECKS_OUT" bash "$HERE/run_checks.sh" "${MATRIX[@]}"
    summarize
}
main "$@"
