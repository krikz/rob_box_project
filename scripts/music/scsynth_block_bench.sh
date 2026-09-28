#!/usr/bin/env bash
# scsynth_block_bench.sh — замер CPU scsynth и xrun'ов JACK по block size (-z).
# Issue #3114. Запускать НА Vision Pi, из клона репо:
#
#   ssh ros2@10.1.1.21
#   cd ~/rob_box_project && git pull
#   bash scripts/music/scsynth_block_bench.sh                 # 1024 512 256 128 64, по 60 с
#   bash scripts/music/scsynth_block_bench.sh --sizes "1024 256 64" --duration 90
#
# Что делает для каждого размера:
#   1. Пересоздаёт контейнер supercollider с SCSYNTH_BLOCK_SIZE=<N> (env
#      интерполируется в docker/vision/docker-compose.yaml). Образ тот же, что
#      крутился до бенча (--pull never, тег берётся из `docker inspect`).
#   2. Проверяет по /proc/<pid>/cmdline, что scsynth реально запущен с -z <N>.
#   3. Idle-замер CPU scsynth/jackd (без нот), затем нагрузка: 6 плееров из
#      scripts/music/scsynth_bench_stimulus.py (молча, в приватные шины) и
#      замер CPU по /proc/<pid>/stat + avg/peak CPU из OSC /status самого scsynth.
#   4. Считает строки с "xrun" в `docker logs` контейнера за время прогона.
# В конце печатает таблицу (и TSV в --out).
#
# Репо и конфиги НЕ меняет: размер передаётся только env'ом процесса compose.
# На время бенча voice-assistant ОСТАНОВЛЕН (робот молчит и не слушает),
# чтобы sclang не лез в scsynth. trap на EXIT/INT/TERM возвращает
# supercollider к исходному SCSYNTH_BLOCK_SIZE (как было в контейнере до
# бенча) и снова запускает voice-assistant, если он был запущен.
#
# Требует: docker compose v2 (или docker-compose), коммит с SCSYNTH_BLOCK_SIZE
# в start_supercollider.sh (иначе шаг 2 даст FAIL «-z не применился»).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
STIMULUS="$SCRIPT_DIR/scsynth_bench_stimulus.py"

COMPOSE_DIR="${COMPOSE_DIR:-$REPO_ROOT/docker/vision}"
SC_SERVICE="supercollider"
SC_CONTAINER="${SC_CONTAINER:-supercollider}"
VA_SERVICE="voice-assistant"
VA_CONTAINER="${VA_CONTAINER:-voice-assistant}"

SIZES="1024 512 256 128 64"
DURATION=60
IDLE=15
SETTLE=10
SYNTHS="tb303,organ,fuzz,pluck,bass,warmpad"
STOP_VOICE=1
OUT=""
DRY_RUN=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

  --sizes "1024 512 256 128 64"  block size'ы для прогона (степени двойки 64..1024)
  --duration SEC                 длительность нагрузки на размер (default $DURATION)
  --idle SEC                     длительность idle-замера (default $IDLE)
  --settle SEC                   пауза после рестарта scsynth (default $SETTLE)
  --synths a,b,c                 SynthDef'ы плееров (default $SYNTHS)
  --keep-voice                   НЕ останавливать voice-assistant (шумнее замер)
  --out FILE                     сохранить TSV-таблицу
  --dry-run                      только проверить окружение и напечатать план
  -h, --help                     эта справка

Env: COMPOSE_DIR (default $REPO_ROOT/docker/vision), SC_CONTAINER, VA_CONTAINER.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --sizes) SIZES="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --idle) IDLE="$2"; shift 2 ;;
        --settle) SETTLE="$2"; shift 2 ;;
        --synths) SYNTHS="$2"; shift 2 ;;
        --keep-voice) STOP_VOICE=0; shift ;;
        --out) OUT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 64 ;;
    esac
done

log() { echo "[bench $(date +%H:%M:%S)] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

for s in $SIZES; do
    case "$s" in
        64|128|256|512|1024) ;;
        *) die "block size '$s' недопустим (64|128|256|512|1024)" ;;
    esac
done
case "$DURATION" in ''|*[!0-9]*) die "--duration должно быть целым числом секунд" ;; esac
[ "$DURATION" -ge 10 ] || die "--duration должно быть >= 10 с"
[ -f "$STIMULUS" ] || die "нет $STIMULUS"
[ -f "$COMPOSE_DIR/docker-compose.yaml" ] || die "нет $COMPOSE_DIR/docker-compose.yaml (задай COMPOSE_DIR)"
grep -q 'SCSYNTH_BLOCK_SIZE' "$COMPOSE_DIR/scripts/supercollider/start_supercollider.sh" \
    || die "start_supercollider.sh без SCSYNTH_BLOCK_SIZE — сделай git pull ветки с #3114"

if docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
    PULL_NEVER=(--pull never)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
    PULL_NEVER=()
else
    die "нет ни docker compose, ни docker-compose"
fi

docker inspect "$SC_CONTAINER" >/dev/null 2>&1 || die "контейнер $SC_CONTAINER не найден"

# ── Исходное состояние (для восстановления) ──────────────────────────────────
ORIG_IMAGE="$(docker inspect -f '{{.Config.Image}}' "$SC_CONTAINER")"
ORIG_BLOCK="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$SC_CONTAINER" \
    | sed -n 's/^SCSYNTH_BLOCK_SIZE=//p' | head -n1)"
# Образ вида <prefix>:supercollider-<tag> → те же SERVICE_IMAGE_PREFIX/IMAGE_TAG,
# чтобы compose не подменил образ (и не полез в registry).
case "$ORIG_IMAGE" in
    *:supercollider-*)
        export SERVICE_IMAGE_PREFIX="${ORIG_IMAGE%:supercollider-*}"
        export IMAGE_TAG="${ORIG_IMAGE##*:supercollider-}"
        ;;
    *) die "образ '$ORIG_IMAGE' не похож на <prefix>:supercollider-<tag>, не рискую пересоздавать" ;;
esac
VA_WAS_RUNNING=0
if [ "$(docker inspect -f '{{.State.Running}}' "$VA_CONTAINER" 2>/dev/null || echo false)" = "true" ]; then
    VA_WAS_RUNNING=1
fi

log "compose dir:   $COMPOSE_DIR"
log "образ:         $ORIG_IMAGE (SERVICE_IMAGE_PREFIX=$SERVICE_IMAGE_PREFIX IMAGE_TAG=$IMAGE_TAG)"
log "исходный SCSYNTH_BLOCK_SIZE в контейнере: '${ORIG_BLOCK:-<не задан, дефолт скрипта>}'"
log "voice-assistant запущен: $VA_WAS_RUNNING, останавливать: $STOP_VOICE"
log "размеры: $SIZES; idle ${IDLE}s, нагрузка ${DURATION}s, синты $SYNTHS"
if [ "$DRY_RUN" = 1 ]; then
    log "dry-run: ничего не пересоздаю"
    exit 0
fi

compose_up_sc() {
    # $1 — block size или пусто (= как было до бенча)
    local block="$1"
    (
        cd "$COMPOSE_DIR"
        if [ -n "$block" ]; then
            export SCSYNTH_BLOCK_SIZE="$block"
        else
            unset SCSYNTH_BLOCK_SIZE
        fi
        "${COMPOSE[@]}" up -d --no-deps "${PULL_NEVER[@]}" --force-recreate "$SC_SERVICE"
    ) >&2
}

restore() {
    local rc=$?
    trap - EXIT INT TERM
    log "восстановление: supercollider → SCSYNTH_BLOCK_SIZE='${ORIG_BLOCK:-<дефолт>}'"
    compose_up_sc "$ORIG_BLOCK" || log "ERROR: не смог пересоздать $SC_SERVICE — проверь вручную!"
    if wait_scsynth >/dev/null; then
        log "после восстановления: scsynth -z $(scsynth_z), образ $(docker inspect -f '{{.Config.Image}}' "$SC_CONTAINER")"
    else
        log "ERROR: scsynth не поднялся после восстановления — docker logs $SC_CONTAINER"
    fi
    if [ "$STOP_VOICE" = 1 ] && [ "$VA_WAS_RUNNING" = 1 ]; then
        log "запускаю voice-assistant обратно"
        (cd "$COMPOSE_DIR" && "${COMPOSE[@]}" start "$VA_SERVICE") >&2 \
            || log "ERROR: voice-assistant не стартовал — проверь вручную!"
    fi
    exit "$rc"
}

sc_exec() { docker exec "$SC_CONTAINER" "$@"; }
scsynth_pid() { sc_exec pgrep -x scsynth 2>/dev/null | head -n1; }
jackd_pid() { sc_exec pgrep -x jackd 2>/dev/null | head -n1; }
scsynth_z() {
    local pid
    pid="$(scsynth_pid)"
    [ -n "$pid" ] || { echo "?"; return; }
    sc_exec cat "/proc/$pid/cmdline" | tr '\0' ' ' | sed -n 's/.* -z \([0-9]*\).*/\1/p'
}

wait_scsynth() {
    for _ in $(seq 1 60); do
        if [ -n "$(scsynth_pid)" ] && sc_exec sh -c 'jack_lsp 2>/dev/null | grep -q "^jack:out_1$"'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# utime+stime (тики) процесса внутри контейнера; поле comm может содержать
# пробелы, поэтому режем после последней ')'.
proc_ticks() {
    local pid="$1"
    [ -n "$pid" ] || { echo 0; return; }
    sc_exec cat "/proc/$pid/stat" | sed 's/.*) //' | awk '{print $12 + $13}'
}

CLK_TCK=""
cpu_pct_over() {
    # $1 seconds; печатает "<scsynth%> <jackd%>" (% одного ядра)
    local secs="$1" sp jp s0 s1 j0 j1 t0 t1
    sp="$(scsynth_pid)"; jp="$(jackd_pid)"
    s0="$(proc_ticks "$sp")"; j0="$(proc_ticks "$jp")"; t0="$(date +%s.%N)"
    sleep "$secs"
    s1="$(proc_ticks "$sp")"; j1="$(proc_ticks "$jp")"; t1="$(date +%s.%N)"
    awk -v s0="$s0" -v s1="$s1" -v j0="$j0" -v j1="$j1" -v t0="$t0" -v t1="$t1" -v hz="$CLK_TCK" \
        'BEGIN { dt = t1 - t0; printf "%.1f %.1f\n", (s1 - s0) / hz / dt * 100, (j1 - j0) / hz / dt * 100 }'
}

json_field() {
    # $1 json, $2 key — без jq (на Pi его может не быть)
    python3 -c 'import json,sys; v=json.loads(sys.argv[1]).get(sys.argv[2]); print("-" if v is None else v)' "$1" "$2"
}

trap restore EXIT INT TERM

if [ "$STOP_VOICE" = 1 ] && [ "$VA_WAS_RUNNING" = 1 ]; then
    log "останавливаю voice-assistant на время бенча"
    (cd "$COMPOSE_DIR" && "${COMPOSE[@]}" stop "$VA_SERVICE") >&2
fi

RESULTS=()
HEADER=$'block\tctl_Hz\tz_actual\tidle_sc%\tidle_jack%\tload_sc%\tload_jack%\tsc_avgCPU\tsc_peakCPU\tmax_synths\tnotes\txruns\tstatus'
ctl_hz() { awk -v b="$1" 'BEGIN { printf "%.1f", 16000 / b }'; }

for size in $SIZES; do
    log "=== block size $size ==="
    compose_up_sc "$size"
    if ! wait_scsynth; then
        RESULTS+=("$size	$(ctl_hz "$size")	-	-	-	-	-	-	-	-	-	-	FAIL:scsynth не поднялся")
        continue
    fi
    [ -n "$CLK_TCK" ] || CLK_TCK="$(sc_exec getconf CLK_TCK)"
    z_actual="$(scsynth_z)"
    since="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    sleep "$SETTLE"

    read -r idle_sc idle_jack < <(cpu_pct_over "$IDLE")

    stim_out="$(mktemp)"
    docker exec -i "$SC_CONTAINER" python3 - --duration "$DURATION" --synths "$SYNTHS" \
        < "$STIMULUS" > "$stim_out" 2>&1 &
    stim_pid=$!
    sleep 3  # d_load + разгон плееров
    read -r load_sc load_jack < <(cpu_pct_over "$((DURATION - 5))")
    stim_rc=0
    wait "$stim_pid" || stim_rc=$?
    res_json="$(sed -n 's/^BENCH_RESULT //p' "$stim_out" | tail -n1)"
    xruns="$(docker logs --since "$since" "$SC_CONTAINER" 2>&1 | grep -ci 'xrun' || true)"

    status="ok"
    [ "$z_actual" = "$size" ] || status="FAIL:-z не применился ($z_actual)"
    if [ -z "$res_json" ]; then
        status="FAIL:stimulus rc=$stim_rc"
        log "stimulus output:"; sed 's/^/    /' "$stim_out" >&2
        avg="-"; peak="-"; maxs="-"; notes="-"
    else
        avg="$(json_field "$res_json" avg_cpu_mean)"
        peak="$(json_field "$res_json" peak_cpu_max)"
        maxs="$(json_field "$res_json" max_synths)"
        notes="$(json_field "$res_json" notes_sent)"
        missing="$(json_field "$res_json" missing)"
        [ "$missing" = "[]" ] || status="$status; нет synthdef: $missing"
        log "stimulus: $res_json"
    fi
    rm -f "$stim_out"
    RESULTS+=("$size	$(ctl_hz "$size")	$z_actual	$idle_sc	$idle_jack	$load_sc	$load_jack	$avg	$peak	$maxs	$notes	$xruns	$status")
done

echo
echo "scsynth block-size bench — $(hostname) — $(date -u +%Y-%m-%dT%H:%M:%SZ) — image $ORIG_IMAGE"
echo "CPU% — доля ОДНОГО ядра по /proc/<pid>/stat; sc_avg/peakCPU — из OSC /status scsynth"
{
    echo "$HEADER"
    printf '%s\n' "${RESULTS[@]}"
} | if command -v column >/dev/null 2>&1; then column -t -s $'\t'; else cat; fi
if [ -n "$OUT" ]; then
    { echo "$HEADER"; printf '%s\n' "${RESULTS[@]}"; } > "$OUT"
    log "TSV: $OUT"
fi
