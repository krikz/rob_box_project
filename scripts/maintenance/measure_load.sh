#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 📈 Замер загрузки Pi (ADR-0137 §2.4, этап 0 ADR-0130: бюджет §2.11, этап 8)
# ═══════════════════════════════════════════════════════════════════════════
# Запускается на ХОСТЕ любого Pi (Main или Vision), ничего не меняет.
# Каждые --interval секунд в течение --duration пишет в CSV:
#   host_load1      — load average за 1 мин (/proc/loadavg)
#   host_cpu_pct    — загрузка CPU хоста за интервал (/proc/stat), 0–100 %
#   temp_c          — температура SoC (vcgencmd, иначе thermal_zone0)
#   container       — docker stats --no-stream: CPU% (100 % = одно ядро), память
#   npu             — один снимок `hailortcli monitor` (хост или контейнер
#                     vision-hailo), если hailortcli есть; иначе честная строка
#                     "NPU metric unavailable: <причина>".
# В конце печатает сводку (среднее/максимум) — её и прикладывать как raw.
#
# Использование:
#   ./measure_load.sh [--duration 120] [--interval 5] [--out-dir .]
#                     [--hailo-container vision-hailo]
#
# CSV: ts,host,kind,name,cpu_pct,mem_usage,mem_pct,value
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

DURATION=120
INTERVAL=5
OUT_DIR="."
HAILO_CONTAINER="vision-hailo"

usage() { sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --duration)        DURATION="${2:-}"; shift 2 ;;
        --interval)        INTERVAL="${2:-}"; shift 2 ;;
        --out-dir)         OUT_DIR="${2:-}"; shift 2 ;;
        --hailo-container) HAILO_CONTAINER="${2:-}"; shift 2 ;;
        -h|--help)         usage; exit 0 ;;
        *) echo "[measure_load] неизвестный аргумент: $1" >&2; usage >&2; exit 1 ;;
    esac
done

case "$DURATION$INTERVAL" in
    *[!0-9]*|"") echo "[measure_load] --duration/--interval — целые секунды" >&2; exit 1 ;;
esac
[ "$INTERVAL" -gt 0 ] || { echo "[measure_load] --interval > 0" >&2; exit 1; }

HOST="$(hostname -s 2>/dev/null || hostname)"
STAMP="$(date +%Y%m%dT%H%M%S)"
mkdir -p "$OUT_DIR"
CSV="$OUT_DIR/load_${HOST}_${STAMP}.csv"
echo "ts,host,kind,name,cpu_pct,mem_usage,mem_pct,value" > "$CSV"

row() { # kind name cpu_pct mem_usage mem_pct value
    echo "$(date +%s),$HOST,$1,$2,$3,$4,$5,$6" >> "$CSV"
}

cpu_times() { # -> "total idle" по первой строке /proc/stat (idle + iowait)
    awk '/^cpu /{t=0; for(i=2;i<=NF;i++) t+=$i; print t, $5+$6; exit}' /proc/stat
}

read_temp() {
    local t=""
    if command -v vcgencmd >/dev/null 2>&1; then
        t="$(vcgencmd measure_temp 2>/dev/null | sed -n "s/^temp=\([0-9.]*\).*/\1/p")"
    fi
    if [ -z "$t" ] && [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        t="$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)"
    fi
    echo "$t"
}

DOCKER_OK=false
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    DOCKER_OK=true
else
    row container unavailable "" "" "" "docker unavailable"
    echo "[measure_load] WARN: docker недоступен — нагрузка по контейнерам не снимается"
fi

sample_containers() {
    [ "$DOCKER_OK" = true ] || return 0
    docker stats --no-stream --format '{{.Name}}|{{.CPUPerc}}|{{.MemUsage}}|{{.MemPerc}}' 2>/dev/null \
        | while IFS='|' read -r name cpu mem memp; do
            row container "$name" "${cpu%\%}" "$mem" "${memp%\%}" ""
        done
}

# ── NPU: один снимок hailortcli monitor, иначе честное «нет метрики» ────────
npu_snapshot() {
    local raw="$OUT_DIR/npu_${HOST}_${STAMP}.txt"
    local -a cmd=()
    if command -v hailortcli >/dev/null 2>&1; then
        cmd=(hailortcli)
    elif [ "$DOCKER_OK" = true ] \
         && docker exec "$HAILO_CONTAINER" sh -c 'command -v hailortcli' >/dev/null 2>&1; then
        cmd=(docker exec "$HAILO_CONTAINER" hailortcli)
    fi
    if [ "${#cmd[@]}" -eq 0 ]; then
        row npu unavailable "" "" "" "NPU metric unavailable: hailortcli not found on host or in $HAILO_CONTAINER"
        echo "[measure_load] NPU metric unavailable: hailortcli нет ни на хосте, ни в $HAILO_CONTAINER"
        return 0
    fi
    # monitor — интерактивный экран; берём то, что он успел вывести за 5 с.
    # Данные появляются, только если у процесса инференса HAILO_MONITOR=1 (ADR-0137 §2.4).
    timeout 5 "${cmd[@]}" monitor > "$raw" 2>&1 || true
    if [ -s "$raw" ]; then
        row npu hailortcli "" "" "" "raw:$raw"
        echo "[measure_load] NPU: снимок hailortcli monitor → $raw (сырой, не разобран)"
    else
        row npu unavailable "" "" "" "NPU metric unavailable: hailortcli monitor printed nothing"
        echo "[measure_load] NPU metric unavailable: hailortcli monitor ничего не вывел"
    fi
}

echo "[measure_load] $HOST: ${DURATION} с, шаг ${INTERVAL} с → $CSV"
npu_snapshot

read -r prev_total prev_idle < <(cpu_times)
elapsed=0
while [ "$elapsed" -lt "$DURATION" ]; do
    sleep "$INTERVAL"
    elapsed=$((elapsed + INTERVAL))
    read -r cur_total cur_idle < <(cpu_times)
    cpu_pct="$(awk -v t=$((cur_total - prev_total)) -v i=$((cur_idle - prev_idle)) \
        'BEGIN{ if (t>0) printf "%.1f", 100*(1-i/t); else print "" }')"
    prev_total=$cur_total
    prev_idle=$cur_idle
    row host_cpu_pct host "$cpu_pct" "" "" ""
    row host_load1 host "" "" "" "$(cut -d' ' -f1 /proc/loadavg)"
    temp="$(read_temp)"
    [ -z "$temp" ] || row temp_c soc "" "" "" "$temp"
    sample_containers
done

# ── Сводка ───────────────────────────────────────────────────────────────────
echo ""
echo "===== СВОДКА $HOST ($(nproc) ядер; container CPU%: 100 = одно ядро) ====="
awk -F, 'NR>1 {
    if ($3=="host_cpu_pct" && $5!="") { hc_n++; hc_s+=$5; if ($5>hc_m) hc_m=$5 }
    if ($3=="host_load1")             { l_n++;  l_s+=$8;  if ($8>l_m)  l_m=$8 }
    if ($3=="temp_c")                 { if ($8>t_m) t_m=$8 }
    if ($3=="container" && $5!="")    { c_n[$4]++; c_s[$4]+=$5; if ($5>c_m[$4]) c_m[$4]=$5 }
}
END {
    if (hc_n) printf "host_cpu_pct: mean=%.1f max=%.1f (n=%d)\n", hc_s/hc_n, hc_m, hc_n
    if (l_n)  printf "load1:        mean=%.2f max=%.2f\n", l_s/l_n, l_m
    if (t_m)  printf "temp_c:       max=%.1f\n", t_m
    for (c in c_n) printf "container %-28s cpu_mean=%6.1f cpu_max=%6.1f\n", c, c_s[c]/c_n[c], c_m[c]
}' "$CSV" | sort
grep ',npu,' "$CSV" | cut -d, -f8 | sed 's/^/npu: /'
echo "CSV: $CSV"
