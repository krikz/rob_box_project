#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🕒 Общее время двух Pi через chrony (ADR-0137, этап 0 ADR-0130 §2.11 / Q14)
# ═══════════════════════════════════════════════════════════════════════════
# Топология:
#   Main Pi   (--role main)   — сервер времени: внешние NTP-пулы + раздаёт
#                                время в 10.1.1.0/24; при отсутствии интернета
#                                продолжает раздавать своё (local stratum 10),
#                                чтобы межпайное время оставалось общим.
#   Vision Pi (--role vision) — клиент ТОЛЬКО Main Pi (без внешних пулов):
#                                межпайное расхождение не должно зависеть
#                                от интернета. Цель: |offset| < 5 мс.
#
# chrony ставится на ХОСТ Pi, не в контейнер: системные часы — общие для
# ядра, контейнерам их менять нельзя (нужен CAP_SYS_TIME). systemd-timesyncd
# при этом отключается: два демона времени на одних часах — конфликт.
# Прежний scripts/maintenance/sync_time.sh (timesyncd) при активном chrony
# ничего не меняет и делегирует проверку сюда (ADR-0137 §2.5).
#
# Использование:
#   sudo ./setup_chrony.sh --role main
#   sudo ./setup_chrony.sh --role vision [--server 10.1.1.10]
#   ./setup_chrony.sh --check [--role main|vision] [--server IP] [--max-offset-ms 5]
#   ./setup_chrony.sh --print-config --role main|vision [--server IP]
#
# --check ничего не меняет и root не требует. Роль и сервер по умолчанию
# берутся из /etc/chrony/chrony.conf, если его писал этот скрипт (маркер).
#
# Коды выхода: 0 — OK; 1 — ошибка аргументов/прав; 2 — проверка не прошла
# (|offset| > порога, источник не Main Pi, нет выбранного источника);
# 3 — chrony не установлен / chronyd не отвечает.
#
# Переменные окружения (для тестов): CHRONY_CONF, CHRONYC (команда chronyc
# целиком, например "chronyc -h 127.0.0.1 -p 11324").
#
# Скрипт идемпотентный: повторный запуск безопасен.
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

# Main Pi на проводном линке с Vision Pi: eth0 = 10.1.1.10 (тот же адрес, что
# docker/vision/config/zenoh_router_config.json5 "tcp/10.1.1.10:7447#iface=eth0").
# 10.1.1.20 — это Wi-Fi Main Pi (SSH), время по Wi-Fi не гоняем (ADR-0130 §2.11).
DEFAULT_SERVER="10.1.1.10"
ALLOW_SUBNET="10.1.1.0/24"
# Те же внешние серверы, что в sync_time.sh (из 10.1.1.x отвечают; ntp.ubuntu.com — нет).
NTP_POOLS="ru.pool.ntp.org 0.ru.pool.ntp.org 1.ru.pool.ntp.org"
FALLBACK_POOL="pool.ntp.org"
CHRONY_CONF="${CHRONY_CONF:-/etc/chrony/chrony.conf}"
MARKER="# managed-by: scripts/maintenance/setup_chrony.sh (ADR-0137)"
WAITSYNC_TRIES="${WAITSYNC_TRIES:-30}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[setup_chrony]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[setup_chrony]${NC} $*"; }
log_error() { echo -e "${RED}[setup_chrony]${NC} $*" >&2; }

usage() {
    sed -n '2,36p' "$0" | sed 's/^# \{0,1\}//'
}

ROLE=""
SERVER=""
MAX_OFFSET_MS="5"
MODE="apply"

while [ $# -gt 0 ]; do
    case "$1" in
        --role)          ROLE="${2:-}"; shift 2 ;;
        --server)        SERVER="${2:-}"; shift 2 ;;
        --max-offset-ms) MAX_OFFSET_MS="${2:-}"; shift 2 ;;
        --check)         MODE="check"; shift ;;
        --print-config)  MODE="print"; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) log_error "Неизвестный аргумент: $1"; usage >&2; exit 1 ;;
    esac
done

read -r -a CHRONYC_CMD <<< "${CHRONYC:-chronyc}"

detect_role_from_conf() {
    # Роль из маркера "# role: main|vision", который пишет generate_config.
    [ -r "$CHRONY_CONF" ] || return 0
    grep -q "^${MARKER}\$" "$CHRONY_CONF" || return 0
    sed -n 's/^# role: \(main\|vision\)$/\1/p' "$CHRONY_CONF" | head -n 1
}

detect_server_from_conf() {
    [ -r "$CHRONY_CONF" ] || return 0
    grep -q "^${MARKER}\$" "$CHRONY_CONF" || return 0
    awk '$1=="server"{print $2; exit}' "$CHRONY_CONF"
}

validate_role() {
    case "$ROLE" in
        main|vision) ;;
        "") log_error "Роль не задана: --role main|vision (или конфиг без маркера setup_chrony.sh)"; exit 1 ;;
        *)  log_error "Неверная роль '$ROLE': ожидается main или vision"; exit 1 ;;
    esac
}

generate_config() {
    echo "$MARKER"
    echo "# role: $ROLE"
    echo "# Не править руками: перезаписывается setup_chrony.sh. Обоснование — docs/adr/0137-*.md"
    echo ""
    if [ "$ROLE" = "main" ]; then
        echo "# Внешнее время (абсолютное). Межпайное — от этого не зависит (Vision ходит только сюда)."
        for pool in $NTP_POOLS; do
            echo "pool $pool iburst maxsources 2"
        done
        echo "pool $FALLBACK_POOL iburst maxsources 2"
        echo ""
        echo "# Раздаём время Vision Pi."
        echo "allow $ALLOW_SUBNET"
        echo "# Без интернета продолжаем раздавать своё время: общее для двух Pi важнее абсолютного."
        echo "local stratum 10"
    else
        echo "# Только Main Pi (проводной линк). Внешних пулов нет намеренно:"
        echo "# межпайное расхождение не должно зависеть от интернета (ADR-0137)."
        echo "server $SERVER iburst prefer minpoll 2 maxpoll 4"
    fi
    echo ""
    echo "driftfile /var/lib/chrony/chrony.drift"
    echo "makestep 1.0 3"
    echo "rtcsync"
    echo "logdir /var/log/chrony"
}

# ── Проверка (read-only) ─────────────────────────────────────────────────────
run_check() {
    local src_csv selected sel_addr sel_off sel_err off_ms err_ms verdict
    if ! command -v "${CHRONYC_CMD[0]}" >/dev/null 2>&1; then
        log_error "chronyc не найден — chrony не установлен. Запустите: sudo $0 --role <main|vision>"
        return 3
    fi
    if ! src_csv=$("${CHRONYC_CMD[@]}" -n -c sources 2>&1); then
        log_error "chronyc не отвечает (chronyd не запущен?): $src_csv"
        return 3
    fi

    log_info "chronyc tracking:"
    "${CHRONYC_CMD[@]}" -n tracking || true
    echo ""
    log_info "chronyc sources:"
    "${CHRONYC_CMD[@]}" -n sources || true
    echo ""

    # CSV sources: mode,state,address,stratum,poll,reach,lastrx,adj_offset_s,meas_offset_s,error_s
    selected=$(printf '%s\n' "$src_csv" | awk -F, '$2=="*"{print $3","$8","$10; exit}')
    IFS=, read -r sel_addr sel_off sel_err <<< "${selected:-,,}"

    if [ -n "$sel_addr" ]; then
        off_ms=$(awk -v v="$sel_off" 'BEGIN{v=v*1000; if(v<0)v=-v; printf "%.3f", v}')
        err_ms=$(awk -v v="$sel_err" 'BEGIN{printf "%.3f", v*1000}')
    else
        off_ms="nan"
        err_ms="nan"
    fi

    verdict="PASS"
    if [ "$ROLE" = "vision" ]; then
        if [ -z "$sel_addr" ]; then
            log_error "Нет выбранного источника (^*): Vision Pi не синхронизирован с Main Pi"
            verdict="FAIL"
        elif [ "$sel_addr" != "$SERVER" ]; then
            log_error "Источник времени $sel_addr, а должен быть Main Pi ($SERVER)"
            verdict="FAIL"
        elif awk -v o="$off_ms" -v m="$MAX_OFFSET_MS" 'BEGIN{exit !(o>m)}'; then
            log_error "|offset| к Main Pi = ${off_ms} мс > ${MAX_OFFSET_MS} мс"
            verdict="FAIL"
        fi
    else
        if ! grep -q "^allow " "$CHRONY_CONF" 2>/dev/null; then
            log_error "В $CHRONY_CONF нет 'allow': Main Pi не раздаёт время Vision Pi"
            verdict="FAIL"
        fi
        if [ -z "$sel_addr" ]; then
            # Не FAIL: межпайное время остаётся общим (Vision ходит сюда), уплывает только абсолютное.
            log_warn "Внешний источник не выбран — Main Pi на 'local stratum 10' (нет интернета?)."
        fi
    fi

    # Одна строка для grep в логах/отчётах (raw-evidence, ADR-0018).
    echo "chrony_check role=$ROLE source=${sel_addr:-none} offset_ms=$off_ms error_ms=$err_ms max_ms=$MAX_OFFSET_MS verdict=$verdict"
    [ "$verdict" = "PASS" ] || return 2
    return 0
}

# ── Режимы ───────────────────────────────────────────────────────────────────
if [ "$MODE" = "check" ]; then
    [ -n "$ROLE" ] || ROLE="$(detect_role_from_conf)"
    [ -n "$SERVER" ] || SERVER="$(detect_server_from_conf)"
fi
[ -n "$SERVER" ] || SERVER="$DEFAULT_SERVER"

if [ "$MODE" = "print" ]; then
    validate_role
    generate_config
    exit 0
fi

if [ "$MODE" = "check" ]; then
    validate_role
    rc=0
    run_check || rc=$?
    exit "$rc"
fi

# apply
validate_role
if [ "$(id -u)" -ne 0 ]; then
    log_error "Нужны права root. Запустите: sudo $0 --role $ROLE"
    exit 1
fi

if ! command -v chronyd >/dev/null 2>&1; then
    log_info "Устанавливаю chrony (apt)..."
    apt-get update -q
    DEBIAN_FRONTEND=noninteractive apt-get install -y -q chrony
else
    log_info "chrony уже установлен: $(chronyd --version 2>/dev/null | head -n 1)"
fi

# Два демона времени на одних часах — конфликт. На Ubuntu 22.04+ пакет chrony
# обычно сам удаляет systemd-timesyncd; если unit остался — выключаем.
if systemctl list-unit-files systemd-timesyncd.service >/dev/null 2>&1 \
   && systemctl list-unit-files systemd-timesyncd.service | grep -q '^systemd-timesyncd.service'; then
    log_info "Отключаю systemd-timesyncd..."
    systemctl disable --now systemd-timesyncd || log_warn "Не удалось отключить systemd-timesyncd"
fi

tmp_conf="$(mktemp)"
trap 'rm -f "$tmp_conf"' EXIT
generate_config > "$tmp_conf"

changed=false
if [ -f "$CHRONY_CONF" ] && cmp -s "$tmp_conf" "$CHRONY_CONF"; then
    log_info "$CHRONY_CONF уже в нужном виде, не трогаю"
else
    if [ -f "$CHRONY_CONF" ]; then
        backup="${CHRONY_CONF}.bak.$(date +%Y%m%d%H%M%S)"
        cp -a "$CHRONY_CONF" "$backup"
        log_info "Бэкап прежнего конфига: $backup"
    fi
    install -D -m 0644 "$tmp_conf" "$CHRONY_CONF"
    changed=true
    log_info "Записан $CHRONY_CONF (role=$ROLE)"
fi

systemctl enable chrony >/dev/null 2>&1 || log_warn "systemctl enable chrony не удался"
if [ "$changed" = true ] || ! systemctl is-active --quiet chrony; then
    log_info "Перезапускаю chrony..."
    systemctl restart chrony
fi

log_info "Жду синхронизацию (до ${WAITSYNC_TRIES} с)..."
if [ "$ROLE" = "vision" ]; then
    # waitsync <tries> <max-correction, s> <max-skew> <interval, s>
    "${CHRONYC_CMD[@]}" waitsync "$WAITSYNC_TRIES" 0.005 0 1 || log_warn "waitsync: не дождались |коррекции| < 5 мс"
else
    "${CHRONYC_CMD[@]}" waitsync "$WAITSYNC_TRIES" 0 0 1 || log_warn "waitsync: внешний источник не выбран (нет интернета?)"
fi
log_info "chronyc makestep (шаг часов сразу, без медленного подтягивания)..."
"${CHRONYC_CMD[@]}" makestep || log_warn "makestep не выполнен"

echo ""
rc=0
run_check || rc=$?
exit "$rc"
