#!/usr/bin/env bash
# scripts/vision_pi_healthcheck.sh
#
# Идемпотентная диагностика Vision Pi (10.1.1.21) для on-call runbook
# `docs/runbooks/vision-pi-down-recovery.md` (issue #2648).
#
# Назначение: помочь on-call инженеру, который подошёл к Pi (--local)
# или работает с builder/249 (--remote), собрать read-only слепок
# состояния Vision Pi:
#   - питание, аптайм, throttling (Raspberry Pi);
#   - сетевые интерфейсы (wlan0=10.1.1.21, eth0=10.1.1.11) и ARP;
#   - WiFi link (ESSID=ROS2, signal);
#   - docker-контейнеры (voice-assistant, hailort, zenoh);
#   - journalctl NetworkManager за последние 5 минут;
#   - swap (после PR #2643 ожидается zram0).
#
# Скрипт НЕ правит конфиги, НЕ рестартит сервисы, НЕ трогает SD-карту.
# Все действия read-only. Идемпотентность — можно запускать подряд много
# раз, ничего не ломается.
#
# Использование:
#   bash scripts/vision_pi_healthcheck.sh --local
#   bash scripts/vision_pi_healthcheck.sh --remote
#   bash scripts/vision_pi_healthcheck.sh --help
#
# Переменные окружения (опционально, переопределяют дефолты):
#   VISION_PI_HOST   — IP Vision Pi (по умолчанию 10.1.1.21)
#   VISION_PI_USER   — ssh user (по умолчанию ros2)
#   VISION_PI_ETH_IP — ethernet-адрес для fallback (по умолчанию 10.1.1.11)

set -uo pipefail

# ---------- Дефолты ----------
VISION_PI_HOST="${VISION_PI_HOST:-10.1.1.21}"
VISION_PI_USER="${VISION_PI_USER:-ros2}"
VISION_PI_ETH_IP="${VISION_PI_ETH_IP:-10.1.1.11}"
SCRIPT_NAME="$(basename "$0")"

# ---------- Утилиты ----------
die() { printf '❌ %s\n' "$*" >&2; exit 1; }
log() { printf '%s\n' "$*"; }

have_cmd() { command -v "$1" >/dev/null 2>&1; }

# ---------- Аргументы ----------
MODE=""
if [ $# -eq 0 ]; then
    die "Не указан режим. Используйте --local или --remote. Запустите с --help."
fi
case "$1" in
    --local)  MODE="local" ;;
    --remote) MODE="remote" ;;
    -h|--help|help)
        cat <<EOF
$SCRIPT_NAME — идемпотентная диагностика Vision Pi ($VISION_PI_HOST).

Использование:
  $SCRIPT_NAME --local    запустить НА самой Pi (через консоль/eth0)
  $SCRIPT_NAME --remote   запустить с builder/249, скрипт сам подключится
                          по ssh и выполнит --local на Pi
  $SCRIPT_NAME --help     эта справка

Переменные окружения:
  VISION_PI_HOST    IP Vision Pi (по умолчанию 10.1.1.21)
  VISION_PI_USER    ssh user (по умолчанию ros2)
  VISION_PI_ETH_IP  ethernet-fallback (по умолчанию 10.1.1.11)

Скрипт READ-ONLY и ИДЕМПОТЕНТНЫЙ. Ничего не правит, не рестартит, не
трогает SD-карту. Подробности — docs/runbooks/vision-pi-down-recovery.md.
EOF
        exit 0 ;;
    *)
        die "Неизвестный аргумент: $1. Используйте --local, --remote или --help."
        ;;
esac

# ---------- Хедер ----------
print_header() {
    log "=================================================="
    log " Vision Pi Healthcheck — $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
    log " Режим:    $MODE"
    log " Хост:     $VISION_PI_HOST (eth0=$VISION_PI_ETH_IP)"
    log " Скрипт:   $SCRIPT_NAME (read-only, идемпотентный)"
    log "=================================================="
    log ""
}

# ---------- Секции (каждая read-only) ----------

section_power() {
    log "[1] Питание и аптайм"
    log "---"
    if have_cmd uptime; then
        log "uptime:"; uptime
    fi
    if have_cmd vcgencmd; then
        log ""
        log "vcgencmd get_throttled (0x0 = ок):"
        vcgencmd get_throttled 2>/dev/null || echo "  (vcgencmd недоступен)"
    else
        log "(vcgencmd недоступен — не Raspberry Pi?)"
    fi
    if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        local temp_c
        temp_c=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
        log ""
        log "CPU температура: ${temp_c:-?} °C"
    fi
    log ""
}

section_network() {
    log "[2] Сеть: ip a (только IPv4)"
    log "---"
    if have_cmd ip; then
        ip -4 -o addr show | awk '{printf "  %-10s inet %-18s scope %s %s\n", $2, $4, $6, $7}'
    else
        log "(ip недоступен)"
    fi
    log ""
}

section_arp() {
    log "[3] ARP: ip neigh (ожидаем REACHABLE для роутера и Main Pi)"
    log "---"
    if have_cmd ip; then
        ip neigh show | head -20 || true
    else
        log "(ip neigh недоступен)"
    fi
    log ""
}

section_wifi() {
    log "[4] WiFi: iw dev wlan0 link (ожидаем ESSID=ROS2, signal > -75 dBm)"
    log "---"
    if have_cmd iw; then
        iw dev wlan0 link 2>&1 | sed 's/^/  /' || echo "  (wlan0 отсутствует?)"
    else
        log "(iw недоступен)"
    fi
    log ""
}

section_docker() {
    log "[5] Docker: контейнеры (ожидаем running: voice-assistant, hailort, zenoh)"
    log "---"
    if have_cmd docker; then
        docker ps --format '  table {{.Names}}\t{{.Status}}' 2>&1 | grep -E 'voice-assistant|hailort|zenoh|oak-d|apriltag' || echo "  (ничего из критичных сервисов не запущено)"
    else
        log "(docker недоступен)"
    fi
    log ""
}

section_journal() {
    log "[6] Журнал NetworkManager (--since '-5 min')"
    log "---"
    if have_cmd journalctl; then
        journalctl -u NetworkManager --since '-5 min' --no-pager 2>&1 | tail -40 || echo "  (журнал недоступен / требуются права)"
    else
        log "(journalctl недоступен)"
    fi
    log ""
}

section_swap() {
    log "[7] swap (после PR #2643 ожидаем /dev/zram0)"
    log "---"
    if have_cmd swapon; then
        swapon --show 2>&1 || echo "  (swap не активен)"
    else
        log "(swapon недоступен)"
    fi
    if have_cmd free; then
        log ""
        log "free -h:"
        free -h | sed 's/^/  /'
    fi
    log ""
}

# ---------- Главные режимы ----------

run_local() {
    # Проверка, что мы действительно на Pi (хотя бы uname -r совпадает)
    log "Проверка, что мы на Pi..."
    if have_cmd uname; then
        log "  $(uname -a)"
    fi
    log ""

    section_power
    section_network
    section_arp
    section_wifi
    section_docker
    section_journal
    section_swap

    log "=================================================="
    log " Готово. Следующий шаг — docs/runbooks/vision-pi-down-recovery.md"
    log " Шаг 5: подтвердить с двух хостов (builder + 249)."
    log "=================================================="
}

run_remote() {
    # 1. Сначала проверяем, что Pi вообще отвечает
    log "Проверка доступности $VISION_PI_HOST..."
    if ! ping -c 2 -W 2 "$VISION_PI_HOST" >/dev/null 2>&1; then
        log "❌ ping $VISION_PI_HOST НЕ прошёл — Pi не доступен по WiFi."
        log "   Попробую ethernet ($VISION_PI_ETH_IP)..."
        if ! ping -c 2 -W 2 "$VISION_PI_ETH_IP" >/dev/null 2>&1; then
            die "И WiFi ($VISION_PI_HOST), и ethernet ($VISION_PI_ETH_IP) недоступны. Pi либо выключен, либо не в сети. См. runbook § 4c (SD/swap)."
        fi
        log "✅ ethernet доступен ($VISION_PI_ETH_IP). Использую его."
        VISION_PI_HOST="$VISION_PI_ETH_IP"
    else
        log "✅ WiFi доступен ($VISION_PI_HOST)."
    fi
    log ""

    # 2. Проверка ssh
    log "Проверка ssh ${VISION_PI_USER}@${VISION_PI_HOST}..."
    if ! have_cmd ssh; then
        die "ssh не установлен на хосте, с которого запущен --remote."
    fi
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
            "${VISION_PI_USER}@${VISION_PI_HOST}" 'echo OK' >/dev/null 2>&1; then
        die "ssh на ${VISION_PI_USER}@${VISION_PI_HOST} не прошёл. Возможно нужен sshpass или ключ. См. local_test/README.md § 'SSH доступ'."
    fi
    log "✅ ssh ок."
    log ""

    # 3. Транслируем stdin (наш скрипт) на Pi и запускаем там в --local
    log "Транслирую скрипт на ${VISION_PI_HOST} в режиме --local..."
    log ""

    # shellcheck disable=SC2029  # намеренно: переменные окружения не нужны на Pi
    ssh -o ConnectTimeout=10 -o BatchMode=yes "${VISION_PI_USER}@${VISION_PI_HOST}" \
        'bash -s -- --local' < "$0"
    local rc=$?
    log ""
    if [ "$rc" -eq 0 ]; then
        log "=================================================="
        log " Удалённый сбор данных завершён."
        log " Следующий шаг — docs/runbooks/vision-pi-down-recovery.md § 5"
        log " (кросс-проверка с двух хостов: builder + 249)."
        log "=================================================="
    else
        log "❌ Удалённый запуск вернул код $rc. Проверьте ssh/сеть."
        exit "$rc"
    fi
}

# ---------- main ----------
print_header
case "$MODE" in
    local)  run_local ;;
    remote) run_remote ;;
esac