#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🔧 Синхронизация системного времени (NTP)
# ═══════════════════════════════════════════════════════════════════════════
# Лечит рассинхронизацию часов между Pi, из-за которой zenoh-router режет
# входящие данные ("Error treating timestamp ... exceeding delta 500ms is
# rejected") и деплой-воркфлоу создаёт ложные critical issue.
#
# Корневая причина: systemd-timesyncd по умолчанию ходит на ntp.ubuntu.com,
# который из локальной сети 10.1.1.x часто недоступен (UDP 123 таймаутит),
# а ru.pool.ntp.org отвечает нормально.
#
# Использование:
#   sudo ./sync_time.sh            # настроить NTP и синхронизировать
#   sudo ./sync_time.sh --check    # только проверить, ничего не менять
#
# Скрипт идемпотентный: повторный запуск безопасен.
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

# Рабочие NTP-серверы (проверено из сети 10.1.1.x: ru.pool.ntp.org отвечает,
# ntp.ubuntu.com — нет). FallbackNTP оставляем как запасной вариант.
NTP_SERVERS="ru.pool.ntp.org 0.ru.pool.ntp.org 1.ru.pool.ntp.org"
FALLBACK_NTP="pool.ntp.org ntp.ubuntu.com"
TIMESYNCD_CONF="/etc/systemd/timesyncd.conf"
SYNC_TIMEOUT_SEC="${SYNC_TIMEOUT_SEC:-30}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[sync_time]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[sync_time]${NC} $*"; }
log_error() { echo -e "${RED}[sync_time]${NC} $*"; }

CHECK_ONLY=false
[ "${1:-}" = "--check" ] && CHECK_ONLY=true

# ── Проверка прав ────────────────────────────────────────────────────────────
if [ "$(id -u)" -ne 0 ]; then
    log_error "Нужны права root. Запустите: sudo $0 $*"
    exit 1
fi

# ── Текущее состояние ────────────────────────────────────────────────────────
log_info "Текущее состояние времени:"
timedatectl 2>/dev/null || true
echo ""

if ! timedatectl 2>/dev/null | grep -q "NTP service: active"; then
    log_warn "NTP service не активен — включаем..."
    if [ "$CHECK_ONLY" = true ]; then
        log_error "NTP service не активен (--check, ничего не меняем)"
        exit 2
    fi
    timedatectl set-ntp true || true
fi

# ── Конфигурация NTP-серверов ────────────────────────────────────────────────
write_timesyncd_conf() {
    cat > "$TIMESYNCD_CONF" <<EOF
[Time]
NTP=$NTP_SERVERS
FallbackNTP=$FALLBACK_NTP
RootDistanceMaxSec=5
PollIntervalMinSec=32
PollIntervalMaxSec=2048
EOF
}

if [ "$CHECK_ONLY" = true ]; then
    log_info "Текущий NTP-конфиг ($TIMESYNCD_CONF):"
    grep -E "^(NTP|FallbackNTP)" "$TIMESYNCD_CONF" 2>/dev/null || echo "  (серверы не заданы)"
    echo ""
    if timedatectl 2>/dev/null | grep -q "System clock synchronized: yes"; then
        log_info "Часы синхронизированы ✅"
        exit 0
    fi
    log_error "Часы НЕ синхронизированы (--check, ничего не меняем)"
    exit 2
fi

# Обновляем конфиг, только если там нет наших серверов
if ! grep -q "ru.pool.ntp.org" "$TIMESYNCD_CONF" 2>/dev/null; then
    log_info "Прописываем рабочие NTP-серверы ($NTP_SERVERS)..."
    write_timesyncd_conf
else
    log_info "NTP-серверы уже настроены, пропускаем запись конфига"
fi

# ── Рестарт и ожидание синхронизации ────────────────────────────────────────
log_info "Перезапускаем systemd-timesyncd..."
systemctl restart systemd-timesyncd || log_warn "Не удалось перезапустить systemd-timesyncd (продолжаем)"

log_info "Ожидаем синхронизацию (до ${SYNC_TIMEOUT_SEC}с)..."
SYNCED=false
for i in $(seq 1 "$SYNC_TIMEOUT_SEC"); do
    if timedatectl 2>/dev/null | grep -q "System clock synchronized: yes"; then
        SYNCED=true
        break
    fi
    sleep 1
done

echo ""
log_info "Итоговое состояние:"
timedatectl 2>/dev/null || true
echo ""

if [ "$SYNCED" = true ]; then
    log_info "Часы синхронизированы ✅"
    exit 0
fi

# ── Резервный вариант: принудительная установка из NTP-ответа ────────────────
log_warn "systemd-timesyncd не смог синхронизироваться за ${SYNC_TIMEOUT_SEC}с."
log_warn "Пробуем принудительную синхронизацию через chrony/ntpdate (если доступны)..."

if command -v chronyd >/dev/null 2>&1; then
    systemctl restart chrony 2>/dev/null || chronyd -q 2>/dev/null || true
elif command -v ntpdate >/dev/null 2>&1; then
    ntpdate -u "$NTP_SERVERS" 2>/dev/null || true
elif command -v sntp >/dev/null 2>&1; then
    # sntp есть в пакете ntpdate; используем первый сервер
    first_server=$(echo "$NTP_SERVERS" | awk '{print $1}')
    sntp -sS "$first_server" 2>/dev/null || true
else
    log_error "Нет chrony/ntpdate/sntp для принудительной синхронизации."
    log_error "Установите: sudo apt install -y chrony  (или ntpdate)"
    log_error "Или вручную: sudo timedatectl set-time '$(date -u '+%Y-%m-%d %H:%M:%S')'"
    exit 3
fi

# Повторная проверка
if timedatectl 2>/dev/null | grep -q "System clock synchronized: yes"; then
    log_info "Принудительная синхронизация удалась ✅"
    exit 0
fi

log_error "Синхронизация времени не удалась. Проверьте сеть до NTP-серверов:"
log_error "  timeout 3 bash -c 'echo > /dev/udp/$first_server/123'"
exit 3
