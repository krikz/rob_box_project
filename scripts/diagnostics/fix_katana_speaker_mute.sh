#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🔊 fix_katana_speaker_mute.sh
# ═══════════════════════════════════════════════════════════════════════════
# MSi Katana GF66 11UD (ALC3266 codec) — Node 0x17 (Speaker Pin Complex)
# остаётся в mute state на hardware-уровне, даже когда amixer "Speaker
# Playback Switch" показывает on.  amixer не пишет в аппаратный регистр
# codec widget 0x17, поэтому paplay "выходит", а динамик молчит.
#
# Симптомы (см. issue #1731):
#   - paplay exit=0, sink state RUNNING, pa volume 100%
#   - amixer "Speaker Playback Switch" = on/on
#   - cat /proc/asound/card0/codec#0 → Node 0x17 Amp-Out vals: [0x00 0x00]
#   - звука на динамике НЕТ
#
# Что делает скрипт:
#   hda-verb <hwC0D0> 0x17 SET_AMP 0xB027
#     - Verb:  SET_AMP (0x3)
#     - Param: 0xB027
#         * 0x80 = GET_AMP_L_MUTE / SET_AMP_L_MUTE bit
#         * 0xB0 = gain = 0xB0 (≈ 0 dB), MUTE-bit снят (бит 7 = 0)
#         * 0x27 = index/right-channel mask: оба канала, без mute
#
# Использование:
#   sudo ./scripts/diagnostics/fix_katana_speaker_mute.sh         # по умолчанию hwC0D0
#   sudo ./scripts/diagnostics/fix_katana_speaker_mute.sh hwC0D0 # явное имя
#   sudo ./scripts/diagnostics/fix_katana_speaker_mute.sh --check # только проверка (без записи)
#   sudo ./scripts/diagnostics/fix_katana_speaker_mute.sh --install-systemd
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
HDA_CARD="${1:-hwC0D0}"
NODE="0x17"
VERB="0x3"          # SET_AMP
PARAM="0xB027"      # оба канала, gain=0xB0 (≈0 dB), mute=0
MODE="${FIX_MODE:-apply}"  # apply | check

# ── Цвета для отчёта ───────────────────────────────────────────────────
if [ -t 1 ]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
    BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; BLUE=''; BOLD=''; NC=''
fi

log()   { echo -e "${BLUE}[${SCRIPT_NAME}]${NC} $*"; }
ok()    { echo -e "${GREEN}[✓]${NC} $*"; }
warn()  { echo -e "${YELLOW}[!]${NC} $*"; }
fail()  { echo -e "${RED}[✗]${NC} $*" >&2; }

usage() {
    cat <<EOF
Usage: sudo $SCRIPT_NAME [HDA_CARD] [--check | --install-systemd]
  HDA_CARD          имя HDA-устройства (по умолчанию hwC0D0)
  --check           только проверка состояния Node 0x17
  --install-systemd установить systemd unit + скрипт и enable (требует root)
  --help            эта справка
EOF
}

# ── Парсинг аргументов ─────────────────────────────────────────────────
INSTALL_SYSTEMD=0
for arg in "$@"; do
    case "$arg" in
        --check)           MODE="check" ;;
        --install-systemd) INSTALL_SYSTEMD=1 ;;
        --help|-h)         usage; exit 0 ;;
        hwC[0-9]D[0-9])    HDA_CARD="$arg" ;;
        *) ;;
    esac
done

# ── Установка systemd unit ─────────────────────────────────────────────
install_systemd() {
    log "Установка systemd unit для fix-katana-speaker.service"

    if [ "$EUID" -ne 0 ]; then
        fail "Требуется root (sudo $SCRIPT_NAME --install-systemd)"
        exit 1
    fi

    if ! command -v hda-verb >/dev/null 2>&1; then
        warn "hda-verb не найден — ставлю alsa-tools"
        apt-get update -qq && apt-get install -y -qq alsa-tools
    fi

    local lib_dir
    lib_dir="$(cd "$(dirname "$0")/.." && pwd)"
    local svc_src="${lib_dir}/maintenance/systemd/fix-katana-speaker.service"
    if [ ! -f "$svc_src" ]; then
        fail "Не найден unit-файл: $svc_src"
        exit 1
    fi

    install -m 0644 "$svc_src" /etc/systemd/system/fix-katana-speaker.service
    systemctl daemon-reload
    systemctl enable fix-katana-speaker.service
    systemctl start  fix-katana-speaker.service

    ok "systemd unit enabled. Статус:"
    systemctl --no-pager --full status fix-katana-speaker.service | sed 's/^/    /'
}

# ── Preflight ──────────────────────────────────────────────────────────
HDA_DEV="/dev/snd/${HDA_CARD}"
CODEC_FILE="/proc/asound/card0/codec#0"

if [ ! -e "$HDA_DEV" ]; then
    fail "HDA-устройство ${HDA_DEV} не найдено. Доступные:"
    ls -1 /dev/snd/ 2>/dev/null | sed 's/^/    /' || true
    exit 2
fi

if [ ! -r "$CODEC_FILE" ]; then
    fail "Не удаётся прочитать ${CODEC_FILE} (нужен root)"
    exit 3
fi

if [ "$INSTALL_SYSTEMD" -eq 1 ]; then
    install_systemd
    exit 0
fi

if [ "$MODE" = "check" ]; then
    log "CHECK: читаю ${CODEC_FILE} → Node ${NODE}"
    if grep -A6 "Node ${NODE} \[Pin Complex\]" "$CODEC_FILE" | grep -q "Amp-Out vals:.*\[0x00 0x00\]"; then
        fail "Node ${NODE} Amp-Out vals = [0x00 0x00] — MUTED"
        exit 10
    fi
    ok "Node ${NODE} Amp-Out vals ≠ [0x00 0x00] — OK"
    exit 0
fi

# ── Apply ──────────────────────────────────────────────────────────────
log "Карта: ${BOLD}${HDA_DEV}${NC}, Node ${NODE}, verb=SET_AMP(0x3), param=${PARAM}"

if ! command -v hda-verb >/dev/null 2>&1; then
    warn "hda-verb не установлен. Ставлю alsa-tools (нужен root)…"
    if [ "$EUID" -ne 0 ]; then
        fail "Запусти от root: sudo apt install alsa-tools && $SCRIPT_NAME"
        exit 4
    fi
    apt-get update -qq && apt-get install -y -qq alsa-tools
fi

if [ "$EUID" -ne 0 ]; then
    fail "hda-verb требует root. Запусти: sudo $SCRIPT_NAME"
    exit 5
fi

log "Перед фиксом:"
BEFORE="$(grep -A6 "Node ${NODE} \[Pin Complex\]" "$CODEC_FILE" | grep "Amp-Out vals:" || true)"
echo "    ${BEFORE}"

hda-verb "${HDA_DEV}" "${NODE}" "${VERB}" "${PARAM}"
ok "hda-verb выполнен"

log "После фикса:"
AFTER="$(grep -A6 "Node ${NODE} \[Pin Complex\]" "$CODEC_FILE" | grep "Amp-Out vals:" || true)"
echo "    ${AFTER}"

if echo "$AFTER" | grep -q "\[0x00 0x00\]"; then
    fail "Node ${NODE} всё ещё [0x00 0x00]. Фикс не сработал."
    exit 6
fi

ok "Node ${NODE} unmute SUCCESS — динамик должен звучать."
warn "Это TEMPORARY фикс (до reboot). Чтобы закрепить:"
echo "    sudo $SCRIPT_NAME --install-systemd"

exit 0
