#!/usr/bin/env bash
# local_test/quest_smoke.sh — Meta Quest WebXR smoke test (Phase 1.7, ADR-0027 §6).
#
# Контракт (issue #1643 acceptance):
#   --check-only : DNS resolve + TLS cert (subject, SAN, expiry) — без живого сервера.
#                  Запускается прямо на dev-машине для raw-evidence.
#   --all        : --check-only + /healthz + WSS handshake (HELLO/WELCOME)
#                  + heartbeat interval (median ~200ms ±50)
#                  + SUBSCRIBE→BINARY_FRAME JPEG (end-to-end)
#                  + GOODBYE → server close.
#                  Требует ЗАПУЩЕННЫЙ Phase 1.6 (Caddy + quest_node).
#   --json       : machine-readable вывод.
#   --no-color   : выкл ANSI (для CI/log capture).
#   --host, --port, --pin : overrides (по умолчанию quest.rob_box.local:8443,
#                  PIN из $QUEST_PIN или $QUEST_PIN_FILE).
#
# Exit codes (issue acceptance: "exit codes 0..N по разделам"):
#   0  : все секции PASS.
#   99 : partial (есть SKIP, но FAIL нет).
#   bitwise-OR fail bits:
#       1  dns                5  heartbeat
#       2  tls                6  subscribe_jpeg
#       3  healthz            7  goodbye
#       4  wss_handshake
#   64 : usage / unknown flag (EX_USAGE).
#   65 : prerequisite (websockets lib, python3) missing.
#
# WIP-scaffold (issue #1643): WIP-коммит с --check-only возможен уже сейчас,
# остальные проверки добиваются после deploy Phase 1.6 (#1641).
#
# Источники истины:
#   - docs/architecture/meta-quest-api.md §2/§3/§7/§8 (wire protocol + heartbeat)
#   - src/rob_box_quest/rob_box_quest/protocol/frame.py (frame codec contract)
#   - src/rob_box_quest/rob_box_quest/server/session.py (HEARTBEAT_INTERVAL_S, WATCHDOG)
#   - docker/vision/quest/Caddyfile (TLS + reverse_proxy flush_interval=-1)
#   - docker/vision/scripts/quest/start_quest.sh (self-signed cert generation)
#
# ADR-0018: «Честный FAIL лучше красивого PASS». Этот скрипт НЕ маскирует
# ошибки: пропущенная зависимость → exit 65, отсутствующий сервер → fail
# в соответствующей секции (НЕ silent skip). См. --check-only для явного
# режима без сервера.

set -euo pipefail

# --- Defaults ----------------------------------------------------------------
HOST="${QUEST_HOST:-quest.rob_box.local}"
PORT="${QUEST_PORT:-8443}"
PIN="${QUEST_PIN:-}"
PIN_FILE="${QUEST_PIN_FILE:-/var/run/quest/pin}"  # start_quest.sh пишет PIN сюда
MODE="all"  # 'all' | 'check-only'
JSON_OUT=0
NO_COLOR=0

LIB_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LIB_PY="${LIB_DIR}/quest_smoke_lib.py"

# --- Arg parsing -------------------------------------------------------------
usage() {
    sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        --check-only) MODE="check-only"; shift ;;
        --all)        MODE="all";        shift ;;
        --host)       HOST="$2";         shift 2 ;;
        --port)       PORT="$2";         shift 2 ;;
        --pin)        PIN="$2";          shift 2 ;;
        --pin-file)   PIN_FILE="$2";     shift 2 ;;
        --json)       JSON_OUT=1;        shift ;;
        --no-color)   NO_COLOR=1;        shift ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "FATAL: unknown flag '$1' (try --help)" >&2; exit 64 ;;
    esac
done

# --- Prereqs ------------------------------------------------------------------
command -v python3 >/dev/null 2>&1 || {
    echo "FATAL: python3 not found in PATH" >&2
    exit 65
}

if [ ! -f "${LIB_PY}" ]; then
    echo "FATAL: quest_smoke_lib.py not found at ${LIB_PY}" >&2
    echo "  (quest_smoke.sh и библиотека должны лежать рядом в local_test/)" >&2
    exit 65
fi

# Sanity-check что websockets установлен — иначе exit 65, а не fail в WSS.
if ! python3 -c "import websockets" 2>/dev/null; then
    echo "FATAL: python3 module 'websockets' not installed." >&2
    echo "  Установите: pip3 install 'websockets>=10'" >&2
    exit 65
fi

# --- PIN resolution ----------------------------------------------------------
if [ -z "${PIN}" ] && [ -r "${PIN_FILE}" ]; then
    # PIN file: первая строка, trim whitespace.
    PIN="$(head -n1 "${PIN_FILE}" | tr -d '[:space:]')"
fi

# --- Mode banner (честный, чтобы не было сюрпризов) --------------------------
if [ "${MODE}" = "check-only" ]; then
    echo "[mode=check-only] DNS + TLS only; остальные секции будут SKIP" >&2
fi
if [ "${MODE}" = "all" ] && [ -z "${PIN}" ]; then
    echo "[WARN] --all без PIN: WSS секции будут SKIP." >&2
    echo "       Используйте --pin 123456 или QUEST_PIN / QUEST_PIN_FILE." >&2
fi

# --- Build Python argv -------------------------------------------------------
PY_ARGS=(
    "--host"  "${HOST}"
    "--port"  "${PORT}"
    "--mode"  "${MODE}"
)

if [ -n "${PIN}" ]; then
    PY_ARGS+=("--pin" "${PIN}")
fi
if [ "${JSON_OUT}" -eq 1 ]; then
    PY_ARGS+=("--json")
fi
if [ "${NO_COLOR}" -eq 1 ] || [ ! -t 1 ]; then
    PY_ARGS+=("--no-color")
fi

# --- Run ---------------------------------------------------------------------
exec python3 "${LIB_PY}" "${PY_ARGS[@]}"
