#!/bin/bash
# Опрос апдейтов студийного бота: кнопки Шифу и его текстовые сообщения.
#
# Запускается по cron (no-agent): бот студии (@robboxcontentbot) читается ТОЛЬКО здесь.
# Hermes этот бот не обслуживает: его адаптер знает только свои префиксы (mp:, cl:, ea:),
# а префикс studio: он проглотил бы молча.
#
# Секреты — из docker/studio/.env.secrets (в git не попадает).
# Переменные: STUDIO_REPO (корень репо), STUDIO_TIMEOUT (таймаут long-poll, с).
set -euo pipefail

STUDIO_REPO="${STUDIO_REPO:-/home/builder/hermes-share/rob_box_project}"
SECRETS="$STUDIO_REPO/docker/studio/.env.secrets"
TIMEOUT="${STUDIO_TIMEOUT:-25}"

if [ ! -f "$SECRETS" ]; then
    echo "[studio-poll] FATAL: нет файла секретов: $SECRETS" >&2
    exit 1
fi

set +e
# shellcheck disable=SC1090
. "$SECRETS"
set -e

if [ -z "${TG_STUDIO_BOT_TOKEN:-}" ]; then
    echo "[studio-poll] FATAL: TG_STUDIO_BOT_TOKEN пуст в $SECRETS" >&2
    exit 1
fi

cd "$STUDIO_REPO"

# flock: два опроса одного бота одновременно — Telegram вернёт 409 и апдейты потеряются.
LOCK="/tmp/studio-poll.lock"
exec 9>"$LOCK"
if ! flock -n 9; then
    echo "[studio-poll] другой опрос уже идёт, пропускаю"
    exit 0
fi

python3 scripts/telegram/channel_studio.py --state "$STUDIO_REPO/docker/studio/state.json" \
    poll --timeout "$TIMEOUT"
