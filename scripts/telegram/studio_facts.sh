#!/bin/bash
# Сессия анализа студии: собрать факты и отдать их агенту.
#
# Cron с агентом: скрипт печатает факты (stdout), агент по ним пишет идеи и шлёт их Шифу
# кнопками (scripts/telegram/channel_studio.py ideas).
#
# Факты собирает код, текст пишет модель, проверяет код — по ADR-0148.
# Если фактов нет или все невидимые — сессия молчит, лишних сообщений Шифу не приходит.
#
# Секреты — из docker/studio/.env.secrets (в git не попадает).
set -euo pipefail

STUDIO_REPO="${STUDIO_REPO:-/home/builder/hermes-share/rob_box_project}"
SECRETS="$STUDIO_REPO/docker/studio/.env.secrets"

if [ ! -f "$SECRETS" ]; then
    echo "[studio-facts] FATAL: нет файла секретов: $SECRETS" >&2
    exit 1
fi

set +e
# shellcheck disable=SC1090
. "$SECRETS"
set -e

cd "$STUDIO_REPO"

# Метка прошлой сессии обновится только после успешной отправки идей: иначе пропавшая
# сессия потеряла бы факты. Поэтому --no-save, а метку двигает сам агент после отправки.
python3 scripts/telegram/studio_facts.py --no-save --state "$STUDIO_REPO/docker/studio/state.json"
