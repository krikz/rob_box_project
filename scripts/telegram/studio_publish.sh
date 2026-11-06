#!/bin/bash
# Публикация поста в канал @robbox_project по слоту (вторник, четверг, суббота, 11:00 GMT+1).
#
# Cron (no-agent): правило 48 ч и время слота проверяет channel_post.py, модель тут не участвует.
# Публикует только approved-пост, у которого наступил not_before.
#
# Секреты — из docker/studio/.env.secrets (в git не попадает).
set -euo pipefail

STUDIO_REPO="${STUDIO_REPO:-/home/builder/hermes-share/rob_box_project}"
SECRETS="$STUDIO_REPO/docker/studio/.env.secrets"

if [ ! -f "$SECRETS" ]; then
    echo "[studio-publish] FATAL: нет файла секретов: $SECRETS" >&2
    exit 1
fi

set +e
# shellcheck disable=SC1090
. "$SECRETS"
set -e

if [ -z "${TG_STUDIO_BOT_TOKEN:-}" ]; then
    echo "[studio-publish] FATAL: TG_STUDIO_BOT_TOKEN пуст в $SECRETS" >&2
    exit 1
fi

cd "$STUDIO_REPO"

# Публикация идёт в канал — два процесса одновременно опубликуют два поста.
LOCK="/tmp/studio-publish.lock"
exec 9>"$LOCK"
if ! flock -n 9; then
    echo "[studio-publish] публикация уже идёт, пропускаю"
    exit 0
fi

python3 scripts/telegram/channel_post.py --transport direct
