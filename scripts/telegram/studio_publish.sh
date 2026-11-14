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
# set -a: переменные из файла секретов должны попасть в ОКРУЖЕНИЕ дочерних процессов.
# Без него python3 внутри скрипта не видит TG_STUDIO_BOT_TOKEN — ровно так и падал опрос.
set -a
. "$SECRETS"
set +a
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

PUBLISH_RC=0
# set -e убьёт скрипт на ненулевом коде до записи в PUBLISH_RC — ловим явно.
python3 scripts/telegram/channel_post.py --transport direct || PUBLISH_RC=$?

# Публикатор правит front matter очереди (posted_at, message_id). Без коммита
# в репозитории остаётся грязное дерево, и правка теряется при следующем pull.
#
# Раньше коммит проходил только если дерево было чистое КРОМЕ очереди. Но у
# студийного хоста в дереве всегда есть untracked-файлы студии
# (docs/channel/infographics/loc-drift.json и т.п.), поэтому условие
# "OTHER = 0" не выполнялось НИКОГДА: отметка posted никогда не попадала
# в git (разбор 11.11, очередь 900-post.md осталась untracked).
#
# Теперь коммитим ТОЛЬКО путь очереди — `git commit -- docs/channel/queue`
# не трогает чужие файлы даже если они есть в дереве. Отдельно проверяем,
# что в индексе нет постороннего: иначе коммит утащил бы чужую работу.
if [ "$PUBLISH_RC" = "0" ]; then
    DIRTY=$(git -C "$STUDIO_REPO" status --porcelain -- docs/channel/queue 2>/dev/null | wc -l)
    if [ "$DIRTY" != "0" ]; then
        STAGED_OTHER=$(git -C "$STUDIO_REPO" diff --cached --name-only 2>/dev/null | grep -v "^docs/channel/queue/" | wc -l)
        if [ "$STAGED_OTHER" != "0" ]; then
            echo "[studio-publish] в индексе чужое — коммит очереди пропущен, чтобы не утащить чужую работу"
        else
            git -C "$STUDIO_REPO" add -- docs/channel/queue
            if git -C "$STUDIO_REPO" commit -q -m "channel: опубликован пост" -- docs/channel/queue; then
                echo "[studio-publish] правка очереди закоммичена"
            else
                echo "[studio-publish] не смог закоммитить правку очереди"
            fi
        fi
    fi
fi

exit $PUBLISH_RC
