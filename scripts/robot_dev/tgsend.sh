#!/usr/bin/env bash
# Отправить текстовый файл в Telegram через контейнер telegram-bot на Sound Pi (с Windows/Git Bash).
#
#   scripts/robot_dev/tgsend.sh <файл.txt>
#
# Деплой пересоздаёт telegram-bot, поэтому tgsend.py копируется в контейнер при каждом запуске.
# Токен остаётся в окружении контейнера и сюда не попадает. TG_CHAT_ID — другой получатель.
# Переменные: ROBOT (ros2@10.1.1.22).
set -euo pipefail
export MSYS_NO_PATHCONV=1

[ $# -eq 1 ] && [ -f "$1" ] || { sed -n '2,6p' "$0"; exit 2; }
ROBOT="${ROBOT:-ros2@10.1.1.22}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$ROBOT")

scp -q -o BatchMode=yes "$HERE/tgsend.py" "$1" "$ROBOT:/tmp/"
"${SSH[@]}" "docker cp /tmp/tgsend.py telegram-bot:/tmp/tgsend.py && docker exec -i -e TG_CHAT_ID=\"${TG_CHAT_ID:-495039871}\" telegram-bot python3 /tmp/tgsend.py < /tmp/$(basename "$1")"
