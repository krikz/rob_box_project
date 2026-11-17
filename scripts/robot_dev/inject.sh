#!/usr/bin/env bash
# Инъекция реплики в диалог v2 на Sound Pi (топик /voice/stt/utterance), без микрофона.
#
#   scripts/robot_dev/inject.sh "<фраза>" [SOURCE] [CHAT_ID]
#
# SOURCE — respeaker (по умолчанию) или telegram. CHAT_ID нужен для telegram; по умолчанию фиктивный 1,
# чтобы ответ не ушёл живому человеку.
# Фраза едет на робота ФАЙЛОМ через stdin: кириллица в аргументах ssh/plink с Windows портится.
# timeout 15 обязателен: ros2 topic pub в топик без подписчика висит вечно.
# Печатает время отправки (UTC) — по нему потом мерить задержку по логам.
# Переменные: ROBOT (ros2@10.1.1.22), CONTAINER (voice-assistant).
set -euo pipefail
export MSYS_NO_PATHCONV=1

[ $# -ge 1 ] || { sed -n '2,11p' "$0"; exit 2; }
PHRASE="$1"
SOURCE="${2:-respeaker}"
CHAT_ID="${3:-1}"
ROBOT="${ROBOT:-ros2@10.1.1.22}"
CONTAINER="${CONTAINER:-voice-assistant}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$ROBOT")

printf '%s' "$PHRASE" | "${SSH[@]}" 'cat > /tmp/inject_phrase.txt'

extra=""
[ "$SOURCE" = telegram ] && extra=", chat_id: $CHAT_ID"
echo "отправлено: $(date -u +%FT%T.%3NZ) source=$SOURCE"
"${SSH[@]}" "T=\"\$(cat /tmp/inject_phrase.txt)\"; timeout 15 docker exec -e T=\"\$T\" $CONTAINER bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 topic pub --once /voice/stt/utterance rob_box_dialog_msgs/msg/Utterance \"{text: \\\"\$T\\\", source: $SOURCE, provider: yandex$extra}\"' >/dev/null 2>&1" \
  && echo "✅ опубликовано" || { echo "❌ pub не прошёл (таймаут 15 с или нет подписчика)" >&2; exit 1; }
