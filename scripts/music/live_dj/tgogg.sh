#!/bin/bash
# usage: TG_CHAT_ID=<chat> bash tgogg.sh <wav on host> "<caption>"
# на Vision Pi; токен берётся из окружения контейнера telegram-bot. Моно не форсируем (без -ac 1).
# Отправка — tg_send_voice.py: повторы при сетевой ошибке, токен в вывод не попадает,
# exit != 0 если не отправилось.
set -o pipefail
W="$1"; C="$2"; B=$(basename "$W" .wav)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker cp "$W" telegram-bot:/tmp/"$B".wav || exit 1
docker exec telegram-bot ffmpeg -y -loglevel error -i /tmp/"$B".wav -c:a libopus -b:a 48k /tmp/"$B".ogg || exit 1
docker exec -i -e C="$C" -e B="$B" -e CHAT="${TG_CHAT_ID:?set TG_CHAT_ID}" telegram-bot python3 - < "$HERE/tg_send_voice.py"
