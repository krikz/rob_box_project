#!/usr/bin/env bash
# record_scene.sh — снять одну сцену набора (ADR-0144 §2, §4). Запуск на ХОСТЕ Vision Pi.
#
#   bash record_scene.sh scenes/s01_owner_enter_leave.yaml
#   bash record_scene.sh --probe-anchor 10        # коридор подставки: поставить маску, выйти из кадра
#
# Что делает:
#   1. копирует scene_set/ в контейнер voice-assistant (/tmp/scene_set) —
#      единственный, где есть и audio_common_msgs, и rob_box_perception_msgs,
#      и compressed_depth_image_transport;
#   2. запускает там conductor.py: он пишет бэг (sqlite3) и по шагам
#      командует голосом робота, в конце кладёт scene.yaml рядом с бэгом;
#   3. копирует каталог сцены на хост в ~/scenes/<сцена>_<UTC>/ (НЕ /data, НЕ git)
#      и удаляет его из /tmp контейнера.
#
# Остановить досрочно — Ctrl+C: бэг закроется, scene.yaml НЕ будет записан
# (сцена без разметки в набор не идёт — удалить каталог руками).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CONTAINER="${CONTAINER:-voice-assistant}"
SCENES_DIR="${SCENES_DIR:-$HOME/scenes}"
REPO="${REPO:-$HOME/rob_box_project}"
ROS_ENV='source /opt/ros/humble/setup.bash && source /ws/install/setup.bash'

docker exec "$CONTAINER" rm -rf /tmp/scene_set
docker cp "$HERE" "$CONTAINER:/tmp/scene_set"

if [ "${1:-}" = "--probe-anchor" ]; then
    docker exec "$CONTAINER" bash -lc "$ROS_ENV && python3 /tmp/scene_set/conductor.py --script /tmp/scene_set/scenes/s06_mask_silent.yaml --probe-anchor-s ${2:-10}"
    exit 0
fi

SCRIPT="${1:?usage: record_scene.sh scenes/<сцена>.yaml}"
SCRIPT_IN="/tmp/scene_set/scenes/$(basename "$SCRIPT")"
COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
FACE_IMAGE="$(docker inspect -f '{{.Config.Image}}' vision-face 2>/dev/null || echo unknown)"
VOICE_IMAGE="$(docker inspect -f '{{.Config.Image}}' "$CONTAINER")"

# -t — чтобы Ctrl+C дошёл до дирижёра в контейнере и тот закрыл бэг.
TTY_FLAG="-i"
[ -t 0 ] && TTY_FLAG="-it"
LOG="$(mktemp)"
docker exec $TTY_FLAG "$CONTAINER" bash -lc "$ROS_ENV && python3 /tmp/scene_set/conductor.py \
    --script $SCRIPT_IN --out /tmp/scenes \
    --meta repo_commit=$COMMIT --meta face_image=$FACE_IMAGE --meta voice_image=$VOICE_IMAGE" | tee "$LOG"

SCENE_IN="$(tr -d '\r' < "$LOG" | sed -n 's/^SCENE_DIR=//p' | tail -1)"
rm -f "$LOG"
if [ -z "$SCENE_IN" ]; then
    echo "record_scene: дирижёр не дошёл до конца — scene.yaml нет, сцена не сохранена" >&2
    exit 1
fi
mkdir -p "$SCENES_DIR"
docker cp "$CONTAINER:$SCENE_IN" "$SCENES_DIR/"
docker exec "$CONTAINER" rm -rf "$SCENE_IN"
DEST="$SCENES_DIR/$(basename "$SCENE_IN")"
echo "record_scene: $DEST"
du -sh "$DEST"
grep -E 'message_count|name:' "$DEST/bag/metadata.yaml" | paste - - | head -20
