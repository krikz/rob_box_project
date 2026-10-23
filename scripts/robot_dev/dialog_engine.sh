#!/usr/bin/env bash
# Переключатель движка диалога (v1/v2) на Sound Pi.
#
#   scripts/robot_dev/dialog_engine.sh v1|v2   # поправить dialog_engine.yaml на роботе и перезапустить сервисы
#   scripts/robot_dev/dialog_engine.sh status  # только показать значение и строки dialog_node/dialogue_node из логов
#
# Каждый деплой возвращает значение из репо — после деплоя запускать заново.
# Порядок рестарта важен (issue #3795): supercollider → пауза → voice-assistant/avatar-supervisor/telegram-bot,
# иначе после рестарта voice-assistant появляется дубль ноды.
# Переменные: ROBOT (ros2@10.1.1.22), COMPOSE_DIR (/home/ros2/rob_box_project/docker/sound).
set -euo pipefail
export MSYS_NO_PATHCONV=1

ROBOT="${ROBOT:-ros2@10.1.1.22}"
COMPOSE_DIR="${COMPOSE_DIR:-/home/ros2/rob_box_project/docker/sound}"
CFG="$COMPOSE_DIR/config/voice_assistant/dialog_engine.yaml"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$ROBOT")

cmd="${1:-status}"

wait_healthy() {
  st=unknown
  for _ in $(seq 1 60); do
    st=$("${SSH[@]}" "docker inspect -f '{{.State.Health.Status}}' voice-assistant" 2>/dev/null || echo unknown)
    [ "$st" = healthy ] && { echo "✅ voice-assistant healthy"; return 0; }
    sleep 5
  done
  echo "❌ voice-assistant не стал healthy за 5 мин (последний статус: $st)" >&2
  return 1
}

show() {
  "${SSH[@]}" "grep dialog_engine: $CFG"
  "${SSH[@]}" "docker logs --since 2m voice-assistant 2>&1 | grep -E 'dialog_node v2|dialogue_node|Traceback|duplicate|already exists|ERROR' | head -15 || true"
  "${SSH[@]}" "docker logs --since 2m avatar-supervisor 2>&1 | grep -E 'dialog_engine|Traceback|ERROR' | head -5 || true"
  "${SSH[@]}" "docker logs --since 2m telegram-bot 2>&1 | grep -E 'dialog_engine|Traceback|ERROR' | head -5 || true"
  "${SSH[@]}" "docker ps --format '{{.Names}} {{.Status}}' | grep -E 'voice|super|telegram' || true"
}

case "$cmd" in
  v1|v2)
    [ "$cmd" = v1 ] && other=v2 || other=v1
    "${SSH[@]}" "sed -i 's/dialog_engine: \"$other\"/dialog_engine: \"$cmd\"/' $CFG && grep dialog_engine: $CFG"
    "${SSH[@]}" "docker restart supercollider >/dev/null"
    sleep 8
    "${SSH[@]}" "docker restart voice-assistant avatar-supervisor telegram-bot >/dev/null"
    wait_healthy
    show
    ;;
  status)
    show
    ;;
  *)
    sed -n '2,10p' "$0"; exit 2 ;;
esac
