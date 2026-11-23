#!/usr/bin/env bash
# Горячая отладка на роботе: залить Python-пакеты из рабочей копии в контейнер, перезапустить, откатить.
#
# Экспериментальный код живёт в слое контейнера до его пересоздания. Синхронизация с репо — только через PR,
# сборку и деплой; `revert` возвращает контейнер к образу (пересоздание через docker compose).
#
#   scripts/robot_dev/hotpatch.sh push   [SRC_ROOT]   # пакеты из SRC_ROOT (по умолчанию корень репо) → контейнер, рестарт
#   scripts/robot_dev/hotpatch.sh status              # что залито (метка /ws/HOTPATCH) и когда стартовал контейнер
#   scripts/robot_dev/hotpatch.sh revert              # пересоздать контейнер из образа — горячий код исчезает
#   scripts/robot_dev/hotpatch.sh logs   [SINCE]      # хвост логов контейнера (SINCE — docker logs --since, по умолчанию 5m)
#
# Хост — Sound Pi (ADR-0159): voice-assistant, supercollider и avatar-supervisor живут там, не на Vision Pi.
# Переменные: ROBOT (ros2@10.1.1.22), CONTAINER (voice-assistant), PACKAGES ("rob_box_music rob_box_mcp_tools"),
# NO_RESTART=1 — только залить, COMPOSE_DIR (/home/ros2/rob_box_project/docker/sound),
# CHUANGONG_DIRTY_FLAG (/var/lib/chuangong/dirty) — флаг dirty на хосте для chuangong (ADR-0168 §dirty).
set -euo pipefail
export MSYS_NO_PATHCONV=1

ROBOT="${ROBOT:-ros2@10.1.1.22}"
CONTAINER="${CONTAINER:-voice-assistant}"
PACKAGES="${PACKAGES:-rob_box_music rob_box_mcp_tools}"
COMPOSE_DIR="${COMPOSE_DIR:-/home/ros2/rob_box_project/docker/sound}"
# Путь, который читает chuangong.agent.Config.dirty_flag (host/chuangong/.../agent.py).
CHUANGONG_DIRTY_FLAG="${CHUANGONG_DIRTY_FLAG:-/var/lib/chuangong/dirty}"
PY_SITE="lib/python3.10/site-packages"
# Тест может заранее задать SSH=(...) — тогда реальный ssh не вызывается.
if ! declare -p SSH >/dev/null 2>&1; then
  SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$ROBOT")
fi

wait_healthy() {
  for _ in $(seq 1 60); do
    st=$("${SSH[@]}" "docker inspect -f '{{.State.Health.Status}}' $CONTAINER" 2>/dev/null || echo unknown)
    [ "$st" = healthy ] && { echo "✅ $CONTAINER healthy"; return 0; }
    sleep 5
  done
  echo "❌ $CONTAINER не стал healthy за 5 мин (последний статус: $st)" >&2
  return 1
}

# Флаг на хосте робота (не в контейнере): chuangong probe_flags читает state_dir/dirty.
set_chuangong_dirty_flag() {
  local flag="${CHUANGONG_DIRTY_FLAG:-/var/lib/chuangong/dirty}"
  "${SSH[@]}" "mkdir -p \"\$(dirname -- $(printf '%q' "$flag"))\" && printf '%s\n' hotpatch > $(printf '%q' "$flag")"
  echo "chuangong dirty: $flag"
}

clear_chuangong_dirty_flag() {
  local flag="${CHUANGONG_DIRTY_FLAG:-/var/lib/chuangong/dirty}"
  "${SSH[@]}" "rm -f -- $(printf '%q' "$flag")"
  echo "chuangong dirty снят: $flag"
}

hotpatch_main() {
  local cmd="${1:-status}"
  shift || true

  case "$cmd" in
    push)
      local root sha branch dirty pkg dest stamp
      root="${1:-$(git rev-parse --show-toplevel)}"
      sha=$(git -C "$root" rev-parse --short HEAD)
      branch=$(git -C "$root" rev-parse --abbrev-ref HEAD)
      dirty=$(git -C "$root" status --porcelain -- src | wc -l | tr -d ' ')
      for pkg in $PACKAGES; do
        [ -d "$root/src/$pkg/$pkg" ] || { echo "нет пакета $root/src/$pkg/$pkg" >&2; exit 2; }
        dest="/ws/install/$pkg/$PY_SITE"
        "${SSH[@]}" "docker exec $CONTAINER test -d $dest" || { echo "в $CONTAINER нет $dest" >&2; exit 2; }
        tar -C "$root/src/$pkg" --exclude=__pycache__ --exclude='*.pyc' -cf - "$pkg" \
          | "${SSH[@]}" "docker exec -i $CONTAINER sh -c 'rm -rf $dest/$pkg.hotpatch_new && mkdir $dest/$pkg.hotpatch_new && tar -C $dest/$pkg.hotpatch_new -xf - && rm -rf $dest/$pkg && mv $dest/$pkg.hotpatch_new/$pkg $dest/$pkg && rmdir $dest/$pkg.hotpatch_new'"
        echo "→ $pkg залит в $CONTAINER:$dest"
      done
      stamp="$(date -u +%FT%TZ) $branch@$sha dirty_src_files=$dirty packages=[$PACKAGES]"
      "${SSH[@]}" "docker exec $CONTAINER sh -c 'echo \"$stamp\" >> /ws/HOTPATCH'"
      echo "метка: $stamp"
      set_chuangong_dirty_flag
      if [ "${NO_RESTART:-0}" != 1 ]; then
        "${SSH[@]}" "docker restart $CONTAINER" >/dev/null
        wait_healthy
      fi
      ;;
    status)
      "${SSH[@]}" "docker inspect -f '{{.Config.Image}} started={{.State.StartedAt}} health={{.State.Health.Status}}' $CONTAINER; docker exec $CONTAINER sh -c 'cat /ws/HOTPATCH 2>/dev/null || echo \"горячих заливок нет (чистый образ)\"'"
      ;;
    revert)
      # Имя compose-сервиса != имени контейнера (avatar-supervisor → supervisor): берём из метки compose.
      local svc
      svc=$("${SSH[@]}" "docker inspect -f '{{index .Config.Labels \"com.docker.compose.service\"}}' $CONTAINER" 2>/dev/null || true)
      svc="${svc:-$CONTAINER}"
      # #3795: после рестарта voice-assistant без перезапуска supercollider появляется дубль ноды.
      if [ "$CONTAINER" = voice-assistant ]; then
        "${SSH[@]}" "docker restart supercollider >/dev/null"
        sleep 8
      fi
      "${SSH[@]}" "cd $COMPOSE_DIR && docker compose up -d --force-recreate --no-deps $svc"
      wait_healthy
      "${SSH[@]}" "docker exec $CONTAINER sh -c 'test -f /ws/HOTPATCH && echo ❌ метка осталась || echo ✅ контейнер из образа, горячего кода нет'"
      # После отката к образу hotpatch не должен оставлять dirty.
      clear_chuangong_dirty_flag
      ;;
    logs)
      "${SSH[@]}" "docker logs $CONTAINER --since ${1:-5m} 2>&1 | tail -n ${TAIL:-200}"
      ;;
    *)
      sed -n '2,15p' "$0"; exit 2 ;;
  esac
}

# Подмена SSH в тестах: source + HOTPATCH_LIB_ONLY=1 — только функции, без main.
if [ "${HOTPATCH_LIB_ONLY:-0}" != 1 ] && [ "${BASH_SOURCE[0]-}" = "${0:-}" ]; then
  hotpatch_main "$@"
fi
