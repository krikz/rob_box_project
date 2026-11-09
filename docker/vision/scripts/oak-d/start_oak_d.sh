#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D с интегрированной детекцией AprilTag
# Это объединяет камеру и AprilTag детектор в одном контейнере
# для снижения нагрузки на Vision Pi
cd /config/oak-d/launch

# #4532: рядом с launch — сторож кадров. Если камеру переткнули и camera_node
# завис на мёртвом XLink (X_LINK_ERROR, процесс жив, кадров нет), сторож выходит,
# мы гасим launch и выходим с 1: `restart: unless-stopped` поднимает контейнер
# на свежем устройстве. Тот же сторож отвечает за healthcheck в compose.
ros2 launch oakd_with_apriltag.launch.py &
LAUNCH_PID=$!
python3 /scripts/oak_frame_watchdog.py run &
WATCHDOG_PID=$!

# docker stop шлёт SIGTERM нам (PID 1) — передаём детям, иначе ждали бы SIGKILL.
trap 'kill -TERM "$LAUNCH_PID" "$WATCHDOG_PID" 2>/dev/null; wait; exit 143' TERM INT

# Выходит первый из двух. Любой выход, включая 0 у launch, — повод перезапуска.
wait -n "$LAUNCH_PID" "$WATCHDOG_PID"
STATUS=$?
echo "[start_oak_d] launch или сторож кадров завершились (код $STATUS), перезапуск контейнера" >&2
kill -TERM "$LAUNCH_PID" "$WATCHDOG_PID" 2>/dev/null
wait
exit "$STATUS"
