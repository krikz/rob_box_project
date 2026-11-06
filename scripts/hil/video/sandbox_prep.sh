#!/usr/bin/env bash
# Подготовка песочницы к видео из HIL (docs/runbooks/hil-video.md): отдельная копия частей репо,
# Fuel-мир Depot в кэше, правка копии compose. Боевой чекаут /home/ros2/rob_box_hil не трогается.
#
# Из корня репо, ключ в authorized_keys песочницы (пароли не используются):
#   scripts/hil/video/sandbox_prep.sh
# Переменные: HIL_SANDBOX_HOST (ros2@10.1.1.250), VIDEO_DIR (/home/ros2/rob_box_hilvideo),
#   DEPOT_VERSION (6; лицензия CC-BY 4.0, автор Open Robotics).
set -euo pipefail
export MSYS_NO_PATHCONV=1

HOST="${HIL_SANDBOX_HOST:-ros2@10.1.1.250}"
DIR="${VIDEO_DIR:-/home/ros2/rob_box_hilvideo}"
VER="${DEPOT_VERSION:-6}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10)

cd "$ROOT"
"${SSH[@]}" "$HOST" "mkdir -p '$DIR/out' '$DIR/fuel' && chmod 777 '$DIR/out'" </dev/null
# 1. части репо, нужные docker/sim/compose.yaml (extends ../main/docker-compose.yaml, тома конфигов и миров)
tar cf - docker/sim docker/main/config docker/main/docker-compose.yaml docker/common src/rob_box_sim/worlds \
    | "${SSH[@]}" "$HOST" "tar xf - -C '$DIR'"
# 2. наши скрипты
tar cf - -C scripts/hil/video . | "${SSH[@]}" "$HOST" "tar xf - -C '$DIR'"
# 3. CRLF: на Windows git отдаёт .sh/.py с \r, и ros_with_namespace.sh падает «/bin/bash^M: bad interpreter» (код 127)
"${SSH[@]}" "$HOST" "cd '$DIR' && find . -path ./fuel -prune -o -type f \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.json5' -o -name '*.sdf' \) -exec sed -i 's/\r\$//' {} +" </dev/null
# 4. Depot из Fuel (82 МБ), один раз
"${SSH[@]}" "$HOST" "test -f '$DIR/fuel/Depot/model.sdf' || (mkdir -p '$DIR/fuel/Depot' && cd '$DIR/fuel/Depot' \
    && curl -fsSL -m 600 -o d.zip 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot/$VER/Depot.zip' \
    && unzip -oq d.zip && rm d.zip)" </dev/null
# 5. Depot без крыши/вентиляторов/окон и без collision; в контейнер каталог идёт жёсткими ссылками (симлинки на хост не видны)
"${SSH[@]}" "$HOST" "cd '$DIR' && python3 trim_depot.py '$DIR/fuel' && python3 patch_compose.py '$DIR'" </dev/null
echo "готово: $DIR; дальше HIL_SANDBOX_REPO=$DIR HIL_MAPS_DIR=<свежий путь> scripts/hil/hil.sh on"
