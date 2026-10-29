#!/bin/bash
# ros1.sh '<команда>' [имя-контейнера] : одноразовый контейнер образа voice-assistant с тем же ROS/zenoh-окружением,
# что у сервиса (compose узла Sound Pi), но БЕЗ запуска ros2 CLI внутри сервисных контейнеров (демон ros2 съедает
# память, ловили OOM). --rm: после команды контейнера нет. Каталог ~/vp26 хоста виден внутри как /vp26.
# Ключей (YANDEX/MINIMAX) этот контейнер НЕ получает: они только в сервисном voice-assistant.
IMG=10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-dev
exec docker run --rm ${2:+--name $2} --network host --ipc host \
  -e ROBOT_ID=RBXU100001 -e ZENOH_LISTEN_IP=10.1.1.12 -e RMW_IMPLEMENTATION=rmw_zenoh_cpp -e ROS_DOMAIN_ID=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ZENOH_ROUTER_CHECK_ATTEMPTS=10 \
  -e ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5 \
  -e ZENOH_CONFIG=/config/shared/zenoh_session_config.json5 -e PYTHONUNBUFFERED=1 \
  -v /home/ros2/rob_box_project/docker/sound/config:/config:ro \
  -v /home/ros2/rob_box_project/docker/common/config:/common_config:ro \
  -v /home/ros2/rob_box_project/docker/common/scripts:/ros_scripts:ro \
  -v /home/ros2/vp26:/vp26 \
  "$IMG" /ros_scripts/ros_with_namespace.sh bash -c "$1"
