# OAK-D Camera - использует специализированный базовый образ depthai
# Базовый образ luxonis/depthai-ros уже содержит все необходимое для OAK-D
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:depthai
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:depthai
FROM ${BASE_IMAGE}

# Все необходимые зависимости (rmw-zenoh, image-transport, opencv, depthai) 
# уже установлены в базовом образе 

# ВАЖНО: Launch файлы и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ./oak-d/config:/config/oak-d:ro (конфиги oak-d)
# - ./oak-d/launch:/oak-d/launch:ro (launch файлы)
# - ./oak-d/scripts:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_oak_d.sh
CMD ["/scripts/start_oak_d.sh"]
