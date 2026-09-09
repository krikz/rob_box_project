# RTAB-Map SLAM - использует специализированный базовый образ rtabmap
# Базовый образ уже содержит rtabmap, image-transport и rmw-zenoh-cpp
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:rtabmap
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
FROM ${BASE_IMAGE}

# Все необходимые зависимости уже установлены в базовом образе

# ВАЖНО: Конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ./rtabmap/config:/config/rtabmap:ro (конфиг rtabmap)

# CMD будет задан в docker-compose.yaml

