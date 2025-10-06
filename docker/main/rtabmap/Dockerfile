FROM introlab3it/rtabmap_ros:kilted-latest

# Установка плагинов для сжатия изображений
RUN apt-get update && \
    apt-get install -y \
        ros-kilted-image-transport-plugins \
        ros-kilted-compressed-image-transport \
        ros-kilted-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем rmw_zenoh
RUN apt-get update && \
    apt-get install -y \
    ros-kilted-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5 \
    ZENOH_ROUTER_CHECK_ATTEMPTS=10 \
    ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/multicast/enabled=false'