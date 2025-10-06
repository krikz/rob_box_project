ARG ROS_DISTRO=kilted
ARG PREFIX=

FROM osrf/ros:${PREFIX}${ROS_DISTRO}-desktop

RUN apt update && apt install -y \
      ros-${ROS_DISTRO}-apriltag-ros \
      ros-${ROS_DISTRO}-v4l2-camera \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-image-proc && \
    apt clean &&  \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-apriltag-ros | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt

# Устанавливаем rmw_zenoh
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Копируем конфигурационные файлы Zenoh
COPY config/zenoh_session_config.json5 /config/zenoh_session_config.json5

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_ROUTER_CONFIG_URI=/config/zenoh_router_config.json5 \
    ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5 \
    ZENOH_ROUTER_CHECK_ATTEMPTS=10 \
    ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/multicast/enabled=false'

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_apriltag.sh /start_apriltag.sh
RUN chmod +x /start_apriltag.sh

CMD ["/start_apriltag.sh"]