ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-image-proc && \
    apt clean &&  \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-apriltag-ros | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt


# Устанавливаем rmw_zenoh для лучшей производительности и надёжности
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp 

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_apriltag.sh /start_apriltag.sh
RUN chmod +x /start_apriltag.sh

CMD ["/start_apriltag.sh"]