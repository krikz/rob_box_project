# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:jazzy-latest

# Добавить перед RUN apt-get update
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
# Или используйте официальный способ из документации ROS 2
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Устанавливаем rmw_zenoh
RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5 \
    ZENOH_ROUTER_CHECK_ATTEMPTS=10 \
    ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/multicast/enabled=false'

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]