# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:jazzy-latest

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    mkdir -p /etc/apt/keyrings && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update

# Устанавливаем rmw_cyclonedds
RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    
# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]