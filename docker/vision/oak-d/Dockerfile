# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:humble-latest

# Обновляем существующий ключ ROS 2 в базовом образе
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --batch --yes --no-tty --dearmor -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Устанавливаем rmw_cyclonedds
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    
# Экспортируем переменные окружения по умолчанию
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]