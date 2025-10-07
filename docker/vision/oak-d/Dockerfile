# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:jazzy-latest

RUN apt-get update &&  apt-get install curl gnupg2 -y && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
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