# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:jazzy-latest

# Обновляем ключ GPG и устанавливаем rmw_cyclonedds для Jazzy
RUN apt-get update && \
    # Устанавливаем curl, если его нет, и обновляем ключ GPG
    apt-get install -y curl gnupg2 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    # Повторяем apt-get update, чтобы убрать ошибку с подписью
    apt-get update && \
    # Устанавливаем нужный пакет для Jazzy
    apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]