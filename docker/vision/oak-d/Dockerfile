# Используем базовый образ depthai-ros
FROM luxonis/depthai-ros:kilted-latest

# Устанавливаем rmw_zenoh
RUN apt-get update && \
    apt-get install -y \
    ros-kilted-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Копируем конфигурационные файлы Zenoh
#COPY config/zenoh_session_config.json5 /config/zenoh_session_config.json5

# Экспортируем переменные окружения по умолчанию
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5 \
    ZENOH_ROUTER_CHECK_ATTEMPTS=10 \
    ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/multicast/enabled=false'

# Устанавливаем права на выполнение скрипта запуска
COPY config/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]