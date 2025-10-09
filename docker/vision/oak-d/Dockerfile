# OAK-D Camera - использует специализированный базовый образ depthai
# Базовый образ luxonis/depthai-ros уже содержит все необходимое для OAK-D
ARG BASE_IMAGE=rob_box_base:depthai
FROM ${BASE_IMAGE}

# Все необходимые зависимости (rmw-zenoh, image-transport, opencv, depthai) 
# уже установлены в базовом образе 

# Устанавливаем права на выполнение скрипта запуска
COPY scripts/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]
