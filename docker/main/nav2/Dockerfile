# Nav2 Navigation Stack для Main Pi
# Автономная навигация с планированием пути и избежанием препятствий
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest
FROM ${BASE_IMAGE}

LABEL org.opencontainers.image.title="Nav2 Navigation Stack"
LABEL org.opencontainers.image.description="ROS2 Nav2 для автономной навигации rob_box"
LABEL org.opencontainers.image.authors="krikz"

# APT через apt-cacher-ng на katana (APT_PROXY передаёт gen_build_matrix.py).
# Базовый образ удаляет свой 02proxy, поэтому без этого блока apt ходил
# напрямую, а прямой путь с katana ненадёжен: GPG error (run 34387765562),
# Hash Sum mismatch у .deb того же размера (run 36421581380, nav2/voice-base).
# Шаблон и причина Pipeline-Depth 0 — docker/base/Dockerfile.ros2-zenoh.
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            printf '%s\n' "Acquire::http::Proxy \"$APT_PROXY\";" 'Acquire::http::Timeout "10";' \
                'Acquire::Retries "3";' 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# Установка Nav2 пакетов
RUN apt-get update && apt-get install -y \
    # Nav2 Core
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    # Costmaps (локальная и глобальная)
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    # Планировщик пути
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-smac-planner \
    # Контроллер (следование по траектории)
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-dwb-controller \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    # Поведения (recovery behaviors - переименован из recoveries в behaviors)
    ros-${ROS_DISTRO}-nav2-behaviors \
    # Локализация
    ros-${ROS_DISTRO}-nav2-amcl \
    # Map Server
    ros-${ROS_DISTRO}-nav2-map-server \
    # Waypoint следование
    ros-${ROS_DISTRO}-nav2-waypoint-follower \
    # Текущий depth image -> LaserScan для obstacle avoidance перед камерой
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    # Утилиты
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-msgs \
    # Behavior Tree плагины
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    # Дополнительные зависимости
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Создание рабочих директорий
RUN mkdir -p /maps /config/nav2

WORKDIR /workspace

# Health check - проверка Nav2 lifecycle узлов
HEALTHCHECK --interval=15s --timeout=10s --start-period=60s --retries=3 \
    CMD ros2 node list | grep -q "bt_navigator\|controller_server\|planner_server" || exit 1

# Точка входа - ROS2 setup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
