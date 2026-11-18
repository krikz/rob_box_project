# Nav2 Navigation Stack для Main Pi
# Автономная навигация с планированием пути и избежанием препятствий
# issue #1497 / t_31c93faa: на lyrical-ветке base-тег суффиксируется -lyrical-latest,
# чтобы docker pull не возвращал humble-base из общего GHCR-пакета
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-lyrical-latest
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
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    # Costmaps (локальная и глобальная)
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    # Планировщик пути
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
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
    # Утилиты
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-msgs \
    # Behavior Tree плагины (в lyrical пакет переименован из behaviortree-cpp-v3)
    ros-${ROS_DISTRO}-behaviortree-cpp \
    python3-dev \
    python3-yaml \
    # Дополнительные зависимости
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Nav2 1.1.20 pin from develop does not compile against lyrical Nav2 1.5.2
# (packages.ros.org resolute, 2026-11-18). The humble patches stay in-tree;
# they are not applied on this distro. Porting them is a separate task.
# #4475: узел дерева WouldSpinOnStall. Угол считает rob_box_teleop.turn_in_place
# (том compose, PYTHONPATH в start_nav2_direct.sh) — тот файл, что проверяет pytest.
# doTransform<PoseStamped> в tf2_geometry_msgs.hpp зовёт KDL. Без -lorocos-kdl
# dlopen падает на KDL::Rotation::Quaternion, хотя getTimestamp уже свой.
COPY would_spin_on_stall.cpp turn_in_place_py.hpp /tmp/turn-bt/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /tmp/turn-bt && \
    g++ -std=c++17 -fPIC -shared -DBT_PLUGIN_EXPORT \
      -I/opt/ros/${ROS_DISTRO}/include \
      \$(for d in /opt/ros/${ROS_DISTRO}/include/*/; do printf ' -I%s' \"\$d\"; done) \
      -I/tmp/turn-bt -I/usr/include/python3.14 \
      -o /opt/ros/${ROS_DISTRO}/lib/librob_box_would_spin_on_stall_bt_node.so \
      would_spin_on_stall.cpp \
      -L/opt/ros/${ROS_DISTRO}/lib -Wl,-rpath,/opt/ros/${ROS_DISTRO}/lib \
      -Wl,--no-as-needed \
      -lbehaviortree_cpp -lrclcpp -ltf2 -ltf2_ros -lorocos-kdl -lpython3.14 \
      -lnav_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp \
      -lgeometry_msgs__rosidl_typesupport_cpp -ltf2_msgs__rosidl_typesupport_cpp" \
    && rm -rf /tmp/turn-bt
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
