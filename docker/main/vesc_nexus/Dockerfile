# Dockerfile для VESC Nexus - ROS2 драйвер для управления VESC через CAN
# Базируется на ros2-zenoh образе для интеграции с Zenoh middleware
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, §3 п.1, §4.3
# вариант «б» — ЛОКАЛЬНАЯ builder-стадия, без общего builder-образа):
# builder-стадия ставит сборочный инструментарий (ros-dev-tools,
# rosidl-default-generators) и прогоняет rosdep+colcon build; runtime-стадия
# получает только /ws/install и рантайм-пакеты apt.
#
# ЧТО РЕАЛЬНО УХОДИТ ИЗ РАНТАЙМА этим швом:
#   - ros-dev-tools (мета-пакет: colcon-расширения, rosdep, vcstool, линтеры)
#   - ros-${ROS_DISTRO}-rosidl-default-generators (кодогенератор .msg → .c/.cpp/.py;
#     сгенерированный код уже лежит в /ws/install)
#   - test_depend-пакеты, которые rosdep тянет по package.xml обоих пакетов
#     (ament_lint_auto, ament_lint_common и их ament_cmake_*-линтеры)
#   - /ws/src и /ws/build (исходники и объектные файлы сборки)
#
# ЧТО НЕ УХОДИТ И НЕ МОЖЕТ УЙТИ (план §2.1): build-essential, gcc, g++, cmake,
# colcon, rosdep, git, vcstool запечены в ros:humble-ros-base (апстрим OSRF),
# от которого наследуется ghcr.io/krikz/rob_box_base:ros2-zenoh. Обе стадии
# используют один и тот же BASE_IMAGE, поэтому унаследованный тулчейн остаётся
# в рантайме. Образ НЕ «без компилятора» — шов убирает только то, что этот
# Dockerfile добавляет поверх базы.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
ARG VESC_NEXUS_SHA=unknown

# ============================================================
# Stage 1: builder — тулчейн + rosdep + colcon build
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG ROS_DISTRO
ARG VESC_NEXUS_SHA
ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер передаёт APT_PROXY этому сервису
# ("L-Build Main Pi Services.yml", build-ros2-control/build-lslidar и соседи),
# но этот Dockerfile его никогда не объявлял — build-arg молча игнорировался,
# и apt ходил напрямую в packages.ros.org. Ровно та же дыра, что чинил коммит
# dc7ed68a2 в пилоте led_matrix; multi-stage удваивает число apt-get update,
# поэтому прокси нужен в ОБЕИХ стадиях.
# Отличие от пилота: разбор host/port сделан через подстановку параметров, а не
# `echo | sed` — pipe в RUN даёт hadolint DL4006 (led_matrix получил два таких
# предупреждения), здесь их нет.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOSTPORT="${APT_PROXY#*://}"; \
        PROXY_HOSTPORT="${PROXY_HOSTPORT%%/*}"; \
        PROXY_HOST="${PROXY_HOSTPORT%%:*}"; \
        PROXY_PORT="${PROXY_HOSTPORT##*:}"; \
        if [ "$PROXY_PORT" = "$PROXY_HOST" ]; then PROXY_PORT=3142; fi; \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Сборочные зависимости. ros-dev-tools и rosidl-default-generators ставятся
# ТОЛЬКО здесь. Остальное (ros2_control stack, pluginlib) нужно и на сборке
# (заголовки в тех же .deb), и в рантайме — поэтому дублируется в Stage 2.
RUN apt-get update && apt-get install -y \
    # ROS 2 development tools (build-only)
    ros-dev-tools \
    # ROS 2 message generation: генератор — build-only, runtime — нужен обоим
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # ros2_control stack — заголовки для компиляции hardware_interface плагина
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# ============================================================
# ОПТИМИЗАЦИЯ КЭШИРОВАНИЯ: Копируем файлы от наименее к наиболее изменяемым
# ============================================================

# Инвалидация кэша при изменении субмодуля vesc_nexus
# Этот ARG используется для принудительной пересборки при обновлении субмодуля
RUN echo "Building with vesc_nexus SHA: ${VESC_NEXUS_SHA}"

# 1. Копируем package.xml (метаданные зависимостей) - изменяются редко
COPY src/vesc_nexus/src/vesc_msgs/package.xml /ws/src/vesc_msgs/
COPY src/vesc_nexus/src/vesc_nexus/package.xml /ws/src/vesc_nexus/

# 2. Установка ROS зависимостей через rosdep (кэшируется если package.xml не изменился)
# apt-get update ОБЯЗАТЕЛЕН перед rosdep: apt-блок выше заканчивается
# `rm -rf /var/lib/apt/lists/*`, а rosdep install внутри вызывает
# `apt-get install` — без списков он падает с «Unable to locate package»
# (реально поймано на katana, run 35617451547, build-ros2-control:
# E: Unable to locate package ros-humble-xacro). До шва этого не было
# видно: xacro ставился явным apt-блоком, и rosdep'у нечего было
# доустанавливать. Там, где rosdep стоит с `|| true`, эта же дыра молча
# превращала его в no-op.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "serial_driver joint_state_publisher joint_state_publisher_gui rviz2" || true

# 3. Копируем CMakeLists.txt - изменяются реже чем код
COPY src/vesc_nexus/src/vesc_msgs/CMakeLists.txt /ws/src/vesc_msgs/
COPY src/vesc_nexus/src/vesc_nexus/CMakeLists.txt /ws/src/vesc_nexus/

# 4. Копируем msg/srv определения (изменяются редко)
COPY src/vesc_nexus/src/vesc_msgs/msg /ws/src/vesc_msgs/msg

# 5. Только ТЕПЕРЬ копируем весь исходный код (изменяется часто)
COPY src/vesc_nexus/src/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc_nexus/src/vesc_nexus /ws/src/vesc_nexus

# 6. Сборка пакетов
#
# УБРАН --symlink-install (урок пилота led_matrix, распространённый здесь на
# ament_cmake и проверенный экспериментально в этом же базовом образе):
# с этим флагом colcon кладёт в /ws/install не файлы, а symlink'и наружу —
#   install/<pkg>/share/<pkg>/package.xml       -> /ws/src/<pkg>/package.xml
#   install/<pkg>/share/<pkg>/launch/*.py       -> /ws/src/<pkg>/launch/*.py
#   install/<pkg>/share/<pkg>/local_setup.bash  -> /ws/build/<pkg>/...
# Последнее критично: через шов идёт только /ws/install, и без /ws/build
# `source /ws/install/setup.bash` (ENTRYPOINT/CMD и start-скрипты) читает
# битый symlink. Без флага colcon кладёт настоящие копии.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select vesc_msgs vesc_nexus \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# ============================================================
# Stage 2: runtime — та же база, БЕЗ ros-dev-tools и rosidl-генераторов
# ============================================================
FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ARG APT_PROXY=""

# apt-cacher-ng — см. комментарий в builder-стадии. Нужен в ОБЕИХ стадиях.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOSTPORT="${APT_PROXY#*://}"; \
        PROXY_HOSTPORT="${PROXY_HOSTPORT%%/*}"; \
        PROXY_HOST="${PROXY_HOSTPORT%%:*}"; \
        PROXY_PORT="${PROXY_HOSTPORT##*:}"; \
        if [ "$PROXY_PORT" = "$PROXY_HOST" ]; then PROXY_PORT=3142; fi; \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Метаданные
LABEL org.opencontainers.image.title="VESC Nexus Driver"
LABEL org.opencontainers.image.description="Multi-CAN VESC driver with ros2_control support"
LABEL org.opencontainers.image.source="https://github.com/krikz/vesc_nexus"
LABEL org.opencontainers.image.version="1.0.0"

# Рантайм-зависимости. БЕЗ ros-dev-tools и rosidl-default-generators —
# см. шапку файла.
#
# rclcpp-lifecycle и std-msgs перечислены явно: раньше они попадали в образ
# неявно, как <depend>/<exec_depend> из package.xml через `rosdep install`
# (vesc_nexus: rclcpp, rclcpp_lifecycle, hardware_interface, pluginlib,
# vesc_msgs, controller_manager; vesc_msgs: std_msgs, rosidl_default_runtime).
# В рантайм-стадии rosdep не гоняется, поэтому неявное стало явным — иначе
# класс регрессии из плана §10: нода падает на роботе, а не в CI.
RUN apt-get update && apt-get install -y \
    # CAN-утилиты
    can-utils \
    iproute2 \
    net-tools \
    # rosidl: только runtime-часть (typesupport .so), генератор остался в builder
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # ros2_control stack
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-xacro \
    # exec-зависимости пакетов, ранее приходившие через rosdep
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# colcon workspace из builder-стадии — единственный продукт сборки, который
# обязан пересечь шов (план §2.2). /ws/src и /ws/build не копируются.
COPY --from=builder /ws/install /ws/install

WORKDIR /ws

# Добавляем source в bashrc для интерактивного использования
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Переменные окружения для ROS 2 и Zenoh
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5

# Healthcheck для проверки работоспособности ноды
HEALTHCHECK --interval=10s --timeout=5s --start-period=30s --retries=3 \
    CMD ros2 node list | grep -q vesc_can_driver || exit 1

# По умолчанию запускаем через скрипт (будет переопределено в docker-compose)
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch vesc_nexus vesc_nexus_node.launch.py"]
