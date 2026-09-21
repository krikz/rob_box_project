# Dockerfile для ROS2 Control Manager с VESC Hardware Interface
# Этот контейнер запускает controller_manager который загружает VescSystemHardwareInterface
#
# VESC Nexus работает как hardware_interface плагин внутри controller_manager,
# а НЕ как отдельная нода!
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, §3 п.2, §4.3
# вариант «б» — ЛОКАЛЬНАЯ builder-стадия). Общий builder-образ на пару
# vesc_nexus↔ros2_control (§4.3, отступление от варианта «б») здесь НЕ сделан:
# он требует нового CI-job'а и правки L-Build workflow, что выходит за границы
# этой карточки. Дублирующая компиляция vesc_msgs/vesc_nexus между двумя
# образами сохраняется.
#
# ЧТО РЕАЛЬНО УХОДИТ ИЗ РАНТАЙМА этим швом:
#   - ros-humble-rosidl-default-generators (кодогенератор .msg → .c/.cpp/.py)
#   - test_depend-пакеты, которые rosdep тянет по package.xml трёх пакетов
#     (ament_lint_auto, ament_lint_common + ament_cmake_*-линтеры)
#   - build-зависимости, которые rosdep ставит по <build_depend>/<buildtool_depend>
#   - /ws/src и /ws/build (исходники vesc_nexus + meshes/STL из rob_box_description
#     и объектные файлы сборки)
#
# ЧТО НЕ УХОДИТ (план §2.1): компилятор и colcon/rosdep наследуются от
# ghcr.io/krikz/rob_box_base:rtabmap и остаются в рантайме — обе стадии на
# одном BASE_IMAGE. Образ НЕ «без компилятора».

# ВАЖНО: Используем rtabmap базовый образ вместо минимального ros2-zenoh
# rtabmap базируется на полном ROS2 с всеми необходимыми библиотеками для компиляции
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
# Версия сборки для инвалидации кеша при изменении зависимостей
ARG BUILD_VERSION=2025-10-12-rtabmap
ARG VESC_NEXUS_SHA=unknown

# ============================================================
# Stage 1: builder — тулчейн + rosdep + colcon build
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG BUILD_VERSION
ARG VESC_NEXUS_SHA
ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер передаёт APT_PROXY этому сервису
# ("L-Build Main Pi Services.yml", build-ros2-control), но Dockerfile его
# никогда не объявлял — build-arg молча игнорировался, apt ходил напрямую в
# packages.ros.org. Та же дыра, что чинил коммит dc7ed68a2 в пилоте led_matrix;
# multi-stage удваивает число apt-get update, поэтому нужен в ОБЕИХ стадиях.
# Разбор host/port через подстановку параметров, без `echo | sed` — pipe в RUN
# даёт hadolint DL4006.
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
            # Pipeline-Depth 0 обязателен под qemu: с включённым
            # конвейером (по умолчанию 10 и у apt, и у apt-cacher-ng)
            # метод apt зависает намертво. Поймано 21.09.2026 на katana:
            # apt-get висел 68 минут с НУЛЁМ секунд CPU, два сокета к
            # прокси в CLOSE-WAIT с Recv-Q=1, процесс
            # qemu-binfmt/aarch64 /usr/lib/apt/methods/store — живой, но
            # ничего не делает.
            echo "Acquire::http::Pipeline-Depth \"0\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Принудительная инвалидация кеша для слоев установки
RUN echo "Build version: ${BUILD_VERSION}"

# Сборочные зависимости. rosidl-default-generators ставится ТОЛЬКО здесь.
# Остальное нужно и на сборке (заголовки), и в рантайме — дублируется в Stage 2.
RUN apt-get update && apt-get install -y \
    # ROS 2 message generation: генератор — build-only
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    # ROS2 Control (ros-base уже в базовом образе)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# Инвалидация кэша при изменении субмодуля vesc_nexus
# Этот ARG используется для принудительной пересборки при обновлении субмодуля
RUN echo "Building with vesc_nexus SHA: ${VESC_NEXUS_SHA}"

# Копируем vesc_nexus пакеты (vesc_msgs и vesc_nexus) из их src/
# vesc_nexus - это отдельный репозиторий, пакеты находятся в vesc_nexus/src/
COPY src/vesc_nexus/src/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc_nexus/src/vesc_nexus /ws/src/vesc_nexus

# Копируем rob_box_description (URDF с ros2_control блоком)
COPY src/rob_box_description /ws/src/rob_box_description

# Установка зависимостей через rosdep и сборка пакетов
# Явно задаём ROS_DISTRO для rosidl_generator_rs (Rust IDL)
ENV ROS_DISTRO=humble
#
# УБРАН --symlink-install (урок пилота led_matrix, распространённый здесь на
# ament_cmake и проверенный экспериментально в базовом ros2-zenoh образе):
# с этим флагом colcon кладёт в /ws/install не файлы, а symlink'и наружу —
#   install/<pkg>/share/<pkg>/package.xml      -> /ws/src/<pkg>/package.xml
#   install/<pkg>/share/<pkg>/urdf, meshes     -> /ws/src/rob_box_description/...
#   install/<pkg>/share/<pkg>/local_setup.bash -> /ws/build/<pkg>/...
# Через шов идёт только /ws/install: без /ws/src URDF/xacro у
# rob_box_description оказались бы битыми ссылками, а без /ws/build ломается
# сам `source /ws/install/setup.bash`. Без флага colcon кладёт настоящие копии.
# apt-get update ОБЯЗАТЕЛЕН перед rosdep: apt-блок выше заканчивается
# `rm -rf /var/lib/apt/lists/*`, а rosdep install внутри вызывает
# `apt-get install` — без списков он падает с «Unable to locate package»
# (реально поймано на katana, run 35617451547, build-ros2-control:
# E: Unable to locate package ros-humble-xacro). До шва этого не было
# видно: xacro ставился явным apt-блоком, и rosdep'у нечего было
# доустанавливать. Там, где rosdep стоит с `|| true`, эта же дыра молча
# превращала его в no-op.
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "rviz2 joint_state_publisher joint_state_publisher_gui serial_driver" && \
    colcon build \
    --packages-select vesc_msgs vesc_nexus rob_box_description \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# ============================================================
# Stage 2: runtime — та же база, БЕЗ rosidl-генераторов и build-зависимостей
# ============================================================
FROM ${BASE_IMAGE}

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
            # Pipeline-Depth 0 обязателен под qemu: с включённым
            # конвейером (по умолчанию 10 и у apt, и у apt-cacher-ng)
            # метод apt зависает намертво. Поймано 21.09.2026 на katana:
            # apt-get висел 68 минут с НУЛЁМ секунд CPU, два сокета к
            # прокси в CLOSE-WAIT с Recv-Q=1, процесс
            # qemu-binfmt/aarch64 /usr/lib/apt/methods/store — живой, но
            # ничего не делает.
            echo "Acquire::http::Pipeline-Depth \"0\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Метаданные
LABEL org.opencontainers.image.title="ROS2 Control Manager with VESC"
LABEL org.opencontainers.image.description="Controller Manager with VescSystemHardwareInterface for 4-wheel robot"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Рантайм-зависимости. БЕЗ ros-humble-rosidl-default-generators — см. шапку.
#
# rclcpp-lifecycle, std-msgs и robot-state-publisher перечислены явно: раньше
# они попадали в образ неявно, через `rosdep install` по package.xml
# (vesc_nexus: rclcpp, rclcpp_lifecycle, hardware_interface, pluginlib,
# vesc_msgs, controller_manager; vesc_msgs: std_msgs, rosidl_default_runtime;
# rob_box_description: robot_state_publisher, xacro — остальные exec_depend
# гасятся --skip-keys). В рантайм-стадии rosdep не гоняется, поэтому неявное
# стало явным — иначе класс регрессии из плана §10: падение на роботе, не в CI.
RUN apt-get update && apt-get install -y \
    # CAN утилиты
    can-utils \
    iproute2 \
    net-tools \
    # rosidl: только runtime-часть (typesupport .so)
    ros-humble-rosidl-default-runtime \
    # ROS2 Control (ros-base уже в базовом образе)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    # Контроллеры
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    # Утилиты
    ros-humble-xacro \
    ros-humble-ros2controlcli \
    # exec-зависимости пакетов, ранее приходившие через rosdep
    ros-humble-rclcpp-lifecycle \
    ros-humble-std-msgs \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# colcon workspace из builder-стадии — единственный продукт сборки, который
# обязан пересечь шов (план §2.2). /ws/src и /ws/build не копируются.
COPY --from=builder /ws/install /ws/install

WORKDIR /ws

# Source workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# ROS2 и Zenoh environment variables
# ROS_DISTRO дублирует builder-стадию: базовый образ rtabmap его не задаёт,
# а одностадийная версия этого файла выставляла его в итоговом образе.
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5

# Healthcheck - проверяем что controller_manager запущен
HEALTHCHECK --interval=10s --timeout=5s --start-period=30s --retries=3 \
    CMD ros2 control list_controllers | grep -q diff_drive_controller || exit 1

# Команда запуска будет переопределена в docker-compose через startup script
CMD ["bash", "-c", "echo 'Use docker-compose to start this container'"]
