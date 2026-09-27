# LSLIDAR N10 Driver для ROS 2 Humble
# Использует базовый образ PCL с предустановленными зависимостями для облаков точек
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:pcl
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, §3 п.6, этап 5,
# §4.3 вариант «б» — ЛОКАЛЬНАЯ builder-стадия).
#
# ЧТО РЕАЛЬНО УХОДИТ ИЗ РАНТАЙМА этим швом:
#   - ros-humble-rosidl-default-generators (кодогенератор .msg → .c/.cpp/.py)
#   - ros-humble-ament-cmake (build-система)
#   - libboost-dev, libboost-thread-dev (заголовки Boost; в рантайме нужна
#     только разделяемая библиотека, см. libboost-thread1.74.0 ниже)
#   - /ws/src с клоном github.com/Lslidar/lslidar_ros2_driver и /ws/build
#
# Плюс изоляция сетевой зависимости: `git clone` с github.com/Lslidar живёт
# только в builder-стадии — это и есть заявленный смысл этапа 5 плана.
#
# ЧТО НЕ УХОДИТ (план §2.1): компилятор, cmake, colcon, git наследуются от
# ghcr.io/krikz/rob_box_base:pcl (через ros:humble-ros-base) и остаются в
# рантайме — обе стадии на одном BASE_IMAGE. Образ НЕ «без компилятора».
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl

# ============================================================
# Stage 1: builder — тулчейн, git clone драйвера, colcon build
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер передаёт APT_PROXY этому сервису
# ("L-Build Main Pi Services.yml", build-lslidar), но Dockerfile его никогда
# не объявлял — build-arg молча игнорировался, apt ходил напрямую в
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

# Все необходимые рантайм-пакеты (rmw-zenoh-cpp, PCL, diagnostic-updater,
# libpcap, boost) уже установлены в базовом образе. Здесь — только то, что
# нужно дополнительно ДЛЯ СБОРКИ драйвера.
RUN apt-get update && apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ament-cmake \
    libboost-dev \
    libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/*

# Создаём рабочее пространство
WORKDIR /ws/src

# Клонируем LSLIDAR N10 драйвер (ветка M10P/N10P для N10)
RUN git clone --depth 1 -b M10P/N10P https://github.com/Lslidar/lslidar_ros2_driver.git

# Собираем все пакеты из репозитория
# ВАЖНО: конфиги и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ./lslidar/config:/config/lslidar:ro (конфиги lslidar)
# - ./lslidar/scripts:/scripts:ro (скрипт запуска)
#
# УБРАН --symlink-install (урок пилота led_matrix, распространённый здесь на
# ament_cmake и проверенный экспериментально в базовом ros2-zenoh образе):
# с этим флагом colcon кладёт в /ws/install не файлы, а symlink'и наружу —
#   install/<pkg>/share/<pkg>/package.xml      -> /ws/src/<pkg>/package.xml
#   install/<pkg>/share/<pkg>/local_setup.bash -> /ws/build/<pkg>/...
# Через шов идёт только /ws/install, а /scripts/start_lslidar.sh делает
# `source /ws/install/setup.bash` — без /ws/build это битые ссылки.
# --parallel-workers 1 --executor sequential сохранены (ограничение памяти
# на сборочном хосте под qemu).
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --parallel-workers 1 --executor sequential

# ============================================================
# Stage 2: runtime — та же база, БЕЗ генераторов, ament-cmake и Boost-заголовков
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

# Рантайм-зависимости драйвера. БЕЗ rosidl-генераторов, ament-cmake и
# -dev-пакетов Boost.
# libboost-thread1.74.0 — разделяемая библиотека, к которой линкуется драйвер
# (раньше приходила транзитивно вместе с libboost-thread-dev). Версия в имени
# пакета — жёсткая для Ubuntu 22.04 jammy, на которой стоит база humble;
# при смене дистрибутива имя придётся обновить (сборка упадёт явно, не молча).
RUN apt-get update && apt-get install -y \
    ros-humble-rosidl-default-runtime \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    libboost-thread1.74.0 \
    && rm -rf /var/lib/apt/lists/*

# colcon workspace из builder-стадии — единственный продукт сборки, который
# обязан пересечь шов (план §2.2). Клон драйвера (/ws/src) и /ws/build
# в рантайм не попадают.
COPY --from=builder /ws/install /ws/install

WORKDIR /ws

# Entrypoint монтируется из volume: /scripts/start_lslidar.sh
CMD ["/scripts/start_lslidar.sh"]
