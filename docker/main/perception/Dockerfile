# Internal Dialogue Agent - Dockerfile для Main Pi
# Использует базовый образ ros2-zenoh
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, этап 4):
# builder-стадия держит rosidl-кодогенератор и ament_cmake — они нужны ТОЛЬКО
# во время `colcon build`, чтобы превратить .msg пакета rob_box_perception_msgs
# в C/C++/Python-исходники. Рантайм-стадия их не содержит: сгенерированный код
# и его typesupport-библиотеки уже лежат в /ws/install, а подключается он через
# rosidl-default-runtime (apt-пакет, ставится в рантайм-стадии явно — см. §2.2
# плана: generators build-only, runtime обязателен).
#
# ЧЕСТНАЯ ОГОВОРКА (план §2.1): BASE_IMAGE наследует build-essential/cmake/
# colcon/rosdep/git/vcstool от ros:humble-ros-base (апстрим OSRF). Обе стадии
# стоят на одном и том же BASE_IMAGE, поэтому этот унаследованный тулчейн
# остаётся и в рантайме. Шов убирает только то, что ДОБАВЛЯЕТ поверх базы сам
# этот файл: rosidl-default-generators, ament-cmake, ament-cmake-python,
# /ws/src, /ws/build, /ws/log и промежуточные артефакты pip. Писать здесь
# "рантайм без компилятора" было бы неправдой (ADR-0018).
#
# Про pip и --prefix (урок пилота led_matrix): на Debian/Ubuntu
# `pip3 install --prefix=/pyinstall` кладёт файлы НЕ в /pyinstall/lib/...,
# а в /pyinstall/local/lib/pythonX.Y/dist-packages — патч Debian в
# sysconfig добавляет уровень "local" независимо от заданного --prefix
# (проверено локально на ubuntu:22.04 + python3-pip 22.0.2). Поэтому в рантайм
# копируется /pyinstall/local → /usr/local, а не /pyinstall → /usr/local.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

# ============================================================
# Stage 1: builder — тулчейн кодогенерации + colcon build
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG ROS_DISTRO
ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер уже передаёт APT_PROXY этому сервису
# ("L-Build Main Pi Services.yml", build-args джоба build-perception), но
# Dockerfile его не объявлял — build-arg молча игнорировался, и apt ходил
# напрямую в packages.ros.org. Та же дыра, что нашёл пилот led_matrix
# (коммит dc7ed68a2): ARG APT_PROXY обязан быть в ОБЕИХ стадиях, иначе
# multi-stage гоняет apt-get update дважды мимо кеша и удваивает шанс поймать
# "Hash Sum mismatch" на рассинхронизированном edge CDN.
# Паттерн 1:1 из docker/base/Dockerfile.ros2-zenoh — с проверкой доступности
# прокси и тихим fallback на прямое соединение.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Build-time тулчейн для rosidl-кодогенерации ament_cmake-пакета
# rob_box_perception_msgs. rosidl-default-generators / ament-cmake /
# ament-cmake-python нужны только здесь — в рантайм они не попадут.
# rosidl-default-runtime и msg-пакеты ставятся и тут (нужны как
# find_package-зависимости при генерации), и в рантайм-стадии отдельно.
RUN apt-get update && apt-get install -y \
    python3-pip \
    # ROS 2 development packages needed for building message packages
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    # ROS 2 message packages
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория для ROS2 workspace
WORKDIR /ws
RUN mkdir -p src

# ============================================================
# ОПТИМИЗАЦИЯ КЭШИРОВАНИЯ: Копируем файлы от наименее к наиболее изменяемым
# ============================================================

# 1. Копируем package.xml (метаданные зависимостей) - изменяются редко
COPY src/rob_box_perception_msgs/package.xml /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception/package.xml /ws/src/rob_box_perception/

# 2. Устанавливаем ROS зависимости через rosdep (кэшируется если package.xml не изменился)
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
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. Копируем setup.py и requirements.txt - изменяются реже чем код
COPY src/rob_box_perception/setup.py /ws/src/rob_box_perception/
COPY docker/main/perception/requirements.txt /tmp/perception_requirements.txt

# 4. Устанавливаем Python зависимости (кэшируется отдельно от кода).
# --prefix=/pyinstall: в рантайм уезжает только готовый результат (.py/.so),
# без pip-кеша и промежуточных артефактов сборки. test -d ловит ситуацию,
# когда будущая версия pip перестанет добавлять уровень "local" — тогда
# COPY --from=builder /pyinstall/local скопировал бы пустоту, а openai/pytz
# не нашлись бы уже на роботе, а не в CI (ровно тот класс регрессии, который
# план §10 требует ловить здесь).
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --retries 5 --timeout 120 --prefix=/pyinstall -r /tmp/perception_requirements.txt && \
    rm /tmp/perception_requirements.txt && \
    if [ ! -d /pyinstall/local/lib ]; then \
        echo "FATAL: pip --prefix layout изменился, ожидался /pyinstall/local/lib"; \
        find /pyinstall -maxdepth 3 -type d; \
        exit 1; \
    fi

# 5. Копируем ресурсы (prompts) - изменяются иногда
COPY src/rob_box_perception/prompts /ws/src/rob_box_perception/prompts

# 6. Только ТЕПЕРЬ копируем весь исходный код (изменяется часто)
COPY src/rob_box_perception_msgs /ws/src/rob_box_perception_msgs
COPY src/rob_box_perception /ws/src/rob_box_perception

# 7. Сборка пакетов (сначала msgs, потом nodes)
#
# --symlink-install УБРАН (урок пилота led_matrix, план §2.2 уточнён коммитом
# 037d982bf): с этим флагом colcon кладёт в /ws/install не копии, а symlink'и
# и egg-link обратно в /ws/build и /ws/src. Интерфейс шва — только /ws/install,
# поэтому /ws/install с symlink'ами в рантайме оказался бы битым
# (ModuleNotFoundError rob_box_perception, отсутствующие launch/prompts).
# В одностадийном образе это не проявлялось, потому что /ws/src и /ws/build
# лежали в том же слое.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_perception_msgs rob_box_perception \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: runtime — та же база, БЕЗ rosidl-генераторов и ament-cmake
# ============================================================
FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ARG APT_PROXY=""

# См. комментарий про APT_PROXY в builder-стадии — обязан быть в обеих.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Runtime-зависимости. rosidl-default-runtime ОБЯЗАТЕЛЕН: сгенерированный код
# rob_box_perception_msgs подключает typesupport-библиотеки из /opt/ros, и без
# него `import rob_box_perception_msgs.msg` упадёт при старте ноды, а не при
# сборке (план §2.2). nav-msgs/control-msgs не входят в метапакет ros-base и
# перечислены явно: раньше их подтягивал `rosdep install` builder-стадии, через
# шов этот побочный эффект не проходит.
# rosidl-default-generators / ament-cmake / ament-cmake-python СОЗНАТЕЛЬНО
# отсутствуют — это build-only инструменты.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# pip site-packages из builder-стадии (openai, pytz и их транзитивные зависимости).
# ABI совпадает — обе стадии стоят на одном BASE_IMAGE, значит одна версия
# Python/glibc. Копируем /pyinstall/local, а не /pyinstall — см. шапку файла.
COPY --from=builder /pyinstall/local /usr/local

# colcon workspace — единственный продукт сборки, который обязан пересечь шов.
# Здесь же лежит python-часть сообщений
# (/ws/install/rob_box_perception_msgs/lib/pythonX.Y/site-packages/...),
# rosidl её в системные пути не ставит.
COPY --from=builder /ws/install /ws/install

WORKDIR /ws

# Source workspace в bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Версия пакета
RUN echo "0.1.0-alpha" > /version.txt

# Entry point от базового образа
# ros_entrypoint.sh уже настроен
