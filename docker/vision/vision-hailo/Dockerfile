# ADR-0089 Phase 1: AI HAT+ 26 TOPS inference service (Vision Pi).
#
# Base: ros2-zenoh (та же база, что oak-d / led-matrix).
# Workspace: rob_box_perception_msgs + rob_box_perception собираются
# здесь через colcon в /ws/install (паттерн docker/main/perception).
# Скрипты запуска НЕ копируются в образ — монтируются volume'ами:
#   ./scripts/vision-hailo:/scripts:ro   (start_vision_hailo.sh)
#   ./scripts:/ros_scripts:ro            (ros_with_namespace.sh)
#
# Phase 1 (default): HAILO_ENABLED=false → stub-режим, железо/HailoRT
# не нужны. Реальный inference (hailo_enabled=true) — Phase 1.5,
# отдельной задачей после установки HEF на Vision Pi.
#
# Phase 1 acceptance (см. ADR-0089 §9):
#   hailortcli scan на Vision Pi возвращает `hailo8` device.
#
# Phase 1 binding strategy — см. ADR-0099:
#   - hailo_platform (Python binding) **опциональный** на Phase 1 —
#     capability-honest (ADR-0018) stub-режим. Real install — Phase 1.5
#     (отдельная карточка, .deb/.whl от Hailo Developer Zone).
#   - HAILO_INSTALL_C_DEV=true (default) — ставит libhailort-dev через apt
#     как C-заголовки для Phase 1.5/2 build-wheel. На CI без Hailo apt-repo
#     ставится best-effort || true.
#   - HAILO_INSTALL_C_DEV=false — пропускаем apt-шаг полностью (для offline build).
#
# ============================================================
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, этап 4)
# ============================================================
# Интерфейс шва здесь ШИРЕ, чем /ws/install (план §2.3):
#   1. /ws/install            — продукт colcon build, идёт через COPY --from=builder.
#   2. hailort .whl           — ГОТОВЫЙ cp310-wheel, компиляции не требует, поэтому
#                               ставится ПРЯМО в рантайм-стадии, минуя builder
#                               (план §2.3 п.3). ABI совпадает: обе стадии — один
#                               и тот же BASE_IMAGE, один Python 3.10.
#   3. libhailort.so.4.24.0   — внешний бинарный артефакт (не продукт сборки),
#                               ставится в рантайм-стадии рядом с wheel'ом: без
#                               него `import hailo_platform` падает при старте.
#   4. hailort-pcie-driver    — kernel-модуль, живёт на ХОСТЕ (Vision Pi), в образ
#                               не попадает вообще (ADR-0089 §4, /dev/hailo0
#                               пробрасывается как device). Вне зоны шва.
# Из рантайма шов убирает: rosidl-default-generators, ament-cmake,
# ament-cmake-python, libhailort-dev (C-заголовки), /ws/src, /ws/build, /ws/log
# и промежуточные артефакты pip.
#
# ЧЕСТНАЯ ОГОВОРКА (план §2.1): BASE_IMAGE наследует build-essential/cmake/
# colcon/rosdep/git от ros:humble-ros-base (апстрим OSRF), и обе стадии стоят
# на нём же — этот унаследованный тулчейн в рантайме ОСТАЁТСЯ. Шов убирает
# только то, что добавляет поверх базы сам этот файл. Говорить "рантайм без
# компилятора" было бы неправдой (ADR-0018).
#
# Про pip и --prefix (урок пилота led_matrix): на Debian/Ubuntu
# `pip3 install --prefix=/pyinstall` кладёт файлы в
# /pyinstall/local/lib/pythonX.Y/dist-packages, а не в /pyinstall/lib/... —
# патч Debian в sysconfig добавляет уровень "local". Поэтому в рантайм
# копируется /pyinstall/local → /usr/local.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-${ROS_DISTRO}

# ============================================================
# Stage 1: builder — тулчейн rosidl-кодогенерации + colcon build
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG ROS_DISTRO=humble
ARG APT_PROXY=""

ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive

# apt-cacher-ng на build-хосте. Раннер уже передаёт APT_PROXY этому сервису
# (build-args в "L-Build Vision Pi Services.yml" / "L-Build Single Service.yml"),
# но Dockerfile его не объявлял — build-arg молча игнорировался, и apt ходил
# напрямую в packages.ros.org. Та же дыра, что нашёл пилот led_matrix
# (коммит dc7ed68a2): ARG APT_PROXY обязан быть в ОБЕИХ стадиях, иначе
# multi-stage гоняет apt-get update дважды мимо кеша и удваивает шанс поймать
# "Hash Sum mismatch" на рассинхронизированном edge CDN.
# Паттерн 1:1 из docker/base/Dockerfile.ros2-zenoh.
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

# Build-time тулчейн для rosidl msg-пакета rob_box_perception_msgs.
# rosidl-default-generators / ament-cmake / ament-cmake-python — build-only,
# в рантайм не попадут. nav-msgs/control-msgs нужны и на генерации, и в
# рантайме (в рантайм-стадии ставятся отдельно).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-python \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-control-msgs \
    && rm -rf /var/lib/apt/lists/*

# HailoRT C-binding (libhailort-dev) — C-ЗАГОЛОВКИ для сборки Python wheel из
# исходников (ADR-0099 §2.1 п.2). Это build-time артефакт, поэтому после
# введения шва он живёт в builder-стадии и в рантайм не переезжает.
# Optional на CI без Hailo apt-repo (best-effort, capability-honest stub).
# Реальная рантайм-библиотека — libhailort.so.4.24.0 из вендорского пака,
# она ставится в рантайм-стадии ниже, и её отсутствие здесь ничего не ломает.
ARG HAILO_INSTALL_C_DEV=true
RUN if [ "${HAILO_INSTALL_C_DEV}" = "true" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends libhailort-dev || \
            echo "WARN: libhailort-dev install skipped (Hailo apt-repo недоступен на CI — capability-honest, Phase 1 stub OK)"; \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "HAILO_INSTALL_C_DEV=false — пропускаем apt-шаг (offline build)"; \
    fi

# Smoke-test `hailortcli` перенесён в start_vision_hailo.sh:100-110 — там он
# запускается только когда HAILO_ENABLED=true, т.е. когда реально нужно проверить
# наличие device. На CI Phase 1 (HAILO_ENABLED=false) smoke не имеет смысла.

# ---------- workspace (паттерн docker/main/perception) ----------
WORKDIR /ws
RUN mkdir -p src

# 1. package.xml (метаданные) — кэшируются отдельно
COPY src/rob_box_perception_msgs/package.xml /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception/package.xml /ws/src/rob_box_perception/

# 2. rosdep (best-effort — apt-зависимости явно установлены выше)
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
    rosdep update || true; \
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. setup.py / setup.cfg / requirements — реже, чем код.
# --prefix=/pyinstall: в рантайм уезжает только готовый результат (.py/.so),
# без pip-кеша и промежуточных артефактов. Проверка на /pyinstall/local/lib
# ловит смену install-scheme будущим pip — иначе COPY --from=builder
# /pyinstall/local скопировал бы пустоту, и openai/pytz не нашлись бы уже на
# роботе, а не в CI (класс регрессии из плана §10).
COPY src/rob_box_perception/setup.py /ws/src/rob_box_perception/
COPY src/rob_box_perception/setup.cfg /ws/src/rob_box_perception/
COPY src/rob_box_perception/requirements.txt /tmp/perception_requirements.txt
RUN pip3 install --retries 5 --timeout 120 --no-cache-dir --prefix=/pyinstall -r /tmp/perception_requirements.txt && \
    rm /tmp/perception_requirements.txt && \
    if [ ! -d /pyinstall/local/lib ]; then \
        echo "FATAL: pip --prefix layout изменился, ожидался /pyinstall/local/lib"; \
        find /pyinstall -maxdepth 3 -type d; \
        exit 1; \
    fi

# 4. data_files (resource / prompts / launch из setup.py) — изменяются иногда
COPY src/rob_box_perception/resource /ws/src/rob_box_perception/resource
COPY src/rob_box_perception/prompts /ws/src/rob_box_perception/prompts
COPY src/rob_box_perception/launch /ws/src/rob_box_perception/launch

# 5. весь исходный код (горячий слой). SOURCE_HASH инвалидирует кэш
# при изменении src/rob_box_perception* (issue #2314 pattern).
ARG SOURCE_HASH=unknown
RUN echo "Source hash: ${SOURCE_HASH}" > /tmp/.source_hash
COPY src/rob_box_perception_msgs /ws/src/rob_box_perception_msgs
COPY src/rob_box_perception /ws/src/rob_box_perception

# 6. сборка (сначала msgs, потом nodes)
#
# --symlink-install УБРАН (урок пилота led_matrix, коммит 037d982bf, уточнение
# к плану §2.2): с этим флагом colcon кладёт в /ws/install не копии, а
# symlink'и/egg-link обратно в /ws/build и /ws/src. Интерфейс шва — только
# /ws/install, поэтому в рантайме такой install оказался бы битым
# (ModuleNotFoundError rob_box_perception, отсутствующие launch/prompts).
# В одностадийном образе это не проявлялось: /ws/src и /ws/build лежали рядом.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_perception_msgs rob_box_perception \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: runtime — та же база, БЕЗ rosidl-генераторов, ament-cmake
#                    и libhailort-dev
# ============================================================
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble
ARG APT_PROXY=""

LABEL org.opencontainers.image.title="rob_box vision-hailo" \
      org.opencontainers.image.description="AI HAT+ 26 TOPS inference (ADR-0089)" \
      org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive

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
# rob_box_perception_msgs подключает typesupport-библиотеки из /opt/ros, без
# него `import rob_box_perception_msgs.msg` падает при старте ноды, а не при
# сборке (план §2.2). nav-msgs/control-msgs не входят в ros-base.
# numpy/opencv/yaml — рантайм-зависимости vision_hailo/vision_face нод.
# rosidl-default-generators / ament-cmake / ament-cmake-python СОЗНАТЕЛЬНО
# отсутствуют — это build-only инструменты, они остались в builder-стадии.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        python3-opencv \
        python3-yaml \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-control-msgs \
    && rm -rf /var/lib/apt/lists/*

# pip site-packages из builder-стадии (openai, pytz и транзитивные зависимости).
# ABI совпадает — обе стадии на одном BASE_IMAGE, один Python 3.10/glibc.
# Копируем /pyinstall/local, а не /pyinstall — см. шапку файла.
COPY --from=builder /pyinstall/local /usr/local

# colcon workspace — продукт сборки, единственное, что обязано пересечь шов
# из builder'а. Python-часть сообщений лежит здесь же
# (/ws/install/rob_box_perception_msgs/lib/pythonX.Y/site-packages/...),
# rosidl не ставит её в системные пути.
COPY --from=builder /ws/install /ws/install

# HailoRT Python binding (hailo_platform) — Phase 1.5 real inference (ADR-0099 §2.2).
# Wheel + libhailort.so.4.24.0 — артефакты Hailo Developer Zone (аккаунт), НЕ в git.
# Wheel бандлит python-обвязку, но НЕ libhailort.so — её извлекаем из
# hailort_4.24.0_arm64.deb. Workflow кладёт ОБА файла в build-context из
# /opt/rob_box/vendor/ на build-машине (см. L-Build Vision Pi Services.yml)
# и жёстко падает, если пака нет — сознательно, см. коммит 23dd5b5fb.
# Без wheel'а (HAILO_INSTALL_BINDING != whl) нода capability-honest деградирует
# в stub — HAILO_ENABLED=true без биндинга НЕ даёт real inference.
#
# ПОЧЕМУ В РАНТАЙМ-СТАДИИ, А НЕ В BUILDER (план §2.3 п.2-3): wheel готовый
# (cp310, prebuilt), компилятор ему не нужен; .so — внешний бинарь, не продукт
# сборки. Гнать их через шов значило бы копировать в builder то, что builder'у
# не нужно (`import hailo_platform` в rob_box_perception ленивый —
# hailo_device.py:105, vision_hailo_loader.py:322 — colcon build его не
# трогает), и раскладывать установленные файлы по рантайму вручную.
COPY docker/vision/vision-hailo/wheels/ /wheels/
ARG HAILO_INSTALL_BINDING=none
RUN if [ "${HAILO_INSTALL_BINDING}" = "whl" ]; then \
        pip3 install --no-cache-dir /wheels/hailort-4.24.0-cp310-cp310-linux_aarch64.whl && \
        install -m 0755 /wheels/libhailort.so.4.24.0 /usr/lib/ && \
        ln -sf libhailort.so.4.24.0 /usr/lib/libhailort.so && \
        ldconfig && \
        python3 -c "import hailo_platform; print('hailo_platform', hailo_platform.__version__)"; \
    else \
        echo "HAILO_INSTALL_BINDING=${HAILO_INSTALL_BINDING}: hailo_platform НЕ установлен (capability-honest stub, ADR-0099 §2.2)"; \
    fi && \
    rm -rf /wheels

WORKDIR /ws

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# entrypoint — ros_with_namespace.sh (volume) + start script (volume).
CMD ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_vision_hailo.sh"]
