# syntax=docker/dockerfile:1
# checkov:skip=CKV_DOCKER_2: no HEALTHCHECK — this image is a build/test toolbox
#   (colcon + pytest), never run as a long-lived service; there is nothing to probe.
# checkov:skip=CKV_DOCKER_3: runs as root — GitHub Actions `container:` jobs need
#   root to write into the mounted workspace and apt-managed rosdep state; this
#   image is only ever pulled by trusted CI workflows in this repo, never exposed.
# checkov:skip=CKV2_DOCKER_1: `sudo` is required by rosdep/ROS2 tooling conventions
#   inside the CI container; same trust boundary as CKV_DOCKER_3 above.
# ── CI Runner Image for Rob Box ────────────────────────────────────────
# Contains ROS2 Humble + all test deps + Python packages.
# Rebuilt weekly via .github/workflows/B-Build CI Image.yml.
#
# Usage in workflows:
#   jobs:
#     unit-tests:
#       runs-on: ubuntu-22.04
#       container:
#         image: ghcr.io/krikz/rob-box-ci:humble
#       steps:
#         - uses: actions/checkout@v7
#         - run: source /opt/ros/humble/setup.bash && colcon build ...
#         - run: colcon test ...
# ────────────────────────────────────────────────────────────────────────

FROM ubuntu:22.04

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

# ── Locale & timezone (non-interactive) ─────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && \
    apt-get install -y --no-install-recommends locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── ROS2 Humble ────────────────────────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        ca-certificates \
        gnupg \
        lsb-release \
        sudo \
        rsync && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /tmp/ros.key && \
    gpg --dearmor < /tmp/ros.key > /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm /tmp/ros.key && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool \
        build-essential \
        cmake \
        git \
        libssl-dev \
        python3-dev \
        clang \
        lcov \
        libasio-dev \
        libtinyxml2-dev \
        python3-pip && \
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# ── ROS2 test / linting deps ───────────────────────────────────────────
# audio_common_msgs / nav2_msgs — интерфейсы, которые импортируют юнит-тесты
# (voice / mcp-tools).  Раньше `G: Run Tests` ставил их apt'ом на каждом
# прогоне; запекаем в образ (#3923).
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ament-cmake-pytest \
        python3-flake8-docstrings \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest-repeat \
        python3-pytest-rerunfailures \
        ros-dev-tools \
        ros-humble-audio-common-msgs \
        ros-humble-nav2-msgs \
        python3-colcon-coveragepy-result \
        python3-colcon-lcov-result \
        python3-colcon-meson \
        python3-colcon-mixin && \
    rm -rf /var/lib/apt/lists/*

# ── Python packages (pip) ──────────────────────────────────────────────
# pytest >= 8 is required — apt ships 6.2.5 which is incompatible.
# msgpack / aiohttp / pytz / python-telegram-bot / opencv — зависимости юнит-
# тестов, которые `G: Run Tests` раньше доставлял pip'ом на каждом прогоне
# (#3923).  opencv-python-headless ровно 4.10.0.84: колесо собрано с
# поддержкой numpy 1.21 из образа и не тянет апгрейд numpy (см. комментарий
# перед `colcon test: rob_box_perception` в G-Run Tests.yml); новее — numpy 2.
# pytest-xdist (#3935) — параллельные прогоны юнит-тестов (`pytest -n auto`) в
# rob_box_music / rob_box_voice / rob_box_animations.  Версия >=3.6, верхней
# границы нет: pip берёт свежую, совместимую с pytest>=8 из строки выше.
# Доустановку на прогоне сняли в #3929, поэтому плагин обязан жить в образе.
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        'pytest>=8' 'pytest-xdist>=3.6' pytest-asyncio pytest-cov pytest-mock \
        openai httpx requests pyyaml \
        msgpack aiohttp pytz \
        'python-telegram-bot[job-queue]==21.*' \
        'opencv-python-headless==4.10.0.84'

# ── Verify ─────────────────────────────────────────────────────────────
# `source setup.bash` под SHELL -u и `ros2 --version` в Humble оба падают —
# версию берём у dpkg, и отсутствие ros2cli роняет сборку, а не печать.
# Зависимости тестов (#3923): отсутствие любой роняет сборку, а numpy обязан
# остаться 1.x — opencv 4.10 не должен был утянуть numpy 2.
# pytest-xdist (#3935): `import xdist` падает — сборка падает, а не «параллель
# молча не заработала» на прогоне тестов.
# ROS-окружение подключаем с `set +u` только на source (setup.bash читает
# $AMENT_TRACE_SETUP_FILES без дефолта), затем -u возвращаем.
# hadolint ignore=SC1091
RUN python3 -c "import pytest; assert pytest.__version__ >= '8', f'pytest={pytest.__version__}'" && \
    python3 -c "import cv2, msgpack, aiohttp, telegram, pytz, numpy; assert numpy.__version__.startswith('1.'), f'numpy={numpy.__version__}'" && \
    python3 -c "import pytest, xdist; print(pytest.__version__, xdist.__version__)" && \
    ros2cli="$(dpkg-query -W -f='${Version}' ros-humble-ros2cli)" && \
    set +u && . /opt/ros/humble/setup.bash && set -u && \
    ros2 pkg prefix audio_common_msgs && \
    ros2 pkg prefix nav2_msgs && \
    echo "=== CI image ready ===" && \
    echo "ROS2: humble, ros2cli ${ros2cli}" && \
    echo "pytest: $(python3 -c 'import pytest; print(pytest.__version__)')" && \
    echo "openai: $(python3 -c 'import openai; print(openai.__version__)')" && \
    echo "cv2/numpy: $(python3 -c 'import cv2, numpy; print(cv2.__version__, numpy.__version__)')"

# ── Rosdep already initialized, workspace will be mounted ─────────────
RUN mkdir -p /ws
WORKDIR /ws
