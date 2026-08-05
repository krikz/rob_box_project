# syntax=docker/dockerfile:1
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
        libtinyxml2-dev && \
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# ── ROS2 test / linting deps ───────────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ament-cmake-pytest \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ament-cmake \
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
        python3-colcon-coveragepy-result \
        python3-colcon-lcov-result \
        python3-colcon-meson \
        python3-colcon-mixin && \
    rm -rf /var/lib/apt/lists/*

# ── Python packages (pip) ──────────────────────────────────────────────
# pytest >= 8 is required — apt ships 6.2.5 which is incompatible.
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        'pytest>=8' pytest-asyncio pytest-cov pytest-mock \
        openai httpx

# ── Verify ─────────────────────────────────────────────────────────────
RUN python3 -c "import pytest; assert pytest.__version__ >= '8', f'pytest={pytest.__version__}'" && \
    echo "=== CI image ready ===" && \
    echo "ROS2: $(source /opt/ros/humble/setup.bash && ros2 --version 2>/dev/null || echo 'humble')" && \
    echo "pytest: $(python3 -c 'import pytest; print(pytest.__version__)')" && \
    echo "openai: $(python3 -c 'import openai; print(openai.__version__)')"

# ── Rosdep already initialized, workspace will be mounted ─────────────
RUN mkdir -p /ws
WORKDIR /ws
