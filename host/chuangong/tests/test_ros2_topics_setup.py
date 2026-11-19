"""check_ros2_topics должен звать ros2 через source setup.bash, а не напрямую.

В контейнере роли ros2 появляется в PATH только после source /opt/ros/humble/setup.bash.
Прямой docker exec ros2 падает с "executable file not found", хотя ros2 есть.
Контракт прежний: None = порядок, строка = деталь сбоя, без исключений.
"""

from __future__ import annotations

import subprocess
import textwrap

import pytest

from chuangong.stage import DockerCli

CONTAINER = "zhuravl-ros2setup-4375"


def _docker(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=120)


@pytest.fixture()
def container_with_sourced_ros2():
    """ubuntu: ros2 нет в PATH, но есть после source /opt/ros/humble/setup.bash.

    ros2 — заглушка, которая печатает /odom. Так проверяется путь вызова, а не сам ROS.
    """
    _docker("rm", "-f", CONTAINER)
    script = textwrap.dedent(
        """
        mkdir -p /opt/ros/humble
        printf 'export PATH=/opt/ros-bin:$PATH\\n' > /opt/ros/humble/setup.bash
        mkdir -p /opt/ros-bin
        printf '#!/bin/sh\\necho /odom\\n' > /opt/ros-bin/ros2
        chmod +x /opt/ros-bin/ros2
        sleep infinity
        """
    )
    started = _docker("run", "-d", "--name", CONTAINER, "ubuntu:22.04", "bash", "-lc", script)
    if started.returncode != 0:
        pytest.skip(f"не поднять контейнер: {started.stderr.strip()}")
    yield CONTAINER
    _docker("rm", "-f", CONTAINER)


def test_topics_found_when_ros2_only_after_setup(container_with_sourced_ros2):
    err = DockerCli().check_ros2_topics(["/odom"], container=container_with_sourced_ros2)
    assert err is None, err


def test_missing_topic_reported_when_ros2_sourced(container_with_sourced_ros2):
    err = DockerCli().check_ros2_topics(["/odom", "/scan"], container=container_with_sourced_ros2)
    assert err is not None and "/scan" in err


def test_no_ros2_at_all_returns_detail_not_exception():
    """Контейнер без ros2 и без setup.bash: деталь сбоя, не исключение."""
    name = "zhuravl-ros2setup-noros"
    _docker("rm", "-f", name)
    started = _docker("run", "-d", "--name", name, "ubuntu:22.04", "sleep", "infinity")
    if started.returncode != 0:
        pytest.skip(started.stderr.strip())
    try:
        err = DockerCli().check_ros2_topics(["/odom"], container=name)
        assert isinstance(err, str) and err
    finally:
        _docker("rm", "-f", name)
