"""Реальный DockerCli отвечает на три проверки роли, которые были только у FakeDocker.

execute_role зовёт docker.check_container_status / check_container_logs /
check_ros2_topics. У DockerCli их не было: с --execute вызов падал на
отсутствующем адаптере, а не на health роли. Контракт тот же, что у фейка:
None = порядок, строка = деталь сбоя.
"""

from __future__ import annotations

import subprocess

import pytest

from chuangong.stage import DockerCli

PROJECT = "zhuravl-adapter-4375"
CONTAINER = f"{PROJECT}-probe"


def _docker(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=60)


@pytest.fixture()
def running_container():
    """amd64-контейнер, который реально running: busybox sleep infinity."""
    _docker("rm", "-f", CONTAINER)
    started = _docker("run", "-d", "--name", CONTAINER, "busybox:latest", "sleep", "infinity")
    if started.returncode != 0:
        pytest.skip(f"не поднять тестовый контейнер: {started.stderr.strip()}")
    yield CONTAINER
    _docker("rm", "-f", CONTAINER)


def test_status_of_running_container_is_ok(running_container):
    err = DockerCli().check_container_status(running_container)
    assert err is None, err


def test_status_of_missing_container_is_failure():
    err = DockerCli().check_container_status("zhuravl-adapter-no-such-container")
    assert err is not None and "no such" in err.lower()


def test_logs_of_running_container_are_readable(running_container):
    err = DockerCli().check_container_logs(running_container)
    assert err is None, err


def test_ros2_topics_reports_missing_when_container_has_no_ros(running_container):
    """В busybox нет ros2: проверка не должна падать исключением, а вернуть деталь."""
    err = DockerCli().check_ros2_topics(["/odom"], container=running_container)
    assert isinstance(err, str) and err
    assert "/odom" in err or "ros2" in err
