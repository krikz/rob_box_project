"""Guard #3664 п.2: cAdvisor должен видеть контейнеры на Docker 29 (containerd snapshotter).

cAdvisor <= 0.52.1 ищет слой контейнера в image/overlayfs/layerdb/mounts, а на Docker 29
с containerd image store (Vision, Sound) там пусто: поконтейнерных метрик нет
(google/cadvisor#3709, исправлено в 0.54.0). Проверяем сами compose-файлы: версия образа
и тома, без которых cAdvisor не читает ни cgroup, ни /var/lib/docker.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
DOCKER29_ROLES = ("vision", "sound")  # на них Docker 29 + containerd snapshotter (измерено 09.11)
FIXED_IN = (0, 54, 0)
REQUIRED_MOUNTS = ("/:/rootfs:ro", "/var/run:/var/run:ro", "/sys:/sys:ro", "/var/lib/docker/:/var/lib/docker:ro")


def _cadvisor(role: str) -> dict:
    compose = yaml.safe_load((REPO_ROOT / "docker" / role / "docker-compose.yaml").read_text(encoding="utf-8"))
    return compose["services"]["cadvisor"]


def _version(image: str) -> tuple[int, ...]:
    tag = image.rsplit(":", 1)[1]
    return tuple(int(p) for p in tag.lstrip("v").split("."))


@pytest.mark.parametrize("role", DOCKER29_ROLES)
def test_cadvisor_version_understands_containerd_snapshotter(role: str) -> None:
    assert _version(_cadvisor(role)["image"]) >= FIXED_IN


@pytest.mark.parametrize("role", DOCKER29_ROLES)
def test_cadvisor_image_comes_from_registry_that_has_fixed_versions(role: str) -> None:
    # в gcr.io/cadvisor выше 0.52.1 образов нет
    assert _cadvisor(role)["image"].startswith("ghcr.io/google/cadvisor:")


@pytest.mark.parametrize("role", (*DOCKER29_ROLES, "main"))
def test_cadvisor_mounts_what_it_needs_to_see_containers(role: str) -> None:
    svc = _cadvisor(role)
    for mount in REQUIRED_MOUNTS:
        assert mount in svc["volumes"], mount
    assert "--docker_only=true" in svc["command"]
