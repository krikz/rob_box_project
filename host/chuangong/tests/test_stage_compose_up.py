"""DockerCli.compose_up: digest-images → compose env (*_TAG=image_tag@sha256), без live docker."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from chuangong.stage import (
    DockerCli,
    StageError,
    _service_tag_vars,
    compose_digest_env,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
BUILD_MANIFEST = REPO_ROOT / "docker" / "build-manifest.yaml"
HEX_A = "a" * 64
HEX_B = "b" * 64
DIGEST_A = f"sha256:{HEX_A}"
DIGEST_B = f"sha256:{HEX_B}"
IMAGE_TAG = "dev-b704404"


def _project(tmp_path: Path, role: str = "main") -> Path:
    """Каталог как docker/<role>: рядом лежит build-manifest (symlink на репо)."""
    docker_root = tmp_path / "docker"
    role_dir = docker_root / role
    role_dir.mkdir(parents=True)
    (docker_root / "build-manifest.yaml").symlink_to(BUILD_MANIFEST)
    (role_dir / "docker-compose.yaml").write_text("services: {}\n", encoding="utf-8")
    return role_dir


def _cli_capture(docker: DockerCli) -> dict:
    """Подмена _compose: без docker, возвращает env/args последнего вызова."""
    captured: dict = {}

    def fake_compose(project_dir, *args, env=None):
        captured["project_dir"] = Path(project_dir)
        captured["args"] = args
        captured["env"] = dict(env) if env is not None else None
        return subprocess.CompletedProcess(["docker", "compose", *args], 0, "", "")

    docker._compose = fake_compose  # type: ignore[method-assign]
    return captured


def test_service_tag_vars_from_real_manifest_not_invented():
    """nav2→NAV2_TAG и voice-base отсутствует (image_versions: false) — из манифеста."""
    import yaml

    manifest = yaml.safe_load(BUILD_MANIFEST.read_text(encoding="utf-8"))
    vars_map = _service_tag_vars(manifest)
    assert vars_map["nav2"] == "NAV2_TAG"
    assert vars_map["oak-d"] == "OAK_D_TAG"
    assert "voice-base" not in vars_map
    assert "supercollider" not in vars_map


def test_compose_digest_env_matches_image_versions_text_pinned_rule():
    """То же правило, что image_versions_text(pinned=True): TAG=image_tag@digest."""
    images = {
        "nav2": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_A}",
        "teleop": f"registry.example/krikz/rob_box:teleop-humble-{IMAGE_TAG}@{DIGEST_B}",
        "supercollider": f"registry.example/krikz/rob_box@{DIGEST_A}",  # image_versions: false
    }
    service_vars = {"nav2": "NAV2_TAG", "teleop": "TELEOP_TAG"}  # supercollider нет
    env = compose_digest_env(images, image_tag=IMAGE_TAG, service_vars=service_vars)
    assert env == {
        "NAV2_TAG": f"{IMAGE_TAG}@{DIGEST_A}",
        "TELEOP_TAG": f"{IMAGE_TAG}@{DIGEST_B}",
    }
    assert "SUPERCOLLIDER" not in "".join(env)
    assert "dev" not in env.values()
    assert "latest" not in env.values()


def test_compose_digest_env_bad_ref_raises():
    with pytest.raises(StageError, match="compose_up_bad_ref"):
        compose_digest_env(
            {"nav2": "10.1.1.249:5000/krikz/rob_box:nav2-humble-dev"},
            image_tag=IMAGE_TAG,
            service_vars={"nav2": "NAV2_TAG"},
        )
    with pytest.raises(StageError, match="compose_up_bad_ref"):
        compose_digest_env(
            {"nav2": f"repo@sha256:{'z' * 64}"},
            image_tag=IMAGE_TAG,
            service_vars={"nav2": "NAV2_TAG"},
        )


def test_docker_cli_compose_up_applies_digest_env(tmp_path):
    """Живой DockerCli кладёт *_TAG=image_tag@sha256 в env subprocess, не зовёт docker."""
    project = _project(tmp_path, "main")
    docker = DockerCli()
    captured = _cli_capture(docker)
    images = {
        "nav2": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_A}",
        "twist-mux": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_B}",
        # image_versions: false в манифесте sound — на main нет; неизвестный сервис игнор
        "not-a-manifest-service": f"registry.example/x@{DIGEST_A}",
    }

    docker.compose_up(project, images=images, image_tag=IMAGE_TAG)

    assert captured["args"] == ("up", "-d", "--force-recreate", "--pull", "never")
    env = captured["env"]
    assert env is not None
    assert env["NAV2_TAG"] == f"{IMAGE_TAG}@{DIGEST_A}"
    assert env["TWIST_MUX_TAG"] == f"{IMAGE_TAG}@{DIGEST_B}"
    # dev/latest не появились из воздуха
    assert env["NAV2_TAG"] != "dev"
    assert env["NAV2_TAG"] != "latest"
    assert "latest" not in env.values()
    assert env.get("not-a-manifest-service") is None
    # пример env, который тест увидел
    assert sorted(k for k in env if k.endswith("_TAG")) == ["NAV2_TAG", "TWIST_MUX_TAG"]


def test_docker_cli_compose_up_empty_images_refuses_without_dev_latest(tmp_path):
    """Пустой images / None — StageError, env с dev/latest не собирается."""
    project = _project(tmp_path, "vision")
    docker = DockerCli()
    captured = _cli_capture(docker)

    with pytest.raises(StageError, match="compose_up_unpinned") as exc_none:
        docker.compose_up(project, images=None, image_tag=IMAGE_TAG)
    with pytest.raises(StageError, match="compose_up_unpinned") as exc_empty:
        docker.compose_up(project, images={}, image_tag=IMAGE_TAG)

    assert exc_none.value.reason == "compose_up_unpinned"
    assert exc_empty.value.reason == "compose_up_unpinned"
    assert captured == {}  # _compose не вызывался
    assert "dev" not in str(exc_none.value.extra)
    assert "latest" not in str(exc_empty.value.extra)


def test_docker_cli_compose_up_missing_image_tag_refuses(tmp_path):
    project = _project(tmp_path, "main")
    docker = DockerCli()
    captured = _cli_capture(docker)
    images = {"nav2": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_A}"}

    with pytest.raises(StageError, match="compose_up_unpinned"):
        docker.compose_up(project, images=images)

    assert captured == {}


def test_docker_cli_compose_up_skips_image_versions_false(tmp_path):
    """sound: supercollider image_versions false — в env не попадает; voice-assistant — да."""
    project = _project(tmp_path, "sound")
    docker = DockerCli()
    captured = _cli_capture(docker)
    images = {
        "supercollider": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_A}",
        "voice-assistant": f"10.1.1.249:5000/krikz/rob_box@{DIGEST_B}",
    }

    docker.compose_up(project, images=images, image_tag=IMAGE_TAG)

    env = captured["env"]
    assert env is not None
    assert env["VOICE_ASSISTANT_TAG"] == f"{IMAGE_TAG}@{DIGEST_B}"
    assert not any("SUPERCOLLIDER" in k for k in env)
    assert "dev" not in env.values() and "latest" not in env.values()
