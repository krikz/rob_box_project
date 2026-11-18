"""План переключения одной роли по digest: офлайн, без docker и без compose up."""

from __future__ import annotations

import copy

import pytest
from chuangong import release_format as rf
from chuangong.role_switch import (
    CHECK_CONTAINER_LOGS,
    CHECK_CONTAINER_STATUS,
    CHECK_ROS2_TOPICS,
    ROLE_ACTIONS,
    ROLE_CHECKS,
    HealthResult,
    RoleSwitchError,
    plan_role,
)


def _release_fixture(name: str = "2026.11.09-1") -> dict:
    """Изолированная фикстура подписанного по схеме release.yaml (без сети/реестра)."""
    roles = {}
    for role in rf.ROLES:
        digest0 = "a" * 64
        digest1 = "b" * 64 if role != "vision" else "c" * 64
        # разные digest на роль, чтобы план vision ≠ sound
        if role == "vision":
            digest0, digest1 = "1" * 64, "2" * 64
        elif role == "sound":
            digest0, digest1 = "3" * 64, "4" * 64
        else:
            digest0, digest1 = "5" * 64, "6" * 64
        roles[role] = {
            "bundle": {
                "file": rf.bundle_file_name(role, name),
                "sha256": "d" * 64,
                "size": 128,
            },
            "images": {
                f"{role}-svc0": f"registry.example/krikz/rob_box@sha256:{digest0}",
                f"{role}-svc1": f"registry.example/krikz/rob_box@sha256:{digest1}",
            },
            "external_images": [],
            "resource_pack": None,
            "migrations": (
                [f"scripts/setup/ensure_{role}.sh"] if role == "vision" else []
            ),
        }
    release = {
        "schema": 1,
        "release": name,
        "channel": "dev",
        "git": "abc1234",
        "created": "2026-11-09T12:00:00Z",
        "image_tag": "dev-abc1234",
        "docker_tag": "dev",
        "roles": roles,
        "compat": {"min_previous": None},
    }
    rf.validate(release)
    return release


def test_plan_role_uses_digest_images_and_deploy_checks():
    release = _release_fixture()
    plan = plan_role(release, "vision")

    assert plan["state"] == "role_switch_planned"
    assert plan["execute"] is False
    assert plan["role"] == "vision"
    assert plan["release"] == release["release"]
    assert plan["checks"] == list(ROLE_CHECKS)
    assert CHECK_CONTAINER_STATUS in plan["checks"]
    assert CHECK_CONTAINER_LOGS in plan["checks"]
    assert CHECK_ROS2_TOPICS in plan["checks"]
    assert plan["expected_topics"] == ["/camera/camera/color/image_raw/compressed"]

    assert set(plan["images"]) == {"vision-svc0", "vision-svc1"}
    for ref in plan["images"].values():
        assert "@sha256:" in ref
        assert ":" + release["image_tag"] not in ref
        assert not ref.endswith(":" + release["docker_tag"])

    assert len(plan["plan"]) == 1
    step = plan["plan"][0]
    assert step["op"] == "switch_role"
    assert step["execute"] is False
    assert step["actions"] == list(ROLE_ACTIONS)
    assert step["actions"] == [
        "compose_down",
        "switch_current",
        "host_migrations",
        "compose_up",
    ]
    assert step["images"] == plan["images"]
    assert step["checks"] == list(ROLE_CHECKS)
    assert step["migrations"] == ["scripts/setup/ensure_vision.sh"]
    assert "compose up" not in str(plan).lower() or step["execute"] is False


def test_plan_role_sound_and_main_topic_expectations():
    release = _release_fixture()
    sound = plan_role(release, "sound")
    main = plan_role(release, "main")

    assert sound["expected_topics"] == ["/voice/stt/utterance"]
    assert sound["images"]["sound-svc0"].endswith("3" * 64)
    assert main["expected_topics"] == []
    assert main["images"]["main-svc0"].endswith("5" * 64)
    assert sound["plan"][0]["op"] == "switch_role"
    assert main["plan"][0]["op"] == "switch_role"


def test_unhealthy_health_plans_rollback_to_previous_not_success():
    release = _release_fixture("2026.11.10-1")
    previous = "2026.11.09-1"
    health = HealthResult("unhealthy", "check_container_status: vision-svc0 restarting")

    plan = plan_role(release, "vision", previous=previous, health=health)

    assert plan["state"] == "rollback_planned"
    assert plan["state"] != "role_switch_planned"
    assert plan["reason"] == "health_check_failed"
    assert plan["detail"] == health.detail
    assert plan["release"] == previous
    assert plan["failed_release"] == "2026.11.10-1"
    assert plan["previous"] == previous
    assert plan["execute"] is False
    assert plan["health"] == {"status": "unhealthy", "detail": health.detail}
    assert len(plan["plan"]) == 1
    step = plan["plan"][0]
    assert step["op"] == "rollback"
    assert step["release"] == previous
    assert step["from_release"] == "2026.11.10-1"
    assert step["execute"] is False
    assert step["actions"] == list(ROLE_ACTIONS)
    assert step["checks"] == list(ROLE_CHECKS)
    # Не «успех»: нет switch_role на failed release.
    assert all(s["op"] != "switch_role" for s in plan["plan"])


def test_unhealthy_without_previous_is_blocked_not_success():
    release = _release_fixture()
    plan = plan_role(release, "vision", health=HealthResult("unhealthy", "no topics"))

    assert plan["state"] == "blocked"
    assert plan["reason"] == "no_previous"
    assert plan["plan"] == []
    assert plan["execute"] is False


def test_tag_image_rejected_even_if_schema_bypassed():
    release = _release_fixture()
    broken = copy.deepcopy(release)
    broken["roles"]["vision"]["images"][
        "vision-svc0"
    ] = "registry.example/krikz/rob_box:dev-abc1234"
    # validate не пропустит — обходим через подмену после validate в plan_role:
    # plan_role сам валидирует, поэтому ловим bad_release/image_not_digest.
    with pytest.raises(RoleSwitchError) as exc:
        plan_role(broken, "vision")
    assert exc.value.reason in ("bad_release", "image_not_digest")


def test_unknown_role_rejected():
    release = _release_fixture()
    with pytest.raises(RoleSwitchError, match="bad_role"):
        plan_role(release, "katana")
