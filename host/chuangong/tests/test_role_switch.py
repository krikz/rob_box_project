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
    execute_role,
    plan_role,
)
from fakes import FakeDocker


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
            "migrations": ([f"scripts/setup/ensure_{role}.sh"] if role == "vision" else []),
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
    broken["roles"]["vision"]["images"]["vision-svc0"] = "registry.example/krikz/rob_box:dev-abc1234"
    # validate не пропустит — обходим через подмену после validate в plan_role:
    # plan_role сам валидирует, поэтому ловим bad_release/image_not_digest.
    with pytest.raises(RoleSwitchError) as exc:
        plan_role(broken, "vision")
    assert exc.value.reason in ("bad_release", "image_not_digest")


def test_unknown_role_rejected():
    release = _release_fixture()
    with pytest.raises(RoleSwitchError, match="bad_role"):
        plan_role(release, "katana")


def test_execute_role_records_compose_by_digest_and_marks_switched(tmp_path):
    release = _release_fixture()
    docker = FakeDocker()
    project = tmp_path / "docker" / "vision"
    project.mkdir(parents=True)
    state_dir = tmp_path / "var" / "chuangong"
    state_dir.mkdir(parents=True)

    result = execute_role(
        release,
        "vision",
        docker=docker,
        project_dir=project,
        state_dir=state_dir,
        previous="2026.11.08-1",
        health_after=HealthResult("healthy", "check_container_status: ok"),
    )

    assert result["state"] == "role_switched"
    assert result["execute"] is True
    assert result["health"]["status"] == "healthy"
    assert result["plan"][0]["op"] == "switch_role"
    assert result["plan"][0]["execute"] is True
    images = release["roles"]["vision"]["images"]
    assert ("compose_down", str(project)) in docker.commands
    assert ("compose_up", str(project), images) in docker.commands
    actions = [c["action"] for c in result["commands"]]
    assert actions == [
        "compose_down",
        "switch_current",
        "host_migrations",
        "compose_up",
    ]
    assert result["commands"][2]["migrations"] == ["scripts/setup/ensure_vision.sh"]
    # digest, не тег канала
    up = next(c for c in result["commands"] if c["action"] == "compose_up")
    for ref in up["images"].values():
        assert "@sha256:" in ref


def test_execute_role_unhealthy_after_up_runs_rollback_not_success(tmp_path):
    release = _release_fixture("2026.11.10-1")
    previous = "2026.11.09-1"
    docker = FakeDocker()
    project = tmp_path / "docker" / "vision"
    project.mkdir(parents=True)
    state_dir = tmp_path / "var" / "chuangong"
    state_dir.mkdir(parents=True)
    health = HealthResult("unhealthy", "check_container_status: vision-svc0 restarting")

    result = execute_role(
        release,
        "vision",
        docker=docker,
        project_dir=project,
        state_dir=state_dir,
        previous=previous,
        health_after=health,
    )

    assert result["state"] == "rollback_planned"
    assert result["state"] != "role_switched"
    assert result["state"] != "role_switch_planned"
    assert result["reason"] == "health_check_failed"
    assert result["release"] == previous
    assert result["failed_release"] == "2026.11.10-1"
    assert result["execute"] is True
    assert len(result["plan"]) == 1
    assert result["plan"][0]["op"] == "rollback"
    assert result["plan"][0]["release"] == previous
    assert all(s["op"] != "switch_role" for s in result["plan"])
    # switch attempt + rollback: compose_down/up twice
    downs = [c for c in docker.commands if c[0] == "compose_down"]
    ups = [c for c in docker.commands if c[0] == "compose_up"]
    assert len(downs) == 2 and len(ups) == 2
    actions = [c["action"] for c in result["commands"]]
    assert actions.count("compose_down") == 2
    assert actions.count("compose_up") == 2
    assert "switch_current" in actions


def test_activate_dry_run_uses_plan_role_steps(tmp_path, world):
    """Штатный activate dry-run зовёт plan_role — шаги switch_role с digest, не второй список."""
    import json

    from chuangong import state as st
    from chuangong.agent import Config, Deps
    from chuangong.stage import RoleStager
    from chuangong.state import Mode
    from fakes import FakeZenoh

    cfg = Config(
        robot_id="RBXU100001",
        node="vision",
        pubkey=world.pubkey,
        state_dir=tmp_path / "var" / "chuangong",
        releases_root=tmp_path / "opt" / "releases",
        legacy_root=tmp_path / "legacy",
        docker_root=tmp_path,
        hil_lock=tmp_path / "hil.lock",
        reserve_bytes=0,
    )
    docker = FakeDocker()
    deps = Deps(world.registry, docker, FakeZenoh(), disk_free=lambda p: 1 << 40)
    RoleStager(
        deps.registry,
        deps.docker,
        cfg.releases_root,
        cfg.docker_root,
        cfg.reserve_bytes,
        deps.disk_free,
    ).run(world.release, cfg.node)
    rec = st.new_record(cfg.robot_id, cfg.node, cfg.channel, Mode.OBSERVE, None)
    rec["current"] = "2026.11.08-1"
    st.save(cfg.state_file, rec)

    from chuangong.activation import ActivationMachine

    out = ActivationMachine(cfg, deps).activate(world.release["release"])

    assert out["state"] == "activation_planned"
    assert [s["op"] for s in out["plan"]] == [
        "switch_role",
        "switch_role",
        "switch_role",
    ]
    assert [s["role"] for s in out["plan"]] == ["vision", "sound", "main"]
    # actions/images пришли из plan_role, не из локальной копии в activation
    from chuangong.role_switch import ROLE_ACTIONS, plan_role

    direct = plan_role(world.release, "vision", previous="2026.11.08-1")
    assert out["plan"][0]["actions"] == list(ROLE_ACTIONS) == direct["plan"][0]["actions"]
    assert out["plan"][0]["images"] == direct["plan"][0]["images"]
    assert out["plan"][0]["checks"] == direct["plan"][0]["checks"]
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["current"] == "2026.11.08-1"


def test_execute_role_switch_current_advances_pointers(tmp_path):
    """switch_current реально двигает current/previous через pointers.advance."""
    from chuangong import pointers

    release = _release_fixture("2026.11.10-1")
    previous = "2026.11.09-1"
    state_dir = tmp_path / "var" / "chuangong"
    state_dir.mkdir(parents=True)
    (state_dir / "current").write_text(previous + "\n", encoding="utf-8")
    docker = FakeDocker()
    project = tmp_path / "docker" / "vision"
    project.mkdir(parents=True)

    result = execute_role(
        release,
        "vision",
        docker=docker,
        project_dir=project,
        state_dir=state_dir,
        previous=previous,
        health_after=HealthResult("healthy", "ok"),
    )

    assert result["state"] == "role_switched"
    assert pointers.load(state_dir) == ("2026.11.10-1", previous)
    switch = next(c for c in result["commands"] if c["action"] == "switch_current")
    assert switch["release"] == "2026.11.10-1"
    assert switch["current"] == "2026.11.10-1"
    assert switch["previous"] == previous


def test_plan_role_does_not_advance_pointers(tmp_path):
    """dry-run plan_role не трогает указатели на диске."""
    from chuangong import pointers

    release = _release_fixture("2026.11.10-1")
    previous = "2026.11.09-1"
    state_dir = tmp_path / "var" / "chuangong"
    state_dir.mkdir(parents=True)
    (state_dir / "current").write_text(previous + "\n", encoding="utf-8")

    plan = plan_role(release, "vision", previous=previous)

    assert plan["execute"] is False
    assert plan["state"] == "role_switch_planned"
    assert pointers.load(state_dir) == (previous, None)
    assert not (state_dir / "previous").exists()


def test_execute_role_switch_current_advance_error_is_not_success(tmp_path):
    """Падение advance — ошибка исполнителю, не «успех»."""
    from chuangong.pointers import PointerError

    release = _release_fixture("2026.11.10-1")
    state_dir = tmp_path / "var" / "chuangong"
    state_dir.mkdir(parents=True)
    # symlink current → advance обязан упасть compose_symlink_untouched
    (state_dir / "current").symlink_to(tmp_path / "elsewhere")
    docker = FakeDocker()
    project = tmp_path / "docker" / "vision"
    project.mkdir(parents=True)

    with pytest.raises(PointerError) as exc:
        execute_role(
            release,
            "vision",
            docker=docker,
            project_dir=project,
            state_dir=state_dir,
            previous="2026.11.09-1",
            health_after=HealthResult("healthy", "ok"),
        )
    assert exc.value.reason == "compose_symlink_untouched"
