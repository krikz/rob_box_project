"""C1: adoption установленного релиза и dry-run activation/rollback без переключения стеков."""

from __future__ import annotations

import json

import pytest
from chuangong import state as st
from chuangong.agent import Config, Deps
from chuangong.stage import RoleStager
from chuangong.state import Mode
from fakes import FakeDocker, FakeZenoh

BIG = 1 << 40


def _cfg(tmp_path, world, **kw):
    base = dict(
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
    base.update(kw)
    return Config(**base)


def _deps(world, docker, free=BIG):
    return Deps(world.registry, docker, FakeZenoh(), disk_free=lambda p: free)


def _machine(cfg, deps, health=None):
    from chuangong.activation import ActivationMachine

    return ActivationMachine(cfg, deps, health=health)


def _stage(cfg, deps, release):
    return RoleStager(
        deps.registry, deps.docker, cfg.releases_root, cfg.docker_root, cfg.reserve_bytes, deps.disk_free
    ).run(release, cfg.node)


def _match_legacy_and_images(cfg, world, docker):
    dest = cfg.releases_root / world.release["release"] / cfg.node
    for path in dest.rglob("*"):
        if path.is_file() and path.name != ".chuangong-staged":
            target = cfg.legacy_root / path.relative_to(dest)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
    docker.image_ids = {ref: f"image-{i}" for i, ref in enumerate(world.images[cfg.node].values())}
    docker.running_ids = list(docker.image_ids.values())


def _remove_external_images(world):
    from chuangong import minisign
    from chuangong import release_format as rf
    from fakes import digest_of

    release = world.release
    for role in rf.ROLES:
        release["roles"][role]["external_images"] = []
    data = rf.dumps(release)
    sig = minisign.sign(data, world.secret, f"release:{release['release']}")
    world.transport.blobs[digest_of(data)] = data
    world.transport.blobs[digest_of(sig.encode())] = sig.encode()
    manifest = rf.artifact_manifest(data, sig.encode(), release)
    for ref in (release["release"], release["channel"]):
        world.transport.manifests[(rf.RELEASE_REPO, ref)] = manifest


def _save_history(cfg, current, previous=None):
    rec = st.new_record(cfg.robot_id, cfg.node, cfg.channel, Mode.OBSERVE, None)
    rec["current"] = current
    rec["previous"] = previous
    st.save(cfg.state_file, rec)


class Health:
    def __init__(self, status="planned", detail="health probe"):
        self.status = status
        self.detail = detail
        self.calls = []

    def check(self, role, release, dry_run):
        self.calls.append((role, release, dry_run))
        from chuangong.activation import HealthResult

        return HealthResult(self.status, self.detail)


def test_adoption_without_staged_evidence_is_blocked_and_writes_nothing(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    rec = _machine(cfg, _deps(world, docker)).adopt(world.release["release"])

    assert rec["state"] == "blocked"
    assert rec["reason"] == "release_not_staged"
    assert rec["detail"]["bundle_done"] is False
    assert rec["current"] is None and rec["previous"] is None
    assert not cfg.state_file.exists()


def test_adoption_saves_proven_current_and_null_previous(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "adopted", rec.get("detail")
    assert rec["current"] == world.release["release"]
    assert rec["previous"] is None
    assert rec["adoption"]["evidence"]["running_image_ids"] == sorted(docker.running_ids)
    saved = json.loads(cfg.state_file.read_text(encoding="utf-8"))
    assert saved["current"] == world.release["release"] and saved["previous"] is None


def test_adoption_blocks_unpinned_external_images(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "blocked" and rec["reason"] == "external_images_unpinned"
    assert rec["detail"] == {"node": "vision", "external_images": ["eclipse/zenoh:1.6.2"]}
    assert not cfg.state_file.exists()


def test_adoption_rejects_legacy_file_or_running_image_mismatch(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    compose = cfg.legacy_root / "docker" / "vision" / "docker-compose.yaml"
    compose.write_text("# locally changed\n", encoding="utf-8")

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)
    assert rec["state"] == "blocked" and rec["reason"] == "legacy_bundle_mismatch"
    assert rec["detail"][0]["path"] == "docker/vision/docker-compose.yaml"
    assert not cfg.state_file.exists()

    staged = cfg.releases_root / world.release["release"] / "vision"
    compose.write_bytes((staged / compose.relative_to(cfg.legacy_root)).read_bytes())
    docker.running_ids = []
    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)
    assert rec["state"] == "blocked" and rec["reason"] == "release_images_not_running"
    assert set(rec["detail"]["expected_image_ids"]) == set(docker.image_ids.values())


def test_standalone_preflight_is_read_only(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")

    rec = _machine(cfg, deps).preflight(world.release["release"])

    assert rec["state"] == "preflight_passed"
    assert rec["dry_run"] is True
    assert rec["preflight"] == {
        "release": world.release["release"],
        "staged": True,
        "hil_locked": False,
        "dirty": False,
        "current": "2026.11.08-1",
        "previous": None,
    }
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["current"] == "2026.11.08-1"


def test_activation_dry_run_has_fleet_order_health_and_would_transition(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")
    health = Health()

    rec = _machine(cfg, deps, health).activate(world.release["release"])

    assert rec["state"] == "activation_planned"
    assert rec["dry_run"] is True
    assert rec["current"] == "2026.11.08-1" and rec["previous"] is None
    assert rec["would"] == {"current": world.release["release"], "previous": "2026.11.08-1"}
    assert [step["node"] for step in rec["plan"] if step["op"] == "activate"] == ["vision", "sound", "main"]
    assert all(step["execute"] is False for step in rec["plan"])
    assert health.calls == [("vision", world.release["release"], True)]
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["current"] == "2026.11.08-1"


@pytest.mark.parametrize(
    ("history", "release_name", "reason"),
    [
        ((None, None), "2026.11.09-1", "current_unproven"),
        (("2026.11.09-1", None), "2026.11.09-1", "already_current"),
        (("2026.11.09-1", None), "2026.11.08-1", "no_previous"),
    ],
)
def test_history_preflight_is_machine_blocked(tmp_path, world, history, release_name, reason):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, *history)
    machine = _machine(cfg, deps)

    rec = machine.rollback() if reason == "no_previous" else machine.activate(release_name)

    assert rec["state"] == "blocked" and rec["reason"] == reason
    assert rec["detail"] == {"current": history[0], "previous": history[1]}


def test_activation_preflight_checks_hil_dirty_and_space(tmp_path, world):
    cfg = _cfg(tmp_path, world, node="main", reserve_bytes=1)
    docker = FakeDocker(hotpatched=["nav2"])
    deps = _deps(world, docker, free=0)
    cfg.hil_lock.write_text("hil on", encoding="utf-8")

    rec = _machine(cfg, deps).activate(world.release["release"])
    assert rec["state"] == "blocked" and rec["reason"] == "release_not_staged"

    _stage(cfg, _deps(world, docker), world.release)
    _save_history(cfg, "2026.11.08-1")
    rec = _machine(cfg, deps).activate(world.release["release"])
    assert rec["state"] == "blocked" and rec["reason"] == "hil_locked"
    assert rec["detail"] == str(cfg.hil_lock)

    cfg.hil_lock.unlink()
    rec = _machine(cfg, deps).activate(world.release["release"])
    assert rec["state"] == "blocked" and rec["reason"] == "dirty"
    assert rec["detail"] == ["container:nav2"]

    docker.hotpatched = []
    rec = _machine(cfg, deps).activate(world.release["release"])
    assert rec["state"] == "blocked" and rec["reason"] == "no_space"
    assert rec["detail"]["free_bytes"] == 0


def test_rollback_uses_signed_previous_release_and_does_not_swap_history(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    previous = world.release["release"]
    _stage(cfg, deps, world.release)
    world.publish("2026.11.10-1")
    _save_history(cfg, "2026.11.10-1", previous)
    health = Health()

    rec = _machine(cfg, deps, health).rollback()

    assert rec["state"] == "rollback_planned", rec.get("detail")
    assert rec["would"] == {"current": previous, "previous": "2026.11.10-1"}
    assert rec["current"] == "2026.11.10-1" and rec["previous"] == previous
    assert health.calls == [("vision", previous, True)]
    assert [step["node"] for step in rec["plan"] if step["op"] == "activate"] == ["vision", "sound", "main"]


def test_unhealthy_result_is_blocked_with_raw_detail(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")

    rec = _machine(cfg, deps, Health("unhealthy", "nav2: exited (127)")).activate(world.release["release"])

    assert rec["state"] == "blocked" and rec["reason"] == "health_check_failed"
    assert rec["detail"] == "nav2: exited (127)"


@pytest.mark.parametrize("command", ["adopt", "preflight", "activate", "rollback"])
def test_cli_exposes_c1_commands(command):
    from chuangong.__main__ import main

    with pytest.raises(SystemExit) as exc:
        main([command, "--help"])
    assert exc.value.code == 0
