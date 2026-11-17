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
    docker.compose_images = {service: [docker.image_ids[ref]] for service, ref in world.images[cfg.node].items()}


def _republish(world, *, external_images=None, created=None, min_previous=None):
    from chuangong import minisign
    from chuangong import release_format as rf
    from fakes import digest_of

    release = world.release
    if external_images is not None:
        for role in rf.ROLES:
            release["roles"][role]["external_images"] = external_images
    if created is not None:
        release["created"] = created
    release["compat"]["min_previous"] = min_previous
    data = rf.dumps(release)
    sig = minisign.sign(data, world.secret, f"release:{release['release']}")
    world.transport.blobs[digest_of(data)] = data
    world.transport.blobs[digest_of(sig.encode())] = sig.encode()
    manifest = rf.artifact_manifest(data, sig.encode(), release)
    for ref in (release["release"], release["channel"]):
        world.transport.manifests[(rf.RELEASE_REPO, ref)] = manifest


def _remove_external_images(world):
    _republish(world, external_images=[])


def _save_history(cfg, current, previous=None):
    rec = st.new_record(cfg.robot_id, cfg.node, cfg.channel, Mode.OBSERVE, None)
    rec["current"] = current
    rec["previous"] = previous
    st.save(cfg.state_file, rec)


class Health:
    def __init__(self, status="not_verified", detail="health not executed in dry-run"):
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
    assert rec["adoption"]["evidence"]["compose_services"] == docker.compose_images
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
    docker.compose_images = {}
    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)
    assert rec["state"] == "blocked" and rec["reason"] == "compose_services_mismatch"
    assert rec["detail"]["running_services"] == {}


@pytest.mark.parametrize("case", ["extra_service", "old_managed", "shared_image", "wrong_service"])
def test_adoption_requires_exact_compose_service_matrix(tmp_path, world, case):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    services = list(world.images[cfg.node])
    refs = list(world.images[cfg.node].values())

    if case == "extra_service":
        docker.compose_images["rogue"] = ["image-rogue"]
    elif case == "old_managed":
        docker.compose_images[services[0]].append("image-old")
    elif case == "shared_image":
        shared = docker.image_ids[refs[0]]
        docker.image_ids[refs[1]] = shared
        docker.compose_images = {services[0]: [shared]}
    else:
        docker.compose_images = {
            services[0]: [docker.image_ids[refs[1]]],
            services[1]: [docker.image_ids[refs[0]]],
        }

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "blocked" and rec["reason"] == "compose_services_mismatch"
    assert rec["detail"]["running_services"] == docker.compose_images
    assert not cfg.state_file.exists()


@pytest.mark.parametrize(
    "filename",
    [
        "docker-compose.override.yaml",
        "docker-compose.override.yml",
        "compose.override.yaml",
        "compose.override.yml",
        "docker-compose.local.override.yml",
        "compose.prod.override.yaml",
    ],
)
def test_adoption_blocks_compose_override_files(tmp_path, world, filename):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    override = cfg.legacy_root / "docker" / "vision" / filename
    override.write_text("services: {}\n", encoding="utf-8")

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "blocked" and rec["reason"] == "compose_override_present"
    assert rec["detail"] == [f"docker/vision/{filename}"]
    assert not cfg.state_file.exists()


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
    assert rec["health"] == {"status": "not_verified", "detail": "health not executed in dry-run"}
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


def test_min_previous_blocks_older_current_and_accepts_boundary(tmp_path, world):
    world.publish("2026.11.08-1")
    _republish(world, created="2026-11-08T12:00:00Z")
    world.publish("2026.11.09-1")
    _republish(world, created="2026-11-09T12:00:00Z")
    world.publish("2026.11.10-1")
    _republish(world, created="2026-11-10T12:00:00Z", min_previous="2026.11.09-1")
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")

    rec = _machine(cfg, deps).preflight(world.release["release"])

    assert rec["state"] == "blocked" and rec["reason"] == "compat_min_previous"
    assert rec["detail"] == {"current": "2026.11.08-1", "min_previous": "2026.11.09-1"}

    _save_history(cfg, "2026.11.09-1")
    rec = _machine(cfg, deps).preflight(world.release["release"])
    assert rec["state"] == "preflight_passed"


def test_min_previous_without_current_manifest_is_unproven(tmp_path, world):
    world.publish("2026.11.09-1")
    _republish(world, created="2026-11-09T12:00:00Z")
    world.publish("2026.11.10-1")
    _republish(world, created="2026-11-10T12:00:00Z", min_previous="2026.11.09-1")
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "dev-missing-1")

    rec = _machine(cfg, deps).preflight(world.release["release"])

    assert rec["state"] == "blocked" and rec["reason"] == "compat_unproven"
    assert rec["detail"]["current"] == "dev-missing-1"
    assert rec["detail"]["min_previous"] == "2026.11.09-1"
    assert "release_not_found" in rec["detail"]["error"]


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


def test_dry_run_rejects_health_success_claim(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")

    rec = _machine(cfg, deps, Health("healthy", "unverified fake success")).activate(world.release["release"])

    assert rec["state"] == "blocked" and rec["reason"] == "health_check_not_verified"
    assert rec["detail"] == {"status": "healthy", "detail": "unverified fake success"}


def test_docker_compose_probe_preserves_duplicates_and_ignores_unlabeled():
    import subprocess

    from chuangong.stage import DockerCli

    docker = DockerCli()
    responses = {
        ("ps", "--format", "{{.ID}}"): subprocess.CompletedProcess([], 0, "c1\nc2\nc3\n", ""),
        (
            "inspect",
            "--format",
            '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}',
            "c1",
        ): subprocess.CompletedProcess([], 0, "vision-svc1 sha256:new\n", ""),
        (
            "inspect",
            "--format",
            '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}',
            "c2",
        ): subprocess.CompletedProcess([], 0, "vision-svc1 sha256:old\n", ""),
        (
            "inspect",
            "--format",
            '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}',
            "c3",
        ): subprocess.CompletedProcess([], 0, "<no value> sha256:other\n", ""),
    }
    docker._run = lambda *args, timeout=30.0: responses[args]

    assert docker.running_compose_image_ids() == {"vision-svc1": ["sha256:new", "sha256:old"]}


@pytest.mark.parametrize("command", ["adopt", "preflight", "activate", "rollback"])
def test_cli_exposes_c1_commands(command):
    from chuangong.__main__ import main

    with pytest.raises(SystemExit) as exc:
        main([command, "--help"])
    assert exc.value.code == 0


def test_adoption_blocks_partial_compose_probe_error(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    service = next(iter(world.images[cfg.node]))
    docker.compose_images[service].append("image-old")
    docker.compose_probe_errors[service] = "docker inspect: permission denied"

    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "blocked" and rec["reason"] == "compose_probe_failed"
    assert rec["detail"] == {
        "service": service,
        "container": f"{service}-old",
        "error": "docker inspect: permission denied",
    }
    assert not cfg.state_file.exists()


def test_docker_cli_compose_probe_failure_has_raw_detail():
    import subprocess

    from chuangong.stage import DockerCli, StageError

    docker = DockerCli()
    responses = {
        ("ps", "--format", "{{.ID}}"): subprocess.CompletedProcess([], 0, "c-new\nc-old\n", ""),
        (
            "inspect",
            "--format",
            '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}',
            "c-new",
        ): subprocess.CompletedProcess([], 0, "vision-svc1 sha256:new\n", ""),
        (
            "inspect",
            "--format",
            '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}',
            "c-old",
        ): subprocess.CompletedProcess([], 1, "", "permission denied"),
    }
    docker._run = lambda *args, timeout=30.0: responses[args]

    with pytest.raises(StageError) as exc:
        docker.running_compose_image_ids()
    assert exc.value.reason == "compose_probe_failed"
    assert exc.value.extra == {"service": "unknown", "container": "c-old", "error": "permission denied"}


def test_adoption_and_tick_are_serialized(tmp_path, world, monkeypatch):
    import queue
    import threading
    from contextlib import contextmanager

    from chuangong.agent import tick

    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    _save_history(cfg, None)
    original_lock = getattr(st, "write_lock", None)
    original_load, original_save = st.load, st.save
    events = queue.Queue()
    adoption_save_started = threading.Event()
    allow_adoption_save = threading.Event()
    timer_loaded = threading.Event()
    errors = []

    @contextmanager
    def tracked_lock(state_dir):
        if threading.current_thread().name == "timer":
            events.put("timer_lock_attempt")
        if original_lock is None:
            yield
        else:
            with original_lock(state_dir):
                yield

    def tracked_load(path):
        if threading.current_thread().name == "timer":
            timer_loaded.set()
            events.put("timer_loaded")
        return original_load(path)

    def tracked_save(path, record):
        if threading.current_thread().name == "adoption" and path == cfg.state_file:
            adoption_save_started.set()
            if not allow_adoption_save.wait(5):
                raise TimeoutError("test did not release adoption save")
        return original_save(path, record)

    monkeypatch.setattr(st, "write_lock", tracked_lock, raising=False)
    monkeypatch.setattr(st, "load", tracked_load)
    monkeypatch.setattr(st, "save", tracked_save)

    def run_adoption():
        try:
            _machine(cfg, deps).adopt(world.release["release"], apply=True)
        except Exception as exc:
            errors.append(exc)

    def run_tick():
        try:
            tick(cfg, deps)
        except Exception as exc:
            errors.append(exc)
        finally:
            events.put("timer_finished")

    adoption = threading.Thread(target=run_adoption, name="adoption")
    timer = threading.Thread(target=run_tick, name="timer")
    adoption.start()
    assert adoption_save_started.wait(5)
    timer.start()
    try:
        assert events.get(timeout=5) == "timer_lock_attempt"
        assert not timer_loaded.is_set()
    finally:
        allow_adoption_save.set()
        adoption.join(5)
        timer.join(5)
    assert not adoption.is_alive() and not timer.is_alive()
    assert errors == []
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["current"] == world.release["release"]


def test_state_lock_is_released_after_exception(tmp_path):
    import threading

    lock = getattr(st, "write_lock", None)
    assert lock is not None
    acquired = threading.Event()

    with pytest.raises(RuntimeError, match="boom"):
        with lock(tmp_path / "state"):
            raise RuntimeError("boom")

    def acquire_again():
        with lock(tmp_path / "state"):
            acquired.set()

    thread = threading.Thread(target=acquire_again)
    thread.start()
    assert acquired.wait(5)
    thread.join(5)
    assert not thread.is_alive()


def test_adoption_dry_run_has_no_lock_or_state_side_effects(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)

    rec = _machine(cfg, deps).adopt(world.release["release"])

    assert rec["state"] == "adoption_planned"
    assert not cfg.state_dir.exists()
    assert not (cfg.state_dir / ".lock").exists()


def test_legacy_compare_streams_large_files(tmp_path, world, monkeypatch):
    from pathlib import Path

    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    relative = Path("docker/vision/docker-compose.yaml")
    staged = cfg.releases_root / world.release["release"] / cfg.node / relative
    legacy = cfg.legacy_root / relative
    for path in (staged, legacy):
        with path.open("r+b") as fh:
            fh.truncate(8 * 1024 * 1024)
    read_sizes = []
    original_open = Path.open

    class TrackingReader:
        def __init__(self, fh):
            self.fh = fh

        def read(self, size=-1):
            read_sizes.append(size)
            return self.fh.read(size)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.fh.close()

    def tracked_open(self, *args, **kwargs):
        mode = args[0] if args else kwargs.get("mode", "r")
        fh = original_open(self, *args, **kwargs)
        return TrackingReader(fh) if self in (staged, legacy) and mode == "rb" else fh

    monkeypatch.setattr(Path, "open", tracked_open)
    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "adopted"
    assert read_sizes
    assert all(0 < size <= 1024 * 1024 for size in read_sizes)


def test_legacy_read_oserror_is_structured_blocked(tmp_path, world, monkeypatch):
    from pathlib import Path

    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _remove_external_images(world)
    _stage(cfg, deps, world.release)
    _match_legacy_and_images(cfg, world, docker)
    legacy = cfg.legacy_root / "docker" / "vision" / "docker-compose.yaml"
    original_open = Path.open

    def failing_open(self, *args, **kwargs):
        mode = args[0] if args else kwargs.get("mode", "r")
        if self == legacy and mode == "rb":
            raise OSError("simulated read failure")
        return original_open(self, *args, **kwargs)

    monkeypatch.setattr(Path, "open", failing_open)
    rec = _machine(cfg, deps).adopt(world.release["release"], apply=True)

    assert rec["state"] == "blocked" and rec["reason"] == "legacy_read_failed"
    assert rec["detail"]["path"] == "docker/vision/docker-compose.yaml"
    assert "simulated read failure" in rec["detail"]["error"]
    assert not cfg.state_file.exists()


def test_activation_hotpatch_probe_failure_is_blocked(tmp_path, world):
    import subprocess

    from chuangong.stage import DockerCli

    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")
    cli = DockerCli()

    def failed_hotpatch_probe(*args, timeout=30.0):
        if args[:2] == ("image", "inspect"):
            return subprocess.CompletedProcess(args, 0, "sha256:managed\n", "")
        if args == ("ps", "--format", "{{.Names}}"):
            return subprocess.CompletedProcess(args, 1, "", "permission denied")
        if args == ("ps", "--format", "{{.Image}}"):
            return subprocess.CompletedProcess(args, 1, "", "permission denied")
        raise AssertionError(args)

    cli._run = failed_hotpatch_probe
    deps.docker = cli

    rec = _machine(cfg, deps).activate(world.release["release"])

    assert rec["state"] == "blocked" and rec["reason"] == "dirty_probe_failed"
    assert rec["dirty"] is True
    assert "permission denied" in str(rec["detail"])
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["current"] == "2026.11.08-1"
