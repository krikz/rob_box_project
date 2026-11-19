"""Offline acceptance of one Chuangong role switch and recovery."""

from pathlib import Path
from unittest.mock import Mock

import pytest
from chuangong import state as st
from chuangong.role_switch import HealthResult, execute_role
from chuangong.stage import MARKER, StageError
from fakes import FakeDocker
from test_role_switch import _release_fixture


class RoleDocker(FakeDocker):
    def __init__(self, target: Path, images: set[str]) -> None:
        super().__init__(present=images)
        self.target = target
        self.start_error = None
        self.errors: dict[Path, str] = {}
        self.tags: list[str] = []
        self.image_ids = {ref: ref.rsplit("@", 1)[1] for ref in images}
        self.wrong_images: set[Path] = set()

    def set_running(self, images):
        self.compose_images = {service: [self.image_ids[ref]] for service, ref in images.items()}

    def compose_up(self, project_dir, images=None, *, image_tag=None):
        super().compose_up(project_dir, images)
        self.tags.append(image_tag)
        self.set_running(images)
        if project_dir in self.wrong_images:
            for service in self.compose_images:
                self.compose_images[service] = ["sha256:" + "f" * 64]
        if project_dir == self.target and self.start_error is not None:
            raise StageError("compose_up_failed", self.start_error)

    def check_container_status(self, project_dir):
        super().check_container_status(project_dir)
        return self.errors.get(project_dir)


@pytest.fixture
def switch(tmp_path):
    state_dir = tmp_path / "state"
    releases_root = tmp_path / "releases"
    active, target = _release_fixture("2026.11.09-1"), _release_fixture("2026.11.10-1")
    active["roles"]["main"]["images"]["main-svc0"] = "registry.example/krikz/rob_box@sha256:" + "e" * 64
    active["image_tag"] = "dev-oldtag"
    projects = []
    for manifest in (active, target):
        bundle = releases_root / manifest["release"] / "main"
        project_dir = bundle / "docker" / "main"
        project_dir.mkdir(parents=True)
        (project_dir.parent / "build-manifest.yaml").touch()
        (project_dir / "docker-compose.yaml").write_text("services: {}\n", encoding="utf-8")
        (bundle / MARKER).write_text(manifest["roles"]["main"]["bundle"]["sha256"], encoding="utf-8")
        projects.append(project_dir)
    state_file = state_dir / "state.json"
    initial = {
        "node": "main",
        "channel": "dev",
        "current": active["release"],
        "previous": "2026.11.08-1",
        "staged": target["release"],
    }
    st.save(state_file, initial)
    images = set(active["roles"]["main"]["images"].values()) | set(target["roles"]["main"]["images"].values())
    docker = RoleDocker(projects[1], images)
    docker.set_running(active["roles"]["main"]["images"])
    options = {
        "docker": docker,
        "releases_root": releases_root,
        "state_dir": state_dir,
        "current_release": active,
        "hil_lock": tmp_path / "hil.lock",
        "health_check": lambda _project_dir: HealthResult("healthy"),
    }
    return target, state_file, initial, projects, options


@pytest.mark.parametrize("other_staged", (False, True))
def test_verified_role_commits_current_after_checks(switch, other_staged):
    target, state_file, initial, (active_dir, target_dir), options = switch
    checked = []
    if other_staged:
        initial = {**initial, "staged": "2026.11.11-1"}
        st.save(state_file, initial)

    def healthy(project_dir):
        checked.append((project_dir, st.load(state_file)))
        return HealthResult("healthy")

    result = execute_role(target, "main", **{**options, "health_check": healthy})

    assert result["state"] == "role_switched"
    assert checked == [(target_dir, initial)]
    assert st.load(state_file) == {
        **initial,
        "current": target["release"],
        "previous": initial["current"],
        "staged": initial["staged"] if other_staged else None,
    }
    assert options["docker"].commands[:2] == [
        ("compose_down", str(active_dir)),
        ("compose_up", str(target_dir), target["roles"]["main"]["images"]),
    ]
    assert result["commands"][-1]["action"] == "switch_current"
    assert result["plan"][0]["actions"] == ["compose_down", "compose_up", "switch_current"]


@pytest.mark.parametrize("failure", ("start", "role_check", "running_digest", "unverified", "commit"))
def test_failed_target_restores_exact_current_and_checks_it(switch, failure, monkeypatch):
    target, state_file, initial, (active_dir, target_dir), options = switch
    docker = options["docker"]
    if failure == "start":
        docker.start_error = "compose returned error"
    elif failure == "role_check":
        docker.errors[target_dir] = "container restarting"
    elif failure == "running_digest":
        docker.wrong_images.add(target_dir)
    elif failure == "unverified":
        options["health_check"] = lambda project_dir: HealthResult(
            "not_verified" if project_dir == target_dir else "healthy"
        )
    else:
        monkeypatch.setattr(st, "save", Mock(side_effect=OSError("state filesystem is read-only")))

    result = execute_role(target, "main", **options)

    assert result["state"] == "role_rolled_back"
    assert result["failed_release"] == target["release"]
    assert result["rollback_health"]["status"] == "healthy"
    assert "switch_current" not in result["plan"][0]["actions"]
    assert result["plan"][1]["actions"] == ["compose_down", "compose_up"]
    assert st.load(state_file) == initial
    source = options["current_release"]
    assert [c for c in docker.commands if c[0] == "compose_up"] == [
        ("compose_up", str(target_dir), target["roles"]["main"]["images"]),
        ("compose_up", str(active_dir), source["roles"]["main"]["images"]),
    ]
    assert docker.tags == [target["image_tag"], source["image_tag"]]
    assert ("check_container_status", str(active_dir)) in docker.commands
    assert ("check_container_logs", str(active_dir)) in docker.commands


@pytest.mark.parametrize("rollback_failure", ("role_check", "health"))
def test_failed_recovery_is_reported_instead_of_success(switch, rollback_failure):
    target, state_file, initial, (active_dir, target_dir), options = switch
    options["docker"].errors = {target_dir: "target unhealthy"}
    if rollback_failure == "role_check":
        options["docker"].errors[active_dir] = "old unhealthy"
    else:
        options["health_check"] = lambda _project_dir: HealthResult("not_verified", "old unhealthy")

    result = execute_role(target, "main", **options)

    assert result["state"] == "rollback_failed"
    assert "target unhealthy" in result["detail"]
    assert "old unhealthy" in result["rollback_error"]
    assert st.load(state_file) == initial


@pytest.mark.parametrize(
    "obstacle",
    (
        "hil",
        "restored_bundle_missing",
        "restore_image_missing",
        "restore_manifest_missing",
        "no_health",
        "no_current",
        "running_drift",
        "migration",
    ),
)
def test_switch_stops_before_docker_changes_if_preflight_fails(switch, obstacle):
    target, state_file, initial, (active_dir, _), options = switch
    if obstacle == "hil":
        options["hil_lock"].touch()
    elif obstacle == "restored_bundle_missing":
        (active_dir.parents[1] / MARKER).unlink()
    elif obstacle == "restore_image_missing":
        options["docker"].present.remove(options["current_release"]["roles"]["main"]["images"]["main-svc0"])
    elif obstacle == "restore_manifest_missing":
        (active_dir.parent / "build-manifest.yaml").unlink()
    elif obstacle == "no_health":
        options["health_check"] = None
    elif obstacle == "no_current":
        st.save(state_file, {**initial, "current": None})
    elif obstacle == "running_drift":
        options["docker"].compose_images.clear()
    else:
        target["roles"]["main"]["migrations"] = ["scripts/setup/migration.sh"]

    result = execute_role(target, "main", **options)

    assert result["state"] == "blocked"
    assert result["execute"] is False
    assert options["docker"].commands == []
