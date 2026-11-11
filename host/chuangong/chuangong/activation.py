"""C1: adoption доказанного current и детерминированный dry-run activation/rollback.

Здесь нет compose down/up, смены symlink, миграций или анимации. Только явный `adopt(apply=True)` меняет
state.json: он записывает current/previous после сравнения legacy-файлов и запущенных image ID с подписанным
релизом. Activation и rollback возвращают план с `execute=false`, не меняя историю.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol

from . import state as st
from .agent import Config, Deps, FetchError, fetch_release, probe_flags, report
from .stage import MARKER, RoleStager, StageError

ACTIVATION_ORDER = ("vision", "sound", "main")


@dataclass(frozen=True)
class HealthResult:
    status: str
    detail: str


class HealthCheck(Protocol):
    def check(self, role: str, release: str, dry_run: bool) -> HealthResult: ...


class DryRunHealthCheck:
    def check(self, role: str, release: str, dry_run: bool) -> HealthResult:
        return HealthResult("planned", f"health-check {role}/{release}: dry_run={dry_run}")


def _blocked(record: dict[str, Any], reason: str, detail: Any) -> dict[str, Any]:
    record["state"] = st.State.BLOCKED.value
    record["reason"] = reason
    record["detail"] = detail
    return record


def _legacy_mismatches(staged: Path, legacy_root: Path) -> list[dict[str, str]]:
    mismatches = []
    for path in sorted(staged.rglob("*")):
        if not path.is_file() or path.name == MARKER:
            continue
        relative = path.relative_to(staged)
        legacy = legacy_root / relative
        if not legacy.is_file():
            mismatches.append({"path": relative.as_posix(), "detail": "missing"})
        elif path.read_bytes() != legacy.read_bytes():
            mismatches.append({"path": relative.as_posix(), "detail": "content_differs"})
    return mismatches


class ActivationMachine:
    def __init__(self, cfg: Config, deps: Deps, health: HealthCheck | None = None) -> None:
        self.cfg = cfg
        self.deps = deps
        self.health = health or DryRunHealthCheck()

    def _record(self, dry_run: bool = True) -> dict[str, Any]:
        prev = st.load(self.cfg.state_file)
        record = st.new_record(self.cfg.robot_id, self.cfg.node, self.cfg.channel, self.cfg.mode, prev)
        record["dry_run"] = dry_run
        probe_flags(self.cfg, self.deps.docker, record)
        return record

    def _stager(self) -> RoleStager:
        return RoleStager(
            self.deps.registry,
            self.deps.docker,
            self.cfg.releases_root,
            self.cfg.docker_root,
            self.cfg.reserve_bytes,
            self.deps.disk_free,
        )

    def _release(self, record: dict[str, Any], name: str) -> dict[str, Any] | None:
        try:
            release = fetch_release(self.cfg, self.deps.registry, ref=name)
        except FetchError as exc:
            record["state"] = st.State.FETCH_ERROR.value
            record["reason"] = exc.reason
            record["detail"] = exc.detail
            return None
        record["available"] = name
        record["release_git"] = release.get("git")
        return release

    def _preflight(self, record: dict[str, Any], release: dict[str, Any]) -> tuple[RoleStager, dict[str, Any]] | None:
        stager = self._stager()
        plan = stager.plan(release, self.cfg.node)
        if not plan["bundle_done"] or plan["images_missing"]:
            _blocked(record, "release_not_staged", plan)
            return None
        if record["hil_locked"]:
            _blocked(record, "hil_locked", str(self.cfg.hil_lock))
            return None
        if record["dirty"]:
            _blocked(record, "dirty", record["dirty_sources"])
            return None
        try:
            stager.check_space(release, self.cfg.node, plan)
        except StageError as exc:
            _blocked(record, exc.reason, exc.extra.get("disk", exc.detail))
            return None
        return stager, plan

    def adopt(self, release_name: str, apply: bool = False) -> dict[str, Any]:
        if apply and self.cfg.dry_run:
            raise ValueError("adopt(apply=True) несовместим с Config.dry_run")
        record = self._record(dry_run=not apply)
        release = self._release(record, release_name)
        if release is None:
            return record
        if self._preflight(record, release) is None:
            return record
        external = release["roles"][self.cfg.node]["external_images"]
        if external:
            return _blocked(record, "external_images_unpinned", {"node": self.cfg.node, "external_images": external})
        current = record["current"]
        if current not in (None, release_name):
            return _blocked(
                record,
                "current_conflict",
                {"current": current, "previous": record["previous"], "requested": release_name},
            )
        staged = self.cfg.releases_root / release_name / self.cfg.node
        mismatches = _legacy_mismatches(staged, self.cfg.legacy_root)
        if mismatches:
            return _blocked(record, "legacy_bundle_mismatch", mismatches)

        image_ids = {
            service: self.deps.docker.image_id(ref)
            for service, ref in release["roles"][self.cfg.node]["images"].items()
        }
        expected = sorted(image_id for image_id in image_ids.values() if image_id)
        if len(expected) != len(image_ids):
            return _blocked(record, "release_image_id_unavailable", image_ids)
        running = self.deps.docker.running_image_ids()
        missing = sorted(set(expected) - set(running))
        if missing:
            return _blocked(
                record,
                "release_images_not_running",
                {"expected_image_ids": expected, "running_image_ids": running, "missing_image_ids": missing},
            )

        evidence = {
            "staged": str(staged),
            "legacy_root": str(self.cfg.legacy_root),
            "image_ids": image_ids,
            "running_image_ids": running,
        }
        record["adoption"] = {"release": release_name, "git": release.get("git"), "evidence": evidence}
        previous = record["previous"] if current == release_name else None
        if not apply:
            record["state"] = st.State.ADOPTION_PLANNED.value
            record["would"] = {"current": release_name, "previous": previous}
            return record
        record["current"] = release_name
        record["previous"] = previous
        record["state"] = st.State.ADOPTED.value
        return report(self.cfg, self.deps, record)

    def _prepare_activation(self, release_name: str) -> tuple[dict[str, Any], dict[str, Any] | None]:
        record = self._record()
        release = self._release(record, release_name)
        if release is None:
            return record, None
        if self._preflight(record, release) is None:
            return record, None
        history = {"current": record["current"], "previous": record["previous"]}
        if record["current"] is None:
            return _blocked(record, "current_unproven", history), None
        if record["current"] == release_name:
            return _blocked(record, "already_current", history), None
        return record, release

    def preflight(self, release_name: str) -> dict[str, Any]:
        record, release = self._prepare_activation(release_name)
        if release is None:
            return record
        record["preflight"] = {
            "release": release_name,
            "staged": True,
            "hil_locked": bool(record["hil_locked"]),
            "dirty": record["dirty"],
            "current": record["current"],
            "previous": record["previous"],
        }
        record["state"] = st.State.PREFLIGHT_PASSED.value
        return record

    def activate(self, release_name: str) -> dict[str, Any]:
        record, release = self._prepare_activation(release_name)
        if release is None:
            return record
        return self._planned(record, release, st.State.ACTIVATION_PLANNED)

    def rollback(self) -> dict[str, Any]:
        record = self._record()
        history = {"current": record["current"], "previous": record["previous"]}
        if record["current"] is None:
            return _blocked(record, "current_unproven", history)
        if record["previous"] is None:
            return _blocked(record, "no_previous", history)
        release = self._release(record, record["previous"])
        if release is None:
            return record
        if self._preflight(record, release) is None:
            return record
        return self._planned(record, release, st.State.ROLLBACK_PLANNED)

    def _planned(self, record: dict[str, Any], release: dict[str, Any], state: st.State) -> dict[str, Any]:
        target = release["release"]
        health = self.health.check(self.cfg.node, target, dry_run=True)
        record["health"] = asdict(health)
        if health.status not in ("planned", "healthy"):
            return _blocked(record, "health_check_failed", health.detail)
        record["plan"] = [
            {
                "op": "activate",
                "node": node,
                "release": target,
                "execute": False,
                "actions": ["compose_down", "switch_current", "host_migrations", "compose_up"],
                "migrations": release["roles"][node]["migrations"],
            }
            for node in ACTIVATION_ORDER
        ]
        record["would"] = {"current": target, "previous": record["current"]}
        record["state"] = state.value
        return record
