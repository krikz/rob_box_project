"""C1: adoption доказанного current и детерминированный dry-run activation/rollback.

Здесь нет compose down/up, смены symlink, миграций или анимации. Только явный `adopt(apply=True)` меняет
state.json: он записывает current/previous после сравнения legacy-файлов и точной compose-матрицы
service → container → image ID с подписанным релизом. Activation и rollback возвращают план с `execute=false`,
health остаётся `not_verified`, история не меняется.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
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
        return HealthResult("not_verified", f"health not executed in dry-run: {role}/{release}")


def _blocked(record: dict[str, Any], reason: str, detail: Any) -> dict[str, Any]:
    record["state"] = st.State.BLOCKED.value
    record["reason"] = reason
    record["detail"] = detail
    return record


class LegacyReadError(RuntimeError):
    def __init__(self, path: str, error: str) -> None:
        super().__init__(f"{path}: {error}")
        self.path = path
        self.error = error


def _files_equal(first: Path, second: Path) -> bool:
    with first.open("rb") as left, second.open("rb") as right:
        while True:
            left_chunk = left.read(1024 * 1024)
            right_chunk = right.read(1024 * 1024)
            if left_chunk != right_chunk:
                return False
            if not left_chunk:
                return True


def _legacy_mismatches(staged: Path, legacy_root: Path) -> list[dict[str, str]]:
    mismatches = []
    for path in sorted(staged.rglob("*")):
        if not path.is_file() or path.name == MARKER:
            continue
        relative = path.relative_to(staged)
        legacy = legacy_root / relative
        try:
            if not legacy.is_file():
                mismatches.append({"path": relative.as_posix(), "detail": "missing"})
            elif not _files_equal(path, legacy):
                mismatches.append({"path": relative.as_posix(), "detail": "content_differs"})
        except OSError as exc:
            raise LegacyReadError(relative.as_posix(), str(exc)) from exc
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
        if apply:
            with st.write_lock(self.cfg.state_dir):
                return self._adopt(release_name, apply=True)
        return self._adopt(release_name, apply=False)

    def _adopt(self, release_name: str, apply: bool) -> dict[str, Any]:
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
        compose_dir = self.cfg.legacy_root / "docker" / self.cfg.node
        overrides = sorted(
            path.relative_to(self.cfg.legacy_root).as_posix()
            for pattern in ("*override*.yaml", "*override*.yml")
            for path in compose_dir.glob(pattern)
            if path.is_file()
        )
        if overrides:
            return _blocked(record, "compose_override_present", overrides)
        try:
            mismatches = _legacy_mismatches(staged, self.cfg.legacy_root)
        except LegacyReadError as exc:
            return _blocked(record, "legacy_read_failed", {"path": exc.path, "error": exc.error})
        if mismatches:
            return _blocked(record, "legacy_bundle_mismatch", mismatches)

        image_ids = {
            service: self.deps.docker.image_id(ref)
            for service, ref in release["roles"][self.cfg.node]["images"].items()
        }
        if any(image_id is None for image_id in image_ids.values()):
            return _blocked(record, "release_image_id_unavailable", image_ids)
        try:
            running = self.deps.docker.running_compose_image_ids()
        except StageError as exc:
            return _blocked(
                record,
                "compose_probe_failed",
                {
                    "service": exc.extra.get("service", "unknown"),
                    "container": exc.extra.get("container", "unknown"),
                    "error": exc.extra.get("error", exc.detail),
                },
            )
        if set(running) != set(image_ids) or any(
            running.get(service) != [image_id] for service, image_id in image_ids.items()
        ):
            return _blocked(
                record,
                "compose_services_mismatch",
                {"expected_services": image_ids, "running_services": running},
            )

        evidence = {
            "staged": str(staged),
            "legacy_root": str(self.cfg.legacy_root),
            "image_ids": image_ids,
            "compose_services": running,
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

    @staticmethod
    def _created(release: dict[str, Any]) -> datetime:
        raw = release.get("created")
        if not isinstance(raw, str):
            raise ValueError("created отсутствует в подписанном release.yaml")
        created = datetime.fromisoformat(raw.replace("Z", "+00:00"))
        if created.tzinfo is None:
            raise ValueError(f"created без часового пояса: {raw}")
        return created

    def _compat_block(self, record: dict[str, Any], release: dict[str, Any]) -> dict[str, Any] | None:
        minimum = release["compat"]["min_previous"]
        current = record["current"]
        if minimum is None or current == minimum:
            return None
        detail = {"current": current, "min_previous": minimum}
        try:
            current_release = fetch_release(self.cfg, self.deps.registry, ref=current)
            minimum_release = fetch_release(self.cfg, self.deps.registry, ref=minimum)
            if current_release is None or minimum_release is None:
                raise ValueError("подписанный манифест current/min_previous недоступен")
            current_created = self._created(current_release)
            minimum_created = self._created(minimum_release)
        except (FetchError, ValueError) as exc:
            return _blocked(record, "compat_unproven", {**detail, "error": str(exc)})
        if current_created < minimum_created:
            return _blocked(record, "compat_min_previous", detail)
        return None

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
        compat_block = self._compat_block(record, release)
        if compat_block is not None:
            return compat_block, None
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
        if health.status == "unhealthy":
            return _blocked(record, "health_check_failed", health.detail)
        if health.status != "not_verified":
            return _blocked(record, "health_check_not_verified", asdict(health))
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
