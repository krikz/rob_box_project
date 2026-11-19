"""План и офлайн-исполнитель переключения одной роли по digest.

plan_role остаётся источником шагов для dry-run. execute_role принимает два
проверенных подписанных манифеста и работает через внедрённый адаптер Compose
и проверок. Штатный пульт пока не вызывает исполнитель.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable

from . import cancel
from . import release_format as rf
from . import state as st
from .stage import StageError, bundle_staged

# Единственный список actions роли (activation._planned берёт шаги из plan_role).
ROLE_ACTIONS = ("compose_down", "switch_current", "host_migrations", "compose_up")

# Имена из L: Deploy and Verify (scripts/ci + шаги топиков).
CHECK_CONTAINER_STATUS = "check_container_status"
CHECK_CONTAINER_LOGS = "check_container_logs"
CHECK_ROS2_TOPICS = "check_ros2_topics"
ROLE_CHECKS = (CHECK_CONTAINER_STATUS, CHECK_CONTAINER_LOGS, CHECK_ROS2_TOPICS)

# Ожидаемые топики — ровно те, что названы в L: Deploy and Verify.
# main в workflow считает число топиков, без фиксированного списка имён.
ROLE_EXPECTED_TOPICS: dict[str, tuple[str, ...]] = {
    "vision": ("/camera/camera/color/image_raw/compressed",),
    "sound": ("/voice/stt/utterance",),
    "main": (),
}


@dataclass(frozen=True)
class HealthResult:
    """Результат health-проверки роли. Не исполняет проверки — только значение для плана."""

    status: str
    detail: str = ""


class RoleSwitchError(ValueError):
    """План роли нельзя построить: роль/релиз/образы не сходятся со схемой."""

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


def _digest_images(role: str, entry: dict[str, Any]) -> dict[str, str]:
    """Образы роли только по digest (@sha256:…); тег канала/сборки в план не попадает."""
    images = entry.get("images")
    if not isinstance(images, dict) or not images:
        raise RoleSwitchError("images_missing", f"roles.{role}.images")
    pinned: dict[str, str] = {}
    for service, ref in images.items():
        if not isinstance(ref, str) or "@sha256:" not in ref:
            raise RoleSwitchError("image_not_digest", f"roles.{role}.images.{service}: {ref!r}")
        pinned[str(service)] = ref
    return pinned


def _role_entry(release: dict[str, Any], role: str) -> dict[str, Any]:
    if role not in rf.ROLES:
        raise RoleSwitchError("bad_role", f"{role!r}: ожидалось одно из {list(rf.ROLES)}")
    try:
        rf.validate(release)
    except rf.ReleaseError as exc:
        raise RoleSwitchError("bad_release", str(exc)) from exc
    entry = release["roles"][role]
    if not isinstance(entry, dict):
        raise RoleSwitchError("bad_role_entry", f"roles.{role}")
    return entry


def _switch_step(
    role: str,
    release_name: str,
    images: dict[str, str],
    migrations: list[Any],
    image_tag: str | None = None,
) -> dict[str, Any]:
    step = {
        "op": "switch_role",
        "role": role,
        "release": release_name,
        "execute": False,
        "actions": list(ROLE_ACTIONS),
        "migrations": list(migrations),
        "images": dict(images),
        "checks": list(ROLE_CHECKS),
        "expected_topics": list(ROLE_EXPECTED_TOPICS[role]),
    }
    if image_tag is not None:
        step["image_tag"] = image_tag
    return step


def _rollback_step(role: str, previous: str, failed_release: str) -> dict[str, Any]:
    return {
        "op": "rollback",
        "role": role,
        "release": previous,
        "from_release": failed_release,
        "execute": False,
        "actions": list(ROLE_ACTIONS),
        "checks": list(ROLE_CHECKS),
        "expected_topics": list(ROLE_EXPECTED_TOPICS[role]),
    }


def plan_role(
    release: dict[str, Any],
    role: str,
    *,
    previous: str | None = None,
    health: HealthResult | None = None,
) -> dict[str, Any]:
    """План переключения одной роли: compose по digest + проверки deploy.

    Вход — подписанный (уже проверенный вызывающим) release.yaml-словарь и имя роли.
    Выход — запись плана с ``execute=false``: шаги compose (digest-refs из
    ``roles.<role>.images``) и checks из L: Deploy and Verify.

    Если ``health.status == "unhealthy"`` — план содержит откат на ``previous``,
    а не «успех». Без ``previous`` при провале health — ``blocked`` / ``no_previous``.
    Живой compose up/down не вызывает.
    """
    entry = _role_entry(release, role)
    target = release["release"]
    images = _digest_images(role, entry)
    migrations = list(entry.get("migrations") or [])
    health = health or HealthResult("not_verified", f"health not executed: {role}/{target}")
    health_rec = asdict(health)

    base: dict[str, Any] = {
        "role": role,
        "release": target,
        "previous": previous,
        "execute": False,
        "health": health_rec,
        "images": images,
        "migrations": migrations,
        "checks": list(ROLE_CHECKS),
        "expected_topics": list(ROLE_EXPECTED_TOPICS[role]),
    }

    if health.status == "unhealthy":
        if previous is None:
            base.update(
                {
                    "state": "blocked",
                    "reason": "no_previous",
                    "detail": health.detail,
                    "plan": [],
                }
            )
            return base
        if previous == target:
            base.update(
                {
                    "state": "blocked",
                    "reason": "health_check_failed",
                    "detail": {"previous": previous, "health": health_rec},
                    "plan": [],
                }
            )
            return base
        try:
            rf.check_release_name(previous)
        except rf.ReleaseError as exc:
            raise RoleSwitchError("bad_previous", str(exc)) from exc
        base.update(
            {
                "state": "rollback_planned",
                "reason": "health_check_failed",
                "detail": health.detail,
                "failed_release": target,
                "release": previous,
                "plan": [_rollback_step(role, previous, target)],
            }
        )
        # Образы failed-релиза не подставляем как цель отката — digests previous
        # здесь неизвестны (вход — один signed release). checks остаются.
        base["images"] = {}
        base["migrations"] = []
        return base

    if health.status not in ("not_verified", "healthy"):
        base.update(
            {
                "state": "blocked",
                "reason": "health_check_not_verified",
                "detail": health_rec,
                "plan": [],
            }
        )
        return base

    image_tag = release.get("image_tag")
    image_tag_s = image_tag.strip() if isinstance(image_tag, str) else None
    base.update(
        {
            "state": "role_switch_planned",
            "plan": [_switch_step(role, target, images, migrations, image_tag=image_tag_s)],
        }
    )
    return base


def _run_role_checks(
    docker: Any,
    project_dir: Path,
    step: dict[str, Any],
    commands: list[dict[str, Any]],
) -> str | None:
    """ROLE_CHECKS после compose_up: узкий протокол docker.check_* (фейк пишет вызов, без сети).

    ``check_ros2_topics`` сравнивает expected_topics шага с заранее заданным списком
    на фейке — живой ``ros2 topic`` не вызывается. None = ok; иначе detail fail.
    """
    expected = list(step.get("expected_topics") or [])
    for name in list(step.get("checks") or ROLE_CHECKS):
        if name == CHECK_CONTAINER_STATUS:
            err = docker.check_container_status(project_dir)
            entry: dict[str, Any] = {"action": name, "dir": str(project_dir)}
        elif name == CHECK_CONTAINER_LOGS:
            err = docker.check_container_logs(project_dir)
            entry = {"action": name, "dir": str(project_dir)}
        elif name == CHECK_ROS2_TOPICS:
            err = docker.check_ros2_topics(expected)
            entry = {"action": name, "expected_topics": list(expected)}
        else:
            raise RoleSwitchError("unknown_check", name)
        if err:
            entry["ok"] = False
            entry["detail"] = err
            commands.append(entry)
            return f"{name}: {err}"
        entry["ok"] = True
        commands.append(entry)
    return None


def _ready_project(release: dict[str, Any], role: str, releases_root: Path, docker: Any) -> Path:
    entry = _role_entry(release, role)
    if not isinstance(release.get("image_tag"), str) or not release["image_tag"].strip():
        raise RoleSwitchError("image_tag_missing", release["release"])
    bundle = releases_root / release["release"] / role
    project = bundle / "docker" / role
    if not bundle_staged(bundle, entry["bundle"]["sha256"]) or not (project / "docker-compose.yaml").is_file():
        raise RoleSwitchError("release_not_staged", str(project))
    if not (project.parent / "build-manifest.yaml").is_file():
        raise RoleSwitchError("build_manifest_missing", str(project.parent / "build-manifest.yaml"))
    external = entry.get("external_images", [])
    if any(not rf.is_pinned_ref(ref) for ref in external):
        raise RoleSwitchError("external_images_unpinned", release["release"])
    missing = [ref for ref in [*entry["images"].values(), *external] if not docker.image_present(ref)]
    if missing:
        raise RoleSwitchError("release_images_missing", f"{release['release']}: {missing}")
    return project


def _verify_running_images(docker: Any, images: dict[str, str]) -> None:
    """Сверить image IDs сервисов сборки из подписанного манифеста роли."""
    running = docker.running_compose_image_ids()
    for service, ref in images.items():
        expected = docker.image_id(ref)
        actual = running.get(service)
        if not expected or actual != [expected]:
            raise RoleSwitchError("running_image_mismatch", f"{service}: {actual!r}, expected {ref} ({expected})")


def _require_switch_ready(
    release: dict[str, Any],
    current_release: dict[str, Any],
    role: str,
    *,
    history: dict[str, Any] | None,
    releases_root: Path,
    state_dir: Path,
    hil_lock: Path,
    docker: Any,
) -> tuple[Path, Path]:
    current = current_release.get("release")
    target = release["release"]
    if not current or not history or history.get("current") != current:
        raise RoleSwitchError("current_unproven", str(current))
    if history.get("node") != role or history.get("channel") != release["channel"]:
        raise RoleSwitchError("role_scope_mismatch", role)
    if current_release.get("channel") != release["channel"]:
        raise RoleSwitchError("release_channel_mismatch", str(current))
    if current == target:
        raise RoleSwitchError("already_current", target)
    if cancel.is_cancelled(releases_root, target, role):
        raise RoleSwitchError("release_cancelled", target)
    unsupported = [name for name in ROLE_CHECKS if not callable(getattr(docker, name, None))]
    if unsupported:
        raise RoleSwitchError("role_checks_unavailable", ", ".join(unsupported))
    if release["roles"][role].get("migrations"):
        raise RoleSwitchError("host_migrations_unavailable", target)
    current_dir = _ready_project(current_release, role, releases_root, docker)
    target_dir = _ready_project(release, role, releases_root, docker)
    if hil_lock.exists():
        raise RoleSwitchError("hil_locked", str(hil_lock))
    if (state_dir / "dirty").exists() or docker.hotpatched_containers():
        raise RoleSwitchError("dirty", role)
    _verify_running_images(docker, current_release["roles"][role]["images"])
    return current_dir, target_dir


def _apply(
    docker: Any,
    inactive_dir: Path,
    step: dict[str, Any],
    commands: list[dict[str, Any]],
    *,
    stop_dir: Path | None,
) -> None:
    if stop_dir is not None:
        down = {"action": "compose_down", "dir": str(stop_dir), "ok": False}
        commands.append(down)
        docker.compose_down(stop_dir)
        down["ok"] = True
    up = {"action": "compose_up", "dir": str(inactive_dir), "images": step["images"], "ok": False}
    commands.append(up)
    docker.compose_up(inactive_dir, images=step["images"], image_tag=step["image_tag"])
    up["ok"] = True
    _verify_running_images(docker, step["images"])
    error = _run_role_checks(docker, inactive_dir, step, commands)
    if error:
        raise RoleSwitchError("role_check_failed", error)


def _check_health(result: dict[str, Any], field: str, check: Callable[[Path], HealthResult], project_dir: Path) -> None:
    health = check(project_dir)
    if not isinstance(health, HealthResult):
        raise RoleSwitchError("bad_health_result", type(health).__name__)
    result[field] = asdict(health)
    if health.status != "healthy":
        raise RoleSwitchError("health_check_failed", health.detail or health.status)


def _applied_actions(commands: list[dict[str, Any]]) -> list[str]:
    return [cmd["action"] for cmd in commands if cmd["action"] in ROLE_ACTIONS and cmd.get("ok", True)]


def _execute_switch(
    result: dict[str, Any],
    restore: dict[str, Any],
    current_dir: Path,
    target_dir: Path,
    *,
    docker: Any,
    history: dict[str, Any],
    state_file: Path,
    health_check: Callable[[Path], HealthResult],
) -> dict[str, Any]:
    target = result["plan"][0]
    commands: list[dict[str, Any]] = []
    result.update(execute=True, commands=commands)
    target["execute"] = True
    try:
        _apply(docker, target_dir, target, commands, stop_dir=current_dir)
        _check_health(result, "health", health_check, target_dir)
        activated = {**history, "current": target["release"], "previous": history["current"]}
        if activated.get("staged") == target["release"]:
            activated["staged"] = None
        st.save(state_file, activated)
    except Exception as exc:
        # Ошибка адаптера, проверки или записи состояния требует восстановления.
        result.update(reason=getattr(exc, "reason", "switch_failed"), detail=str(exc), failed_release=result["release"])
    else:
        commands.append({"action": "switch_current", "current": target["release"], "previous": history["current"]})
        target["actions"] = _applied_actions(commands)
        return {**result, "state": "role_switched"}

    target["actions"] = _applied_actions(commands)
    started = any(command["action"] == "compose_up" for command in commands)
    rollback_start = len(commands)
    restore.update(op="rollback", execute=True, from_release=result["release"])
    result["plan"].append(restore)
    result["restore_release"] = restore["release"]
    try:
        _apply(docker, current_dir, restore, commands, stop_dir=target_dir if started else None)
        _check_health(result, "rollback_health", health_check, current_dir)
    except Exception as exc:
        result.update(state="rollback_failed", rollback_error=str(exc))
    else:
        result.update(state="role_rolled_back", release=restore["release"], images=restore["images"])
    restore["actions"] = _applied_actions(commands[rollback_start:])
    return result


def execute_role(
    release: dict[str, Any],
    role: str,
    *,
    docker: Any,
    releases_root: Path,
    state_dir: Path,
    current_release: dict[str, Any],
    hil_lock: Path,
    health_check: Callable[[Path], HealthResult] | None = None,
) -> dict[str, Any]:
    """Переключить одну роль и обновить state.json после успешной проверки.

    Вызывающий проверяет подпись обоих манифестов и выполняет штатный preflight,
    включая блокировки всего робота. Локальный lock общий с тиком и отменой.
    При срыве бандл и digest текущего релиза уже проверены и готовы к откату.
    Проверка health вызывается с каталогом действительно поднятого Compose.
    Адаптер проверяет весь состав проекта Compose, включая внешние сервисы.
    Штатный пульт вызов не делает. Восстановление после остановки процесса
    посреди операции требует отдельного наблюдения и восстановления состояния.
    """
    planned = plan_role(release, role, previous=current_release.get("release"))
    if not callable(health_check):
        return {**planned, "state": "blocked", "reason": "health_check_unavailable"}
    state_file = state_dir / "state.json"
    with st.write_lock(state_dir):
        history = st.load(state_file)
        try:
            current_dir, target_dir = _require_switch_ready(
                release,
                current_release,
                role,
                history=history,
                releases_root=releases_root,
                state_dir=state_dir,
                hil_lock=hil_lock,
                docker=docker,
            )
        except (OSError, RoleSwitchError, StageError) as exc:
            return {
                **planned,
                "state": "blocked",
                "reason": getattr(exc, "reason", "preflight_failed"),
                "detail": str(exc),
            }

        restore = plan_role(current_release, role)["plan"][0]
        assert history is not None  # подтверждён в _require_switch_ready
        return _execute_switch(
            planned,
            restore,
            current_dir,
            target_dir,
            docker=docker,
            history=history,
            state_file=state_file,
            health_check=health_check,
        )
