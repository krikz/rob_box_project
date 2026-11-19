"""План и исполнитель переключения одной роли по digest.

Слот 3 фазы C (ADR-0168): ``plan_role`` строит шаги compose (digest, не тег) и
проверок из L: Deploy and Verify; ``execute_role`` прогоняет actions через
``docker.compose_*`` (реальный DockerCli или фейк, который пишет команды).
Штатный activate dry-run зовёт ``plan_role`` и кладёт шаги в план — список
сервисов/actions здесь один, не дублируется в activation.

switch_current двигает указатели через pointers.advance; после compose_up —
ROLE_CHECKS на docker; host_migrations пока только в журнале. Живой compose
на роботе — следующий шаг.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable

from . import pointers
from . import release_format as rf

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


def _compose_up(docker: Any, project_dir: Path, images: dict[str, str], image_tag: str | None) -> None:
    """DockerCli принимает image_tag (digest-env); FakeDocker — только images (TypeError → fallback)."""
    if image_tag is not None:
        try:
            docker.compose_up(project_dir, images=images, image_tag=image_tag)
            return
        except TypeError:
            pass
    docker.compose_up(project_dir, images=images)


def _apply_step(
    docker: Any,
    project_dir: Path,
    step: dict[str, Any],
    commands: list[dict[str, Any]],
    state_dir: Path,
) -> None:
    """Прогнать actions шага. compose_* — docker; switch_current — pointers.advance."""
    images = dict(step.get("images") or {})
    migrations = list(step.get("migrations") or [])
    image_tag = step.get("image_tag") if isinstance(step.get("image_tag"), str) else None
    for action in step["actions"]:
        if action == "compose_down":
            docker.compose_down(project_dir)
            commands.append({"action": "compose_down", "dir": str(project_dir)})
        elif action == "compose_up":
            _compose_up(docker, project_dir, images, image_tag)
            commands.append({"action": "compose_up", "dir": str(project_dir), "images": images})
        elif action == "switch_current":
            advanced = pointers.advance(state_dir, step["release"])
            commands.append(
                {
                    "action": "switch_current",
                    "release": step["release"],
                    "current": advanced["current"],
                    "previous": advanced["previous"],
                }
            )
        elif action == "host_migrations":
            commands.append({"action": "host_migrations", "migrations": migrations})
        else:
            raise RoleSwitchError("unknown_action", action)


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


def execute_role(
    release: dict[str, Any],
    role: str,
    *,
    docker: Any,
    project_dir: Path | str,
    state_dir: Path | str,
    previous: str | None = None,
    health_after: Callable[[], HealthResult] | HealthResult | None = None,
    previous_release: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Исполнить plan_role: compose_*, pointers.advance, ROLE_CHECKS.

    ``state_dir`` — каталог указателей current/previous (передаёт вызывающий;
    путь /var/lib на хосте теста не подставляется). После actions шага —
    ``docker.check_*`` из ROLE_CHECKS (FakeDocker пишет вызов). Fail проверки →
    ``blocked`` / ``role_check_failed``, не state успеха. ``health_after`` —
    доп. проверка после checks (callable или HealthResult). unhealthy → план
    отката роли и его actions (не «успех»). Падение advance не глотается.
    Живой compose на роботе этим PR не гоняем: тесты — FakeDocker.
    """
    project_dir = Path(project_dir)
    state_dir = Path(state_dir)
    planned = plan_role(release, role, previous=previous)
    if planned["state"] != "role_switch_planned":
        return planned

    commands: list[dict[str, Any]] = []
    for step in planned["plan"]:
        _apply_step(docker, project_dir, step, commands, state_dir)
        step["execute"] = True
        check_fail = _run_role_checks(docker, project_dir, step, commands)
        if check_fail is not None:
            result = dict(planned)
            result["execute"] = True
            result["commands"] = commands
            result["plan"] = [dict(s) for s in planned["plan"]]
            result["state"] = "blocked"
            result["reason"] = "role_check_failed"
            result["detail"] = check_fail
            return result

    result = dict(planned)
    result["execute"] = True
    result["commands"] = commands
    result["plan"] = [dict(step) for step in planned["plan"]]

    if health_after is None:
        return result

    checked = health_after() if callable(health_after) else health_after
    if not isinstance(checked, HealthResult):
        raise RoleSwitchError("bad_health_after", type(checked).__name__)
    result["health"] = asdict(checked)

    if checked.status != "unhealthy":
        if checked.status not in ("not_verified", "healthy"):
            result["state"] = "blocked"
            result["reason"] = "health_check_not_verified"
            result["detail"] = asdict(checked)
            return result
        result["state"] = "role_switched"
        return result

    # Откат поднимает прежний бандл по его digest, не каталог упавшего релиза (#4917).
    # План отката — от упавшего релиза (previous = его имя), а поднимается бандл
    # прежнего релиза по его digest, не каталог упавшего (#4917).
    rollback = plan_role(release, role, previous=previous, health=checked)
    rollback_dir = project_dir
    rollback_images = None
    if previous_release is not None:
        rollback_dir = project_dir.parent.parent / previous_release["release"] / "docker" / role
        rollback_images = _digest_images(role, _role_entry(previous_release, role))
    for step in rollback["plan"]:
        if rollback_images is not None:
            step["images"] = rollback_images
        _apply_step(docker, rollback_dir, step, commands, state_dir)
        step["execute"] = True
        # Откат: checks шага тоже гоняем; fail → blocked, не «успех на previous».
        check_fail = _run_role_checks(docker, project_dir, step, commands)
        if check_fail is not None:
            out = dict(rollback)
            out["execute"] = True
            out["commands"] = commands
            out["plan"] = [dict(s) for s in rollback["plan"]]
            out["state"] = "blocked"
            out["reason"] = "role_check_failed"
            out["detail"] = check_fail
            return out
    out = dict(rollback)
    out["execute"] = True
    out["commands"] = commands
    out["plan"] = [dict(step) for step in rollback["plan"]]
    return out
