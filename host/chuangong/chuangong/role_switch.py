"""План переключения одной роли по digest, без compose up.

Слот 3 фазы C (ADR-0168): локальный исполнитель одной роли строит план шагов
compose (digest, не тег) и проверок из L: Deploy and Verify. Живой стек,
symlink и docker не трогает — только чистая функция плана.

Имена actions согласованы с activation._planned; имена checks — со скриптами
deploy: check_container_status, check_container_logs, check_ros2_topics.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from . import release_format as rf

# Как в activation._planned (vision/sound/main → actions).
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
            raise RoleSwitchError(
                "image_not_digest", f"roles.{role}.images.{service}: {ref!r}"
            )
        pinned[str(service)] = ref
    return pinned


def _role_entry(release: dict[str, Any], role: str) -> dict[str, Any]:
    if role not in rf.ROLES:
        raise RoleSwitchError(
            "bad_role", f"{role!r}: ожидалось одно из {list(rf.ROLES)}"
        )
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
) -> dict[str, Any]:
    return {
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
    health = health or HealthResult(
        "not_verified", f"health not executed: {role}/{target}"
    )
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

    base.update(
        {
            "state": "role_switch_planned",
            "plan": [_switch_step(role, target, images, migrations)],
        }
    )
    return base
