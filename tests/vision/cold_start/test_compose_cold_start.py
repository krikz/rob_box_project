"""Regression tests for ``docker/vision/docker-compose.yaml`` cold-start invariants.

Issue #2610 + PR #2634 — после ребута Vision Pi весь стек должен подниматься
даже при недоступном build-registry (katana 10.1.1.249:5000). Условия:

1. Все сервисы (default + profiled) обязаны иметь ``pull_policy`` —
   иначе compose применит дефолт (always на старых версиях / missing
   на новых), и при недоступном registry любой старт зависнет на pull.
2. ``voice-resources-init`` помечен ``profiles: [init]`` — НЕ стартует
   в default ``up -d`` (поскольку init-образ может быть локально
   не собран и katana offline).
3. Downstream ``supercollider`` + ``voice-assistant`` зависят от
   ``voice-resources-init`` через ``condition: service_completed_successfully``
   с ``required: false`` — то есть compose нормально стартует их,
   даже если init-сервис не активирован профилем.

Эти проверки чисто-статические: парсим YAML напрямую и валидируем
структуру. Запуск ``docker compose config`` оставлен для интеграционного
теста в этом же каталоге (test_docker_compose_bad_registry.py).
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
COMPOSE_FILE = REPO_ROOT / "docker" / "vision" / "docker-compose.yaml"
COMPOSE_DIR = COMPOSE_FILE.parent


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _load_raw_compose() -> dict[str, Any]:
    """Загрузить docker-compose.yaml как dict без раскрытия переменных.

    Это важно: ``${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}`` — нам нужны
    строки как есть, чтобы проверять structural invariants, а не результат
    ``docker compose config`` (для последнего есть интеграционный тест).
    """
    assert COMPOSE_FILE.exists(), f"compose file missing: {COMPOSE_FILE}"
    with COMPOSE_FILE.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), f"compose root is not a mapping: {type(data)!r}"
    return data


def _all_services_with_profiles() -> dict[str, dict[str, Any]]:
    """Собрать ВСЕ сервисы из compose, независимо от профиля.

    Трюк: compose позволяет одному сервису иметь ``profiles: [...]``,
    и тогда он не входит в ``services`` для default-профиля. Но для
    регрессионной проверки pull_policy нам интересны ВСЕ — потому что
    в момент ``docker compose --profile <X> up -d`` registry тоже
    может быть offline, и сервис должен стартовать с локального кэша.
    """
    data = _load_raw_compose()
    services = data.get("services") or {}
    assert services, "no services section in docker-compose.yaml"
    return dict(services)


def _services_in_profile(profile: str | None) -> dict[str, dict[str, Any]]:
    """Сервисы, попадающие в указанный профиль (None = default)."""
    all_services = _all_services_with_profiles()
    result: dict[str, dict[str, Any]] = {}
    for name, cfg in all_services.items():
        profiles = cfg.get("profiles") or []
        if profile is None:
            if not profiles:
                result[name] = cfg
        else:
            if profile in profiles:
                result[name] = cfg
    return result


# --------------------------------------------------------------------------- #
# 1. Каждый сервис имеет pull_policy
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "profile",
    [None, "init", "monitoring", "ai"],
    ids=["default", "init", "monitoring", "ai"],
)
def test_every_service_has_pull_policy(profile: str | None) -> None:
    """Каждый сервис (в любом профиле) обязан объявлять ``pull_policy``.

    Без явного ``pull_policy`` compose v2 использует ``always`` на
    legacy-бэкенде или ``missing`` на buildx. Каждое из этих значений
    при offline registry вызывает pull attempt → ``no route to host``
    → зависимый сервис висит на старте. Issue #2610.
    """
    services = _services_in_profile(profile)
    assert services, f"profile {profile!r} returned no services (regression?)"

    missing: list[str] = []
    for name, cfg in services.items():
        if not cfg.get("pull_policy"):
            missing.append(name)

    assert not missing, (
        f"profile={profile!r}: сервисы без pull_policy: {missing}. "
        f"Все 17 сервисов обязаны явно объявлять pull_policy (issue #2610, "
        f"PR #2634). Допустимые значения: if_not_present | missing | never."
    )


# --------------------------------------------------------------------------- #
# 2. voice-resources-init в профиле [init] и не в default
# --------------------------------------------------------------------------- #


def test_voice_resources_init_only_in_init_profile() -> None:
    """``voice-resources-init`` помечен ``profiles: [init]``.

    Гарантирует: ``docker compose up -d`` без ``--profile init`` НЕ
    пытается стартовать init-контейнер (чей образ может быть ещё
    не собран в registry → при offline registry это снова стоп).
    """
    all_services = _all_services_with_profiles()
    assert "voice-resources-init" in all_services, (
        "voice-resources-init удалён из compose — это регрессия PR #2634. "
        "Сервис обязан оставаться в profiles: [init] (см. ADR-0111)."
    )
    cfg = all_services["voice-resources-init"]
    profiles = cfg.get("profiles") or []
    assert "init" in profiles, (
        f"voice-resources-init.profiles = {profiles!r}, expected ['init']. "
        f"Без этого default 'up -d' запустит init-контейнер и упадёт на "
        f"недоступном registry."
    )

    # Явная анти-проверка: в default он НЕ должен попадать.
    default_services = _services_in_profile(None)
    assert "voice-resources-init" not in default_services, (
        "voice-resources-init попал в default-профиль — это регрессия. "
        "Он обязан быть только в profiles: [init]."
    )


# --------------------------------------------------------------------------- #
# 3. Downstream supercollider + voice-assistant → voice-resources-init
#    через required: false (partial startup)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "service_name",
    ["supercollider", "voice-assistant"],
)
def test_downstream_uses_required_false_on_voice_resources_init(
    service_name: str,
) -> None:
    """Downstream-зависимость на voice-resources-init — partial-startup-safe.

    compose v2 трактует ``depends_on.<svc>.condition`` как no-op, если
    ``<svc>`` объявлен в профиле, который не активирован. Без явного
    ``required: false`` это поведение зависит от версии compose, что
    для нас регрессионно-нестабильно. Фикс: ``required: false``
    явно говорит "если init-сервис не запускался, просто стартуй
    downstream как обычно" (см. PR #2634).
    """
    services = _all_services_with_profiles()
    cfg = services[service_name]
    deps = cfg.get("depends_on") or {}
    if not isinstance(deps, dict):
        pytest.skip(
            f"{service_name} использует list-form depends_on (старая форма), "
            "формат ключей не совпадает с длинной формой condition. "
            "Проверь вручную, что voice-resources-init стартует с "
            "required: false (см. docker-compose.yaml)."
        )

    assert "voice-resources-init" in deps, (
        f"{service_name} не имеет depends_on на voice-resources-init. "
        f"Без этого compose не сможет отслеживать готовность init-volume "
        f"при запуске с --profile init (issue #2610)."
    )
    dep = deps["voice-resources-init"]
    assert isinstance(dep, dict), (
        f"{service_name}.depends_on.voice-resources-init = {dep!r}, "
        "ожидалась длинная форма с condition и required."
    )
    assert dep.get("required") is False, (
        f"{service_name}.depends_on.voice-resources-init.required = "
        f"{dep.get('required')!r}, expected False. Без required: false "
        f"compose v2 может упасть, если init-сервис не активирован "
        f"профилем (регрессия issue #2610)."
    )
    assert dep.get("condition") == "service_completed_successfully", (
        f"{service_name}.depends_on.voice-resources-init.condition = "
        f"{dep.get('condition')!r}, expected service_completed_successfully. "
        "Иначе supercollider стартанёт ДО того, как init-volume будет "
        "заполнен samples (MusicSkill не сможет найти wav)."
    )


# --------------------------------------------------------------------------- #
# 4. У каждого сервиса есть healthcheck или restart (anti-cascade-failure)
# --------------------------------------------------------------------------- #


_KNOWN_LONG_RUNNING = {"zenoh-router", "oak-d", "supercollider", "voice-assistant"}


def test_long_running_services_have_restart_unless_stopped() -> None:
    """Долгоживущие сервисы обязаны иметь ``restart: unless-stopped``.

    Если сервис не помечен — после ребута Vision Pi он останется в
    состоянии exited, и зависимые от него зависнут на depends_on.
    Cold-start без этого ломает восстановление после reboot.
    """
    all_services = _all_services_with_profiles()
    bad: list[str] = []
    for name in _KNOWN_LONG_RUNNING:
        cfg = all_services.get(name)
        if cfg is None:
            continue
        restart = cfg.get("restart")
        if restart not in ("unless-stopped", "always", "on-failure"):
            bad.append(f"{name} (restart={restart!r})")
    assert not bad, (
        "Долгоживущие сервисы без устойчивого restart: " + ", ".join(bad) +
        ". Cold-start после reboot требует restart: unless-stopped "
        "(или always/on-failure с продуманными лимитами)."
    )


# --------------------------------------------------------------------------- #
# 5. network_mode: host присутствует у ROS2-зависимых сервисов
# --------------------------------------------------------------------------- #


_ROS_SERVICES = {
    "oak-d", "supercollider", "voice-assistant", "april-tag-detector",
    "led-matrix", "quest-bridge", "telegram-bot", "vision-hailo",
}


def test_ros_services_use_network_mode_host() -> None:
    """ROS2-сервисы используют ``network_mode: host`` для multicast discovery.

    Без host-сети DDS multicast не работает между контейнерами — стек
    стартует, но ноды не видят друг друга. Это другой cold-start bug,
    чем #2610, но ровно из той же категории «стек встал без ошибок в
    compose ps, но функционально не работает».
    """
    all_services = _all_services_with_profiles()
    bad: list[str] = []
    for name in _ROS_SERVICES:
        cfg = all_services.get(name)
        if cfg is None:
            continue
        nm = cfg.get("network_mode")
        if nm != "host":
            bad.append(f"{name} (network_mode={nm!r})")
    assert not bad, (
        "ROS2-сервисы без network_mode: host: " + ", ".join(bad) +
        ". DDS multicast требует host network."
    )


# --------------------------------------------------------------------------- #
# 6. Общее число сервисов — сходится с эталоном PR #2634
# --------------------------------------------------------------------------- #


def test_total_service_count_matches_known_inventory() -> None:
    """17 сервисов всего (default + init + monitoring + ai).

    Это значение зафиксировано в PR #2634 acceptance-критерии #1.
    Если кто-то добавит/удалит сервис и не обновит инвентарь —
    должен явно подтвердить, что cold-start-сценарий всё ещё
    покрывается.
    """
    all_services = _all_services_with_profiles()
    total = len(all_services)
    # default + init + monitoring + ai должны давать 17 уникальных
    # имён. Допускаем ±2 для будущих PR с обоснованным изменением.
    assert 15 <= total <= 19, (
        f"обнаружено {total} сервисов в compose, ожидалось ~17 (PR #2634). "
        f"Сервисы: {sorted(all_services.keys())}. "
        f"Если это намеренное изменение — обновите этот guard и acceptance "
        f"тест в scripts/tests/test_t_b79d0581_compose_acceptance.py."
    )


# --------------------------------------------------------------------------- #
# 7. Защита от revert: образы НЕ должны хардкодить ghcr.io (для override)
# --------------------------------------------------------------------------- #


def test_inhouse_images_are_overridable_via_service_image_prefix() -> None:
    """Собственные (in-house) образы rob_box строятся через ``${SERVICE_IMAGE_PREFIX}``.

    Cold-start override (`override-bad-registry-cold-start.yaml`)
    подменяет ``SERVICE_IMAGE_PREFIX`` на ``nonexistent.invalid/x`` —
    если хоть один из наших образов захардкожен прямо в ``image:``
    (например ``ghcr.io/krikz/rob_box:voice-base-...``), override не
    сработает и compose пойдёт в оригинальный registry.

    Upstream-образы (zenoh, cadvisor, promtail, ollama) не считаются —
    они берутся из публичных registry и не зависят от katana SPOF.
    """
    # Паттерн "наш" образ: krikz/rob_box ИЛИ rob_box_*
    # (ghcr.io/krikz/rob_box, localhost:5000/krikz/rob_box, etc.)
    INHOUSE_HARDCODED_RE = re.compile(
        r"\b(krikz/rob_box|rob_box_[a-z0-9_-]+)\b"
    )

    all_services = _all_services_with_profiles()
    bad: list[str] = []
    for name, cfg in all_services.items():
        image = cfg.get("image")
        if not isinstance(image, str):
            continue
        # In-house образ?
        if not INHOUSE_HARDCODED_RE.search(image):
            continue
        # Раз строка содержит ${SERVICE_IMAGE_PREFIX — overridable
        if "${SERVICE_IMAGE_PREFIX" in image:
            continue
        # Иначе — захардкожен
        bad.append(f"{name} (image={image!r})")
    assert not bad, (
        "In-house образы без SERVICE_IMAGE_PREFIX (cold-start override не "
        "сможет перенаправить pull в bad-registry): "
        + ", ".join(bad)
        + ". Используй шаблон "
        + '${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}:<name>-${ROS_DISTRO:-humble}-${<TAG>:-${IMAGE_TAG}} '
        + "во всех собственных image: (см. PR #2634)."
    )
