"""Regression tests for ``docker/vision/docker-compose.yaml`` cold-start invariants.

Issue #2610 + PR #2634 — после ребута Vision Pi весь стек должен подниматься
даже при недоступном build-registry (katana 10.1.1.249:5000). Условия:

1. Все сервисы (default + profiled) обязаны иметь ``pull_policy`` —
   иначе compose применит дефолт (always на старых версиях / missing
   на новых), и при недоступном registry любой старт зависнет на pull.
2. Renardo-сэмплы приходят bind-mount'ом с хоста, а не из образа.
3. Ни один сервис не ждёт one-shot init-контейнера.

Пункты 2-3 раньше формулировались иначе: ``voice-resources-init`` в
``profiles: [init]`` + ``depends_on ... required: false`` у обоих
потребителей. Это был обход проблемы, а не её решение — init-контейнер
всё равно тянул с registry отдельный ~600-МБ образ, и вся конструкция
существовала только чтобы этот pull не ронял стек. Образ, init-контейнер
и named-volume ``renardo_samples`` удалены: сэмплы кладёт на хост
Ресурсный пак (``/opt/rob_box/samples``) на шаге деплоя, до
``docker compose up``. Теперь недоступность katana на старте не влияет
на сэмплы вообще — проверять «стартует ли стек без init-образа» стало
нечего, поэтому проверяем инвариант, который пришёл на смену:
потребители читают host-каталог и никого не ждут.

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
    # Профиль "init" ушёл вместе с voice-resources-init — в нём больше нет
    # ни одного сервиса, и параметр падал бы на «profile returned no services».
    "profile",
    [None, "monitoring", "ai"],
    ids=["default", "monitoring", "ai"],
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
# 2. Renardo-сэмплы приходят с хоста, а не из образа
# --------------------------------------------------------------------------- #

SAMPLES_HOST_DIR = "/opt/rob_box/samples"
SAMPLES_CONTAINER_DIR = "/root/.config/renardo/samples"


def test_no_voice_resources_init_service() -> None:
    """Init-контейнера с сэмплами в compose быть не должно.

    Он тянул с registry отдельный ~600-МБ образ при каждом деплое. Его
    возвращение означало бы, что старт стека снова зависит от katana —
    ровно та проблема, из-за которой появился issue #2610.
    """
    all_services = _all_services_with_profiles()
    assert "voice-resources-init" not in all_services, (
        "voice-resources-init вернулся в compose. Сэмплы должны приезжать "
        "на хост Ресурсным паком (запись renardo-samples в "
        "docker/vision/scripts/resource_pack/manifest.yaml), а не образом."
    )


def test_renardo_samples_named_volume_is_gone() -> None:
    """named-volume ``renardo_samples`` удалён — наполнять его больше некому."""
    data = _load_raw_compose()
    volumes = data.get("volumes") or {}
    assert "renardo_samples" not in volumes, (
        "named-volume renardo_samples вернулся. Пустой volume без "
        "init-контейнера — это молча немая музыка: bind-mount с хоста "
        "хотя бы видно в `docker inspect`."
    )


@pytest.mark.parametrize(
    ("service_name", "read_only"),
    [("supercollider", True), ("voice-assistant", False)],
)
def test_samples_come_from_host_bind_mount(service_name: str, read_only: bool) -> None:
    """Оба потребителя монтируют один host-каталог по прежнему пути внутри.

    Путь ВНУТРИ контейнера не менялся при переезде — именно поэтому ни один
    Python-файл (sample_search.py, foxdot_init.sc) трогать не потребовалось.
    supercollider получает каталог ``:ro`` — сэмплы меняет только хост.
    """
    services = _all_services_with_profiles()
    volumes = services[service_name].get("volumes") or []
    expected = f"{SAMPLES_HOST_DIR}:{SAMPLES_CONTAINER_DIR}"
    if read_only:
        expected += ":ro"

    assert expected in volumes, (
        f"{service_name} не монтирует {expected!r}. Найдено: {volumes!r}. "
        f"Без этого монтирования сэмплы, разложенные Ресурсным паком на "
        f"хосте, до контейнера не доедут — музыка уйдёт в synth-only."
    )


# --------------------------------------------------------------------------- #
# 3. Ни один сервис не ждёт one-shot init-контейнера
# --------------------------------------------------------------------------- #


def test_nobody_waits_for_a_one_shot_init_container() -> None:
    """``service_completed_successfully`` в compose быть не должно.

    Это условие имело смысл только для init-контейнера с сэмплами и стоило
    трёх итераций фиксов гонки (issue #2095, PR #2120, PR #2634): compose
    инспектировал уже удалённый exited-контейнер и валил деплой с
    «No such container: <HASH>». Init-контейнера нет — и условия быть не
    должно; если оно появится снова, вернётся и гонка.
    """
    offenders: list[str] = []
    for name, cfg in _all_services_with_profiles().items():
        deps = cfg.get("depends_on") or {}
        if not isinstance(deps, dict):
            continue
        for dep_name, dep_cfg in deps.items():
            if isinstance(dep_cfg, dict) and (
                dep_cfg.get("condition") == "service_completed_successfully"
            ):
                offenders.append(f"{name} → {dep_name}")

    assert not offenders, (
        "Появилось ожидание one-shot контейнера: " + ", ".join(offenders) +
        ". Это возвращает гонку «No such container: <HASH>» из issue #2095."
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
    """Порядка 16 сервисов всего (default + monitoring + ai).

    Это значение зафиксировано в PR #2634 acceptance-критерии #1.
    Если кто-то добавит/удалит сервис и не обновит инвентарь —
    должен явно подтвердить, что cold-start-сценарий всё ещё
    покрывается.
    """
    all_services = _all_services_with_profiles()
    total = len(all_services)
    # Было 17, стало 16: voice-resources-init удалён вместе с образом.
    # Допускаем ±2 для будущих PR с обоснованным изменением.
    assert 15 <= total <= 19, (
        f"обнаружено {total} сервисов в compose, ожидалось ~16. "
        f"Сервисы: {sorted(all_services.keys())}. "
        f"Если это намеренное изменение — обновите этот guard."
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
