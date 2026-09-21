"""Integration test: docker compose config с override-bad-registry.

PR #2634 acceptance #2: при попытке старта ``docker compose up -d`` с
override, указывающим на ``nonexistent.invalid`` registry, compose
ДОЛЖЕН НЕ падать на стадии ``config`` — иначе systemd увидит
exit≠0 и не поднимет стек.

Реальные сценарии, которые мы покрываем:
  1. ``docker compose config`` (default) парсится без ошибок.
  2. ``docker compose config`` с ``-f override-bad-registry.yaml``
     подменяет ``SERVICE_IMAGE_PREFIX`` на TEST-NET-1 (RFC 5737)
     и тоже парсится без ошибок — то есть наш override
     синтаксически валиден и override-механика работает.
  3. Список сервисов default-профиля сходится со статическим
     парсером из test_compose_cold_start.py.

БЫЛО, НО УШЛО: проверки про ``--profile init`` и
``voice-resources-init``. Образ с Renardo-сэмплами и его init-контейнер
удалены — сэмплы кладёт на хост Ресурсный пак
(``/opt/rob_box/samples``), и профиля ``init`` больше нет ни у одного
сервиса. Проверять «стартует ли стек без init-образа» стало нечего:
старт стека больше не ходит в registry за сэмплами вообще. Структурный
инвариант, пришедший на смену (bind-mount вместо volume, никто не ждёт
one-shot контейнера), проверяет test_compose_cold_start.py.

Эти тесты требуют ``docker`` CLI в PATH. На CI без Docker они
пропускаются с пометкой reason. Локально на dev-машине
прогоняются за <2 секунды.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
COMPOSE_DIR = REPO_ROOT / "docker" / "vision"
COMPOSE_FILE = COMPOSE_DIR / "docker-compose.yaml"
BAD_REGISTRY_OVERRIDE = REPO_ROOT / "scripts" / "tests" / "override-bad-registry.yaml"


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _docker_available() -> bool:
    return shutil.which("docker") is not None and subprocess.run(
        ["docker", "compose", "version"],
        capture_output=True,
        text=True,
        timeout=10,
    ).returncode == 0


def _run_compose_config(*extra_args: str, profile: str | None = None) -> subprocess.CompletedProcess:
    """Запустить ``docker compose config`` с заданным профилем/аргументами."""
    cmd = ["docker", "compose"]
    if profile:
        cmd.extend(["--profile", profile])
    cmd.extend(["-f", str(COMPOSE_FILE)])
    cmd.extend(extra_args)
    cmd.append("config")
    return subprocess.run(
        cmd,
        cwd=COMPOSE_DIR,
        capture_output=True,
        text=True,
        timeout=60,
        # Не подавляем ошибки — capture_output=True уже даёт нам доступ.
    )


# --------------------------------------------------------------------------- #
# Skip-маркер для CI без docker
# --------------------------------------------------------------------------- #


pytestmark = pytest.mark.skipif(
    not _docker_available(),
    reason="docker compose CLI недоступен (типично для CI без docker)",
)


# --------------------------------------------------------------------------- #
# 1. docker compose config без override (default)
# --------------------------------------------------------------------------- #


def test_compose_config_default_validates() -> None:
    """``docker compose config`` (default) — exit 0.

    Если этот тест падает — кто-то внёс невалидный YAML.
    """
    # Нужен .env.secrets, иначе compose ругается на отсутствующий файл
    env_secrets = COMPOSE_DIR / ".env.secrets"
    env_secrets.touch(exist_ok=True)
    result = _run_compose_config()
    assert result.returncode == 0, (
        f"docker compose config exit={result.returncode}.\n"
        f"STDERR (tail 1000):\n{result.stderr[-1000:]}"
    )
    # Должен быть валидный JSON/YAML на выходе.
    parsed = yaml.safe_load(result.stdout)
    assert isinstance(parsed, dict) and "services" in parsed, (
        f"compose config не вернул ожидаемый YAML: {result.stdout[:500]!r}"
    )


# --------------------------------------------------------------------------- #
# 2. docker compose config с override-bad-registry (TEST-NET-1)
# --------------------------------------------------------------------------- #


def test_compose_config_with_bad_registry_override_validates() -> None:
    """``docker compose -f override-bad-registry.yaml config`` — exit 0.

    TEST-NET-1 (192.0.2.0/24) гарантированно не маршрутизируется в
    интернете (RFC 5737). Если compose успешно парсит конфиг с этим
    override — значит override-механика работает и наш
    ``${SERVICE_IMAGE_PREFIX}`` правильно подменяется.
    """
    assert BAD_REGISTRY_OVERRIDE.exists(), (
        f"override файл не найден: {BAD_REGISTRY_OVERRIDE}. "
        f"Ожидался из PR #2634 acceptance-теста (scripts/tests/)."
    )

    env_secrets = COMPOSE_DIR / ".env.secrets"
    env_secrets.touch(exist_ok=True)
    result = subprocess.run(
        [
            "docker", "compose",
            "-f", str(COMPOSE_FILE),
            "-f", str(BAD_REGISTRY_OVERRIDE),
            "config",
        ],
        cwd=COMPOSE_DIR,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"docker compose config с override-bad-registry exit={result.returncode}.\n"
        f"STDERR (tail 1500):\n{result.stderr[-1500:]}"
    )
    # Проверим, что override реально применился — ищем TEST-NET-1 в image.
    parsed = yaml.safe_load(result.stdout)
    services = parsed.get("services") or {}
    # Ищем zenoh-router (он в override файле один из сервисов)
    zenoh = services.get("zenoh-router") or {}
    zenoh_image = zenoh.get("image", "")
    assert "192.0.2.1" in zenoh_image or "192.0.2" in zenoh_image, (
        f"override не применился: zenoh-router.image = {zenoh_image!r}, "
        f"ожидался TEST-NET-1 (192.0.2.x). Проверь scripts/tests/override-bad-registry.yaml"
    )


def test_compose_config_with_service_image_prefix_override() -> None:
    """``docker compose config`` с ``SERVICE_IMAGE_PREFIX=192.0.2.1:9999/missing`` — exit 0.

    Это интеграционная проверка cold-start сценария: при
    недоступном registry ВСЕ in-house образы должны подменяться
    через SERVICE_IMAGE_PREFIX. Тест НЕ проверяет, что образы
    реально pull'нутся (это e2e на железе), только что compose
    корректно раскрывает переменную и парсит результат.
    """
    env_secrets = COMPOSE_DIR / ".env.secrets"
    env_secrets.touch(exist_ok=True)
    env = {
        **os.environ,
        "SERVICE_IMAGE_PREFIX": "192.0.2.1:9999/missing",
        "ROS_DISTRO": "humble",
        "IMAGE_TAG": "test",
    }
    result = subprocess.run(
        ["docker", "compose", "-f", str(COMPOSE_FILE), "config"],
        cwd=COMPOSE_DIR,
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
    )
    assert result.returncode == 0, (
        f"docker compose config с SERVICE_IMAGE_PREFIX=192.0.2.1 exit={result.returncode}.\n"
        f"STDERR (tail 1500):\n{result.stderr[-1500:]}"
    )
    # Проверим, что in-house образы действительно подменились.
    parsed = yaml.safe_load(result.stdout)
    services = parsed.get("services") or {}

    # Найдём хотя бы один сервис с in-house image, который должен был
    # подмениться. Проверяем voice-assistant, supercollider,
    # oak-d — точно in-house.
    inhouse_services = ["voice-assistant", "supercollider", "oak-d"]
    bad: list[str] = []
    for svc_name in inhouse_services:
        svc = services.get(svc_name)
        if svc is None:
            continue  # может быть в профиле, не проверяем
        image = svc.get("image", "")
        if not image.startswith("192.0.2.1:9999/missing"):
            bad.append(f"{svc_name}.image = {image!r}")

    assert not bad, (
        f"SERVICE_IMAGE_PREFIX не применился к in-house образам: {bad}. "
        f"Это значит, что compose раскрывает ${'{'}SERVICE_IMAGE_PREFIX{'}'} "
        f"раньше, чем override через env успевает сработать. "
        f"Проверь формат image в docker/vision/docker-compose.yaml."
    )


# --------------------------------------------------------------------------- #
# 3. Полный список сервисов по профилям сходится с inventory из static-теста
# --------------------------------------------------------------------------- #


def test_profile_inventory_matches_static_test() -> None:
    """Количество сервисов в каждом профиле — не меньше чем в static-тесте.

    Это страховка: если dynamic ``docker compose config --services``
    возвращает меньше сервисов, чем ожидает наш статический парсер
    YAML — значит, кто-то добавил profile, который static-тест не
    учитывает (тест_every_service_has_pull_policy с parametrize).
    """
    from tests.vision.cold_start.test_compose_cold_start import (
        _all_services_with_profiles,
    )

    static_count = len(_all_services_with_profiles())

    env_secrets = COMPOSE_DIR / ".env.secrets"
    env_secrets.touch(exist_ok=True)

    # Соберём все сервисы из всех профилей через docker compose.
    # Профиль "init" ушёл вместе с voice-resources-init.
    profiles = [None, "monitoring", "ai"]
    dynamic_set: set[str] = set()
    for profile in profiles:
        cmd = ["docker", "compose"]
        if profile:
            cmd.extend(["--profile", profile])
        cmd.extend(["-f", str(COMPOSE_FILE), "config", "--services"])
        result = subprocess.run(
            cmd,
            cwd=COMPOSE_DIR,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert result.returncode == 0, (
            f"docker compose --profile {profile!r} exit={result.returncode}.\n"
            f"STDERR:\n{result.stderr[-1000:]}"
        )
        dynamic_set.update(result.stdout.strip().split("\n"))

    missing_in_dynamic = set(_all_services_with_profiles().keys()) - dynamic_set
    assert not missing_in_dynamic, (
        f"Static-парсер видит сервисы, которых dynamic compose config "
        f"не показал ни в одном профиле: {missing_in_dynamic}. "
        f"Это расхождение между YAML-парсером и реальным compose — "
        f"нужно синхронизировать тесты."
    )

    # Дополнительно: dynamic счёт должен быть >= static (новые сервисы
    # в compose могут не отслеживаться static-тестом, но не наоборот).
    assert len(dynamic_set) >= static_count, (
        f"dynamic compose config видит {len(dynamic_set)} сервисов, "
        f"static parser — {static_count}. Расхождение в {static_count - len(dynamic_set)} "
        f"сервисов: dynamic МЕНЬШЕ static, что невозможно без рассинхрона."
    )
