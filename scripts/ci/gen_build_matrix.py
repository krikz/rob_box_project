#!/usr/bin/env python3
"""Читатель docker/build-manifest.yaml — единственного источника истины
о составе docker-сборки (docs/plans/2026-09-15-service-manifest.md).

Phase 2/3 (план §5), ADR-0159 PR-3: `prepare`-job единого
".github/workflows/L-Build Pi Services.yml" (вход `pi: main|vision|sound`)
вызывает этот скрипт и кладёт JSON в `$GITHUB_OUTPUT`, откуда его читают
wave-job'ы: `strategy.matrix.include: fromJSON(needs.prepare.outputs.waves)[k]`;
`update-image-versions` берёт из него же список `*_TAG`-переменных вместо
ручного перечисления. Состав сборки не хранится в тексте workflow — только в
docker/build-manifest.yaml.

Библиотечные функции (без побочных эффектов, без сети, без git/docker —
это то, что делает модуль тестируемым без CI, план §4.2):

    load_manifest(path)             -> dict, распарсенный и провалидированный
    services_for_pi(manifest, pi)   -> {имя_сервиса: описание}
    topo_layers(services)           -> [[уровень0], [уровень1], ...] — алгоритм
                                        Кана; уровни = волны сборки
    wave_entries(services, ctx)     -> [[элемент matrix, ...], ...] по волнам
    max_wave_count(manifest)        -> сколько wave-job'ов нужно workflow
    image_versions_map(services)    -> {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса}
    build_context(manifest, ...)    -> BuildContext (префиксы реестров,
                                        ros_distro, apt_proxy, docker_tag)
    matrix_entry(name, svc, ...)    -> один элемент matrix: полностью
                                        разрешённые tags/build-args
    msgs_image_ref(manifest, ...)   -> `<local>:msgs-<distro>-<хэш>` общего
                                        образа ROS-сообщений (читает файлы
                                        пакетов — единственное чтение с диска)

CLI:

    gen_build_matrix.py --pi sound --mode waves --docker-tag dev
    gen_build_matrix.py --pi vision --mode tags [--format shell --docker-tag dev]
    gen_build_matrix.py --pi vision --mode services [--format shell]
    gen_build_matrix.py --mode base
    gen_build_matrix.py --mode msgs

Каждый режим печатает JSON в stdout (план §4.2); `--format shell` — построчный
текст для `while read` в bash-шагах workflow.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any, NamedTuple

import yaml

# Валидное имя переменной *_TAG в docker/{main,vision,sound}/.image-versions.* —
# заглавные буквы/цифры/подчёркивания, обязательно оканчивается на _TAG.
_IMAGE_VERSIONS_VAR_RE = re.compile(r"^[A-Z][A-Z0-9_]*_TAG$")

# Корень репо: относительно него лежат src/<пакет> общего образа сообщений.
REPO_ROOT = Path(__file__).resolve().parents[2]


class ManifestError(ValueError):
    """Манифест синтаксически валиден как YAML, но нарушает контракт схемы
    (план §2.2) — цикл в depends_on, рассинхрон base.service/depends_on,
    невалидное имя image_versions и т.п.
    """


# --- загрузка и базовая валидация ---------------------------------------


def load_manifest(path: str | Path) -> dict[str, Any]:
    """Читает и валидирует docker/build-manifest.yaml.

    Валидация здесь — только структурная (инварианты схемы, §2.2/§2.3
    плана), не сверка с реальными workflow — та живёт в guard-тесте
    scripts/ci/tests/test_service_manifest_sync.py.
    """
    p = Path(path)
    with p.open(encoding="utf-8") as fh:
        manifest = yaml.safe_load(fh)

    if not isinstance(manifest, dict) or "pis" not in manifest:
        raise ManifestError(f"{p}: manifest must be a mapping with a 'pis' key")

    _check_unique_service_names(manifest)
    _validate_msgs_image(manifest)
    for pi_name, pi_data in manifest["pis"].items():
        services = (pi_data or {}).get("services") or {}
        _validate_services(pi_name, services)
        # topo_layers сам кидает ManifestError при цикле — валидируем
        # структуру графа зависимостей здесь же, при загрузке, а не только
        # по требованию потребителя waves/topo_layers.
        topo_layers(services)

    return manifest


def _check_unique_service_names(manifest: dict[str, Any]) -> None:
    """Имя сервиса уникально во всём манифесте: из него строятся тег
    (`<имя>-<distro>-<docker_tag>`) и buildcache-ref в ОДНОМ реестре на все Pi.
    """
    seen: set[str] = set()
    for pi_name, pi_data in manifest["pis"].items():
        for name in (pi_data or {}).get("services") or {}:
            if name in seen:
                raise ManifestError(
                    f"pis.{pi_name}.services.{name}: имя сервиса уже занято другим Pi"
                )
            seen.add(name)


def _validate_msgs_image(manifest: dict[str, Any]) -> None:
    """`msgs_image: true` у сервиса требует секцию msgs_image манифеста."""
    users = []
    for pi_name, pi_data in manifest["pis"].items():
        for name, svc in ((pi_data or {}).get("services") or {}).items():
            flag = svc.get("msgs_image", False)
            if not isinstance(flag, bool):
                raise ManifestError(
                    f"pis.{pi_name}.services.{name}: msgs_image={flag!r} — ожидался true/false"
                )
            if flag:
                users.append(f"pis.{pi_name}.services.{name}")
    section = manifest.get("msgs_image")
    if users and not section:
        raise ManifestError(
            f"{', '.join(users)}: msgs_image: true, но секции msgs_image в манифесте нет"
        )
    if section and not (
        section.get("dockerfile")
        and (section.get("base") or {}).get("family")
        and section.get("packages")
    ):
        raise ManifestError("msgs_image: нужны dockerfile, base.family и непустой packages")


def _validate_services(pi_name: str, services: dict[str, Any]) -> None:
    for name, svc in services.items():
        base = svc.get("base")
        depends_on = svc.get("depends_on") or []

        # Инвариант §2.3: если base.service задан, depends_on ОБЯЗАН
        # содержать ровно этот один сервис — иначе два поля разошлись
        # молча (ради чего они вообще разделены, см. план §2.3).
        if isinstance(base, dict) and base.get("service"):
            base_service = base["service"]
            if depends_on != [base_service]:
                raise ManifestError(
                    f"pis.{pi_name}.services.{name}: base.service={base_service!r} "
                    f"requires depends_on == [{base_service!r}], got {depends_on!r}"
                )

        iv = svc.get("image_versions", False)
        if iv is not False and not _IMAGE_VERSIONS_VAR_RE.match(str(iv)):
            raise ManifestError(
                f"pis.{pi_name}.services.{name}: image_versions={iv!r} is not a "
                f"valid *_TAG identifier (expected UPPER_SNAKE_CASE ending in _TAG, "
                f"or false)"
            )

        # Зависимость только внутри своего Pi (ADR-0159 PR-3): "L: Build All
        # Services" собирает Pi ПАРАЛЛЕЛЬНО, порядок между Pi не задан — ребро
        # в чужой Pi означало бы сборку FROM старого образа (гонка 09.09.2026,
        # supervisor ← voice-assistant). Такой сервис переезжает к своей базе.
        for dep in depends_on:
            if dep not in services:
                raise ManifestError(
                    f"pis.{pi_name}.services.{name}: depends_on references unknown "
                    f"service {dep!r} (зависимость обязана быть в pis.{pi_name})"
                )


# --- выборка сервисов -----------------------------------------------------


def services_for_pi(manifest: dict[str, Any], pi: str) -> dict[str, Any]:
    """Возвращает {имя_сервиса: описание} для одного pi (vision|sound|main)."""
    try:
        pi_data = manifest["pis"][pi]
    except KeyError as exc:
        raise ManifestError(
            f"manifest has no pis.{pi} (known: {sorted(manifest.get('pis', {}))})"
        ) from exc
    return dict((pi_data or {}).get("services") or {})


def topo_layers(services: dict[str, Any]) -> list[list[str]]:
    """Топологическая сортировка depends_on по уровням (алгоритм Кана).

    Уровень 0 — сервисы без зависимостей, уровень k — сервисы, все
    зависимости которых лежат на уровнях < k. Уровни — это волны сборки
    (вариант A плана §3.3, ADR-0159 PR-3): wave_entries, wave-job'ы workflow.

    Кидает ManifestError, если граф содержит цикл — вместо RecursionError
    или бесконечного цикла (план §4.3, тест-кейс "манифест с циклом").
    """
    remaining = {
        name: list(svc.get("depends_on") or []) for name, svc in services.items()
    }
    layers: list[list[str]] = []
    resolved: set[str] = set()

    while remaining:
        layer = sorted(
            name for name, deps in remaining.items() if all(d in resolved for d in deps)
        )
        if not layer:
            raise ManifestError(f"dependency cycle detected among: {sorted(remaining)}")
        layers.append(layer)
        for name in layer:
            del remaining[name]
        resolved.update(layer)

    return layers


def image_versions_map(services: dict[str, Any]) -> dict[str, str]:
    """{ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса} для сервисов с image_versions != false.

    Используется вместо ручного перечисления в tag_and_push/verify_in_registry/
    sed (Phase 3, план §5) — здесь, в Phase 1, только как режим CLI --mode tags
    и как источник для guard-теста.
    """
    out: dict[str, str] = {}
    for name, svc in services.items():
        iv = svc.get("image_versions", False)
        if iv is not False:
            out[str(iv)] = name
    return out


# --- разрешение манифеста в параметры сборки (Phase 2) --------------------
#
# Всё, что ниже, превращает декларативные поля манифеста в ТЕ ЖЕ САМЫЕ
# строки, которые до Phase 2 были записаны руками в каждом build-* job'е
# (tags:/build-args: композит-экшена ./.github/actions/l-build-service).
# Формулы выведены построчной сверкой с кодом обоих workflow на коммите
# 8740ea5ba — см. отчёт Phase 2 и tests/unit/test_workflow_refactor_acceptance.py.


class BuildContext(NamedTuple):
    """Параметры, общие для всех сервисов одного прогона."""

    image_prefix: str  # env.IMAGE_PREFIX   (ghcr.io/krikz/rob_box)
    local_prefix: str  # env.LOCAL_PREFIX   (localhost:5000/krikz/rob_box)
    base_registry: str  # localhost:5000/krikz/rob_box_base
    ros_distro: str  # env.ROS_DISTRO     (humble)
    apt_proxy: str  # http://host.docker.internal:3142
    docker_tag: str  # needs.prepare.outputs.docker_tag (dev|test|latest|local)
    msgs_image: str = ""  # msgs_image_ref(): общий образ ROS-сообщений


def build_context(
    manifest: dict[str, Any],
    *,
    docker_tag: str,
    image_prefix: str | None = None,
    local_prefix: str | None = None,
    ros_distro: str | None = None,
    repo_root: str | Path = REPO_ROOT,
) -> BuildContext:
    """Собирает BuildContext из defaults манифеста, с переопределением из CLI.

    Переопределение нужно потому, что источник истины для префиксов в
    рантайме — `env:` самого workflow (IMAGE_PREFIX/LOCAL_PREFIX/ROS_DISTRO).
    Guard-тест (scripts/ci/tests/test_service_manifest_sync.py) отдельно
    проверяет, что эти env совпадают с defaults манифеста — так два
    источника не разойдутся молча.
    """
    defaults = manifest.get("defaults") or {}
    registry = defaults.get("registry") or {}
    local = local_prefix or registry["local"]
    distro = ros_distro or defaults["ros_distro"]
    return BuildContext(
        image_prefix=image_prefix or registry["ghcr"],
        local_prefix=local,
        base_registry=defaults["base_registry"],
        ros_distro=distro,
        apt_proxy=defaults["apt_proxy"],
        docker_tag=docker_tag,
        msgs_image=msgs_image_ref(
            manifest, local_prefix=local, ros_distro=distro, repo_root=repo_root
        ),
    )


# --- общий образ ROS-сообщений --------------------------------------------
#
# src/rob_box_*_msgs компилируются ОДИН раз за прогон (docker/msgs/Dockerfile,
# ".github/workflows/L-Build Msgs Image.yml"), сервисы берут готовый
# /ws/install через COPY --from=${MSGS_IMAGE}. Тег зависит только от того, что
# реально входит в образ, поэтому одинаков во всех job'ах прогона и между
# прогонами без правок .msg — по нему сборка и решает, нужна ли она.


def _msgs_base_image(manifest: dict[str, Any], ros_distro: str) -> str:
    defaults = manifest.get("defaults") or {}
    family = manifest["msgs_image"]["base"]["family"]
    return f"{defaults['base_registry']}:{family}-{ros_distro}"


def msgs_image_ref(
    manifest: dict[str, Any],
    *,
    local_prefix: str,
    ros_distro: str,
    repo_root: str | Path = REPO_ROOT,
) -> str:
    """`<local_prefix>:msgs-<ros_distro>-<хэш12>`; "" — секции msgs_image нет.

    Хэш sha256 — по ссылке на базу, Dockerfile и всем файлам src/<пакет>:
    относительный путь + содержимое, CRLF → LF (чекаут на Windows и в CI
    обязан давать один и тот же тег). Пересборка самой базы тег не меняет:
    ABI ROS humble внутри дистрибутива стабилен.
    """
    section = manifest.get("msgs_image")
    if not section:
        return ""
    root = Path(repo_root)
    files = [root / section["dockerfile"]]
    for pkg in section["packages"]:
        pkg_dir = root / "src" / pkg
        if not pkg_dir.is_dir():
            raise ManifestError(f"msgs_image.packages: нет каталога {pkg_dir}")
        files += [
            f
            for f in pkg_dir.rglob("*")
            if f.is_file() and "__pycache__" not in f.parts
        ]
    digest = hashlib.sha256(_msgs_base_image(manifest, ros_distro).encode() + b"\0")
    for rel, f in sorted((f.relative_to(root).as_posix(), f) for f in files):
        digest.update(rel.encode() + b"\0")
        digest.update(f.read_bytes().replace(b"\r\n", b"\n") + b"\0")
    return f"{local_prefix}:msgs-{ros_distro}-{digest.hexdigest()[:12]}"


def msgs_image_entry(manifest: dict[str, Any], ctx: BuildContext) -> dict[str, str]:
    """Элемент сборки общего образа — набор ключей как у `matrix_entry`
    (+ `image`), чтобы композит и scripts/build/build.py звали его так же."""
    section = manifest.get("msgs_image")
    if not section:
        raise ManifestError("manifest has no msgs_image")
    build_args = [
        f"BASE_IMAGE={_msgs_base_image(manifest, ctx.ros_distro)}",
        f"APT_PROXY={ctx.apt_proxy}",
    ]
    return {
        "name": "msgs",
        "image": ctx.msgs_image,
        "dockerfile": section["dockerfile"],
        "build_context": section.get("build_context", "."),
        "tags": ctx.msgs_image,
        "build_args": "\n".join(build_args),
        "submodule_sha": "",
        "source_hash_arg": "",
        "source_hash_spec": "",
        "pre_build": "",
        "depends_on": "",
    }


def tag_basename(name: str, svc: dict[str, Any], ctx: BuildContext) -> str:
    """`<service>-<ros_distro>-<docker_tag>` или `<service>-<docker_tag>`.

    Второй вариант — только supercollider (`tag.ros_distro: false`,
    "tag naming is inconsistent" в коде workflow).
    """
    if (svc.get("tag") or {}).get("ros_distro", True):
        return f"{name}-{ctx.ros_distro}-{ctx.docker_tag}"
    return f"{name}-{ctx.docker_tag}"


def _base_image(svc: dict[str, Any], services: dict[str, Any], ctx: BuildContext):
    """BASE_IMAGE build-arg или None (сервис без базового образа)."""
    base = svc.get("base")
    if not base:
        return None
    if base.get("family"):
        return f"{ctx.base_registry}:{base['family']}-{ctx.ros_distro}"
    dep = base["service"]
    return f"{ctx.local_prefix}:{tag_basename(dep, services[dep], ctx)}"


def _source_hash_spec(svc: dict[str, Any]) -> str:
    """Спецификация для generic-шага "Compute source hash" в workflow.

    Одна строка на группу манифеста: `<пути через пробел>\\t<шаблоны find>`.
    Расширение вида ".py" превращается в шаблон "*.py", имя файла
    ("package.xml", "CMakeLists.txt") остаётся литералом — ровно так же,
    как это было написано руками в `find ... \\( -name ... \\)` каждого job'а.
    """
    source_hash = svc.get("source_hash")
    if not source_hash:
        return ""
    lines = []
    for group in source_hash["groups"]:
        patterns = [
            f"*{ext}" if ext.startswith(".") else ext for ext in group["extensions"]
        ]
        lines.append(" ".join(group["paths"]) + "\t" + " ".join(patterns))
    return "\n".join(lines)


def matrix_entry(name: str, services: dict[str, Any], ctx: BuildContext) -> dict[str, str]:
    """Один элемент matrix — полностью разрешённые параметры сборки сервиса.

    Набор ключей ОДИНАКОВ для всех сервисов (пустая строка там, где
    свойства нет): шаблон job'а в workflow один на всех, а `matrix.<key>`
    для отсутствующего ключа дал бы пустую строку неявно — лучше явно.
    """
    svc = services[name]
    basename = tag_basename(name, svc, ctx)

    build_args = []
    base_image = _base_image(svc, services, ctx)
    if base_image:
        build_args.append(f"BASE_IMAGE={base_image}")
    build_args.append(f"APT_PROXY={ctx.apt_proxy}")
    for key in sorted(svc.get("build_args") or {}):
        build_args.append(f"{key}={svc['build_args'][key]}")
    if svc.get("msgs_image"):
        build_args.append(f"MSGS_IMAGE={ctx.msgs_image}")

    return {
        "name": name,
        "dockerfile": svc["dockerfile"],
        "build_context": svc["build_context"],
        "tags": f"{ctx.image_prefix}:{basename}\n{ctx.local_prefix}:{basename}",
        "build_args": "\n".join(build_args),
        "submodule_sha": svc.get("submodule_sha", "") or "",
        "source_hash_arg": (svc.get("source_hash") or {}).get("arg", "") or "",
        "source_hash_spec": _source_hash_spec(svc),
        "pre_build": svc.get("pre_build", "") or "",
        "depends_on": " ".join(svc.get("depends_on") or []),
        # Образ, который сервис ждёт в registry перед сборкой ("" — не ждёт).
        "msgs_image": ctx.msgs_image if svc.get("msgs_image") else "",
    }


def wave_entries(services: dict[str, Any], ctx: BuildContext) -> list[list[dict[str, str]]]:
    """Волны сборки одного Pi: `topo_layers` → элементы matrix (ADR-0159 PR-3).

    Волна k — сервисы, все зависимости которых собраны в волнах < k. Единый
    "L-Build Pi Services.yml" держит по matrix-job'у на волну (`build`,
    `build-wave-1`, …) и вешает волну k на волну k-1 через `needs:` — список
    сервисов в workflow не нужен, ни одного имени сервиса там нет.

    Цена против прежних именованных job'ов (план §3.3, вариант A): волна ждёт
    ВСЮ предыдущую волну, а не только свою зависимость — `voice-assistant`
    ждёт и `supercollider`, хотя наследует только `voice-base`.
    """
    return [
        [matrix_entry(name, services, ctx) for name in layer]
        for layer in topo_layers(services)
    ]


def max_wave_count(manifest: dict[str, Any]) -> int:
    """Сколько волн нужно самому глубокому Pi — столько wave-job'ов обязан
    держать workflow (guard scripts/ci/tests/test_service_manifest_sync.py)."""
    return max(
        len(topo_layers(services_for_pi(manifest, pi))) for pi in manifest["pis"]
    )


def base_image_entries(
    manifest: dict[str, Any], *, ros_distro: str | None = None
) -> dict[str, list[dict[str, str]]]:
    """Базовые образы (секция `base_images`) — {имя: [элемент]}.

    В "L-Build Base Images.yml" по именованному job'у на образ (pcl `needs:` ros2-zenoh), а каждый job
    берёт свой элемент по ИМЕНИ. Набор ключей — как у `matrix_entry`, чтобы
    композит и scripts/build/build.sh звались одинаково для базы и сервиса.
    """
    defaults = manifest.get("defaults") or {}
    section = manifest.get("base_images") or {}
    images = section.get("images") or {}
    if not images:
        raise ManifestError("manifest has no base_images.images")
    distro = ros_distro or defaults["ros_distro"]
    ghcr = section["registry"]["ghcr"]
    local = defaults["base_registry"]
    apt_proxy = defaults["apt_proxy"]
    context = section.get("build_context", "docker/base")

    result: dict[str, list[dict[str, str]]] = {}
    for name, img in images.items():
        depends_on = img.get("depends_on") or []
        for dep in depends_on:
            if dep not in images:
                raise ManifestError(
                    f"base_images.images.{name}: depends_on references unknown image {dep!r}"
                )
        build_args = [f"BASE_IMAGE={local}:{dep}-{distro}-latest" for dep in depends_on]
        build_args.append(f"APT_PROXY={apt_proxy}")
        result[name] = [
            {
                "name": name,
                "dockerfile": img["dockerfile"],
                "build_context": img.get("build_context", context),
                "tags": "\n".join(
                    [
                        f"{ghcr}:{name}-{distro}-latest",
                        f"{local}:{name}-{distro}-latest",
                        f"{local}:{name}-{distro}",
                    ]
                ),
                "build_args": "\n".join(build_args),
                "submodule_sha": "",
                "source_hash_arg": "",
                "source_hash_spec": "",
                "pre_build": "",
                "depends_on": " ".join(depends_on),
            }
        ]
    return result


def image_versions_tags(
    services: dict[str, Any], ctx: BuildContext
) -> list[tuple[str, str]]:
    """[(ИМЯ_ПЕРЕМЕННОЙ, `<service>-<ros_distro>-<docker_tag>`), ...].

    Потребитель — update-image-versions (Phase 3): tag_and_push,
    verify_in_registry и sed по .image-versions.* перестают перечислять
    сервисы руками и идут по этому списку.
    """
    out = []
    for var, name in sorted(image_versions_map(services).items()):
        out.append((var, tag_basename(name, services[name], ctx)))
    return out


# --- CLI --------------------------------------------------------------


def _build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Читает docker/build-manifest.yaml и печатает JSON для одного "
            "из режимов CI-генерации (план §4.2). Phase 1: только read-only, "
            "ничего не пишет обратно в workflow-файлы."
        )
    )
    parser.add_argument(
        "--manifest",
        default="docker/build-manifest.yaml",
        help="Путь к манифесту (по умолчанию docker/build-manifest.yaml)",
    )
    parser.add_argument(
        "--pi",
        required=False,  # обязателен для всех режимов, кроме base (проверка в main)
        choices=["vision", "sound", "main"],
        help="Для какого Pi генерировать вывод",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["waves", "tags", "services", "base", "msgs"],
        help=(
            "waves — [[элемент, ...], ...] по волнам сборки Pi: волна k — "
            "strategy.matrix.include wave-job'а k; "
            "tags — {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса} для .image-versions.* "
            "(с --format shell — строки `ПЕРЕМЕННАЯ <tag>`); "
            "services — все сервисы Pi (для сводки в summary-job'е); "
            "base — {имя: [элемент]} базовых образов (секция base_images, "
            "--pi не нужен); "
            "msgs — элемент сборки общего образа ROS-сообщений (секция "
            "msgs_image, --pi не нужен)"
        ),
    )
    parser.add_argument(
        "--docker-tag",
        default=None,
        help=(
            "needs.prepare.outputs.docker_tag (dev|test|latest|local). "
            "Обязателен для --mode waves и для --mode tags "
            "--format shell — без него теги невозможно разрешить."
        ),
    )
    parser.add_argument(
        "--ros-distro",
        default=None,
        help="env.ROS_DISTRO workflow'а (по умолчанию — defaults.ros_distro манифеста)",
    )
    parser.add_argument(
        "--image-prefix",
        default=None,
        help="env.IMAGE_PREFIX workflow'а (по умолчанию — defaults.registry.ghcr)",
    )
    parser.add_argument(
        "--local-prefix",
        default=None,
        help="env.LOCAL_PREFIX workflow'а (по умолчанию — defaults.registry.local)",
    )
    parser.add_argument(
        "--format",
        default="json",
        choices=["json", "shell"],
        help=(
            "json (по умолчанию) — JSON в одну строку для fromJSON(); "
            "shell — построчный текст для `while read` в bash-шагах "
            "(режимы tags и services)"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_cli().parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        # docker/build-manifest.yaml → корень репо, от него src/<пакет> msgs.
        repo_root = Path(args.manifest).resolve().parents[1]
        if args.mode == "base":
            # Базовые образы не привязаны к Pi и не несут docker_tag.
            bases = base_image_entries(manifest, ros_distro=args.ros_distro)
            print(json.dumps(bases, ensure_ascii=False))
            return 0
        if args.mode == "msgs":
            # Общий образ сообщений тоже вне Pi; тег — хэш, не docker_tag.
            ctx = build_context(
                manifest,
                docker_tag="",
                local_prefix=args.local_prefix,
                ros_distro=args.ros_distro,
                repo_root=repo_root,
            )
            print(json.dumps(msgs_image_entry(manifest, ctx), ensure_ascii=False))
            return 0
        if not args.pi:
            raise ManifestError(f"--mode {args.mode} requires --pi")
        services = services_for_pi(manifest, args.pi)

        needs_tag = args.mode == "waves" or (
            args.mode == "tags" and args.format == "shell"
        )
        if needs_tag and not args.docker_tag:
            raise ManifestError(f"--mode {args.mode} requires --docker-tag")

        if needs_tag:
            ctx = build_context(
                manifest,
                docker_tag=args.docker_tag,
                image_prefix=args.image_prefix,
                local_prefix=args.local_prefix,
                ros_distro=args.ros_distro,
                repo_root=repo_root,
            )

        if args.mode == "waves":
            result: Any = wave_entries(services, ctx)
        elif args.mode == "services":
            result = sorted(services)
        else:  # tags
            result = (
                image_versions_tags(services, ctx)
                if args.format == "shell"
                else image_versions_map(services)
            )
    except (ManifestError, KeyError) as exc:
        print(f"::error::gen_build_matrix: {exc}", file=sys.stderr)
        return 1

    if args.format == "shell":
        if args.mode == "services":
            print(" ".join(result))
        elif args.mode == "tags":
            for var, tag in result:
                print(f"{var} {tag}")
        else:
            print(
                f"::error::gen_build_matrix: --format shell не поддержан для "
                f"--mode {args.mode}",
                file=sys.stderr,
            )
            return 1
        return 0

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
