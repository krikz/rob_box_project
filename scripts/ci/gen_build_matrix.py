#!/usr/bin/env python3
"""Читатель docker/build-manifest.yaml — единственного источника истины
о составе docker-сборки (docs/plans/2026-09-15-service-manifest.md).

Phase 2/3 (план §5): `prepare`-job обоих матричных workflow вызывает этот
скрипт и кладёт JSON в `$GITHUB_OUTPUT`, откуда его читает
`strategy.matrix: fromJSON(needs.prepare.outputs.matrix)`; `update-image-versions`
берёт из него же список `*_TAG`-переменных вместо ручного перечисления.
Состав сборки больше не хранится в тексте workflow — только в
docker/build-manifest.yaml.

Библиотечные функции (без побочных эффектов, без сети, без git/docker —
это то, что делает модуль тестируемым без CI, план §4.2):

    load_manifest(path)             -> dict, распарсенный и провалидированный
    services_for_pi(manifest, pi)   -> {имя_сервиса: описание}
    independent_services(services)  -> [{"name": ..., ...}, ...] без depends_on
    chained_services(services)      -> [{"name": ..., "depends_on": [...], ...}, ...]
                                        топологически отсортировано
    matrix_service_names(services)  -> [имена] — сервисы, которые НЕ участвуют
                                        в графе зависимостей ни концом, ни
                                        началом ребра (кандидаты в matrix-job)
    named_service_names(services)   -> [имена] — сервисы, которые участвуют в
                                        графе (зависят сами или от них зависят);
                                        топологически отсортировано. Они обязаны
                                        остаться ИМЕНОВАННЫМИ job'ами: на цель
                                        ребра (`voice-base`) кто-то ссылается
                                        через `needs:`, а `needs:` умеет
                                        ссылаться только на job ID, не на
                                        элемент матрицы (план §3.1).
    topo_layers(services)           -> [[уровень0], [уровень1], ...] — алгоритм
                                        Кана; задел на вариант A (§3.3 плана),
                                        не используется в MVP, но покрыт тестом
    image_versions_map(services)    -> {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса}
    build_context(manifest, ...)    -> BuildContext (префиксы реестров,
                                        ros_distro, apt_proxy, docker_tag)
    matrix_entry(name, svc, ...)    -> один элемент matrix: полностью
                                        разрешённые tags/build-args

CLI:

    gen_build_matrix.py --pi vision --mode matrix  --docker-tag dev
    gen_build_matrix.py --pi vision --mode chained --docker-tag dev
    gen_build_matrix.py --pi vision --mode tags [--format shell --docker-tag dev]
    gen_build_matrix.py --pi vision --mode services [--format shell]

Каждый режим печатает JSON в stdout (план §4.2); `--format shell` — построчный
текст для `while read` в bash-шагах workflow.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, NamedTuple

import yaml

# Валидное имя переменной *_TAG в docker/{main,vision}/.image-versions.* —
# заглавные буквы/цифры/подчёркивания, обязательно оканчивается на _TAG.
_IMAGE_VERSIONS_VAR_RE = re.compile(r"^[A-Z][A-Z0-9_]*_TAG$")


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

    for pi_name, pi_data in manifest["pis"].items():
        services = (pi_data or {}).get("services") or {}
        _validate_services(pi_name, services)
        # topo_layers сам кидает ManifestError при цикле — валидируем
        # структуру графа зависимостей здесь же, при загрузке, а не только
        # по требованию потребителя chained/topo_layers.
        topo_layers(services)

    return manifest


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

        for dep in depends_on:
            if dep not in services:
                raise ManifestError(
                    f"pis.{pi_name}.services.{name}: depends_on references unknown "
                    f"service {dep!r}"
                )


# --- выборка сервисов -----------------------------------------------------


def services_for_pi(manifest: dict[str, Any], pi: str) -> dict[str, Any]:
    """Возвращает {имя_сервиса: описание} для одного pi (vision|main)."""
    try:
        pi_data = manifest["pis"][pi]
    except KeyError as exc:
        raise ManifestError(
            f"manifest has no pis.{pi} (known: {sorted(manifest.get('pis', {}))})"
        ) from exc
    return dict((pi_data or {}).get("services") or {})


def independent_services(services: dict[str, Any]) -> list[dict[str, Any]]:
    """Сервисы без depends_on — кандидаты в matrix-job (вариант B, §3.4).

    Порядок — по имени сервиса (детерминированный вывод для diff'а/тестов).
    """
    out = []
    for name in sorted(services):
        svc = services[name]
        if not svc.get("depends_on"):
            out.append({"name": name, **_without(svc, "depends_on")})
    return out


def chained_services(services: dict[str, Any]) -> list[dict[str, Any]]:
    """Сервисы С depends_on, топологически отсортированные (§3.4, вариант B).

    Потребитель (workflow-шаблон) проходит список по порядку и знает, что
    зависимости уже "эмитированы" раньше — именно поэтому порядок, а не
    просто множество, важен для этого режима.
    """
    layers = topo_layers(services)
    ordered: list[dict[str, Any]] = []
    for layer in layers:
        for name in layer:
            svc = services[name]
            if svc.get("depends_on"):
                ordered.append({"name": name, **svc})
    return ordered


def topo_layers(services: dict[str, Any]) -> list[list[str]]:
    """Топологическая сортировка depends_on по уровням (алгоритм Кана).

    Уровень 0 — сервисы без зависимостей, уровень k — сервисы, все
    зависимости которых лежат на уровнях < k. Задел на вариант A ("волны",
    план §3.3) — сегодня используется только для валидации отсутствия
    циклов при загрузке манифеста (load_manifest) и в chained_services,
    не для эмиссии wave-job'ов (то не входит в Phase 1).

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


def dependency_targets(services: dict[str, Any]) -> set[str]:
    """Сервисы, на которые кто-то ссылается через depends_on.

    Они не могут жить внутри matrix-job'а: `needs:` в GitHub Actions
    ссылается на job ID, а не на элемент матрицы (план §3.1), поэтому
    цель ребра обязана остаться отдельным именованным job'ом — иначе
    зависимый сервис пришлось бы вешать на ВЕСЬ matrix-job, огрубляя
    граф (ровно то, что §3.4 плана называет минусом варианта A).
    """
    targets: set[str] = set()
    for svc in services.values():
        targets.update(svc.get("depends_on") or [])
    return targets


def matrix_service_names(services: dict[str, Any]) -> list[str]:
    """Сервисы для matrix-job: без depends_on И не являющиеся целью ребра."""
    targets = dependency_targets(services)
    return [
        name
        for name in sorted(services)
        if not services[name].get("depends_on") and name not in targets
    ]


def named_service_names(services: dict[str, Any]) -> list[str]:
    """Сервисы, которые остаются именованными job'ами (вариант B, §3.4).

    Это весь граф зависимостей целиком: и те, у кого есть depends_on, и
    те, на кого ссылаются. Порядок — топологический, чтобы потребитель
    (workflow) мог эмитить job'ы сверху вниз и знать, что `needs:`
    указывает на уже объявленный job.
    """
    targets = dependency_targets(services)
    ordered: list[str] = []
    for layer in topo_layers(services):
        for name in layer:
            if services[name].get("depends_on") or name in targets:
                ordered.append(name)
    return ordered


def _without(d: dict[str, Any], *keys: str) -> dict[str, Any]:
    return {k: v for k, v in d.items() if k not in keys}


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


def build_context(
    manifest: dict[str, Any],
    *,
    docker_tag: str,
    image_prefix: str | None = None,
    local_prefix: str | None = None,
    ros_distro: str | None = None,
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
    return BuildContext(
        image_prefix=image_prefix or registry["ghcr"],
        local_prefix=local_prefix or registry["local"],
        base_registry=defaults["base_registry"],
        ros_distro=ros_distro or defaults["ros_distro"],
        apt_proxy=defaults["apt_proxy"],
        docker_tag=docker_tag,
    )


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


def matrix_entry(
    name: str, services: dict[str, Any], ctx: BuildContext
) -> dict[str, str]:
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
    }


def matrix_entries(services: dict[str, Any], ctx: BuildContext) -> list[dict[str, str]]:
    """matrix-job: список элементов для `strategy.matrix.include`."""
    return [matrix_entry(name, services, ctx) for name in matrix_service_names(services)]


def named_entries(
    services: dict[str, Any], ctx: BuildContext
) -> dict[str, list[dict[str, str]]]:
    """Именованные job'ы: {имя_сервиса: [единственный элемент matrix]}.

    Почему словарь со списком из одного элемента, а не плоский список
    (как предполагал план §4.2): именованный job подставляет себе
    `strategy.matrix.include: fromJSON(...)['voice-base']`, то есть
    обращается к СВОЕМУ элементу по имени. Обращение по индексу списка
    (`[0]`, `[1]`) было бы тихой миной: добавление сервиса в цепочку
    сдвинуло бы индексы, и job собрал бы чужой сервис, не упав.
    Список из одного элемента — потому что `include` требует массив;
    тело такого job'а получается дословно тем же шаблоном, что и у
    общего matrix-job'а (требование плана §5 Phase 2 п.3).
    """
    return {
        name: [matrix_entry(name, services, ctx)]
        for name in named_service_names(services)
    }


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
        required=True,
        choices=["vision", "main"],
        help="Для какого Pi генерировать вывод",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["matrix", "chained", "tags", "services"],
        help=(
            "matrix — сервисы вне графа зависимостей, элементы для "
            "strategy.matrix.include; "
            "chained — {имя: [элемент]} для именованных job'ов графа "
            "зависимостей; "
            "tags — {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса} для .image-versions.* "
            "(с --format shell — строки `ПЕРЕМЕННАЯ <tag>`); "
            "services — все сервисы Pi (для сводки в summary-job'е)"
        ),
    )
    parser.add_argument(
        "--docker-tag",
        default=None,
        help=(
            "needs.prepare.outputs.docker_tag (dev|test|latest|local). "
            "Обязателен для --mode matrix/chained и для --mode tags "
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
        services = services_for_pi(manifest, args.pi)

        needs_tag = args.mode in ("matrix", "chained") or (
            args.mode == "tags" and args.format == "shell"
        )
        if needs_tag and not args.docker_tag:
            raise ManifestError(f"--mode {args.mode} requires --docker-tag")

        if args.mode in ("matrix", "chained") or needs_tag:
            ctx = build_context(
                manifest,
                docker_tag=args.docker_tag,
                image_prefix=args.image_prefix,
                local_prefix=args.local_prefix,
                ros_distro=args.ros_distro,
            )

        if args.mode == "matrix":
            result: Any = matrix_entries(services, ctx)
        elif args.mode == "chained":
            result = named_entries(services, ctx)
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
