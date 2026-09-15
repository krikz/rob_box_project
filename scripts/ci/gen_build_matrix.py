#!/usr/bin/env python3
"""Читатель docker/build-manifest.yaml — единственного источника истины
о составе docker-сборки (docs/plans/2026-09-15-service-manifest.md).

Phase 1 (этот файл, см. план §4): чисто библиотечный читатель + CLI.
Ничего в CI пока не вызывает этот скрипт "по-настоящему" — workflow-файлы
(".github/workflows/L-Build Vision Pi Services.yml" /
".github/workflows/L-Build Main Pi Services.yml") продолжают перечислять
сервисы вручную. Подключение (`prepare`-job читает вывод этого скрипта
через `$GITHUB_OUTPUT`) — Phase 2/3, отдельные PR.

Библиотечные функции (без побочных эффектов, без сети, без git/docker —
это то, что делает модуль тестируемым без CI, план §4.2):

    load_manifest(path)             -> dict, распарсенный и провалидированный
    services_for_pi(manifest, pi)   -> {имя_сервиса: описание}
    independent_services(services)  -> [{"name": ..., ...}, ...] без depends_on
    chained_services(services)      -> [{"name": ..., "depends_on": [...], ...}, ...]
                                        топологически отсортировано
    topo_layers(services)           -> [[уровень0], [уровень1], ...] — алгоритм
                                        Кана; задел на вариант A (§3.3 плана),
                                        не используется в MVP, но покрыт тестом
    image_versions_map(services)    -> {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса}

CLI:

    gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode matrix
    gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode chained
    gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode tags

Каждый режим печатает JSON в stdout (план §4.2).
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

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


def _without(d: dict[str, Any], *keys: str) -> dict[str, Any]:
    return {k: v for k, v in d.items() if k not in keys}


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
        choices=["matrix", "chained", "tags"],
        help=(
            "matrix — независимые сервисы (без depends_on); "
            "chained — сервисы с depends_on, топологически отсортированные; "
            "tags — {ИМЯ_ПЕРЕМЕННОЙ: имя_сервиса} для .image-versions.*"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_cli().parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        services = services_for_pi(manifest, args.pi)

        if args.mode == "matrix":
            result: Any = independent_services(services)
        elif args.mode == "chained":
            result = chained_services(services)
        else:  # tags
            result = image_versions_map(services)
    except ManifestError as exc:
        print(f"::error::gen_build_matrix: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
