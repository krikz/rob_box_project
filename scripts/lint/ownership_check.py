#!/usr/bin/env python3
"""Ownership guard for dialog topics (ADR-0150 section 10.3, PR-2 / #3687).

``architecture/ownership.yml`` был пуст (V22): ни у одного топика диалога не
было заявленного владельца, и «пять писателей в /voice/tts/request» нельзя
было назвать нарушением - проверить было не с чем. Теперь карта заполнена, а
этот гард в CI не даёт ей разойтись с кодом.

Два инварианта:

(а) Каждый топик с префиксом диалога (``/dialog/``, ``/voice/speech/``,
    ``/voice/expression/``, ``/voice/providers/``, ``/voice/stt/``,
    ``/voice/speaker/``, ``/voice/music/``, ``/telegram/``, ``/avatar/session``),
    который код публикует через ``create_publisher``, объявлен в
    ``ownership.yml`` с ``owner``, ``writers_allowed`` и ``readers``.

(б) Писателей в коде не больше ``writers_allowed``. Узел писателя определяется
    по файлу (``nodes.<имя>.source``); файл вне карты узлов - нарушение, а не
    молчаливый пропуск. Запись ``multiple_writers`` в
    ``runtime-baseline.json`` (рантайм-снимок аудита) обязана нести поле
    ``debt`` с номером PR, который снимает долг, и не может относиться к
    топику, где ``writers_allowed`` допускает одного писателя.

Статус ``live`` обязан совпадать с кодом: объявленный ``live`` топик без
писателя в ``src`` и ``planned``-топик, который код уже публикует, - ошибки
(карту обновляет тот PR, что меняет код).

Usage:
  python scripts/lint/ownership_check.py            # проверка (src/)
  python scripts/lint/ownership_check.py --list     # писатели топиков диалога в коде
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import sys
from pathlib import Path
from typing import Any

try:  # запуск как скрипт: каталог scripts/lint в sys.path
    import seam_without_consumer as seam
except ImportError:  # python -m unittest scripts.lint.test_ownership_check
    from scripts.lint import seam_without_consumer as seam

try:
    import yaml
except ImportError:  # pragma: no cover - в CI pyyaml ставится шагом Install linting tools
    yaml = None

REPO_ROOT = Path(__file__).resolve().parents[2]
OWNERSHIP_FILE = REPO_ROOT / "architecture" / "ownership.yml"
RUNTIME_BASELINE_FILE = REPO_ROOT / "architecture" / "runtime-baseline.json"

#: Префиксы топиков диалога, подпадающие под гард (ADR-0150 §10.3).
OWNED_PREFIXES: tuple[str, ...] = (
    "/dialog/",
    "/voice/speech/",
    "/voice/expression/",
    "/voice/providers/",
    "/voice/stt/",
    "/voice/speaker/",
    "/voice/music/",
    "/telegram/",
)
OWNED_EXACT: tuple[str, ...] = ("/avatar/session",)

#: Ручные dev-скрипты внутри src/ (не узлы робота): их публикации не писатели.
NON_NODE_SOURCES: tuple[str, ...] = ("src/rob_box_voice/scripts/",)

REQUIRED_TOPIC_FIELDS = ("owner", "writers_allowed", "readers", "status")
STATUSES = ("live", "planned")


def is_owned_topic(topic: str) -> bool:
    """True, если топик подпадает под гард владельцев."""
    return topic in OWNED_EXACT or topic.startswith(OWNED_PREFIXES)


def load_ownership(path: Path = OWNERSHIP_FILE) -> dict[str, Any]:
    """Прочитать ownership.yml; пустой/битый файл - ошибка, а не «нет данных»."""
    if yaml is None:
        raise RuntimeError("ownership_check: нужен pyyaml (pip install pyyaml)")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: ожидался словарь верхнего уровня")
    for section in ("nodes", "topics"):
        if not isinstance(data.get(section), dict):
            raise ValueError(f"{path}: секция {section!r} должна быть словарём")
    return data


def node_for_file(rel_path: str, nodes: dict[str, Any]) -> str | None:
    """Имя узла, чей ``source`` покрывает файл, или ``None``."""
    for name, node in nodes.items():
        for pattern in node.get("source") or []:
            if fnmatch.fnmatch(rel_path, pattern):
                return name
    return None


def _entry_errors(topic: str, entry: Any, nodes: dict[str, Any]) -> list[str]:
    if not isinstance(entry, dict):
        return [f"{topic}: запись должна быть словарём"]
    errors = [
        f"{topic}: нет поля {field!r}"
        for field in REQUIRED_TOPIC_FIELDS
        if field not in entry
    ]
    if errors:
        return errors
    if not entry["owner"] or entry["owner"] not in nodes:
        errors.append(f"{topic}: owner {entry['owner']!r} не объявлен в nodes")
    if entry["status"] not in STATUSES:
        errors.append(f"{topic}: status {entry['status']!r} не из {STATUSES}")
    writers = entry["writers_allowed"]
    if not isinstance(writers, list) or not writers:
        errors.append(f"{topic}: writers_allowed должен быть непустым списком")
    else:
        errors.extend(
            f"{topic}: писатель {w!r} не объявлен в nodes" for w in writers if w not in nodes
        )
    if not isinstance(entry["readers"], list):
        errors.append(f"{topic}: readers должен быть списком (пустой - если читателей нет)")
    return errors


def check_declarations(ownership: dict[str, Any]) -> list[str]:
    """Структура карты: у каждого объявленного топика есть владелец и списки."""
    nodes = ownership["nodes"]
    errors: list[str] = []
    for name, node in nodes.items():
        if not (isinstance(node, dict) and node.get("owner") and node.get("source")):
            errors.append(f"узел {name}: нужны поля owner и source")
    for topic, entry in ownership["topics"].items():
        if not is_owned_topic(topic):
            errors.append(f"{topic}: префикс вне зоны гарда (OWNED_PREFIXES), запись лишняя")
        errors.extend(_entry_errors(topic, entry, nodes))
    return errors


def _writers_by_node(
    topic: str, hits: list[Any], nodes: dict[str, Any], errors: list[str]
) -> dict[str, list[str]]:
    by_node: dict[str, list[str]] = {}
    for hit in hits:
        if hit.file.startswith(NON_NODE_SOURCES):
            continue
        node = node_for_file(hit.file, nodes)
        if node is None:
            errors.append(
                f"{topic}: публикует {hit.file}:{hit.line}, файл не входит ни в один "
                "узел nodes.*.source - объяви узел в ownership.yml"
            )
            continue
        by_node.setdefault(node, []).append(f"{hit.file}:{hit.line}")
    return by_node


def check_static(ownership: dict[str, Any], pubs: dict[str, list[Any]]) -> list[str]:
    """Инвариант (а) и статическая часть (б): писатели в коде против карты."""
    nodes, topics = ownership["nodes"], ownership["topics"]
    errors: list[str] = []
    for topic in sorted(pubs):
        if not is_owned_topic(topic):
            continue
        entry = topics.get(topic)
        if entry is None:
            sites = ", ".join(f"{h.file}:{h.line}" for h in pubs[topic])
            errors.append(f"{topic}: нет владельца в ownership.yml (публикуют: {sites})")
            continue
        if not isinstance(entry, dict) or "writers_allowed" not in entry:
            continue  # структуру ругает check_declarations
        by_node = _writers_by_node(topic, pubs[topic], nodes, errors)
        allowed = set(entry["writers_allowed"])
        for node, sites in sorted(by_node.items()):
            if node not in allowed:
                errors.append(
                    f"{topic}: писатель {node} ({sites[0]}) вне writers_allowed "
                    f"{sorted(allowed)}"
                )
        if entry.get("status") == "planned" and by_node:
            errors.append(
                f"{topic}: status planned, но код уже публикует - переведи в live "
                "и сверь writers_allowed/readers"
            )
    for topic, entry in topics.items():
        if isinstance(entry, dict) and entry.get("status") == "live" and topic not in pubs:
            errors.append(
                f"{topic}: status live, но create_publisher в src не найден - "
                "топик удалён? поставь planned или убери запись"
            )
    return errors


def check_runtime_debts(ownership: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    """Часть (б) по рантайм-снимку: multiple_writers должны быть долгом с номером PR."""
    topics = ownership["topics"]
    errors: list[str] = []
    for item in baseline.get("known", []):
        key = str(item.get("key", ""))
        if not key.startswith("multiple_writers|"):
            continue
        topic = key.split("|")[1]
        debt = str(item.get("debt") or "").strip()
        if len(debt) < 10:
            errors.append(
                f"runtime-baseline {key}: нет поля debt с номером PR, снимающего долг"
            )
        entry = topics.get(topic)
        if isinstance(entry, dict) and len(entry.get("writers_allowed") or []) < 2:
            errors.append(
                f"runtime-baseline {key}: в рантайме несколько писателей, а ownership.yml "
                "допускает одного - расширь writers_allowed или сними долг"
            )
    return errors


def scan_publishers(src: Path) -> dict[str, list[Any]]:
    """Статические ``create_publisher`` под ``src`` (литералы и константы)."""
    scan = seam.scan_files(seam._expand_targets([src]))
    return scan.pubs


def run_checks(
    ownership: dict[str, Any], pubs: dict[str, list[Any]], baseline: dict[str, Any] | None
) -> list[str]:
    errors = check_declarations(ownership)
    errors += check_static(ownership, pubs)
    if baseline is not None:
        errors += check_runtime_debts(ownership, baseline)
    return errors


def _force_utf8_stdio() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):  # pragma: no cover
            pass


def main(argv: list[str] | None = None) -> int:
    _force_utf8_stdio()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--src", type=Path, default=REPO_ROOT / "src")
    parser.add_argument("--ownership", type=Path, default=OWNERSHIP_FILE)
    parser.add_argument("--runtime-baseline", type=Path, default=RUNTIME_BASELINE_FILE)
    parser.add_argument("--list", action="store_true", help="показать писателей топиков диалога")
    args = parser.parse_args(argv)

    ownership = load_ownership(args.ownership)
    pubs = scan_publishers(args.src)
    if args.list:
        for topic in sorted(t for t in pubs if is_owned_topic(t)):
            sites = ", ".join(f"{h.file}:{h.line}" for h in pubs[topic])
            print(f"{topic}  <- {sites}")
        return 0
    baseline = None
    if args.runtime_baseline.exists():
        baseline = json.loads(args.runtime_baseline.read_text(encoding="utf-8"))
    errors = run_checks(ownership, pubs, baseline)
    owned = sum(1 for t in pubs if is_owned_topic(t))
    if errors:
        print(f"ownership_check: FAIL - {len(errors)} нарушений")
        for line in errors:
            print(f"  - {line}")
        return 1
    print(
        f"ownership_check: OK - {owned} топиков диалога в коде имеют владельца, "
        f"{len(ownership['topics'])} записей в ownership.yml"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
