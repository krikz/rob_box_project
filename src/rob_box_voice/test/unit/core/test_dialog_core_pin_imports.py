"""Хост не импортирует из ядра имя, которого нет в пине образа (ADR-0167 §3, #4245).

``voice-assistant`` ставит ``rob-box-dialog-core`` по тегу из Dockerfile и импортирует его из
``site-packages``, а не из ``src/`` чекаута. Код ядра в монорепо уходит вперёд тега: импорт имени,
которого в теге нет, роняет ноду при первом вызове (так ``hil_waypoint_registry`` уронил
``mcp_server``, #4819). Гард читает тег из Dockerfile и сверяет с деревом этого тега.

Ловится только ``from <модуль-файл> import <имя>``: имя должно быть связано на верхнем уровне файла
тега (определение, импорт, цель присваивания, в том числе распаковка кортежа). ``from <пакет>
import <подмодуль>`` не ловится — пустой ``__init__.py`` подмодуль не реэкспортирует, а импорт
подмодуля работает.

Сеть нужна один раз на прогон (tarball тега, ~3 МБ, публичный репо, без токена);
``ROB_BOX_DIALOG_CORE_PIN_DIR`` подставляет уже распакованный ``src/`` тега. Без сети и без
подмены тест падает, а не зеленеет и не пропускается.
"""

from __future__ import annotations

import ast
import io
import os
import re
import tarfile
import urllib.request
from pathlib import Path

_REPO = Path(__file__).resolve().parents[5]
_DOCKERFILE = _REPO / "docker" / "sound" / "voice_assistant" / "Dockerfile"
_CORE_TOPS = (
    "rob_box_core",
    "rob_box_dialog",
    "rob_box_dialog_host",
    "rob_box_harness",
    "rob_box_llm",
    "rob_box_music",
)
_HOSTS = (
    "src/rob_box_voice/rob_box_voice",
    "src/rob_box_mcp_tools/rob_box_mcp_tools",
    "src/rob_box_supervisor/rob_box_supervisor",
    "src/rob_box_quest/rob_box_quest",
    "src/rob_box_telegram/rob_box_telegram",
)
_PIN_RE = re.compile(r"rob-box-dialog-core @ git\+https://github.com/krikz/rob_box_dialog_core@([\w.]+)")


def _pin() -> str:
    match = _PIN_RE.search(_DOCKERFILE.read_text(encoding="utf-8"))
    assert match, f"пин rob-box-dialog-core не найден в {_DOCKERFILE}"
    return match.group(1)


def _targets(expr: ast.expr, out: set[str]) -> None:
    if isinstance(expr, ast.Name):
        out.add(expr.id)
    elif isinstance(expr, (ast.Tuple, ast.List)):
        for elt in expr.elts:
            _targets(elt, out)


def _bound(source: str) -> set[str]:
    """Имена, связанные на верхнем уровне модуля: их можно импортировать."""
    out: set[str] = set()
    for node in ast.parse(source).body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            out.update((alias.asname or alias.name).split(".")[0] for alias in node.names)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                _targets(target, out)
        elif isinstance(node, ast.AnnAssign):
            _targets(node.target, out)
    return out


def _pin_sources(src_root: Path) -> dict[str, str]:
    """``модуль`` → текст файла тега для модулей-файлов (не пакетов)."""
    sources: dict[str, str] = {}
    for pkg in _CORE_TOPS:
        base = src_root / pkg / pkg
        for path in base.rglob("*.py"):
            if "test" in path.relative_to(base).parts:
                continue
            rel = path.relative_to(src_root / pkg)
            module = ".".join(rel.with_suffix("").parts)
            if path.name != "__init__.py":
                sources[module] = path.read_text(encoding="utf-8-sig")
    return sources


def _host_name_imports() -> list[tuple[str, int, str, str]]:
    """``(файл, строка, модуль, имя)`` для ``from <модуль-файл> import <имя>``."""
    found: list[tuple[str, int, str, str]] = []
    for host in _HOSTS:
        for path in (_REPO / host).rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if not isinstance(node, ast.ImportFrom) or node.level or not node.module:
                    continue
                if not node.module.startswith(_CORE_TOPS):
                    continue
                for alias in node.names:
                    if alias.name != "*":
                        found.append((str(path.relative_to(_REPO)), node.lineno, node.module, alias.name))
    return found


def _missing(sources: dict[str, str]) -> list[str]:
    bound = {module: _bound(text) for module, text in sources.items()}
    return [
        f"{path}:{line} from {module} import {name}"
        for path, line, module, name in _host_name_imports()
        if module in bound and name not in bound[module]
    ]


def _fetch_pin_src(tag: str) -> Path:
    """Распаковать ``src/`` тега ядра во временный каталог (tarball публичного репо, один запрос)."""
    url = f"https://github.com/krikz/rob_box_dialog_core/archive/refs/tags/{tag}.tar.gz"
    request = urllib.request.Request(url, headers={"User-Agent": "rob-box-pin-guard"})
    with urllib.request.urlopen(request, timeout=60) as response:  # noqa: S310 — тег из своего Dockerfile
        payload = response.read()
    root = Path(os.environ.get("PYTEST_DEBUG_TEMPROOT", "/tmp")) / f"dialog-core-pin-{tag}"
    root.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
        archive.extractall(root, filter="data")
    src = next(root.glob("*/src"))
    return src


def test_hosts_import_only_names_present_in_dialog_core_pin():
    """Имя, импортированное хостом из модуля ядра, есть в файле этого модуля на теге образа."""
    override = os.environ.get("ROB_BOX_DIALOG_CORE_PIN_DIR")
    src_root = Path(override) if override else None
    if src_root is None:
        src_root = _fetch_pin_src(_pin())
    missing = _missing(_pin_sources(src_root))
    assert not missing, "хост импортирует имя, которого нет в пине образа:\n" + "\n".join(missing)
