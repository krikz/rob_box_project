"""Регрессия на «забыли прописать новый .json в ``package_data``».

Живой инцидент 24.09.2026, issue #2997:
PR #2983 добавил ``core/sample_fx.py`` + ``data/sample_fx.json``, но
НЕ обновил ``setup.py:package_data``. Глоб ``*.yaml`` ловит только
.yaml, явно перечислены только ``sample_loops.json`` и
``arrangement_presets.json`` — ``sample_fx.json`` не доехал до
install-дерева, ``mcp_server`` стал падать в crash-loop с
``FileNotFoundError: data/sample_fx.json`` сразу после деплоя.

Этот тест автоматически проверяет: каждый ``.json``/``.yaml``/``.jsonl*``
внутри ``rob_box_mcp_tools/data/`` либо упомянут в ``package_data``
явно, либо покрывается расширением glob (``*.yaml``). Аналогичный
тест для ``slice_policy.yaml`` упоминается в setup.py (Issue #1998),
но его самого в репо не было — этот файл заполняет ту же дыру для
всего каталога ``data/``.

Контракт:

* ``setup.py:package_data['rob_box_mcp_tools.data']`` — единственный
  источник правды о том, какие ресурсы доходят до install-дерева.
* Если в ``data/`` добавляется новый ``.json`` — он ОБЯЗАН быть либо
  явно перечислен в этом списке, либо в glob (``*.yaml``). Любой
  новый файл, не покрытый ни одним из способов, должен ломать этот
  тест.
"""

from __future__ import annotations

import ast
import fnmatch
from pathlib import Path

import pytest

PACKAGE_ROOT = (
    Path(__file__).resolve().parent.parent / "rob_box_mcp_tools"
)
DATA_DIR = PACKAGE_ROOT / "data"
SETUP_PY = Path(__file__).resolve().parent.parent / "setup.py"

#: Расширения, доставка которых регулируется ``package_data``. Любой файл
#: с таким расширением в ``data/`` ОБЯЗАН быть покрыт — иначе до install
#: не доедет (см. инцидент #2997 с sample_fx.json).
WATCHED_SUFFIXES = (".json", ".jsonl", ".jsonl.gz", ".yaml", ".yml")


def _load_package_data_globs() -> list[str]:
    """Прочитать список файлов/глобов из ``setup.py:package_data``.

    setup.py — это обычный Python-модуль; через ``ast.parse`` достаём
    литерал ``package_data`` и его вложенный список для ключа
    ``"rob_box_mcp_tools.data"``. Парсим AST вместо exec, чтобы не
    запускать setuptools во время теста (это занимает ~0.5с и тянет
    с собой build_meta).

    Returns:
        список паттернов (``*.yaml``, ``sample_loops.json`` …) как они
        записаны в setup.py. Если ключ не найден — тест должен падать
        явно (``KeyError``), а не молча возвращать ``[]``.
    """
    tree = ast.parse(SETUP_PY.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "setup":
            continue
        for kw in node.keywords:
            if kw.arg != "package_data":
                continue
            assert isinstance(kw.value, ast.Dict), (
                "setup.py:package_data должен быть литералом dict, "
                "не выражением — парсер не сможет проверить статически."
            )
            for key, value in zip(kw.value.keys, kw.value.values):
                if isinstance(key, ast.Constant) and key.value == "rob_box_mcp_tools.data":
                    assert isinstance(value, ast.List), (
                        "package_data['rob_box_mcp_tools.data'] должен быть списком."
                    )
                    return [
                        elt.value
                        for elt in value.elts
                        if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                    ]
    raise AssertionError(
        "setup.py:setup(package_data=...) с ключом 'rob_box_mcp_tools.data' "
        "не найден — этот тест нечего проверять."
    )


def _data_files() -> list[str]:
    """Все файлы в ``rob_box_mcp_tools/data/``, относительные пути."""
    if not DATA_DIR.is_dir():
        return []
    return sorted(
        p.relative_to(DATA_DIR).as_posix()
        for p in DATA_DIR.iterdir()
        if p.is_file() and p.suffix != ".pyc"
    )


def _is_covered(filename: str, patterns: list[str]) -> bool:
    """Файл покрыт одним из паттернов ``package_data``?

    Поддерживаем как явное имя (``sample_loops.json``), так и glob
    (``*.yaml``). Точное сравнение против списка шаблонов через
    :func:`fnmatch.fnmatch` — setuptools использует ту же логику.
    """
    return any(fnmatch.fnmatch(filename, pat) for pat in patterns)


@pytest.mark.parametrize(
    "filename",
    [
        pytest.param(name, id=name)
        for name in _data_files()
        if name.endswith(WATCHED_SUFFIXES)
    ],
)
def test_data_file_covered_by_package_data(filename: str) -> None:
    """Каждый файл в ``data/`` с наблюдаемым расширением покрыт ``package_data``.

    Без этой гарантии новый ресурс (как ``sample_fx.json`` в #2983)
    не доедет до install-дерева, и ``mcp_server`` упадёт в crash-loop
    сразу после деплоя с ``FileNotFoundError`` — что мы наблюдали
    24.09.2026 (issue #2997).
    """
    patterns = _load_package_data_globs()
    assert _is_covered(filename, patterns), (
        f"data/{filename} не покрыт setup.py:package_data. "
        f"Добавьте явное имя в package_data['rob_box_mcp_tools.data'] "
        f"(текущие паттерны: {patterns}). Без этого файл не попадёт в "
        f"install-дерево, и mcp_server упадёт с FileNotFoundError."
    )