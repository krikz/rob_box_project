"""Владельцы тулов доходят до каждого среза явно: ни одного вызова «по умолчанию ядра» в рантайме (ADR-0167 Ш1в).

Без владельцев (``owners=None``) ядро нейтрально - тулов продукта нет (``owners.OWNERS_WITHOUT_PROFILE`` пуст). Хост,
который забыл передать владельцев профиля, молча оставил бы модель без тулов (робот «не умеет» ехать и петь). Поэтому
каждый вызов среза в коде ядра, хостов робота и стендов передаёт владельцев: позиционно или ключом ``owners``. Сессия
получает их при создании (``Session(..., owners=...)``) и раздаёт срезу через снимок (``SessionSnapshot.owners``).

Известные исключения - :data:`KNOWN_IMPLICIT` с числом мест (только вниз).
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Dict, Iterator, Tuple

import pytest

REPO = Path(__file__).resolve().parents[3]
ROOTS = (
    REPO / "src" / "rob_box_dialog" / "rob_box_dialog",
    REPO / "src" / "rob_box_voice" / "rob_box_voice",
    REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor",
    REPO / "scripts",
)

#: Функция → индекс позиционного аргумента с владельцами (``None`` - только ключом ``owners``).
OWNERS_PARAM: Dict[str, object] = {
    "tools_for": 1,
    "widen_for": 3,
    "form_tools": 3,
    "wrapper_for": 3,
    "CommandModeLLM": 4,
    "assemble_llm": None,
    "operator_prompt": 2,
    "Session": None,
}

#: ``файл относительно корня репо`` → число мест без владельцев (почему ещё здесь).
KNOWN_IMPLICIT: Dict[str, int] = {
    # ячейка бенчмарка адаптеров: ``answer`` без ``widen`` - срез не расширяется, тулы ячейки даёт сам бенчмарк
    "scripts/dialog/adapter_benchmark.py": 1,
}


def _calls(path: Path) -> Iterator[Tuple[int, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if name not in OWNERS_PARAM or (name == "Session" and not isinstance(node.func, ast.Name)):
            continue  # ``requests.Session()`` - не сессия диалога
        index = OWNERS_PARAM[name]
        passed = any(k.arg == "owners" for k in node.keywords) or (isinstance(index, int) and len(node.args) > index)
        if not passed:
            yield node.lineno, name


def _implicit() -> Dict[str, list]:
    found: Dict[str, list] = {}
    for root in ROOTS:
        for path in sorted(root.rglob("*.py")):
            if "test" in path.relative_to(root).parts:
                continue
            hits = list(_calls(path))
            if hits:
                found[path.relative_to(REPO).as_posix()] = hits
    return found


pytestmark = pytest.mark.skipif(not ROOTS[1].is_dir(), reason="нет src/rob_box_voice (окружение без хостов робота)")


def test_every_tool_slice_gets_the_owners_explicitly():
    found = _implicit()
    assert {name: len(hits) for name, hits in found.items()} == KNOWN_IMPLICIT, found


def test_detector_sees_an_implicit_call(tmp_path):
    sample = tmp_path / "sample.py"
    sample.write_text(
        "tools_for(spec)\n"
        "tools_for(spec, owners)\n"
        "Session('personality', clock=c, wake_words=w)\n"
        "Session('personality', clock=c, wake_words=w, owners=o)\n"
        "requests.Session()\n"
        "command_bridge.wrapper_for(log=log)\n",
        encoding="utf-8",
    )
    assert [line for line, _ in _calls(sample)] == [1, 3, 6]
