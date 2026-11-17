"""Таблица вейков доходит до каждого правила явно: ни одного вызова «по умолчанию ядра» в рантайме (ADR-0167 Ш1б-2).

Правила, которые снимают вейк с реплики (грамматика Tier-1, ответы на вопросы, перебивание, срез тулов…), раньше
брали вейки робота из ядра молча (``DEFAULT_WAKE_WORDS``, когда таблицу не передали). Чтобы убрать вейки робота из
ядра и не изменить поведение робота, каждый такой вызов в коде ядра и хостов робота передаёт таблицу профиля:
позиционно или ключом ``wake_words``. Сессия получает её при создании (``Session(..., wake_words=...)``) и раздаёт
правилам через снимок.

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
    REPO / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools",
    REPO / "src" / "rob_box_supervisor" / "rob_box_supervisor",
    REPO / "scripts",
)

#: Функция → индекс позиционного аргумента с таблицей вейков (``None`` - только ключом ``wake_words``).
WAKE_PARAM: Dict[str, object] = {
    # rob_box_dialog.words
    "has_wake_word": 1,
    "strip_wake_word": 1,
    "slipped_wake_rest": 1,
    "asks_question": 1,
    "asks_reference": 1,
    "asks_robot_to_act": 1,
    "stop_verdict": 1,
    "stop_command": 1,
    "policy_word": 1,
    "wake_command": 1,
    # правила над ними
    "classify_reply": None,
    "classify_same": None,
    "classify_rename": None,
    "is_greeting_reply": 1,
    "names_generated_topic": 2,
    "bind_last_generated": None,
    "robot_asked": 1,
    "aloud_asked": 1,
    "answer_to_question": None,
    "answer_to_greeting": None,
    "question_reply": None,
    "said_same": None,
    "merge_answer": None,
    "named_self": None,
    "same_voice_step": None,
    "merge_step": None,
    "known_voice_registers": None,
    "begin_turn": 1,
    "voice_turn_context": 3,
    # владелец сессии
    "Session": None,
}

#: ``файл относительно корня репо`` → число мест без таблицы (почему ещё здесь).
KNOWN_IMPLICIT: Dict[str, int] = {}


def _calls(path: Path) -> Iterator[Tuple[int, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if name not in WAKE_PARAM or (name == "Session" and not isinstance(node.func, ast.Name)):
            continue  # ``requests.Session()`` - не сессия диалога
        index = WAKE_PARAM[name]
        passed = any(k.arg == "wake_words" for k in node.keywords) or (
            isinstance(index, int) and len(node.args) > index
        )
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


def test_every_wake_rule_gets_the_table_explicitly():
    found = _implicit()
    assert {name: len(hits) for name, hits in found.items()} == KNOWN_IMPLICIT, found


def test_detector_sees_an_implicit_call(tmp_path):
    sample = tmp_path / "sample.py"
    sample.write_text(
        "strip_wake_word(text)\n"
        "strip_wake_word(text, wake)\n"
        "classify_reply(text, asks_name=False)\n"
        "classify_reply(text, wake_words=w)\n"
        "Session('personality', clock=c)\n"
        "planner.begin_turn(text, w)\n",
        encoding="utf-8",
    )
    assert [line for line, _ in _calls(sample)] == [1, 3, 5]
