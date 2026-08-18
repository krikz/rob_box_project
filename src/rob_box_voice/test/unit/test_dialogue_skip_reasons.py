from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

_CHECKER_PATH = REPO_ROOT / "scripts" / "lint" / "dialogue_skip_reasons.py"
_CHECKER_SPEC = importlib.util.spec_from_file_location(
    "dialogue_skip_reasons", _CHECKER_PATH
)
assert _CHECKER_SPEC and _CHECKER_SPEC.loader
_CHECKER = importlib.util.module_from_spec(_CHECKER_SPEC)
_CHECKER_SPEC.loader.exec_module(_CHECKER)
find_literal_skip_reasons = _CHECKER.find_literal_skip_reasons
find_unknown_skip_reasons = _CHECKER.find_unknown_skip_reasons

from rob_box_voice.core.llm_skip_reasons import LLMSkipReason, new_llm_skip_counter


def test_all_skip_reasons_initialize_to_zero() -> None:
    counter = new_llm_skip_counter()

    assert counter == {reason.value: 0 for reason in LLMSkipReason}


def test_unknown_skip_reason_raises_key_error() -> None:
    counter = new_llm_skip_counter()

    with pytest.raises(KeyError, match="unknown_reason"):
        counter["unknown_reason"] += 1


def test_dialogue_node_uses_only_declared_skip_reasons() -> None:
    dialogue_node = (
        REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
    )

    assert find_unknown_skip_reasons(dialogue_node) == []


def test_static_checker_finds_unknown_literal_key(tmp_path: Path) -> None:
    source = tmp_path / "dialogue_node.py"
    source.write_text(
        'self._llm_skipped_counter["unknown_reason"] += 1\n',
        encoding="utf-8",
    )

    assert find_literal_skip_reasons(source) == {"unknown_reason"}


def test_static_checker_ignores_comments_and_non_counter_subscripts(
    tmp_path: Path,
) -> None:
    source = tmp_path / "dialogue_node.py"
    source.write_text(
        '# self._llm_skipped_counter["unknown_reason"] += 1\n'
        'other["unknown_reason"] += 1\n',
        encoding="utf-8",
    )

    assert find_literal_skip_reasons(source) == set()
