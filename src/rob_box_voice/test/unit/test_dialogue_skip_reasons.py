from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_voice.core.llm_skip_reasons import LLMSkipReason, new_llm_skip_counter
from rob_box_voice.core.skip_reason_checker import (
    find_literal_skip_reasons,
    find_unknown_skip_reasons,
)


def test_all_skip_reasons_initialize_to_zero() -> None:
    counter = new_llm_skip_counter()

    assert counter == {reason.value: 0 for reason in LLMSkipReason}


def test_unknown_skip_reason_raises_key_error() -> None:
    counter = new_llm_skip_counter()

    with pytest.raises(KeyError, match="unknown_reason"):
        counter["unknown_reason"] += 1


def test_dialogue_node_uses_only_declared_skip_reasons() -> None:
    voice_package_root = Path(__file__).resolve().parents[2]
    dialogue_node = voice_package_root / "rob_box_voice" / "dialogue_node.py"

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
