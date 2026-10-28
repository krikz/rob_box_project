"""Счётчик причин пропуска LLM (``core.llm_skip_reasons``): ключи — только из перечисления.

PR-15 (#3711, часть 15p): статическая сверка литеральных ключей в файле ноды v1 (гард
``scripts/lint/dialogue_skip_reasons.py`` и ``core/skip_reason_checker.py``) удалена вместе с нодой.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.llm_skip_reasons import LLMSkipReason, new_llm_skip_counter


def test_all_skip_reasons_initialize_to_zero() -> None:
    counter = new_llm_skip_counter()

    assert counter == {reason.value: 0 for reason in LLMSkipReason}


def test_unknown_skip_reason_raises_key_error() -> None:
    counter = new_llm_skip_counter()

    with pytest.raises(KeyError, match="unknown_reason"):
        counter["unknown_reason"] += 1
