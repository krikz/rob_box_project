#!/usr/bin/env python3
"""Unit tests for wake-word sync across stt_node / dialogue_node (issue #1252).

The bug: stt_node had 3 wake words, dialogue_node had 12 — the accept was
unreliable because STT heard variants («роберт», «рыбок», «роботс») that
stt_node (barge-in) or dialogue_node (wake gate) did not match.

These tests pin the *invariant*: the canonical list (code defaults, YAMLs,
harness DSM) must contain every known STT variant and match 1:1.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS, has_wake_word

# Историческая потеря (9ca7fb29, 21.02) + STT-искажения из e2e-исследования:
# канонический список — ВСЕ варианты, которые должны давать ПРИНЯТО.
CANONICAL_WAKE_WORDS = [
    "робок",
    "робот",
    "роббокс",
    "робокос",
    "роббос",
    "робокс",
    "робэкс",
    "робекс",
    "робакс",
    "рабокс",
    "рубокс",
    "роблокс",
    "роберт",
    "рыбок",
    "рабок",
    "робак",
    "рома",
    "бот",
    "робо",
    "роб",
    "робик",
]

REPO_ROOT = Path(__file__).resolve().parents[5]


def _yaml_wake_words(path: str) -> list[str]:
    """Читаем wake_words из per-node YAML (src + docker live)."""
    import yaml

    full = REPO_ROOT / path
    if not full.exists():
        return []
    data = yaml.safe_load(full.read_text(encoding="utf-8"))
    node = next(iter(data))  # stt_node / dialogue_node
    return list(data[node]["ros__parameters"].get("wake_words", []))


class TestWakeWordSync:
    """stt_node (код+yaml) = dialogue_node (все варианты) — issue #1252."""

    @pytest.mark.parametrize("word", CANONICAL_WAKE_WORDS)
    def test_every_canonical_variant_is_accepted(self, word: str) -> None:
        text = f"{word} расскажи анекдот"
        assert has_wake_word(text.lower(), DEFAULT_WAKE_WORDS) is True

    def test_dialogue_text_defaults_cover_all_canonical(self) -> None:
        assert set(CANONICAL_WAKE_WORDS) <= set(DEFAULT_WAKE_WORDS)

    # ``test_yaml_wake_words_match_canonical`` lived here and asserted that
    # four YAMLs each carried a matching copy of the list. It guarded the
    # duplication instead of removing it — and the copies still drifted:
    # the node code defaults sat at 13 spellings while these YAMLs had 21,
    # and the e2e config had the stale 13, so the test rig could not hear
    # eight spellings the robot could. The list now exists once, in
    # ``dialogue_text.DEFAULT_WAKE_WORDS``; that no YAML re-declares it is
    # checked by ``test_wake_words_are_declared_exactly_once``
    # (test/unit/core/test_dialogue_state_paths.py).
