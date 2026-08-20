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
from rob_box_voice.core.dialogue_manager import DialogueManager

# Историческая потеря (9ca7fb29, 21.02) + STT-искажения из e2e-исследования:
# канонический список — ВСЕ варианты, которые должны давать ПРИНЯТО.
CANONICAL_WAKE_WORDS = [
    "робок",
    "робот",
    "роббокс",
    "робокос",
    "роббос",
    "робокс",
    "роберт",
    "рыбок",
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
    data = yaml.safe_load(full.read_text())
    node = next(iter(data))  # stt_node / dialogue_node
    return list(data[node]["ros__parameters"].get("wake_words", []))


class TestWakeWordSync:
    """stt_node (код+yaml) = dialogue_node (все варианты) — issue #1252."""

    @pytest.mark.parametrize("word", CANONICAL_WAKE_WORDS)
    def test_every_canonical_variant_is_accepted(self, word: str) -> None:
        text = f"{word} расскажи анекдот"
        assert has_wake_word(text.lower(), DEFAULT_WAKE_WORDS) is True
        assert DialogueManager().has_wake_word(text) is True

    def test_dialogue_text_defaults_cover_all_canonical(self) -> None:
        assert set(CANONICAL_WAKE_WORDS) <= set(DEFAULT_WAKE_WORDS)

    def test_dialogue_manager_defaults_cover_all_canonical(self) -> None:
        manager = DialogueManager()
        assert set(CANONICAL_WAKE_WORDS) <= set(manager.wake_words)

    @pytest.mark.parametrize(
        "path",
        [
            "src/rob_box_voice/config/stt_node.yaml",
            "src/rob_box_voice/config/dialogue_node.yaml",
            "docker/vision/config/voice_assistant/stt_node.yaml",
            "docker/vision/config/voice_assistant/dialogue_node.yaml",
        ],
    )
    def test_yaml_wake_words_match_canonical(self, path: str) -> None:
        yaml_words = _yaml_wake_words(path)
        assert yaml_words, f"{path}: wake_words отсутствует в YAML"
        assert yaml_words == CANONICAL_WAKE_WORDS, (
            f"{path}: рассинхрон с каноническим списком:\n"
            f"  в YAML:      {yaml_words}\n"
            f"  канонически: {CANONICAL_WAKE_WORDS}"
        )

    def test_harness_dsm_accepts_robert(self) -> None:
        """Харнесс-гейт (DialogueStateMachine) должен принимать «роберт»."""
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent,
            DialogueStateMachine,
        )

        dsm = DialogueStateMachine()
        event = dsm.on_user_input("роберт расскажи анекдот")
        assert event == DialogueEvent.WAKE_WORD


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
