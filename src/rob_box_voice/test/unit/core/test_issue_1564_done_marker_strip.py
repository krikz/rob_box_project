"""test_issue_1564_done_marker_strip.py — Regression for issue #1564.

Live 23.08 incident: the LLM was appending ``\\n\\ndone`` to the spoken
text it returned to DialogueNode, so TTS literally read «...дан» at the
end of every reply. The downstream equality check (spoken.strip().lower()
in {done, task complete, ...}) only fired when the WHOLE field was the
marker, so the trailing suffix slipped through to TTS.

This test pins the FIX layer rather than the behaviour of
``_handle_result`` directly (which requires a lot of ROS mocking):

  * :func:`strip_done_marker` is exported from ``rob_box_voice.core.speak_helpers``
    alongside the other strip-* helpers;
  * :func:`strip_done_marker` removes the trailing newline-then-marker
    suffix in the exact shape from the live log;
  * :mod:`rob_box_voice.dialogue_node` imports ``strip_done_marker`` —
    without that import, the post-pipeline ``spoken = strip_done_marker(spoken)``
    line in ``_handle_result`` would NameError at runtime.

The integration-level test (full ``_handle_result`` with the exact
``spoken='Говорю голосом надёжного мужчины.\\n\\ndone'`` input) is
covered in production by the e2e voice test — it would be an order of
magnitude more brittle to mock here.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_issue_1564_done_marker_strip.py
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

import rob_box_voice.core.speak_helpers as speak_helpers
from rob_box_voice.core.speak_helpers import strip_done_marker


# Module-level fixture path — used by both wiring tests below.
# Layout: src/rob_box_voice/test/unit/core/<this>.py
#         parents[0] = test/unit/core/
#         parents[1] = test/unit/
#         parents[2] = test/
#         parents[3] = src/rob_box_voice/   ← go up 3 to reach rob_box_voice source root
_DIALOGUE_NODE_PATH = (
    Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "dialogue_node.py"
)


# ─────────────────────────────────────────────────────────────────────────────
# Live-log shape from issue #1564
# ─────────────────────────────────────────────────────────────────────────────

LIVE_LOG_SHAPES = [
    # Exact shapes from the bug report (architect, 23.08.2026 17:55 MSK).
    "Говорю голосом надёжного мужчины.\n\ndone",
    "Под лавкой у окошка, где пылинки в лучах, лежит ко мне картошка, мурчит у меня в руках.\n\ndone",
    "Котёнок под лавкой лежит, мурчит, не тужит. Хвостом по полу стучит, хозяйку ждёт, не спит.\n\ndone",
    "Переключаюсь на голос надёжного мужчины.\n\ndone",
    "Еноты-мутанты уже в городе!\n\ndone",
]


class TestStripDoneMarkerLiveShapes:
    """Pins the FIX for the exact shapes observed in the live log."""

    @pytest.mark.parametrize("text", LIVE_LOG_SHAPES)
    def test_live_log_shape_marker_stripped(self, text: str) -> None:
        # Strip removes the trailing "\n\ndone" suffix; what remains is
        # the user-facing answer that should reach TTS.
        result = strip_done_marker(text)
        assert not result.lower().rstrip().endswith("done")
        assert not result.endswith("\n")
        # The real spoken content is preserved.
        assert "\n\n" not in result

    def test_live_log_shape_returns_real_answer(self) -> None:
        # The exact string from the bug report — strip should leave the
        # actual phrase intact.
        assert strip_done_marker(
            "Говорю голосом надёжного мужчины.\n\ndone"
        ) == "Говорю голосом надёжного мужчины."

    def test_live_log_shape_with_bang(self) -> None:
        assert strip_done_marker(
            "Еноты-мутанты уже в городе!\n\ndone"
        ) == "Еноты-мутанты уже в городе!"


# ─────────────────────────────────────────────────────────────────────────────
# Module wiring — `strip_done_marker` is in the public API of speak_helpers
# and is imported by dialogue_node.
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeakHelpersExports:
    def test_strip_done_marker_in_all(self) -> None:
        assert "strip_done_marker" in speak_helpers.__all__

    def test_strip_done_marker_is_callable(self) -> None:
        assert callable(strip_done_marker)


class TestDialogueNodeImportsStripDoneMarker:
    """Regression: if dialogue_node stops importing strip_done_marker,
    the new ``spoken = strip_done_marker(spoken)`` line at the head of
    the done-marker block in _handle_result would NameError at runtime."""

    @pytest.fixture(scope="module")
    def dialogue_node_source(self) -> str:
        return _DIALOGUE_NODE_PATH.read_text(encoding="utf-8")

    def test_imports_strip_done_marker(self, dialogue_node_source: str) -> None:
        # Match either:
        #   * `import strip_done_marker` (anywhere in the file)
        #   * `from X import (...)` parenthesised block that contains
        #     `strip_done_marker` (the style used in dialogue_node.py —
        #     import statement spans multiple lines)
        single_line = re.compile(r"^\s*import\s+strip_done_marker\b",
                                 flags=re.MULTILINE)
        # Parenthesised `from X import ( ... strip_done_marker ... )` —
        # DOTALL so `.` matches newlines inside the parens.
        from_block = re.compile(
            r"from\s+\S+\s+import\s*\([^)]*\bstrip_done_marker\b[^)]*\)",
            flags=re.DOTALL,
        )
        assert single_line.search(dialogue_node_source) or from_block.search(
            dialogue_node_source
        ), (
            "dialogue_node.py must import strip_done_marker from "
            "rob_box_voice.core.speak_helpers — otherwise the new "
            "`spoken = strip_done_marker(spoken)` line in _handle_result "
            "raises NameError at runtime (issue #1564)."
        )

    def test_calls_strip_done_marker_in_handle_result(
        self, dialogue_node_source: str
    ) -> None:
        # We don't pin the exact line number — any grow of the file
        # between commits shifts it. We DO require that ``strip_done_marker``
        # is called at least once in the file (it has only one caller).
        assert dialogue_node_source.count("strip_done_marker(") >= 1, (
            "Expected at least one `strip_done_marker(...)` call in "
            "dialogue_node.py — the post-strip chain in _handle_result."
        )
