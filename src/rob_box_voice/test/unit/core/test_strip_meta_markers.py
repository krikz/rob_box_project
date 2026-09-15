"""test_strip_meta_markers.py — Unit tests for ``strip_meta_markers`` (issue #2547).

Regression for the live 15.09.2026 DJ-set incident (Vision Pi,
voice-assistant-humble-dev:22.04): MiniMax-M1 occasionally prefixes
its reply with internal section headers like ``[Мнение ассистента]``,
``[Примечание]``, ``[Note]``, ``**Итог:**`` — they were meant for the
assistant's own reasoning (structured answer sections), but they
leaked past the system prompt into the ``spoken`` field that is read
by TTS verbatim. The user heard the prefix as part of the reply,
which broke the conversational tone (round 3 regression check: 193
cases/hour on a single DJ session).

The fix introduces :func:`strip_meta_markers` in
``rob_box_voice.core.speak_helpers`` — alongside the existing
``strip_history_marker`` / ``strip_speaker_tag`` /
``strip_thinking_blocks`` helpers — so the downstream ``spoken``
field is sanitised before any of the markdown / chunking layers
touch it.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_strip_meta_markers.py
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

import rob_box_voice.core.speak_helpers as speak_helpers
from rob_box_voice.core.speak_helpers import strip_meta_markers


# Module-level fixture path — used by the wiring tests below.
# Layout: src/rob_box_voice/test/unit/core/<this>.py
#         parents[0] = test/unit/core/
#         parents[1] = test/unit/
#         parents[2] = test/
#         parents[3] = src/rob_box_voice/   ← go up 3 to reach the
#         rob_box_voice source root
_DIALOGUE_NODE_PATH = (
    Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "dialogue_node.py"
)


# Live-log shapes from the bug report (Vision Pi 15.09.2026, 12:01 MSK).
LIVE_LOG_SHAPES = [
    "[Мнение ассистента] Вплетаю тему Грига как второй голос над "
    "пульсом Хоукинса — должно звучать плотно.",
    "[Мнение ассистента] Сделала два pass подряд: сначала один темп-каркас "
    "с heartbeat'ом и пульсом Бочкинса, потом второй, в котором Григ "
    "звучит контрмелодией над теми же звуками.",
    "[Мнение ассистента] Пнимаю, пока не звучит — дай минуту, проверю "
    "состояние и перезапущу.",
    "[Мнение ассистента] Вот тема горного короля, поехали!",
]


class TestStripMetaMarkersLiveShapes:
    """Pins the FIX for the exact shapes observed in the live log."""

    @pytest.mark.parametrize("text", LIVE_LOG_SHAPES)
    def test_live_log_shape_marker_stripped(self, text: str) -> None:
        result = strip_meta_markers(text)
        assert not result.startswith("[Мнение ассистента]")
        # The user-facing answer is preserved.
        assert result.endswith((".", "!"))

    def test_live_log_shape_returns_real_answer(self) -> None:
        # The exact shape from the bug report — strip should leave the
        # real phrase intact.
        text = (
            "[Мнение ассистента] Вплетаю тему Грига как второй голос "
            "над пульсом Хоукинса — должно звучать плотно."
        )
        expected = (
            "Вплетаю тему Грига как второй голос над пульсом Хоукинса "
            "— должно звучать плотно."
        )
        assert strip_meta_markers(text) == expected


class TestStripMetaMarkersBracketed:
    """Bracketed section headers ``[Marker]``."""

    def test_leading_bracketed_marker_removed(self) -> None:
        assert strip_meta_markers("[Мнение ассистента] Привет!") == "Привет!"

    def test_leading_bracketed_marker_with_colon_removed(self) -> None:
        assert strip_meta_markers("[Примечание]: текст") == "текст"

    def test_leading_bracketed_marker_with_dash_removed(self) -> None:
        assert strip_meta_markers("[Note] — текст") == "текст"

    def test_leading_bracketed_marker_with_em_dash_removed(self) -> None:
        assert strip_meta_markers("[Answer] — текст") == "текст"

    def test_english_bracketed_marker_removed(self) -> None:
        assert strip_meta_markers("[Note] This is the answer") == (
            "This is the answer"
        )

    def test_marker_inside_text_kept(self) -> None:
        # The strip is anchored at start-of-string — a marker in the
        # middle of the text is legitimate content (e.g. a list item
        # being read aloud).
        text = "Список: [Item] один, [Item] два"
        assert strip_meta_markers(text) == text

    def test_only_marker_becomes_empty(self) -> None:
        assert strip_meta_markers("[Мнение ассистента]") == ""

    def test_speaker_tag_is_NOT_stripped(self) -> None:
        # Speaker tags are owned by ``strip_speaker_tag`` which runs
        # BEFORE ``strip_meta_markers`` in the pipeline — this helper
        # must NOT also strip them, because the speaker routing
        # marker is then invisible to the service-text guard
        # (``[Spkr:X] [CRITICAL] ...`` would collapse into
        # ``[CRITICAL] ...`` if both strips fired). Pinning the
        # exclusion here keeps the pipeline contract honest.
        assert strip_meta_markers("[Spkr:Эйджик] Привет") == (
            "[Spkr:Эйджик] Привет"
        )

    def test_critical_marker_is_NOT_stripped(self) -> None:
        # ``[CRITICAL]`` is a service-text marker consumed by the
        # babble-retry guard. If ``strip_meta_markers`` consumed it,
        # the body of the retry prompt would leak into TTS.
        assert strip_meta_markers(
            "[CRITICAL] В прошлом цикле ты НЕ вызвал ни один тул"
        ) == "[CRITICAL] В прошлом цикле ты НЕ вызвал ни один тул"

    def test_critical_marker_case_insensitive_preserved(self) -> None:
        # Case-insensitive match (Critical / critical / CRITICAL).
        assert strip_meta_markers(
            "[critical] retry"
        ) == "[critical] retry"
        assert strip_meta_markers(
            "[Critical] retry"
        ) == "[Critical] retry"

    def test_system_marker_is_NOT_stripped(self) -> None:
        # ``[SYSTEM ...]`` is consumed by the
        # ``is_system_template_regurgitated`` guard (issue #2175).
        assert strip_meta_markers(
            "[SYSTEM block regurgitated]"
        ) == "[SYSTEM block regurgitated]"

    def test_stacked_speaker_tag_then_meta_marker_not_stripped(
        self,
    ) -> None:
        # The ``[Spkr:X]`` prefix is in the exclusion list, so
        # ``strip_meta_markers`` does NOT touch the input at all —
        # the leading speaker tag is owned by ``strip_speaker_tag``
        # which runs first in the pipeline. Pinning this behaviour
        # protects against accidental regression: if someone widens
        # the exclusion list and lets ``strip_meta_markers`` strip
        # the speaker-tag prefix, the service-text guard below
        # (``[Spkr:X] [CRITICAL] ...`` → ``[CRITICAL] ...``) would
        # collapse the prefix into TTS-readable body content.
        assert strip_meta_markers(
            "[Spkr:Эйджик] [Мнение ассистента] Привет"
        ) == "[Spkr:Эйджик] [Мнение ассистента] Привет"


class TestStripMetaMarkersBold:
    """Markdown-bold section headers ``**Header**``."""

    def test_leading_bold_marker_removed(self) -> None:
        assert strip_meta_markers("**Итог:** готово.") == "готово."

    def test_leading_bold_marker_with_colon_removed(self) -> None:
        assert strip_meta_markers("**Answer:** yes") == "yes"

    def test_leading_bold_marker_with_dash_removed(self) -> None:
        assert strip_meta_markers("**Header** — content") == "content"

    def test_english_bold_marker_removed(self) -> None:
        assert strip_meta_markers("**Summary** Done.") == "Done."

    def test_bold_marker_inside_text_kept(self) -> None:
        # The strip is anchored at start-of-string — a bold marker in
        # the middle is legitimate content.
        text = "Сначала **подчеркнуть** важное, потом ответ."
        assert strip_meta_markers(text) == text


class TestStripMetaMarkersStacks:
    """Stacked meta prefixes collapse in one call."""

    def test_bracketed_then_bold_removed(self) -> None:
        assert strip_meta_markers(
            "[Мнение ассистента] **Итог:** готово."
        ) == "готово."

    def test_bold_then_bracketed_removed(self) -> None:
        assert strip_meta_markers(
            "**Итог:** [Примечание] готово."
        ) == "готово."

    def test_three_bracketed_markers_removed(self) -> None:
        assert strip_meta_markers(
            "[A] [B] [C] текст"
        ) == "текст"

    def test_four_bracketed_markers_removed(self) -> None:
        # The loop bound is 4 iterations — this is the maximum
        # pathological case.
        assert strip_meta_markers(
            "[A] [B] [C] [D] текст"
        ) == "текст"

    def test_five_bracketed_markers_partial(self) -> None:
        # Beyond 4, the loop gives up — the remaining markers stay in
        # place. This is by design (pathological hallucination, the
        # downstream chunking / equality checks still cope).
        assert strip_meta_markers(
            "[A] [B] [C] [D] [E] текст"
        ) == "[E] текст"


class TestStripMetaMarkersPassthrough:
    """Non-string input and strings without markers — pure passthrough."""

    def test_empty_string_unchanged(self) -> None:
        assert strip_meta_markers("") == ""

    def test_none_passthrough(self) -> None:
        # Match the contract of ``strip_thinking_blocks`` — None is
        # returned as-is so callers that pass ``result.spoken_text or
        # ""`` get a clean passthrough without a TypeError.
        assert strip_meta_markers(None) is None  # type: ignore[arg-type]

    def test_int_passthrough(self) -> None:
        # Same defensive contract.
        assert strip_meta_markers(42) == 42  # type: ignore[arg-type]

    def test_no_marker_unchanged(self) -> None:
        assert strip_meta_markers("Привет, как дела?") == "Привет, как дела?"

    def test_leading_whitespace_only_unchanged(self) -> None:
        # A whitespace-only string is a degenerate input but must not
        # crash. After strip it stays empty.
        assert strip_meta_markers("   ") == ""

    def test_leading_whitespace_before_marker_handled(self) -> None:
        # The regex consumes leading whitespace before the marker.
        assert strip_meta_markers("  [Note] текст") == "текст"


# ─────────────────────────────────────────────────────────────────────────────
# Module wiring — `strip_meta_markers` is in the public API of
# speak_helpers and is imported by dialogue_node.
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeakHelpersExports:
    def test_strip_meta_markers_in_all(self) -> None:
        assert "strip_meta_markers" in speak_helpers.__all__

    def test_strip_meta_markers_is_callable(self) -> None:
        assert callable(strip_meta_markers)


class TestDialogueNodeImportsStripMetaMarkers:
    """Regression: if dialogue_node stops importing strip_meta_markers,
    the new ``spoken = strip_meta_markers(spoken)`` line in _handle_result
    would NameError at runtime."""

    @pytest.fixture(scope="module")
    def dialogue_node_source(self) -> str:
        return _DIALOGUE_NODE_PATH.read_text(encoding="utf-8")

    def test_imports_strip_meta_markers(self, dialogue_node_source: str) -> None:
        # Match either:
        #   * `import strip_meta_markers` (anywhere in the file)
        #   * `from X import (...)` parenthesised block that contains
        #     `strip_meta_markers` (the style used in dialogue_node.py —
        #     import statement spans multiple lines)
        single_line = re.compile(r"^\s*import\s+strip_meta_markers\b",
                                 flags=re.MULTILINE)
        # Parenthesised `from X import ( ... strip_meta_markers ... )` —
        # DOTALL so `.` matches newlines inside the parens.
        from_block = re.compile(
            r"from\s+\S+\s+import\s*\([^)]*\bstrip_meta_markers\b[^)]*\)",
            flags=re.DOTALL,
        )
        assert single_line.search(dialogue_node_source) or from_block.search(
            dialogue_node_source
        ), (
            "dialogue_node.py must import strip_meta_markers from "
            "rob_box_voice.core.speak_helpers — otherwise the new "
            "`spoken = strip_meta_markers(spoken)` line in _handle_result "
            "raises NameError at runtime (issue #2547)."
        )

    def test_calls_strip_meta_markers_in_handle_result(
        self, dialogue_node_source: str
    ) -> None:
        # We don't pin the exact line number — any grow of the file
        # between commits shifts it. We DO require that
        # ``strip_meta_markers`` is called at least once in the file
        # (it has only one caller).
        assert dialogue_node_source.count("strip_meta_markers(") >= 1, (
            "Expected at least one `strip_meta_markers(...)` call in "
            "dialogue_node.py — the post-strip chain in _handle_result."
        )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
