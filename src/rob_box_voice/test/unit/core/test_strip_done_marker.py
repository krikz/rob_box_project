"""test_strip_done_marker.py — Unit tests for ``strip_done_marker`` (issue #1564).

Regression for the live 23.08 incident: every TTS reply ended with «...дан»
because the LLM appended ``\\n\\ndone`` (the agentic-cycle terminator it was
told to emit AFTER the LAST ``speak_text``) into the ``spoken`` field instead
of replacing the spoken content with it. The dialogue_node equality check
only fired when ``spoken.strip().lower()`` was EXACTLY one of the markers —
so ``"Говорю голосом надёжного мужчины.\\n\\ndone"`` slipped through and TTS
literally read «дон» at the end of every reply.

The fix introduces a pre-strip in the same pipeline as
``strip_history_marker`` / ``strip_speaker_tag`` / ``strip_thinking_blocks``
so the downstream equality check is the single source of truth for
"skip auto-TTS" vs "publish".

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_strip_done_marker.py
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.speak_helpers import strip_done_marker


class TestStripDoneMarkerTrailing:
    """Marker appended AFTER the spoken answer (the live #1564 shape)."""

    def test_double_newline_done_removed(self):
        # The exact shape from the live log:
        #   spoken='Говорю голосом надёжного мужчины.\n\ndone'
        text = "Говорю голосом надёжного мужчины.\n\ndone"
        assert strip_done_marker(text) == "Говорю голосом надёжного мужчины."

    def test_single_newline_done_removed(self):
        text = "Котёнок под лавкой лежит, мурчит.\ndone"
        assert strip_done_marker(text) == "Котёнок под лавкой лежит, мурчит."

    def test_crlf_newline_done_removed(self):
        # Some providers normalise to \r\n.
        text = "Еноты-мутанты уже в городе!\r\ndone"
        assert strip_done_marker(text) == "Еноты-мутанты уже в городе!"

    def test_done_with_trailing_dot_removed(self):
        text = "Переключаюсь на голос надёжного мужчины.\ndone."
        assert strip_done_marker(text) == "Переключаюсь на голос надёжного мужчины."

    def test_done_with_trailing_whitespace_removed(self):
        text = "Говорю.\n\ndone   "
        assert strip_done_marker(text) == "Говорю."

    def test_done_with_no_preceding_newline_not_stripped(self):
        # Intentional contract: strip is end-anchored AND requires a
        # newline before the marker. This protects against false
        # positives in legitimate phrases like "хорошо done", "well
        # done", "я давно done ждал". The LLM is taught (master prompt
        # issue #1564 rule) to put the marker on its OWN line — so the
        # inline form should not occur in practice.
        text = "Готово к работе done"
        assert strip_done_marker(text) == text


class TestStripDoneMarkerBareWithNewline:
    """Marker as the WHOLE response with trailing whitespace/newlines —
    strip leaves empty string so the downstream equality check still
    recognises it. NOTE: the marker MUST be preceded by a newline
    (or be the bare answer at the end) — this is the regex contract
    that protects against false positives like "well done" inside a
    sentence. The dialogue_node equality check covers the bare case."""

    def test_bare_done_with_leading_newlines(self):
        # LLM writes "\n\ndone" as the WHOLE answer (no spoken text).
        # strip_done_marker removes the marker; equality check downstream
        # then matches "done" → skip auto-TTS.
        assert strip_done_marker("\n\ndone") == ""

    def test_bare_done_with_surrounding_whitespace(self):
        assert strip_done_marker("  \ndone\n  ") == ""

    def test_bare_task_complete(self):
        assert strip_done_marker("\ntask complete") == ""

    def test_bare_task_complete_underscore(self):
        assert strip_done_marker("\ntask_complete") == ""

    def test_bare_gotovo(self):
        assert strip_done_marker("\nготово") == ""

    def test_bare_gotova(self):
        assert strip_done_marker("\nготова") == ""

    def test_bare_vsyo(self):
        assert strip_done_marker("\nвсё") == ""

    def test_bare_vypolneno(self):
        assert strip_done_marker("\nвыполнено") == ""

    def test_bare_zaversheno(self):
        assert strip_done_marker("\nзавершено") == ""


class TestStripDoneMarkerCaseInsensitive:
    """The LLM capitalises the marker inconsistently — all must match."""

    def test_uppercase_done(self):
        assert strip_done_marker("Говорю.\n\nDONE") == "Говорю."

    def test_titlecase_done(self):
        assert strip_done_marker("Говорю.\nDone.") == "Говорю."

    def test_mixed_case_task_complete(self):
        assert strip_done_marker("Готово.\nTask Complete") == "Готово."

    def test_uppercase_gotovo(self):
        assert strip_done_marker("Сделал.\n\nГОТОВО") == "Сделал."


class TestStripDoneMarkerNegative:
    """Inputs that MUST be left untouched (no marker, or marker in middle)."""

    def test_no_marker_unchanged(self):
        text = "Просто обычный ответ без всяких маркеров."
        assert strip_done_marker(text) == text

    def test_marker_in_middle_not_stripped(self):
        # The strip is end-anchored — a marker inside the answer is
        # a real word, not a terminator.
        text = "DONE — это мой любимый трек!"
        assert strip_done_marker(text) == text

    def test_marker_substring_not_stripped(self):
        # «готово» as part of «приготовить» must NOT be stripped.
        text = "Я приготовил ужин."
        assert strip_done_marker(text) == text

    def test_word_ending_with_done_not_stripped(self):
        # English words that happen to END with "done" must NOT be stripped.
        text = "Он это давно сделал"
        assert strip_done_marker(text) == text

    def test_well_done_not_stripped(self):
        # Common phrase — not the cycle marker (marker needs a newline before it).
        text = "Хорошо, well done!"
        assert strip_done_marker(text) == text

    def test_empty_string(self):
        assert strip_done_marker("") == ""

    def test_whitespace_only(self):
        assert strip_done_marker("   \n\n  ") == ""


class TestStripDoneMarkerNonString:
    """Defensive contract — non-string inputs pass through unchanged."""

    def test_none_passthrough(self):
        assert strip_done_marker(None) is None  # type: ignore[arg-type]

    def test_int_passthrough(self):
        assert strip_done_marker(42) == 42  # type: ignore[arg-type]

    def test_list_passthrough(self):
        # Не наша забота, но контракт такой же как у strip_markdown.
        assert strip_done_marker(["done"]) == ["done"]  # type: ignore[arg-type]


class TestStripDoneMarkerOrder:
    """Sanity check — strip only removes the TRAILING marker,
    leaves the spoken answer untouched otherwise."""

    def test_preserves_inner_newlines(self):
        text = "Первая строка.\nВторая строка.\n\ndone"
        assert strip_done_marker(text) == "Первая строка.\nВторая строка."

    def test_preserves_inner_punctuation(self):
        # Inline "done" without preceding newline is NOT a marker — leave it.
        text = "Что? Готово?! Нет, ещё нет... done"
        assert strip_done_marker(text) == text

    def test_preserves_inner_punctuation_with_newline_marker(self):
        # Same phrase, but marker properly isolated on its own line — strip.
        text = "Что? Готово?! Нет, ещё нет...\ndone"
        assert strip_done_marker(text) == "Что? Готово?! Нет, ещё нет..."

    def test_preserves_emojis(self):
        text = "Сделал! 🎉\n\ndone"
        assert strip_done_marker(text) == "Сделал! 🎉"
