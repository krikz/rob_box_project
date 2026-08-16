"""Unit tests for :mod:`rob_box_voice.core.dialogue_guards` (TD-1 extraction).

Covers the Issue #992 guard heuristics that were extracted from
``dialogue_node.py``:

* babble / metalanguage detection (Bug D),
* performance / music request detection (Bug C),
* music stop-command and vocal-request classification,
* retry prompt builders.

These are pure-Python functions — no ROS2 node required.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.dialogue_guards import (
    BABBLE_BANNED_OPENERS,
    BABBLE_PERFORMANCE_KEYWORDS,
    MUSIC_GUARD_KEYWORDS,
    MUSIC_GUARD_VOCAL_KEYWORDS,
    MUSIC_STOP_OVERRIDES,
    build_babble_retry_prompt,
    build_music_retry_prompt,
    is_metalanguage_babble,
    is_music_stop_command,
    is_vocal_request,
    user_wants_music,
    user_wants_performance,
)


# ---------------------------------------------------------------------------
# is_metalanguage_babble
# ---------------------------------------------------------------------------


class TestIsMetalanguageBabble:
    def test_detects_zachitayu(self) -> None:
        assert is_metalanguage_babble("Зачитаю рэпчик про космос!") is True
        assert is_metalanguage_babble("Зачитаю стих про дождь") is True
        assert is_metalanguage_babble("зачитаем песенку про кота") is True

    def test_detects_mogu(self) -> None:
        assert is_metalanguage_babble("Могу бит добавить, хочешь?") is True
        assert is_metalanguage_babble("могу спеть колыбельную") is True

    def test_detects_pognali(self) -> None:
        assert is_metalanguage_babble("Погнали!") is True
        assert is_metalanguage_babble("Ну что, погнали?") is True

    def test_detects_slushai_seichas(self) -> None:
        assert is_metalanguage_babble("Слушай, сейчас расскажу сказку.") is True
        assert is_metalanguage_babble("Слушай, у меня есть идея") is True

    def test_detects_pereklyuch(self) -> None:
        assert is_metalanguage_babble("Переключаюсь на рэп") is True
        assert is_metalanguage_babble("переключу тему") is True

    def test_mid_sentence_openers_are_safe(self) -> None:
        # Openers that appear AFTER the 30-char detector window are safe —
        # this is the pre-existing contract from test_issue_992_babble_guard.
        text = (
            "Сначала робот едет вперёд, потом поворачивает. "
            "Если хочешь, могу остановиться — просто скажи. "
            "А сейчас продолжу маршрут."
        )
        assert is_metalanguage_babble(text) is False
        # Short strings where the opener sits INSIDE the 30-char window
        # ARE detected — pin that behaviour too.
        assert is_metalanguage_babble("Если хочешь, могу остановиться") is True

    def test_markdown_stripped_head(self) -> None:
        # Leading "**" must not mask the opener
        assert is_metalanguage_babble("**Зачитаю рэп про космос!**") is True

    def test_empty_text(self) -> None:
        assert is_metalanguage_babble("") is False
        assert is_metalanguage_babble(None) is False  # type: ignore[arg-type]

    def test_normal_answers_pass(self) -> None:
        assert is_metalanguage_babble("Вот твой маршрут: поверни налево") is False
        assert is_metalanguage_babble("Расскажи анекдот про программиста") is False
        assert is_metalanguage_babble("Ракета мчится через тьму!") is False
        assert is_metalanguage_babble("Жил-был енотик, полоски на спинке!") is False


# ---------------------------------------------------------------------------
# user_wants_performance
# ---------------------------------------------------------------------------


class TestUserWantsPerformance:
    def test_performance_keywords(self) -> None:
        assert user_wants_performance("зачитай рэп") is True
        assert user_wants_performance("расскажи стих про осень") is True
        assert user_wants_performance("спой песню") is True
        assert user_wants_performance("сыграй джаз") is True

    def test_case_insensitive(self) -> None:
        assert user_wants_performance("ЗАЧИТАЙ РЭП") is True

    def test_non_performance(self) -> None:
        assert user_wants_performance("как дела?") is False
        assert user_wants_performance("что нового") is False

    def test_empty(self) -> None:
        assert user_wants_performance("") is False
        assert user_wants_performance(None) is False  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# user_wants_music
# ---------------------------------------------------------------------------


class TestUserWantsMusic:
    def test_music_keywords(self) -> None:
        assert user_wants_music("спой про мурку") is True
        assert user_wants_music("включи музыку") is True
        assert user_wants_music("зачитай рэп") is True
        assert user_wants_music("поставь диджея") is True

    def test_case_insensitive(self) -> None:
        assert user_wants_music("ВКЛЮЧИ МУЗЫКУ") is True

    def test_non_music(self) -> None:
        assert user_wants_music("как дела") is False
        assert user_wants_music("") is False
        assert user_wants_music(None) is False  # type: ignore[arg-type]

    def test_optional_logger_receives_diagnostics(self) -> None:
        import logging

        records: list[logging.LogRecord] = []
        handler = logging.Handler()
        handler.emit = lambda record: records.append(record)  # type: ignore[method-assign]
        logger = logging.getLogger("test_music_guard")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        assert user_wants_music("спой песню", logger=logger) is True
        assert any("wants_music=True" in r.getMessage() for r in records)

        # Broad-match diagnostic (music-ish but not in MUSIC_GUARD_KEYWORDS)
        records.clear()
        assert user_wants_music("нужен бит", logger=logger) is False
        assert any("broad_performance" in r.getMessage() for r in records)


# ---------------------------------------------------------------------------
# is_music_stop_command / is_vocal_request
# ---------------------------------------------------------------------------


class TestIsMusicStopCommand:
    def test_stop_phrases(self) -> None:
        assert is_music_stop_command("хватит диджеить") is True
        assert is_music_stop_command("выключи музыку") is True
        assert is_music_stop_command("стоп музыку") is True
        assert is_music_stop_command("убери музыку") is True

    def test_case_insensitive(self) -> None:
        assert is_music_stop_command("ВЫКЛЮЧИ МУЗЫКУ") is True

    def test_start_phrases_are_not_stop(self) -> None:
        assert is_music_stop_command("включи музыку") is False
        assert is_music_stop_command("спой песню") is False

    def test_empty(self) -> None:
        assert is_music_stop_command("") is False
        assert is_music_stop_command(None) is False  # type: ignore[arg-type]


class TestIsVocalRequest:
    def test_vocal_phrases(self) -> None:
        assert is_vocal_request("спой песню") is True
        assert is_vocal_request("пой про кота") is True
        assert is_vocal_request("песня про мурку") is True

    def test_non_vocal(self) -> None:
        assert is_vocal_request("сыграй джаз") is False
        assert is_vocal_request("") is False
        assert is_vocal_request(None) is False  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Retry prompt builders
# ---------------------------------------------------------------------------


class TestBuildBabbleRetryPrompt:
    def test_echoes_user_input(self) -> None:
        prompt = build_babble_retry_prompt("зачитай рэп про космос")
        assert "зачитай рэп про космос" in prompt

    def test_demands_tool_call(self) -> None:
        prompt = build_babble_retry_prompt("x")
        assert "execute_music_code" in prompt
        assert "[CRITICAL]" in prompt

    def test_empty_user_input(self) -> None:
        prompt = build_babble_retry_prompt("")
        assert "[CRITICAL]" in prompt


class TestBuildMusicRetryPrompt:
    def test_echoes_user_input(self) -> None:
        prompt = build_music_retry_prompt("включи бит")
        assert "включи бит" in prompt

    def test_demands_execute_music_code(self) -> None:
        prompt = build_music_retry_prompt("x")
        assert "execute_music_code" in prompt
        assert "[CRITICAL]" in prompt

    def test_empty_user_input(self) -> None:
        prompt = build_music_retry_prompt("")
        assert "[CRITICAL]" in prompt


# ---------------------------------------------------------------------------
# Smoke — keyword tuples must not silently shrink
# ---------------------------------------------------------------------------


def test_keyword_tuples_non_empty() -> None:
    assert BABBLE_BANNED_OPENERS
    assert BABBLE_PERFORMANCE_KEYWORDS
    assert MUSIC_GUARD_KEYWORDS
    assert MUSIC_GUARD_VOCAL_KEYWORDS
    assert MUSIC_STOP_OVERRIDES


def test_stop_overrides_are_caught_by_stop_detector() -> None:
    """A stop-command must be caught by ``is_music_stop_command`` whenever
    ``user_wants_music`` would also match it (the caller checks stop FIRST,
    so this ordering must hold or Bug C would re-enable music)."""
    for stop in MUSIC_STOP_OVERRIDES:
        assert is_music_stop_command(stop) is True
        if user_wants_music(stop):
            # The guard relies on the stop-check winning over the
            # music-request check in ``DialogueNode._apply_music_guard``.
            assert is_music_stop_command(stop) is True
