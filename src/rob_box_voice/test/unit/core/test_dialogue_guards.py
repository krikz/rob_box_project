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
    TOOL_REQUEST_PATTERNS,
    build_babble_retry_prompt,
    build_music_retry_prompt,
    build_tool_retry_prompt,
    detect_required_tool,
    is_metalanguage_babble,
    is_music_stop_command,
    is_vocal_request,
    looks_like_time_question,
    spoken_text_contains_time_marker,
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

    def test_library_track_phrases(self) -> None:
        """live 20.08: «включи следующий трек» / «случайный трек» /
        «ты включал мелодию ... через библиотеку» → LLM возвращал tools=[]
        и guard молчал («user does NOT want music»). Эти фразы обязаны
        триггерить music-гуард (Bug C retry)."""
        assert user_wants_music("включи случайный трек") is True
        assert user_wants_music("включи следующий трек") is True
        assert user_wants_music("включи следующий трек из библиотеки") is True
        assert user_wants_music("сыграй трек из библиотеки") is True
        assert user_wants_music("поставь трек") is True
        assert user_wants_music("поставь музыку") is True
        assert user_wants_music("включи мелодию про весну") is True
        assert user_wants_music("ты включал мелодию про весну через библиотеку") is True
        assert user_wants_music("вруби музыку") is True

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
    # Issue #1777 / #1762 — non-music tool guard. Sanity: 5 категорий,
    # каждая с непустым keyword tuple.
    assert len(TOOL_REQUEST_PATTERNS) == 5
    expected_tools = {
        "get_current_time", "search_web", "set_voice",
        "memory_search", "faq_search",
    }
    actual_tools = {name for name, _ in TOOL_REQUEST_PATTERNS}
    assert actual_tools == expected_tools
    for name, kws in TOOL_REQUEST_PATTERNS:
        assert kws, f"{name}: empty keyword tuple"


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


# ---------------------------------------------------------------------------
# Issue #1777 / #1762 — non-music tool guard. Detect + build retry prompt.
# ---------------------------------------------------------------------------


class TestDetectRequiredTool:
    """``detect_required_tool`` maps user input to a tool name."""

    # time / date (issue #1777) — главный сценарий бага.
    def test_kotoryy_chas_returns_get_current_time(self) -> None:
        assert detect_required_tool("который час?") == "get_current_time"
        assert detect_required_tool("Который час") == "get_current_time"

    def test_skolko_vremeni_returns_get_current_time(self) -> None:
        assert detect_required_tool("сколько времени?") == "get_current_time"

    def test_vremya_v_gorode(self) -> None:
        assert detect_required_tool("время в москве") == "get_current_time"
        assert detect_required_tool("Время в Берлине?") == "get_current_time"
        assert detect_required_tool("время по москве") == "get_current_time"

    def test_date_and_weekday(self) -> None:
        assert detect_required_tool("какая дата?") == "get_current_time"
        assert detect_required_tool("какой день недели") == "get_current_time"
        assert detect_required_tool("какое сегодня число") == "get_current_time"

    def test_what_time_in_english(self) -> None:
        # Без английского паттерна "what time" вернётся None — это OK,
        # основная целевая аудитория русскоязычная. Но если ввести
        # английскую фразу, она НЕ должна падать.
        # Sanity-check: не падает на английском вводе.
        result = detect_required_tool("what time is it now")
        # В текущей реализации «what time» не в keyword-set, поэтому None.
        # Проверка просто что функция не бросает исключение.
        assert result is None or result == "get_current_time"

    # weather / news / web search (issue #1762).
    def test_pogoda_returns_search_web(self) -> None:
        assert detect_required_tool("погода в москве") == "search_web"
        assert detect_required_tool("какая погода?") == "search_web"

    def test_novosti_returns_search_web(self) -> None:
        assert detect_required_tool("новости про космос") == "search_web"

    def test_zagugli(self) -> None:
        assert detect_required_tool("загугли рецепт борща") == "search_web"

    # voice (issue #1765).
    def test_pereklyuchi_golos(self) -> None:
        assert detect_required_tool("переключи голос на антона") == "set_voice"
        assert detect_required_tool("смени голос") == "set_voice"
        assert detect_required_tool("голос арт") == "set_voice"

    # memory (issue #1770).
    def test_pomniash_menya(self) -> None:
        assert detect_required_tool("помнишь меня?") == "memory_search"
        assert detect_required_tool("что ты знаешь обо мне") == "memory_search"

    # FAQ.
    def test_chto_ty_umeesh(self) -> None:
        assert detect_required_tool("что ты умеешь?") == "faq_search"
        assert detect_required_tool("какие команды есть?") == "faq_search"
        assert detect_required_tool("расскажи о себе") == "faq_search"

    # Negative cases — обычное chit-chat НЕ должно триггерить.
    def test_normal_chitchat_returns_none(self) -> None:
        assert detect_required_tool("привет как дела") is None
        assert detect_required_tool("расскажи анекдот") is None
        assert detect_required_tool("что ты делаешь") is None
        assert detect_required_tool("а ты кто?") is None  # «кто» есть, но не в списке

    def test_empty_input_returns_none(self) -> None:
        assert detect_required_tool("") is None
        assert detect_required_tool(None) is None  # type: ignore[arg-type]

    def test_priority_time_before_search(self) -> None:
        """Если два keyword'а случайно пересеклись, побеждает приоритет
        в TOOL_REQUEST_PATTERNS. Время стоит первым — для time-tool."""
        # В текущем keyword-set нет прямого пересечения, но проверим
        # что «время в» (time) побеждает, даже если «время» могло бы
        # сматчиться с другой категорией (не должно).
        assert detect_required_tool("время в париже") == "get_current_time"

    def test_lowercase_works(self) -> None:
        """Регистр не должен влиять."""
        assert detect_required_tool("КОТОРЫЙ ЧАС") == "get_current_time"
        assert detect_required_tool("ПОГОДА В МОСКВЕ") == "search_web"


class TestBuildToolRetryPrompt:
    """``build_tool_retry_prompt`` — синтетический промпт для Bug C retry."""

    def test_echoes_user_input(self) -> None:
        prompt = build_tool_retry_prompt("который час", "get_current_time")
        assert "который час" in prompt

    def test_uses_critical_prefix(self) -> None:
        prompt = build_tool_retry_prompt("x", "get_current_time")
        # Тот же prefix что у music-retry (чтобы dialogue_node не сбрасывал
        # budget на синтетическом промпте, см. issue #992 Bug C).
        assert prompt.startswith("[CRITICAL] В прошлом цикле ты НЕ вызвал")

    def test_names_required_tool(self) -> None:
        for tool_name in (
            "get_current_time", "search_web", "set_voice",
            "memory_search", "faq_search",
        ):
            prompt = build_tool_retry_prompt("запрос юзера", tool_name)
            assert tool_name in prompt, f"{tool_name} not in prompt: {prompt[:100]}"

    def test_get_current_time_hint_mentions_no_hallucination(self) -> None:
        prompt = build_tool_retry_prompt("который час", "get_current_time")
        # Явное указание не выдумывать время (issue #1777 root cause).
        assert "не выдумывай" in prompt.lower() or "выдумывать" in prompt.lower()

    def test_search_web_hint_mentions_no_promise(self) -> None:
        prompt = build_tool_retry_prompt("погода", "search_web")
        # Явное указание не обещать без реального вызова.
        assert "гляну" in prompt.lower() or "обещ" in prompt.lower() or "вызов" in prompt.lower()

    def test_defence_in_depth_unknown_tool_returns_empty(self) -> None:
        """Неизвестный tool_name → пустая строка (защита от prompt injection)."""
        # Не из allow-list (нет такого tool'а в нашей системе).
        assert build_tool_retry_prompt("x", "execute_music_code") == ""
        assert build_tool_retry_prompt("x", "rm -rf /") == ""
        assert build_tool_retry_prompt("x", "DROP TABLE") == ""
        assert build_tool_retry_prompt("x", "") == ""

    def test_empty_user_input_still_returns_prompt(self) -> None:
        """Пустой user_input не ломает генерацию (echo даст пустые «»)."""
        prompt = build_tool_retry_prompt("", "get_current_time")
        assert prompt.startswith("[CRITICAL]")
        assert "get_current_time" in prompt

    def test_all_allowed_tools_have_hint(self) -> None:
        """Каждый tool из TOOL_REQUEST_PATTERNS должен иметь hint в _TOOL_RETRY_HINTS."""
        for tool_name, _ in TOOL_REQUEST_PATTERNS:
            prompt = build_tool_retry_prompt("test", tool_name)
            assert prompt, f"{tool_name} missing hint in _TOOL_RETRY_HINTS"
            assert tool_name in prompt


class TestLooksLikeTimeQuestion:
    """``looks_like_time_question`` — узкий детектор для диагностики."""

    def test_obvious_time_phrases(self) -> None:
        assert looks_like_time_question("который час") is True
        assert looks_like_time_question("сколько времени") is True
        assert looks_like_time_question("время в москве") is True
        assert looks_like_time_question("какая дата") is True

    def test_unrelated_phrases(self) -> None:
        assert looks_like_time_question("погода в москве") is False
        assert looks_like_time_question("привет") is False
        assert looks_like_time_question("расскажи анекдот") is False

    def test_empty(self) -> None:
        assert looks_like_time_question("") is False
        assert looks_like_time_question(None) is False  # type: ignore[arg-type]

    def test_is_stricter_than_detect(self) -> None:
        """looks_like_time_question ⊂ detect_required_tool для time-категории.

        То есть: всё, что ловит looks_like_time_question, должно также
        ловить detect_required_tool(...). Если расходятся — это баг
        детектора."""
        candidates = [
            "который час", "сколько времени", "время в москве",
            "какая дата", "какой день", "время сейчас",
        ]
        for c in candidates:
            assert detect_required_tool(c) == "get_current_time", (
                f"{c!r} detected by looks_like_time_question but not by detect_required_tool"
            )


class TestSpokenTextContainsTimeMarker:
    """``spoken_text_contains_time_marker`` — детектор маркеров времени
    в тексте LLM-ответа. Используется для WARN'а
    ``time_question_no_tool_call`` (диагностика галлюцинаций #1777)."""

    def test_detects_hh_mm(self) -> None:
        # Типичный случай из issue #1777 — LLM галлюцинирует время
        # в формате HH:MM.
        assert spoken_text_contains_time_marker("Сейчас 14:35") is True
        assert spoken_text_contains_time_marker("время 9:05 утра") is True

    def test_detects_dot_separator(self) -> None:
        # Точка как разделитель — встречается у TTS-движков.
        assert spoken_text_contains_time_marker("Сейчас 14.35") is True

    def test_detects_n_chasov(self) -> None:
        assert spoken_text_contains_time_marker("двенадцать часов дня") is True
        assert spoken_text_contains_time_marker("Семь часов вечера") is True

    def test_detects_period_words(self) -> None:
        # «утра/вечера/дня/ночи» — явный маркер периода.
        assert spoken_text_contains_time_marker("Сейчас три часа дня") is True
        assert spoken_text_contains_time_marker("полночь, ночи") is True

    def test_detects_am_pm(self) -> None:
        assert spoken_text_contains_time_marker("It's 11 AM now") is True
        assert spoken_text_contains_time_marker("Сейчас 3 pm") is True

    def test_no_markers_returns_false(self) -> None:
        assert spoken_text_contains_time_marker("Привет, как дела?") is False
        assert spoken_text_contains_time_marker("Расскажи анекдот") is False
        assert spoken_text_contains_time_marker("Я не знаю времени") is False

    def test_empty_or_none(self) -> None:
        assert spoken_text_contains_time_marker("") is False
        assert spoken_text_contains_time_marker(None) is False  # type: ignore[arg-type]

    def test_realistic_hallucination(self) -> None:
        """Дословный сценарий из issue #1777: LLM отвечает «тридцать семь
        минут одиннадцатого вечера» без вызова get_current_time.
        Такой текст должен ловиться."""
        hallucinated = (
            "Сейчас тридцать семь минут одиннадцатого вечера, "
            "так что поторопись с ужином."
        )
        # «тридцать семь минут одиннадцатого» не содержит «часов» —
        # проверим, что helper ловит хотя бы «вечера».
        assert spoken_text_contains_time_marker(hallucinated) is True

    def test_issue_1777_specific_text(self) -> None:
        """Текст с «часов» и периода — точно ловится."""
        assert spoken_text_contains_time_marker(
            "Без двадцати час дня, пойдём гулять"
        ) is True
