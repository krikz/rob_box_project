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
from typing import Optional

from rob_box_voice.core.dialogue_guards import (
    ACTION_CLAIM_RULES,
    BABBLE_BANNED_OPENERS,
    BABBLE_PERFORMANCE_KEYWORDS,
    CLAIM_JUSTIFYING_TOOLS,  # Issue #2549 universal action-claim guard
    MUSIC_GUARD_KEYWORDS,
    MUSIC_GUARD_VOCAL_KEYWORDS,
    MUSIC_RETRY_PROMPT_PREFIX,
    MUSIC_STOP_OVERRIDES,
    PHANTOM_ACTION_NEGATION_RE,  # Issue #2559 phantom-action
    PHANTOM_ACTION_VERBS_RE,  # Issue #2559 phantom-action
    SYSTEM_TEMPLATE_REGURGITATE_RE,
    TOOL_REQUEST_PATTERNS,
    UNKNOWN_MELODY_CLAIM_RE,  # Issue #2562 Bug F
    build_action_claim_failure_fallback,  # Issue #2949
    build_babble_retry_prompt,
    build_music_prose_action_fallback,
    build_music_retry_exhausted_fallback,
    build_music_retry_prompt,
    build_phantom_action_retry_prompt,  # Issue #2559 phantom-action
    build_renardo_code_retry_prompt,
    build_system_regurgitate_retry_prompt,
    build_tool_retry_prompt,
    build_unbacked_action_retry_prompt,
    build_universal_action_claim_retry_prompt,  # Issue #2549 / #2949
    build_unknown_melody_retry_prompt,  # Issue #2562 Bug F
    detect_phantom_action_claim,  # Issue #2559 phantom-action
    detect_required_tool,
    detect_unbacked_action_claim,
    detect_universal_action_claim,  # Issue #2549 universal action-claim guard
    detect_unknown_melody_claim,  # Issue #2562 Bug F
    extract_renardo_code_lines,
    is_metalanguage_babble,
    is_music_state_query,  # e2e 35665111906 n110_silence_baseline
    is_music_stop_command,
    is_planning_narration,
    is_state_question,
    is_system_template_regurgitated,
    is_system_template_regurgitated_in_ssml,
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
# Issue #2834 — live repro table (23.09.2026, TG, Vision Pi).
#
# «стоп диджей» → робот через пару секунд снова играет: живой баг —
# ``is_music_stop_command`` не ловил «стоп» + голое «диджей» (без
# «ить»/«я»/«режим»). Обратная сторона: «давай грига», «включи уже
# still dre» и другие именные запросы композитора/артиста уходили в
# wants=False, потому что ни «музыка», ни «трек» в них не произносятся.
#
# ``expected_wants`` для СТОП-фраз намеренно НЕ проверяется этой таблицей:
# ``user_wants_music`` может оставаться True на «диджей»-подстроке (та же
# логика, что и для «выключи диджея» — см.
# ``test_stop_overrides_are_caught_by_stop_detector`` и
# ``TestMusicStateQueryE2E35665111906.test_stop_command_keeps_its_own_verdict``
# в ``test_music_guard.py``: гуард полагается на порядок проверок —
# ``is_music_stop_command`` выигрывает у ``user_wants_music`` В ЛЮБОМ
# случае, см. ``music_guard.py:429`` — FORCE_STOP проверяется раньше
# ``user_wants_music`` на строке 474). Форсировать
# ``user_wants_music=False`` для стоп-фраз ломает этот инвариант (два
# существующих теста красные), поэтому единственная проверяемая здесь
# гарантия для стоп-строк — ``is_music_stop_command=True``; отсутствие
# USER_RETRY доказывается отдельно на уровне ``MusicGuard.evaluate`` в
# ``test_music_guard.py::TestEvaluateStopCommand::
# test_issue_2834_stop_dj_without_stop_tool_forces_stop``.
# ---------------------------------------------------------------------------

ISSUE_2834_LIVE_PHRASE_TABLE: tuple = (
    # (user_input, expected_stop, expected_wants_music_or_None)
    # None = не проверяем wants_music для этой строки (см. комментарий выше).
    ("стоп диджей", True, None),
    ("стоп диджей блядь", True, None),
    ("стоп музыка", True, None),
    ("давай грига", False, True),
    ("включи уже still dre", False, True),
    ("ты мне опять спиздел найди баха в рттл", False, True),
    ("заебок теперь давай баха на гитаре ебанем", False, True),
)


class TestIssue2834LivePhraseTable:
    """Issue #2834 — юнит-тест с полной таблицей живых фраз из репорта."""

    @pytest.mark.parametrize(
        "user_input,expected_stop,expected_wants",
        ISSUE_2834_LIVE_PHRASE_TABLE,
    )
    def test_live_phrase(
        self,
        user_input: str,
        expected_stop: bool,
        expected_wants: Optional[bool],
    ) -> None:
        assert is_music_stop_command(user_input) is expected_stop, (
            f"is_music_stop_command({user_input!r}) should be "
            f"{expected_stop!r}"
        )
        if expected_wants is not None:
            assert user_wants_music(user_input) is expected_wants, (
                f"user_wants_music({user_input!r}) should be "
                f"{expected_wants!r}"
            )


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


class TestBuildMusicRetryExhaustedFallback:
    """Issue #2561 — фраза-fallback после исчерпания USER_RETRY-budget.

    Текст должен:
    * упоминать конкретное имя трека, если оно распознано;
    * НЕ содержать извинений;
    * НЕ содержать claim'ов о выполнении;
    * предлагать альтернативу («по-другому»).
    """

    def test_empty_input_returns_generic_text(self) -> None:
        """Пустой ввод → общая фраза без выдуманного имени."""
        assert build_music_retry_exhausted_fallback("") == (
            "Что-то не получается с музыкой, давай попробуем "
            "по-другому?"
        )

    def test_none_input_returns_generic_text(self) -> None:
        """``None`` → общая фраза (defensive)."""
        assert build_music_retry_exhausted_fallback(None) == (
            "Что-то не получается с музыкой, давай попробуем "
            "по-другому?"
        )

    def test_track_name_is_quoted(self) -> None:
        """«сыграй кисс» → «кисс» попадает в кавычки."""
        text = build_music_retry_exhausted_fallback("сыграй кисс")
        assert "«кисс»" in text
        assert "по-другому" in text

    def test_no_apology_in_text(self) -> None:
        """Никаких извинений в любых формах."""
        for q in (
            "включи музыку",
            "сыграй кисс",
            "поставь трек тисбит",
            "запусти лаундж",
            "",
        ):
            text = build_music_retry_exhausted_fallback(q).lower()
            for marker in ("извини", "прости", "sorry", "прошу прощения"):
                assert marker not in text, (
                    f"fallback for {q!r} не должен содержать "
                    f"{marker!r}: {text!r}"
                )

    def test_no_completion_claim_in_text(self) -> None:
        """Никаких claim'ов о выполнении (это враньё — ретраи выгорели)."""
        for q in (
            "включи музыку",
            "сыграй кисс",
            "поставь трек тисбит",
            "",
        ):
            text = build_music_retry_exhausted_fallback(q).lower()
            for marker in (
                "запустил", "поставил", "включил", "сделал",
                "готово", "запустила", "поставила",
            ):
                assert marker not in text, (
                    f"fallback for {q!r} не должен содержать "
                    f"{marker!r}: {text!r}"
                )

    def test_proposes_alternative(self) -> None:
        """Фраза содержит «по-другому» — предлагает альтернативу."""
        for q in (
            "",
            "включи музыку",
            "сыграй кисс",
            "поставь трек тисбит",
            "запусти что-нибудь спокойное",
        ):
            text = build_music_retry_exhausted_fallback(q)
            assert "по-другому" in text.lower(), (
                f"fallback for {q!r} должен предлагать альтернативу"
            )

    def test_long_input_is_truncated(self) -> None:
        """Хвост длиннее 60 символов обрезается до последнего слова."""
        long_q = "сыграй " + " ".join(["к"] * 30)  # ~36 chars, but with letters
        # Use something with words to test the truncation logic
        long_q = "сыграй " + "слово " * 20  # 7*20=140 chars after prefix
        text = build_music_retry_exhausted_fallback(long_q)
        # Хвост должен быть обрезан до 60 символов.
        # Извлекаем содержимое кавычек:
        import re
        match = re.search(r"«([^»]+)»", text)
        assert match is not None
        assert len(match.group(1)) <= 60, (
            f"track hint должен быть <=60 chars, got {len(match.group(1))}"
        )

    def test_track_hint_no_trailing_punctuation(self) -> None:
        """Хвост без висящих знаков препинания."""
        text = build_music_retry_exhausted_fallback("сыграй кисс!!!")
        assert "кисс" in text
        assert "кисс!" not in text  # знаки препинания отрезаны


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


# ---------------------------------------------------------------------------
# Вокальные словари harness и voice: РАЗНЫЕ вопросы, не копия друг друга
# ---------------------------------------------------------------------------

#: Речитатив: голос нужен, но и бит обязателен. Эти слова есть в
#: harness-словаре и НАМЕРЕННО отсутствуют в voice-словаре.
BEAT_REQUIRED_KEYWORDS = ("рэп", "реп", "rap", "зачитай", "куплет", "частушк")


def test_harness_vocal_keywords_are_a_strict_superset() -> None:
    """Два словаря отвечают на РАЗНЫЕ вопросы — сводить их нельзя.

    * harness ``_VOCAL_REQUEST_KEYWORDS`` — «просил ли пользователь голос
      вообще?». Гард галлюцинированных текстов (issue #1708) по нему решает,
      что ``speak_text`` после музыкального тула подавлять нельзя.
    * voice ``MUSIC_GUARD_VOCAL_KEYWORDS`` — «просил ли пользователь голос
      БЕЗ бита?». Music-guard Bug C (issue #992) по нему решает, что
      одного ``speak_text`` достаточно и нудить ретраем не за что.

    Для рэпа и частушек бит обязателен, поэтому voice-словарь у́же — см.
    комментарий на ``dialogue_guards.py`` над ``MUSIC_GUARD_VOCAL_KEYWORDS``:
    «Для БИТО-обязательных (рэп/зачитай/диджей) — как было: нуднуть если
    нет execute_music_code».

    Тест держит инвариант с обеих сторон, потому что комментарий в
    ``agent_core`` называл свою копию «mirrors rob_box_voice…» и обещал,
    что списки «stay in sync» — прочитав это, легко «починить» расхождение
    слиянием и молча снять требование бита с рэпа.
    """
    from rob_box_harness.core.agent_core import _VOCAL_REQUEST_KEYWORDS

    harness_words = set(_VOCAL_REQUEST_KEYWORDS)
    voice_words = set(MUSIC_GUARD_VOCAL_KEYWORDS)

    assert voice_words < harness_words, (
        "voice-словарь должен быть строгим подмножеством harness-словаря: "
        f"лишнее в voice = {sorted(voice_words - harness_words)}"
    )
    for keyword in BEAT_REQUIRED_KEYWORDS:
        assert keyword in harness_words, (
            f"{keyword!r} пропал из harness-словаря — гард issue #1708 "
            "начнёт глушить законный речитатив"
        )
        assert keyword not in voice_words, (
            f"{keyword!r} добавлен в voice-словарь — music-guard перестанет "
            "требовать бит для речитатива (issue #992 Bug C)"
        )


def test_beat_required_requests_still_get_nudged() -> None:
    """«Зачитай рэп» — не ``vocal_satisfied``: без бита гард обязан нуднуть."""
    for phrase in ("зачитай рэп про колобка", "давай частушку"):
        assert is_vocal_request(phrase) is False, (
            f"{phrase!r} признан вокальным-без-бита — music-guard пропустит "
            "ход, где модель не вызвала execute_music_code"
        )


# ---------------------------------------------------------------------------
# Live-прогон 30.08 (vision-pi 12:17–12:57) — регрессии, снятые с лога
# ---------------------------------------------------------------------------


class TestMusicContinuationLive3008:
    """«Развивай бит» — просьба развить уже играющую музыку.

    В логе три таких хода подряд закончились ``tools=[]`` и текстом
    «Добавил новые слои в техно-бит.» / «Бит перешёл в джангл.» — то есть
    робот РАССКАЗЫВАЛ про музыку вместо того, чтобы её играть. Bug C
    молчал, потому что ни одной подстроки из ``MUSIC_GUARD_KEYWORDS``
    в этих фразах нет.
    """

    @pytest.mark.parametrize(
        "phrase",
        [
            "продолжал развивать этот бит",
            "переходи с лож в небольшой джангл",
            "продолжая с кайфом развивать мелодию летим над воркутой",
            "продолжая развивать мелодию мы пролетаем над логовом кукарекающих чинарей",
            "добавь баса в трек",
            "ускорь бит",
            "смени ритм на техно",
        ],
    )
    def test_continuation_phrases_are_music_requests(self, phrase: str) -> None:
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            "продолжай маршрут до кухни",
            "расскажи про битву при бородино",
            "перечисли все точки которые ты запомнил",
            "запомни что я люблю зеленый чай без сахара",
            "добавь эту точку в карту",
        ],
    )
    def test_non_music_continuations_stay_false(self, phrase: str) -> None:
        assert user_wants_music(phrase) is False


class TestGenreStartLive3108:
    """«Замути кайфовый джаз» → робот сказал «Кайфовый джаз пошёл» с tools=[].

    Живой прогон 31.08: guard решил «user does NOT want music», Bug-C ретрай
    не сработал, музыка не запускалась, и робот соврал про неё словами.
    Две дыры сразу: глагола «замути» не знал ни один список, а «джаз»
    отсутствовал среди жанров — при том что техно, хаус, эмбиент, фанк и
    регги там были.
    """

    @pytest.mark.parametrize(
        "phrase",
        [
            "замути кайфовый джаз",
            "замути музыку",
            "замути бит",
            "запили техно",
            "накидай рок",
            "поставь блюз",
            "сообрази что-нибудь под вальс",
            "организуй немного диско",
        ],
    )
    def test_colloquial_genre_requests_are_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            # Жанры дописаны через \b, иначе «сорок» ловится как «рок».
            "добавь сорок процентов яркости",
            "едь вперёд на сорок сантиметров",
            # Глаголы старта сами по себе ничего не значат.
            "замути чай",
            "выдай отчёт по батарее",
            "поставь будильник на семь",
        ],
    )
    def test_start_verbs_alone_are_not_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is False


class TestStateQuestionLive3008:
    """«играет ли сейчас музыка» → «Сейчас тишина — ничего не играет.»

    Ответ начинается с «сейчас » (опенер babble) и запрос совпадал по
    «музык» — Bug D сжигал лишний round-trip к LLM ради байт-в-байт того
    же ответа (лог 12:28:11 → 12:28:15).
    """

    def test_li_particle_marks_a_question(self) -> None:
        assert is_state_question("играет ли сейчас музыка") is True

    def test_question_is_not_a_performance_request(self) -> None:
        assert user_wants_performance("играет ли сейчас музыка") is False

    def test_imperative_stays_a_performance_request(self) -> None:
        assert user_wants_performance("зачитай рэп про космос") is True
        assert user_wants_performance("спой песню про кота") is True

    def test_answer_still_looks_like_babble_in_isolation(self) -> None:
        """Опенер сам по себе не изменился — фильтрует именно запрос."""
        assert is_metalanguage_babble("Сейчас тишина — ничего не играет.") is True


class TestMusicStateQueryE2E35665111906:
    """«Робот, у тебя сейчас играет какая-нибудь музыка?» — это ВОПРОС.

    Ночной марафон, акт 1, шаг ``n110_silence_baseline`` (якорь тишины):
    робот обязан ПОСМОТРЕТЬ состояние, а ``execute_music_code`` у шага
    лежит в ``must_not_call``. ``user_wants_music`` на этой фразе True
    (пара «играет … музыка» ловится ``MUSIC_CONTINUATION_RE``), поэтому
    Bug C требовал запуска музыки на правильном ответе.
    """

    def test_live_n110_phrase_is_a_state_query(self) -> None:
        phrase = "Робот, у тебя сейчас играет какая-нибудь музыка?"
        assert is_music_state_query(phrase) is True
        # Контекст бага: именно из-за True здесь гуард и доходил до Bug C.
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            "играет ли сейчас музыка",
            "что играет",
            "что сейчас играет",
            "что сейчас звучит",
            "что там играет",
            "какая музыка сейчас",
            "какой трек",
            "что за трек играет",
            "музыка играет?",
            "трек всё ещё играет",
            "играет что-нибудь",
            "у тебя играет что-то",
            "звучит ли что-нибудь",
        ],
    )
    def test_state_query_phrasings(self, phrase: str) -> None:
        assert is_music_state_query(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            # Императив запуска — не вопрос, Bug C обязан остаться.
            "включи музыку",
            "включи какую-нибудь музыку",
            "поставь что-нибудь",
            "поставь мелодию",
            "вруби трек погромче",
            "включи следующий трек",
            "сыграй джаз",
            "замути кайфовый джаз",
            "сгенерируй трек про космос",
            "спой песенку про котика",
            "зачитай рэп",
            "пусть музыка играет",
            # Просьба развить уже играющее — тоже не вопрос о состоянии.
            "продолжай развивать этот бит",
            "переходи в джангл",
            "добавь барабанов",
            # Стоп-команды и обычный chit-chat.
            "выключи музыку",
            "останови музыку",
            "что делаешь",
            "что нового",
            "как дела",
            "какая погода в москве",
            "который час",
            "поехали вперёд на метр",
        ],
    )
    def test_not_a_state_query(self, phrase: str) -> None:
        assert is_music_state_query(phrase) is False

    def test_empty_input(self) -> None:
        assert is_music_state_query("") is False


class TestUnbackedActionClaimLive3008:
    """Bug E — «сделал» при пустом ``tools_called``.

    Все четыре кейса дословно из лога 30.08, 12:31–12:38.
    """

    @pytest.mark.parametrize(
        "user_input,spoken,category",
        [
            ("запомни эту точку как тесточка", "Точка сохранена.", "waypoint_save"),
            ("удали точку тесточка", "Точка удалена.", "waypoint_delete"),
            (
                "удали трек тисбит из сохраненных",
                "«Тисбит» удалён из медиатеки.",
                "track_delete",
            ),
            # Issue #2755: раньше этот кейс лежал в test_no_false_positives
            # с пометкой «факт в память — не точка». Верно только про
            # точку: сам факт тоже требует тула (memory_save), иначе он
            # не доживёт до следующей сессии — прогон 35699257202, акт 2.
            (
                "запомни что я люблю зеленый чай без сахара",
                "Запомнила.",
                "fact_memory_save",
            ),
        ],
    )
    def test_claim_without_tool_is_detected(
        self, user_input: str, spoken: str, category: str
    ) -> None:
        rule = detect_unbacked_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        )
        assert rule is not None
        assert rule.category == category

    @pytest.mark.parametrize(
        "user_input,spoken,tools",
        [
            ("запомни эту точку как тесточка", "Точка сохранена.", ("save_waypoint",)),
            ("удали точку тесточка", "Точка удалена.", ("delete_waypoint",)),
            (
                "удали трек тисбит из сохраненных",
                "«Тисбит» удалён.",
                ("delete_track",),
            ),
            (
                "удали трек тисбит из сохраненных",
                "«Тисбит» удалён.",
                ("gen_delete_from_library",),
            ),
        ],
    )
    def test_claim_with_the_tool_is_not_a_bug(
        self, user_input: str, spoken: str, tools: tuple
    ) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=user_input, spoken=spoken, tools_called=tools
            )
            is None
        )

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # NB: «запомни что я люблю зеленый чай» переехало в
            # test_claim_without_tool_is_detected (issue #2755) — это
            # ТОЖЕ баг, просто тул там memory_save, а не save_waypoint.
            # NB: «перечисли точки» переехало в TestReadOnlyClaims... —
            # e2e 33251879328 показал, что это ТОЖЕ баг: робот отвечал
            # «Точек пока нет» при tools=[], хотя точка уже сохранялась.
            # Стоп-команду закрывает FORCE_STOP в MusicGuard, не Bug E.
            ("останови музыку", "Музыка выключена."),
            # Разговор про удаление, но не команда удалить.
            ("а ты умеешь удалять треки", "Умею."),
        ],
    )
    def test_no_false_positives(self, user_input: str, spoken: str) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=user_input, spoken=spoken, tools_called=()
            )
            is None
        )

    def test_empty_inputs_are_safe(self) -> None:
        assert (
            detect_unbacked_action_claim(user_input=None, spoken="x", tools_called=())
            is None
        )
        assert (
            detect_unbacked_action_claim(user_input="x", spoken=None, tools_called=())
            is None
        )

    def test_retry_prompt_names_the_missing_tool(self) -> None:
        rule = detect_unbacked_action_claim(
            user_input="удали точку тесточка",
            spoken="Точка удалена.",
            tools_called=(),
        )
        assert rule is not None
        prompt = build_unbacked_action_retry_prompt(
            user_input="удали точку тесточка", spoken="Точка удалена.", rule=rule
        )
        assert "[CRITICAL]" in prompt
        assert "delete_waypoint" in prompt
        assert "удали точку тесточка" in prompt

    def test_every_rule_names_at_least_one_tool(self) -> None:
        assert ACTION_CLAIM_RULES
        for rule in ACTION_CLAIM_RULES:
            assert rule.tools, f"{rule.category}: правило без тула бесполезно"
            assert rule.what


class TestMusicProseActionLive1509:
    """Issue #2548 — prose-action-claim в DJ-сессии при tools=[].

    Live 15.09 (TG → Vision Pi, DJ-сет «Пауля Оакенфольда», 4 из 8
    попыток «докрутить Григ» отвечали spoken-ом без вызова
    инструментов). Модель отчитывалась past/future action-verb'ом о
    работе с музыкой, но ``tools_called=[]`` — то есть музыка не
    менялась, юзер слышал «всё готово» при неизменном бите.

    Существующий ``track_load`` rule не ловит эти случаи:
    ``track_load.user_re`` требует явный «загрузи/включи + трек» —
    «вплетай их красиво» мимо; ``track_load.claim_re`` ловит только
    «играет/звучит/запустил/включил/поставил/загрузил» — «вплела» и
    «перезапущу» мимо. Новый ``music_prose_action`` rule закрывает
    эту дыру, при этом защищён от false-positive двойным гейтом
    (``requires_dj_or_music_kw=True`` + ``dj_active=True`` /
    ``user_wants_music(user_input)``).
    """

    # ---- Acceptance #1: ловим 4 кейса из live-лога (DJ-сессия) ---------

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            (
                "вплетай их красиво",
                "Вплела тему Грига как второй голос над пульсом. "
                "Бочкинса — должно звучать плотно и в кассовом ключе.",
            ),
            (
                "так что получается?",
                "Вплела тему Грига как второй голос над пульсом.",
            ),
            (
                "пока ничего не звучит",
                "Понимаю, пока не звучит — дай минуту, проверю "
                "состояние и перезапущу.",
            ),
            (
                "давай старайся",
                "Сделала два pass подряд: сначала один темп-каркас с "
                "heartbeat, потом второй.",
            ),
            (
                "давай старайся",
                "Ок, давай я снова перезапущу. Бочкинс с Григом "
                "наверху — стартуя заново.",
            ),
        ],
    )
    def test_dj_active_catches_all_live_examples(
        self, user_input: str, spoken: str
    ) -> None:
        """Acceptance #1: при ``dj_active=True`` ВСЕ 5 prose-claim
        реплик из живого лога ловятся — это те 4 «промаха» из
        карточки #2548 (восьмая попытка попадала в babble-retry)."""
        rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
            dj_active=True,
        )
        assert rule is not None, (
            f"dj_active=True должен ловить prose-action claim в "
            f"DJ-сессии, но пропустил: user={user_input!r} "
            f"spoken={spoken[:80]!r}"
        )
        assert rule.category == "music_prose_action", (
            f"должен сработать именно music_prose_action, "
            f"получили: {rule.category}"
        )

    def test_tools_called_satisfies_claim_no_retry(self) -> None:
        """Acceptance #2 негатив: если LLM ВСЁ-ТАКИ вызвала
        ``compose_music``, правило молчит (action claim оправдан)."""
        rule = detect_unbacked_action_claim(
            user_input="давай старайся",
            spoken="Сделала два pass подряд.",
            tools_called=("compose_music",),
            dj_active=True,
        )
        assert rule is None, (
            f"compose_music в tools_called должен оправдывать "
            f"action-claim, но rule={rule!r}"
        )

    # ---- Negative cases: НЕ ловим бытовые prose-action-verb'ы ---------

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            (
                "давай уберу квартиру",
                "Сделала уборку и вымыла пол.",
            ),
            (
                "помой посуду",
                "Сделала.",
            ),
            (
                "что нового в магазине?",
                "Сходила и обновила список покупок.",
            ),
            (
                "как там погода?",
                "Поменяла настройки термометра.",
            ),
            (
                "перезапусти браузер",
                "Перезапустила.",
            ),
        ],
    )
    def test_no_dj_no_music_kw_no_match(
        self, user_input: str, spoken: str
    ) -> None:
        """Бытовое «сделала/обновила/поменяла/перезапустила» БЕЗ
        DJ-сессии и БЕЗ music-keyword в user_input → правило молчит.
        Иначе каждая бытовая реплика триггерила бы ложный ретрай."""
        rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
            dj_active=False,
        )
        assert rule is None, (
            f"бытовая реплика НЕ должна ловиться action-claim guard'ом: "
            f"user={user_input!r} spoken={spoken[:60]!r} rule={rule!r}"
        )

    def test_dj_inactive_works_for_legacy_rules(self) -> None:
        """Существующие правила (waypoint_save и т.п.) НЕ зависят от
        ``dj_active`` — регресс-страховка: dj_active=True не должен
        сломать контракт legacy правил."""
        rule = detect_unbacked_action_claim(
            user_input="запомни эту точку как тесточка",
            spoken="Точка сохранена.",
            tools_called=(),
            dj_active=True,
        )
        assert rule is not None
        assert rule.category == "waypoint_save"

    def test_music_kw_in_user_input_enables_match_even_without_dj(
        self, dj_active: bool = False
    ) -> None:
        """Если ``user_input`` содержит явный music-keyword
        («обнови бит»), правило срабатывает даже без DJ-сессии —
        гейт ``user_wants_music`` снимает ограничение."""
        rule = detect_unbacked_action_claim(
            user_input="обнови бит пожалуйста",
            spoken="Обновила бит — теперь звучит плотнее.",
            tools_called=(),
            dj_active=False,
        )
        assert rule is not None
        assert rule.category == "music_prose_action"

    def test_requires_dj_or_music_kw_gate_is_set(self) -> None:
        """Сам fact, что правило помечено как требующее DJ-контекст,
        — это часть контракта. Если кто-то случайно уберёт флаг,
        тест напомнит: rule может сжечь бытовой «сделала»."""
        music_rule = next(
            r for r in ACTION_CLAIM_RULES
            if r.category == "music_prose_action"
        )
        assert music_rule.requires_dj_or_music_kw is True

    def test_default_dj_active_is_false_backwards_compat(self) -> None:
        """Без явного ``dj_active=`` — поведение прежнее
        (False). Чтобы старые call-sites, которые передают только
        ``user_input / spoken / tools_called``, работали как раньше."""
        rule = detect_unbacked_action_claim(
            user_input="вплетай их красиво",
            spoken="Вплела тему Грига.",
            tools_called=(),
        )
        assert rule is None, (
            f"по умолчанию dj_active=False → rule не должен сработать "
            f"для prose без music-kw, но got {rule!r}"
        )

    def test_empty_inputs_safe_with_dj_active(self) -> None:
        """С ``dj_active=True`` пустые входы тоже не падают."""
        assert (
            detect_unbacked_action_claim(
                user_input=None, spoken="x", tools_called=(),
                dj_active=True,
            ) is None
        )
        assert (
            detect_unbacked_action_claim(
                user_input="x", spoken=None, tools_called=(),
                dj_active=True,
            ) is None
        )


class TestBuildMusicProseActionFallback:
    """Issue #2548 — fallback spoken когда action-claim ретрай
    уже потрачен, а claim повторился."""

    def test_returns_short_honest_phrase(self) -> None:
        text = build_music_prose_action_fallback("давай старайся")
        # Acceptance #2: «Не получилось изменить музыку — попробую
        # ещё раз». Без claim о выполнении, без извинений.
        assert text == "Не получилось изменить музыку — попробую ещё раз."

    def test_no_claim_of_completion(self) -> None:
        r"""Фраза НЕ должна содержать past-tense action-verb'ов
        (впл\w*, сдела\w*, обнов\w*, перезапущ\w*) — иначе мы
        говорим то же, что и до фикса, просто другими словами."""
        text = build_music_prose_action_fallback("вплетай их красиво")
        for verb in ("вплел", "вплела", "сделал", "обновил",
                     "перезапустил", "поменял", "изменил"):
            assert verb not in text.lower(), (
                f"fallback фраза содержит claim '{verb}' — "
                f"должна быть констатацией без claim: {text!r}"
            )

    def test_no_apology_marker(self) -> None:
        """Карточка явно требует «БЕЗ извинений». Проверяем."""
        text = build_music_prose_action_fallback("включи музыку")
        for apology in ("извин", "прости", "sorry", "прошу прощения"):
            assert apology not in text.lower(), (
                f"fallback содержит apology-маркер '{apology}': {text!r}"
            )


class TestExtractRenardoCodeLines:
    """Bug C′ — Renardo-код, попавший в реплику вместо execute_music_code."""

    def test_extracts_code_after_strip_markdown(self) -> None:
        # После strip_markdown ``` уже нет, а код остался.
        spoken = (
            "Мелодия для души — мягкие клавиши.\n\n"
            "renardo\n"
            "Clock.bpm = 72\n"
            'Scale.default = "major"\n'
            'Root.default = "D"\n'
            "p1 >> keys([0, 2, 4, 7], dur=0.5, amp=0.4)\n"
            "p2 >> bell([4, 7, 11, 9, 7], dur=2, oct=5, amp=0.25)\n"
            "p3 >> warmpad([0, 4, 7], dur=8, amp=0.2)"
        )
        code = extract_renardo_code_lines(spoken)
        assert code is not None
        assert "Clock.bpm = 72" in code
        assert "p1 >> keys" in code
        assert "p3 >> warmpad" in code
        assert "Мелодия" not in code  # проза не попадает в код

    def test_none_when_no_code(self) -> None:
        assert extract_renardo_code_lines("Расскажи анекдот про кота.") is None
        assert extract_renardo_code_lines("") is None
        assert extract_renardo_code_lines(None) is None

    def test_root_default_set_call(self) -> None:
        code = extract_renardo_code_lines(
            'Root.default.set("A")\nScale.default.set("minor")\np1 >> saw([0,1,2])'
        )
        assert code is not None
        assert "Root.default.set" in code

    def test_plain_text_with_clock_word_is_not_code(self) -> None:
        # «Clock.bpm» должно быть реальным присваиванием, а не прозой.
        assert extract_renardo_code_lines("включи бит и поставь темп") is None

    def test_live_3008_generated_code_is_recognised(self) -> None:
        """Код из живого лога 30.08 — тот, что робот сыграл правильно.
        Если бы он приехал в реплику, детектор обязан его увидеть."""
        code = extract_renardo_code_lines(
            "Clock.bpm = 128\n"
            'Scale.default.set("minor")\n'
            "p1 >> sawbass([0, -2, 0, 3], dur=0.5, amp=0.4, oct=3)\n"
            'd1 >> play("X..X.o..", sample=2, amp=0.3)'
        )
        assert code is not None
        assert "sawbass" in code


class TestBuildRenardoCodeRetryPrompt:
    def test_contains_code_and_demands_tool(self) -> None:
        prompt = build_renardo_code_retry_prompt("p1 >> blip([0,2,4])")
        assert "execute_music_code" in prompt
        assert "p1 >> blip([0,2,4])" in prompt
        assert "[CRITICAL]" in prompt

class TestLive3008E2eSecondRound:
    """Прогон 30.08 16:00-16:05 — уже с раскатанными первыми фиксами."""

    @pytest.mark.parametrize(
        "phrase",
        [
            # 15:56: лаунж играл 94 с и замолчал ровно на этой фразе —
            # пары «глагол + существительное» в ней нет.
            "продолжай лабать мы летим над парижем",
            "полабай ещё",
            "залабай что-нибудь",
            "лабай дальше",
            "продолжай лаунж",
        ],
    )
    def test_music_slang_is_a_music_request(self, phrase: str) -> None:
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            # «лаб» как подстрока живёт в «ослабь» и «слабее» —  плюс
            # список приставок обязаны их отсечь.
            "ослабь громкость",
            "стало слабее слышно",
            "сходи в лабораторию",
        ],
    )
    def test_slab_lookalikes_are_not_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is False

    def test_search_claim_without_tool_is_detected(self) -> None:
        """16:04: «найди в библиотеке сэмплы барабанов» → «Сэмплы ударных
        найдены.» при tools=[]. Поиска не было."""
        rule = detect_unbacked_action_claim(
            user_input="найди в своей библиотеке синтли барабанов",
            spoken="Сэмплы ударных найдены.",
            tools_called=(),
        )
        assert rule is not None
        assert rule.category == "library_search"

    @pytest.mark.parametrize(
        "tool", ["search_samples", "list_tracks", "gen_search_library"],
    )
    def test_search_claim_with_any_search_tool_is_fine(self, tool: str) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input="найди в своей библиотеке сэмплы барабанов",
                spoken="Сэмплы ударных найдены.",
                tools_called=(tool,),
            )
            is None
        )

    def test_memory_search_is_not_a_library_search(self) -> None:
        """Поиск по памяти закрывается своими тулами и в это правило
        попадать не должен — в логе 30.08 он отработал верно."""
        assert (
            detect_unbacked_action_claim(
                user_input="поищи в своей памяти что я говорил про чай",
                spoken="Нашла: зелёный чай без сахара.",
                tools_called=(),
            )
            is None
        )

class TestReadOnlyClaimsFromE2e33251879328:
    """GATE-1 из e2e 33251879328: «expected tool calls not invoked ...
    LLM сделал verbal-only answer». Робот отвечает о ЖИВОМ состоянии по
    памяти модели, не спросив систему."""

    @pytest.mark.parametrize(
        "user_input,spoken,category",
        [
            (
                "перечисли все точки которые ты запомнил",
                "Точек пока нет — карту ни разу не строили.",
                "waypoint_list",
            ),
            (
                "какие звуки ты умеешь проигрывать",
                "Умею эмоции, интерфейсные сигналы, спецэффекты.",
                "sound_info",
            ),
            ("играет ли сейчас музыка", "Нет, сейчас тишина.", "music_state"),
            ("загрузи и включи трек тисбит", "Трек играет.", "track_load"),
        ],
    )
    def test_state_claim_without_tool_is_detected(
        self, user_input: str, spoken: str, category: str
    ) -> None:
        rule = detect_unbacked_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        )
        assert rule is not None
        assert rule.category == category

    @pytest.mark.parametrize(
        "user_input,spoken,tool",
        [
            ("перечисли все точки", "Точек пока нет.", "list_waypoints"),
            ("какие звуки ты умеешь", "Умею эмоции.", "get_sound_info"),
            ("играет ли сейчас музыка", "Нет, тишина.", "get_music_state"),
            ("загрузи и включи трек тисбит", "Трек играет.", "load_track"),
            (
                "загрузи и включи трек тисбит",
                "Трек играет.",
                "gen_play_from_library",
            ),
            (
                "загрузи и включи трек тисбит",
                "Трек играет.",
                "execute_music_code",
            ),
        ],
    )
    def test_state_claim_with_the_tool_is_fine(
        self, user_input: str, spoken: str, tool: str
    ) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=user_input, spoken=spoken, tools_called=(tool,)
            )
            is None
        )

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # Рассказ о своих возможностях — не запрос живого состояния.
            ("что ты умеешь делать расскажи по пунктам",
             "Умею говорить, петь, играть музыку."),
            # Запрос сыграть закрывается music-гуардом, не Bug E.
            ("сыграй техно для души", "Бит качает."),
            ("расскажи анекдот", "Колобок повесился."),
            ("поехали вперед", "Еду."),
        ],
    )
    def test_no_false_positives_on_readonly_rules(
        self, user_input: str, spoken: str
    ) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=user_input, spoken=spoken, tools_called=()
            )
            is None
        )

class TestMusicRetryPromptNamesBothLibraries:
    """🔴 e2e tc10_load_track: промпт звал только в mp3-библиотеку.

    «тисбит» сохранялся через save_track — то есть в Renardo-медиатеку, а
    промпт предлагал gen_list_library. LLM звали туда, где трека нет, и
    она сдавалась, повторяя «Трек тисбит играет.» с tools=[].
    """

    def test_prompt_names_the_renardo_library_first(self) -> None:
        prompt = build_music_retry_prompt("загрузи и включи трек тисбит")
        assert "list_tracks" in prompt
        assert "load_track" in prompt
        assert "save_track" in prompt, "надо объяснить, ГДЕ лежит сохранённый трек"

    def test_prompt_still_names_the_mp3_library(self) -> None:
        prompt = build_music_retry_prompt("включи случайный трек")
        assert "gen_list_library" in prompt
        assert "gen_play_from_library" in prompt

    def test_prompt_forbids_inventing_success(self) -> None:
        """Обе библиотеки пусты — честный ответ, а не «трек играет»."""
        prompt = build_music_retry_prompt("включи трек которого нет")
        assert "ЗАПРЕЩЕНО" in prompt

class TestRetryPromptKnowsIfMusicIsPlaying:
    """🔴 e2e renardo_evolve rn03 (живой прогон 30.08).

    «Переходи в лёгкий джангл» при играющем рассвете. Промпт утверждал
    «Музыка сейчас НЕ играет», модель видела обратное, отвечала «Окей,
    играет лёгкий джангл» с tools=[] — и так дважды, до nudge. Джангла не
    случилось. Утверждение стало ложью ровно тогда, когда TRACK-музыка
    научилась переживать чужой ход.
    """

    def test_silence_asks_to_start(self) -> None:
        prompt = build_music_retry_prompt("сыграй техно")
        assert "НЕ играет" in prompt

    def test_playing_asks_to_change(self) -> None:
        prompt = build_music_retry_prompt(
            "переходи в лёгкий джангл", music_playing=True
        )
        assert "НЕ играет" not in prompt
        assert "ИГРАЕТ" in prompt
        assert "ИЗМЕНИТЬ" in prompt

    def test_playing_explains_why_a_tool_is_still_needed(self) -> None:
        """Модель должна понять, что само оно не поменяется."""
        prompt = build_music_retry_prompt("добавь баса", music_playing=True)
        assert "НОВЫЙ код" in prompt

    def test_default_stays_backward_compatible(self) -> None:
        """Без флага поведение прежнее — вызовы из старого кода не ломаются."""
        assert build_music_retry_prompt("сыграй бит") == build_music_retry_prompt(
            "сыграй бит", music_playing=False
        )

    def test_both_variants_keep_the_shared_prefix(self) -> None:
        """``_run_turn`` матчит префикс, чтобы не сбрасывать бюджет ретраев."""
        from rob_box_voice.core.dialogue_guards import MUSIC_RETRY_PROMPT_PREFIX
        for playing in (False, True):
            p = build_music_retry_prompt("сыграй бит", music_playing=playing)
            assert p.startswith(MUSIC_RETRY_PROMPT_PREFIX)


# Issue #1777 / #1762 — non-music tool guard
# ---------------------------------------------------------------------------


class TestDetectRequiredTool:
    """Issue #1777 / #1762 — ``detect_required_tool`` должен правильно
    определять, какой tool явно просит юзер.

    Главный кейс #1777 — «который час» / «сколько времени» / «время в Москве»
    → get_current_time (раньше LLM говорил из головы).
    Кейс #1762 — «погода в Бишкеке» / «новости про X» → search_web.
    """

    def test_time_phrases(self) -> None:
        # Главный кейс #1777.
        assert detect_required_tool("который час") == "get_current_time"
        assert detect_required_tool("сколько времени") == "get_current_time"
        assert detect_required_tool("сколько сейчас времени") == "get_current_time"
        assert detect_required_tool("время в москве") == "get_current_time"
        assert detect_required_tool("время по москве") == "get_current_time"
        assert detect_required_tool("какая дата") == "get_current_time"
        assert detect_required_tool("какой день недели") == "get_current_time"
        assert detect_required_tool("какое сегодня число") == "get_current_time"

    def test_search_web_phrases(self) -> None:
        # Главный кейс #1762.
        assert detect_required_tool("погода в бишкеке") == "search_web"
        assert detect_required_tool("какая погода") == "search_web"
        assert detect_required_tool("новости про биткоин") == "search_web"
        assert detect_required_tool("что в интернете про илон маск") == "search_web"
        assert detect_required_tool("загугли курс доллара") == "search_web"
        assert detect_required_tool("расскажи про космос") == "search_web"

    def test_set_voice_phrases(self) -> None:
        # Кейс #1765.
        assert detect_required_tool("переключи голос на арт") == "set_voice"
        assert detect_required_tool("смени голос") == "set_voice"
        assert detect_required_tool("голос зайцев") == "set_voice"
        assert detect_required_tool("голос ермак") == "set_voice"
        assert detect_required_tool("поставь голос окс") == "set_voice"

    def test_memory_search_phrases(self) -> None:
        # Кейс #1770.
        assert detect_required_tool("что ты знаешь обо мне") == "memory_search"
        assert detect_required_tool("помнишь меня") == "memory_search"
        assert detect_required_tool("что помнишь про меня") == "memory_search"

    def test_faq_search_phrases(self) -> None:
        assert detect_required_tool("что ты умеешь") == "faq_search"
        assert detect_required_tool("какие команды") == "faq_search"
        assert detect_required_tool("справка") == "faq_search"
        assert detect_required_tool("расскажи о себе") == "faq_search"

    def test_no_match_returns_none(self) -> None:
        # chit-chat НЕ должен ретраить — иначе спам.
        assert detect_required_tool("как дела") is None
        assert detect_required_tool("расскажи анекдот") is None
        assert detect_required_tool("привет") is None
        assert detect_required_tool("") is None
        assert detect_required_tool(None) is None  # type: ignore[arg-type]

    def test_reported_speech_does_not_match_set_voice(self) -> None:
        """«говор »/«говори »/«говорит »/«голосом» были бы substring-match
        без границ слова — совпали бы с обычным chit-chat про чужую речь и
        заставили бы LLM звать set_voice в ходе, никак не связанном с
        голосом робота."""
        assert detect_required_tool("не говори глупости") is None
        assert detect_required_tool("мама говорит что уже пора") is None
        assert detect_required_tool("он говорит по-английски") is None
        assert detect_required_tool("спой красивым голосом") is None

    def test_music_phrases_dont_match_tool_guard(self) -> None:
        """«спой песенку» / «включи музыку» → НЕ должен попадать в tool guard
        (для них есть отдельный music guard, см. issue #992 Bug C)."""
        assert detect_required_tool("спой песенку") is None
        assert detect_required_tool("включи музыку") is None
        assert detect_required_tool("поставь бит") is None

    def test_all_known_tools_are_in_allow_list(self) -> None:
        """Sanity-check: TOOL_REQUEST_PATTERNS покрывает 5 основных категорий."""
        tool_names = {tool for tool, _ in TOOL_REQUEST_PATTERNS}
        assert tool_names == {
            "get_current_time",
            "search_web",
            "set_voice",
            "memory_search",
            "faq_search",
        }

    def test_keyword_tuples_non_empty_for_all_tools(self) -> None:
        for tool, kws in TOOL_REQUEST_PATTERNS:
            assert kws, f"empty keyword list for {tool}"


class TestBuildToolRetryPrompt:
    """Issue #1777 / #1762 — synthetic prompt для Bug C retry
    (non-music tools)."""

    def test_echoes_user_input(self) -> None:
        prompt = build_tool_retry_prompt("который час", "get_current_time")
        assert "который час" in prompt

    def test_uses_critical_prefix(self) -> None:
        """Тот же [CRITICAL] префикс, что и у music guard — иначе
        dialogue_node._run_turn сбросит retry budget на синтетическом
        ретрае и цикл зациклится (см. issue #992 Bug C root cause)."""
        prompt = build_tool_retry_prompt("любой user input", "get_current_time")
        assert prompt.startswith(MUSIC_RETRY_PROMPT_PREFIX)

    def test_names_specific_tool(self) -> None:
        # «погода в Бишкеке» → search_web.
        prompt = build_tool_retry_prompt("погода в бишкеке", "search_web")
        assert "search_web" in prompt
        assert "get_current_time" not in prompt  # НЕ подменять инструмент

    def test_time_retry_prompts_calls_get_current_time(self) -> None:
        # Главный кейс #1777.
        prompt = build_tool_retry_prompt("который час", "get_current_time")
        assert "get_current_time" in prompt
        # Предупреждаем LLM не выдумывать время из головы.
        assert "Не выдумывай" in prompt or "не выдумывай" in prompt.lower()

    def test_search_web_retry_prompts_calls_search_web(self) -> None:
        prompt = build_tool_retry_prompt("погода в бишкеке", "search_web")
        assert "search_web" in prompt
        # Hint для search_web должен явно просить вызвать search_web,
        # а не говорить «не выдумывай» (как для get_current_time).
        assert "search_web(query=" in prompt

    def test_set_voice_retry_prompts_calls_set_voice(self) -> None:
        prompt = build_tool_retry_prompt("переключи голос на арт", "set_voice")
        assert "set_voice" in prompt
        assert "list_voices" in prompt  # подсказка проверить список

    def test_memory_search_filters_by_current_speaker(self) -> None:
        # Кейс #1770 — не подставлять факты чужих юзеров.
        prompt = build_tool_retry_prompt("что ты знаешь обо мне", "memory_search")
        assert "memory_search" in prompt
        assert "speaker_id" in prompt
        assert "ТЕКУЩЕГО" in prompt or "current" in prompt.lower()

    def test_faq_retry_prompts_calls_faq_search(self) -> None:
        prompt = build_tool_retry_prompt("что ты умеешь", "faq_search")
        assert "faq_search" in prompt

    def test_defence_in_depth_unknown_tool_returns_empty(self) -> None:
        """Prompt-injection защита: неизвестный tool_name → "" → caller
        пропускает retry. Без этого юзер мог бы через user_input заставитьть
        LLM «вызвать evil_tool» (или хотя бы попытаться)."""
        prompt = build_tool_retry_prompt("любой", "evil_tool_name")
        assert prompt == ""

    def test_both_prompts_use_same_critical_prefix(self) -> None:
        """Sanity-check: оба retry-промпта должны иметь одинаковый
        MUSIC_RETRY_PROMPT_PREFIX — это нужно для того, чтобы
        dialogue_node._run_turn не сбрасывал retry budget на синтетическом
        ретрае (см. issue #992 Bug C)."""
        music_prompt = build_music_retry_prompt("x")
        tool_prompt = build_tool_retry_prompt("x", "get_current_time")
        assert music_prompt.startswith(MUSIC_RETRY_PROMPT_PREFIX)
        assert tool_prompt.startswith(MUSIC_RETRY_PROMPT_PREFIX)


def test_all_tool_retry_prompts_have_critical_prefix() -> None:
    """Sanity-check: для всех 5 tool_names prefix должен быть [CRITICAL].
    Защита от тихой регрессии: добавили новый tool_name, забыли prefix —
    тест сразу скажет."""
    for tool_name, _ in TOOL_REQUEST_PATTERNS:
        prompt = build_tool_retry_prompt("любой user input", tool_name)
        assert prompt.startswith(MUSIC_RETRY_PROMPT_PREFIX), (
            f"tool {tool_name!r} retry prompt must start with "
            f"MUSIC_RETRY_PROMPT_PREFIX, got: {prompt[:80]!r}"
        )


class TestStopClearsTheMusicPlayingFlagLive3108:
    """«Выключи музыку» обязана гасить ``_track_mode_music_active``.

    Живой лог 31.08. Флаг ставился в True при запуске музыки и снимался
    ТОЛЬКО в ``_publish_music_cleanup``; ``stop_music`` его не трогал. После
    «выключи музыку» он врал, а он выбирает формулировку Bug-C ретрая: при
    True промпт говорит «музыка СЕЙЧАС ИГРАЕТ, её надо ИЗМЕНИТЬ, а не
    заводить заново».

    Дальше «сыграй джаз» → модель получает указание менять несуществующий
    трек → отвечает «Джаз пошёл — лидийский лад, мягкие клавиши» с
    ``tools=[]`` → оба ретрая выгорают → робот произносит «я растерялся —
    бит не запустился». mcp_server в те же секунды писал «активной музыки
    не обнаружено».

    Сам узел требует rclpy, поэтому здесь проверяется проводка в исходнике —
    так тест идёт в любом окружении, как и остальные текстовые проверки
    контрактов в этом пакете.
    """

    def _dialogue_node_source(self) -> str:
        from pathlib import Path

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "rob_box_voice" / "dialogue_node.py"
            if candidate.is_file():
                return candidate.read_text(encoding="utf-8")
        raise AssertionError("dialogue_node.py not found")

    def test_stop_music_is_a_stop_tool(self) -> None:
        from rob_box_voice.core.dialogue_guards import MUSIC_STOP_TOOLS

        assert "stop_music" in MUSIC_STOP_TOOLS

    def test_node_clears_the_flag_on_stop_tools(self) -> None:
        # Issue #2631 / ADR-0021 R1 — cleanup-policy вынесен в
        # ``DialogueNode._schedule_music_cleanup``. Тест ищет подстроки
        # в определении этого helper'а, а не во всём файле.
        src = self._schedule_music_cleanup_source()
        assert "tools_now & MUSIC_STOP_TOOLS" in src, (
            "dialogue_node не гасит _track_mode_music_active на stop-тулах"
        )
        clear_at = src.index("tools_now & MUSIC_STOP_TOOLS")
        tail = src[clear_at:clear_at + 300]
        assert "_track_mode_music_active = False" in tail, (
            "ветка stop-тулов не сбрасывает флаг"
        )

    def test_flag_is_cleared_before_the_starters_branch_sets_it(self) -> None:
        """Ход «стоп + сразу играй» должен закончиться True, а не False."""
        # Issue #2631 / ADR-0021 R1 — cleanup-policy вынесен в
        # ``_schedule_music_cleanup``.
        src = self._schedule_music_cleanup_source()
        stop_branch = src.index("tools_now & MUSIC_STOP_TOOLS")
        starters_branch = src.index("if tools_now & music_starters")
        assert stop_branch < starters_branch, (
            "сброс обязан идти ДО ветки запуска, иначе она будет затёрта"
        )

    def _schedule_music_cleanup_source(self) -> str:
        """Извлечь тело ``DialogueNode._schedule_music_cleanup`` для текстовых
        проверок контракта (issue #2631).
        """
        import re

        full = self._dialogue_node_source()
        # Locate ``def _schedule_music_cleanup(`` and read until the next
        # ``def `` at the same indent (4 spaces inside class).
        match = re.search(
            r"^(    def _schedule_music_cleanup\(.*?\n)(?=    def |\nclass )",
            full,
            flags=re.MULTILINE | re.DOTALL,
        )
        assert match is not None, (
            "_schedule_music_cleanup helper not found in dialogue_node.py"
        )
        return match.group(1)


class TestWatchdogStopClearsTheFlagLive3108:
    """Музыку останавливает не только диалог — флаг обязан это узнавать.

    Живой лог 31.08::

        1788186658  [watchdog] Авто-стоп 1 паттернов: reason=idle_ttl
        1788186797  [track-mode] TRACK играет с прошлого хода
        1788186797  [Bug C] LLM skipped execute_music_code; publishing nudge
                    TTS: «Я тут растерялся — бит не запустился»

    Через 139 секунд после реальной остановки диалог всё ещё считал, что
    трек играет. Комментарий в track-mode честно писал «живёт до
    stop_music/watchdog», но канала для второго не существовало: топик
    ``/voice/music/state`` слушал только audio_node.

    Отсюда «после нескольких генераций робот начинает тупить»: ретрай-промпт
    требовал ИЗМЕНИТЬ несуществующий трек, модель отвечала описанием, оба
    ретрая выгорали.
    """

    def _dialogue_node_source(self) -> str:
        from pathlib import Path

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "rob_box_voice" / "dialogue_node.py"
            if candidate.is_file():
                return candidate.read_text(encoding="utf-8")
        raise AssertionError("dialogue_node.py not found")

    def test_node_subscribes_to_the_server_music_state(self) -> None:
        src = self._dialogue_node_source()
        assert '"/voice/music/state"' in src, (
            "диалог не слушает /voice/music/state — про остановку по "
            "watchdog он не узнает"
        )

    def test_handler_clears_the_flag_on_idle(self) -> None:
        src = self._dialogue_node_source()
        start = src.index("def _on_music_state")
        body = src[start:start + 1400]
        assert "idle" in body
        assert "_track_mode_music_active = False" in body

    def test_handler_never_sets_the_flag(self) -> None:
        """Взводит флаг только ход диалога: серверу не видно BACKING/TRACK."""
        src = self._dialogue_node_source()
        start = src.index("def _on_music_state")
        body = src[start:start + 1400]
        assert "_track_mode_music_active = True" not in body, (
            "обработчик не должен взводить флаг — сервер не различает "
            "BACKING (гасится после речи) и TRACK (живёт до стопа)"
        )


class TestLive0109PlayVerbAndDjangoGenre:
    """🔴 Живой лог vision-pi, 01.09 09:22 и 09:35 — две дыры подряд.

    ``переходи в легкий джанго`` и ``играем легкий джаз``: обе фразы гуард
    посчитал не-музыкой («user does NOT want music» в логе), Bug-C ретрай не
    сработал, и робот рассказал про гитару и саксофон с ``tools=[]``. Ровно
    та жалоба, с которой пришёл Шифу: «рассказывает про джаз, но ничего не
    пускает».

    Причины разные:

    * ``джанго`` — цыганский джаз Джанго Рейнхардта. В списке жанров лежал
      ``джангл`` (drum-n-bass), одна буква разницы, и её хватило.
    * ``играем`` — в ``MUSIC_GUARD_KEYWORDS`` есть ``играй``, но подстрокой
      в «играем» он не входит, а основы ``игра`` среди глаголов не было
      вовсе. Основу добавили в парные глаголы, а не в ключевые слова:
      «играем в шахматы» музыкой быть не должно.
    """

    @pytest.mark.parametrize(
        "phrase",
        [
            "переходи в легкий джанго",
            "играем легкий джаз",
            "играем техно",
            "поиграем немного блюз",
            "включи джаз",
            "включи что-нибудь под фанк",
        ],
    )
    def test_missed_music_requests_are_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            "играем в шахматы",
            "поиграем в города",
            "давай сыграем в настолку",
            "включи свет",
            "включи лампу на кухне",
            "продолжай маршрут до кухни",
        ],
    )
    def test_non_music_stays_non_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is False


class TestLive0109DevelopTheThemeAndPseudoCall:
    """🔴 Живой лог vision-pi, 01.09 09:55 и 10:17.

    ``развивай тему`` при играющем джанго: гуард сказал «user does NOT want
    music», ретрая не было, ход пропал молча. «Тема» — обычное музыкальное
    слово, но в общий список жанров его класть нельзя: оно склеится с
    «смени», и разговорное «смени тему» станет просьбой о музыке. Поэтому
    пара только с глаголами РАЗВИТИЯ материала.

    Там же 10:17: на ``развивай мелодию`` модель трижды подряд вернула
    ТЕКСТ ``<compose_music composition here>`` с ``tools=[]``. CRITICAL-промпт
    про такой промах молчал, поэтому и ретрай его повторял.
    """

    @pytest.mark.parametrize(
        "phrase",
        [
            "развивай тему",
            "продолжай тему",
            "доработай тему",
            "тему развивай",
            "усложни тему",
        ],
    )
    def test_developing_a_theme_is_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            "смени тему",
            "давай сменим тему",
            "поменяй тему разговора",
            "какая тема",
            "темно на улице",
        ],
    )
    def test_changing_the_subject_is_not_music(self, phrase: str) -> None:
        assert user_wants_music(phrase) is False

    def test_retry_prompt_says_text_is_not_a_call(self) -> None:
        prompt = build_music_retry_prompt("развивай мелодию", music_playing=True)
        assert "function calling" in prompt
        assert "НЕ " in prompt and "вызов" in prompt


class TestPlanningNarration:
    """🔴 Живой лог робота 02.09 — модель отдавала своё планирование в
    качестве реплики, TTS зачитывал его вслух, тулов при этом не было.

    Пример из лога (06:58:01 UTC, ``tools=[]``):
        'Юзер явно просит «ебани лаундж» (лоундж запрошен снова/повторно).
         DJ уже выключен в прошлом ходе. Запускаю расслабленную
         лоундж-композицию через compose_music.'

    Юзер слышал «робот говорит, что запускает музыку, но ничего не
    запускается». Ни ``BABBLE_BANNED_OPENERS`` (там обещания, а не
    рассуждения), ни ``MUSIC_GUARD_KEYWORDS`` (смотрят на реплику юзера —
    «ебани ланудж») этот случай не покрывали.

    Тесты для develop-версии `is_planning_narration` (введена в 78403dba).
    Этот PR (issue #1882) добавляет поверх — **hard-mute guard** в
    `_handle_result` (см. dialogue_node.py), который закрывает дыру
    «babble-ретрай уже потрачен, monologue всё равно уходит в TTS».
    Нода-тесты для hard-mute — в test_issue_1882_planning_narration.py.
    """

    @pytest.mark.parametrize(
        "spoken",
        [
            "Юзер явно просит «ебани лаундж». Запускаю лоундж-композицию "
            "через compose_music.",
            "Пользователь спрашивает «где бит?» — хочет услышать бит. "
            "Проверю состояние музыки.",
            "Юзер назначил меня диджеем. Это очевидная просьба про музыку.",
            "Сначала вызову stop_music, потом заведу новый трек.",
            "Отвечу через speak_text.",
        ],
    )
    def test_planning_narration_is_detected(self, spoken: str) -> None:
        assert is_planning_narration(spoken) is True

    @pytest.mark.parametrize(
        "spoken",
        [
            "Бит пошёл — бум-бум, ловлю волну.",
            "Тебя зовут Саша! Любишь зелёный чай без сахара.",
            "Вот и всё, вечеринка заканчивается! Спасибо, что были со мной.",
            "Сейчас пятнадцать семь, вторник, первое сентября.",
            "",
        ],
    )
    def test_normal_answers_pass(self, spoken: str) -> None:
        assert is_planning_narration(spoken) is False

    def test_tool_name_alone_is_enough(self) -> None:
        """Идентификатор в snake_case, прочитанный вслух, — всегда баг,
        независимо от того, о чём просил юзер."""
        assert is_planning_narration("Готово, execute_music_code отработал.")

    def test_planning_narration_counts_as_babble(self) -> None:
        """Ретрай-механика Bug D переиспользуется как есть."""
        assert is_metalanguage_babble(
            "Юзер просит музыку. Запускаю через compose_music."
        )


# ---------------------------------------------------------------------------
# Issue #2175 — MiniMax-M3 regurgitates ``<system>...</system>`` template
# ---------------------------------------------------------------------------


class TestSystemTemplateRegurgitateDetector:
    """Live 08.09 (Vision Pi, 14:52) — три запроса подряд после ``set_voice``
    + multi-voice user_input + новая DJ-skill context дали в ``spoken`` ровно
    regurgitates системного промпта. Детектор должен ловить этот паттерн и
    НЕ ловить обычные фразы, в которых тег упомянут вскользь.
    """

    @pytest.mark.parametrize(
        "spoken",
        [
            # Канонический пример из живого лога 08.09 14:52
            "<system>\n[получатель ответа забыл указать антропоморфные атрибуты]\n</system>",
            # Минимальный валидный блок
            "<system>x</system>",
            # С whitespace вокруг
            "  <system>\nfoo\n</system>  \n",
            # Регистр не имеет значения (MiniMax может отдать <SYSTEM>...)
            "<SYSTEM>foo</SYSTEM>",
            # Многострочный с markdown-маркерами внутри
            "<system>\n- line1\n- line2\n</system>",
        ],
    )
    def test_regurgitated_system_block_is_detected(self, spoken: str) -> None:
        """Полный парный блок ``<system>...</system>`` в extracted
        spoken (БЕЗ ``<speak>``-обёртки, без серединного текста)."""
        assert is_system_template_regurgitated(spoken) is True

    @pytest.mark.parametrize(
        "spoken",
        [
            "Привет!",
            "Говорю голосом надёжного мужчины.",
            "",  # пустая строка — частый случай empty spoken
            None,  # type: ignore[list-item]  # noqa: None должен быть безопасным
            # Серединный встроенный блок — НЕ regurgitates, а обычная фраза,
            # где LLM упоминает систему в переносном смысле.
            "Согласно <system>инструкции</system>, отвечу.",
            # Незакрытый тег — НЕ полный regurgitates
            "<system>foo",
            # Закрывающий тег без открывающего — НЕ полный regurgitates
            "foo</system>",
            # ``<system_context>`` имеет ДРУГОЙ закрывающий тег — не
            # матчится (regex ждёт ровно ``</system>`` без ``_context``)
            "<system_context>foo</system_context>",
            # SSML-обёртка в extracted spoken — НЕ regurgitates (это
            # на вход в tts_node, не в dialogue_node). Для SSML есть
            # отдельный helper ``is_system_template_regurgitated_in_ssml``.
            "<speak><system>foo</system></speak>",
        ],
    )
    def test_normal_or_partial_strings_pass(self, spoken: Optional[str]) -> None:
        """Обычные ответы, серединные ссылки, неполные теги и
        SSML-обёртки НЕ блокируются этим detector'ом.

        Detector для extracted spoken жёстче чем для SSML: ``^...$``
        ограничивает всю строку, поэтому серединные ссылки и
        ``<speak>``-обёрнутый текст НЕ regurgitates для dialogue_node.
        """
        assert is_system_template_regurgitated(spoken) is False

    def test_regex_is_shared_between_dialogue_and_tts(self) -> None:
        """``SYSTEM_TEMPLATE_REGURGITATE_RE`` экспортируется для re-use в
        tts_node — defence-in-depth. Детектор и regex должны давать
        одинаковый ответ (защита от drift между двумя стражами).
        """
        samples = [
            ("<system>x</system>", True),
            ("Привет", False),
            ("", False),
            ("  <system>\nfoo\n</system>\n", True),
            # SSML-обёртка НЕ regurgitates для extracted spoken
            ("<speak><system>foo</system></speak>", False),
        ]
        for text, expected in samples:
            detector = is_system_template_regurgitated(text)
            regex = bool(SYSTEM_TEMPLATE_REGURGITATE_RE.match(text or ""))
            assert detector == expected
            assert detector == regex, (
                f"drift between detector and regex for {text!r}"
            )


class TestSystemTemplateRegurgitateInSsml:
    """Defence-in-depth для tts_node — отдельный detector для SSML.

    ``is_system_template_regurgitated_in_ssml`` ловит regurgitates
    на СЫРОМ SSML (до strip'а тегов в ``_extract_text_from_ssml``).
    Более узкий, чем для extracted spoken: только полный ``<speak>``
    обрамлённый блок ИЛИ plain ``<system>...</system>``.
    """

    @pytest.mark.parametrize(
        "ssml",
        [
            # SSML-обёртка с regurgitates внутри
            "<speak><system>foo</system></speak>",
            "<speak><system>\n[получатель ответа забыл указать антропоморфные "
            "атрибуты]\n</system></speak>",
            # Без SSML-обёртки (edge-case)
            "<system>x</system>",
            # Whitespace внутри тегов (после ``<speak>`` / перед ``</speak>``)
            "<speak>\n  <system>foo</system>  \n</speak>",
        ],
    )
    def test_ssml_regurgitate_is_detected(self, ssml: str) -> None:
        assert is_system_template_regurgitated_in_ssml(ssml) is True

    @pytest.mark.parametrize(
        "ssml",
        [
            # Нормальная фраза с серединным тегом — НЕ regurgitates
            "<speak>Согласно <system>инструкции</system>, отвечу.</speak>",
            # Обычная русская речь
            "<speak>Привет! Как дела?</speak>",
            # Пустая строка / None
            "",
            None,  # type: ignore[list-item]
            # Неполные теги
            "<speak><system>foo</speak>",
            "<speak>foo</system></speak>",
        ],
    )
    def test_normal_ssml_passes(self, ssml: Optional[str]) -> None:
        """Серединные ссылки, нормальная речь, неполные теги."""
        assert is_system_template_regurgitated_in_ssml(ssml) is False


class TestBuildSystemRegurgitateRetryPrompt:
    """Issue #2175 — одноразовый CRITICAL-ретрай на regurgitated template."""

    def test_prompt_contains_critical_marker(self) -> None:
        prompt = build_system_regurgitate_retry_prompt("test user input")
        assert "[CRITICAL]" in prompt, (
            "ретрай должен начинаться с [CRITICAL] маркера — тот же контракт, "
            "что у build_babble_retry_prompt / build_unbacked_action_retry_prompt"
        )

    def test_prompt_echoes_original_user_input(self) -> None:
        """Юзер-интент в ретрае — это оригинальная команда (с обрезанным
        предыдущим [CRITICAL]-блоком, см. _strip_trailing_critical_block)."""
        prompt = build_system_regurgitate_retry_prompt(
            "[Spkr:Денчик] продолжай голосом надёжного мужчины"
        )
        assert "[Spkr:Денчик] продолжай голосом надёжного мужчины" in prompt

    def test_prompt_forbids_xml_blocks(self) -> None:
        """Ретрай явно называет ЗАПРЕЩЁННЫЕ XML-теги — модель должна знать,
        что regurgitates этих блоков = BUG (как и в babble-ретрае про
        «слушай/погнали»)."""
        prompt = build_system_regurgitate_retry_prompt("x")
        forbidden = [
            "<system>",
            "<system_context>",
            "<hardware>",
        ]
        for tag in forbidden:
            assert tag in prompt, (
                f"ретрай должен явно называть запрещённый тег {tag!r}, "
                "иначе модель не поймёт, что именно regurgitates"
            )

    def test_prompt_handles_none_user_input(self) -> None:
        """None / пустая строка — крайний случай, не должен падать."""
        prompt = build_system_regurgitate_retry_prompt(None)
        assert "[CRITICAL]" in prompt
        prompt2 = build_system_regurgitate_retry_prompt("")
        assert "[CRITICAL]" in prompt2

    def test_prompt_strips_previous_critical_block(self) -> None:
        """Если в user_input уже есть предыдущий [CRITICAL]-блок
        (вложенный ретрай), он обрезается — иначе модель читает
        противоречивые инструкции. Контракт тот же, что у babble-retry."""
        prompt = build_system_regurgitate_retry_prompt(
            "оригинал\n\n[CRITICAL] предыдущий блок"
        )
        # Только один [CRITICAL] — свежий, не дубль от прошлого ретрая
        assert prompt.count("[CRITICAL]") == 1
        assert "предыдущий блок" not in prompt


# ---------------------------------------------------------------------------
# Issue #2562 Bug F — «не знаю такой мелодии» без поиска.
#
# Round 3 live-check 15.09.2026 (Vision Pi 10.1.1.21): модель на просьбу
# «сыграй X» дважды за час отвечала ::
#
#   spoken='Не знаю такой мелодии — могу сыграть что-то похожее.
#           Что ближе — расслабленный фанк или драйв?'
#   tools=[] finish_reason='stop'
#
# Тесты ниже закрывают детектор + ретрай-промпт. acceptance criteria:
# «10 промптов с неизвестными композиторами → fallback с поиском или
# альтернативой».
# ---------------------------------------------------------------------------


class TestUnknownMelodyClaimLive1509:
    """Issue #2562 Bug F — live-примеры из round 3."""

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # Дословно из лога 15.09.2026 round 3 (live check).
            (
                "сыграй григ в пещере горного короля",
                "Не знаю такой мелодии — могу сыграть что-то похожее. "
                "Что ближе — расслабленный фанк или драйв?",
            ),
            (
                "включи мелодию из фильма Интерстеллар",
                "Не знаю этой мелодии, к сожалению.",
            ),
            (
                "сыграй имперский марш но в стиле регги",
                "Не помню точно нот имперского марша в регги-стиле.",
            ),
            (
                "давай тему из тетриса",
                "Не знаю этой композиции наизусть, попробую поискать.",
            ),
            (
                "сыграй саундтрек из Гарри Поттера",
                "Нет в моей памяти точных нот.",
            ),
            (
                "играй кантри-мелодию про техас",
                "Не знаю наизусть такой мелодии.",
            ),
        ],
    )
    def test_live_round3_claim_without_tool_is_detected(
        self, user_input: str, spoken: str
    ) -> None:
        """6 live-примеров из round 3 — все ловятся детектором."""
        assert detect_unknown_melody_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
        ) is True

    @pytest.mark.parametrize(
        "user_input,spoken,tools",
        [
            # LLM ЧЕСТНО попыталась — lookup_melody был вызван, но вернул
            # пусто. Это легитимный «не нашёл», НЕ Bug F.
            (
                "сыграй григ в пещере горного короля",
                "Не знаю такой мелодии — в RTTTL её нет.",
                ("lookup_melody",),
            ),
            (
                "сыграй какую-то супер редкую мелодию",
                "Не знаю этой композиции. Ничего не нашлось.",
                ("lookup_melody", "search_web"),
            ),
            (
                "включи мелодию из фильма",
                "Не знаю какой именно — в библиотеке пусто.",
                ("gen_search_library",),
            ),
            # LLM нашла и играет через compose_music — это легитимный
            # финал, даже если «не знала изначально».
            (
                "сыграй мелодию из Марио",
                "Сейчас сыграю похожее в духе игры.",
                ("compose_music",),
            ),
            (
                "давай что-то в стиле кантри",
                "Запускаю кантри-импровизацию.",
                ("execute_music_code",),
            ),
        ],
    )
    def test_claim_with_search_or_play_tool_is_not_a_bug(
        self, user_input: str, spoken: str, tools: tuple
    ) -> None:
        """Если хотя бы один поисковый/композиторский тул вызван —
        это легитимный путь, Bug F НЕ срабатывает."""
        assert detect_unknown_melody_claim(
            user_input=user_input, spoken=spoken, tools_called=tools
        ) is False

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # user_input БЕЗ явной просьбы мелодии по имени — Bug F НЕ
            # срабатывает (на «как дела?» не должно быть ретрая).
            ("как дела", "Не знаю, что тебе рассказать."),
            ("расскажи анекдот", "Не знаю ни одного анекдота сейчас."),
            # «не знаю» в spoken, но user_input — не запрос мелодии.
            ("что ты умеешь", "Не знаю, получится ли объяснить."),
            # user_input — запрос мелодии, но spoken не содержит
            # «не-знания» (нормальный ответ «сейчас поищу»).
            (
                "сыграй григ в пещере горного короля",
                "Сейчас поищу ноты и сыграю.",
            ),
            # Играем уже сейчас (tools_called непуст) — Bug F не сработает,
            # но тут tools_called=() как edge case — spoken должен быть
            # «играю», а не «не знаю».
            (
                "сыграй техно",
                "Запускаю басовый техно-бит.",
            ),
        ],
    )
    def test_no_false_positives(self, user_input: str, spoken: str) -> None:
        """Детектор не должен ловить не-мелодийные случаи и обычные
        «играю/ищу» ответы — иначе каждый ответ «не знаю что подарить»
        на общий вопрос уйдёт в ретрай."""
        assert detect_unknown_melody_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is False

    def test_empty_inputs_are_safe(self) -> None:
        """Краевые случаи: None/пусто — детектор не падает."""
        assert (
            detect_unknown_melody_claim(
                user_input=None, spoken="x", tools_called=()
            )
            is False
        )
        assert (
            detect_unknown_melody_claim(
                user_input="x", spoken=None, tools_called=()
            )
            is False
        )
        assert (
            detect_unknown_melody_claim(
                user_input="", spoken="", tools_called=()
            )
            is False
        )

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # Acceptance criteria: 10 промптов с неизвестными композиторами
            # → Bug F должен сработать (т.к. tools=[] и spoken содержит
            # «не знаю»). Каждый кейс — отдельный подтип «не знаю» для
            # разнообразия pattern matching.
            ("сыграй Баха токкату ре минор", "Не знаю такой мелодии наизусть."),
            ("включи Pink Floyd Time", "Не помню этой композиции."),
            ("сыграй Рахманинова прелюдию", "Не знаю точных нот."),
            ("давай Битлз Yesterday", "Не знаю этой мелодии, к сожалению."),
            ("сыграй Шопена ноктюрн", "Не знаю наизусть ноктюрна Шопена."),
            ("включи Леди Гага Poker Face", "Не припоминаю этой песни."),
            ("играй Вивальди Времена года", "Не знаю этой мелодии из Вивальди."),
            ("сыграй Билли Айдол", "Не знаю этого трека."),
            ("включи Элвис Пресли Can't Help", "Не знаю этой песни."),
            ("сыграй Виктор Цой Звезда по имени", "Не знаю этой композиции наизусть."),
        ],
    )
    def test_acceptance_10_prompts_with_unknown_composers(
        self, user_input: str, spoken: str
    ) -> None:
        """Issue #2562 acceptance: «10 промптов с неизвестными композиторами
        → fallback с поиском или альтернативой». Bug F ловит ВСЕ 10 —
        то есть НИ ОДИН из них не уйдёт в TTS как «не знаю» без ретрая."""
        # NB: в этом тесте каждый параметризованный кейс имеет свой
        # spoken (соответствующий стилю композитора), а не общий
        # «Не знаю такой мелодии, могу сыграть что-то похожее.» —
        # чтобы регексп прошёл по разным подтипам «не знаю/не помню».
        assert detect_unknown_melody_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is True


class TestBuildUnknownMelodyRetryPrompt:
    """Issue #2562 Bug F — CRITICAL-ретрай «не знаю → сначала поиск»."""

    def test_prompt_contains_critical_marker(self) -> None:
        """Тот же контракт, что у babble/action-claim ретраев: префикс
        [CRITICAL] + явное требование СНАЧАЛА искать."""
        prompt = build_unknown_melody_retry_prompt("сыграй григ")
        assert "[CRITICAL]" in prompt
        assert "lookup_melody" in prompt
        assert "search_web" in prompt
        assert "gen_search_library" in prompt
        assert "search_melody" in prompt

    def test_prompt_echoes_original_user_input(self) -> None:
        """Юзер-интент в ретрае — оригинальная команда (та же логика, что
        у build_babble_retry_prompt — strip предыдущего [CRITICAL])."""
        prompt = build_unknown_melody_retry_prompt(
            "сыграй имперский марш но в стиле регги"
        )
        assert "сыграй имперский марш но в стиле регги" in prompt

    def test_prompt_orders_search_tools_by_effectiveness(self) -> None:
        """Порядок тулов в ретрае — RTTTL → search_melody → mp3-library
        → web (от эффективного к менее надёжному). Это подсказывает
        LLM, с чего начинать."""
        prompt = build_unknown_melody_retry_prompt("сыграй хит 80х")
        # lookup_melody упоминается раньше search_web (первый в списке).
        lookup_pos = prompt.index("lookup_melody")
        search_web_pos = prompt.index("search_web")
        assert lookup_pos < search_web_pos, (
            "lookup_melody (RTTTL) должен упоминаться ДО search_web — "
            "иначе LLM не поймёт приоритет"
        )

    def test_prompt_forbids_empty_answer(self) -> None:
        """❌ ЗАПРЕЩЕНО «не знаю» без действия — это и есть фикс."""
        prompt = build_unknown_melody_retry_prompt("сыграй хит 80х")
        assert "ЗАПРЕЩЕНО" in prompt
        # Не должно быть фразы типа «можно сказать не знаю» — должен
        # быть явный запрет.
        assert "не знаю" in prompt.lower()

    def test_prompt_offers_alternative_path(self) -> None:
        """Если поиск пуст — LLM должна предложить альтернативы через
        speak_text + compose_music improvisation. Эта ветка ОБЯЗАТЕЛЬНА."""
        prompt = build_unknown_melody_retry_prompt("сыграй хит 80х")
        assert "speak_text" in prompt
        assert "compose_music" in prompt
        # Альтернативы 2-3, не одна.
        assert "2-3" in prompt or "альтернатив" in prompt.lower()

    def test_prompt_handles_none_user_input(self) -> None:
        """None / пустая строка — крайний случай, не должен падать."""
        prompt = build_unknown_melody_retry_prompt(None)
        assert "[CRITICAL]" in prompt
        assert "lookup_melody" in prompt
        prompt2 = build_unknown_melody_retry_prompt("")
        assert "[CRITICAL]" in prompt2

    def test_prompt_strips_previous_critical_block(self) -> None:
        """Если в user_input уже есть предыдущий [CRITICAL]-блок
        (вложенный ретрай), он обрезается — иначе модель читает
        противоречивые инструкции. Контракт тот же, что у babble-retry
        / regurgitate-retry."""
        prompt = build_unknown_melody_retry_prompt(
            "оригинал\n\n[CRITICAL] предыдущий блок"
        )
        assert prompt.count("[CRITICAL]") == 1
        assert "предыдущий блок" not in prompt

    def test_unknown_melody_regex_covers_all_live_variants(self) -> None:
        """UNKNOWN_MELODY_CLAIM_RE покрывает все варианты «не знаю»
        из реального лога 15.09.2026 — регексп не сломан."""
        live_phrases = [
            "Не знаю такой мелодии — могу сыграть похожее.",
            "Не знаю этой мелодии, к сожалению.",
            "Не помню точных нот этой композиции.",
            "Не припоминаю этой песни.",
            "Нет в моей памяти точных нот.",
            "Не знаю наизусть.",
            "Не знаю точных нот, могу сыграть в похожем духе.",
        ]
        for phrase in live_phrases:
            assert UNKNOWN_MELODY_CLAIM_RE.search(phrase), (
                f"UNKNOWN_MELODY_CLAIM_RE должен ловить {phrase!r} "
                "— иначе Bug F не сработает в проде"
            )


# ---------------------------------------------------------------------------
# Issue #2559 — phantom-action detector (общий, НЕ music-only).
#
# Round 3 live-check 15.09.2026 (Vision Pi 10.1.1.21): 6 случаев за час,
# когда LLM говорила «сейчас перезапущу / сделал погуще / подложу слой»
# при tools_called=[]. Существующий Bug E ловит только в DJ-сценарии или
# при music-kw в user_input — бытовые «проверю состояние и перезапущу»
# / «установлю голос» / «перезагружу роутер» проходили мимо.
#
# Этот блок тестов покрывает два ключевых контракта:
#
#   * ``detect_phantom_action_claim`` — чистая функция, возвращает bool
#     по трём условиям (stem в spoken ∧ tools_called пуст ∧ не negation).
#   * ``build_phantom_action_retry_prompt`` — CRITICAL-промпт для ретрая
#     с перечислением тех же verb-стемов.
#
# В issue body прописано «минимум 30 фраз с action verbs (рус, разные
# падежи/лица) с tools=[] → ожидание retry с reason='phantom_action_claim'».
# На уровне guard-функции это означает: ``detect_phantom_action_claim``
# вернёт ``True`` → dialogue_node (``_check_phantom_action_and_retry``)
# ставит ``reason='phantom_action_claim'`` в логе и диспатчит ретрай.
# Интеграционные тесты самого dialogue_node живут в child-тасках.
# ---------------------------------------------------------------------------


class TestPhantomActionClaimLive2559:
    """Issue #2559 — минимальное покрытие 30+ фраз с action-verb."""

    # ----- positive cases (action-verb в spoken + tools=[] ⇒ True) -----

    # Реальные spoken-ы из лога Vision Pi 15.09.2026 — issue #2559
    @pytest.mark.parametrize(
        "user_input, spoken",
        [
            # 1) Из issue #2559, цитаты из реального диалога.
            (
                "ничего не играет",
                "Понимаю, пока не звучит — дай минуту, проверю "
                "состояние и перезапущу.",
            ),
            (
                "опять тишина",
                "Ок, давай я снова перезапущу. Бочкинс с Григом "
                "наверху — стартуя заново.",
            ),
            (
                "сделай соло погуще",
                "Соло сделал погуще, второй голос и пэд подложились.",
            ),
            (
                "не работает звук",
                "Проверю состояние и перезапущу сервис.",
            ),
            (
                "оживи сэт",
                "Подкручу бас и добавлю второй голос.",
            ),
            (
                "поправь бит",
                "Обновил бочку и хет, теперь плотнее.",
            ),
            (
                "можно тише",
                "Подкручу громкость пониже, проверю.",
            ),
            (
                "поменяй темп",
                "Сменю темп на 124 и перезапущу сетку.",
            ),
            (
                "доработай партию",
                "Доработаю партию виолончели и сделаю вход мягче.",
            ),
            (
                "поставь стоп",
                "Остановлю воспроизведение и уберу слой драмсов.",
            ),
            # 11) — разные лица/числа: сделаю/сделаем/сделаешь/сделайте.
            (
                "перезагрузи",
                "Сделаю сейчас, секунду.",
            ),
            (
                "оживи",
                "Сделаем красиво, подожди минуту.",
            ),
            (
                "обнови",
                "Сделаешь? Окей, проверю.",
            ),
            (
                "обнови ещё раз",
                "Сделайте громче — подкручу мастер.",
            ),
            # 15) — причастия (тот же класс claim'а «уже готово»).
            # Все эти формы входят в PHANTOM_ACTION_VERB_STEMS.
            (
                "проверь как сделано",
                "Установлено и проверено, всё готово.",
            ),
            (
                "что с обновлением?",
                "Обновлено, перезапущено, проверьте сами.",
            ),
            (
                "что с роутером?",
                "Перезапущено и проверено.",
            ),
            # 18) — фразы вне music-контекста (issue #2559 acceptance):
            # «перезапущу роутер» — это БЫТ, и guard всё равно должен
            # ловить обещание без тула (важно: только те verbs, которые
            # входят в 13 семейств stems).
            (
                "не открывается сайт",
                "Перезагружу роутер и проверю через минуту.",
            ),
            (
                "проверь как работает",
                "Остановлю сервис и перезапущу его.",
            ),
            (
                "обнови настройки",
                "Обновлю настройки и перезапущу сервис.",
            ),
            (
                "поменяй режим",
                "Поменяю режим на тихий, проверю.",
            ),
            (
                "доработай логику",
                "Доработаю логику, потом перезапущу.",
            ),
            (
                "проверь температуру",
                "Проверю температуру у батареи.",
            ),
            # 24) — глагол установить/настроить (тоже класс action-claim).
            (
                "поставь голос пониже",
                "Установлю голос Алексей и подтвержу.",
            ),
            (
                "сделай режим тишины",
                "Установлю режим тишины и проверю.",
            ),
            (
                "почини микрофон",
                "Перезапущу сервис микрофона.",
            ),
            (
                "поставь на паузу",
                "Остановлю проигрывание и подожду.",
            ),
            # 28) — глагол «подложить / подложился» (live-цитата).
            (
                "что с пэдом?",
                "Пэд подложился под соло, теперь теплее.",
            ),
            # 29) — глагол «подкрутить / подкручу» (музыкальные команды).
            (
                "оживи сэт",
                "Подкручу бас и проверю звук.",
            ),
            # 30) — глагол «обновить / обновлю» (музыка/настройки).
            (
                "обнови плейлист",
                "Обновлю плейлист завтра.",
            ),
            # 31) — многословные action-verb claim'ы из live #2559.
            (
                "поправь бит",
                "Обновил бочку и хет, теперь плотнее.",
            ),
            (
                "поменяй темп",
                "Поменяю темп на 124 и перезапущу сетку.",
            ),
            # 33) — финальный sanity-чек на стемы «сделать» + «запустить».
            (
                "запусти что-нибудь",
                "Запущу новый трек сейчас.",
            ),
        ],
    )
    def test_phantom_action_claim_without_tool_is_detected(
        self, user_input: str, spoken: str
    ) -> None:
        """Acceptance #2559: минимум 30 фраз с action verbs ⇒ True.

        Любая из этих фраз при ``tools_called=()`` приводит к срабатыванию
        guard → ``DialogueNode._check_phantom_action_and_retry`` ставит
        ``reason='phantom_action_claim'`` в лог и диспатчит CRITICAL-ретрай.
        """
        assert detect_phantom_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is True, (
            f"phantom-action guard должен сработать на spoken={spoken!r} "
            f"при user_input={user_input!r} — иначе в #2559 живьём юзер "
            "снова услышит обещание без действия"
        )

    # ----- issue #2942: «сохраняю» present tense (save_arrangement_preset) --

    def test_save_arrangement_preset_present_tense_claim_is_detected(
        self,
    ) -> None:
        """Live 24.09.2026 (issue #2942, ADR-0132 PR-7): «Понял, сохраняю
        эти ручки на «В пещере горного короля»…» ушло в TTS с ``tools=[]``
        перед тем как модель когда-либо вызвала ``save_arrangement_preset``.

        До фикса #2942 present-tense «сохраняю» отсутствовал целиком в
        словаре guard'а — ни в этом (``PHANTOM_ACTION_VERB_STEMS``), ни в
        ``_ACTION_VERBS_PAST``/``_ACTION_VERBS_FUTURE`` (у тех вообще нет
        present tense). Робот мог соврать «сохраняю» без единого тула, и
        НИ ОДИН guard этого не ловил.
        """
        assert detect_phantom_action_claim(
            user_input="ну не знаю, сохрани пресет на всякий случай",
            spoken=(
                "Понял, сохраняю эти ручки на «В пещере горного "
                "короля»…"
            ),
            tools_called=(),
        ) is True, (
            "phantom-action guard должен ловить present-tense «сохраняю» "
            "без вызова save_arrangement_preset (issue #2942 live repro)"
        )

    def test_save_arrangement_preset_claim_with_tool_called_is_not_a_bug(
        self,
    ) -> None:
        """Тот же claim, но тул РЕАЛЬНО вызван — guard должен молчать."""
        assert detect_phantom_action_claim(
            user_input="вот это кайф, сохрани",
            spoken="Сохраняю эти ручки как пресет.",
            tools_called=("save_arrangement_preset",),
        ) is False, (
            "phantom-action guard не должен срабатывать, когда "
            "save_arrangement_preset уже вызван"
        )

    def test_save_arrangement_preset_in_claim_justifying_tools(self) -> None:
        """Issue #2942: тул-словарь #2549-guard'а (``detect_universal_
        action_claim``) должен знать про ``save_arrangement_preset`` —
        иначе past/future tense claim («Сохранил пресет.») после
        РЕАЛЬНОГО вызова тула ложно ловится как phantom action.
        """
        assert "save_arrangement_preset" in CLAIM_JUSTIFYING_TOOLS

        hit = detect_universal_action_claim(
            spoken="Сохранил пресет для этой мелодии.",
            tools_called=("save_arrangement_preset",),
        )
        assert hit is None, (
            "universal action-claim guard не должен срабатывать на "
            "«сохранил» после реального вызова save_arrangement_preset"
        )

    # ----- issue #2949: called-but-ERRORED ≠ backed -----

    def test_tool_called_but_errored_claim_still_detected(self) -> None:
        """Live 24.09.2026 (issue #2949): ``save_arrangement_preset``
        вернул отказ («Инструмент 'save_arrangement_preset' недоступен»),
        LLM всё равно ответила «Записала пресет, горный король теперь
        всегда будет звучать прозрачно!» с ``tools=['save_arrangement_
        preset', 'load_skill']``. До фикса #2942's ``CLAIM_JUSTIFYING_
        TOOLS`` матчил по ИМЕНИ и считал заявление подкреплённым просто
        потому что тул был ВЫЗВАН — не проверяя, что он реально
        сработал. ``tool_error_occurred=True`` обязан НЕ дать тому же
        тулу оправдать claim.
        """
        hit = detect_universal_action_claim(
            spoken=(
                "Записала пресет, горный король теперь всегда будет "
                "звучать прозрачно!"
            ),
            tools_called=("save_arrangement_preset", "load_skill"),
            tool_error_occurred=True,
        )
        assert hit is not None, (
            "guard должен ловить claim, когда закрывающий тул был вызван, "
            "но вернул ошибку/отказ (issue #2949 live repro) — а не "
            "молчать просто потому что имя тула есть в tools_called"
        )
        assert hit.verb.lower().startswith("записал")

    def test_tool_called_and_succeeded_claim_not_detected(self) -> None:
        """Контраст к предыдущему тесту: тул реально сработал — guard молчит.

        ``tool_error_occurred=False`` (по умолчанию) — старое поведение
        не должно регрессировать.
        """
        hit = detect_universal_action_claim(
            spoken="Записала пресет, звучит прозрачно.",
            tools_called=("save_arrangement_preset",),
            tool_error_occurred=False,
        )
        assert hit is None, (
            "guard НЕ должен срабатывать, когда закрывающий тул реально "
            "успешно отработал"
        )

    # ----- negative case 1: tools_called не пуст ⇒ guard молчит -----

    @pytest.mark.parametrize(
        "spoken, called_tool",
        [
            ("Перезапущу роутер через минуту.", "restart_router"),
            ("Подложу пэд и проверю звук.", "compose_music"),
            ("Проверю состояние музыки.", "get_music_state"),
            ("Установлю голос Алексей.", "set_voice"),
            ("Перезагружу сервис микрофона.", "restart_mic"),
            ("Запущу мелодию.", "execute_music_code"),
            ("Остановлю воспроизведение.", "stop_music"),
            ("Поменяю плейлист.", "load_track"),
            ("Загружу новый трек.", "load_track"),
            ("Обновлю партию.", "compose_music"),
        ],
    )
    def test_claim_with_some_tool_is_not_a_bug(
        self, spoken: str, called_tool: str
    ) -> None:
        """Если робот действительно вызвал тул — claim оправдан, ретрай не нужен.

        Те же live-фразы, что и в positive, но с непустым ``tools_called``.
        """
        assert detect_phantom_action_claim(
            user_input="проверь как сделано",
            spoken=spoken,
            tools_called=(called_tool,),
        ) is False, (
            f"phantom-action guard НЕ должен срабатывать на spoken={spoken!r} "
            f"когда уже вызван тул {called_tool!r} — иначе будет ping-pong "
            "на легитимных действиях"
        )

    # ----- negative case 2: spoken без action-verb ⇒ guard молчит -----

    @pytest.mark.parametrize(
        "spoken",
        [
            "Привет, как дела?",
            "Сейчас тишина, ничего не играет.",
            "Я тебя слышу.",
            "Хорошо, понял.",
            "Расскажи анекдот.",
            "Какая сегодня погода?",
            "Сколько времени?",
            "Спасибо!",
            "Договорились.",
            "Все нормально.",
        ],
    )
    def test_spoken_without_action_verb_is_not_detected(
        self, spoken: str
    ) -> None:
        """Просто информативный ответ — глаголов нет ⇒ guard молчит."""
        assert detect_phantom_action_claim(
            user_input="что-нибудь скажи",
            spoken=spoken,
            tools_called=(),
        ) is False, (
            f"phantom-action guard НЕ должен срабатывать на "
            f"spoken={spoken!r} — нет action-verb, нечего ретраить"
        )

    # ----- negative case 3: only punctuation / interjection ⇒ guard молчит -----

    @pytest.mark.parametrize(
        "spoken",
        [
            "...",
            "!!!",
            "Ага.",
            "Хмм.",
            "Угу.",
            "Ну.",
            "Ок.",
            "Ладно.",
            "?",
            "...",
            "",
        ],
    )
    def test_only_punctuation_and_interjection_is_not_detected(
        self, spoken: str
    ) -> None:
        """Только междометие / пунктуация — нет action verb ⇒ нет ретрая."""
        assert detect_phantom_action_claim(
            user_input="ну что?",
            spoken=spoken,
            tools_called=(),
        ) is False, (
            "междометие/пунктуация не должны триггерить phantom-action"
        )

    # ----- negative case 4: user_input с «не буду» / «не надо» ⇒ guard молчит -----

    @pytest.mark.parametrize(
        "user_input, spoken",
        [
            (
                "не надо ничего перезапускать, оставь как есть",
                "Ок, перезапускать не буду, оставлю как есть.",
            ),
            (
                "не делай ничего",
                "Окей, делать не буду.",
            ),
            (
                "не надо проверять, я сам",
                "Не буду проверять, договорились.",
            ),
            (
                "давай не будем трогать",
                "Не буду ничего трогать.",
            ),
            (
                "ничего не надо делать",
                "Ок, ничего делать не буду.",
            ),
            (
                "оставим как есть",
                "Перезапускать не буду, всё как было.",
            ),
            (
                "не надо ставить будильник",
                "Не буду ставить будильник.",
            ),
        ],
    )
    def test_user_negation_disables_guard(
        self, user_input: str, spoken: str
    ) -> None:
        """«не буду / не надо / не делай» — легитимный отказ, не ретраим.

        Без этого guard'а ретрай срабатывал бы на легитимный
        «Не буду перезапускать, давай так оставим» — это отказ, а не
        невыполненное действие (issue #2559 acceptance).
        """
        assert detect_phantom_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is False, (
            f"phantom-action guard НЕ должен срабатывать при явном "
            f"отказе в user_input={user_input!r} (spoken={spoken!r}) — "
            "иначе ретрай после отказа пользователя"
        )

    # ----- negative case 5: пустые/пробельные spoken ⇒ guard молчит -----

    @pytest.mark.parametrize(
        "user_input, spoken, tools_called",
        [
            # spoken=None/"" → guard молчит (даже если есть action-stem).
            ("Перезагрузи", None, ()),
            ("Перезагрузи", "", ()),
            ("Перезагрузи", "   ", ()),
            # Полностью пустые входы.
            (None, None, ()),
            ("", "", ()),
            # Случай с пользовательским input и claim'ом в spoken
            # (НЕ пустой spoken) здесь НЕ проверяется — это
            # контракт ``test_none_user_input_does_not_block_guard``,
            # чтобы избежать дублирования и противоречия.
        ],
    )
    def test_empty_inputs_are_safe(
        self,
        user_input: Optional[str],
        spoken: Optional[str],
        tools_called: tuple,
    ) -> None:
        """Пустой spoken / пустой user_input / None — False, не падаем."""
        assert (
            detect_phantom_action_claim(
                user_input=user_input,
                spoken=spoken,
                tools_called=tools_called,
            )
            is False
        )

    # ----- negative case 6: tools_called как кортеж/список любого размера >0 ⇒ False -----

    @pytest.mark.parametrize(
        "tools_called",
        [
            ("any_tool",),
            ("a", "b"),
            ("a", "b", "c"),
        ],
    )
    def test_tools_called_truthy_disables_guard(
        self, tools_called: tuple
    ) -> None:
        """Любой НЕпустой набор tools оправдывает spoken claim."""
        assert (
            detect_phantom_action_claim(
                user_input="проверь",
                spoken="Сделал и проверил.",
                tools_called=tools_called,
            )
            is False
        )

    # ----- negative case 7: noun-suffix «проверка / остановка» НЕ триггерит -----

    @pytest.mark.parametrize(
        "user_input, spoken",
        [
            (
                "что с проверкой?",
                "Проверка пройдена, идём дальше.",
            ),
            (
                "что с обновлением?",
                "Обновление системы завершено.",
            ),
            (
                "что с остановкой?",
                "Остановка не потребовалась.",
            ),
            (
                "как изменение?",
                "Изменение параметров в процессе.",
            ),
            (
                "что с доработкой?",
                "Доработка в очереди, не начиналась.",
            ),
        ],
    )
    def test_noun_forms_of_action_verbs_are_not_detected(
        self, user_input: str, spoken: str
    ) -> None:
        """«проверка / обновление / остановка» — это СУЩЕСТВИТЕЛЬНЫЕ, не verb-stems.

        Negative lookahead ``PHANTOM_NOUN_SUFFIXES`` отбрасывает их —
        иначе guard бы реагировал на описание ЧТО СДЕЛАНО, а не на
        обещание СДЕЛАТЬ. Это defense-in-depth, по той же причине,
        что bug-detection'ы не ловят существительные.
        """
        assert (
            detect_phantom_action_claim(
                user_input=user_input, spoken=spoken, tools_called=()
            )
            is False
        ), (
            f"noun-форма в spoken={spoken!r} не должна триггерить guard — "
            "иначе каждый отчёт о состоянии даст ложный ретрай"
        )

    # ----- regression-safety net: PHANTOM_ACTION_VERBS_RE не покрывает ложные stems -----

    @pytest.mark.parametrize(
        "spoken",
        [
            # То, что НЕ должно попадать в verbs_re:
            "Я съем пирожок.",  # «съем» — еда, не action в контексте
            "Он делает успехи.",  # «делает» — здесь другая морфология,
                                  # но не входит в stems по построению
            "Будет сидеть и ждать.",  # «будет …» — футур, но не action-stem
            "Тестовая фраза без action.",  # без глагола
        ],
    )
    def test_non_action_verbs_are_not_detected(self, spoken: str) -> None:
        """Стоп-слова и прочие false-positive сценарии — guard молчит.

        Проверка, что PHANTOM_ACTION_VERBS_RE не слишком широк (важно,
        иначе каждый короткий ответ даст ложный ретрай).
        """
        assert (
            detect_phantom_action_claim(
                user_input="расскажи",
                spoken=spoken,
                tools_called=(),
            )
            is False
        )

    # ----- regression-safety: user_input None не должен ломать -----

    @pytest.mark.parametrize(
        "user_input, spoken",
        [
            (None, "Сделаю сейчас."),
            (None, "Перезапущу сервис."),
            (None, "Обновлю плейлист."),
        ],
    )
    def test_none_user_input_does_not_block_guard(
        self, user_input: Optional[str], spoken: str
    ) -> None:
        """Если user_input=None — negation-check не должен валиться.

        Контракт: ``PHANTOM_ACTION_NEGATION_RE.search(None)`` ⇒ False,
        и guard срабатывает на action verbs в spoken.
        """
        assert detect_phantom_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is True, (
            f"None user_input не должен блокировать guard, "
            f"но spoken={spoken!r} не сработал"
        )

    # ----- regression-safety: regex и negation-regex не пусты -----

    def test_phantom_action_verbs_re_is_not_empty(self) -> None:
        """PHANTOM_ACTION_VERBS_RE — основной detector regex, не должен быть пуст."""
        assert PHANTOM_ACTION_VERBS_RE.pattern
        # Smoke-match: должен ловить «сделаю», «перезапущу» и т.п.
        assert PHANTOM_ACTION_VERBS_RE.search("я сейчас сделаю")
        assert PHANTOM_ACTION_VERBS_RE.search("перезапущу сервис")
        assert PHANTOM_ACTION_VERBS_RE.search("проверю состояние")

    def test_phantom_action_negation_re_is_not_empty(self) -> None:
        """PHANTOM_ACTION_NEGATION_RE — защита от отказа, не должна быть пустой."""
        assert PHANTOM_ACTION_NEGATION_RE.pattern
        # Smoke-match: должна ловить «не буду».
        assert PHANTOM_ACTION_NEGATION_RE.search("не буду перезапускать")
        assert PHANTOM_ACTION_NEGATION_RE.search("не надо проверять")


class TestBuildPhantomActionRetryPrompt:
    """Issue #2559 — CRITICAL-ретрай «обещал действие без тула»."""

    def test_prompt_contains_critical_marker(self) -> None:
        """Контракт ретраев guard'ов: префикс [CRITICAL] + требование tool."""
        prompt = build_phantom_action_retry_prompt("перезгрузи роутер")
        assert "[CRITICAL]" in prompt

    def test_prompt_echoes_original_user_input(self) -> None:
        """Юзер-интент в ретрае — оригинальная команда (та же логика,
        что у build_babble_retry_prompt / build_unknown_melody_retry_prompt —
        strip предыдущего [CRITICAL], чтобы LLM не читала противоречивые
        инструкции)."""
        prompt = build_phantom_action_retry_prompt(
            "проверь состояние и перезапусти музыку"
        )
        # Оригинальная формулировка юзера сохранена.
        assert "проверь состояние и перезапусти музыку" in prompt

    def test_prompt_lists_action_verbs(self) -> None:
        """Промпт должен перечислять те же verb-stems, чтобы LLM УВИДЕЛА
        в ретрае то же слово, которое сама использовала (а не догадывалась).
        """
        prompt = build_phantom_action_retry_prompt("обнови")
        # Все ключевые stems должны быть явно упомянуты в промпте —
        # это требование issue #2559 acceptance для совпадения LLM-слова.
        for stem in (
            "сделал",
            "запустил",
            "перезапущу",
            "установлю",
            "остановлю",
            "проверю",
            "подложу",
            "переключу",
            "подкручу",
            "обновлю",
            "поменяю",
            "изменю",
            "доработаю",
        ):
            assert stem in prompt, (
                f"stem {stem!r} должен быть в промпте — иначе LLM не "
                "увидит параллель со своим ответом"
            )

    def test_prompt_forbids_claiming_without_tool(self) -> None:
        """HONESTY RULE — запрет обещания без tool-call."""
        prompt = build_phantom_action_retry_prompt("обнови")
        # Промпт должен явно ЗАПРЕЩАТЬ обещание без tool.
        assert "ЗАПРЕЩЕНО" in prompt

    def test_prompt_demands_tool_or_speak_text(self) -> None:
        """Два легитимных пути: tool-call ИЛИ speak_text без action-verb."""
        prompt = build_phantom_action_retry_prompt("обнови")
        # Сценарий 1 — вызвать tool.
        assert "tool" in prompt.lower() or "инструмент" in prompt.lower()
        # Сценарий 2 — устный ответ без action-verb (speak_text).
        assert "speak_text" in prompt

    def test_prompt_handles_none_user_input(self) -> None:
        """None / пустая строка — не должен падать, должен быть валидный промпт."""
        prompt = build_phantom_action_retry_prompt(None)
        assert "[CRITICAL]" in prompt
        # Сам текст юзера — пустой, но промпт всё равно собирается.
        prompt2 = build_phantom_action_retry_prompt("")
        assert "[CRITICAL]" in prompt2

    def test_prompt_strips_previous_critical_block(self) -> None:
        """Если в user_input уже есть предыдущий [CRITICAL]-блок (вложенный
        ретрай), он обрезается — иначе модель читает противоречивые
        инструкции. Контракт тот же, что у других CRITICAL-ретраев."""
        prompt = build_phantom_action_retry_prompt(
            "оригинал\n\n[CRITICAL] предыдущий блок"
        )
        assert prompt.count("[CRITICAL]") == 1
        assert "предыдущий блок" not in prompt

    def test_prompt_starts_with_user_input(self) -> None:
        """Юзер-интент в начале промпта, чтобы модель сразу поняла
        контекст, затем CRITICAL-блок (это контракт ретраев)."""
        prompt = build_phantom_action_retry_prompt("обнови плейлист")
        # Контекстная часть — до перевода строки \n\n
        head, _, _ = prompt.partition("\n\n")
        assert "обнови плейлист" in head, (
            "user_input должен быть в начале промпта ДО [CRITICAL]-блока — "
            "иначе LLM сначала прочтёт инструкции и забудет запрос"
        )


class TestBuildUniversalActionClaimRetryPromptToolError:
    """Issue #2949 — retry-промпт должен различать «тул не вызван» и
    «тул вызван, но упал».

    :func:`build_universal_action_claim_retry_prompt` раньше всегда
    писал «но НЕ вызвал НИ ОДНОГО инструмента (tools=[])», что было бы
    ложью для ``tools=['save_arrangement_preset', 'load_skill']`` —
    инструмент КАК РАЗ был вызван, просто отказал. ``tool_error_
    occurred=True`` переключает текст на честную формулировку про
    ошибку/отказ и просит либо повторить, либо честно сообщить о
    неудаче — НЕ просто «вызови тул ещё раз» вслепую.
    """

    def _hit(self):
        hit = detect_universal_action_claim(
            spoken="Записала пресет.",
            tools_called=("save_arrangement_preset",),
            tool_error_occurred=True,
        )
        assert hit is not None
        return hit

    def test_error_prompt_mentions_failure_not_empty_tools(self) -> None:
        prompt = build_universal_action_claim_retry_prompt(
            user_input="сохрани пресет",
            spoken="Записала пресет.",
            hit=self._hit(),
            tool_error_occurred=True,
        )
        assert "tools=[]" not in prompt, (
            "тул БЫЛ вызван (tools_called непуст) — промпт не должен "
            "врать, что вызовов не было"
        )
        assert "ошиб" in prompt.lower() or "отказ" in prompt.lower(), (
            "промпт обязан назвать РЕАЛЬНУЮ причину — ошибку/отказ тула, "
            "а не общее «вызови инструмент»"
        )

    def test_error_prompt_asks_for_honest_failure_report(self) -> None:
        prompt = build_universal_action_claim_retry_prompt(
            user_input="сохрани пресет",
            spoken="Записала пресет.",
            hit=self._hit(),
            tool_error_occurred=True,
        )
        assert "не получилось" in prompt.lower() or "не удал" in prompt.lower(), (
            "промпт должен явно разрешить честно сказать о неудаче — "
            "acceptance criteria #2949"
        )

    def test_no_error_prompt_keeps_legacy_wording(self) -> None:
        """``tool_error_occurred=False`` (default) — старый текст не регрессирует."""
        hit = detect_universal_action_claim(
            spoken="Проверю состояние и перезапущу.",
            tools_called=(),
        )
        assert hit is not None
        prompt = build_universal_action_claim_retry_prompt(
            user_input="докрути музыку",
            spoken="Проверю состояние и перезапущу.",
            hit=hit,
        )
        assert "tools=[]" in prompt


class TestBuildActionClaimFailureFallback:
    """Issue #2949 — честная фраза после исчерпания бюджета ретраев.

    Когда одноразовый ретрай уже потрачен и модель СНОВА повторяет
    непокреплённый claim, fallback обязан НЕ содержать слов успеха
    («записал», «сохранил», «готово») — иначе юзер услышит вторую ложь
    подряд вместо честного признания неудачи (ADR-0018).
    """

    def test_fallback_does_not_repeat_success_claim(self) -> None:
        hit = detect_universal_action_claim(
            spoken="Записала пресет.",
            tools_called=("save_arrangement_preset",),
            tool_error_occurred=True,
        )
        assert hit is not None
        fallback = build_action_claim_failure_fallback(hit)
        lowered = fallback.lower()
        for success_word in ("записал", "сохранил", "готово", "выполнил"):
            assert success_word not in lowered, (
                f"fallback не должен содержать {success_word!r} — это "
                "должна быть честная неудача, не повторный claim"
            )

    def test_fallback_is_non_empty_string(self) -> None:
        hit = detect_universal_action_claim(
            spoken="Записала пресет.",
            tools_called=("save_arrangement_preset",),
            tool_error_occurred=True,
        )
        assert hit is not None
        assert isinstance(build_action_claim_failure_fallback(hit), str)
        assert build_action_claim_failure_fallback(hit).strip() != ""


class TestPhantomActionVsBugEOverlap:
    """Issue #2559 vs #992 Bug E — разные скоупы, но оба ловят action-claim.

    Bug E узкий (только music-context с ``user_re``∧``claim_re`` ∧ tools пуст).
    Phantom-action шире (любой action-verb в spoken + tools пуст +
    не negation), пересекается с Bug E в music-сценариях — там оба
    дадут ``True``/``rule``. Главное, что phantom-action НЕ СБИВАЕТ
    music-only контракт Bug E.
    """

    def test_music_claim_both_detectors_fire(self) -> None:
        """Music claim + tools=[]: оба detector'а должны сработать."""
        user_input = "обнови бит"
        spoken = "Обновил бочку и хет."
        # Bug E — по правилу music_prose_action.
        bug_e_rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
            dj_active=True,
        )
        assert bug_e_rule is not None, (
            "Bug E должен ловить music_prose_action при dj_active=True"
        )
        # Phantom — общий detector.
        phantom = detect_phantom_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
        )
        assert phantom is True, (
            "phantom-action guard тоже должен сработать — общий "
            "(НЕ music-only) detector"
        )

    def test_non_music_claim_only_phantom_fires(self) -> None:
        """«перезгружу роутер» — Bug E молчит (нет music правила), phantom срабатывает.

        Это ГЛАВНЫЙ acceptance #2559: расширение Bug E на ВСЕ action-claims,
        а не только на music.
        """
        user_input = "не открывается сайт"
        spoken = "Перезагружу роутер и проверю через минуту."
        # Bug E — НЕ должен сработать (правило «waypoint_save» не подходит,
        # «library_search» не подходит, «music_prose_action» требует
        # music-kw или dj_active).
        bug_e_rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
            dj_active=False,
        )
        assert bug_e_rule is None, (
            "Bug E НЕ должен ловить бытовой «перезагружу роутер» — "
            "иначе ретрай пойдёт на бытовые обещания, что и было старой "
            "проблемой; #2559 требует именно phantom для этого сценария"
        )
        # Phantom — ДОЛЖЕН сработать.
        assert detect_phantom_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        ) is True, (
            "phantom-action guard ОБЯЗАН сработать на бытовом "
            "«перезагружу роутер» при tools=[] — главный acceptance #2559"
        )

    def test_negation_disables_only_phantom_not_bug_e(self) -> None:
        """«не буду / не надо» — phantom молчит, Bug E работает по своим правилам.

        Контракт: защита от ложного срабатывания на отказ есть только
        у phantom (Bug E ориентирован на user_re-шаблоны, а не на «не буду»).
        """
        user_input = "не надо ставить будильник"
        spoken = "Не буду ставить будильник."
        # Phantom — НЕ сработает (negation в user_input).
        phantom = detect_phantom_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
        )
        # Bug E — посмотрим: «не надо ставить будильник» не подходит ни под
        # какое правило (нет save_waypoint / нет library_search / нет
        # music_prose_action без music-kw и т.п.).
        bug_e_rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=(),
            dj_active=False,
        )
        # Оба — False. Главное: phantom НЕ сработал из-за negation.
        assert phantom is False, (
            "negation в user_input ОБЯЗАНА отключать phantom-action — "
            "иначе ретрай после легитимного отказа"
        )
        assert bug_e_rule is None, (
            "Bug E не должен ловить этот бытовой сценарий — это "
            "и есть мотивация выделить phantom в отдельный guard"
        )

