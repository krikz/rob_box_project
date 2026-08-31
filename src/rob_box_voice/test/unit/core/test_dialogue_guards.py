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
    ACTION_CLAIM_RULES,
    BABBLE_BANNED_OPENERS,
    BABBLE_PERFORMANCE_KEYWORDS,
    MUSIC_GUARD_KEYWORDS,
    MUSIC_GUARD_VOCAL_KEYWORDS,
    MUSIC_RETRY_PROMPT_PREFIX,
    MUSIC_STOP_OVERRIDES,
    TOOL_REQUEST_PATTERNS,
    build_babble_retry_prompt,
    build_music_retry_prompt,
    build_renardo_code_retry_prompt,
    build_tool_retry_prompt,
    build_unbacked_action_retry_prompt,
    detect_required_tool,
    detect_unbacked_action_claim,
    extract_renardo_code_lines,
    is_metalanguage_babble,
    is_music_stop_command,
    is_state_question,
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
    ``dialog_core`` называл свою копию «mirrors rob_box_voice…» и обещал,
    что списки «stay in sync» — прочитав это, легко «починить» расхождение
    слиянием и молча снять требование бита с рэпа.
    """
    from rob_box_harness.core.dialog_core import _VOCAL_REQUEST_KEYWORDS

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
            # Факт в память — не точка; save_waypoint тут ни при чём.
            ("запомни что я люблю зеленый чай без сахара", "Запомнила."),
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
        src = self._dialogue_node_source()
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
        src = self._dialogue_node_source()
        stop_branch = src.index("tools_now & MUSIC_STOP_TOOLS")
        starters_branch = src.index("if tools_now & _music_starters")
        assert stop_branch < starters_branch, (
            "сброс обязан идти ДО ветки запуска, иначе она будет затёрта"
        )


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
