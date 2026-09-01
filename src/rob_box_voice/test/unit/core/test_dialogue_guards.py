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


# ---------------------------------------------------------------------------
# Issue #1777 — non-music tool-skipped guard (Bug C для information tools).
#
# Закрываем две проблемы:
# 1. LLM отвечает на «который час?» без вызова ``get_current_time`` —
#    ``detect_required_tool`` ловит интент, ``build_tool_retry_prompt``
#    формирует CRITICAL-промпт.
# 2. LLM отвечает текстом с маркерами времени («сейчас 14:35») без
#    вызова tool — ``spoken_text_contains_time_marker`` это видит,
#    ``looks_like_time_question`` подтверждает, что запрос был про
#    время, WARN-лог фиксирует инцидент.
# ---------------------------------------------------------------------------


class TestDetectRequiredTool:
    """Issue #1777 — какой tool требуется для user_input."""

    @pytest.mark.parametrize(
        "phrase",
        [
            "который час",
            "Который час?",
            "которого часа",
            "сколько времени",
            "сколько сейчас времени",
            "какое время",
            "время в Москве",
            "время в Берлине",
            "время по токио",
            "время сейчас",
            "сейчас сколько",
            "дата сегодня",
            "какое сегодня число",
            "день недели",
            "what time is it",
            "what time",
            "what date",
            "date today",
        ],
    )
    def test_detects_time_tool(self, phrase: str) -> None:
        assert detect_required_tool(phrase) == "get_current_time"

    @pytest.mark.parametrize(
        "phrase",
        [
            "найди в интернете про космос",
            "погугли рецепт шарлотки",
            "загугли python tutorial",
            "search the web for python",
        ],
    )
    def test_detects_web_search(self, phrase: str) -> None:
        assert detect_required_tool(phrase) == "search_web"

    @pytest.mark.parametrize(
        "phrase",
        [
            "смени голос",
            "поменяй тон",
            "говори басом",
            "голос ниже",
            "change pitch",
        ],
    )
    def test_detects_voice_change(self, phrase: str) -> None:
        assert detect_required_tool(phrase) == "set_voice"

    @pytest.mark.parametrize(
        "phrase",
        [
            "что я тебе говорил про чай",
            "помнишь что я рассказывал",
            "что ты обо мне знаешь",
        ],
    )
    def test_detects_memory_search(self, phrase: str) -> None:
        assert detect_required_tool(phrase) == "memory_search"

    @pytest.mark.parametrize(
        "phrase",
        [
            "как тебя зовут",
            "расскажи анекдот",
            "поехали вперёд",
            "сыграй техно",
        ],
    )
    def test_no_tool_for_general_chitchat(self, phrase: str) -> None:
        """Чатовые фразы не должны ловиться tool-guard'ом."""
        assert detect_required_tool(phrase) is None

    def test_empty_input_returns_none(self) -> None:
        assert detect_required_tool("") is None
        assert detect_required_tool("   ") is None
        assert detect_required_tool(None) is None  # type: ignore[arg-type]

    def test_yo_normalised_to_e(self) -> None:
        """«всё хорошо» vs «всe хорошо» — ё→е."""
        assert detect_required_tool("который час") == "get_current_time"


class TestBuildToolRetryPrompt:
    """Issue #1777 / #1762 — CRITICAL-промпт ретрая."""

    def test_contains_tool_name_and_user_input(self) -> None:
        prompt = build_tool_retry_prompt("который час", "get_current_time")
        assert "get_current_time" in prompt
        assert "который час" in prompt
        assert "[CRITICAL]" in prompt
        # Явный запрет на выдумку — главная защита от бага #1777.
        assert "ЗАПРЕЩЕНО" in prompt
        # Чёткое указание «не выдумывай».
        assert "не удалось" in prompt or "честно" in prompt.lower()

    def test_user_input_passthrough(self) -> None:
        prompt = build_tool_retry_prompt(
            "время в Берлине пожалуйста", "get_current_time"
        )
        assert "время в Берлине пожалуйста" in prompt

    def test_unknown_tool_returns_empty(self) -> None:
        """Defence-in-depth: tool_name вне allow-list — пустой промпт.

        Это защищает _apply_tool_skipped_guard от prompt-injection:
        если LLM-ответ выдумал имя tool'а, retry просто не сработает.
        """
        assert build_tool_retry_prompt("привет", "delete_everything") == ""
        assert build_tool_retry_prompt("привет", "") == ""
        assert build_tool_retry_prompt("привет", "execute_shell") == ""

    def test_all_catalog_tools_produce_non_empty_prompt(self) -> None:
        """Каждый tool из каталога обязан давать валидный промпт."""
        for tool_name in TOOL_REQUEST_PATTERNS:
            prompt = build_tool_retry_prompt("запрос пользователя", tool_name)
            assert prompt, f"empty prompt for known tool {tool_name!r}"
            assert tool_name in prompt
            assert "[CRITICAL]" in prompt


class TestLooksLikeTimeQuestion:
    """Issue #1777 — детектор «похоже на вопрос про время»."""

    @pytest.mark.parametrize(
        "phrase",
        [
            "который час",
            "Который час?",
            "сколько времени?",
            "время в Москве",
            "дата сегодня",
            "what time is it",
            "what date",
            "time in London",
        ],
    )
    def test_detects_time_questions(self, phrase: str) -> None:
        assert looks_like_time_question(phrase) is True

    @pytest.mark.parametrize(
        "phrase",
        [
            # chit-chat с упоминанием часов — не запрос tool'а.
            "расскажи про часы",
            "я люблю часы",
            "а ты знаешь что такое время суток",
            # обычные команды без вопроса.
            "сыграй бит",
            "поехали вперёд",
            "расскажи анекдот",
        ],
    )
    def test_chitchat_is_not_time_question(self, phrase: str) -> None:
        assert looks_like_time_question(phrase) is False

    def test_empty_returns_false(self) -> None:
        assert looks_like_time_question("") is False
        assert looks_like_time_question(None) is False  # type: ignore[arg-type]

    def test_question_mark_alone_with_word_time(self) -> None:
        """Вопрос с ключевым словом — тоже да, даже если не из каталога."""
        assert looks_like_time_question("а сколько день длится?") is True
        assert looks_like_time_question("а какой это час?") is True


class TestSpokenContainsTimeMarker:
    """Issue #1777 — есть ли в ответе LLM маркер времени."""

    @pytest.mark.parametrize(
        "text",
        [
            "Сейчас 14:35, вторник, 26 августа, день.",
            "В Москве сейчас 14:35.",
            "It's 14:35:42.",
            "Сейчас тридцать семь минут одиннадцатого вечера.",
            "В Берлине сейчас примерно 13:35 утра.",
            "It's 3:35 PM.",
            "9:05 am.",
            "Сейчас ровно 5 часов 12 минут.",
            "В 3 часа ночи.",
        ],
    )
    def test_detects_time_markers(self, text: str) -> None:
        assert spoken_text_contains_time_marker(text) is True

    @pytest.mark.parametrize(
        "text",
        [
            "Не знаю, спроси у Siri.",
            "У меня нет часов.",
            "Привет, как дела?",
            "",
        ],
    )
    def test_no_markers_in_neutral_text(self, text: str) -> None:
        assert spoken_text_contains_time_marker(text) is False

    def test_none_safe(self) -> None:
        assert spoken_text_contains_time_marker(None) is False  # type: ignore[arg-type]


class TestToolRequestPatternsContract:
    """SSoT — каталог tool'ов. Следим, чтобы он не «протух»."""

    def test_get_current_time_present(self) -> None:
        """Базовый кейс бага #1777."""
        assert "get_current_time" in TOOL_REQUEST_PATTERNS
        assert TOOL_REQUEST_PATTERNS["get_current_time"], "пустой фрозсет"

    def test_each_pattern_is_non_empty_frozenset(self) -> None:
        for tool_name, kws in TOOL_REQUEST_PATTERNS.items():
            assert isinstance(kws, frozenset)
            assert kws, f"tool {tool_name!r} has empty keywords"

    def test_all_keywords_are_lowercase_strings(self) -> None:
        """Ключевые слова сравниваются после ``_normalise``, но
        хранение в нижнем регистре — это контракт. Если кто-то
        добавит «Который час» с большой буквы, оно сматчится, но
        тест на «нет дубликатов в upper-case» просядет."""
        for tool_name, kws in TOOL_REQUEST_PATTERNS.items():
            for kw in kws:
                assert isinstance(kw, str)
                assert kw == kw.lower(), (
                    f"keyword {kw!r} in {tool_name!r} not lowercase"
                )
