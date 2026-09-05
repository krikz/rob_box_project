"""Issue #1882 — planning-narration hard-mute в _handle_result.

Live 02.09.2026 (vision-pi 15:01): MiniMax-M3 при выключенном thinking
выдаёт НЕ финальный ответ, а ВНУТРЕННИЙ МОНОЛОГ — 2512 символов, 16
TTS-чанков. Юзер слышит кухню модели.

Bug D (babble-retry) ловит только meta-promise («зачитаю», «погнали»);
planning narration выглядит иначе (без babble-opener'ов), и когда
babble-ретрай уже потрачен — monologue всё равно уходит в TTS.

Контракт guard'а (acceptance карточки):

* planning spoken + tools_called=() + speak_text_real=0 → TTS НЕ
  публикуется, есть diagnostic лог «🤐 [issue 1882]»;
* тот же spoken при is_babble_retry=True / _babble_retry_used=True
  → всё равно НЕ озвучивается (регресс именно этого бага);
* нормальный ответ с упоминанием имени тула при speak_text_real>0 →
  озвучивается (НЕ сжигать легитимные ответы);
* babble-фразы («Погнали, зачитаю рэп про космос!») → это другой
  guard, сюда не падают.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.dialogue_node import DialogueNode


def _make_node() -> DialogueNode:
    """Минимальная DialogueNode для вызова ``_handle_result`` через ``__new__``."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._tts_control_pub = MagicMock()
    n._music_cleanup_pub = MagicMock()
    n._dj_mode_pub = None
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._active_tg_chat_id = None
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._effects = MagicMock()
    n._effects.handle_tts_finished = MagicMock()
    n._effects.handle_sound_state = MagicMock()
    n._verbose_llm = False
    # Retry-флаги — должны быть инициализированы, чтобы
    # ``_check_*_and_retry`` не упал в AttributeError.
    n._babble_retry_used = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    n._track_mode_music_active = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    return n


def _make_result(spoken: str, tools=None, real_count: int = 0) -> DialogResult:
    """Реальный DialogResult с полем speak_text_real_count."""
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools or []),
        speak_text_real_count=real_count,
        finish_reason="stop",
    )


def _published_texts(node: DialogueNode) -> list[str]:
    """Тексты, реально опубликованные в /voice/dialogue/response."""
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


# Прямой паттерн из живого лога карточки, len=2512 в оригинале.
PLANNING_SPOKEN_2512 = (
    "Юзер Иван (65e62885) — оператор. DJ-сет Пауля Оакенфольда уже играет "
    "(dj=\"playing: супер вечеринка...\", ai=\"idle\", beat=\"active\").\n\n"
    "Но [CRITICAL] говорит, что в прошлом цикле я НЕ вызвал composemusic, "
    "и DJ был без музыки. Сейчас нужно сделать переход #1 — следующий трек в сете...\n\n"
    "[CRITICAL] ещё раз подчёркивает: НЕ метатекст, ТОЛЬКО composemusic. "
    "Но в инструкции написано сначала исследовать, потом план...\n\n"
    "Решение: фокус — запустить трек через composemusic.\n\n"
    "Аргументы для composemusic в стиле Оакенфольда:\n"
    "bpm: 140 (классический транс)\n"
    "root: A (ля)\n"
    "scale: minor\n"
    "form: buildup..."
)


class TestPlanningNarrationHardMute:
    """Главный контракт из карточки: planning + пусто + babble уже был
    → TTS НЕ публикуется."""

    def test_planning_with_empty_tools_and_zero_real_count_does_not_publish(self):
        """Acceptance #1: planning narration + tools=() → TTS НЕ происходит."""
        n = _make_node()
        result = _make_result(
            spoken=PLANNING_SPOKEN_2512,
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="сыграй рэп")

        published = _published_texts(n)
        assert published == [], (
            f"planning narration должна быть заглушена, "
            f"но опубликовано: {published!r}"
        )

    def test_planning_with_empty_tools_logs_diagnostic(self):
        """В лог уходит warning с тегом [issue 1882] — для диагностики."""
        n = _make_node()
        result = _make_result(
            spoken=PLANNING_SPOKEN_2512,
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="сыграй рэп")

        # Logger.warning должен быть вызван хотя бы раз с тегом issue 1882.
        warning_calls = n.get_logger().warning.call_args_list
        tags = [c.args[0] for c in warning_calls if c.args]
        assert any("[issue 1882]" in t for t in tags), (
            f"нет диагностического warning с тегом [issue 1882]: {tags!r}"
        )

    def test_planning_when_babble_retry_already_used_still_muted(self):
        """Acceptance #2: регресс именно этого бага — при
        _babble_retry_used=True planning narration всё равно сжигается."""
        n = _make_node()
        n._babble_retry_used = True  # babble-ретрай уже потрачен
        result = _make_result(
            spoken=PLANNING_SPOKEN_2512,
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="сыграй рэп")

        published = _published_texts(n)
        assert published == [], (
            f"babble-retry_used=True, planning должна быть заглушена, "
            f"но опубликовано: {published!r}"
        )

    def test_planning_short_pattern_also_muted(self):
        """Короткий паттерн (не 2.5 КБ) — guard должен всё равно сработать."""
        n = _make_node()
        result = _make_result(
            spoken=(
                "Юзер снова просит рэп. "
                "Аргументы для executemusiccode готовы, bpm=140."
            ),
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="продолжай бит")

        published = _published_texts(n)
        assert published == [], (
            f"короткий planning тоже должен быть заглушен: {published!r}"
        )


class TestPlanningNarrationNegative:
    """Acceptance #3: guard НЕ должен сжигать легитимные ответы."""

    def test_normal_answer_with_speak_text_real_published(self):
        """Обычный ответ с реальным speak_text → озвучивается как раньше."""
        n = _make_node()
        # Имя тула упомянуто в ответе как часть справки, при этом
        # speak_text_real > 0 → guard НЕ должен сработать.
        result = _make_result(
            spoken=(
                "speak_text озвучивает произвольный текст голосом робота, "
                "а get_current_time возвращает текущее время."
            ),
            tools=["speak_text"],
            real_count=1,
        )
        n._handle_result(result, user_input="что умеешь?")

        # Issue #988: реальный speak_text → auto-TTS скипается.
        # Это нормально, просто проверим, что planning-mute не съел текст.
        published = _published_texts(n)
        # При speak_text_real>0 и tools содержит speak_text — auto-TTS скип.
        # Главное: НЕ было planning-mute (т.е. нет нашего warning).
        warning_calls = n.get_logger().warning.call_args_list
        tags = [c.args[0] for c in warning_calls if c.args]
        assert not any("[issue 1882]" in t for t in tags), (
            f"planning-mute сработал на легитимном ответе со speak_text: {tags!r}"
        )
        # sanity-check: тест не должен падать на отсутствии публикации —
        # тут проверяем только что mute НЕ сработал.
        assert published == []  # speak_text_real>0 подавляет auto-TTS (issue #988)

    def test_normal_factual_answer_published(self):
        """Самый обычный ответ → озвучивается."""
        n = _make_node()
        result = _make_result(
            spoken="Чёрное море находится на юге России.",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="где чёрное море?")

        published = _published_texts(n)
        assert published, "обычный ответ должен озвучиваться"
        assert "Чёрное море" in " | ".join(published)

    def test_normal_answer_mentioning_tool_name_only_published(self):
        """Ответ с упоминанием имени тула, без оператора/decision —
        не planning → озвучивается."""
        n = _make_node()
        result = _make_result(
            spoken=(
                "Я умею: speak_text, get_current_time, search_web. "
                "Спрашивай что хочешь!"
            ),
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="что умеешь?")

        published = _published_texts(n)
        assert published, (
            f"упоминание имён тулов без оператора не должно "
            f"триггерить mute: {published!r}"
        )

    def test_babble_phrase_without_planning_markers_routes_to_babble(self):
        """«Погнали, зачитаю рэп про космос!» — это babble, не planning.
        Babble-guard может отправить ретрай. Здесь главное, что planning
        guard НЕ молчит: текст не planning, и либо babble отправит
        ретрай, либо текст дойдёт до TTS — но НЕ через наш mute."""
        n = _make_node()
        result = _make_result(
            spoken="Погнали, зачитаю рэп про космос!",
            tools=[],
            real_count=0,
        )
        # Мокаем _check_babble_and_retry чтобы не зависеть от его логики:
        # planning-guard не должен вмешиваться, и если babble не
        # заретраил, текст дойдёт до TTS.
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
        n._check_unbacked_action_claim_and_retry = MagicMock(return_value=False)

        n._handle_result(result, user_input="зачитай рэп")

        # planning-guard НЕ должен сработать (нет оператора/decision).
        warning_calls = n.get_logger().warning.call_args_list
        tags = [c.args[0] for c in warning_calls if c.args]
        assert not any("[issue 1882]" in t for t in tags), (
            f"planning-mute сработал на чистой babble-фразе: {tags!r}"
        )


class TestPlanningNarrationGateSafety:
    """Гейт (speak_text_real == 0 AND tools_called пуст) обязателен,
    иначе guard сожжёт легитимный ответ."""

    def test_planning_text_with_tools_called_is_NOT_muted(self):
        """Если LLM вернула planning-text, НО вызвала speak_text —
        ответ валиден (speak_text_real>0), planning-mute НЕ срабатывает."""
        n = _make_node()
        # Реалистичный сценарий: LLM начала планировать, потом всё-таки
        # вызвала speak_text с реальным текстом. Финальный spoken всё ещё
        # содержит planning-маркеры, но speak_text_real>0.
        result = _make_result(
            spoken=(
                "Юзер попросил. Решение: скажу короткий ответ. "
                "Аргументы для speak_text: текст про погоду."
            ),
            tools=["speak_text"],
            real_count=1,  # <-- вот это и спасает от planning-mute
        )
        n._handle_result(result, user_input="расскажи про погоду")

        # Issue #988 + planning-mute: при speak_text_real>0 mute НЕ
        # срабатывает (гейт), babble-retry НЕ срабатывает (real_count>0),
        # issue-988 подавляет auto-TTS. Главное — НЕТ [issue 1882] warning.
        warning_calls = n.get_logger().warning.call_args_list
        tags = [c.args[0] for c in warning_calls if c.args]
        assert not any("[issue 1882]" in t for t in tags), (
            f"planning-mute сработал при speak_text_real>0 "
            f"(нарушение гейта): {tags!r}"
        )


if __name__ == "__main__":  # pragma: no cover
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
