"""
test_issue_2548_music_prose_action.py — Issue #2548 acceptance tests.

Live 15.09 (Vision Pi, TG → DJ-сет Пауля Оакенфольда): четыре из
восьми попыток «докрутить Григ» отвечали spoken-ом про работу с
музыкой при ``tools_called=[]``. Существующие Bug E правила
(``track_load`` и т.п.) ловили только узкие случаи вроде
«Трек играет.» / «Точка сохранена.» — а «Вплела тему Грига…» /
«Сделала два pass…» / «…перезапущу» проходили мимо. Юзер слышал
«всё готово» четыре раза подряд, пока на 8-й итерации не срабатывал
babble-retry (issue #992 Bug D).

Этот файл — integration-уровень: тестирует ``_handle_result`` на
реальном ``DialogueNode`` (через ``object.__new__`` + ручные моки,
как в test_issue_1882_planning_narration.py).

Acceptance criteria:
  1. tools=[] + spoken содержит «запустил/сделал/обновил» в DJ-сессии
     → ОДИН ретрай отправлен, текст НЕ публикуется в TTS;
  2. tools=['compose_music'] + spoken «запустил DJ» → НЕ ретрай;
  3. is_dj_auto=True (юзер молчал) → НЕ ретрай (музыка-пrose-action
     не должен сжигать DJ auto-transition);
  4. Если ретрай уже потрачен, а claim повторился — fallback
     «Не получилось изменить музыку — попробую ещё раз» БЕЗ claim
     о выполнении;
  5. В БЫТУ (dj=False + no music-kw) — guard молчит;
  6. Никакой ранее зелёный тест не сломан (test_issue_1882_… и пр.).
"""

from __future__ import annotations

from unittest.mock import MagicMock

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.dialogue_node import DialogueNode


# ---------------------------------------------------------------------------
# _make_node / _make_result — те же помощники, что в
# test_issue_1882_planning_narration.py (общие контракты моков).
# ---------------------------------------------------------------------------

def _make_node(*, dj_enabled: bool = False, dj_active_during_handle: bool | None = None) -> DialogueNode:
    """Минимальная DialogueNode для ``_handle_result`` через ``__new__``.

    ``dj_enabled`` — состояние DJ-сессии (``self._dj.state.enabled``),
    которое ``_handle_result`` читает в новой ветке issue #2548.
    По умолчанию False (быт/не-DJ); для acceptance #1 нужно True.
    """
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
    n._dj.state.enabled = dj_enabled
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
    # Issue #1881 — общий бюджет synthetic-ретраев; здесь
    # не нужен (action-claim правит своим one-shot флагом), но
    # если guard проверяет — нужно подменить метод.
    n._consume_synthetic_retry = MagicMock(return_value=True)
    return n


def _make_result(
    spoken: str,
    tools=None,
    real_count: int = 0,
    error: str | None = None,
) -> DialogResult:
    """Реальный DialogResult с полем speak_text_real_count."""
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools or []),
        speak_text_real_count=real_count,
        finish_reason="stop",
        error=error,
    )


def _published_texts(node: DialogueNode) -> list[str]:
    """Тексты, реально опубликованные в /voice/dialogue/response."""
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


# ---------------------------------------------------------------------------
# Live-кейсы из карточки (карточка issue #2548, дословно из лога 15.09)
# ---------------------------------------------------------------------------

LIVE_EXAMPLES_DJ_ACTIVE: list[tuple[str, str]] = [
    # (user_input, spoken) — DJ-сессия активна (TG → Vision Pi, DJ-сет).
    (
        "вплетай их красиво",
        "Вплела тему Грига как второй голос над пульсом. "
        "Бочкинса — должно звучать плотно и в кассовом ключе.",
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
]


class TestMusicProseActionRetryFires:
    """Acceptance #1: tools=[] + action-claim в DJ-сессии → ОДИН ретрай.

    Проверяем через мок ``_dispatch_turn``: при срабатывании guard'а
    _handle_result вызывает ``_dispatch_turn`` с CRITICAL-промптом.
    Это integration-уровень «retail-сигнал ретрая», как и для
    других guards в этой цепочке.
    """

    def test_dispatch_called_for_each_live_example(self) -> None:
        """Все 4 live-кейса → guard отправляет ОДИН retry."""
        for user_input, spoken in LIVE_EXAMPLES_DJ_ACTIVE:
            n = _make_node(dj_enabled=True)
            # ВАЖНО: перехватываем _dispatch_turn — это «хвост» guard'а,
            # который публикует CRITICAL-промпт в LLM. Если guard
            # не сработал — _dispatch_turn НЕ вызывается.
            n._dispatch_turn = MagicMock()
            # Аналогично: babble и renardo-guard молчат (это prose-
            # action claim, не babble-opener и не Renardo-код).
            n._check_babble_and_retry = MagicMock(return_value=False)
            n._check_embedded_renardo_code_and_retry = MagicMock(
                return_value=False
            )

            result = _make_result(spoken=spoken, tools=[], real_count=0)
            n._handle_result(result, user_input=user_input)

            assert n._dispatch_turn.called, (
                f"guard НЕ отправил ретрай для prose-action claim "
                f"(live-кейс из issue #2548): "
                f"user={user_input!r} spoken={spoken[:60]!r}"
            )
            # ВАЖНО: при retry ТЕКСТ НЕ публикуется в TTS — иначе
            # юзер услышит «всё готово» + потом ответ ретрая.
            assert _published_texts(n) == [], (
                f"guard отправил ретрай, но spoken всё равно "
                f"опубликован в TTS: {_published_texts(n)!r}"
            )

    def test_one_shot_flag_is_set_after_retry(self) -> None:
        """Acceptance #1.1: после ретрая флаг ``_action_claim_retry_used``
        становится True — иначе на втором круге guard выстрелит
        опять (ping-pong)."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        user_input, spoken = LIVE_EXAMPLES_DJ_ACTIVE[0]
        result = _make_result(spoken=spoken, tools=[], real_count=0)
        n._handle_result(result, user_input=user_input)

        assert n._action_claim_retry_used is True, (
            "one-shot флаг должен выставиться после ретрая, "
            "иначе guard будет пинг-понгом на каждом ходе"
        )

    def test_retry_dispatched_in_turn_flag(self) -> None:
        """Issue #992 / #1882: ``_retry_dispatched_in_turn=True``
        обязателен, иначе parent'ский finally закроет DSM до того,
        как ретрай успеет отработать (regression из #992 Bug C)."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        user_input, spoken = LIVE_EXAMPLES_DJ_ACTIVE[0]
        result = _make_result(spoken=spoken, tools=[], real_count=0)
        n._handle_result(result, user_input=user_input)

        assert n._retry_dispatched_in_turn is True


class TestMusicProseActionRetryDoesNotFire:
    """Acceptance #2 + #3: НЕ ретрай, если claim оправдан тул-коллом
    или is_dj_auto=True."""

    def test_tools_called_satisfies_claim_no_retry(self) -> None:
        """Acceptance #2: tools=['compose_music'] + spoken «запустил DJ»
        → guard молчит (action claim оправдан тулом)."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        result = _make_result(
            spoken="Запустил DJ — сейчас играет.",
            tools=["set_dj_mode"],
            real_count=0,
        )
        n._handle_result(result, user_input="включи диджей")

        assert not n._dispatch_turn.called, (
            "tools=['set_dj_mode'] оправдывает claim → ретрая быть "
            f"не должно, но _dispatch_turn called={n._dispatch_turn.called}"
        )

    def test_is_dj_auto_means_no_retry(self) -> None:
        """Acceptance #3: ``is_dj_auto=True`` (DJ auto-transition,
        юзер НЕ говорил) — guard НЕ должен ретраить. Этот кейс
        НЕ покрыт issue #2548 (юзер-то молчал), но guard
        срабатывал бы на claim «выставил бит» в DJ-тике — мы
        защищаемся от ложного срабатывания."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        # Юзер молчал, DJ-тикер сгенерировал переход, LLM ответил
        # claim-verb'ом «выставил бит» при tools=[]. Это НЕ
        # action-claim про юзера — guard должен молчать.
        result = _make_result(
            spoken="Выставил бит для перехода.",
            tools=[],
            real_count=0,
        )
        n._handle_result(
            result,
            user_input="",
            is_dj_auto=True,
        )

        assert not n._dispatch_turn.called, (
            "is_dj_auto=True → guard НЕ должен ретраить "
            "(юзер молчал), но _dispatch_turn called"
        )

    def test_dj_inactive_no_retry(self) -> None:
        """Acceptance #5: ``dj_enabled=False`` + prose-action claim в
        быту — guard молчит (нет DJ-контекста, нет music-kw в
        user_input)."""
        n = _make_node(dj_enabled=False)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        # Бытовая ситуация: «сделала уборку», нет DJ.
        result = _make_result(
            spoken="Сделала уборку и вымыла пол.",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="давай уберу квартиру")

        assert not n._dispatch_turn.called, (
            "бытовая реплика НЕ должна триггерить action-claim "
            "retry в не-DJ-сессии"
        )


class TestMusicProseActionFallbackSpoken:
    """Acceptance #2 (fallback после ретрая): если guard уже
    отстрелял, а claim повторился на новом ходе → публикуется
    «Не получилось изменить музыку — попробую ещё раз» БЕЗ
    claim о выполнении."""

    def test_fallback_after_retry_used(self) -> None:
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
        # Симулируем, что ретрай уже отстрелял в этой user-turn.
        n._action_claim_retry_used = True

        # Новый ход: та же самая ситуация — claim-verb, tools=[].
        user_input, spoken = LIVE_EXAMPLES_DJ_ACTIVE[0]
        result = _make_result(spoken=spoken, tools=[], real_count=0)
        n._handle_result(result, user_input=user_input)

        published = _published_texts(n)
        assert published, (
            "после ретрая + повтор claim'а должна опубликоваться "
            "fallback-фраза в TTS, но _response_pub пустой"
        )
        text = " | ".join(published)
        assert "Не получилось изменить музыку" in text, (
            f"fallback-фраза не соответствует контракту #2 "
            f"«не получилось изменить музыку — попробую ещё раз»: "
            f"got={text!r}"
        )
        # Главное: в fallback нет claim о выполнении.
        for forbidden in (
            "Вплела", "сделал", "обновил", "перезапустил",
            "поменял", "изменил",
        ):
            assert forbidden not in text, (
                f"fallback-фраза содержит claim '{forbidden}' — "
                f"должна быть констатацией без claim: {text!r}"
            )

    def test_no_fallback_when_retry_not_used_yet(self) -> None:
        """Если guard ещё НЕ стрелял (one-shot флаг False) — fallback
        НЕ публикуется: guard либо сам сработает (acceptance #1),
        либо молчит и текст идёт дальше по цепочке."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
        # _action_claim_retry_used = False (по умолчанию)

        # Случай: НЕ prose-action claim — claim_re НЕ ловит. Guard
        # молчит. Никакой fallback (он ждёт повтор claim'а после
        # уже потраченного ретрая). Текст идёт в обычный auto-TTS.
        result = _make_result(
            spoken="Понял, продолжаю.",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="включи музыку")

        # Fallback НЕ публикуется. Текст публикуется обычным путём
        # (это не наш concern — здесь проверяем только, что
        # fallback-модуль не вмешался без повторного claim'а).
        published = _published_texts(n)
        # Если бы fallback вмешался, было бы «Не получилось».
        assert "Не получилось" not in " | ".join(published), (
            f"fallback НЕ должен публиковаться, если guard ещё не "
            f"стрелял в этой turn: got={published!r}"
        )

    def test_no_fallback_when_dj_disabled(self) -> None:
        """Fallback требует DJ-контекста (acceptance #2 явно
        описывает «если после retry модель опять tools=[] —
        fallback-фраза» — это в DJ-сет-сценарии). В быту
        ``_action_claim_retry_used`` мог быть True от предыдущего
        turn'а (waypoint-save), но fallback всё равно НЕ
        публикуется — иначе «Не получилось» прозвучит на
        неподходящем контексте."""
        n = _make_node(dj_enabled=False)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
        n._action_claim_retry_used = True  # предыдущий turn waypoint-save

        result = _make_result(
            spoken="Сделала уборку и вымыла пол.",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="давай уберу квартиру")

        published = _published_texts(n)
        assert "Не получилось" not in " | ".join(published), (
            f"fallback в быту НЕ должен публиковаться: {published!r}"
        )


class TestRegressionSafety:
    """Sanity-check: новый guard НЕ сломал существующие пути."""

    def test_no_retry_for_normal_answer(self) -> None:
        """Обычный ответ (без claim-verb'ов) → ни ретрая, ни fallback."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        result = _make_result(
            spoken="Привет! Я готов помогать.",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="привет")

        assert not n._dispatch_turn.called
        published = _published_texts(n)
        assert "Привет" in " | ".join(published), (
            f"обычный ответ должен опубликоваться: {published!r}"
        )

    def test_no_retry_when_speak_text_already_spoke(self) -> None:
        """Если LLM уже говорил через speak_text_real>0 — auto-TTS
        подавлен (issue #988), guard НЕ должен вмешиваться."""
        n = _make_node(dj_enabled=True)
        n._dispatch_turn = MagicMock()
        n._check_babble_and_retry = MagicMock(return_value=False)
        n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)

        result = _make_result(
            spoken="Вот твоя песня.",
            tools=["speak_text"],
            real_count=1,
        )
        n._handle_result(result, user_input="спой")

        assert not n._dispatch_turn.called, (
            "speak_text_real>0 → auto-TTS подавлен, "
            "action-claim guard не должен срабатывать"
        )


if __name__ == "__main__":  # pragma: no cover
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
