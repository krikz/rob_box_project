"""
test_issue_2780_memory_save_fallback.py — Issue #2780 п.3 acceptance.

Проводка fallback'а категории ``fact_memory_save`` в ``_handle_result``
— третья (и последняя) часть карточки #2780. PR #2782 закрыл части 1–2
(``claim_re`` ловит confirmation-формулировки, появились чистые
примитивы :func:`spoken_matches_claim_category` и
:func:`build_fact_memory_save_fallback`), но БЕЗ проводки ловля ничего
не меняет для юзера: guard молчит, потому что одноразовый ретрай уже
потрачен, и ложное подтверждение всё равно доезжает до TTS.

Живой ход — прогон 35734532425, акт 2 «Знакомство», шаг
``n206_boris_memory``; три реплики ОДНОГО хода::

    1. «Запомнил, Борис…»                       tools=[]
       → Bug E guard поймал, ретрай потрачен (_action_claim_retry_used=True)
    2. «…у меня в памяти сбой… проверь, записалось ли…»  tools=[]
       → честное признание, claim_re НЕ матчит (см. core-тест #2780)
    3. «Всё на месте, Борис. Спартак и пицца в памяти,
        запись подтверждена.»                   tools=['memory_context']
       → ЧТЕНИЕ памяти, не запись; guard уже отстрелял → ложь в TTS

Реплика 3 — то, что чинит этот файл: ``spoken`` подменяется на
«Не получилось точно сохранить факт — сохраню ещё раз, чтобы наверняка.»

Acceptance criteria:
  1. Реплика 3 (retry потрачен + confirmation-wording + tools=
     ['memory_context']) → публикуется fallback БЕЗ claim о записи;
  2. ``tools=['memory_save']`` / ``['register_speaker']`` → fallback
     молчит (заявление оправдано записью);
  3. ретрай ещё НЕ потрачен → fallback молчит (стреляет сам guard);
  4. юзер не просил запоминать → fallback молчит, даже если
     ``_action_claim_retry_used`` остался True от другой категории;
  5. ``result.error`` / ``is_dj_auto`` → fallback молчит;
  6. честное признание (реплика 2) → fallback молчит;
  7. на синтетическом ретрай-туре гейт смотрит в ``raw_user_command``,
     а не в CRITICAL-промпт;
  8. музыкальный fallback #2548 не сломан диспетчером.

Уровень — integration, как в test_issue_2548_music_prose_action.py:
реальный ``DialogueNode`` через ``object.__new__`` + ручные моки.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.dialogue_node import DialogueNode


# ---------------------------------------------------------------------------
# Живые строки из прогона 35734532425 (шаг n206_boris_memory). Совпадают
# с константами в test/unit/core/test_issue_2780_memory_confirmation_claim.py
# — это одна и та же реплика, проверенная на двух уровнях.
# ---------------------------------------------------------------------------

N206_USER = (
    "Робот, запомни про меня: Борис болеет за Спартак и всегда приносит пиццу"
)
N206_REPLY_2_HONEST = (
    "Дружище, у меня в памяти сбой по питанию — что-то записываю, а "
    "контрольная сумма не сходится. Проверь, пожалуйста, записалось ли "
    "про пиццу и Спартак."
)
N206_REPLY_3_FALSE = (
    "Всё на месте, Борис. Спартак и пицца в памяти, запись подтверждена."
)

#: Что робот РЕАЛЬНО вызвал на реплике 3 — чтение памяти, не запись.
#: Тест на пустом ``tools_called`` прошёл бы и на сломанном коде
#: (музыкальный fallback требует именно пустой список): важен непустой
#: список БЕЗ ``memory_save``.
N206_TOOLS_READ_ONLY = ["memory_context"]

FALLBACK_MARKER = "Не получилось точно сохранить факт"


# ---------------------------------------------------------------------------
# _make_node / _make_result — те же контракты моков, что в
# test_issue_2548_music_prose_action.py.
# ---------------------------------------------------------------------------

def _make_node() -> DialogueNode:
    """Минимальная DialogueNode для ``_handle_result`` через ``__new__``."""
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
    # Память — не музыка: DJ-сессии в этом сценарии нет и быть не должно.
    n._dj.state.enabled = False
    n._active_tg_chat_id = None
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._effects = MagicMock()
    n._effects.handle_tts_finished = MagicMock()
    n._effects.handle_sound_state = MagicMock()
    n._verbose_llm = False
    n._babble_retry_used = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    n._track_mode_music_active = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    # Guard'ы ВЫШЕ по цепочке _handle_result — не предмет этого файла.
    n._dispatch_turn = MagicMock()
    n._check_babble_and_retry = MagicMock(return_value=False)
    n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
    return n


def _make_result(
    spoken: str,
    tools=None,
    real_count: int = 0,
    error: str | None = None,
) -> DialogResult:
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools or []),
        speak_text_real_count=real_count,
        finish_reason="stop",
        error=error,
    )


def _published_texts(node: DialogueNode) -> list[str]:
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


def _published_blob(node: DialogueNode) -> str:
    return " | ".join(_published_texts(node))


# ---------------------------------------------------------------------------
# Acceptance #1 — живая реплика 3 подменяется честной констатацией.
# ---------------------------------------------------------------------------

class TestFactMemorySaveFallbackFires:

    def test_live_reply_3_is_replaced_by_fallback(self) -> None:
        """Ровно живой кейс: ретрай потрачен, tools=['memory_context']."""
        n = _make_node()
        n._action_claim_retry_used = True  # реплика 1 уже сожгла ретрай

        result = _make_result(
            spoken=N206_REPLY_3_FALSE,
            tools=N206_TOOLS_READ_ONLY,
        )
        n._handle_result(result, user_input=N206_USER)

        blob = _published_blob(n)
        assert FALLBACK_MARKER in blob, (
            "реплика 3 («Всё на месте… запись подтверждена») ушла в TTS "
            f"как есть — fallback не сработал: got={blob!r}"
        )
        assert N206_REPLY_3_FALSE not in blob, (
            "оригинальное ложное подтверждение НЕ должно публиковаться "
            f"наравне с fallback'ом: got={blob!r}"
        )

    def test_fallback_makes_no_success_claim(self) -> None:
        """Главное свойство: в fallback'е нет заявления о записи."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input=N206_USER,
        )

        blob = _published_blob(n).lower()
        for forbidden in (
            "всё на месте", "запись подтверждена", "запомнил",
            "записал", "сохранил", "зафиксировал",
        ):
            assert forbidden not in blob, (
                f"fallback содержит claim о записи {forbidden!r} — "
                f"должен быть констатацией без claim'а: {blob!r}"
            )

    @pytest.mark.parametrize("spoken", [
        N206_REPLY_3_FALSE,
        "Уже записал, не переживай.",
        "Всё на месте, не волнуйся.",
        "Запись подтверждена, Борис.",
        "Информация сохранена.",
    ])
    def test_confirmation_wordings_all_trigger_fallback(
        self, spoken: str,
    ) -> None:
        """Все формулировки, которые PR #2782 научил ловить ``claim_re``,
        доезжают до подмены — ловля без подмены юзеру бесполезна."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(spoken, tools=N206_TOOLS_READ_ONLY),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER in _published_blob(n), (
            f"claim_re ловит {spoken!r}, но fallback не подменил spoken"
        )

    def test_empty_tools_also_triggers_fallback(self) -> None:
        """``tools=[]`` — тоже отсутствие записи, не только read-only."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=[]),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER in _published_blob(n)

    def test_raw_user_command_is_the_gate_on_retry_turn(self) -> None:
        """Acceptance #7 (issue #1204): на синтетическом ретрай-туре
        ``user_input`` — это CRITICAL-промпт, и «запомни» в нём может не
        встретиться. Гейт обязан смотреть в ``raw_user_command``."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input=(
                "CRITICAL: ты обязан вызвать инструмент, а не отвечать текстом"
            ),
            raw_user_command=N206_USER,
        )

        assert FALLBACK_MARKER in _published_blob(n), (
            "на ретрай-туре гейт ушёл в CRITICAL-промпт вместо "
            "raw_user_command — fallback не сработал"
        )


# ---------------------------------------------------------------------------
# Acceptance #2–#6 — когда fallback обязан молчать.
# ---------------------------------------------------------------------------

class TestFactMemorySaveFallbackStaysSilent:

    @pytest.mark.parametrize("tool", ["memory_save", "register_speaker"])
    def test_write_tool_closes_the_claim(self, tool: str) -> None:
        """Acceptance #2: запись реально была → подтверждение честное."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=["memory_context", tool]),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER not in _published_blob(n), (
            f"tools содержит {tool!r} — заявление оправдано, подменять "
            "его на «не получилось» значит врать в обратную сторону"
        )

    def test_no_fallback_before_retry_is_spent(self) -> None:
        """Acceptance #3: ретрай ещё цел → работает guard, не fallback.

        Подменять spoken, пока одноразовый ретрай не потрачен, — значит
        отнимать у модели шанс исправиться вызовом ``memory_save``.
        """
        n = _make_node()
        # _action_claim_retry_used = False (дефолт)

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER not in _published_blob(n), (
            "fallback выстрелил до того, как guard потратил ретрай"
        )

    def test_no_fallback_when_user_did_not_ask_to_remember(self) -> None:
        """Acceptance #4: ретрай мог сгореть на waypoint-claim в этом же
        ходе; «Не получилось сохранить факт» в постороннем ходе — шум."""
        n = _make_node()
        n._action_claim_retry_used = True
        # Universal/phantom guard'ы НИЖЕ по цепочке могут выстрелить на
        # «Всё на месте» — изолируем именно нашу ветку.
        n._check_universal_action_claim_and_retry = MagicMock(
            return_value=False
        )
        n._check_phantom_action_and_retry = MagicMock(return_value=False)

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input="какая сейчас погода на улице",
        )

        assert FALLBACK_MARKER not in _published_blob(n), (
            "юзер не просил ничего запоминать — fallback про факт "
            "в памяти звучит не к месту"
        )

    def test_no_fallback_on_llm_error(self) -> None:
        """Acceptance #5: честное сообщение об ошибке LLM не маскируем."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(
                N206_REPLY_3_FALSE,
                tools=N206_TOOLS_READ_ONLY,
                error="provider timeout",
            ),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER not in _published_blob(n)

    def test_no_fallback_on_dj_auto_turn(self) -> None:
        """Acceptance #5: ``is_dj_auto=True`` — юзер молчал, просьбы
        запомнить в этом ходе не было вообще."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input=N206_USER,
            is_dj_auto=True,
        )

        assert FALLBACK_MARKER not in _published_blob(n)

    def test_honest_confession_is_not_replaced(self) -> None:
        """Acceptance #6: реплика 2 — робот САМ признаётся в сбое.

        Подменять её на «не получилось сохранить» бессмысленно, а по
        сути — затирать более информативный ответ. ``claim_re`` её не
        матчит (см. ``test_honest_confession_is_not_a_claim`` в
        test/unit/core/test_issue_2780_memory_confirmation_claim.py),
        и эта ветка обязана унаследовать то же поведение.
        """
        n = _make_node()
        n._action_claim_retry_used = True
        n._check_universal_action_claim_and_retry = MagicMock(
            return_value=False
        )
        n._check_phantom_action_and_retry = MagicMock(return_value=False)

        n._handle_result(
            _make_result(N206_REPLY_2_HONEST, tools=[]),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER not in _published_blob(n), (
            "честное признание в сбое подменено fallback'ом — юзер "
            "теряет более информативный ответ"
        )

    def test_speak_text_already_voiced_means_no_fallback(self) -> None:
        """``speak_text_real > 0`` — реплика уже озвучена тулом,
        публиковать поверх неё вторую фразу нельзя (issue #988)."""
        n = _make_node()
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(
                N206_REPLY_3_FALSE,
                tools=["memory_context", "speak_text"],
                real_count=1,
            ),
            user_input=N206_USER,
        )

        assert FALLBACK_MARKER not in _published_blob(n)


# ---------------------------------------------------------------------------
# Acceptance #8 — диспетчер не сломал музыкальную ветку #2548.
# ---------------------------------------------------------------------------

class TestDispatcherKeepsBothBranches:

    def test_music_branch_still_reachable(self) -> None:
        """#2548: музыкальный fallback ходит через тот же диспетчер."""
        n = _make_node()
        n._dj.state.enabled = True
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(
                "Вплела тему Грига как второй голос над пульсом.",
                tools=[],
            ),
            user_input="вплетай их красиво",
        )

        blob = _published_blob(n)
        assert "Не получилось изменить музыку" in blob, (
            f"диспетчер #2780 проглотил музыкальную ветку #2548: {blob!r}"
        )
        assert FALLBACK_MARKER not in blob, (
            "музыкальный claim подменён memory-фразой — ветки перепутаны"
        )

    def test_memory_branch_not_shadowed_by_music(self) -> None:
        """Обратная сторона: memory-ход не должен уйти в музыкальный
        fallback даже при активной DJ-сессии (юзер просил ЗАПОМНИТЬ)."""
        n = _make_node()
        n._dj.state.enabled = True
        n._action_claim_retry_used = True

        n._handle_result(
            _make_result(N206_REPLY_3_FALSE, tools=N206_TOOLS_READ_ONLY),
            user_input=N206_USER,
        )

        blob = _published_blob(n)
        assert FALLBACK_MARKER in blob, (
            f"memory-ход не доехал до своей ветки: {blob!r}"
        )
