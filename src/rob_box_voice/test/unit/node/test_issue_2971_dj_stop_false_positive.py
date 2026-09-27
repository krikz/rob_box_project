"""Issue #2971 — «диджея» в хвосте длинного промпта = ложная стоп-команда.

Живой инцидент 24.09.2026 11:10 UTC («Paul Oakenfold»): юзер продиктовал
роботу 143-строчный системный промпт для DJ-персоны, заканчивавшийся
словами «...вставь как системный промт для робота-диджея». LLM
корректно вызвала ``set_dj_mode(enabled=true)`` + запустила трек, но
``_apply_music_guard`` в ТОМ ЖЕ ходе сканировал сырой ``user_input`` на
стоп-команды, нашёл голую подстроку «диджея» в ``MUSIC_STOP_OVERRIDES``
и вызвал ``_force_dj_off_for_stop_command`` — DJ выключился через 4с
после включения, трек застрял на #1 (``repeat=True``).

Двойной фикс:
1. ``MUSIC_STOP_OVERRIDES`` (``core/dialogue_guards.py``) больше не
   содержит голых подстрок «диджея»/«диджеить»/«диджей режим» — только
   ``MUSIC_STOP_COMMAND_RE`` (стоп-глагол + муз. существительное) решает
   такие случаи. См. ``test_dialogue_guards.py::TestIsMusicStopCommand``.
2. ``DialogueNode._apply_music_guard`` (``dialogue_node.py``) не гасит
   DJ по стоп-эвристике, если модель САМА вызвала ``set_dj_mode`` в этом
   же ходе (значит она уже явно приняла решение по DJ-флагу) —
   defense-in-depth для промптов, где стоп-подстрока может появиться
   вместе с легитимным запуском сета по другим (пока не найденным)
   причинам.

DialogueNode собирается через ``object.__new__`` — тот же паттерн, что
и ``test_issue_2897_stop_disables_dj.py``.
"""

from __future__ import annotations

import json
import logging
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode

_DJ_ON = json.dumps({"enabled": True, "persona": "ДиДжей Paul Oakenfold"})

_LIVE_PROMPT = (
    "Ты диджей PAUL OAKENFOLD и у нас сегодня вечеринка. Играй транс "
    "и общайся с публикой. Скопируй весь блок выше и вставь как "
    "системный промт для робота-диджея."
)


def _real_dj(on_stop: MagicMock) -> DJModeController:
    return DJModeController(
        hook=DJHook(
            dispatch=MagicMock(),
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
            on_stop=on_stop,
        ),
        logger=logging.getLogger("test_2971"),
    )


def _make_node() -> DialogueNode:
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._farewell = MagicMock()
    n._dj = _real_dj(n._farewell)
    n._dj_mode_pub = MagicMock()
    n._retry_dispatched_in_turn = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._discard_last_music_reply = MagicMock()
    n._speak_direct = MagicMock()
    n._mark_retry_dispatched = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._dispatch_dj_turn = MagicMock()
    n._dispatch_turn = MagicMock()
    n._build_music_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._build_dj_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._publish_music_cleanup = MagicMock()
    n._classify_music_user_input_kind = MagicMock(return_value="other")
    return n


class TestLiveDjPromptDoesNotForceStop:
    """Длинный DJ-промпт с «диджея» в хвосте — DJ включён моделью в этом
    же ходе (``set_dj_mode`` в ``tools_called``) — не должен выключаться."""

    def test_dj_stays_enabled_when_set_dj_mode_called_this_turn(self) -> None:
        n = _make_node()
        n._dj.handle_message(_DJ_ON)
        assert n._dj.state.enabled

        n._apply_music_guard(
            was_dj_auto=False,
            user_input=_LIVE_PROMPT,
            tools_called=("set_dj_mode", "compose_music", "speak_text"),
            spoken="Врубаю транс, полетели!",
        )

        assert n._dj.state.enabled is True

    def test_dj_off_not_published_when_set_dj_mode_called_this_turn(
        self,
    ) -> None:
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input=_LIVE_PROMPT,
            tools_called=("set_dj_mode", "compose_music", "speak_text"),
            spoken="Врубаю транс, полетели!",
        )

        n._dj_mode_pub.publish.assert_not_called()


class TestBareDjNounAloneDoesNotForceStop:
    """Даже без ``set_dj_mode`` в этом ходе — голая подстрока «диджея» без
    стоп-глагола больше не гасит DJ (fix #1 — ``MUSIC_STOP_OVERRIDES``)."""

    def test_dj_stays_enabled(self) -> None:
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="расскажи про диджея, который тут был",
            tools_called=(),
            spoken="Окей.",
        )

        assert n._dj.state.enabled is True


class TestRealStopStillDisablesDjWithoutSetDjMode:
    """Регресс #2897 — реальная стоп-команда БЕЗ ``set_dj_mode`` в этом
    ходе по-прежнему выключает DJ (модель забыла позвать тул)."""

    def test_dj_disabled(self) -> None:
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )

        assert n._dj.state.enabled is False


class TestStopCommandWinsEvenIfSetDjModeCalledFalse:
    """Стоп-команда + модель САМА вызвала ``set_dj_mode`` (скорее всего
    ``enabled=false``) в этом ходе — код доверяет явному решению модели
    и не дублирует выключение (не регрессия: результат тот же — DJ
    выключен, — просто без второго force-off поверх уже принятого
    решения модели)."""

    def test_dj_off_not_double_forced_but_model_can_still_disable_itself(
        self,
    ) -> None:
        n = _make_node()
        n._dj.handle_message(_DJ_ON)
        # Модель сама вызвала set_dj_mode(enabled=false) — эмулируем эту
        # часть эффекта явным reset, как это в реальности сделал бы
        # mcp_server через топик.
        n._dj.reset_silently()

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=("set_dj_mode", "stop_music"),
            spoken="Хорошо, стою.",
        )

        assert n._dj.state.enabled is False
