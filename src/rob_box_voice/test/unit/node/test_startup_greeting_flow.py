"""
test_startup_greeting_flow.py — Unit-тесты inline-приветствия DialogueNode (issue #1003).

Покрывает новую трёхфазную схему (редизайн 06.08 + доработка 12.08):
  _on_startup_greeting → [thinking] → _on_startup_greeting_finish →
  [cute/very_cute] → _on_startup_greeting_speak → [случайная фраза].

Проверяем:
  - приветствие одноразовое (_startup_greeting_fired);
  - пропуск, если диалог активен (state != IDLE) — не перебиваем юзера;
  - звуки: thinking → pick_finish_sound (cute/very_cute);
  - фраза: случайная из GREETINGS, если startup_greeting_text пустой;
  - override: явный startup_greeting_text выигрывает.

Не требует ROS2 — rclpy замокан в conftest.py (test/unit/node).
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.dialogue_node import DialogueNode
from rob_box_voice.startup_greeting import GREETINGS, THINKING_SOUND
from rob_box_harness.core.dialogue_state_machine import DialogueStateKind


@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__."""
    n = object.__new__(DialogueNode)

    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    n._sound_trigger_pub = MagicMock()
    n._response_pub = MagicMock()
    n._startup_greeting_fired = False
    n._startup_greeting_text = ""
    n._greeting_timer = None

    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.IDLE

    # Таймеры: запоминаем созданные (period, callback), чтобы тест мог
    # вручную вызвать следующую фазу.
    n._created_timers = []  # list[tuple[float, Callable]]

    def _create_timer(period, callback):
        fake = MagicMock()
        fake.period = period
        fake.callback = callback
        fake.cancel = MagicMock()
        n._created_timers.append((period, callback))
        return fake

    n.create_timer = MagicMock(side_effect=_create_timer)
    return n


def _last_timer(node) -> tuple[float, object]:
    assert node._created_timers, "таймер не создан"
    return node._created_timers[-1]


def _published_sound(node) -> str:
    call = node._sound_trigger_pub.publish.call_args
    assert call is not None, "звук не публиковался"
    return call[0][0].data


def _published_response_payload(node) -> str:
    call = node._response_pub.publish.call_args
    assert call is not None, "фраза не публиковалась"
    return call[0][0].data


class TestStartupGreetingFlow:
    def test_greeting_fires_when_idle(self, node) -> None:
        node._on_startup_greeting()
        # thinking-звук сразу.
        assert _published_sound(node) == THINKING_SOUND
        # Запланирована вторая фаза через 2с.
        period, callback = _last_timer(node)
        assert period == 2.0
        assert callback == node._on_startup_greeting_finish

    def test_greeting_skips_when_dialogue_active(self, node) -> None:
        node._dsm.current_state = DialogueStateKind.DIALOGUE
        node._on_startup_greeting()
        # Ни звука, ни таймера — приветствие пропущено.
        node._sound_trigger_pub.publish.assert_not_called()
        assert node._created_timers == []
        # Но флаг всё равно выставлен — повторно не попробуем.
        assert node._startup_greeting_fired is True

    def test_greeting_one_shot(self, node) -> None:
        node._on_startup_greeting()
        node._on_startup_greeting()
        # Звук опубликован ровно один раз.
        assert node._sound_trigger_pub.publish.call_count == 1
        assert len(node._created_timers) == 1

    def test_finish_phase_plays_cute_sound(self, node) -> None:
        node._on_startup_greeting()
        _, finish_cb = _last_timer(node)
        finish_cb()
        # Радостный звук из FINISH_SOUNDS.
        from rob_box_voice.startup_greeting import FINISH_SOUNDS

        assert _published_sound(node) in FINISH_SOUNDS
        # Третья фаза запланирована через 1.5с.
        period, speak_cb = _last_timer(node)
        assert period == 1.5
        assert speak_cb == node._on_startup_greeting_speak

    def test_speak_phase_publishes_random_greeting(self, node) -> None:
        with patch(
            "rob_box_voice.startup_greeting.random.choice",
            return_value=GREETINGS[0],
        ):
            node._on_startup_greeting()
            _, finish_cb = _last_timer(node)
            finish_cb()
            _, speak_cb = _last_timer(node)
            speak_cb()
        payload = _published_response_payload(node)
        assert GREETINGS[0] in payload
        assert "<speak>" in payload

    def test_speak_phase_uses_override_text(self, node) -> None:
        node._startup_greeting_text = "Я на связи, все системы в норме!"
        node._on_startup_greeting()
        _, finish_cb = _last_timer(node)
        finish_cb()
        _, speak_cb = _last_timer(node)
        speak_cb()
        payload = _published_response_payload(node)
        assert "Я на связи, все системы в норме!" in payload
