"""
test_dialogue_control.py — ADR-0054 §6.5 unit-тесты для sub
``/dialogue/control`` (String JSON) и pub ``/dialogue/control_ack``.

Покрывает:
  * happy path pause → FSM=SILENCED → ack {state:"paused", reason}
  * idempotent pause (повторный не сбрасывает since_ms — инвариант §2.5)
  * resume → FSM=IDLE → ack {state:"idle"}
  * resume без предшествующего pause → no-op + ack с текущим state
  * invalid JSON → WARNING, FSM не меняется, ack НЕ шлётся
  * unknown action → WARNING, FSM не меняется, ack НЕ шлётся
  * FSM удерживается в SILENCED без авто-resume (без TTL, инвариант 8)
  * публикация на ``/voice/dialogue/state`` вызывается ДО ack
    (инвариант 3: «ack синхронен с изменением FSM»)

Не требует ROS2 — rclpy замокан в conftest.py.
"""

import json
import time
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
)

from .test_dialogue_node import _make_node


def _msg(payload):
    """String-like ROS message wrapper для удобства."""
    m = MagicMock()
    m.data = json.dumps(payload) if isinstance(payload, (dict, list)) else payload
    return m


def _make_idle_node():
    """Узел с FSM=IDLE; ``on_event(SILENCE_COMMAND)`` переводит в SILENCED."""
    n = _make_node()
    n._dsm.current_state = DialogueStateKind.IDLE
    n._publish_state = MagicMock()

    def _on_event(event):
        if event == DialogueEvent.SILENCE_COMMAND:
            n._dsm.current_state = DialogueStateKind.SILENCED
        elif event == DialogueEvent.UNSILENCE:
            n._dsm.current_state = DialogueStateKind.IDLE
    n._dsm.on_event.side_effect = _on_event
    return n


class TestDialogueControl:
    """ADR-0054 — sub /dialogue/control + pub /dialogue/control_ack."""

    def _ack(self, n):
        """Последний опубликованный ack на /dialogue/control_ack."""
        assert n._dialogue_control_pub.publish.called, "ack не опубликован"
        last = n._dialogue_control_pub.publish.call_args_list[-1]
        return json.loads(last.args[0].data)

    def test_pause_moves_fsm_to_silenced_and_publishes_ack(self):
        n = _make_idle_node()

        n._on_dialogue_control(_msg({"action": "pause", "reason": "operator"}))

        # FSM перешёл в SILENCED через SILENCE_COMMAND
        n._dsm.on_event.assert_called_with(DialogueEvent.SILENCE_COMMAND)
        assert n._dsm.current_state == DialogueStateKind.SILENCED
        n._publish_state.assert_called_once()
        ack = self._ack(n)
        assert ack["state"] == "paused"
        assert ack["reason"] == "operator"
        assert ack["since_ms"] > 0
        assert "ts_s" in ack
        # Внутренний cache выставлен
        assert n._paused_at_ms is not None
        assert n._paused_at_ms == ack["since_ms"]
        assert n._pause_reason == "operator"

    def test_pause_is_idempotent_does_not_reset_since_ms(self):
        """ADR-0054 §2.5 — повторный pause НЕ обновляет since_ms
        (чтобы напоминание в супервизоре не сбивалось)."""
        n = _make_idle_node()
        # Первый pause переводит FSM в SILENCED через side_effect
        n._on_dialogue_control(_msg({"action": "pause", "reason": "first"}))
        first_since = n._paused_at_ms
        first_ack_since = self._ack(n)["since_ms"]
        assert first_ack_since == first_since
        assert n._dsm.current_state == DialogueStateKind.SILENCED
        # Сбрасываем моки, чтобы второй вызов отслеживался отдельно
        n._dsm.on_event.reset_mock()
        n._publish_state.reset_mock()
        time.sleep(0.002)  # гарантируем другой monotonic()
        n._on_dialogue_control(_msg({"action": "pause", "reason": "second"}))
        # FSM НЕ перешёл (no-op)
        n._dsm.on_event.assert_not_called()
        n._publish_state.assert_not_called()
        # since_ms НЕ изменился
        assert n._paused_at_ms == first_since
        # ack содержит тот же since_ms
        assert self._ack(n)["since_ms"] == first_ack_since

    def test_resume_from_silenced_moves_to_idle(self):
        n = _make_idle_node()
        # Сначала в pause, чтобы выставить since_ms
        n._on_dialogue_control(_msg({"action": "pause", "reason": "operator"}))
        # FSM в SILENCED через side_effect; pause-выставил since_ms
        assert n._dsm.current_state == DialogueStateKind.SILENCED
        n._dsm.on_event.reset_mock()
        n._publish_state.reset_mock()

        n._on_dialogue_control(_msg({"action": "resume"}))

        n._dsm.on_event.assert_called_with(DialogueEvent.UNSILENCE)
        assert n._dsm.current_state == DialogueStateKind.IDLE
        n._publish_state.assert_called_once()
        ack = self._ack(n)
        assert ack["state"] == "idle"
        assert ack["reason"] == ""  # reason очищен
        assert n._paused_at_ms is None
        assert n._pause_reason == ""

    def test_resume_when_not_silenced_is_noop(self):
        """ADR-0054 §2.5 — resume из IDLE/LISTENING/DIALOGUE = no-op + ack
        с текущим состоянием (защита от гонок)."""
        for st in (DialogueStateKind.IDLE, DialogueStateKind.LISTENING):
            n = _make_node()
            n._dsm.current_state = st
            n._publish_state = MagicMock()
            n._on_dialogue_control(_msg({"action": "resume"}))
            # FSM НЕ перешёл
            n._dsm.on_event.assert_not_called()
            n._publish_state.assert_not_called()
            # ack всё равно шлём — с текущим состоянием
            ack = self._ack(n)
            assert ack["state"] == st.name.lower()

    def test_invalid_json_does_not_change_fsm_and_does_not_publish_ack(self):
        """ADR-0054 §2.5 + решение t_d058dc6f: невалидный JSON → ack НЕ шлём."""
        n = _make_node()
        n._dsm.current_state = DialogueStateKind.IDLE
        n._publish_state = MagicMock()

        n._on_dialogue_control(_msg("not json"))

        n._dsm.on_event.assert_not_called()
        n._publish_state.assert_not_called()
        n._dialogue_control_pub.publish.assert_not_called()

    def test_unknown_action_does_not_change_fsm_and_does_not_publish_ack(self):
        n = _make_node()
        n._dsm.current_state = DialogueStateKind.IDLE
        n._publish_state = MagicMock()

        n._on_dialogue_control(_msg({"action": "snooze"}))

        n._dsm.on_event.assert_not_called()
        n._publish_state.assert_not_called()
        n._dialogue_control_pub.publish.assert_not_called()

    def test_pause_without_reason_uses_empty_string(self):
        n = _make_idle_node()

        n._on_dialogue_control(_msg({"action": "pause"}))  # no reason

        n._dsm.on_event.assert_called_with(DialogueEvent.SILENCE_COMMAND)
        ack = self._ack(n)
        assert ack["state"] == "paused"
        assert ack["reason"] == ""

    def test_publish_state_before_ack(self):
        """ADR-0054 инвариант 3: /voice/dialogue/state публикуется ДО ack."""
        n = _make_node()
        n._dsm.current_state = DialogueStateKind.IDLE

        # MagicMock порядок: _publish_state() должен быть вызван до _dialogue_control_pub.publish()
        order = []

        n._publish_state = MagicMock(side_effect=lambda: order.append("state"))
        original_publish = n._dialogue_control_pub.publish

        def track_ack(msg):
            order.append("ack")
            return original_publish(msg)
        n._dialogue_control_pub.publish = MagicMock(side_effect=track_ack)

        n._on_dialogue_control(_msg({"action": "pause", "reason": "x"}))

        assert order == ["state", "ack"], (
            f"expected state-then-ack, got {order}"
        )


class TestDialogueControlNoAutoResume:
    """ADR-0054 инвариант 8: без TTL — pause без resume держит FSM=SILENCED."""

    def test_fsm_stays_silenced_after_long_pause(self):
        """ADR-0054 инвариант 8: pause без resume держит FSM=SILENCED
        бесконечно (нет TTL). После второго pause FSM не должен двинуться."""
        n = _make_idle_node()
        # Первый pause — FSM → SILENCED через side_effect.
        n._on_dialogue_control(_msg({"action": "pause", "reason": "operator"}))
        assert n._dsm.current_state == DialogueStateKind.SILENCED
        since_at_pause = n._paused_at_ms
        assert since_at_pause is not None
        n._dsm.on_event.reset_mock()
        n._publish_state.reset_mock()
        # Второй pause — no-op.
        n._on_dialogue_control(_msg({"action": "pause", "reason": "again"}))
        n._dsm.on_event.assert_not_called()
        n._publish_state.assert_not_called()
        # since_ms не изменился.
        assert n._paused_at_ms == since_at_pause
