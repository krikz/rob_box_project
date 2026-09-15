"""Unit tests for ADR-0101 PR-B: _on_startup_greeting_finish gating via OccasionGate.

После введения Повода (issue #2536, PR-A) — миграция
``_on_startup_greeting_finish`` на ``OccasionGate.may_speak`` (PR-B).
Старый флаг ``_startup_greeting_fired`` остаётся как fallback.

Acceptance (PR-B §Тесты):
  - test_startup_greeting_fires_once: после первого ALLOW + mark_consumed
    второй вызов → DEFER.
  - test_startup_greeting_disabled_in_pr_b_falls_back_to_old_flag:
    monkey-patch ``self._occasion.may_speak`` → DEFER, проверить что
    СТАРЫЙ флаг всё равно держит startup после первой попытки
    (не двойной sfx).

Совместимость с test_startup_greeting_flow.py:
  - все 6 существующих тестов продолжают работать (старый флаг
    остаётся, fallback логика не меняет поведение для случая
    «нет _occasion» / «may_speak всегда ALLOW»).

Не требует ROS2 — rclpy замокан в conftest.py (test/unit/node).
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.core.occasion import Occasion, VerdictKind
from rob_box_voice.dialogue_node import DialogueNode
from rob_box_voice.startup_greeting import GREETINGS, THINKING_SOUND


# ─────────────────────────────────────────────────────────────────────
# Fixture (минимальный DialogueNode без __init__).
# ─────────────────────────────────────────────────────────────────────


@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__ — повторяет test_startup_greeting_flow."""
    n = object.__new__(DialogueNode)

    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    n._sound_trigger_pub = MagicMock()
    n._response_pub = MagicMock()
    n._startup_greeting_fired = False
    n._startup_greeting_text = ""
    n._greeting_timer = None
    n._active_tg_chat_id = None

    n._dsm = MagicMock()
    n._dsm.current_state = "IDLE"  # DialogueStateKind.IDLE — строкой для простоты.

    n._created_timers: list[tuple[float, object]] = []

    def _create_timer(period, callback):
        fake = MagicMock()
        fake.period = period
        fake.callback = callback
        fake.cancel = MagicMock()
        n._created_timers.append((period, callback))
        return fake

    n.create_timer = MagicMock(side_effect=_create_timer)

    # ADR-0101 PR-B: OccasionGate.
    from rob_box_voice.core.occasion import OccasionGate
    n._occasion = OccasionGate(global_debounce_s=0.0)

    return n


# ─────────────────────────────────────────────────────────────────────
# Helper.
# ─────────────────────────────────────────────────────────────────────


def _last_timer(node) -> tuple[float, object]:
    assert node._created_timers, "таймер не создан"
    return node._created_timers[-1]


def _published_sound(node) -> str:
    call = node._sound_trigger_pub.publish.call_args
    assert call is not None, "звук не публиковался"
    return call[0][0].data


# ─────────────────────────────────────────────────────────────────────
# §PR-B Acceptance: один раз за uptime через OccasionGate.
# ─────────────────────────────────────────────────────────────────────


def test_startup_greeting_fires_once(node) -> None:
    """После первого ALLOW + mark_consumed второй вызов → DEFER.

    Повторный вызов _on_startup_greeting_finish не должен публиковать
    sfx повторно (один раз за uptime).
    """
    # Первая попытка: ALLOW.
    node._on_startup_greeting_finish()

    # Публикация finish-sound состоялась.
    from rob_box_voice.startup_greeting import FINISH_SOUNDS
    assert _published_sound(node) in FINISH_SOUNDS

    # Внутреннее состояние OccasionGate: startup помечен consumed.
    stats = node._occasion.stats()
    assert "startup" in stats["consumed_one_shot"], (
        f"ожидаем consumed_one_shot содержит 'startup', got {stats}"
    )

    # Сбросить mock, чтобы видеть только эффекты второй попытки.
    node._sound_trigger_pub.publish.reset_mock()
    node._created_timers.clear()

    # Вторая попытка: DEFER (one-shot already consumed).
    node._on_startup_greeting_finish()

    # sfx НЕ опубликован.
    node._sound_trigger_pub.publish.assert_not_called()
    # Таймер НЕ создан.
    assert node._created_timers == []

    # Лог содержит «deferred».
    info_calls = [
        c.args[0]
        for c in node._logger.info.call_args_list
        if "startup greeting deferred" in (c.args[0] if c.args else "")
    ]
    assert info_calls, "ожидаем лог «startup greeting deferred»"
    assert "one-shot already consumed" in info_calls[0]


def test_startup_greeting_disabled_in_pr_b_falls_back_to_old_flag(node) -> None:
    """Если OccasionGate DEFER'ит — СТАРЫЙ флаг всё равно держит startup.

    Это требование двойной защиты (PR-B): новый шов не должен ломать
    гарантию «ровно один раз за uptime» даже при сбое Повода.
    Monkey-patch may_speak → DEFER (имитация сбоя). Старый флаг
    _startup_greeting_fired должен оставаться True после первой
    попытки.
    """
    # Сэмулировать «Повод отказал» через monkey-patch.
    node._occasion.may_speak = MagicMock(
        return_value=MagicMock(kind=VerdictKind.DEFER, reason="forced defer", retry_after_s=None)
    )
    # Стартовый сценарий: фаза 1 уже прошла — флаг установлен, идём в фазу 2.
    node._startup_greeting_fired = True
    node._sound_trigger_pub.publish.reset_mock()

    # Фаза 2: должна DEFER'нуть и НЕ публиковать finish-sound.
    node._on_startup_greeting_finish()

    # sfx не публиковался.
    node._sound_trigger_pub.publish.assert_not_called()
    # Никаких таймеров.
    assert node._created_timers == []
    # Старый флаг ВСЁ ЕЩЁ True — двойная защита.
    assert node._startup_greeting_fired is True, (
        "старый флаг не должен сбрасываться при DEFER от Повода"
    )


def test_startup_greeting_refused_in_pr_b_no_sfx(node) -> None:
    """REFUSE от Повода → тоже return + лог."""
    node._occasion.may_speak = MagicMock(
        return_value=MagicMock(
            kind=VerdictKind.REFUSE,
            reason="stub event",
            retry_after_s=None,
        )
    )
    node._startup_greeting_fired = True
    node._sound_trigger_pub.publish.reset_mock()

    node._on_startup_greeting_finish()

    node._sound_trigger_pub.publish.assert_not_called()
    assert node._created_timers == []
    assert node._startup_greeting_fired is True


def test_startup_greeting_default_text_in_payload(node) -> None:
    """payload Occasion содержит _startup_greeting_text (override или пустую)."""
    node._startup_greeting_text = "Я на связи, все системы в норме!"

    # Перехватить may_speak.
    captured: list[Occasion] = []
    real_may_speak = node._occasion.may_speak

    def capture(occ: Occasion, now=None):
        captured.append(occ)
        return real_may_speak(occ, now=now)

    node._occasion.may_speak = capture
    node._on_startup_greeting_finish()

    assert len(captured) == 1, f"ожидаем 1 вызов may_speak, got {len(captured)}"
    occ = captured[0]
    assert occ.kind == "startup"
    assert occ.is_user_initiated is False
    assert occ.payload.get("text") == "Я на связи, все системы в норме!"


def test_startup_greeting_full_flow_still_works(node) -> None:
    """Полный happy path (как test_greeting_fires_when_idle, но с _occasion)."""
    # Фаза 1: thinking-sound + 2с таймер.
    # Имитируем DialogueStateKind.IDLE через строковый dsm.
    from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
    node._dsm.current_state = DialogueStateKind.IDLE

    # Вызываем _on_startup_greeting — старая логика (флаг → sfx → таймер).
    node._on_startup_greeting()
    assert _published_sound(node) == THINKING_SOUND
    period, callback = _last_timer(node)
    assert period == 2.0

    # Стартовое состояние: finish-sound не публиковался.
    node._sound_trigger_pub.publish.reset_mock()
    node._created_timers.clear()

    # Фаза 2: ALLOW → finish-sound + 1.5с таймер.
    callback()
    from rob_box_voice.startup_greeting import FINISH_SOUNDS
    assert _published_sound(node) in FINISH_SOUNDS
    period2, callback2 = _last_timer(node)
    assert period2 == 1.5

    # Стартовое состояние: response ещё не публиковался.
    node._sound_trigger_pub.publish.reset_mock()
    node._response_pub.publish.reset_mock()
    node._created_timers.clear()

    # Фаза 3: speak.
    with patch(
        "rob_box_voice.startup_greeting.random.choice",
        return_value=GREETINGS[0],
    ):
        callback2()

    # _publish_response вызван с фразой.
    response_call = node._response_pub.publish.call_args
    assert response_call is not None
    assert GREETINGS[0] in response_call[0][0].data