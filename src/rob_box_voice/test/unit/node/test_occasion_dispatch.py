"""test_occasion_dispatch.py — PR-A #2536 acceptance §6.8.

Тестовый адаптер: ``OccasionGate.may_speak`` → ``DialogueNode._dispatch_turn(
..., is_synthetic=True, occasion=Occasion(...))`` → ``_run_turn`` доходит
до LLM. В captured logs — ``started turn occasion=<kind>`` без wake_word.

NB: PR-A HE подключает живой повод к wake-gate — это PR-B…F. Здесь мы
только проверяем, что:

1. ``OccasionGate.may_speak(Occasion("meeting", source_camera="main_camera"))``
   → ALLOW (т.е. gate пропускает НЕ-stub meeting).
2. ``DialogueNode._dispatch_turn(..., occasion=Occasion("meeting", ...))``
   логирует ``started turn occasion=meeting`` и НЕ блокирует wake-gate
   (т.е. поведение occasion=None остаётся прежним).
3. ``OccasionGate.may_speak(Occasion("meeting", source_camera="unknown"))``
   → REFUSE (стаб-фильтр), и в этом случае dispatch вызывать НЕ нужно —
   gate должен сам отказать.

Не требует ROS2: rclpy замокан в ``test/unit/node/conftest.py`` (как в
``test_startup_greeting_flow.py``).
"""

from __future__ import annotations

import asyncio
import logging
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.occasion import (
    Occasion,
    OccasionGate,
    VerdictKind,
)
from rob_box_voice.dialogue_node import DialogueNode


def _fake_run_coroutine_threadsafe(coro, loop):
    """Stand-in: закрываем переданный coroutine (чтобы не было 'never awaited')
    и возвращаем MagicMock-future."""
    if asyncio.iscoroutine(coro):
        try:
            coro.close()
        except Exception:
            pass
    return MagicMock()


# ---------------------------------------------------------------------------
# Fixture: DialogueNode без __init__ (как в test_startup_greeting_flow.py)
# ---------------------------------------------------------------------------


@pytest.fixture
def node(monkeypatch):
    """Минимальная DialogueNode с моками для ``_dispatch_turn`` без ROS2."""
    n = object.__new__(DialogueNode)

    # Реальный logger (а не MagicMock) — иначе ``caplog`` не увидит логи.
    logger = logging.getLogger("rob_box_voice")
    n._logger = logger
    n.get_logger = lambda: logger

    # Прочие зависимости, чтобы ``_dispatch_turn`` не упал на ранних шагах.
    n._pending_music_cleanup = False
    n._publish_music_cleanup = MagicMock()
    n._session_started_at = None
    n._session_end_reason = None
    n._loop = MagicMock()

    # Подменяем глобальный asyncio.run_coroutine_threadsafe на fake —
    # нам важен только путь до и включая «started turn occasion=...» лог,
    # а не реальное выполнение coroutine.
    monkeypatch.setattr(
        "rob_box_voice.dialogue_node.asyncio.run_coroutine_threadsafe",
        _fake_run_coroutine_threadsafe,
    )
    return n


# ---------------------------------------------------------------------------
# Acceptance §6.8 — dispatch через тестовый адаптер
# ---------------------------------------------------------------------------


class TestMeetingOccasionDispatch:
    """``may_speak(meeting, main_camera)`` → ALLOW; ``_dispatch_turn(
    ..., occasion=meeting)`` логирует «started turn occasion=meeting»."""

    def test_gate_allows_meeting_with_real_camera(self) -> None:
        """Gate пропускает meeting без stub-камеры."""
        gate = OccasionGate(global_debounce_s=10.0)
        verdict = gate.may_speak(
            Occasion(
                kind="meeting",
                payload={
                    "event_type": "person",
                    "source_camera": "main_camera",
                },
            ),
            now=100.0,
        )
        assert verdict.kind == VerdictKind.ALLOW

    def test_gate_refuses_stub_meeting(self) -> None:
        """Gate REFUSE'ит meeting/person/unknown — этот тест-адаптер
        НЕ вызывает dispatch (gate сам отказал)."""
        gate = OccasionGate()
        verdict = gate.may_speak(
            Occasion(
                kind="meeting",
                payload={"event_type": "person", "source_camera": "unknown"},
            ),
            now=100.0,
        )
        assert verdict.kind == VerdictKind.REFUSE

    def test_dispatch_occasion_logs_started_turn(self, node, caplog) -> None:
        """При передаче ``occasion=Occasion('meeting', ...)`` в
        ``_dispatch_turn`` логируется ``started turn occasion=meeting``
        БЕЗ wake_word (т.к. PR-A не подключает повод к wake-gate)."""
        occasion = Occasion(
            kind="meeting",
            payload={"event_type": "person", "source_camera": "main_camera"},
        )
        # ``asyncio.run_coroutine_threadsafe`` уже подменён в fixture
        # (monkeypatch.setattr) — корутины закрываются без выполнения.
        with caplog.at_level(logging.INFO, logger="rob_box_voice"):
            node._dispatch_turn(
                user_input="Денис вошёл",
                is_synthetic=True,
                occasion=occasion,
            )

        # 1) Лог содержит «started turn occasion=meeting».
        occasion_logs = [
            r for r in caplog.records
            if "started turn occasion=meeting" in r.getMessage()
        ]
        assert len(occasion_logs) == 1, (
            f"ожидался 1 лог 'started turn occasion=meeting', "
            f"получено {len(occasion_logs)}: "
            f"{[r.getMessage() for r in caplog.records]}"
        )

        # 2) Лог НЕ содержит wake_word-маркеров (PR-A не подключает).
        wake_logs = [
            r for r in caplog.records
            if "wake_word" in r.getMessage().lower()
        ]
        assert wake_logs == [], (
            f"в PR-A повод НЕ должен подменять wake-gate, "
            f"но в логах есть wake-маркеры: "
            f"{[r.getMessage() for r in wake_logs]}"
        )

    def test_dispatch_without_occasion_does_not_log_started(self, node, caplog) -> None:
        """Без ``occasion`` — байт-в-байт прежнее поведение, лог
        ``started turn occasion=...`` НЕ появляется."""
        with caplog.at_level(logging.INFO, logger="rob_box_voice"):
            node._dispatch_turn(user_input="привет", is_synthetic=True)
        occasion_logs = [
            r for r in caplog.records
            if "started turn occasion" in r.getMessage()
        ]
        assert occasion_logs == [], (
            "без occasion kwarg НЕ должно быть 'started turn occasion' лога, "
            f"получено: {[r.getMessage() for r in occasion_logs]}"
        )


# ---------------------------------------------------------------------------
# Дополнительно: тест «gate → dispatch → log» сквозной интеграционный путь.
# ---------------------------------------------------------------------------


class TestGateDrivesDispatch:
    """Сквозной сценарий: gate решает ALLOW → dispatch с occasion."""

    def test_meeting_occasion_runs_turn(self, node) -> None:
        """gate.may_speak → ALLOW → node._dispatch_turn с occasion."""
        gate = OccasionGate(global_debounce_s=10.0)
        occasion = Occasion(
            kind="meeting",
            payload={"event_type": "person", "source_camera": "main_camera"},
        )

        # Step 1: gate решает.
        verdict = gate.may_speak(occasion, now=100.0)
        assert verdict.kind == VerdictKind.ALLOW

        # Step 2: dispatch с тем же occasion (без wake_word — PR-A не подключает).
        # (asyncio.run_coroutine_threadsafe уже подменён в fixture.)
        node._dispatch_turn(
            user_input="Денис вошёл в мастерскую",
            is_synthetic=True,
            occasion=occasion,
        )

        # Step 3: gate.mark_consumed обновляет last_fire_at (это делает
        # production-код после успешного LLM-ответа; здесь — сразу).
        gate.mark_consumed(occasion, now=100.0)

        # Step 4: gate.mark_consumed прописал bookkeeping (last_fire_at +
        # last_any_at). EventDetector-зависимость убрана (#2612) — проверяем
        # оба счётчика через stats().
        stats = gate.stats()
        assert stats["last_fire_at"]["meeting"] == 100.0
        assert stats["last_any_at"] == 100.0