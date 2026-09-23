"""
test_issue_2856_dj_set_time_limit.py — сет без плана заканчивается по
ВРЕМЕНИ, а не через 24 перехода (issue #2856).

Живой прогон 23.09.2026: ``set_dj_mode`` без ``plan`` в 14:50, переход #17
в 15:39 — конца не видно. Лимит без плана был только по числу переходов
(``DJ_AUTO_MAX_TRANSITIONS = 24``, откалиброван под 45 с), а с #2461 переход
ждёт конца формы — реальный интервал 150-200 с, 24 перехода ≈ час с лишним.

Тесты гоняют :class:`DJModeController` на поддельных часах (``clock=``):
тик каждые 5 с, после каждого перехода форма играет 180 с
(``form_ends_at``), модель финал НЕ выключает сама — проверяем страховку.
"""
from __future__ import annotations

import json
import logging
import sys
import unittest
from pathlib import Path

# Make the in-tree package importable without `pip install -e`.
ROOT = Path(__file__).resolve().parents[4]  # .../test/unit/core/
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import DJModeController  # noqa: E402

T0 = 1_800_000_000.0
FORM_S = 180.0
FINAL_MARK = "ФИНАЛЬНЫЙ ТРЕК"


class _Clock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _StubHook:
    def __init__(self, clock: _Clock) -> None:
        self.persona_default = "Роббокс"
        self._clock = clock
        self.dispatches: list = []  # (time, prompt)
        self.stops: list = []  # time of on_stop
        self.on_stop = lambda persona: self.stops.append(clock.now)

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:  # noqa: ARG002
        self.dispatches.append((self._clock.now, prompt))

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _run_set(
    payload: dict,
    *,
    horizon_s: float = 3 * 3600.0,
    failed_transitions: frozenset = frozenset(),
):
    """Прогнать сет на поддельных часах до остановки DJ (или горизонта).

    Issue #2875 — каждый переход сообщает контроллеру, что трек запущен
    (так делает нода по ``MUSIC_STARTING_TOOLS``), кроме переходов из
    ``failed_transitions``: там модель музыку не запустила.
    """
    clock = _Clock(T0)
    hook = _StubHook(clock)
    ctrl = DJModeController(
        hook=hook, logger=logging.getLogger("test"), clock=clock,
    )
    ctrl.handle_message(json.dumps({"enabled": True, **payload}))
    seen = 0
    while ctrl.state.enabled and clock.now < T0 + horizon_s:
        clock.now += DJModeController.DJ_TICK_INTERVAL_S
        ctrl.tick()
        if len(hook.dispatches) > seen:
            seen = len(hook.dispatches)
            if seen in failed_transitions:
                continue
            ctrl.note_turn_tools(
                ["compose_music"], {"compose_music"}, is_dj_auto=True
            )
            # compose_music сыграл форму на 180 с — mcp_server публикует
            # её конец, переход ждёт его (#2461).
            ctrl.state.form_ends_at = clock.now + FORM_S
    return ctrl, hook


class TestPlanlessSetEndsByTime(unittest.TestCase):
    def test_no_plan_180s_intervals_ends_within_default_limit(self) -> None:
        ctrl, hook = _run_set({"next_transition_sec": 45})

        self.assertFalse(ctrl.state.enabled, "сет без плана должен закончиться сам")
        self.assertEqual(len(hook.stops), 1)
        limit = DJModeController.DJ_SET_DEFAULT_MAX_S
        self.assertLessEqual(
            hook.stops[0] - T0, limit,
            f"DJ выключился через {hook.stops[0] - T0:.0f}с, лимит {limit:.0f}с",
        )
        # И это лимит по времени, а не по страховочному счётчику переходов.
        self.assertLess(
            len(hook.dispatches), DJModeController.DJ_AUTO_MAX_TRANSITIONS,
        )
        # Сет не обрезан на старте: он занял большую часть лимита.
        self.assertGreater(hook.stops[0] - T0, limit - 2 * FORM_S)

    def test_final_track_prompt_emitted_once_right_before_stop(self) -> None:
        ctrl, hook = _run_set({"next_transition_sec": 45})

        finals = [i for i, (_, p) in enumerate(hook.dispatches) if FINAL_MARK in p]
        self.assertEqual(len(finals), 1, "финальный трек объявляется ровно один раз")
        self.assertEqual(
            finals[0], len(hook.dispatches) - 1,
            "финальный трек — последний переход перед остановкой",
        )
        final_at = hook.dispatches[finals[0]][0]
        self.assertLessEqual(
            final_at - T0, DJModeController.DJ_SET_DEFAULT_MAX_S,
            "финальный трек стартует до лимита",
        )
        # Остановка — на следующем переходе после финала, не позже.
        self.assertGreaterEqual(hook.stops[0], final_at + FORM_S)
        self.assertLess(
            hook.stops[0],
            final_at + FORM_S + 2 * DJModeController.DJ_TICK_INTERVAL_S,
        )

    def test_max_minutes_from_user_is_honoured(self) -> None:
        ctrl, hook = _run_set({"next_transition_sec": 45, "max_minutes": 10})

        self.assertFalse(ctrl.state.enabled)
        self.assertLessEqual(hook.stops[0] - T0, 10 * 60.0)
        self.assertGreater(hook.stops[0] - T0, 10 * 60.0 - 2 * FORM_S)
        self.assertEqual(
            sum(FINAL_MARK in p for _, p in hook.dispatches), 1,
        )

    def test_longer_max_minutes_beats_default(self) -> None:
        ctrl, hook = _run_set({"next_transition_sec": 45, "max_minutes": 40})

        self.assertFalse(ctrl.state.enabled)
        elapsed = hook.stops[0] - T0
        self.assertGreater(elapsed, DJModeController.DJ_SET_DEFAULT_MAX_S)
        self.assertLessEqual(elapsed, 40 * 60.0)

    def test_max_tracks_from_user_is_honoured(self) -> None:
        ctrl, hook = _run_set({"next_transition_sec": 45, "max_tracks": 4})

        self.assertFalse(ctrl.state.enabled)
        self.assertEqual(len(hook.dispatches), 4)
        self.assertIn(FINAL_MARK, hook.dispatches[-1][1])
        self.assertTrue(all(FINAL_MARK not in p for _, p in hook.dispatches[:-1]))

    def test_repeated_max_minutes_does_not_push_deadline(self) -> None:
        """Модель повторяет аргументы на каждом переходе — отсчёт от старта."""
        clock = _Clock(T0)
        hook = _StubHook(clock)
        ctrl = DJModeController(
            hook=hook, logger=logging.getLogger("test"), clock=clock,
        )
        ctrl.handle_message(json.dumps({"enabled": True, "max_minutes": 10}))
        clock.now += 500
        ctrl.handle_message(json.dumps({
            "enabled": True, "max_minutes": 10, "next_transition_sec": 60,
        }))
        self.assertEqual(ctrl.state.started_at, T0)
        self.assertEqual(ctrl.state.max_seconds, 600.0)

    def test_garbage_limits_are_ignored(self) -> None:
        clock = _Clock(T0)
        ctrl = DJModeController(
            hook=_StubHook(clock), logger=logging.getLogger("test"), clock=clock,
        )
        ctrl.handle_message(json.dumps({
            "enabled": True, "max_minutes": "много", "max_tracks": -3,
        }))
        self.assertIsNone(ctrl.state.max_seconds)
        self.assertEqual(ctrl.state.max_tracks, 0)

    def test_limits_do_not_leak_into_next_set(self) -> None:
        clock = _Clock(T0)
        ctrl = DJModeController(
            hook=_StubHook(clock), logger=logging.getLogger("test"), clock=clock,
        )
        ctrl.handle_message(json.dumps({"enabled": True, "max_minutes": 5}))
        ctrl.handle_message(json.dumps({"enabled": False}))
        clock.now += 1000
        ctrl.handle_message(json.dumps({"enabled": True}))
        self.assertIsNone(ctrl.state.max_seconds)
        self.assertEqual(ctrl.state.started_at, T0 + 1000)
        self.assertFalse(ctrl.state.final_dispatched)


class TestPlanPathUnchanged(unittest.TestCase):
    # Issue #2875: 8 треков по 180 с > 20 мин дефолтного лимита. Было 6 —
    # сет «переживал» лимит только за счёт трёх повторов финала до
    # страховки plan+3; теперь DJ выключается сразу после финала плана.
    PLAN = "\n".join(f"Трек {i}: номер {i}" for i in range(1, 9))

    def test_plan_set_runs_past_default_time_limit(self) -> None:
        """План из 8 треков по 180 с — финал по плану, не по дефолтному лимиту."""
        ctrl, hook = _run_set({"next_transition_sec": 45, "plan": self.PLAN})

        self.assertFalse(ctrl.state.enabled)
        prompts = [p for _, p in hook.dispatches]
        # Финал на треке #8; после реально сыгранного финального трека
        # плана DJ выключается на следующем тике (issue #2875).
        self.assertEqual(len(prompts), 8)
        self.assertTrue(all(FINAL_MARK not in p for p in prompts[:7]))
        self.assertIn("переход #8 — ФИНАЛЬНЫЙ ТРЕК", prompts[7])
        self.assertGreater(
            hook.stops[0] - T0, DJModeController.DJ_SET_DEFAULT_MAX_S,
            "дефолтный лимит времени не должен резать сет с планом",
        )


if __name__ == "__main__":
    unittest.main()
