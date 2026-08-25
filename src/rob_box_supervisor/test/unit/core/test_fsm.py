"""Unit tests for ModeManager (AV-3, ADR-0028 §4.1).

Pure logic — no ROS, no asyncio, no threading. The FSM owns its
view of who currently holds which floor (so it can refuse illegal
transitions like ``off → avatar_present`` while voice_floor is held by
Telegram). That is a separate concern from AV-4 LockManager: the FSM
needs enough state to decide, but it is not a lock manager and it
does not implement heartbeat/dead-man — those are LockManager's job.

Acceptance criteria (issue #1597, AV-3):
  1. off → telegram_active (telegram_acquire_floor)
  2. off → avatar_present  (quest_acquire_floor)
  3. telegram_active → mixed (quest_acquire_floor teleop only)
  4. avatar_present → mixed (telegram_acquire_voice_floor)
  5. mixed → off          (both_release)
  6. telegram_active → off (telegram_release with timeout 30s, fake clock)
  7. Conflict: off → avatar_present while voice_floor is held → ConflictError
  8. * → off is always allowed (escape hatch)
  9. Idempotency: repeat acquire from same client — no-op
 10. Unknown event → ValueError
"""

import unittest

from rob_box_supervisor.core.fsm import (
    EVENT_BOTH_RELEASE,
    EVENT_QUEST_ACQUIRE_FLOOR,
    EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
    EVENT_QUEST_RELEASE,
    EVENT_TELEGRAM_ACQUIRE_FLOOR,
    EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
    EVENT_TELEGRAM_RELEASE,
    ClientId,
    ConflictError,
    IDLE_TIMEOUT_S,
    Mode,
    ModeManager,
)


# Удобные мнемоники, чтобы тесты читались.
CLIENT_TELEGRAM: ClientId = "telegram"
CLIENT_QUEST: ClientId = "quest"


class TestModeManagerBasicAcquires(unittest.TestCase):
    """Cases 1–2: off → telegram_active / off → avatar_present."""

    def test_off_to_telegram_active_on_telegram_acquire_floor(self):
        fsm = ModeManager()
        self.assertEqual(fsm.mode, Mode.OFF)
        new_mode = fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        self.assertEqual(new_mode, Mode.TELEGRAM_ACTIVE)
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)

    def test_off_to_avatar_present_on_quest_acquire_floor(self):
        fsm = ModeManager()
        new_mode = fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        self.assertEqual(new_mode, Mode.AVATAR_PRESENT)
        self.assertEqual(fsm.mode, Mode.AVATAR_PRESENT)


class TestModeManagerMixedTransitions(unittest.TestCase):
    """Cases 3–4: → mixed (both clients active, different floors)."""

    def test_telegram_active_to_mixed_on_quest_acquire_teleop_only(self):
        fsm = ModeManager()
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        new_mode = fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id=CLIENT_QUEST)
        self.assertEqual(new_mode, Mode.MIXED)
        self.assertEqual(fsm.mode, Mode.MIXED)

    def test_avatar_present_to_mixed_on_telegram_acquire_voice_floor(self):
        fsm = ModeManager()
        fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        new_mode = fsm.transition(EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR, client_id=CLIENT_TELEGRAM)
        self.assertEqual(new_mode, Mode.MIXED)
        self.assertEqual(fsm.mode, Mode.MIXED)


class TestModeManagerBothRelease(unittest.TestCase):
    """Case 5: mixed → off on both_release."""

    def test_mixed_to_off_on_both_release(self):
        fsm = ModeManager()
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id=CLIENT_QUEST)
        self.assertEqual(fsm.mode, Mode.MIXED)
        new_mode = fsm.transition(EVENT_BOTH_RELEASE)
        self.assertEqual(new_mode, Mode.OFF)
        self.assertEqual(fsm.mode, Mode.OFF)


class TestModeManagerIdleTimeout(unittest.TestCase):
    """Case 6: telegram_active → off after IDLE_TIMEOUT_S of inactivity.

    ADR-0028 §4.1: «telegram_active → off : telegram_release (timeout 30s no
    activity)». То есть FSM сам уходит в ``off``, если с последней
    активности прошло > 30 с. Проверяется через ``tick()`` — отдельный
    метод «проверь timeout и верни mode» (без побочного acquire/release).
    """

    def test_telegram_active_to_off_after_idle_timeout(self):
        clock = [0]

        def advance(seconds: float) -> None:
            clock[0] += int(seconds * 1000)

        fsm = ModeManager(clock=lambda: clock[0])
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)

        # Чуть меньше таймаута — ещё telegram_active.
        advance(IDLE_TIMEOUT_S - 0.001)
        self.assertEqual(fsm.tick(), Mode.TELEGRAM_ACTIVE)

        # После таймаута бездействия — tick() видит просрочку и
        # возвращает off, без необходимости явного release.
        advance(IDLE_TIMEOUT_S)
        self.assertEqual(fsm.tick(), Mode.OFF)
        self.assertEqual(fsm.mode, Mode.OFF)

        # mode в mixed/idle timeout не срабатывает (оба клиента активны).
        fsm2 = ModeManager(clock=lambda: clock[0])
        # Перемотаем clock в нёс для свежего FSM.
        clock[0] = 0
        fsm2.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        fsm2.transition(EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id=CLIENT_QUEST)
        advance(IDLE_TIMEOUT_S + 1)
        self.assertEqual(fsm2.tick(), Mode.MIXED)


class TestModeManagerConflict(unittest.TestCase):
    """Case 7: Conflict when target floor is held by another client."""

    def test_off_to_avatar_present_blocked_when_voice_floor_held(self):
        fsm = ModeManager()
        # Telegram зашёл и держит voice_floor.
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)

        # Quest пытается взять «полный» floor, пока Telegram держит voice —
        # FSM должен отклонить с ConflictError.
        with self.assertRaises(ConflictError) as ctx:
            fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)
        self.assertEqual(ctx.exception.requested_by, CLIENT_QUEST)
        self.assertEqual(ctx.exception.held_by, CLIENT_TELEGRAM)


class TestModeManagerEscapeHatch(unittest.TestCase):
    """Case 8: * → off is always allowed (operational escape)."""

    def test_any_mode_can_force_off(self):
        # off → off (no-op, но mode всё равно off)
        fsm = ModeManager()
        new_mode = fsm.transition("force_off")
        self.assertEqual(new_mode, Mode.OFF)
        self.assertEqual(fsm.mode, Mode.OFF)

        # telegram_active → off
        fsm2 = ModeManager()
        fsm2.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        new_mode = fsm2.transition("force_off")
        self.assertEqual(new_mode, Mode.OFF)

        # avatar_present → off
        fsm3 = ModeManager()
        fsm3.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        new_mode = fsm3.transition("force_off")
        self.assertEqual(new_mode, Mode.OFF)

        # mixed → off
        fsm4 = ModeManager()
        fsm4.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        fsm4.transition(EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id=CLIENT_QUEST)
        new_mode = fsm4.transition("force_off")
        self.assertEqual(new_mode, Mode.OFF)


class TestModeManagerIdempotency(unittest.TestCase):
    """Case 9: repeated acquire from the same client is a no-op."""

    def test_repeat_telegram_acquire_floor_is_noop(self):
        fsm = ModeManager()
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        # Повторный acquire от того же клиента — no-op, без ConflictError.
        new_mode = fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        self.assertEqual(new_mode, Mode.TELEGRAM_ACTIVE)
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)

    def test_repeat_quest_acquire_floor_is_noop(self):
        fsm = ModeManager()
        fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        new_mode = fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        self.assertEqual(new_mode, Mode.AVATAR_PRESENT)


class TestModeManagerUnknownEvent(unittest.TestCase):
    """Case 10: unknown event → ValueError."""

    def test_unknown_event_raises_value_error(self):
        fsm = ModeManager()
        with self.assertRaises(ValueError):
            fsm.transition("definitely_not_an_event")


class TestModeManagerReleaseSemantics(unittest.TestCase):
    """Smoke-tests beyond the 10 acceptance cases: release из простых режимов."""

    def test_avatar_present_releases_to_off_when_only_quest_active(self):
        fsm = ModeManager()
        fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR, client_id=CLIENT_QUEST)
        # Телеграм ещё ничего не делал — release телеграма не должен выкинуть FSM в off.
        fsm.transition(EVENT_TELEGRAM_RELEASE)
        self.assertEqual(fsm.mode, Mode.AVATAR_PRESENT)

    def test_quest_release_after_mixed_returns_to_telegram_active(self):
        # ADR-0028 §4.1: mixed → telegram_active : quest_release_teleop
        fsm = ModeManager()
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id=CLIENT_TELEGRAM)
        fsm.transition(EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id=CLIENT_QUEST)
        self.assertEqual(fsm.mode, Mode.MIXED)
        new_mode = fsm.transition(EVENT_QUEST_RELEASE, client_id=CLIENT_QUEST)
        self.assertEqual(new_mode, Mode.TELEGRAM_ACTIVE)


if __name__ == "__main__":
    unittest.main()
