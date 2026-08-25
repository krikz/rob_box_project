"""Unit tests for LockManager (AV-4, ADR-0028 §4.2 + §6 Q4).

Pure logic — no ROS, no asyncio, no threading. Dead-man timeout uses
an injectable clock so tests are deterministic.

Acceptance criteria (issue #1598, AV-4):
  1. acquire → holder; release → None
  2. acquire(telegram, teleop), acquire(quest, voice) — оба держат (разные floors)
  3. acquire(quest, voice), acquire(telegram, voice) → второго ConflictError
  4. heartbeat refreshes; no heartbeat 500 мс → auto-release
  5. release by wrong client_id → PermissionError
  6. heartbeat by wrong client_id → PermissionError
  7. release уже-released floor — no-op (идемпотентность)
  8. holder(None) → ValueError
"""

import unittest

from rob_box_supervisor.core.locks import (
    ConflictError,
    DEAD_MAN_TIMEOUT_MS,
    FLOOR_TELEOP,
    FLOOR_VOICE,
    Floor,
    LockManager,
)


class TestLockManagerBasics(unittest.TestCase):
    """Case 1: acquire → holder; release → None."""

    def test_acquire_then_holder_returns_client_id(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")
        self.assertEqual(lm.holder(FLOOR_VOICE), None)

    def test_release_clears_holder(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        lm.release("telegram", FLOOR_TELEOP)
        self.assertIsNone(lm.holder(FLOOR_TELEOP))


class TestLockManagerIndependentFloors(unittest.TestCase):
    """Case 2: different floors are independent."""

    def test_two_clients_hold_different_floors(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        lm.acquire("quest", FLOOR_VOICE)
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")
        self.assertEqual(lm.holder(FLOOR_VOICE), "quest")

    def test_release_one_floor_does_not_affect_other(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        lm.acquire("quest", FLOOR_VOICE)
        lm.release("telegram", FLOOR_TELEOP)
        self.assertIsNone(lm.holder(FLOOR_TELEOP))
        self.assertEqual(lm.holder(FLOOR_VOICE), "quest")

    def test_same_client_can_hold_both_floors(self):
        lm = LockManager()
        lm.acquire("quest", FLOOR_TELEOP)
        lm.acquire("quest", FLOOR_VOICE)
        self.assertEqual(lm.holder(FLOOR_TELEOP), "quest")
        self.assertEqual(lm.holder(FLOOR_VOICE), "quest")


class TestLockManagerConflict(unittest.TestCase):
    """Case 3: second acquire on same floor by another client → ConflictError."""

    def test_second_acquire_same_floor_raises_conflict(self):
        lm = LockManager()
        lm.acquire("quest", FLOOR_VOICE)
        with self.assertRaises(ConflictError) as ctx:
            lm.acquire("telegram", FLOOR_VOICE)
        # ConflictError должен сообщать, кто держит floor
        self.assertEqual(ctx.exception.floor, FLOOR_VOICE)
        self.assertEqual(ctx.exception.held_by, "quest")
        self.assertEqual(ctx.exception.requested_by, "telegram")
        # Состояние не меняется на конфликте
        self.assertEqual(lm.holder(FLOOR_VOICE), "quest")

    def test_same_client_reacquire_is_idempotent(self):
        # Документируем поведение: повторный acquire от того же клиента — no-op
        # (это контракт FSM идемпотентности из AV-3, но и для LockManager полезно).
        lm = LockManager()
        lm.acquire("quest", FLOOR_VOICE)
        lm.acquire("quest", FLOOR_VOICE)  # no-op
        self.assertEqual(lm.holder(FLOOR_VOICE), "quest")


class TestLockManagerDeadMan(unittest.TestCase):
    """Case 4: heartbeat refreshes; no heartbeat > 500ms → auto-release."""

    def test_heartbeat_keeps_floor_alive(self):
        clock = [1000]
        lm = LockManager(clock=lambda: clock[0])

        lm.acquire("telegram", FLOOR_TELEOP, now_ms=clock[0])
        # Через 400 мс — ещё держит
        clock[0] += 400
        lm.heartbeat("telegram", FLOOR_TELEOP, now_ms=clock[0])
        # holder() внутри сам делает expire-check по clock
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")

    def test_no_heartbeat_over_500ms_auto_releases(self):
        clock = [1000]
        lm = LockManager(clock=lambda: clock[0])

        lm.acquire("telegram", FLOOR_TELEOP, now_ms=clock[0])
        # Перематываем на 501 мс — heartbeat не приходил
        clock[0] += 501
        # holder() должен вернуть None и пометить floor как expired
        self.assertIsNone(lm.holder(FLOOR_TELEOP))

    def test_deadman_timeout_constant_is_500ms(self):
        # ADR-0028 §6 Q4: dead-man 500 мс.
        self.assertEqual(DEAD_MAN_TIMEOUT_MS, 500)

    def test_deadman_at_exact_boundary_holds(self):
        # На ровно 500 мс — ещё держит (>500, не >=500).
        clock = [1000]
        lm = LockManager(clock=lambda: clock[0])

        lm.acquire("telegram", FLOOR_TELEOP, now_ms=clock[0])
        clock[0] += 500  # ровно 500 мс
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")
        clock[0] += 1  # 501 мс
        self.assertIsNone(lm.holder(FLOOR_TELEOP))


class TestLockManagerPermissions(unittest.TestCase):
    """Cases 5, 6: wrong client_id → PermissionError на release и heartbeat."""

    def test_release_by_wrong_client_raises(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        with self.assertRaises(PermissionError):
            lm.release("quest", FLOOR_TELEOP)
        # Состояние не меняется
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")

    def test_heartbeat_by_wrong_client_raises(self):
        # Используем инжектированные часы, чтобы dead-man не сработал
        # между acquire и holder() из-за реального времени.
        clock = [1000]
        lm = LockManager(clock=lambda: clock[0])
        lm.acquire("telegram", FLOOR_TELEOP, now_ms=clock[0])
        with self.assertRaises(PermissionError):
            lm.heartbeat("quest", FLOOR_TELEOP, now_ms=clock[0])
        # Floor всё ещё держится
        self.assertEqual(lm.holder(FLOOR_TELEOP), "telegram")


class TestLockManagerIdempotency(unittest.TestCase):
    """Case 7: release уже-released floor — no-op (идемпотентность)."""

    def test_release_already_released_floor_is_noop(self):
        lm = LockManager()
        lm.acquire("telegram", FLOOR_TELEOP)
        lm.release("telegram", FLOOR_TELEOP)
        # Повторный release не должен падать
        lm.release("telegram", FLOOR_TELEOP)
        self.assertIsNone(lm.holder(FLOOR_TELEOP))

    def test_release_never_acquired_floor_is_noop(self):
        lm = LockManager()
        # Никто не acquired — release тоже no-op
        lm.release("telegram", FLOOR_TELEOP)
        self.assertIsNone(lm.holder(FLOOR_TELEOP))


class TestLockManagerValidation(unittest.TestCase):
    """Case 8 + валидация аргументов."""

    def test_holder_with_none_raises_value_error(self):
        lm = LockManager()
        with self.assertRaises(ValueError):
            lm.holder(None)

    def test_holder_with_unknown_floor_raises_value_error(self):
        lm = LockManager()
        with self.assertRaises(ValueError):
            lm.holder("bogus_floor")

    def test_acquire_with_empty_client_id_raises_value_error(self):
        lm = LockManager()
        with self.assertRaises(ValueError):
            lm.acquire("", FLOOR_TELEOP)

    def test_floor_constants(self):
        self.assertEqual(FLOOR_TELEOP, "teleop_floor")
        self.assertEqual(FLOOR_VOICE, "voice_floor")
        self.assertIn(FLOOR_TELEOP, Floor.values())
        self.assertIn(FLOOR_VOICE, Floor.values())


if __name__ == "__main__":
    unittest.main()
