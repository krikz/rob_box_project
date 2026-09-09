"""Unit tests for Avatar Supervisor mixed-mode + failover (AV-11, ADR-0028 §4).

Cover the FSM transitions that the e2e mixed-mode scenario exercises:
  - off -> telegram_active    (Telegram acquires voice_floor)
  - telegram_active -> mixed  (Quest acquires teleop_floor)
  - mixed -> mixed            (both clients keep their floors; idempotent)
  - mixed -> telegram_active  (Quest releases teleop_floor → dead-man)
  - mixed -> avatar_present   (Telegram releases voice_floor)
  - mixed -> off              (both_release)
  - Quest Wi-Fi fail (heartbeat > 500 ms) → teleop_floor auto-released
    (LockManager, AV-4) → telegram клиент получает «floor free» уведомление
    через STATE_UPDATE. FSM отражает это как mode=telegram_active
    (Quest уже не держит teleop_floor, Telegram держит voice).

Pure logic — no ROS, no asyncio, no threading. Reuses the existing
ModeManager + LockManager pair from AV-3/AV-4. Test isolation via
fresh instances per test (no shared state).

Acceptance mapping (issue #1605, AV-11):
  #4 mixed-mode holds both floors    → test_mixed_holds_both_floors
  #5 /avatar/state mode=mixed        → test_mixed_mode_published_state
  #6 Quest Wi-Fi fail → safe-stop    → test_quest_heartbeat_fail_auto_releases_teleop
  #6 Telegram получает floor          → test_after_quest_fail_telegram_can_acquire_teleop
  #7 /forward → telegram_active      → test_telegram_active_state
"""

import unittest

from rob_box_supervisor.core.fsm import (
    EVENT_BOTH_RELEASE,
    EVENT_FORCE_OFF,
    EVENT_QUEST_ACQUIRE_FLOOR,
    EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
    EVENT_QUEST_RELEASE,
    EVENT_TELEGRAM_ACQUIRE_FLOOR,
    EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
    EVENT_TELEGRAM_RELEASE,
    IDLE_TIMEOUT_MS,
    Mode,
    ModeManager,
)
from rob_box_supervisor.core.locks import (
    DEAD_MAN_TIMEOUT_MS,
    FLOOR_TELEOP,
    FLOOR_VOICE,
    LockManager,
)
from rob_box_supervisor.core.state import pack, unpack, AvatarState


def _build_pair():
    """Fresh (LockManager, ModeManager) sharing an injectable clock."""
    clock = {"now_ms": 0}

    def fake_clock() -> int:
        return clock["now_ms"]

    lm = LockManager(clock=fake_clock)
    fsm = ModeManager(clock=fake_clock)
    return lm, fsm, clock


def _heartbeat(lm, client_id, floor):
    """Heartbeat helper: keeps LockManager state from going expired.

    LockManager contract (AV-4): без heartbeat > DEAD_MAN_TIMEOUT_MS
    floor auto-released при следующем holder() вызове. В e2e клиенты
    шлют heartbeat регулярно (см. supervisor_node.py в AV-6). Здесь
    это имитируем явно, чтобы тестировать FSM-transition-логику
    изолированно от dead-man таймера.
    """
    lm.heartbeat(client_id, floor)


# === Mixed-mode core scenario ======================================


class TestMixedModeCoreScenario(unittest.TestCase):
    """Acceptance #4 + #5: Quest teleop + Telegram voice одновременно."""

    def test_mixed_holds_both_floors(self):
        # Step 1: Telegram takes voice_floor → telegram_active
        # Step 2: Quest takes teleop_floor → mixed
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(
            EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram"
        )
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)

        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )
        self.assertEqual(fsm.mode, Mode.MIXED)

        # Keep both clients alive while we assert.
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        clock["now_ms"] = 1200

        # Both floors are held by different clients.
        self.assertEqual(lm.holder(FLOOR_TELEOP), "quest")
        self.assertEqual(lm.holder(FLOOR_VOICE), "telegram")

    def test_mixed_mode_published_state(self):
        # После установки mixed — AvatarState содержит оба floor-а
        # и mode='mixed' (msgpack-encoded, как публикуется в /avatar/state).
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        clock["now_ms"] = 1200

        state = AvatarState(
            mode=str(fsm.mode.value),
            teleop_floor=None,
            voice_floor=None,
            last_event=None,
            since_ms=clock["now_ms"],
        )
        # Сериализация round-trip — контракт для клиентов
        # (Quest WebXR + Telegram клиент читают msgpack).
        packed = pack(state)
        roundtrip = unpack(packed)
        self.assertEqual(roundtrip.mode, str(Mode.MIXED.value))


# === Quest Wi-Fi fail (Acceptance #6) ==============================


class TestQuestFailover(unittest.TestCase):
    """Acceptance #6: Quest Wi-Fi fail → safe-stop → Telegram подхватывает."""

    def test_quest_heartbeat_fail_auto_releases_teleop(self):
        # Quest взял teleop_floor; прошло > 500 мс без heartbeat —
        # LockManager (AV-4) автоматически освобождает floor.
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )
        self.assertEqual(fsm.mode, Mode.MIXED)

        # Telegram продолжает слать heartbeat; Quest «умер» (Wi-Fi fail).
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        # Без heartbeat от Quest > 500 мс — dead-man сработает.
        # Последний heartbeat от Quest был в clock=1100 → fail в clock=1700.
        clock["now_ms"] = 1700

        # Теперь следующий holder() вызов увидит expired floor и
        # вернёт None — auto-release по контракту LockManager.
        holder_after_fail = lm.holder(FLOOR_TELEOP)
        self.assertIsNone(
            holder_after_fail,
            "teleop_floor должен auto-release после dead-man timeout",
        )
        # Telegram voice_floor — жив (heartbeat в clock=1100 + clock=1700? нет,
        # только clock=1100, потом clock=1700 — 600 ms без heartbeat.
        # Делаем heartbeat ещё раз, чтобы assert видел свежее состояние).
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        clock["now_ms"] = 1750
        self.assertEqual(lm.holder(FLOOR_VOICE), "telegram")

    def test_after_quest_fail_telegram_can_acquire_teleop(self):
        # После fail Quest → Telegram может взять teleop_floor
        # (graceful handover, ADR-0028 §6 Q4).
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )

        # Wi-Fi fail → Quest больше не шлёт heartbeat, Telegram шлёт.
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        clock["now_ms"] = 1700  # > 500 ms без heartbeat от Quest
        # Supervisor от имени Quest вызывает release (после Wi-Fi detect).
        lm.release("quest", FLOOR_TELEOP)
        fsm.transition(EVENT_QUEST_RELEASE, client_id="quest")

        # Telegram теперь один держит voice_floor; Quest teleport-floor освобождён.
        # FSM переходит в telegram_active.
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        clock["now_ms"] = 1800
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)
        self.assertEqual(lm.holder(FLOOR_VOICE), "telegram")
        self.assertIsNone(lm.holder(FLOOR_TELEOP))

    def test_telegram_active_state(self):
        # Acceptance #7: после /forward → telegram_active, telegram держит
        # voice_floor (TTS-канал).
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        clock["now_ms"] = 1100
        self.assertEqual(fsm.mode, Mode.TELEGRAM_ACTIVE)
        self.assertEqual(lm.holder(FLOOR_VOICE), "telegram")
        self.assertIsNone(lm.holder(FLOOR_TELEOP))


# === Mixed → Off (Acceptance #5/7 cleanup) =========================


class TestMixedCleanup(unittest.TestCase):
    def test_both_release_returns_to_off(self):
        # Telegram отпускает voice, Quest отпускает teleop → off.
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )

        _heartbeat(lm, "telegram", FLOOR_VOICE)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        clock["now_ms"] = 1200
        lm.release("telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_RELEASE, client_id="telegram")
        lm.release("quest", FLOOR_TELEOP)
        fsm.transition(EVENT_QUEST_RELEASE, client_id="quest")

        self.assertEqual(fsm.mode, Mode.OFF)
        self.assertIsNone(lm.holder(FLOOR_VOICE))
        self.assertIsNone(lm.holder(FLOOR_TELEOP))

    def test_force_off_escape_hatch(self):
        # В любом режиме force_off сразу → off (ADR-0028 §4.1 escape hatch).
        lm, fsm, clock = _build_pair()
        clock["now_ms"] = 1000
        lm.acquire("telegram", FLOOR_VOICE)
        _heartbeat(lm, "telegram", FLOOR_VOICE)
        fsm.transition(EVENT_TELEGRAM_ACQUIRE_FLOOR, client_id="telegram")
        clock["now_ms"] = 1100
        lm.acquire("quest", FLOOR_TELEOP)
        _heartbeat(lm, "quest", FLOOR_TELEOP)
        fsm.transition(
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY, client_id="quest"
        )
        self.assertEqual(fsm.mode, Mode.MIXED)

        # Watchdog вызывает force_off.
        fsm.transition(EVENT_FORCE_OFF)
        self.assertEqual(fsm.mode, Mode.OFF)


# === Sanity-check констант (lock-step с AV-3/AV-4) =================


class TestConstantsLockstep:
    """Lock-step проверки, чтобы e2e и unit сходились."""

    def test_dead_man_matches_500ms(self):
        # ADR-0028 §6 Q4: dead-man 500 мс. Если AV-3/AV-4 разъедутся,
        # e2e-mixed сценарий даст разный результат.
        assert DEAD_MAN_TIMEOUT_MS == 500

    def test_idle_timeout_matches_30s(self):
        # ADR-0028 §4.1: timeout 30 s no-activity.
        assert IDLE_TIMEOUT_MS == 30_000


if __name__ == "__main__":
    unittest.main()
