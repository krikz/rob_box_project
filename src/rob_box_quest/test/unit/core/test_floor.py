"""Unit-тесты AvatarStateFloorCache (core/floor.py).

После ADR-0051 §2.2 (issue #1999) tracker больше **не** имеет
acquire/release — это зона LockManager. Этот модуль — read-only
кэш, обновляемый из /avatar/state.

Покрывает:
- начальное состояние (пустое);
- update snapshot-а из /avatar/state;
- read-only свойства holder / voice_holder / avatar_mode;
- is_held_by / held_by_other — основной gate для teleop_twist;
- reset — очистка кэша при shutdown.
"""

from __future__ import annotations

from rob_box_quest.core.floor import (
    AvatarFloorSnapshot,
    AvatarStateFloorCache,
    QUEST_DEFAULT_CLIENT_ID,
)


class _FakeClock:
    """Не используется — оставлен чтобы pytest не падал на conftest-импортах."""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def test_initial_state_is_empty():
    cache = AvatarStateFloorCache()
    assert cache.holder is None
    assert cache.voice_holder is None
    assert cache.avatar_mode == "off"
    assert cache.is_held_by("anyone") is False
    assert cache.is_held_by(None) is False
    assert cache.held_by_other("anyone") is False


def test_update_holder():
    cache = AvatarStateFloorCache()
    cache.update(AvatarFloorSnapshot(teleop_holder="questA", avatar_mode="avatar_present"))
    assert cache.holder == "questA"
    assert cache.is_held_by("questA") is True
    assert cache.is_held_by("questB") is False
    assert cache.held_by_other("questB") is True
    # questA видит floor как свой, не как чужой.
    assert cache.held_by_other("questA") is False


def test_update_clears_holder_on_none():
    """После release в LockManager → avatar_arbiter пушит snapshot с
    teleop_holder=None → кэш должен отражать это."""
    cache = AvatarStateFloorCache()
    cache.update(AvatarFloorSnapshot(teleop_holder="questA"))
    assert cache.holder == "questA"

    cache.update(AvatarFloorSnapshot(teleop_holder=None))
    assert cache.holder is None
    assert cache.is_held_by("questA") is False


def test_update_voice_holder_independent():
    """Кэш хранит voice_floor holder-а отдельно от teleop — они независимы."""
    cache = AvatarStateFloorCache()
    cache.update(
        AvatarFloorSnapshot(
            teleop_holder="questA",
            voice_holder="telegram",
            avatar_mode="mixed",
        )
    )
    assert cache.holder == "questA"
    assert cache.voice_holder == "telegram"
    assert cache.avatar_mode == "mixed"


def test_held_by_other_when_free_returns_false():
    """Когда floor свободен, ``held_by_other`` должен возвращать False
    (а не True по умолчанию) — иначе ws_server будет гейтить
    teleop_twist когда никто не держит."""
    cache = AvatarStateFloorCache()
    assert cache.held_by_other("questA") is False
    # Не-зависит от client_id.
    assert cache.held_by_other(None) is False


def test_held_by_other_distinguishes_self_vs_other():
    cache = AvatarStateFloorCache()
    cache.update(AvatarFloorSnapshot(teleop_holder="questA"))
    assert cache.held_by_other("questA") is False
    assert cache.held_by_other("questB") is True
    assert cache.held_by_other(None) is True


def test_initial_snapshot_constructor():
    """Можно сразу инициализировать кэш с готовым snapshot (например,
    latched /avatar/state в startup)."""
    snap = AvatarFloorSnapshot(
        teleop_holder="questA",
        voice_holder="telegram",
        avatar_mode="mixed",
        schema_version=2,
    )
    cache = AvatarStateFloorCache(initial_snapshot=snap)
    assert cache.holder == "questA"
    assert cache.voice_holder == "telegram"
    assert cache.avatar_mode == "mixed"


def test_reset_clears_snapshot():
    cache = AvatarStateFloorCache()
    cache.update(AvatarFloorSnapshot(teleop_holder="questA"))
    cache.reset()
    assert cache.holder is None
    assert cache.is_held_by("questA") is False
    assert cache.held_by_other("anyone") is False


def test_update_is_idempotent():
    """Повторный update с тем же snapshot — корректное состояние без
    побочных эффектов."""
    cache = AvatarStateFloorCache()
    snap = AvatarFloorSnapshot(teleop_holder="questA")
    cache.update(snap)
    cache.update(snap)
    cache.update(snap)
    assert cache.holder == "questA"


def test_is_held_by_with_none_client_id_returns_false():
    """Если кто-то вызывает is_held_by(None) — никогда не True."""
    cache = AvatarStateFloorCache()
    cache.update(AvatarFloorSnapshot(teleop_holder=QUEST_DEFAULT_CLIENT_ID))
    assert cache.is_held_by(None) is False
    # Телеграм не держит:
    assert cache.held_by_other(None) is True
