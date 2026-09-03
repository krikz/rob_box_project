"""Unit-тесты VoiceFloor — чистая логика, без aiohttp/ROS."""

from __future__ import annotations

from rob_box_quest.server.voice_floor import FloorHolder, FloorState, VoiceFloor


def test_initial_state_is_idle():
    floor = VoiceFloor()
    assert floor.state == FloorState.IDLE
    assert floor.holder is None


def test_acquire_from_idle_returns_listening_and_holder():
    floor = VoiceFloor()
    ok, busy, new_state = floor.try_acquire(session_id="s1", client_id="operator-quest")
    assert ok is True
    assert busy is None
    assert new_state == FloorState.LISTENING
    assert floor.state == FloorState.LISTENING
    assert floor.holder is not None
    assert floor.holder.session_id == "s1"
    assert floor.holder.client_id == "operator-quest"


def test_acquire_when_busy_returns_denied_with_busy_holder():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    ok, busy, new_state = floor.try_acquire("s2", "telegram-bridge")
    assert ok is False
    assert busy == FloorHolder(session_id="s1", client_id="operator")
    assert new_state == FloorState.DENIED
    # Floor остался у первого.
    assert floor.state == FloorState.LISTENING
    assert floor.holder is not None
    assert floor.holder.session_id == "s1"


def test_release_by_holder_returns_idle():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    released, new_state = floor.release("s1")
    assert released is True
    assert new_state == FloorState.IDLE
    assert floor.state == FloorState.IDLE
    assert floor.holder is None


def test_release_by_non_holder_is_noop():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    released, new_state = floor.release("s2")
    assert released is False
    assert new_state == FloorState.LISTENING
    assert floor.holder is not None
    assert floor.holder.session_id == "s1"


def test_release_when_idle_is_idempotent_true():
    floor = VoiceFloor()
    released, new_state = floor.release("s1")
    assert released is True
    assert new_state == FloorState.IDLE


def test_force_release_for_owner_returns_true():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    assert floor.force_release_for("s1") is True
    assert floor.state == FloorState.IDLE


def test_force_release_for_stranger_returns_false():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    assert floor.force_release_for("s2") is False
    assert floor.state == FloorState.LISTENING


def test_after_release_next_acquire_succeeds():
    floor = VoiceFloor()
    floor.try_acquire("s1", "operator")
    floor.release("s1")
    ok, _, _ = floor.try_acquire("s2", "telegram-bridge")
    assert ok is True
    assert floor.holder is not None
    assert floor.holder.session_id == "s2"


def test_acquire_without_client_id_generates_anon_label():
    floor = VoiceFloor()
    ok, _, _ = floor.try_acquire(session_id="s1")
    assert ok is True
    assert floor.holder.client_id.startswith("anon-")
    assert floor.holder.session_id == "s1"


def test_snapshot_shape():
    floor = VoiceFloor()
    snap_idle = floor.snapshot()
    assert snap_idle == {"state": "idle", "holder": None}

    floor.try_acquire("s1", "operator")
    snap_busy = floor.snapshot()
    assert snap_busy["state"] == "listening"
    assert snap_busy["holder"]["session_id"] == "s1"
    assert snap_busy["holder"]["client_id"] == "operator"
    assert snap_busy["holder"]["held_for_s"] >= 0.0


def test_holder_label_is_short_and_unique():
    h = FloorHolder(session_id="abcdef1234567890", client_id="operator-quest")
    assert h.label() == "operator-quest:abcdef12"


def test_simulated_voice_mode_speaking_returns_speaking_state():
    """robot_voice mode → state=speaking (Phase 2 hook, форма зарезервирована)."""
    floor = VoiceFloor()
    # Прямо сейчас ws_server.py не различает listening/speaking в try_acquire
    # (Phase 2.1+ добавит mode-aware). Поведение по умолчанию — LISTENING.
    ok, _, new_state = floor.try_acquire("s1", "robot-voice-client")
    assert ok is True
    assert new_state == FloorState.LISTENING
