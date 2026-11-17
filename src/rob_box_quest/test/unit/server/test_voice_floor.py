"""Unit-тесты VoiceFloorCache — чистая логика, без aiohttp/ROS.

После ADR-0051 §2.2 (issue #1999) VoiceFloor больше **не** имеет
try_acquire/release/force_release_for — это зона LockManager. Этот
модуль — read-only кэш голосового floor-а, обновляемый из
/avatar/state.
"""

from __future__ import annotations

from rob_box_quest.server.voice_floor import FloorHolder, FloorState, VoiceFloorCache


def test_initial_state_is_idle():
    floor = VoiceFloorCache()
    assert floor.state == FloorState.IDLE
    assert floor.holder is None


def test_update_state_listening_with_holder():
    """avatar_arbiter пушит LISTENING + holder — кэш отражает."""
    floor = VoiceFloorCache()
    holder = FloorHolder(session_id="s1", client_id="operator-quest")
    floor.update(FloorState.LISTENING, holder)
    assert floor.state == FloorState.LISTENING
    assert floor.holder is not None
    assert floor.holder.session_id == "s1"
    assert floor.holder.client_id == "operator-quest"


def test_update_state_denied_with_busy_holder():
    """avatar_arbiter отдал DENIED — busy_holder показывает, кто держит."""
    floor = VoiceFloorCache()
    busy = FloorHolder(session_id="s1", client_id="operator")
    floor.update(FloorState.DENIED, busy)
    assert floor.state == FloorState.DENIED
    assert floor.holder == busy


def test_update_state_idle_clears_holder():
    """После release в LockManager → avatar_arbiter пушит IDLE → кэш чист."""
    floor = VoiceFloorCache()
    floor.update(
        FloorState.LISTENING,
        FloorHolder(session_id="s1", client_id="operator"),
    )
    assert floor.holder is not None

    floor.update(FloorState.IDLE, None)
    assert floor.state == FloorState.IDLE
    assert floor.holder is None


def test_update_state_speaking():
    """speaking state используется для TTS-канала — кэш отражает."""
    floor = VoiceFloorCache()
    floor.update(
        FloorState.SPEAKING,
        FloorHolder(session_id="s2", client_id="tts-bridge"),
    )
    assert floor.state == FloorState.SPEAKING
    assert floor.holder.session_id == "s2"


def test_is_held_by_session_when_idle_returns_false():
    floor = VoiceFloorCache()
    assert floor.is_held_by_session("s1") is False
    assert floor.is_held_by_session(None) is False


def test_is_held_by_session_returns_true_for_current_holder():
    floor = VoiceFloorCache()
    floor.update(
        FloorState.LISTENING,
        FloorHolder(session_id="s1", client_id="operator"),
    )
    assert floor.is_held_by_session("s1") is True
    assert floor.is_held_by_session("s2") is False


def test_held_by_other_session_when_idle_returns_false():
    floor = VoiceFloorCache()
    assert floor.held_by_other_session("s1") is False


def test_held_by_other_session_distinguishes_self_vs_other():
    floor = VoiceFloorCache()
    floor.update(
        FloorState.LISTENING,
        FloorHolder(session_id="s1", client_id="operator"),
    )
    assert floor.held_by_other_session("s1") is False
    assert floor.held_by_other_session("s2") is True


def test_initial_holder_constructor():
    """Можно сразу инициализировать с готовым holder (latched /avatar/state)."""
    holder = FloorHolder(session_id="s1", client_id="operator")
    floor = VoiceFloorCache(
        initial_state=FloorState.LISTENING,
        initial_holder=holder,
    )
    assert floor.state == FloorState.LISTENING
    assert floor.holder == holder


def test_reset_returns_to_idle():
    floor = VoiceFloorCache()
    floor.update(
        FloorState.LISTENING,
        FloorHolder(session_id="s1", client_id="operator"),
    )
    floor.reset()
    assert floor.state == FloorState.IDLE
    assert floor.holder is None


def test_holder_label_is_short_and_unique():
    """FloorHolder.label() остался без изменений — это просто форматтер."""
    h = FloorHolder(session_id="abcdef1234567890", client_id="operator-quest")
    assert h.label() == "operator-quest:abcdef12"


def test_update_is_idempotent():
    floor = VoiceFloorCache()
    holder = FloorHolder(session_id="s1", client_id="operator")
    floor.update(FloorState.LISTENING, holder)
    floor.update(FloorState.LISTENING, holder)
    floor.update(FloorState.LISTENING, holder)
    assert floor.state == FloorState.LISTENING
    assert floor.holder == holder
