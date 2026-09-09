"""Tests for /avatar/state msgpack schema and serialization round-trip.

References:
    docs/adr/0028-avatar-supervisor.md §4.3 (AvatarState.msg)
    docs/architecture/SYSTEM_OVERVIEW.md §5.4 (/avatar/state table)
"""

from __future__ import annotations

import msgpack

from rob_box_supervisor.core.state import (
    AvatarEvent,
    AvatarState,
    FloorState,
    StateVersionError,
    pack,
    unpack,
)

SCHEMA_VERSION = 1


# ---------------------------------------------------------------------------
# 1. Round-trip AvatarState (all fields populated)
# ---------------------------------------------------------------------------


def test_round_trip_avatar_state_all_fields():
    state = AvatarState(
        mode="active",
        teleop_floor=FloorState(
            client_id="quest-001",
            since_ms=1_700_000_000_000,
            last_heartbeat_ms=1_700_000_000_500,
        ),
        voice_floor=FloorState(
            client_id="telegram-bot",
            since_ms=1_700_000_000_100,
            last_heartbeat_ms=1_700_000_000_400,
        ),
        last_event=AvatarEvent(
            timestamp_ms=1_700_000_000_500,
            client_id="quest-001",
            kind="mode_change",
            args={"old": "idle", "new": "active"},
        ),
        since_ms=1_700_000_000_000,
        version=SCHEMA_VERSION,
    )

    data = pack(state)
    decoded = unpack(data)

    assert decoded == state
    assert decoded.mode == "active"
    assert decoded.teleop_floor is not None
    assert decoded.teleop_floor.client_id == "quest-001"
    assert decoded.voice_floor is not None
    assert decoded.voice_floor.client_id == "telegram-bot"
    assert decoded.last_event is not None
    assert decoded.last_event.kind == "mode_change"
    assert decoded.last_event.args == {"old": "idle", "new": "active"}
    assert decoded.version == SCHEMA_VERSION


# ---------------------------------------------------------------------------
# 2. Round-trip with None floors (off-mode / idle)
# ---------------------------------------------------------------------------


def test_round_trip_avatar_state_off_mode_no_floors():
    state = AvatarState(
        mode="off",
        teleop_floor=None,
        voice_floor=None,
        last_event=None,
        since_ms=1_700_000_000_000,
        version=SCHEMA_VERSION,
    )

    data = pack(state)
    decoded = unpack(data)

    assert decoded == state
    assert decoded.mode == "off"
    assert decoded.teleop_floor is None
    assert decoded.voice_floor is None
    assert decoded.last_event is None


# ---------------------------------------------------------------------------
# 3. Forward-compat: payload with version+1 — raise (current policy: raise)
# ---------------------------------------------------------------------------


def test_forward_compat_higher_version_raises():
    # Manually craft a payload one version ahead of ours.
    future = {
        "mode": "active",
        "teleop_floor": None,
        "voice_floor": None,
        "last_event": None,
        "since_ms": 1_700_000_000_000,
        "version": SCHEMA_VERSION + 1,
        # some future field the receiver does not know about
        "_future_field": [1, 2, 3],
    }
    data = msgpack.packb(future, use_bin_type=True)

    import pytest

    with pytest.raises(StateVersionError) as exc_info:
        unpack(data)
    assert "version" in str(exc_info.value).lower()


# ---------------------------------------------------------------------------
# 4. Backward-compat: payload from an old client without last_event -> defaults
# ---------------------------------------------------------------------------


def test_backward_compat_missing_last_event():
    # Simulate an older client that did not yet emit last_event.
    legacy = {
        "mode": "idle",
        "teleop_floor": None,
        "voice_floor": None,
        # last_event is absent entirely
        "since_ms": 1_700_000_000_000,
        "version": SCHEMA_VERSION,
    }
    data = msgpack.packb(legacy, use_bin_type=True)

    decoded = unpack(data)

    assert decoded.mode == "idle"
    assert decoded.last_event is None
    assert decoded.teleop_floor is None
    assert decoded.voice_floor is None
    assert decoded.since_ms == 1_700_000_000_000
    assert decoded.version == SCHEMA_VERSION


# ---------------------------------------------------------------------------
# 5. Typical payload size: off-mode (no floors, no event) <= 200 bytes
# ---------------------------------------------------------------------------


def test_off_mode_payload_size_under_200_bytes():
    state = AvatarState(
        mode="off",
        teleop_floor=None,
        voice_floor=None,
        last_event=None,
        since_ms=1_700_000_000_000,
        version=SCHEMA_VERSION,
    )
    data = pack(state)

    assert len(data) <= 200, (
        f"off-mode payload too large: {len(data)} bytes (budget 200) — "
        "1 Hz publication is 1 byte/sec extra over a 200 B/s budget"
    )


# ---------------------------------------------------------------------------
# 6. msgpack contract: external msgpack.unpackb can decode pack() output
# ---------------------------------------------------------------------------


def test_msgpack_contract_external_unpackb_decodes_pack():
    state = AvatarState(
        mode="active",
        teleop_floor=FloorState(
            client_id="quest",
            since_ms=1,
            last_heartbeat_ms=2,
        ),
        voice_floor=None,
        last_event=AvatarEvent(
            timestamp_ms=10,
            client_id="quest",
            kind="ping",
            args={"k": "v"},
        ),
        since_ms=1,
        version=SCHEMA_VERSION,
    )
    data = pack(state)

    # Decode with the *external* msgpack API (no helper from our package).
    raw = msgpack.unpackb(data, raw=False, strict_map_key=False)

    assert isinstance(raw, dict)
    assert raw["mode"] == "active"
    assert raw["version"] == SCHEMA_VERSION
    assert raw["teleop_floor"]["client_id"] == "quest"
    assert raw["voice_floor"] is None
    assert raw["last_event"]["kind"] == "ping"
    assert raw["last_event"]["args"] == {"k": "v"}
