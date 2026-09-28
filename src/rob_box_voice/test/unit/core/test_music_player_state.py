"""Issue #3133 (ADR-0141) — контракт снимка плеера ``/voice/music/state``."""

import json

from rob_box_voice.core.music_player_state import (
    STOPS_AT_GRACE_S,
    MusicPlayerState,
    build_music_state_payload,
    parse_music_state,
)


def test_round_trip_playing():
    data = build_music_state_payload(
        playing=True, track_id="a-1", form_ends_at=100.0, stops_at=200.0, dj=True, ts=50.0,
    )
    snap = parse_music_state(data)
    assert snap == MusicPlayerState(
        state="playing", track_id="a-1", form_ends_at=100.0, stops_at=200.0,
        dj=True, finished_track_id=None, ts=50.0,
    )


def test_finished_track_id_only_in_idle():
    playing = json.loads(build_music_state_payload(playing=True, finished_track_id="a-1"))
    idle = json.loads(build_music_state_payload(playing=False, finished_track_id="a-1"))
    assert playing["finished_track_id"] is None
    assert idle["finished_track_id"] == "a-1"
    assert idle["state"] == "idle"


def test_no_playing_after_stops_at_plus_grace():
    """Свежий idle не дошёл — после stops_at + grace музыка не «играет»."""
    snap = MusicPlayerState(state="playing", stops_at=1000.0)
    assert snap.is_playing(now=1000.0) is True
    assert snap.is_playing(now=1000.0 + STOPS_AT_GRACE_S) is True
    assert snap.is_playing(now=1000.0 + STOPS_AT_GRACE_S + 0.01) is False


def test_looping_track_plays_without_stops_at():
    assert MusicPlayerState(state="playing", stops_at=None).is_playing(now=1e12) is True


def test_idle_is_never_playing():
    assert MusicPlayerState(state="idle").is_playing() is False


def test_legacy_plain_strings_are_understood():
    assert parse_music_state("playing") == MusicPlayerState(state="playing")
    assert parse_music_state(" idle ") == MusicPlayerState(state="idle")


def test_garbage_is_ignored():
    for data in (None, "", "   ", "{", "[]", '{"state": "loud"}', "PLAYING"):
        assert parse_music_state(data) is None


def test_bad_field_types_are_dropped():
    snap = parse_music_state(json.dumps({
        "state": "playing", "track_id": 5, "stops_at": True, "dj": "yes",
    }))
    assert snap == MusicPlayerState(state="playing")
