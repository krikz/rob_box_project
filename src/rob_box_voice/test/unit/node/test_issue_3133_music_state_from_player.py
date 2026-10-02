"""Issue #3133 (ADR-0141) — dialogue_node читает «играет» только у плеера.

Живой прогон 28.09.2026 18:43–18:45 UTC: конечный club-трек 79 с доиграл,
jack_rec писал −180 dBFS, а dialogue_node логировал «TRACK играет с прошлого
хода», гуард #3125 — «retry budget exhausted while music is playing», и робот
сказал «Музыка играет, а вот эту просьбу я выполнить не смог».
"""

from __future__ import annotations

import json
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_voice.core.dialogue_guards import MUSIC_MODE_TOOLS, MUSIC_STOP_TOOLS
from rob_box_voice.dialogue_node import DialogueNode


def _node():
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._track_mode_music_active = False
    return n


def _msg(payload) -> SimpleNamespace:
    return SimpleNamespace(data=payload if isinstance(payload, str) else json.dumps(payload))


class TestMusicPlayingNowFromPlayer:
    def test_no_snapshot_means_not_playing(self):
        assert _node()._music_playing_now() is False

    def test_tool_name_flag_is_not_a_source(self):
        """Флаг TRACK-режима (ставится по именам тулов) больше не «играет»."""
        n = _node()
        n._track_mode_music_active = True
        assert n._music_playing_now() is False

    def test_playing_snapshot(self):
        n = _node()
        n._on_music_state(_msg({"state": "playing", "track_id": "a-1", "stops_at": None}))
        assert n._music_playing_now() is True

    def test_finished_track_is_not_playing(self):
        n = _node()
        n._track_mode_music_active = True
        n._on_music_state(_msg({"state": "playing", "track_id": "a-1"}))
        n._on_music_state(_msg({"state": "idle", "finished_track_id": "a-1"}))
        assert n._music_playing_now() is False
        # idle гасит и политику cleanup.
        assert n._track_mode_music_active is False

    def test_no_playing_after_stops_at_even_without_idle(self):
        """idle потерялся — после stops_at + 1 с всё равно не «играет»."""
        n = _node()
        n._on_music_state(_msg({"state": "playing", "stops_at": time.time() - 5.0}))
        assert n._music_playing_now() is False

    def test_garbage_keeps_previous_snapshot(self):
        n = _node()
        n._on_music_state(_msg({"state": "playing"}))
        n._on_music_state(_msg("not json"))
        assert n._music_playing_now() is True

    def test_handler_never_arms_the_track_flag(self):
        n = _node()
        n._on_music_state(_msg({"state": "playing"}))
        assert n._track_mode_music_active is False


class TestStopAndModeToolSets:
    def test_set_dj_mode_is_neither_stop_nor_mode_tool(self):
        # ADR-0149 PR-13a: set_dj_mode — тул старого пути, DJ-сет ведёт dj_set движка v2.
        assert "set_dj_mode" not in MUSIC_STOP_TOOLS | MUSIC_MODE_TOOLS
        assert MUSIC_STOP_TOOLS == frozenset({"stop_music"})
