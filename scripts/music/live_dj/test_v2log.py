"""Самопроверка разбора v2-лога: ``python -m pytest scripts/music/live_dj/test_v2log.py``."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import v2log  # noqa: E402

STARTED = ("[mcp_server-10] [INFO] [1791212154.284156992] [mcp_server]: 🎵 [music v2] started "
           "track_id=set12076:02:B:7e3c14d8 clock_beat=736.065 start_beat=736.0 phase_in_form=0.0 "
           "late_beats=0.065 bpm=128.0 deck=B form_beats=192.0 players_aligned=True latency_s=0.5")
SET = ("[mcp_server-10] [INFO] [1791212154.1] [mcp_server]: 🎧 [set v2] set12076 трек 2 started "
       "track_id=set12076:02:B:7e3c14d8 form_end_beat=928.0 source=theme A11=1/2")
PLAN = ("[mcp_server-10] [INFO] [1791212100.5] [mcp_server]: 🧠 [reasoner] set12816 план применён: "
        "row=cyber mode=minor hooks=['a', 'b']")


def test_started():
    r = v2log.parse_started(STARTED)
    assert r["track_id"] == "set12076:02:B:7e3c14d8"
    assert r["bpm"] == 128 and r["deck"] == "B"
    assert abs(r["t_abs"] - 1791212154.284156992) < 1e-3


def test_started_ignores_set_line():
    assert v2log.parse_started(SET) is None


def test_set_started():
    assert v2log.parse_set_started(SET) == ("set12076:02:B:7e3c14d8", "theme")
    assert v2log.parse_set_started(STARTED) is None


def test_plan():
    assert v2log.parse_plan(PLAN) == {"row": "cyber", "mode": "minor", "hooks": "['a', 'b']"}
