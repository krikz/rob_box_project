"""Текст ходов — только в e2e-БД вне /data (ADR-0150 §7.4, В8; PR-8 #3702)."""

import json
import sqlite3
import threading

import pytest

from rob_box_dialog.e2e_events import KINDS, TABLE, inside_robot_data, open_events


def test_prod_default_writes_nothing(tmp_path):
    """Дефолт (прод): путь пуст — писателя нет, записать текст невозможно."""
    assert open_events("", "s") is None
    assert open_events("   ", "s") is None
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("path", ["/data/dialog.e2e.db", "/data", "/data/sub/x.db", "/tmp/../data/x.db",
                                  "\\data\\x.db"])
def test_robot_data_is_refused_loudly(path):
    assert inside_robot_data(path)
    with pytest.raises(ValueError):
        open_events(path, "s")


@pytest.mark.parametrize("path", ["/database/x.db", "/tmp/dialog.e2e.db", "/ws/e2e/data/x.db", "data/x.db"])
def test_other_paths_are_not_robot_data(path):
    assert not inside_robot_data(path)


def test_events_are_written_with_text(tmp_path):
    path = tmp_path / "sub" / "e2e.db"
    events = open_events(str(path), "sess-1", clock=lambda: 1000.5)
    events.write("user", turn_id="t1", epoch=2, agent="personality", text="включи музыку")
    events.write("action", turn_id="t1", epoch=2, agent="personality",
                 data={"tool": "music_library", "status": "done"})
    events.close()
    rows = sqlite3.connect(path).execute(
        f"SELECT ts, session_id, epoch, turn_id, agent, kind, text, data_json FROM {TABLE} ORDER BY id").fetchall()
    assert rows[0] == (1000.5, "sess-1", 2, "t1", "personality", "user", "включи музыку", "{}")
    assert rows[1][5] == "action" and json.loads(rows[1][7]) == {"tool": "music_library", "status": "done"}


def test_unknown_kind_is_refused(tmp_path):
    events = open_events(str(tmp_path / "e2e.db"), "s")
    with pytest.raises(ValueError):
        events.write("thought", turn_id="t", epoch=0, agent="personality")
    events.close()


def test_writes_from_threads(tmp_path):
    events = open_events(str(tmp_path / "e2e.db"), "s")

    def write(n):
        events.write("user", turn_id=f"t{n}", epoch=0, agent="personality", text=str(n))

    threads = [threading.Thread(target=write, args=(n,)) for n in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    events.close()
    assert sqlite3.connect(tmp_path / "e2e.db").execute(f"SELECT COUNT(*) FROM {TABLE}").fetchone() == (8,)


def test_kinds_are_what_metrics_read():
    assert KINDS == ("user", "speech_requested", "speech_started", "speech_spoken", "action", "command", "turn")
