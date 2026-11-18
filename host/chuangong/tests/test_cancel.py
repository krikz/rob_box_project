"""Локальная отмена staged: маркер на диске, current не переключается, сети нет."""

from __future__ import annotations

import json

import pytest
from chuangong import cancel, pointers
from chuangong import state as st
from chuangong.stage import MARKER
from chuangong.state import Mode


def _paths(tmp_path):
    return (
        tmp_path / "var" / "chuangong",
        tmp_path / "opt" / "releases",
        tmp_path / "opt" / "rob_box",
    )


def _plant_staged(releases_root, name, node="vision"):
    dest = releases_root / name / node
    dest.mkdir(parents=True)
    (dest / MARKER).write_text("deadbeef" * 8 + "\n", encoding="utf-8")
    (dest / "docker-compose.yaml").write_text("stay\n", encoding="utf-8")
    return dest


def _save_staged(state_dir, node="vision", staged="2026.11.09-1", current="2026.11.08-1", previous=None):
    rec = st.new_record("RBXU100001", node, "dev", Mode.OBSERVE, None)
    rec["current"] = current
    rec["previous"] = previous
    rec["staged"] = staged
    rec["state"] = st.State.STAGED.value
    st.save(state_dir / "state.json", rec)
    return rec


def test_cancel_marks_staged_and_keeps_current(tmp_path):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    dest = _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    record = cancel.cancel_staged(state_dir, releases_root, "vision")

    assert record["state"] == "cancelled"
    assert record["cancelled"] == "2026.11.09-1"
    assert record["staged"] is None
    assert record["current"] == "2026.11.08-1"
    assert record["previous"] is None
    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)
    saved = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.08-1" and saved["previous"] is None
    assert saved["staged"] is None and saved["cancelled"] == "2026.11.09-1"
    assert (dest / cancel.CANCEL_MARKER).read_text(encoding="utf-8").strip() == "2026.11.09-1"
    assert (dest / MARKER).is_file()
    assert (dest / "docker-compose.yaml").read_text(encoding="utf-8") == "stay\n"
    assert not (compose_root / "current").exists()
    assert not (releases_root / "current").exists()


def test_cancel_leaves_existing_pointers_untouched(tmp_path):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir, previous="2026.11.07-1")
    pointers.advance(state_dir, "2026.11.07-1")
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)
    compose_root.mkdir(parents=True)
    live = compose_root / "docker-compose.yaml"
    live.write_text("live\n", encoding="utf-8")
    (compose_root / "current").symlink_to(live)

    record = cancel.cancel_staged(state_dir, releases_root, "vision")

    assert record["current"] == "2026.11.08-1"
    assert record["previous"] == "2026.11.07-1"
    assert pointers.load(state_dir) == before == ("2026.11.08-1", "2026.11.07-1")
    assert (state_dir / "current").is_file() and not (state_dir / "current").is_symlink()
    assert (compose_root / "current").is_symlink()
    assert live.read_text(encoding="utf-8") == "live\n"


def test_nothing_staged_does_not_write(tmp_path):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    with pytest.raises(cancel.CancelError, match="nothing_staged"):
        cancel.cancel_staged(state_dir, releases_root, "vision")

    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)
    assert not (state_dir / "state.json").exists()
    assert not (compose_root / "current").exists()
    assert not releases_root.exists()


def test_not_on_disk_does_not_change_current(tmp_path):
    state_dir, releases_root, _ = _paths(tmp_path)
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    with pytest.raises(cancel.CancelError, match="release_not_staged"):
        cancel.cancel_staged(state_dir, releases_root, "vision")

    saved = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.08-1"
    assert saved["staged"] == "2026.11.09-1"
    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)


def test_bad_node_does_not_write(tmp_path):
    state_dir, releases_root, _ = _paths(tmp_path)
    _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)
    with pytest.raises(cancel.CancelError, match="bad_node"):
        cancel.cancel_staged(state_dir, releases_root, "gpu")
    saved = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert saved["staged"] == "2026.11.09-1" and saved["current"] == "2026.11.08-1"
    assert pointers.load(state_dir) == before


def test_cancel_does_not_open_network(tmp_path, monkeypatch):
    def boom(*_a, **_k):
        raise AssertionError("сеть")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    monkeypatch.setattr("socket.create_connection", boom)
    state_dir, releases_root, _ = _paths(tmp_path)
    _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    record = cancel.cancel_staged(state_dir, releases_root, "vision")
    assert record["state"] == "cancelled"
    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)
