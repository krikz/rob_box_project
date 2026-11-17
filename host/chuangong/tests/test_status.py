"""Наблюдатель F: Update Robot (action: status) на фейковом zenoh."""

from __future__ import annotations

from chuangong.status import collect, render, verdict
from fakes import FakeZenoh


def _rec(node, state="available", available="2026.11.09-1", **kw):
    return {"node": node, "state": state, "available": available, "staged": None, "current": None, **kw}


KEY = "robots/RBXU100001/ops/chuangong/state/"


def test_all_nodes_from_subscription():
    z = FakeZenoh(stream=[(KEY + n, _rec(n)) for n in ("main", "vision", "sound")] + [(KEY + "main", "junk")])
    found, notes = collect(z, "RBXU100001", wait_s=5)
    assert set(found) == {"main", "vision", "sound"}
    ok, problems = verdict(found, "2026.11.09-1")
    assert ok and problems == []
    text = render("RBXU100001", found, ok, problems, notes)
    assert "| vision | available | 2026.11.09-1 |" in text
    assert "все узлы отчитались" in text


def test_missing_node_and_error_state_fail():
    z = FakeZenoh(
        stored=[(KEY + "main", _rec("main"))],
        stream=[(KEY + "vision", _rec("vision", state="fetch_error", reason="bad_signature: x"))],
    )
    found, _ = collect(z, "RBXU100001", wait_s=5)
    ok, problems = verdict(found, None)
    assert not ok
    assert "sound: нет отчёта агента" in problems
    assert any(p.startswith("vision: fetch_error") for p in problems)


def test_wrong_release_and_split_brain_fail():
    found = {"main": _rec("main"), "vision": _rec("vision", available="2026.11.08-3"), "sound": _rec("sound")}
    ok, problems = verdict(found, "2026.11.09-1")
    assert not ok
    assert any("vision: релиз 2026.11.09-1 не виден" in p for p in problems)
    assert any("разные релизы" in p for p in problems)


def test_no_wait_no_subscription():
    z = FakeZenoh(stream=[(KEY + "main", _rec("main"))])
    found, notes = collect(z, "RBXU100001", wait_s=0)
    assert found == {}
    assert len(notes) == 1
