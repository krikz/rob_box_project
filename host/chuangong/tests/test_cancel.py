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


def _save_staged(
    state_dir,
    node="vision",
    staged="2026.11.09-1",
    current="2026.11.08-1",
    previous=None,
):
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


def test_cancel_refuses_the_current_release(tmp_path):
    """Отмена не трогает релиз, который уже current (#4917)."""
    state_dir, releases_root, _compose = _paths(tmp_path)
    _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.09-1")

    with pytest.raises(cancel.CancelError) as caught:
        cancel.cancel_staged(state_dir, releases_root, "vision")

    assert caught.value.reason == "release_is_current"
    assert pointers.load(state_dir) == ("2026.11.09-1", None)


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


def test_is_cancelled_requires_matching_marker_name(tmp_path):
    state_dir, releases_root, _ = _paths(tmp_path)
    dest = _plant_staged(releases_root, "2026.11.09-1")
    assert cancel.is_cancelled(releases_root, "2026.11.09-1", "vision") is False
    (dest / cancel.CANCEL_MARKER).write_text("other\n", encoding="utf-8")
    assert cancel.is_cancelled(releases_root, "2026.11.09-1", "vision") is False
    (dest / cancel.CANCEL_MARKER).write_text("2026.11.09-1\n", encoding="utf-8")
    assert cancel.is_cancelled(releases_root, "2026.11.09-1", "vision") is True


def test_tick_respects_cancel_marker_and_does_not_restage(tmp_path, world):
    from chuangong.agent import Config, Deps, tick
    from chuangong.state import Mode
    from fakes import FakeDocker, FakeZenoh

    state_dir, releases_root, _ = _paths(tmp_path)
    name = world.release["release"]
    dest = _plant_staged(releases_root, name)
    _save_staged(state_dir, staged=name, current="2026.11.08-1")
    cancel.cancel_staged(state_dir, releases_root, "vision")
    assert (dest / cancel.CANCEL_MARKER).is_file()
    assert cancel.is_cancelled(releases_root, name, "vision")

    cfg = Config(
        robot_id="RBXU100001",
        node="vision",
        pubkey=world.pubkey,
        state_dir=state_dir,
        releases_root=releases_root,
        docker_root=tmp_path,
        mode=Mode.STAGE,
        reserve_bytes=0,
    )
    docker = FakeDocker()
    docker.pulled = []
    deps = Deps(world.registry, docker, FakeZenoh(), disk_free=lambda p: 1 << 40)

    rec = tick(cfg, deps)

    assert rec["state"] == "cancelled"
    assert rec["cancelled"] == name
    assert rec["staged"] is None
    assert rec["available"] == name
    assert docker.pulled == []
    saved = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert saved["state"] == "cancelled" and saved["staged"] is None
    assert saved["current"] == "2026.11.08-1"
    assert (dest / cancel.CANCEL_MARKER).is_file()


def test_activation_blocks_cancelled_release(tmp_path, world):
    from chuangong.activation import ActivationMachine
    from chuangong.agent import Config, Deps
    from chuangong.stage import RoleStager
    from fakes import FakeDocker, FakeZenoh

    state_dir, releases_root, _ = _paths(tmp_path)
    name = world.release["release"]
    cfg = Config(
        robot_id="RBXU100001",
        node="vision",
        pubkey=world.pubkey,
        state_dir=state_dir,
        releases_root=releases_root,
        legacy_root=tmp_path / "legacy",
        docker_root=tmp_path,
        hil_lock=tmp_path / "hil.lock",
        reserve_bytes=0,
    )
    docker = FakeDocker()
    deps = Deps(world.registry, docker, FakeZenoh(), disk_free=lambda p: 1 << 40)
    RoleStager(
        deps.registry,
        deps.docker,
        cfg.releases_root,
        cfg.docker_root,
        0,
        deps.disk_free,
    ).run(world.release, cfg.node)
    _save_staged(state_dir, staged=name, current="2026.11.08-1")
    cancel.cancel_staged(state_dir, releases_root, "vision")

    rec = ActivationMachine(cfg, deps).activate(name)

    assert rec["state"] == "blocked" and rec["reason"] == "release_cancelled"
    assert rec["detail"]["release"] == name
    assert rec["current"] == "2026.11.08-1"
    saved = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.08-1"
    assert saved["state"] == "cancelled"
    assert pointers.load(state_dir) == (None, None)


def test_cli_cancel_marks_staged_and_keeps_current(tmp_path, capsys):
    from chuangong.__main__ import main

    state_dir, releases_root, compose_root = _paths(tmp_path)
    dest = _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    code = main(
        [
            "cancel",
            "--node",
            "vision",
            "--state-dir",
            str(state_dir),
            "--releases-root",
            str(releases_root),
        ]
    )
    out = capsys.readouterr().out
    record = json.loads(out)

    assert code == 0
    assert record["state"] == "cancelled"
    assert record["cancelled"] == "2026.11.09-1"
    assert record["staged"] is None
    assert record["current"] == "2026.11.08-1"
    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)
    assert (dest / cancel.CANCEL_MARKER).read_text(encoding="utf-8").strip() == "2026.11.09-1"
    assert not (compose_root / "current").exists()


def test_cli_cancel_nothing_staged_exits_error(tmp_path, capsys):
    from chuangong.__main__ import main

    state_dir, releases_root, _ = _paths(tmp_path)
    pointers.advance(state_dir, "2026.11.08-1")
    before = pointers.load(state_dir)

    code = main(
        [
            "cancel",
            "--node",
            "vision",
            "--state-dir",
            str(state_dir),
            "--releases-root",
            str(releases_root),
        ]
    )
    out = json.loads(capsys.readouterr().out)

    assert code == 1
    assert out["state"] == "error" and out["reason"] == "nothing_staged"
    assert pointers.load(state_dir) == before == ("2026.11.08-1", None)
    assert not (state_dir / "state.json").exists()


def test_cli_cancel_does_not_open_network(tmp_path, monkeypatch, capsys):
    from chuangong.__main__ import main

    def boom(*_a, **_k):
        raise AssertionError("сеть")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    monkeypatch.setattr("socket.create_connection", boom)
    state_dir, releases_root, _ = _paths(tmp_path)
    _plant_staged(releases_root, "2026.11.09-1")
    _save_staged(state_dir)
    pointers.advance(state_dir, "2026.11.08-1")

    code = main(
        [
            "cancel",
            "--node",
            "vision",
            "--state-dir",
            str(state_dir),
            "--releases-root",
            str(releases_root),
        ]
    )
    record = json.loads(capsys.readouterr().out)
    assert code == 0
    assert record["state"] == "cancelled"
    assert pointers.load(state_dir) == ("2026.11.08-1", None)


def test_cli_exposes_cancel_help():
    from chuangong.__main__ import main

    with pytest.raises(SystemExit) as exc:
        main(["cancel", "--help"])
    assert exc.value.code == 0
