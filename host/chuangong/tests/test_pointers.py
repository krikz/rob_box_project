"""Указатели current/previous: после проверки подписи staged-релиза, без сети и без compose symlink."""

from __future__ import annotations

import copy

import pytest
from chuangong import minisign, pointers
from chuangong import release_format as rf
from chuangong.stage import MARKER


def _plant_staged(releases_root, release, node="vision"):
    dest = releases_root / release["release"] / node
    dest.mkdir(parents=True)
    (dest / MARKER).write_text(release["roles"][node]["bundle"]["sha256"] + "\n", encoding="utf-8")
    return dest


def _signed(world, release=None):
    rel = release or world.release
    data = rf.dumps(rel)
    sig = minisign.sign(data, world.secret, f"release:{rel['release']}")
    return data, sig


def _rename(release, name):
    rel = copy.deepcopy(release)
    rel["release"] = name
    for role, entry in rel["roles"].items():
        entry["bundle"]["file"] = rf.bundle_file_name(role, name)
    return rel


def _paths(tmp_path):
    return (
        tmp_path / "var" / "chuangong",
        tmp_path / "opt" / "releases",
        tmp_path / "opt" / "rob_box",
    )


def test_commit_after_verify_sets_current_and_null_previous(tmp_path, world):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    _plant_staged(releases_root, world.release)
    data, sig = _signed(world)

    result = pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)

    name = world.release["release"]
    assert result == {"current": name, "previous": None}
    assert (state_dir / "current").is_file() and not (state_dir / "current").is_symlink()
    assert (state_dir / "current").read_text(encoding="utf-8").strip() == name
    assert not (state_dir / "previous").exists()
    assert pointers.load(state_dir) == (name, None)
    assert not (compose_root / "current").exists()
    assert not (releases_root / "current").exists()
    assert list(state_dir.glob(".*")) == []


def test_second_commit_moves_old_current_to_previous(tmp_path, world):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    first = world.release
    second = _rename(first, "2026.11.10-1")
    _plant_staged(releases_root, first)
    _plant_staged(releases_root, second)
    data1, sig1 = _signed(world, first)
    data2, sig2 = _signed(world, second)

    pointers.commit(state_dir, releases_root, "vision", data1, sig1, world.pubkey)
    result = pointers.commit(state_dir, releases_root, "vision", data2, sig2, world.pubkey)

    assert result == {"current": "2026.11.10-1", "previous": first["release"]}
    assert (state_dir / "current").read_text(encoding="utf-8").strip() == "2026.11.10-1"
    assert (state_dir / "previous").read_text(encoding="utf-8").strip() == first["release"]
    assert not (state_dir / "current").is_symlink()
    assert not (state_dir / "previous").is_symlink()
    assert not (compose_root / "current").exists()


def test_idempotent_when_already_current(tmp_path, world):
    state_dir, releases_root, _ = _paths(tmp_path)
    _plant_staged(releases_root, world.release)
    data, sig = _signed(world)
    first = pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)
    again = pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)
    assert again == first
    assert not (state_dir / "previous").exists()


def test_bad_signature_does_not_write_pointers(tmp_path, world):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    _plant_staged(releases_root, world.release)
    data, sig = _signed(world)
    with pytest.raises(pointers.PointerError, match="bad_signature"):
        pointers.commit(state_dir, releases_root, "vision", data + b"\n", sig, world.pubkey)
    assert list(state_dir.glob("*")) == []
    assert not (compose_root / "current").exists()


def test_not_staged_does_not_write_pointers(tmp_path, world):
    state_dir, releases_root, _ = _paths(tmp_path)
    data, sig = _signed(world)
    with pytest.raises(pointers.PointerError, match="release_not_staged"):
        pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)
    assert not state_dir.exists() or list(state_dir.glob("*")) == []


def test_refuses_existing_current_symlink(tmp_path, world):
    state_dir, releases_root, compose_root = _paths(tmp_path)
    compose_root.mkdir(parents=True)
    live = compose_root / "docker-compose.yaml"
    live.write_text("stay\n", encoding="utf-8")
    state_dir.mkdir(parents=True)
    (state_dir / "current").symlink_to(compose_root)
    _plant_staged(releases_root, world.release)
    data, sig = _signed(world)

    with pytest.raises(pointers.PointerError, match="compose_symlink_untouched"):
        pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)

    assert (state_dir / "current").is_symlink()
    assert live.read_text(encoding="utf-8") == "stay\n"
    assert not (compose_root / "current").exists()


def test_commit_does_not_open_network(tmp_path, world, monkeypatch):
    def boom(*_a, **_k):
        raise AssertionError("сеть")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    monkeypatch.setattr("socket.create_connection", boom)
    state_dir, releases_root, _ = _paths(tmp_path)
    _plant_staged(releases_root, world.release)
    data, sig = _signed(world)
    result = pointers.commit(state_dir, releases_root, "vision", data, sig, world.pubkey)
    assert result["current"] == world.release["release"]
