"""Хост Химеры отказывает честно, пока пина нет (карточка #5205, ADR-0175 §2.1).

Текста песни здесь нет: проверяется только пин, тембр и сборка команды на обезличенном имени файла.
"""

import os

import pytest

from rob_box_mcp_tools.chimera_host.invoke import (
    PINNED_SHA,
    ChimeraHost,
    ChimeraUnavailable,
    pinned_inputs,
    sha256_file,
)

STUB = "заглушка"  # не текст песни: имя файла, которого нет


def test_missing_host_refuses(tmp_path):
    host = ChimeraHost(str(tmp_path))
    assert host.refusal() == "missing_binary"
    with pytest.raises(ChimeraUnavailable) as caught:
        host.notes_command(STUB, "out")
    assert caught.value.reason == "missing_binary"


def test_wrong_sha_refuses(tmp_path):
    (tmp_path / "chimera").write_bytes(b"not-the-binary")
    (tmp_path / "SOURCE_SHA").write_text("deadbeef\n", encoding="utf-8")
    assert ChimeraHost(str(tmp_path)).refusal() == "sha_mismatch"


def test_pinned_host_builds_command(tmp_path):
    binary = tmp_path / "chimera"
    binary.write_bytes(b"binary-bytes")
    (tmp_path / "SOURCE_SHA").write_text(PINNED_SHA + "\n", encoding="utf-8")
    data = tmp_path / "data"
    data.mkdir()
    (data / "cmudict.txt.gz").write_bytes(b"dict")
    host = ChimeraHost(str(tmp_path))
    assert host.refusal() == ""
    assert host.notes_command("notes.json", "out/song", "male") == (
        str(binary),
        "render",
        "notes",
        "notes.json",
        "out/song",
        "male",
    )
    assert host.data_mount() == (str(data), "/src/public/data")


def test_unknown_timbre_refuses(tmp_path):
    (tmp_path / "chimera").write_bytes(b"x")
    (tmp_path / "SOURCE_SHA").write_text(PINNED_SHA, encoding="utf-8")
    (tmp_path / "data").mkdir()
    (tmp_path / "data" / "cmudict.txt.gz").write_bytes(b"d")
    with pytest.raises(ChimeraUnavailable) as caught:
        ChimeraHost(str(tmp_path)).notes_command(STUB, "out", "robot")
    assert caught.value.reason == "unsupported_timbre"


def test_sha256_of_present_file(tmp_path):
    path = tmp_path / "chimera"
    path.write_bytes(b"binary-bytes")
    assert (
        sha256_file(str(path))
        == "71227a7f160afca3fb3c39f448735886dda7bd366252580c2222fb87d4bb4d85"
    )
    rows = dict(pinned_inputs(str(tmp_path)))
    assert rows["chimera"] == sha256_file(str(path))
    assert rows[os.path.join("data", "cmudict.txt.gz")] == ""
