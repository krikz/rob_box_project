"""Tests for scripts/e2e/archive_e2e_artifacts.py (issue #4512): gh is faked."""

from __future__ import annotations

import importlib.util
import io
import json
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location("archive_e2e_artifacts", REPO / "scripts/e2e/archive_e2e_artifacts.py")
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)

NOSLEEP = lambda _s: None  # noqa: E731


def art(id_, name, created="2026-11-06T10:00:00Z", expired=False, run=100):
    return {
        "id": id_,
        "name": name,
        "expired": expired,
        "created_at": created,
        "expires_at": "2026-11-20T10:00:00Z",
        "size_in_bytes": 1048576,
        "workflow_run": {"id": run, "head_branch": "develop", "head_sha": "abc"},
    }


def make_zip() -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("log.txt", "привет")
    return buf.getvalue()


def test_name_filter_excludes_recording_expired_and_old():
    assert mod.wanted(art(1, "e2e-voice-logs-123"), None)
    for n in ("artifacts", "timing", "model", "diff"):
        assert mod.wanted(art(1, f"e2e-voice-{n}-5"), None)
    assert not mod.wanted(art(1, "e2e-voice-recording-123"), None)
    assert not mod.wanted(art(1, "e2e-voice-logs-abc"), None)
    assert not mod.wanted(art(1, "e2e-voice-logs-1", expired=True), None)
    assert not mod.wanted(art(1, "e2e-voice-logs-1", created="2026-11-01T00:00:00Z"), "2026-11-05")
    assert mod.wanted(art(1, "e2e-voice-logs-1", created="2026-11-05T00:00:00Z"), "2026-11-05")


def test_list_retries_broken_page_and_stops_on_empty():
    calls = {"n": 0}
    pages = {
        1: {"artifacts": [art(1, "e2e-voice-logs-1"), art(2, "e2e-voice-recording-1")]},
        2: {"artifacts": [art(3, "e2e-voice-timing-1")]},
        3: {"artifacts": []},
    }

    def gh(args):
        calls["n"] += 1
        if calls["n"] == 1:
            return b""  # unexpected end of JSON input
        page = int(args[1].rsplit("page=", 1)[1])
        return json.dumps(pages[page]).encode()

    got = mod.list_artifacts(gh, None, NOSLEEP)
    assert [a["id"] for a in got] == [1, 3]
    assert calls["n"] == 4


def test_download_extracts_writes_meta_then_skips(tmp_path):
    a = art(7, "e2e-voice-logs-100")
    gh = lambda args: make_zip()  # noqa: E731
    assert mod.download(gh, a, tmp_path, NOSLEEP) == "ok"
    d = tmp_path / "100" / "e2e-voice-logs-100"
    assert (d / "log.txt").read_text(encoding="utf-8") == "привет"
    meta = json.loads((d / "_artifact.json").read_text(encoding="utf-8"))
    assert meta["id"] == 7 and meta["run_id"] == 100 and meta["head_sha"] == "abc"

    def boom(args):
        raise AssertionError("не должно качать повторно")

    assert mod.download(boom, a, tmp_path, NOSLEEP) == "skip"


def test_non_zip_counts_as_fail_and_exit_code(tmp_path, capsys):
    a = art(8, "e2e-voice-logs-100")

    def gh(args):
        if "zip" in args[1]:
            return b"<html>oops"
        return json.dumps({"artifacts": [a]} if "&page=1" in args[1] else {"artifacts": []}).encode()

    assert mod.archive(gh, tmp_path, None, False, NOSLEEP) == 1
    assert "ok 0 skip 0 fail 1" in capsys.readouterr().out


def test_dry_run_prints_total_and_downloads_nothing(tmp_path, capsys):
    arts = [art(1, "e2e-voice-logs-1"), art(2, "e2e-voice-model-1")]

    def gh(args):
        assert "zip" not in args[1]
        return json.dumps({"artifacts": arts} if "&page=1" in args[1] else {"artifacts": []}).encode()

    assert mod.archive(gh, tmp_path, None, True, NOSLEEP) == 0
    assert "2.0 МБ" in capsys.readouterr().out
    assert not any(tmp_path.iterdir())
