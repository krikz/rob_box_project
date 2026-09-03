"""supercollider_synthdefs_count must reflect what scsynth actually holds.

Issue #1809. The metric used to count ``.scsyndef`` files in the shared
volume. That volume is almost always empty or has a handful of files —
sclang pushes definitions into scsynth over OSC (``/d_recv``), it never
writes them to disk. The metric reported ~5 while scsynth actually held
around ~300 SynthDefs (startup preload plus everything Python renardo pushes
later at runtime). The only source of truth is scsynth itself: ``/status``
over OSC to 127.0.0.1:57110 replies with ``numSynthDefs`` as the 5th int in
its ``iiiii`` block (unused, numUGens, numSynths, numGroups, numSynthDefs).

This test does not require a real scsynth. It runs a tiny fake UDP responder
on the real scsynth port (127.0.0.1:57110, loopback-only) that answers with a
hand-built ``/status.reply`` and checks the metrics module decodes it
correctly — including the OSC string-padding edge case that has already bit
this codebase once (commit 8c4079a7): a string whose padded length lands
exactly on a multiple of 4 still needs its null terminator, and the naive
``(4 - len(b) % 4) % 4`` formula silently drops it for such strings.
"""
from __future__ import annotations

import importlib.util as _ilu
import os
import socket
import struct
import sys
import threading
from pathlib import Path

import pytest


def _repo_root(start: Path) -> Path:
    """Locate the rob_box_project repo root, robust to dev vs CI layouts.

    Three layouts we must support:

    1. **Dev / krikz worktree** — test file lives under ``src/rob_box_voice/test/...``.
       Walking up: ``core → unit → test → rob_box_voice → src → <repo>``.
       The ``<repo>`` directory has both ``src/`` and ``docker/`` next to it.

    2. **GitHub Actions ``test_ws``** — CI mirrors ``src/`` into
       ``test_ws/src/`` and ``docker/`` into ``test_ws/docker/`` (see
       ``.github/workflows/G-Run Tests.yml``). Same walk lands at ``test_ws``
       and the parent's ``src/`` + ``docker/`` check matches.

    3. **``colcon test`` build tree** — when integration tests run under
       ``colcon test``, pytest may discover the test file from
       ``test_ws/build/<pkg>/...`` or from ``test_ws/install/...`` where the
       ``src/`` and ``docker/`` siblings are absent. In that case the env
       var ``ROB_BOX_REPO_ROOT`` (set by the workflow) is the only reliable
       anchor.

    Search order:

    * explicit override via ``ROB_BOX_REPO_ROOT`` env var (set in CI);
    * any ancestor whose direct children are both ``src/`` and ``docker/``;
    * any ancestor that contains a ``src/rob_box_voice`` subdir (dev-repo);
    * as a last resort, walk all the way up looking for a sibling pair.

    A hard ``RuntimeError`` is kept ONLY if nothing matches at all — that
    means the file is genuinely outside the repo, which is a configuration
    problem, not a CI vs dev mismatch.
    """
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override:
        candidate = Path(override).expanduser().resolve()
        if (candidate / "src").is_dir():
            return candidate

    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent

    for parent in [start, *start.parents]:
        if (parent / "src" / "rob_box_voice").is_dir():
            return parent

    raise RuntimeError(
        f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT or run "
        "from inside rob_box_project"
    )


REPO_ROOT = _repo_root(Path(__file__).resolve())
_TARGET = str(
    REPO_ROOT / "docker" / "vision" / "scripts" / "supercollider" / "metrics_server.py"
)

_spec = _ilu.spec_from_file_location("supercollider_metrics_server", _TARGET)
metrics = _ilu.module_from_spec(_spec)
sys.modules["supercollider_metrics_server"] = metrics
_spec.loader.exec_module(metrics)


# ── osc_string: the padding bug this codebase already got burned by once ────

@pytest.mark.parametrize(
    "s",
    [
        "",  # 0 chars: terminator alone must still pad to 4
        ",",  # 1 char
        "/status",  # 7 chars -> +1 null = 8, ALREADY a multiple of 4
        "/status.reply",  # 13 chars -> +1 null = 14, needs 2 more
        ",iiiiiffdd",  # 10 chars -> +1 null = 11, needs 1 more
        "abcd",  # 4 chars: the exact case the naive formula breaks on
        "abcdefgh",  # 8 chars: same trap, next multiple
    ],
)
def test_osc_string_always_has_a_terminator_and_is_a_multiple_of_four(s: str) -> None:
    encoded = metrics.osc_string(s)
    assert len(encoded) % 4 == 0, f"len({encoded!r}) not a multiple of 4"
    assert b"\x00" in encoded, f"{encoded!r} has no null terminator at all"
    # The terminator must appear immediately after the string content, i.e.
    # decoding back must round-trip through the null-terminated prefix.
    assert encoded[: len(s)] == s.encode()
    assert encoded[len(s)] == 0


def test_osc_string_keeps_the_terminator_when_already_aligned() -> None:
    """The exact trap that bit commit 8c4079a7: strings whose length is a
    multiple of 4 (here: 7 chars + 1 terminator = 8, already aligned).

    A naive ``(4 - len(b) % 4) % 4`` padding formula computes 0 extra bytes
    for such strings — which is the *correct* padding amount — but a buggy
    variant of that idea (padding based on ``len(s)`` instead of ``len(b)``,
    or forgetting the terminator is unconditional) drops the terminator
    itself here. The right invariant to check is simply: terminator present,
    length unchanged at 8 (no bogus extra padding), multiple of 4.
    """
    encoded = metrics.osc_string("/status")
    assert encoded == b"/status\x00"
    assert len(encoded) == 8
    assert len(encoded) % 4 == 0


def _build_status_reply(
    *, num_ugens: int, num_synths: int, num_groups: int, num_synthdefs: int,
    avg_cpu: float = 1.5, peak_cpu: float = 3.0,
    nominal_sr: float = 16000.0, actual_sr: float = 16000.0,
) -> bytes:
    """Hand-builds a real ``/status.reply`` OSC message for the fake server."""
    address = metrics.osc_string("/status.reply")
    typetags = metrics.osc_string(",iiiiiffdd")
    values = (
        struct.pack(">i", 0)  # unused
        + struct.pack(">i", num_ugens)
        + struct.pack(">i", num_synths)
        + struct.pack(">i", num_groups)
        + struct.pack(">i", num_synthdefs)
        + struct.pack(">f", avg_cpu)
        + struct.pack(">f", peak_cpu)
        + struct.pack(">d", nominal_sr)
        + struct.pack(">d", actual_sr)
    )
    return address + typetags + values


def test_parse_status_reply_reads_numsynthdefs_as_the_fifth_int() -> None:
    reply = _build_status_reply(
        num_ugens=4978, num_synths=540, num_groups=6, num_synthdefs=287,
    )
    values = metrics._parse_status_reply(reply)
    assert values[0] == 0
    assert values[1] == 4978
    assert values[2] == 540
    assert values[3] == 6
    assert values[4] == 287


def test_parse_status_reply_rejects_wrong_address() -> None:
    bogus = metrics.osc_string("/something.else") + metrics.osc_string(",i") + struct.pack(">i", 1)
    with pytest.raises(ValueError):
        metrics._parse_status_reply(bogus)


class _FakeScsynthStatus:
    """Answers exactly one ``/status`` with a canned ``/status.reply``.

    Binds to the real scsynth port on loopback so ``_synthdefs_count()``'s
    hardcoded ``(127.0.0.1, 57110)`` target actually reaches it — this is
    what the function talks to in production too, just a stand-in server.
    """

    def __init__(self, reply: bytes) -> None:
        self._reply = reply
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(2.0)
        self._sock.bind((metrics.SCSYNTH_HOST, metrics.SCSYNTH_PORT))
        self._thread = threading.Thread(target=self._serve_once, daemon=True)

    def _serve_once(self) -> None:
        try:
            _, addr = self._sock.recvfrom(1024)
            self._sock.sendto(self._reply, addr)
        except OSError:
            pass

    def __enter__(self) -> "_FakeScsynthStatus":
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._thread.join(timeout=2.0)
        self._sock.close()


def _scsynth_port_available() -> bool:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((metrics.SCSYNTH_HOST, metrics.SCSYNTH_PORT))
        return True
    except OSError:
        return False
    finally:
        probe.close()


@pytest.mark.skipif(
    not _scsynth_port_available(),
    reason="127.0.0.1:57110 занят (реальный scsynth или другой тест)",
)
def test_synthdefs_count_queries_scsynth_over_osc() -> None:
    reply = _build_status_reply(
        num_ugens=1000, num_synths=100, num_groups=3, num_synthdefs=287,
    )
    with _FakeScsynthStatus(reply):
        assert metrics._synthdefs_count() == 287


@pytest.mark.skipif(
    not _scsynth_port_available(),
    reason="127.0.0.1:57110 занят (реальный scsynth или другой тест)",
)
def test_synthdefs_count_is_zero_when_scsynth_does_not_answer() -> None:
    """No responder bound to the port -> timeout -> honest 0, not a crash."""
    assert metrics._synthdefs_count() == 0


def test_synthdefs_count_no_longer_reads_the_synthdefs_directory() -> None:
    """Regression guard: the old file-count approach must be fully gone.

    That approach undercounted by two orders of magnitude (issue #1809)
    because scsynth never writes compiled defs to disk in this pipeline.
    """
    assert not hasattr(metrics, "SYNTHDEFS_DIR"), (
        "SYNTHDEFS_DIR всё ещё существует — похоже, вернулся файловый метод "
        "подсчёта, который и был причиной issue #1809"
    )
