"""Regression tests for kanban task ``t_7f77015e``.

Verifies that ``tts_node.py`` contains **zero live**
``threading.Thread(target=..., daemon=True)`` constructs. The previous PR
#907 BLK-9 review landed a bounded ``ThreadPoolExecutor`` for synthesis
fan-out. This test enforces the post-fix invariant so any future
regression is caught immediately.

Acceptance criteria covered here:

  * No line outside a comment/string calls ``threading.Thread(...)``.
  * The bounded ``ThreadPoolExecutor`` constants for the synth executor
    exist at module scope with documented sizing.

PR-15 (#3711, part 15p): the v1 dialogue node (asyncio-loop driver checks)
was deleted together with its tests.
"""

from __future__ import annotations

import io
import tokenize
from pathlib import Path

import pytest


# Resolve the file under test without needing a working rclpy.
_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TARGETS = [
    _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py",
]


def _strip_strings_and_comments(source: str) -> str:
    """Return *source* with comments and string literals blanked out.

    We blank rather than strip so line numbers in the residual text still
    align with the original file — the assertion message includes the
    offending line number.

    PR-15 (#3711, part 15-core): the spans come from ``tokenize``, not from a
    regex over the whole file. The old regex paired quote characters across
    comments and docstrings, so ONE apostrophe added to or removed from any
    comment above shifted every later "string" and either hid or exposed a
    ``daemon=True`` marker far below (the file passed only by accident of
    quote parity).
    """
    lines = source.splitlines(keepends=True)
    grid = [list(line) for line in lines]
    fstring_middle = getattr(tokenize, "FSTRING_MIDDLE", None)  # Python 3.12+
    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        if tok.type not in (tokenize.STRING, tokenize.COMMENT) and tok.type != fstring_middle:
            continue
        (row1, col1), (row2, col2) = tok.start, tok.end
        for row in range(row1, row2 + 1):
            cells = grid[row - 1]
            lo = col1 if row == row1 else 0
            hi = col2 if row == row2 else len(cells)
            for col in range(lo, min(hi, len(cells))):
                if cells[col] not in "\r\n":
                    cells[col] = " "
    return "".join("".join(cells) for cells in grid)


def _live_substring_matches(source: str, pattern: str) -> list[tuple[int, str]]:
    """Return [(lineno, line_text), ...] for non-comment, non-string.
    occurrences of *pattern* in *source*."""
    cleaned = _strip_strings_and_comments(source)
    hits: list[tuple[int, str]] = []
    for lineno, line in enumerate(cleaned.splitlines(), start=1):
        if pattern in line:
            hits.append((lineno, line.rstrip()))
    return hits


# ── Per-file fixtures ───────────────────────────────────────────────────────


@pytest.fixture(params=_TARGETS, ids=lambda p: p.name)
def target_file(request) -> Path:
    return request.param


# ── Acceptance: zero live ``threading.Thread(...)`` calls ───────────────────


def test_no_live_threading_thread_construction(target_file: Path) -> None:
    """The exact offender from PR #907 review must be gone (no live spawn)."""
    src = target_file.read_text(encoding="utf-8")
    offenders = _live_substring_matches(src, "threading.Thread(")
    assert not offenders, (
        f"{target_file.name} still contains live `threading.Thread(...)` "
        f"spawns (must use a bounded ``ThreadPoolExecutor`` instead). "
        f"Found: {offenders}"
    )


def test_no_live_daemon_true_in_code(target_file: Path) -> None:
    """No code path sets ``daemon=True`` (only historical comments remain)."""
    src = target_file.read_text(encoding="utf-8")
    cleaned = _strip_strings_and_comments(src)
    offenders = [
        (lineno, line)
        for lineno, line in enumerate(cleaned.splitlines(), start=1)
        if "daemon=True" in line or "daemon = True" in line
    ]
    assert not offenders, (
        f"{target_file.name} still has live `daemon=True` usage: {offenders}"
    )


# ── Acceptance: named-constant executor sizing ──────────────────────────────


def test_tts_node_synthesis_constants_present() -> None:
    """tts_node must export the module-level synthesis sizing constants."""
    src = (_PACKAGE_ROOT / "rob_box_voice" / "tts_node.py").read_text(
        encoding="utf-8"
    )
    for name in (
        "SYNTHESIS_MAX_WORKERS_DEFAULT",
        "SYNTHESIS_MAX_QUEUE_DEFAULT",
        "SYNTHESIS_SHUTDOWN_TIMEOUT_S",
        "SYNTHESIS_THREAD_NAME_PREFIX",
        "ASYNC_BRIDGE_MAX_WORKERS",
    ):
        assert name in src, f"Missing module-level constant {name!r} in tts_node.py"
