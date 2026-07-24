"""Regression tests for kanban task ``t_7f77015e``.

Verifies that ``tts_node.py`` and ``dialogue_node.py`` contain **zero
live** ``threading.Thread(target=..., daemon=True)`` constructs. The
previous PR #907 BLK-9 review landed a bounded ``ThreadPoolExecutor``
for synthesis fan-out, but left a bare ``daemon=True`` thread for the
asyncio loop driver in ``dialogue_node.__init__``. This test enforces
the post-fix invariant so any future regression is caught immediately.

Acceptance criteria covered here:

  * No line outside a comment/string in either file calls
    ``threading.Thread(...)``.
  * The bounded ``ThreadPoolExecutor`` constants for the synth executor
    and the asyncio-loop driver exist at module scope with documented
    sizing.
  * The asyncio-loop driver is hosted via the bounded
    ``ThreadPoolExecutor`` (``ASYNCIO_LOOP_DRIVER_MAX_WORKERS``), not
    a bare daemon thread, and the node exposes a ``destroy_node`` /
    ``shutdown_asyncio_loop`` hook to drain the worker.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


# Resolve both files under test without needing a working rclpy.
_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TARGETS = [
    _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py",
    _PACKAGE_ROOT / "rob_box_voice" / "dialogue_node.py",
]


def _strip_strings_and_comments(source: str) -> str:
    """Return *source* with comments and string literals blanked out.

    We blank rather than strip so line numbers in the residual text still
    align with the original file — the assertion message includes the
    offending line number.
    """
    # Triple-quoted strings (incl. f-strings) — non-greedy across lines.
    no_triple = re.sub(
        r'("[^"\\]*(?:\\.[^"\\]*)*"|\'[^\'\\]*(?:\\.[^\'\\]*)*\')',
        lambda m: " " * len(m.group(0)),
        source,
    )
    # Single-line comments.
    no_comments = re.sub(r"(?m)#.*$", lambda m: " " * len(m.group(0)), no_triple)
    return no_comments


def _live_substring_matches(source: str, pattern: str) -> list[tuple[int, str]]:
    """Return [(lineno, line_text), ...] for non-comment, non-string
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


def test_dialogue_node_asyncio_loop_constants_present() -> None:
    """dialogue_node must export the module-level asyncio loop sizing constants."""
    src = (_PACKAGE_ROOT / "rob_box_voice" / "dialogue_node.py").read_text(
        encoding="utf-8"
    )
    for name, expected in (
        ("ASYNCIO_LOOP_DRIVER_MAX_WORKERS", "1"),
        ("ASYNCIO_LOOP_DRIVER_NAME_PREFIX", "dialogue-async-loop"),
        ("ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S", None),
    ):
        assert name in src, f"Missing module-level constant {name!r} in dialogue_node.py"

    # The constant must literally be the int 1 (not a magic-number copy).
    m = re.search(
        r"^ASYNCIO_LOOP_DRIVER_MAX_WORKERS\s*:\s*int\s*=\s*(\d+)\s*$",
        src,
        re.MULTILINE,
    )
    assert m is not None, "ASYNCIO_LOOP_DRIVER_MAX_WORKERS must be defined as int"
    assert m.group(1) == "1", (
        "ASYNCIO_LOOP_DRIVER_MAX_WORKERS must stay at 1 — the asyncio loop "
        "is single-threaded by contract; a larger pool would just create "
        "idle workers."
    )


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


def test_dialogue_node_loop_is_bounded_executor() -> None:
    """The asyncio loop driver must use a bounded ThreadPoolExecutor.

    This is the structural fix for the bare ``daemon=True`` thread. We
    inspect the ``DialogueNode.__init__`` body via AST so the test does
    not need the full ``dialogue_node`` module to be importable (it
    pulls in ``rclpy`` + ``agents`` + ``openai`` which may not be present
    in the test environment).
    """
    import ast

    src = (_PACKAGE_ROOT / "rob_box_voice" / "dialogue_node.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(src)

    # Locate the DialogueNode class.
    cls = next(
        (n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DialogueNode"),
        None,
    )
    assert cls is not None, "DialogueNode class definition not found"

    init_method = next(
        (n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__"),
        None,
    )
    assert init_method is not None, "DialogueNode.__init__ not found"

    init_src = ast.unparse(init_method)
    assert "ThreadPoolExecutor" in init_src, (
        "DialogueNode.__init__ must host the asyncio loop via "
        "concurrent.futures.ThreadPoolExecutor (bounded primitive)."
    )
    assert "ASYNCIO_LOOP_DRIVER_MAX_WORKERS" in init_src, (
        "DialogueNode.__init__ must reference the module-level "
        "ASYNCIO_LOOP_DRIVER_MAX_WORKERS constant for sizing."
    )
    assert "threading.Thread(" not in _strip_strings_and_comments(init_src), (
        "DialogueNode.__init__ must not contain a live threading.Thread(...) call."
    )


def test_dialogue_node_exposes_shutdown_hook() -> None:
    """DialogueNode must expose a controlled shutdown for the asyncio loop."""
    import ast

    src = (_PACKAGE_ROOT / "rob_box_voice" / "dialogue_node.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(src)

    cls = next(
        (n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DialogueNode"),
        None,
    )
    assert cls is not None, "DialogueNode class definition not found"

    method_names = {
        n.name for n in cls.body if isinstance(n, ast.FunctionDef)
    }
    assert "shutdown_asyncio_loop" in method_names, (
        "DialogueNode must define `shutdown_asyncio_loop()` so the bounded "
        "executor can be drained instead of relying on daemon=True."
    )
    assert "destroy_node" in method_names, (
        "DialogueNode must override destroy_node() to invoke the shutdown hook."
    )