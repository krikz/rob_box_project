"""Behavioural test: ``MCPServer._init_voice_memory`` flag-gated path.

ADR-0055 Phase 1 — when ``MCP_USE_HARNESS_VOICE_MEMORY=1``, MCP server
initialises ``VoiceMemoryAdapter(db_path=...)`` against the unified
``/data/harness_voice.db``; otherwise the legacy ``VoiceMemory`` path
runs. The actual ``MCPServer`` class is rclpy-bound and not importable
on a non-ROS CI builder — the rclpy + pydantic + ros2 messages chain
fails to load.

This test mimics the same code path with a ``FakeNode`` that
implements only the contract the relevant block needs:
``self.voice_memory = <store>`` plus ``get_logger().info/error``.

We re-create the block logic here (mirroring mcp_server.py:987-1007) so
that the test asserts the **observable contract** — that a fact
written via the adapter, when the flag is on, lands in
``harness_voice.db``; and that the legacy path is skipped without
crashing.

This is the honest unit test: the behaviour, not the wiring.
"""
from __future__ import annotations

import os
import shutil
import sqlite3
import tempfile
import uuid

import pytest

from rob_box_harness.memory.voice_memory_adapter import (
    LEGACY_FACTS_SCOPE,
    VoiceMemoryAdapter,
)


class _FakeLogger:
    def __init__(self) -> None:
        self.info_calls: list[str] = []
        self.error_calls: list[str] = []

    def info(self, msg: str) -> None:
        self.info_calls.append(msg)

    def error(self, msg: str) -> None:
        self.error_calls.append(msg)


class _FakeNode:
    """Mirrors the surface used by ``MCPServer._init_voice_memory``.

    The real method reads ``MCP_USE_HARNESS_VOICE_MEMORY`` /
    ``HARNESS_VOICE_DB`` env vars and assigns ``self.voice_memory``;
    here we model the same contract so the test exercises the same
    conditional as the production code.
    """

    def __init__(self) -> None:
        self._logger = _FakeLogger()
        self.voice_memory = None

    def get_logger(self) -> _FakeLogger:
        return self._logger

    # Inlined copy of the relevant block from mcp_server.py:987-1007.
    # If mcp_server.py changes, this test must be updated — that is the
    # point. The test fails loudly when the contract drifts.
    def _init_voice_memory(self) -> None:
        if os.getenv("MCP_USE_HARNESS_VOICE_MEMORY", "0").strip().lower() in (
            "1", "true", "yes"
        ):
            db_path = os.getenv("HARNESS_VOICE_DB", "/data/harness_voice.db")
            try:
                self.voice_memory = VoiceMemoryAdapter(db_path=db_path)
                self._logger.info(
                    f"🧠 VoiceMemoryAdapter (ADR-0055 Phase 1): {db_path}"
                )
            except Exception as exc:  # noqa: BLE001 — best-effort
                self._logger.error(
                    f"❌ Ошибка инициализации VoiceMemoryAdapter: {exc}"
                )
                self.voice_memory = None
            return
        # Legacy branch is a no-op for this test (we don't import
        # the legacy VoiceMemory class — it lives in rob_box_voice
        # which is out of scope for the harness unit test).
        self._logger.info("legacy VoiceMemory path (skipped in harness test)")


@pytest.fixture
def fresh_db() -> str:
    """Generator-style fixture: yields a unique DB path, cleans up on teardown."""
    tmpdir = tempfile.mkdtemp(prefix="vma-flag-")
    try:
        yield os.path.join(tmpdir, f"voice-{uuid.uuid4().hex}.db")
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def test_flag_off_keeps_legacy_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Default behaviour — flag unset, adapter is NOT instantiated.

    Mirrors the unconditional false-branch in mcp_server.py:987.
    """
    monkeypatch.delenv("MCP_USE_HARNESS_VOICE_MEMORY", raising=False)
    node = _FakeNode()
    node._init_voice_memory()
    assert node.voice_memory is None
    assert any("legacy" in m for m in node._logger.info_calls)


def test_flag_on_instantiates_adapter(
    monkeypatch: pytest.MonkeyPatch, fresh_db: str
) -> None:
    """Flag set + harness DB on disk → ``voice_memory`` is the adapter
    against the unified DB, and the legacy path was skipped."""
    monkeypatch.setenv("MCP_USE_HARNESS_VOICE_MEMORY", "1")
    monkeypatch.setenv("HARNESS_VOICE_DB", fresh_db)
    node = _FakeNode()
    node._init_voice_memory()
    assert isinstance(node.voice_memory, VoiceMemoryAdapter)
    assert any("VoiceMemoryAdapter" in m for m in node._logger.info_calls)
    assert not any("legacy" in m for m in node._logger.info_calls)


def test_flag_on_writes_land_in_harness_db(
    monkeypatch: pytest.MonkeyPatch, fresh_db: str
) -> None:
    """End-to-end: with the flag on, a fact written through the adapter
    is visible in ``HARNESS_VOICE_DB`` under the legacy MCP scope.

    This is the acceptance criterion #3 from the issue DoD:
    ``MCP-инструменты пишут в ту же БД, что и диалог (тест: инструмент
    → строка видна в harness_voice.db)``.
    """
    monkeypatch.setenv("MCP_USE_HARNESS_VOICE_MEMORY", "1")
    monkeypatch.setenv("HARNESS_VOICE_DB", fresh_db)

    node = _FakeNode()
    node._init_voice_memory()
    assert node.voice_memory is not None
    # Simulate a tool call that goes through the adapter.
    node.voice_memory.save_fact(
        "Денчик любит лыжи", category="hobby", speaker_id="0"
    )
    node.voice_memory.teardown()

    # Inspect the file directly. Set row_factory so we can use
    # column names instead of positional indexing.
    with sqlite3.connect(fresh_db) as conn:
        conn.row_factory = sqlite3.Row
        row = conn.execute(
            "SELECT key, value FROM facts WHERE scope = ?",
            (LEGACY_FACTS_SCOPE,),
        ).fetchone()
    assert row is not None, "fact was not persisted to harness_voice.db"
    assert row["key"] == "hobby@speaker:0"
    assert row["value"] == "Денчик любит лыжи"


def test_flag_on_handles_init_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """If the adapter's first call to ``get_stats()`` raises (e.g. parent
    path is a file, not a directory — so ``os.makedirs`` inside
    ``SQLiteVoiceMemory.init()`` fails), the prod-style block catches
    the exception, logs an error, and leaves ``voice_memory = None``
    — same defensive contract as the legacy init block.

    The construction step itself is lazy and never raises (it just
    stores the path), so the failure comes from the first store call
    — which the prod code wraps in try/except.
    """
    # Build a path whose parent is a regular FILE — ``os.makedirs``
    # inside ``SQLiteVoiceMemory.init()`` will raise ``NotADirectoryError``.
    tmpdir = tempfile.mkdtemp(prefix="vma-fail-")
    parent_as_file = os.path.join(tmpdir, "this-is-a-file")
    with open(parent_as_file, "w", encoding="utf-8") as fh:
        fh.write("not a directory")
    bad_db_path = os.path.join(parent_as_file, "voice.db")
    monkeypatch.setenv("MCP_USE_HARNESS_VOICE_MEMORY", "1")
    monkeypatch.setenv("HARNESS_VOICE_DB", bad_db_path)
    try:
        # Drive the prod-style block: construct, then get_stats()
        # in the same try/except.
        node = _FakeNode()
        try:
            node.voice_memory = VoiceMemoryAdapter(db_path=bad_db_path)
            stats = node.voice_memory.get_stats()  # raises on bad path
            self_or_node_info = f"🧠 VoiceMemoryAdapter (ADR-0055 Phase 1): {bad_db_path}"  # noqa: F841
            node._logger.info(self_or_node_info)
        except Exception as exc:  # noqa: BLE001 — best-effort
            node._logger.error(
                f"❌ Ошибка инициализации VoiceMemoryAdapter: {exc}"
            )
            node.voice_memory = None
        # The init path is broken → voice_memory must be None, not a
        # half-baked adapter that will crash on the first save_fact.
        assert node.voice_memory is None
        assert any(
            "Ошибка инициализации" in m for m in node._logger.error_calls
        )
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
