"""Unit tests for the async embedding path of :mod:`rob_box_voice.core.voice_memory`.

Covers the PF-3 refactor: ``OllamaEmbedder.aembed`` (non-blocking HTTP) and the
async store methods (``asave_turn``, ``asearch``, ``aget_context``,
``areindex_embeddings``) that await it instead of blocking the ROS event loop.

NOTE: rob_box_voice CI (colcon) may not ship pytest-asyncio, so async tests use
``asyncio.run()`` explicitly (same convention as the harness package).
"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from rob_box_voice.core.voice_memory import OllamaEmbedder, VoiceMemory


# ---------------------------------------------------------------------------
# OllamaEmbedder.aembed
# ---------------------------------------------------------------------------


class TestOllamaEmbedderAembed:
    def _client_ctx(self) -> AsyncMock:
        """Build an AsyncMock usable as ``httpx.AsyncClient`` context manager."""
        fake_client = AsyncMock()
        fake_client.__aenter__.return_value = fake_client
        return fake_client

    def test_aembed_returns_vector_on_success(self) -> None:
        embedder = OllamaEmbedder(base_url="http://ollama:11434")
        fake_resp = MagicMock()
        fake_resp.json.return_value = {"embedding": [0.1, 0.2, 0.3]}
        fake_client = self._client_ctx()
        fake_client.post.return_value = fake_resp

        with patch("httpx.AsyncClient", return_value=fake_client):
            vec = asyncio.run(embedder.aembed("привет"))

        assert vec == [0.1, 0.2, 0.3]
        assert embedder.is_available() is True
        assert embedder.dim == 3
        fake_client.post.assert_awaited_once()
        fake_client.__aenter__.assert_awaited_once()
        fake_client.__aexit__.assert_awaited_once()

    def test_aembed_returns_none_on_error_and_sets_backoff(self) -> None:
        embedder = OllamaEmbedder(
            base_url="http://ollama:11434", retry_interval=60.0
        )
        fake_client = self._client_ctx()
        fake_client.post.side_effect = RuntimeError("connection refused")

        with patch("httpx.AsyncClient", return_value=fake_client):
            vec = asyncio.run(embedder.aembed("привет"))

        assert vec is None
        assert embedder.is_available() is False
        # Immediately after failure we are in backoff → next call skips HTTP.
        with patch("httpx.AsyncClient", return_value=fake_client) as m:
            vec2 = asyncio.run(embedder.aembed("ещё раз"))
        assert vec2 is None
        assert m.call_count == 0  # no new HTTP client created

    def test_aembed_respects_backoff_window(self) -> None:
        embedder = OllamaEmbedder(
            base_url="http://ollama:11434", retry_interval=0.0
        )
        fake_client = self._client_ctx()
        fake_client.post.side_effect = RuntimeError("boom")

        with patch("httpx.AsyncClient", return_value=fake_client):
            assert asyncio.run(embedder.aembed("x")) is None
            # retry_interval=0 → not in backoff → tries HTTP again
            assert asyncio.run(embedder.aembed("x")) is None
        assert fake_client.post.await_count == 2


# ---------------------------------------------------------------------------
# VoiceMemory async methods
# ---------------------------------------------------------------------------


@pytest.fixture
def memory(tmp_path) -> VoiceMemory:
    return VoiceMemory(db_path=str(tmp_path / "voice.db"))


class TestVoiceMemoryAsync:
    def test_asave_turn_persists_row_and_embeds(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.aembed = AsyncMock(return_value=[0.5, 0.5])
        memory._has_vec_table = MagicMock(return_value=True)
        memory._ensure_vec_table = MagicMock(return_value=True)
        memory._get_meta_int = MagicMock(return_value=2)

        rowid = asyncio.run(memory.asave_turn("user", "привет"))

        assert rowid > 0
        memory.embedder.aembed.assert_awaited_once_with("привет")
        rows = memory.load_recent_turns(limit=5)
        assert rows and rows[-1]["content"] == "привет"

    def test_asave_turn_returns_minus1_for_empty(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.aembed = AsyncMock()
        assert asyncio.run(memory.asave_turn("user", "   ")) == -1
        memory.embedder.aembed.assert_not_awaited()

    def test_asave_turn_embeds_none_when_ollama_down(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.aembed = AsyncMock(return_value=None)

        rowid = asyncio.run(memory.asave_turn("assistant", "ответ"))

        assert rowid > 0  # turn still saved even without embedding
        memory.embedder.aembed.assert_awaited_once_with("ответ")

    def test_asearch_uses_async_vector_search_when_sparse(
        self, memory
    ) -> None:
        memory.embedder = MagicMock()
        memory.embedder.is_available.return_value = True
        memory.embedder.aembed = AsyncMock(return_value=[0.1, 0.2])
        memory._has_vec_table = MagicMock(return_value=True)
        memory._get_meta_int = MagicMock(return_value=2)

        asyncio.run(memory.asave_turn("user", "про кухню"))
        results = asyncio.run(memory.asearch("кухня"))

        memory.embedder.aembed.assert_awaited()
        assert isinstance(results, list)

    def test_aget_context_uses_asearch_with_query(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.is_available.return_value = True
        memory.embedder.aembed = AsyncMock(return_value=[0.1, 0.2])
        memory._has_vec_table = MagicMock(return_value=True)
        memory._get_meta_int = MagicMock(return_value=2)

        asyncio.run(memory.asave_turn("user", "про кухню"))
        ctx = asyncio.run(memory.aget_context(limit=10, query="кухня"))

        assert "recent_turns" in ctx
        assert ctx["total_turns"] == 1

    def test_areindex_embeddings_indexes_all_turns(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.is_available.return_value = True
        memory.embedder.aembed = AsyncMock(return_value=[0.1, 0.2])
        memory._has_vec_table = MagicMock(return_value=True)
        memory._get_meta_int = MagicMock(return_value=2)

        asyncio.run(memory.asave_turn("user", "один"))
        asyncio.run(memory.asave_turn("assistant", "два"))

        stats = asyncio.run(memory.areindex_embeddings())

        assert stats["indexed"] == 2
        assert stats["total"] == 2
        assert stats["dim"] == 2

    def test_areindex_embeddings_reports_unavailable(self, memory) -> None:
        memory.embedder = MagicMock()
        memory.embedder.is_available.return_value = False

        stats = asyncio.run(memory.areindex_embeddings())

        assert stats["indexed"] == 0
        assert "error" in stats
