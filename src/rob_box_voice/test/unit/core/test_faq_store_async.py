"""Unit tests for FAQStore async search/replace (PF-3 async embedding path)."""

from __future__ import annotations

import asyncio
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from rob_box_voice.core.faq_store import FAQStore


@pytest.fixture
def store(tmp_path: Path) -> FAQStore:
    return FAQStore(db_path=str(tmp_path / "faq.db"))


class TestFAQStoreAsync:
    def test_asearch_returns_fts_when_ollama_unavailable(self, store) -> None:
        store.replace_items(
            event_id="test",
            items=[
                {
                    "question": "Где проходит день открытых дверей?",
                    "answer": "В главном корпусе.",
                    "category": "general",
                    "source": "faq.xlsx",
                }
            ],
        )
        store.embedder = MagicMock()
        store.embedder.is_available.return_value = False

        results = asyncio.run(
            store.asearch("день открытых дверей", event_id="test", limit=3)
        )

        assert len(results) == 1
        assert results[0]["source_type"] == "fts"

    def test_asearch_uses_vector_when_sparse(self, store) -> None:
        store.replace_items(
            event_id="test",
            items=[
                {
                    "question": "Где проходит день открытых дверей?",
                    "answer": "В главном корпусе.",
                    "category": "general",
                    "source": "faq.xlsx",
                }
            ],
        )
        store.embedder = MagicMock()
        store.embedder.is_available.return_value = True
        store.embedder.aembed = AsyncMock(return_value=[0.1, 0.2, 0.3])
        store._has_vec_table = MagicMock(return_value=True)
        store._get_meta_int = MagicMock(return_value=3)

        results = asyncio.run(
            store.asearch("что-то семантическое", event_id="test", limit=3)
        )

        store.embedder.aembed.assert_awaited_once()
        assert isinstance(results, list)

    def test_asearch_empty_query_returns_empty(self, store) -> None:
        store.embedder = MagicMock()
        assert asyncio.run(store.asearch("   ", event_id="test")) == []

    def test_areplace_items_embeds_each_item(self, store) -> None:
        store.embedder = MagicMock()
        store.embedder.aembed = AsyncMock(return_value=[0.1, 0.2])
        store._has_vec_table = MagicMock(return_value=True)
        store._get_meta_int = MagicMock(return_value=2)

        inserted = asyncio.run(
            store.areplace_items(
                event_id="open-day-2026",
                items=[
                    {
                        "question": "Вопрос один?",
                        "answer": "Ответ один.",
                        "category": "general",
                    },
                    {
                        "question": "Вопрос два?",
                        "answer": "Ответ два.",
                        "category": "general",
                    },
                ],
            )
        )

        assert inserted == 2
        assert store.embedder.aembed.await_count == 2

    def test_areplace_items_requires_event_id(self, store) -> None:
        with pytest.raises(ValueError):
            asyncio.run(store.areplace_items("", [{"question": "q", "answer": "a"}]))


class TestFAQStoreAsyncWithPatch:
    def test_asearch_never_blocks_with_sync_httpx(
        self, store, monkeypatch
    ) -> None:
        """Guard: asearch must never call the blocking sync embedder."""
        store.replace_items(
            event_id="test",
            items=[
                {
                    "question": "Какой сегодня день?",
                    "answer": "Пятница.",
                    "category": "general",
                    "source": "faq.xlsx",
                }
            ],
        )
        store.embedder = MagicMock()
        store.embedder.is_available.return_value = True
        store.embedder.aembed = AsyncMock(return_value=[0.1, 0.2, 0.3])
        # If asearch called sync embed(), this raises and the test fails.
        store.embedder.embed = MagicMock(side_effect=AssertionError("sync embed called!"))
        store._has_vec_table = MagicMock(return_value=True)
        store._get_meta_int = MagicMock(return_value=3)

        results = asyncio.run(
            store.asearch("пятница", event_id="test", limit=3)
        )

        assert isinstance(results, list)
        store.embedder.embed.assert_not_called()
