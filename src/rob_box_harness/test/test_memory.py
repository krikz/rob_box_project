"""Tests for the in-memory MemoryStore and the Turn / Fact value objects.

Covers:

* ``append_turn`` keeps an ordered history per scope.
* ``load_recent`` returns the most recent turns in newest-first order.
* ``save_fact`` is idempotent on the fact key.
* ``search_facts`` ranks by token overlap with ``tags`` and ``key``.
* Invalid inputs (empty query, non-positive limits) are rejected.
"""

from __future__ import annotations

import pytest

from rob_box_harness.memory import Fact, InMemoryStore, MemoryStore, Turn


@pytest.mark.asyncio
async def test_append_and_load_recent() -> None:
    """Appended turns appear in ``load_recent`` (newest first)."""
    store = InMemoryStore()
    await store.append_turn("user:1", Turn(role="user", content="hi"))
    await store.append_turn(
        "user:1", Turn(role="assistant", content="hello there")
    )
    recent = await store.load_recent("user:1", limit=10)
    assert [t.role for t in recent] == ["assistant", "user"]
    assert recent[0].content == "hello there"


@pytest.mark.asyncio
async def test_load_recent_respects_limit() -> None:
    """``limit`` caps the number of returned turns."""
    store = InMemoryStore()
    for i in range(5):
        await store.append_turn(
            "scope", Turn(role="user", content=f"msg-{i}")
        )
    recent = await store.load_recent("scope", limit=2)
    assert len(recent) == 2
    # Newest first: msg-4, msg-3
    assert recent[0].content == "msg-4"
    assert recent[1].content == "msg-3"


@pytest.mark.asyncio
async def test_load_recent_rejects_zero_limit() -> None:
    """``limit`` must be positive."""
    store = InMemoryStore()
    with pytest.raises(ValueError, match="limit must be positive"):
        await store.load_recent("x", limit=0)


@pytest.mark.asyncio
async def test_save_fact_is_idempotent() -> None:
    """Saving a fact with the same key replaces the prior one."""
    store = InMemoryStore()
    await store.save_fact("user:1", Fact(key="genre", value="jazz"))
    await store.save_fact("user:1", Fact(key="genre", value="rock"))
    facts = await store.search_facts("user:1", "genre", top_k=5)
    assert len(facts) == 1
    assert facts[0].value == "rock"


@pytest.mark.asyncio
async def test_search_facts_ranks_by_overlap() -> None:
    """``search_facts`` returns the most relevant fact first.

    The current ``InMemoryStore`` indexes on (key, tags) — the
    fact's value text is not part of the match. A fact with the
    tag ``"jazz"`` and a key containing the substring "jazz" will
    match a query for "jazz"; a fact whose only overlap is the
    value text will not. This test pins the documented contract
    so a future redesign can update both sides together.
    """
    store = InMemoryStore()
    await store.save_fact("user:1", Fact(key="favorite_food", value="sushi"))
    await store.save_fact(
        "user:1", Fact(key="music_genre", value="jazz", tags=("jazz", "music"))
    )
    await store.save_fact(
        "user:1", Fact(key="mood", value="happy", tags=("jazz", "energetic"))
    )
    facts = await store.search_facts("user:1", "jazz", top_k=5)
    keys = [f.key for f in facts]
    # Both ``music_genre`` and ``mood`` have ``jazz`` in their tags.
    assert "music_genre" in keys
    assert "mood" in keys
    # ``favorite_food`` has neither ``jazz`` in its key nor in its tags.
    assert "favorite_food" not in keys


@pytest.mark.asyncio
async def test_search_facts_rejects_empty_query() -> None:
    """Empty query strings are rejected."""
    store = InMemoryStore()
    with pytest.raises(ValueError, match="non-empty"):
        await store.search_facts("scope", "")


@pytest.mark.asyncio
async def test_search_facts_rejects_zero_top_k() -> None:
    """``top_k`` must be positive."""
    store = InMemoryStore()
    with pytest.raises(ValueError, match="top_k"):
        await store.search_facts("scope", "x", top_k=0)


@pytest.mark.asyncio
async def test_load_recent_empty_scope_returns_empty() -> None:
    """An unknown scope returns an empty list, not an error."""
    store = InMemoryStore()
    assert await store.load_recent("nonexistent") == []


@pytest.mark.asyncio
async def test_in_memory_store_is_aclose_safe() -> None:
    """``aclose`` is a no-op on the in-memory store."""
    store = InMemoryStore()
    await store.aclose()
    # The store is still usable.
    await store.append_turn("x", Turn(role="user", content="hi"))
    assert len(await store.load_recent("x")) == 1


def test_in_memory_store_name() -> None:
    """The store's name is the canonical ``in_memory``."""
    assert InMemoryStore().name == "in_memory"
