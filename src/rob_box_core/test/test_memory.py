"""Tests for `MemoryStore` / `InMemoryStore`."""

from __future__ import annotations

import asyncio

import pytest

from rob_box_core.memory import (
    Fact,
    InMemoryStore,
    MemoryHit,
    MemoryStore,
    Turn,
)


async def _new_store(*, t: float = 0.0) -> InMemoryStore:
    counter = {"t": t}
    return InMemoryStore(clock_factory=lambda: counter["t"])


# ---------------------------------------------------------------------------
# turns
# ---------------------------------------------------------------------------


def test_turns_are_assigned_increasing_ids():
    async def go():
        s = await _new_store()
        a = await s.append_turn("user", "hi")
        b = await s.append_turn("assistant", "hello")
        c = await s.append_turn("user", "again")
        return a, b, c

    a, b, c = asyncio.run(go())
    assert a < b < c


def test_load_recent_returns_oldest_first_within_scope():
    async def go():
        s = await _new_store(t=0.0)
        await s.append_turn("user", "first", scope="u1")
        await s.append_turn("user", "second", scope="u2")
        await s.append_turn("user", "third", scope="u1")
        out = await s.load_recent(10, scope="u1")
        return out

    out = asyncio.run(go())
    assert [t.content for t in out] == ["first", "third"]


def test_load_recent_respects_limit_and_excludes_other_scopes():
    async def go():
        s = await _new_store()
        for i in range(5):
            await s.append_turn("user", f"u1-{i}", scope="u1")
            await s.append_turn("user", f"u2-{i}", scope="u2")
        return await s.load_recent(3, scope="u1")

    out = asyncio.run(go())
    assert [t.content for t in out] == ["u1-2", "u1-3", "u1-4"]


def test_load_recent_zero_or_negative_limit_returns_empty():
    async def go():
        s = await _new_store()
        await s.append_turn("user", "x")
        return await s.load_recent(0), await s.load_recent(-5)

    a, b = asyncio.run(go())
    assert a == [] and b == []


def test_turn_meta_is_preserved():
    async def go():
        s = await _new_store()
        await s.append_turn("user", "hi", source="telegram", chat_id=42)
        return (await s.load_recent(1))[0]

    turn = asyncio.run(go())
    assert turn.meta.get("source") == "telegram"
    assert turn.meta.get("chat_id") == 42


# ---------------------------------------------------------------------------
# facts
# ---------------------------------------------------------------------------


def test_save_fact_assigns_increasing_ids():
    async def go():
        s = await _new_store()
        a = await s.save_fact("loves pizza")
        b = await s.save_fact("lives in Spb")
        return a, b

    a, b = asyncio.run(go())
    assert a < b


def test_search_facts_scores_by_token_overlap():
    async def go():
        s = await _new_store()
        await s.save_fact("user loves spicy pizza")
        await s.save_fact("user has two cats")
        await s.save_fact("user is allergic to nuts")
        return await s.search_facts("pizza spicy")

    hits = asyncio.run(go())
    assert hits and hits[0].text.startswith("user loves spicy pizza")
    assert all(isinstance(h, Fact) for h in hits)


def test_search_facts_empty_query_returns_empty():
    async def go():
        s = await _new_store()
        await s.save_fact("anything")
        return await s.search_facts("")

    assert asyncio.run(go()) == []


# ---------------------------------------------------------------------------
# hybrid search
# ---------------------------------------------------------------------------


def test_search_returns_mixed_hits_sorted_by_score():
    async def go():
        s = await _new_store()
        await s.append_turn("user", "let's order pizza tonight")
        await s.append_turn("assistant", "ok, pepperoni?")
        await s.save_fact("user loves pepperoni pizza")
        await s.save_fact("user has two cats")
        return await s.search("pepperoni pizza", limit=10)

    hits = asyncio.run(go())
    assert len(hits) >= 2
    assert all(isinstance(h, MemoryHit) for h in hits)
    # Sorted by score descending.
    scores = [h.score for h in hits]
    assert scores == sorted(scores, reverse=True)
    # Both turn and fact present.
    assert {h.source for h in hits} == {"turn", "fact"}


def test_search_with_empty_query_returns_empty():
    async def go():
        s = await _new_store()
        await s.append_turn("user", "hi")
        return await s.search("")

    assert asyncio.run(go()) == []


# ---------------------------------------------------------------------------
# concurrency
# ---------------------------------------------------------------------------


def test_store_is_thread_safe_under_concurrent_writes():
    import threading

    s = InMemoryStore()

    async def writer(prefix: str, n: int) -> None:
        for i in range(n):
            await s.append_turn("user", f"{prefix}-{i}")

    async def go():
        await asyncio.gather(*(writer(f"w{i}", 50) for i in range(8)))

    asyncio.run(go())
    out = asyncio.run(s.load_recent(1000))
    # All 400 turns (8 writers × 50) should be present, no lost writes.
    assert len(out) == 400
    assert len({t.content for t in out}) == 400


# ---------------------------------------------------------------------------
# port contract
# ---------------------------------------------------------------------------


def test_in_memory_store_satisfies_port():
    s = InMemoryStore()
    assert isinstance(s, MemoryStore)


def test_aclose_is_default_noop():
    s = InMemoryStore()
    asyncio.run(s.aclose())  # must not raise
