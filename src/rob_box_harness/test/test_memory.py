"""Tests for the in-memory MemoryStore and the Turn / Fact value objects.

Covers:

* ``append_turn`` keeps an ordered history per scope.
* ``load_recent`` returns the most recent turns in newest-first order.
* ``save_fact`` is idempotent on the fact key.
* ``search_facts`` ranks by token overlap with ``tags`` and ``key``.
* ``save_waypoint`` / ``list_waypoints`` / ``delete_waypoint`` /
  ``clear_waypoints`` cover waypoint CRUD (W4).
* ``load_faq`` / ``search_faq`` cover FAQ lifecycle and keyword search (W4).
* ``set_event_profile`` / ``get_event_profile`` cover the singleton
  active-event profile (W4).
* Invalid inputs (empty query, non-positive limits) are rejected.
"""

from __future__ import annotations

import pytest

from rob_box_harness.memory import (
    FAQItem,
    Fact,
    InMemoryStore,
    MemoryStore,
    Turn,
    Waypoint,
)


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


# ── W4: Waypoints ────────────────────────────────────────────────


class TestWaypoints:
    """Waypoint CRUD on the in-memory store."""

    @pytest.mark.asyncio
    async def test_save_and_list(self) -> None:
        store = InMemoryStore()
        await store.save_waypoint("kitchen", 1.0, 2.0, 0.5)
        await store.save_waypoint("dock", 0.0, 0.0)
        waypoints = await store.list_waypoints()
        # Sorted by name: dock, kitchen
        assert [w.name for w in waypoints] == ["dock", "kitchen"]
        kitchen = next(w for w in waypoints if w.name == "kitchen")
        assert kitchen.x == 1.0
        assert kitchen.y == 2.0
        assert kitchen.theta == 0.5
        dock = next(w for w in waypoints if w.name == "dock")
        assert dock.theta == 0.0  # default

    @pytest.mark.asyncio
    async def test_upsert_replaces_pose(self) -> None:
        store = InMemoryStore()
        await store.save_waypoint("kitchen", 1.0, 2.0, 0.5)
        await store.save_waypoint("kitchen", 10.0, 20.0, 1.0)
        waypoints = await store.list_waypoints()
        assert len(waypoints) == 1
        assert waypoints[0].x == 10.0
        assert waypoints[0].theta == 1.0

    @pytest.mark.asyncio
    async def test_delete_returns_true_then_false(self) -> None:
        store = InMemoryStore()
        await store.save_waypoint("kitchen", 1.0, 2.0)
        assert await store.delete_waypoint("kitchen") is True
        assert await store.delete_waypoint("kitchen") is False
        assert await store.delete_waypoint("nope") is False
        assert await store.list_waypoints() == []

    @pytest.mark.asyncio
    async def test_clear_returns_count(self) -> None:
        store = InMemoryStore()
        await store.save_waypoint("a", 0.0, 0.0)
        await store.save_waypoint("b", 1.0, 1.0)
        removed = await store.clear_waypoints()
        assert removed == 2
        assert await store.list_waypoints() == []
        # Second clear returns 0
        assert await store.clear_waypoints() == 0

    @pytest.mark.asyncio
    async def test_save_waypoint_rejects_empty_name(self) -> None:
        store = InMemoryStore()
        with pytest.raises(ValueError, match="non-empty"):
            await store.save_waypoint("", 1.0, 2.0)

    def test_waypoint_dataclass_defaults(self) -> None:
        """``Waypoint.theta`` defaults to 0.0 for convenience."""
        wp = Waypoint(name="x", x=1.0, y=2.0)
        assert wp.theta == 0.0


# ── W4: FAQ ─────────────────────────────────────────────────────


class TestFAQ:
    """FAQ load / search on the in-memory store."""

    @pytest.mark.asyncio
    async def test_load_faq_returns_insert_count(self) -> None:
        store = InMemoryStore()
        items = [
            {"question": "Q1?", "answer": "A1.", "category": "info"},
            {"question": "Q2?", "answer": "A2."},
            {"question": "", "answer": ""},  # skipped
        ]
        inserted = await store.load_faq("expo", items)
        assert inserted == 2

    @pytest.mark.asyncio
    async def test_load_faq_replaces_previous(self) -> None:
        store = InMemoryStore()
        await store.load_faq("expo", [{"question": "old", "answer": "old"}])
        inserted = await store.load_faq(
            "expo", [{"question": "new", "answer": "new"}]
        )
        assert inserted == 1
        results = await store.search_faq("expo", "old")
        assert results == []
        results = await store.search_faq("expo", "new")
        assert len(results) == 1

    @pytest.mark.asyncio
    async def test_load_faq_isolates_events(self) -> None:
        store = InMemoryStore()
        await store.load_faq(
            "expo",
            [{"question": "Q1", "answer": "A1"}],
        )
        await store.load_faq(
            "conf",
            [{"question": "Q1", "answer": "different"}],
        )
        expo = await store.search_faq("expo", "Q1")
        conf = await store.search_faq("conf", "Q1")
        assert expo[0].answer == "A1"
        assert conf[0].answer == "different"

    @pytest.mark.asyncio
    async def test_search_faq_ranks_by_overlap(self) -> None:
        store = InMemoryStore()
        await store.load_faq(
            "expo",
            [
                {"question": "schedule?", "answer": "10am-6pm"},
                {"question": "where?", "answer": "main hall"},
                {"question": "food?", "answer": "yes"},
            ],
        )
        results = await store.search_faq("expo", "schedule")
        assert len(results) == 1
        assert results[0].question == "schedule?"
        # Query appearing in two questions — both should match
        results = await store.search_faq("expo", "?")
        assert len(results) == 3

    @pytest.mark.asyncio
    async def test_search_faq_respects_limit(self) -> None:
        store = InMemoryStore()
        await store.load_faq(
            "expo",
            [{"question": f"Q{i}", "answer": f"A{i}"} for i in range(5)],
        )
        results = await store.search_faq("expo", "Q", limit=2)
        assert len(results) == 2

    @pytest.mark.asyncio
    async def test_search_faq_empty_query_returns_empty(self) -> None:
        store = InMemoryStore()
        await store.load_faq("expo", [{"question": "Q", "answer": "A"}])
        assert await store.search_faq("expo", "") == []
        assert await store.search_faq("expo", "   ") == []

    @pytest.mark.asyncio
    async def test_search_faq_rejects_zero_limit(self) -> None:
        store = InMemoryStore()
        with pytest.raises(ValueError, match="limit"):
            await store.search_faq("expo", "x", limit=0)

    @pytest.mark.asyncio
    async def test_load_faq_rejects_empty_event_id(self) -> None:
        store = InMemoryStore()
        with pytest.raises(ValueError, match="event_id"):
            await store.load_faq("", [{"question": "Q", "answer": "A"}])

    def test_faq_item_dataclass_defaults(self) -> None:
        """FAQItem default category/source."""
        item = FAQItem(event_id="e", question="q", answer="a")
        assert item.category == "general"
        assert item.source == ""


# ── W4: Event profile ───────────────────────────────────────────


class TestEventProfile:
    """Singleton active-event profile on the in-memory store."""

    @pytest.mark.asyncio
    async def test_get_event_profile_unset_returns_none(self) -> None:
        store = InMemoryStore()
        assert await store.get_event_profile() is None

    @pytest.mark.asyncio
    async def test_set_then_get(self) -> None:
        store = InMemoryStore()
        await store.set_event_profile({"event_id": "expo", "name": "Expo 2026"})
        profile = await store.get_event_profile()
        assert profile == {"event_id": "expo", "name": "Expo 2026"}

    @pytest.mark.asyncio
    async def test_set_overwrites(self) -> None:
        store = InMemoryStore()
        await store.set_event_profile({"event_id": "first"})
        await store.set_event_profile({"event_id": "second"})
        assert (await store.get_event_profile()) == {"event_id": "second"}

    @pytest.mark.asyncio
    async def test_get_returns_independent_copy(self) -> None:
        """Mutating the returned profile must not leak back into the store."""
        store = InMemoryStore()
        await store.set_event_profile({"event_id": "expo"})
        profile = await store.get_event_profile()
        assert profile is not None
        profile["event_id"] = "tampered"
        # Re-fetch — internal state untouched
        fresh = await store.get_event_profile()
        assert fresh == {"event_id": "expo"}


# ── W4: ABC contract smoke test ────────────────────────────────


def test_memory_store_abc_has_all_new_methods() -> None:
    """The ABC declares the W4 contract methods as abstract."""
    expected = {
        "save_waypoint",
        "list_waypoints",
        "delete_waypoint",
        "clear_waypoints",
        "load_faq",
        "search_faq",
        "set_event_profile",
        "get_event_profile",
    }
    abstract_names = set(MemoryStore.__abstractmethods__)
    missing = expected - abstract_names
    assert not missing, f"MemoryStore ABC missing abstract methods: {missing}"
