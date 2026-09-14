#!/usr/bin/env python3
"""test_identity.py — тесты шва идентичности «Знакомый» (issue #2440).

Покрывает памятный слой шва (``rob_box_harness.identity``) на
``InMemoryStore``: note_seen / since_last_seen / merge и value-объект
``Acquaintance``. Голосовой адаптер (``VoiceIdentitySeam``) тестируется
отдельно в ``src/rob_box_voice/test/test_identity_seam.py``.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_harness.identity import (
    Acquaintance,
    IdentitySeam,
    MemoryIdentitySeam,
)
from rob_box_harness.memory import Fact, InMemoryStore, speaker_scope


def _run(coro):
    # Свежий event loop на каждый вызов (устойчиво к закрытому loop после
    # других async-тестов в общем прогоне).
    return asyncio.run(coro)


class _FakeSeam(IdentitySeam):
    """Фейковый адаптер: сигнал IS id (для тестов памятного слоя)."""

    def resolve(self, signal):
        return Acquaintance(id=str(signal))


class TestAcquaintance:
    def test_value_object(self):
        a = Acquaintance(id="u1", name="Иван", epithet="Кулибин", confidence=0.93)
        assert a.id == "u1"
        assert a.name == "Иван"
        assert a.epithet == "Кулибин"
        assert a.confidence == 0.93

    def test_defaults_are_none(self):
        a = Acquaintance(id="u1")
        assert a.name is None
        assert a.epithet is None
        assert a.confidence is None


class TestNoteSeenSinceLastSeen:
    def test_no_profile_yet(self):
        store = InMemoryStore()
        _run(store.init())
        seam = _FakeSeam(store)
        assert _run(seam.since_last_seen(Acquaintance(id="u1"), now=100.0)) is None

    def test_note_seen_creates_and_updates(self):
        store = InMemoryStore()
        _run(store.init())
        seam = _FakeSeam(store)
        person = Acquaintance(id="u1")
        p1 = _run(seam.note_seen(person, now=100.0))
        assert p1["dialog_count"] == 1
        assert p1["first_seen"] == 100.0
        p2 = _run(seam.note_seen(person, now=200.0))
        assert p2["dialog_count"] == 2
        assert p2["first_seen"] == 100.0  # first_seen не трогаем

    def test_since_last_seen_positive_interval(self):
        store = InMemoryStore()
        _run(store.init())
        seam = _FakeSeam(store)
        person = Acquaintance(id="u1")
        _run(seam.note_seen(person, now=100.0))
        assert _run(seam.since_last_seen(person, now=460.0)) == 360.0

    def test_since_last_seen_profile_without_last_seen(self):
        # Профиль без last_seen (например, битая запись) → None, не 0.
        store = InMemoryStore()
        _run(store.init())
        seam = _FakeSeam(store)
        _run(
            store.save_fact(
                speaker_scope("u1"), Fact(key="profile", value={"name": "x"})
            )
        )
        assert _run(seam.since_last_seen(Acquaintance(id="u1"), now=100.0)) is None


class TestMerge:
    def test_base_merge_moves_facts_only(self):
        store = InMemoryStore()
        _run(store.init())
        seam = _FakeSeam(store)
        _run(store.save_fact(speaker_scope("src"), Fact(key="likes", value="кофе")))
        emb_moved, facts_moved = _run(seam.merge("src", "dst"))
        assert emb_moved == 0  # базовый шов не трогает биометрию
        assert facts_moved == 1
        dst_facts = _run(store.list_facts(speaker_scope("dst")))
        assert {f.key for f in dst_facts} == {"likes"}
        assert _run(store.list_facts(speaker_scope("src"))) == []


class TestMemoryIdentitySeam:
    def test_resolve_not_implemented(self):
        store = InMemoryStore()
        _run(store.init())
        seam = MemoryIdentitySeam(store)
        with pytest.raises(NotImplementedError):
            seam.resolve("any-signal")
