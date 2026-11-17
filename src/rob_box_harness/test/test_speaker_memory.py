#!/usr/bin/env python3
"""
test_speaker_memory.py — тесты хелперов профилей спикеров (issue #1077).

Проверяет на обоих бэкендах (InMemoryStore + SQLiteVoiceMemory):
- ensure_speaker_profile: создаёт профиль {first_seen, last_seen, dialog_count}
- touch_speaker: инкрементит dialog_count, обновляет last_seen
- get_speaker_profile: читает существующий / None для нового
- list_facts: перечисляет все факты scope (для LLM-контекста)
- Профили переживают рестарт (SQLite: новый инстанс на том же файле)
"""

from __future__ import annotations

import asyncio
import os
import tempfile

from rob_box_harness.memory import (
    Fact,
    InMemoryStore,
    ensure_speaker_profile,
    get_speaker_profile,
    merge_speaker_facts,
    speaker_scope,
    touch_speaker,
)
from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory


def _run(coro):
    # asyncio.run создаёт СВЕЖИЙ event loop на каждый вызов — устойчиво к
    # «закрытому» loop после предыдущих async-тестов в общем прогоне
    # (get_event_loop() падал с RuntimeError в полном suite).
    return asyncio.run(coro)


def _make_sqlite_store(tmp_path=None):
    if tmp_path is None:
        tmp_path = tempfile.mkdtemp()
    db = os.path.join(tmp_path, "voice.db")
    store = SQLiteVoiceMemory(db_path=db)
    _run(store.init())
    return store, db


class TestSpeakerProfileInMemory:
    def test_scope_key(self):
        assert speaker_scope("0") == "speaker:0"
        assert speaker_scope("1") == "speaker:1"

    def test_ensure_creates_profile(self):
        store = InMemoryStore()
        _run(store.init())
        profile = _run(ensure_speaker_profile(store, "0", now=100.0))
        assert profile["first_seen"] == 100.0
        assert profile["last_seen"] == 100.0
        assert profile["dialog_count"] == 0

    def test_ensure_is_idempotent(self):
        store = InMemoryStore()
        _run(store.init())
        p1 = _run(ensure_speaker_profile(store, "0", now=100.0))
        p2 = _run(ensure_speaker_profile(store, "0", now=200.0))
        assert p1 == p2  # не перезаписывает first_seen

    def test_touch_increments_dialog_count(self):
        store = InMemoryStore()
        _run(store.init())
        p1 = _run(touch_speaker(store, "0", now=100.0))
        assert p1["dialog_count"] == 1
        assert p1["first_seen"] == 100.0
        p2 = _run(touch_speaker(store, "0", now=200.0))
        assert p2["dialog_count"] == 2
        assert p2["last_seen"] == 200.0
        assert p2["first_seen"] == 100.0  # first_seen не трогаем

    def test_get_profile(self):
        store = InMemoryStore()
        _run(store.init())
        assert _run(get_speaker_profile(store, "0")) is None
        _run(touch_speaker(store, "0", now=100.0))
        profile = _run(get_speaker_profile(store, "0"))
        assert profile is not None
        assert profile["dialog_count"] == 1

    def test_scopes_are_isolated(self):
        """Инвариант: speaker:<tag> изолирован — факты не протекают."""
        store = InMemoryStore()
        _run(store.init())
        _run(touch_speaker(store, "0", now=100.0))
        assert _run(get_speaker_profile(store, "1")) is None
        # Общий scope не видит спикерских фактов.
        assert _run(store.list_facts("default")) == []

    def test_list_facts_returns_all(self):
        store = InMemoryStore()
        _run(store.init())
        _run(touch_speaker(store, "0", now=100.0))
        _run(store.save_fact(
            speaker_scope("0"),
            Fact(key="name", value="Саша", tags=("speaker", "name")),
        ))
        facts = _run(store.list_facts(speaker_scope("0")))
        keys = {f.key for f in facts}
        assert "profile" in keys
        assert "name" in keys


class TestSpeakerProfileSQLite:
    def test_profile_persists_across_restart(self):
        """Acceptance: профили переживают рестарт робота (SQLite)."""
        store, db = _make_sqlite_store()
        _run(touch_speaker(store, "0", now=100.0))
        _run(store.save_fact(
            speaker_scope("0"),
            Fact(key="name", value="Пётр", tags=("speaker", "name")),
        ))
        _run(store.teardown())

        # «Рестарт» — новый инстанс на том же файле.
        store2 = SQLiteVoiceMemory(db_path=db)
        _run(store2.init())
        profile = _run(get_speaker_profile(store2, "0"))
        assert profile is not None
        assert profile["dialog_count"] == 1
        facts = _run(store2.list_facts(speaker_scope("0")))
        keys = {f.key for f in facts}
        assert "name" in keys

    def test_touch_speaker_sqlite(self):
        store, _ = _make_sqlite_store()
        p1 = _run(touch_speaker(store, "1", now=10.0))
        assert p1["dialog_count"] == 1
        p2 = _run(touch_speaker(store, "1", now=20.0))
        assert p2["dialog_count"] == 2
        assert p2["last_seen"] == 20.0

    def test_list_facts_sqlite_newest_first(self):
        store, _ = _make_sqlite_store()
        _run(store.save_fact(speaker_scope("0"), Fact(key="a", value=1)))
        _run(store.save_fact(speaker_scope("0"), Fact(key="b", value=2)))
        facts = _run(store.list_facts(speaker_scope("0")))
        # Новые (b) — первыми (created_at DESC).
        assert facts[0].key == "b"
        assert {f.key for f in facts} == {"a", "b"}


class TestMergeSpeakerFacts:
    """Issue W5-4 — merge_speaker_facts(): перенос фактов при склейке профилей.

    Пара к ``SpeakerDatabase.merge_speakers()`` (rob_box_voice) — тот
    метод склеивает эмбеддинги голоса, эта функция — факты в слое
    памяти. Проверяется на обоих бэкендах (InMemoryStore + SQLiteVoiceMemory),
    как и остальные хелперы в этом файле.
    """

    def test_moves_facts_from_src_to_dst_in_memory(self):
        store = InMemoryStore()
        _run(store.init())
        _run(store.save_fact(
            speaker_scope("src"),
            Fact(key="likes", value="джаз", tags=("speaker",)),
        ))

        moved = _run(merge_speaker_facts(store, "src", "dst"))

        assert moved == 1
        dst_facts = {f.key: f.value for f in _run(store.list_facts(speaker_scope("dst")))}
        assert dst_facts["likes"] == "джаз"
        # src scope должен быть очищен — иначе факты "застрянут" под
        # старым speaker_id, который никто больше не опознаёт.
        assert _run(store.list_facts(speaker_scope("src"))) == []

    def test_dst_fact_wins_on_key_conflict(self):
        """Issue W5-4: dst — основной (уже подтверждённый) профиль, его
        факты не должны перезаписываться данными временного/дублирующего src."""
        store = InMemoryStore()
        _run(store.init())
        _run(store.save_fact(speaker_scope("dst"), Fact(key="name", value="Денчик")))
        _run(store.save_fact(speaker_scope("src"), Fact(key="name", value="Эйджик")))

        moved = _run(merge_speaker_facts(store, "src", "dst"))

        assert moved == 0  # единственный факт src конфликтовал по ключу — не перенесён
        dst_facts = {f.key: f.value for f in _run(store.list_facts(speaker_scope("dst")))}
        assert dst_facts["name"] == "Денчик"  # dst выигрывает
        assert _run(store.list_facts(speaker_scope("src"))) == []  # src всё равно очищен

    def test_noop_when_src_equals_dst(self):
        store = InMemoryStore()
        _run(store.init())
        _run(store.save_fact(speaker_scope("a"), Fact(key="k", value="v")))
        assert _run(merge_speaker_facts(store, "a", "a")) == 0
        assert len(_run(store.list_facts(speaker_scope("a")))) == 1

    def test_noop_when_src_empty(self):
        store = InMemoryStore()
        _run(store.init())
        assert _run(merge_speaker_facts(store, "empty-src", "dst")) == 0
        assert _run(store.list_facts(speaker_scope("dst"))) == []

    def test_moves_facts_sqlite(self):
        store, _ = _make_sqlite_store()
        _run(touch_speaker(store, "src", now=100.0))  # создаёт profile-факт
        _run(store.save_fact(speaker_scope("src"), Fact(key="likes", value="кофе")))

        moved = _run(merge_speaker_facts(store, "src", "dst"))

        assert moved == 2  # profile + likes
        dst_keys = {f.key for f in _run(store.list_facts(speaker_scope("dst")))}
        assert dst_keys == {"profile", "likes"}
        assert _run(store.list_facts(speaker_scope("src"))) == []
