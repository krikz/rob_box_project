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

import pytest

from rob_box_harness.memory import (
    Fact,
    InMemoryStore,
    ensure_speaker_profile,
    get_speaker_profile,
    speaker_scope,
    touch_speaker,
)
from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory
from rob_box_voice.speaker_profiles import SpeakerTracker, extract_speaker_name


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


class TestSpeakerFullPathAcceptance:
    """Acceptance #1077: yandex speaker_analysis → профиль спикера в SQLite.

    Воспроизводит полный путь dialogue_node._handle_speaker_turn:
    две фразы подряд с одним tag (x2 .ogg: «Робот, меня зовут Саша» ×2)
    → SpeakerTracker подтверждает → touch_speaker создаёт профиль
    → имя «Саша» сохраняется в scope=speaker:<tag>.
    """

    def test_two_phrases_confirm_profile_with_name(self):
        store, db = _make_sqlite_store()
        tracker = SpeakerTracker()  # min_phrases=2, min_duration_s=0.8

        # 1-я фраза: streak=1, профиль ещё не создаётся.
        assert tracker.note_phrase("0", 2.28) is False
        assert tracker.is_confirmed("0") is False
        assert _run(get_speaker_profile(store, "0")) is None

        # 2-я фраза подряд: tag подтверждён (just_confirmed=True).
        assert tracker.note_phrase("0", 2.31) is True
        assert tracker.is_confirmed("0") is True

        # touch_speaker — как в _handle_speaker_turn после подтверждения.
        profile = _run(touch_speaker(store, "0", now=100.0))
        assert profile["dialog_count"] == 1
        assert profile["first_seen"] == 100.0

        # Имя из «меня зовут Саша» → сохраняется в профиль.
        name = extract_speaker_name("робот меня зовут Саша")
        assert name == "Саша"
        profile["name"] = name
        _run(store.save_fact(
            speaker_scope("0"),
            Fact(key="profile", value=profile, tags=("speaker", "profile")),
        ))

        # Acceptance: профиль реально создан в SQLite (scope=speaker:0).
        saved = _run(get_speaker_profile(store, "0"))
        assert saved is not None
        assert saved["name"] == "Саша"
        assert saved["dialog_count"] == 1
        facts = _run(store.list_facts(speaker_scope("0")))
        assert any(f.key == "profile" for f in facts)
        # Другие спикеры изолированы.
        assert _run(get_speaker_profile(store, "1")) is None

    def test_vosk_fallback_no_profile_no_crash(self):
        """Edge case #4: tag=None (Vosk fallback) — профиль не создаётся,
        код не падает."""
        store, _ = _make_sqlite_store()
        tracker = SpeakerTracker()

        assert tracker.note_phrase(None, 2.0) is False
        assert tracker.is_confirmed(None) is False
        assert _run(get_speaker_profile(store, "0")) is None
        assert _run(get_speaker_profile(store, "1")) is None

    def test_short_phrase_does_not_confirm(self):
        """Edge case #1: короткая фраза (<0.8с) не создаёт профиль."""
        store, _ = _make_sqlite_store()
        tracker = SpeakerTracker()

        assert tracker.note_phrase("0", 0.5) is False
        assert tracker.note_phrase("0", 0.7) is False
        assert tracker.is_confirmed("0") is False
        assert _run(get_speaker_profile(store, "0")) is None

    def test_unstable_tag_resets_streak(self):
        """Edge case #1: смена tag сбрасывает streak — профили не смешиваются."""
        store, _ = _make_sqlite_store()
        tracker = SpeakerTracker()

        assert tracker.note_phrase("0", 2.0) is False
        assert tracker.note_phrase("1", 2.0) is False  # другой tag → streak сброс
        assert tracker.note_phrase("0", 2.0) is False  # streak=1 снова
        assert tracker.note_phrase("0", 2.0) is True   # streak=2 → подтверждён
        assert tracker.is_confirmed("0") is True
        assert tracker.is_confirmed("1") is False
        assert _run(get_speaker_profile(store, "1")) is None
