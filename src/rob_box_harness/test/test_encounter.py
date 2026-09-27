#!/usr/bin/env python3
"""test_encounter.py — тесты шва «Встреча» (issue #2442).

Покрывает модуль ``rob_box_harness.encounter`` через его интерфейс:
``EncounterSeam.current()`` (чтение) / ``EncounterSeam.observe()``
(запись адаптером) и голосовой адаптер ``VoiceEncounterAdapter``.

Acceptance-сценарий issue #2442 (п. 7а): голосовой сигнал (``кто=X``,
``уверенность=0.9``) и сигнал канала «лицо» (``кто=X``,
``уверенность=0.8``) с разницей по времени < 1с → одна Встреча с
``каналы={голос, лицо}`` и уверенностью не ниже максимума входных —
воспроизведён в ``TestEncounterSeamMerge.
test_two_channels_same_person_merge_into_one_encounter``.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_harness.encounter import (
    Encounter,
    EncounterChannel,
    EncounterSeam,
    VoiceEncounterAdapter,
    acquaintance_from_speaker_result,
)
from rob_box_harness.identity import Acquaintance, IdentitySeam
from rob_box_harness.memory import InMemoryStore


def _run(coro):
    # Свежий event loop на каждый вызов — как в test_identity.py.
    return asyncio.run(coro)


class _FakeIdentitySeam(IdentitySeam):
    """Памятный шов идентичности без биометрии (сигнал не резолвится)."""

    def resolve(self, signal):
        raise NotImplementedError


def _seam(store: InMemoryStore | None = None) -> EncounterSeam:
    store = store or InMemoryStore()
    _run(store.init())
    identity = _FakeIdentitySeam(store)
    return EncounterSeam(identity)


class TestEncounterValueObject:
    def test_fields(self):
        who = Acquaintance(id="u1", name="Денис")
        enc = Encounter(
            who=who,
            confidence=0.9,
            channels=frozenset({EncounterChannel.VOICE}),
            since=100.0,
            since_last_seen=3600.0,
        )
        assert enc.who is who
        assert enc.confidence == 0.9
        assert enc.channels == {EncounterChannel.VOICE}
        assert enc.since == 100.0
        assert enc.since_last_seen == 3600.0

    def test_who_none_is_valid(self):
        # Присутствие без опознания — валидная Встреча (issue #2442 п.1).
        enc = Encounter(
            who=None, confidence=0.4, channels=frozenset(), since=0.0
        )
        assert enc.who is None
        assert enc.channels == frozenset()
        assert enc.since_last_seen is None


class TestEncounterSeamMerge:
    def test_current_is_none_before_any_signal(self):
        seam = _seam()
        assert seam.current(now=0.0) is None

    def test_two_channels_same_person_merge_into_one_encounter(self):
        """Acceptance #2442 п.7а: voice(0.9) + face(0.8) < 1с → одна Встреча."""
        seam = _seam()
        who = Acquaintance(id="denis", name="Денис")

        _run(seam.observe(EncounterChannel.VOICE, who, 0.9, now=1000.0))
        merged = _run(seam.observe(EncounterChannel.FACE, who, 0.8, now=1000.4))

        assert merged.who == who
        assert merged.channels == {EncounterChannel.VOICE, EncounterChannel.FACE}
        assert merged.confidence == pytest.approx(0.9)  # не ниже максимума входных
        assert merged.confidence >= 0.9 and merged.confidence >= 0.8
        assert seam.current(now=1000.4) is merged

    def test_since_does_not_move_on_merge(self):
        seam = _seam()
        who = Acquaintance(id="denis")
        first = _run(seam.observe(EncounterChannel.VOICE, who, 0.5, now=500.0))
        second = _run(seam.observe(EncounterChannel.FACE, who, 0.6, now=500.9))
        assert first.since == 500.0
        assert second.since == 500.0  # непрерывное присутствие не прервалось

    def test_different_person_opens_new_encounter(self):
        seam = _seam()
        a = Acquaintance(id="a")
        b = Acquaintance(id="b")
        _run(seam.observe(EncounterChannel.VOICE, a, 0.9, now=10.0))
        enc_b = _run(seam.observe(EncounterChannel.VOICE, b, 0.7, now=10.5))
        assert enc_b.who == b
        assert enc_b.channels == {EncounterChannel.VOICE}
        assert enc_b.since == 10.5  # новая Встреча — свой since

    def test_presence_expires_by_timeout(self):
        """Присутствие истекает по таймауту (issue #2442, тесты п.3)."""
        seam = EncounterSeam(_FakeIdentitySeam(_init_store()), presence_timeout_sec=30.0)
        who = Acquaintance(id="denis")
        _run(seam.observe(EncounterChannel.VOICE, who, 0.8, now=0.0))

        assert seam.current(now=29.0) is not None  # ещё в пределах окна
        assert seam.current(now=31.0) is None  # таймаут истёк

    def test_new_signal_after_timeout_opens_fresh_encounter(self):
        seam = EncounterSeam(_FakeIdentitySeam(_init_store()), presence_timeout_sec=10.0)
        who = Acquaintance(id="denis")
        _run(seam.observe(EncounterChannel.VOICE, who, 0.8, now=0.0))
        assert seam.current(now=50.0) is None

        fresh = _run(seam.observe(EncounterChannel.VOICE, who, 0.6, now=50.0))
        assert fresh.since == 50.0  # не унаследовал since истёкшей Встречи

    def test_since_last_seen_populated_from_identity_seam(self):
        store = InMemoryStore()
        _run(store.init())
        identity = _FakeIdentitySeam(store)
        seam = EncounterSeam(identity)
        who = Acquaintance(id="denis")

        _run(identity.note_seen(who, now=1000.0))
        enc = _run(seam.observe(EncounterChannel.VOICE, who, 0.9, now=1000.0 + 3600.0))
        assert enc.since_last_seen == pytest.approx(3600.0)

    def test_since_last_seen_none_when_who_is_none(self):
        seam = _seam()
        enc = _run(seam.observe(EncounterChannel.VOICE, None, 0.3, now=0.0))
        assert enc.who is None
        assert enc.since_last_seen is None

    def test_since_last_seen_frozen_for_lifetime_of_encounter(self):
        # Повторные observe() того же человека внутри одной Встречи не
        # должны пересчитывать since_last_seen на каждый сигнал — иначе
        # к концу диалога оно "усохнет" почти до нуля.
        store = InMemoryStore()
        _run(store.init())
        identity = _FakeIdentitySeam(store)
        seam = EncounterSeam(identity)
        who = Acquaintance(id="denis")
        _run(identity.note_seen(who, now=100.0))

        first = _run(seam.observe(EncounterChannel.VOICE, who, 0.5, now=3700.0))
        second = _run(seam.observe(EncounterChannel.VOICE, who, 0.5, now=3705.0))
        assert first.since_last_seen == pytest.approx(3600.0)
        assert second.since_last_seen == pytest.approx(3600.0)


def _init_store() -> InMemoryStore:
    store = InMemoryStore()
    _run(store.init())
    return store


class TestVoiceEncounterAdapter:
    def test_known_speaker_feeds_voice_channel(self):
        seam = _seam()
        adapter = VoiceEncounterAdapter(seam)
        payload = {
            "is_known": True,
            "speaker_id": "u1",
            "name": "Денис",
            "confidence": 0.93,
        }
        enc = _run(adapter.on_speaker_result(payload, now=100.0))
        assert enc.who == Acquaintance(id="u1", name="Денис", confidence=0.93)
        assert enc.channels == {EncounterChannel.VOICE}
        assert enc.confidence == pytest.approx(0.93)

    def test_unknown_speaker_still_valid_encounter_without_who(self):
        seam = _seam()
        adapter = VoiceEncounterAdapter(seam)
        enc = _run(adapter.on_speaker_result({"is_known": False}, now=0.0))
        assert enc.who is None
        assert enc.channels == {EncounterChannel.VOICE}

    def test_registered_event_is_not_a_presence_signal(self):
        seam = _seam()
        adapter = VoiceEncounterAdapter(seam)
        result = _run(
            adapter.on_speaker_result(
                {"event": "registered", "speaker_id": "u1"}, now=0.0
            )
        )
        assert result is None  # ack не создаёт Встречу
        assert seam.current(now=0.0) is None

    def test_two_channels_via_adapter_and_direct_observe_merge(self):
        """Тот же сценарий, что и acceptance, но voice идёт через адаптер."""
        seam = _seam()
        adapter = VoiceEncounterAdapter(seam)
        payload = {
            "is_known": True,
            "speaker_id": "denis",
            "name": "Денис",
            "confidence": 0.9,
        }
        _run(adapter.on_speaker_result(payload, now=1000.0))
        merged = _run(
            seam.observe(
                EncounterChannel.FACE,
                Acquaintance(id="denis"),
                0.8,
                now=1000.4,
            )
        )
        assert merged.channels == {EncounterChannel.VOICE, EncounterChannel.FACE}
        assert merged.confidence == pytest.approx(0.9)


class TestAcquaintanceFromSpeakerResult:
    def test_junk_name_filtered(self):
        a = acquaintance_from_speaker_result(
            {"is_known": True, "speaker_id": "u1", "name": "Null", "confidence": 0.5}
        )
        assert a is not None
        assert a.name is None

    def test_missing_speaker_id_is_unresolved(self):
        assert (
            acquaintance_from_speaker_result({"is_known": True, "name": "X"}) is None
        )

    def test_unknown_is_none(self):
        assert acquaintance_from_speaker_result({"is_known": False}) is None
