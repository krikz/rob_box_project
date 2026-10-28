"""test_issue_3024_face_hint_suppresses_voice_recheck.py

ADR-0139 / issue #3024 — «Робот поздоровался по имени (лицо), а через 26
сек спрашивает „Дэнчик, это ты?"».

Фикс (ADR-0139 §2-3): сигнал ``FaceSignal`` в ``IdentitySeam`` (метод
``note_face_seen`` + ``recent_face_observation``): свежий face-hint с тем же
именем и ``sim >= high_threshold`` снимает переспрос голоса.

PR-15 (#3711, часть 15p): потребитель шва в ноде диалога v1 удалён вместе с
ней; здесь остаётся контракт самого шва (``rob_box_harness.identity``).
"""

from __future__ import annotations

import asyncio

from rob_box_harness.identity import MemoryIdentitySeam
from rob_box_harness.identity.base import FaceObservation, FaceSignal
from rob_box_harness.memory import InMemoryStore


def _run(coro):
    return asyncio.run(coro)


def test_face_signal_and_observation_dataclasses_exist():
    """Публичные типы шва: ``FaceSignal`` и ``FaceObservation`` с окном свежести."""
    sig = FaceSignal(
        person_id="4ff0ddc5",
        name="Дэнчик",
        similarity=0.81,
        is_new=False,
        source_camera="oak_d",
        captured_at=1_000_000.0,
    )
    obs = FaceObservation(
        person_id=sig.person_id,
        name=sig.name,
        similarity=sig.similarity,
        captured_at=sig.captured_at,
        is_new=sig.is_new,
        confidence_band="high",
    )
    assert obs.person_id == "4ff0ddc5"
    assert obs.name == "Дэнчик"
    assert obs.confidence_band == "high"
    assert obs.is_recent(window_sec=30, now=1_000_010.0) is True
    assert obs.is_recent(window_sec=30, now=1_000_100.0) is False


def test_identity_seam_note_face_seen_and_recent_observation_roundtrip():
    """Шов хранит последний face-signal per person_id, читается окном.

    Это контракт ADR-0139 §2.2: ``note_face_seen`` кладёт наблюдение в
    кольцевой буфер, ``recent_face_observation(person_id)`` возвращает
    самое свежее (или ``None`` если пусто/протухло).
    """
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)

    sig = FaceSignal(
        person_id="4ff0ddc5",
        name="Дэнчик",
        similarity=0.81,
        is_new=False,
        source_camera="oak_d",
        captured_at=1_000_000.0,
    )
    # До — нет наблюдения.
    assert seam.recent_face_observation("4ff0ddc5", now=1_000_000.0) is None

    seam.note_face_seen(sig, now=1_000_000.0)
    obs = seam.recent_face_observation("4ff0ddc5", now=1_000_010.0)
    assert obs is not None
    assert obs.person_id == "4ff0ddc5"
    assert obs.name == "Дэнчик"
    assert obs.similarity == 0.81
    assert obs.confidence_band == "high"  # 0.81 >= high=0.78
    # За окном — None (TTL = window_sec).
    assert (
        seam.recent_face_observation("4ff0ddc5", window_sec=30, now=1_000_100.0)
        is None
    )


def test_identity_seam_face_observation_confidence_bands():
    """Полосы уверенности: high/tentative/low по порогам из ADR-0139 §2.4."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)

    base_kw = dict(person_id="x", is_new=False, source_camera="oak_d")

    seam.note_face_seen(
        FaceSignal(name=None, similarity=0.50, captured_at=1.0, **base_kw),
        now=1.0,
    )
    obs_low = seam.recent_face_observation("x", now=2.0)
    assert obs_low is not None and obs_low.confidence_band == "low"

    seam.note_face_seen(
        FaceSignal(name="A", similarity=0.70, captured_at=3.0, **base_kw),
        now=3.0,
    )
    obs_tent = seam.recent_face_observation("x", now=4.0)
    assert obs_tent is not None and obs_tent.confidence_band == "tentative"

    seam.note_face_seen(
        FaceSignal(name="A", similarity=0.85, captured_at=5.0, **base_kw),
        now=5.0,
    )
    obs_high = seam.recent_face_observation("x", now=6.0)
    assert obs_high is not None and obs_high.confidence_band == "high"


def test_face_hint_cache_evicts_expired_person_ids():
    """TTL eviction keeps the per-person cache bounded over process lifetime."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    seam.configure_face_hint(window_sec=30.0, buffer_capacity=8)

    seam.note_face_seen(
        FaceSignal(
            person_id="old-person",
            name="Старый",
            similarity=0.81,
            is_new=False,
            source_camera="oak_d",
            captured_at=100.0,
        ),
        now=100.0,
    )
    assert "old-person" in seam._face_observations

    # A later event triggers TTL eviction; the expired UUID must not remain
    # in the dictionary even though its deque itself had maxlen=8.
    seam.note_face_seen(
        FaceSignal(
            person_id="new-person",
            name="Новый",
            similarity=0.81,
            is_new=False,
            source_camera="oak_d",
            captured_at=131.0,
        ),
        now=131.0,
    )
    assert "old-person" not in seam._face_observations
    assert "new-person" in seam._face_observations
