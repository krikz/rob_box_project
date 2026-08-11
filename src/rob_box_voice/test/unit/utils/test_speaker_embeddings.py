#!/usr/bin/env python3
"""
test_speaker_embeddings.py — Pure-Python тесты голосовой биометрии (issue #1077).

Проверяет SpeakerDatabase (utils/speaker_embeddings.py) без resemblyzer:
- register: создание спикера + эмбеддинг в SQLite
- identify: cosine similarity, порог IDENTIFY_THRESHOLD (0.75)
- rename / list_speakers / delete_speaker
- embed_audio: ленивый импорт resemblyzer, None при недоступности

Модуль импортируется напрямую по пути файла — минует utils/__init__.py
(который тянет pyaudio через audio_utils), поэтому тест идёт в CI без
тяжёлых зависимостей. resemblyzer/sklearn — ленивые импорты внутри методов.
"""

from __future__ import annotations

import importlib.util
import os
import sqlite3
import sys
from pathlib import Path

import numpy as np
import pytest

_PKG_ROOT = Path(__file__).resolve().parents[3] / "rob_box_voice"
_SPEAKER_EMB = _PKG_ROOT / "utils" / "speaker_embeddings.py"

_spec = importlib.util.spec_from_file_location(
    "rob_box_voice.utils.speaker_embeddings", _SPEAKER_EMB
)
_se = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_voice.utils.speaker_embeddings"] = _se
_spec.loader.exec_module(_se)

SpeakerDatabase = _se.SpeakerDatabase
SpeakerMatch = _se.SpeakerMatch
IDENTIFY_THRESHOLD = _se.IDENTIFY_THRESHOLD


def _random_embedding(seed: int = 0, dim: int = 256) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(dim).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path):
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


class TestRegister:
    def test_register_creates_speaker_and_embedding(self, db):
        sid = db.register("Шифу", _random_embedding(1))
        assert sid and len(sid) == 36  # uuid4
        rows = db._conn.execute("SELECT * FROM speakers").fetchall()
        assert len(rows) == 1
        assert rows[0][1] == "Шифу"
        embs = db._conn.execute("SELECT * FROM embeddings").fetchall()
        assert len(embs) == 1
        assert len(embs[0][2]) == 256 * 4  # float32 blob

    def test_register_existing_id_adds_embedding(self, db):
        sid = db.register("Шифу", _random_embedding(1))
        db.register("Шифу", _random_embedding(2), speaker_id=sid)
        embs = db._conn.execute("SELECT * FROM embeddings").fetchall()
        assert len(embs) == 2
        assert len(db.list_speakers()) == 1


class TestIdentify:
    def test_same_voice_matches_high_confidence(self, db):
        emb = _random_embedding(42)
        sid = db.register("Саша", emb)
        match = db.identify(emb)
        assert isinstance(match, SpeakerMatch)
        assert match.name == "Саша"
        assert match.confidence > 0.95  # identical vector → ~1.0

    def test_close_voice_matches_above_threshold(self, db):
        emb = _random_embedding(7)
        db.register("Саша", emb)
        noisy = emb + 0.05 * _random_embedding(99)
        noisy = noisy / np.linalg.norm(noisy)
        match = db.identify(noisy)
        assert match is not None
        assert match.confidence >= IDENTIFY_THRESHOLD

    def test_different_voice_below_threshold(self, db):
        db.register("Саша", _random_embedding(1))
        other = _random_embedding(2)  # ортогональный голос
        match = db.identify(other)
        assert match is None  # similarity < 0.75 → unknown

    def test_no_speakers_returns_none(self, db):
        assert db.identify(_random_embedding(5)) is None


class TestRenameListDelete:
    def test_rename(self, db):
        sid = db.register("Саша", _random_embedding(1))
        assert db.rename(sid, "Александр") is True
        assert db.list_speakers()[0]["name"] == "Александр"

    def test_rename_missing_returns_false(self, db):
        assert db.rename("no-such-id", "X") is False

    def test_rename_by_name(self, db):
        """Issue #1101 — name-based rename (LLM corrections «я не X, я Y»)."""
        sid = db.register("Эйджик", _random_embedding(1))
        renamed_sid = db.rename_by_name("Эйджик", "Денис")
        assert renamed_sid == sid
        assert db.list_speakers()[0]["name"] == "Денис"

    def test_rename_by_name_case_insensitive(self, db):
        db.register("Эйджик", _random_embedding(1))
        assert db.rename_by_name("эйджик", "Денис") is not None
        assert db.list_speakers()[0]["name"] == "Денис"

    def test_rename_by_name_missing_returns_none(self, db):
        assert db.rename_by_name("Нет-такого", "Денис") is None

    def test_list_speakers_counts_embeddings(self, db):
        sid = db.register("Саша", _random_embedding(1))
        db.register("Саша", _random_embedding(2), speaker_id=sid)
        speakers = db.list_speakers()
        assert len(speakers) == 1
        assert speakers[0]["embeddings"] == 2

    def test_delete_speaker(self, db):
        sid = db.register("Саша", _random_embedding(1))
        assert db.delete_speaker(sid) is True
        assert db.list_speakers() == []


class TestEmbedAudio:
    def test_embed_audio_none_without_resemblyzer(self, db, monkeypatch):
        # resemblyzer не установлен в CI → ленивый импорт вернёт None.
        monkeypatch.setattr(_se, "_resemblyzer_loaded", False)
        assert db.embed_audio(b"\x00\x00" * 16000, sample_rate=16000) is None

    def test_short_audio_returns_none_even_with_encoder(self, db, monkeypatch):
        class _FakeEncoder:
            def embed_utterance(self, wav):
                return np.zeros(256, dtype=np.float32)

        monkeypatch.setattr(_se, "_encoder", _FakeEncoder())
        monkeypatch.setattr(_se, "_resemblyzer_loaded", True)
        # 0.1s < MIN_AUDIO_DURATION_SEC (0.3)
        assert db.embed_audio(b"\x00\x00" * 1600, sample_rate=16000) is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
