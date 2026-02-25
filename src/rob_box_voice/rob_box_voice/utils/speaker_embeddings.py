#!/usr/bin/env python3
"""
speaker_embeddings.py — Speaker identification via resemblyzer d-vectors.

Stores per-speaker embeddings in a SQLite database at /data/speakers.db.
Each speaker can have multiple reference embeddings (one per registration),
the identity decision uses mean-of-cosine similarity across all references.

Usage:
    db = SpeakerDatabase("/data/speakers.db")
    embedding = db.embed_audio(pcm_bytes, sample_rate=16000)
    result = db.identify(embedding)   # → SpeakerMatch | None
    new_id = db.register("Иван", embedding)
"""

from __future__ import annotations

import json
import logging
import sqlite3
import struct
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ── Tuning constants ─────────────────────────────────────────────────────────
IDENTIFY_THRESHOLD: float = 0.75    # cosine similarity to accept a match
MIN_AUDIO_DURATION_SEC: float = 0.8  # minimum speech length for reliable embedding
SAMPLE_RATE: int = 16000            # resemblyzer expects 16 kHz mono float32


@dataclass
class SpeakerMatch:
    """Result of a successful speaker identification."""

    speaker_id: str
    name: str
    confidence: float  # 0.0–1.0 (cosine similarity)
    is_known: bool = True


# ── Lazy import of resemblyzer (not available at build time on CI) ────────────
_resemblyzer_loaded = False
_encoder = None


def _load_resemblyzer() -> bool:
    global _resemblyzer_loaded, _encoder
    if _resemblyzer_loaded:
        return _encoder is not None
    try:
        from resemblyzer import VoiceEncoder, preprocess_wav  # noqa: F401

        _encoder = VoiceEncoder(device="cpu")
        _resemblyzer_loaded = True
        logger.info("✅ resemblyzer VoiceEncoder loaded")
        return True
    except Exception as exc:
        logger.warning(f"⚠️ resemblyzer not available: {exc}")
        _resemblyzer_loaded = True  # don't retry every call
        return False


# ── Database helpers ─────────────────────────────────────────────────────────

_CREATE_SQL = """
CREATE TABLE IF NOT EXISTS speakers (
    speaker_id  TEXT PRIMARY KEY,
    name        TEXT NOT NULL,
    created_at  REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS embeddings (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    speaker_id  TEXT NOT NULL REFERENCES speakers(speaker_id) ON DELETE CASCADE,
    embedding   BLOB NOT NULL,
    created_at  REAL NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_emb_speaker ON embeddings(speaker_id);
"""


class SpeakerDatabase:
    """SQLite-backed speaker embedding store."""

    def __init__(self, db_path: str = "/data/speakers.db") -> None:
        self._db_path = db_path
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.executescript(_CREATE_SQL)
        self._conn.commit()
        logger.info(f"SpeakerDatabase opened: {db_path}")

    # ── Serialisation ─────────────────────────────────────────────────────────

    @staticmethod
    def _ndarray_to_blob(arr: np.ndarray) -> bytes:
        return arr.astype(np.float32).tobytes()

    @staticmethod
    def _blob_to_ndarray(blob: bytes) -> np.ndarray:
        return np.frombuffer(blob, dtype=np.float32)

    # ── Core API ──────────────────────────────────────────────────────────────

    def embed_audio(self, pcm_bytes: bytes, sample_rate: int = 16000) -> Optional[np.ndarray]:
        """Convert raw PCM int16 bytes → 256-dim d-vector (numpy float32 array).

        Returns None if resemblyzer is unavailable or the audio is too short.
        """
        if not _load_resemblyzer():
            return None

        try:
            from resemblyzer import VoiceEncoder, preprocess_wav  # noqa: F811

            # Convert int16 PCM → float32 [-1, 1]
            pcm = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0
            duration = len(pcm) / sample_rate
            if duration < MIN_AUDIO_DURATION_SEC:
                logger.debug(f"Audio too short for embedding: {duration:.2f}s < {MIN_AUDIO_DURATION_SEC}s")
                return None

            # Resample to 16 kHz if needed
            if sample_rate != SAMPLE_RATE:
                import scipy.signal

                pcm = scipy.signal.resample_poly(pcm, SAMPLE_RATE, sample_rate)

            wav = preprocess_wav(pcm, source_sr=SAMPLE_RATE)
            embedding = _encoder.embed_utterance(wav)
            return embedding.astype(np.float32)
        except Exception as exc:
            logger.error(f"embed_audio failed: {exc}")
            return None

    def identify(self, embedding: np.ndarray) -> Optional[SpeakerMatch]:
        """Find the closest known speaker.  Returns None if confidence < threshold."""
        rows = self._conn.execute(
            "SELECT s.speaker_id, s.name, e.embedding "
            "FROM embeddings e JOIN speakers s USING (speaker_id)"
        ).fetchall()

        if not rows:
            return None

        # Build per-speaker pools and compute mean similarity
        from sklearn.metrics.pairwise import cosine_similarity  # noqa: PLC0415

        query = embedding.reshape(1, -1)
        speaker_scores: dict[str, Tuple[str, float]] = {}
        for speaker_id, name, blob in rows:
            ref = self._blob_to_ndarray(blob).reshape(1, -1)
            sim = float(cosine_similarity(query, ref)[0][0])
            if speaker_id not in speaker_scores or sim > speaker_scores[speaker_id][1]:
                speaker_scores[speaker_id] = (name, sim)

        best_id, (best_name, best_score) = max(speaker_scores.items(), key=lambda kv: kv[1][1])

        if best_score < IDENTIFY_THRESHOLD:
            logger.debug(f"Best match {best_name!r} score={best_score:.3f} below threshold {IDENTIFY_THRESHOLD}")
            return None

        return SpeakerMatch(speaker_id=best_id, name=best_name, confidence=best_score)

    def register(self, name: str, embedding: np.ndarray, speaker_id: Optional[str] = None) -> str:
        """Create a new speaker (or add another embedding to existing speaker_id)."""
        now = time.time()
        if speaker_id is None:
            speaker_id = str(uuid.uuid4())
            self._conn.execute(
                "INSERT OR IGNORE INTO speakers (speaker_id, name, created_at) VALUES (?, ?, ?)",
                (speaker_id, name, now),
            )
        else:
            # Update name in case it changed
            self._conn.execute(
                "INSERT OR REPLACE INTO speakers (speaker_id, name, created_at) VALUES (?, ?, ?)",
                (speaker_id, name, now),
            )
        self._conn.execute(
            "INSERT INTO embeddings (speaker_id, embedding, created_at) VALUES (?, ?, ?)",
            (speaker_id, self._ndarray_to_blob(embedding), now),
        )
        self._conn.commit()
        logger.info(f"Registered speaker '{name}' id={speaker_id[:8]}")
        return speaker_id

    def rename(self, speaker_id: str, new_name: str) -> bool:
        """Rename an existing speaker."""
        cur = self._conn.execute(
            "UPDATE speakers SET name=? WHERE speaker_id=?", (new_name, speaker_id)
        )
        self._conn.commit()
        return cur.rowcount > 0

    def list_speakers(self) -> List[dict]:
        """Return all registered speakers with embedding count."""
        rows = self._conn.execute(
            "SELECT s.speaker_id, s.name, s.created_at, COUNT(e.id) AS emb_count "
            "FROM speakers s LEFT JOIN embeddings e USING (speaker_id) "
            "GROUP BY s.speaker_id ORDER BY s.created_at"
        ).fetchall()
        return [
            {"id": r[0], "name": r[1], "created_at": r[2], "embeddings": r[3]}
            for r in rows
        ]

    def delete_speaker(self, speaker_id: str) -> bool:
        """Remove a speaker and all their embeddings."""
        cur = self._conn.execute("DELETE FROM speakers WHERE speaker_id=?", (speaker_id,))
        self._conn.commit()
        return cur.rowcount > 0

    def close(self) -> None:
        self._conn.close()
