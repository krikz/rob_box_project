#!/usr/bin/env python3
"""
speaker_embeddings.py — Speaker identification via resemblyzer d-vectors.

Stores per-speaker embeddings in a SQLite database at /data/speakers.db.
Each speaker can have multiple reference embeddings (one per registration),
the identity decision uses MAX-of-cosine similarity across all references
(не mean — старая формулировка докстринга была неверной, см. issue W5-4:
``identify()`` берёт лучший, а не средний скор по пулу эмбеддингов спикера).

Usage:
    db = SpeakerDatabase("/data/speakers.db")
    embedding = db.embed_audio(pcm_bytes, sample_rate=16000)
    result = db.identify(embedding)   # → SpeakerMatch | None
    new_id = db.register("Иван", embedding)

Issue W5-4 (баг «один голос — два профиля»): голая ``register()`` ВСЕГДА
создаёт новый speaker_id, если явно не передан ``speaker_id=``. До этой
задачи вызывающий код (speaker_id_node) никогда его не передавал — то
есть КАЖДЫЙ вызов register_speaker(name=...) от LLM создавал новый
профиль, даже если голос уже был опознан. ``register_or_merge()`` — новый
метод, который сначала пытается опознать говорящего по уже сохранённым
эмбеддингам (более строгий порог, чем обычная идентификация, — см.
REGISTER_MATCH_THRESHOLD) и, при совпадении, дописывает эмбеддинг в
СУЩЕСТВУЮЩИЙ профиль вместо создания нового. ``merge_speakers()`` —
ручная склейка уже расползшихся дублей (например, найденных оператором
в списке спикеров).
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
# Issue W5-4 — порог для решения «дописать эмбеддинг в СУЩЕСТВУЮЩИЙ профиль
# vs завести новый» внутри register_or_merge(). Сознательно ВЫШЕ
# IDENTIFY_THRESHOLD: обычная идентификация ошибается дёшево (один ход
# «не узнал» — не страшно, поправится на следующей фразе), а решение при
# регистрации — дорогое и малообратимое: ложное слияние смешает факты
# ДВУХ разных людей под одним профилем (хуже, чем временный дубль, который
# чинится merge_speakers() после факта). Синтетический бенчмарк (см.
# отчёт задачи W5-4, docs/plans) не показал роста риска ложного слияния
# между 0.75 и 0.85 для независимых голосов — граница взята консервативно
# с запасом, а не в максимуме диапазона.
REGISTER_MATCH_THRESHOLD: float = 0.82
MIN_AUDIO_DURATION_SEC: float = 0.3  # minimum speech length for reliable embedding
SAMPLE_RATE: int = 16000            # resemblyzer expects 16 kHz mono float32


@dataclass
class SpeakerMatch:
    """Result of a successful speaker identification."""

    speaker_id: str
    name: str
    confidence: float  # 0.0–1.0 (cosine similarity)
    is_known: bool = True
    # Issue #1787 — внутренняя кличка робота («Гроссмейстер»). Живёт
    # ПАРАЛЛЕЛЬНО с ``name``: name — что говорит юзер, epithet — чем робот
    # различает тёзок. None, пока профиль её не получил (старые записи до
    # миграции + спикеры, зарегистрированные вне speaker_id_node).
    epithet: Optional[str] = None


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

# Issue #1787 — колонки эпитета. Добавляются миграцией, а не в _CREATE_SQL:
# на роботе уже лежит /data/speakers.db с 42 профилями, и CREATE TABLE
# IF NOT EXISTS для существующей таблицы — no-op, новые колонки в ней сами
# не появятся. Все — nullable, backfill не нужен (research §5.5): старый
# профиль получит эпитет при первой же реплике после апдейта.
_EPITHET_COLUMNS: Tuple[Tuple[str, str], ...] = (
    ("epithet", "TEXT"),              # текущая кличка
    ("epithet_history", "TEXT"),      # JSON: [{ts, old, new, reason}]
    ("tags", "TEXT"),                 # CSV кластеров: «шахматы,техно»
    ("sentiment_score", "REAL"),      # лексическая валентность [-1, 1]
    ("last_epithet_review", "REAL"),  # unix ts последнего пересмотра
)


class SpeakerDatabase:
    """SQLite-backed speaker embedding store."""

    def __init__(self, db_path: str = "/data/speakers.db") -> None:
        self._db_path = db_path
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.executescript(_CREATE_SQL)
        self._conn.commit()
        self._migrate_epithet_columns()
        logger.info(f"SpeakerDatabase opened: {db_path}")

    # ── Migrations ────────────────────────────────────────────────────────────

    def _migrate_epithet_columns(self) -> None:
        """Issue #1787 — добить недостающие колонки эпитета в ``speakers``.

        Идемпотентно: смотрит PRAGMA table_info и добавляет только то,
        чего нет. SQLite не умеет ``ADD COLUMN IF NOT EXISTS``, а падать
        на втором старте нельзя — это путь запуска ноды, не миграционный
        скрипт.
        """
        existing = {
            row[1] for row in self._conn.execute("PRAGMA table_info(speakers)")
        }
        added = []
        for column, sql_type in _EPITHET_COLUMNS:
            if column in existing:
                continue
            self._conn.execute(f"ALTER TABLE speakers ADD COLUMN {column} {sql_type}")
            added.append(column)
        if added:
            self._conn.commit()
            logger.info(f"🔤 speakers: добавлены колонки эпитета {added}")

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
                logger.info(f"⏭️ Audio too short for embedding: {duration:.2f}s < {MIN_AUDIO_DURATION_SEC}s — skipped")
                return None

            # Resample to 16 kHz if needed
            if sample_rate != SAMPLE_RATE:
                import scipy.signal

                pcm = scipy.signal.resample_poly(pcm, SAMPLE_RATE, sample_rate)

            wav = preprocess_wav(pcm, source_sr=SAMPLE_RATE)
            embedding = _encoder.embed_utterance(wav)
            logger.info(
                f"✅ embed_audio: {len(pcm_bytes)} bytes → "
                f"{len(embedding)}-dim vector (norm={float((embedding**2).sum()**0.5):.3f})"
            )
            return embedding.astype(np.float32)
        except Exception as exc:
            logger.error(f"embed_audio failed: {type(exc).__name__}: {exc}")
            return None

    def _score_all(self, embedding: np.ndarray) -> List[Tuple[str, str, float]]:
        """Посчитать best-of-pool cosine similarity для КАЖДОГО известного спикера.

        Возвращает список ``(speaker_id, name, score)``, отсортированный по
        убыванию score. ``score`` — MAX косинусной близости по всем
        сохранённым эмбеддингам спикера (не mean — устойчивее к разнородному
        пулу: шум/громкость/дистанция одной "плохой" фразы не размывают уже
        подтверждённое совпадение с лучшей референсной записью).

        Общий метод для ``identify()`` (порог IDENTIFY_THRESHOLD) и
        ``identify_candidates()`` (диагностика без порога, issue W5-4 п.4).
        """
        rows = self._conn.execute(
            "SELECT s.speaker_id, s.name, e.embedding "
            "FROM embeddings e JOIN speakers s USING (speaker_id)"
        ).fetchall()
        if not rows:
            return []

        query = embedding.reshape(1, -1)
        if query.shape[1] == 0:
            return []

        # numpy-only cosine similarity — не тянем sklearn как обязательную
        # зависимость (issue #1077).
        speaker_scores: dict[str, Tuple[str, float]] = {}
        for speaker_id, name, blob in rows:
            ref = self._blob_to_ndarray(blob).reshape(1, -1)
            if ref.shape[1] != query.shape[1]:
                continue
            q_norm = query / (np.linalg.norm(query) + 1e-9)
            r_norm = ref / (np.linalg.norm(ref) + 1e-9)
            sim = float(np.dot(q_norm, r_norm.T)[0][0])
            if speaker_id not in speaker_scores or sim > speaker_scores[speaker_id][1]:
                speaker_scores[speaker_id] = (name, sim)

        ranked = sorted(
            ((sid, name, score) for sid, (name, score) in speaker_scores.items()),
            key=lambda t: -t[2],
        )
        return ranked

    def identify(
        self, embedding: np.ndarray, threshold: Optional[float] = None
    ) -> Optional[SpeakerMatch]:
        """Find the closest known speaker.  Returns None if confidence < threshold.

        ``threshold`` по умолчанию — модульный ``IDENTIFY_THRESHOLD``
        (обычное распознавание на ход диалога). Вызывающий код может
        передать более строгий порог — например, ``register_or_merge()``
        использует ``REGISTER_MATCH_THRESHOLD`` для решения «слить с
        существующим профилем vs завести новый» (issue W5-4).
        """
        ranked = self._score_all(embedding)
        if not ranked:
            return None
        thr = IDENTIFY_THRESHOLD if threshold is None else threshold
        best_id, best_name, best_score = ranked[0]

        if best_score < thr:
            logger.debug(f"Best match {best_name!r} score={best_score:.3f} below threshold {thr}")
            return None

        return SpeakerMatch(
            speaker_id=best_id,
            name=best_name,
            confidence=best_score,
            epithet=self.get_epithet(best_id),
        )

    def identify_candidates(
        self, embedding: np.ndarray, top_n: int = 2
    ) -> List[SpeakerMatch]:
        """Топ-N кандидатов БЕЗ порога — для диагностики (issue W5-4 п.4).

        Без этого в проде виден только булев результат identify() —
        «известен / неизвестен» — и дрейф голоса между двумя дублирующими
        профилями невозможно отследить постфактум: неясно, насколько
        близко было решение и с кем именно конкурировал победитель.
        Возвращает пустой список, если спикеров в БД ещё нет.
        """
        ranked = self._score_all(embedding)[: max(0, top_n)]
        return [
            SpeakerMatch(speaker_id=sid, name=name, confidence=score)
            for sid, name, score in ranked
        ]

    def register(self, name: str, embedding: np.ndarray, speaker_id: Optional[str] = None) -> str:
        """Create a new speaker (or add another embedding to existing speaker_id).

        ⚠️ Issue W5-4: эта функция ВСЕГДА создаёт новый профиль, если
        ``speaker_id`` не передан явно — она НЕ проверяет, похож ли голос
        на уже известного спикера. Для потока «LLM вызвал register_speaker
        по имени, услышанному в речи» используйте ``register_or_merge()``
        — он сначала проверяет совпадение и только потом решает, создавать
        новый профиль или дописать эмбеддинг в существующий.
        """
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

    def register_or_merge(
        self, name: str, embedding: np.ndarray, speaker_id: Optional[str] = None
    ) -> Tuple[str, bool]:
        """Зарегистрировать эмбеддинг, избегая создания дубля (issue W5-4).

        Если ``speaker_id`` передан явно — поведение как у ``register()``
        (вызывающий код уже знает, к какому профилю привязать эмбеддинг).

        Иначе сначала проверяется, похож ли голос на уже известного
        спикера (``identify(embedding, threshold=REGISTER_MATCH_THRESHOLD)``
        — порог строже обычной идентификации, см. комментарий у константы).
        При совпадении эмбеддинг дописывается в НАЙДЕННЫЙ профиль (имя
        обновляется на переданное ``name`` — это же путь, которым раньше
        шёл ``rename``), новый профиль не создаётся. Иначе — обычная
        регистрация нового профиля.

        Возвращает ``(speaker_id, reused)``: ``reused=True``, если
        эмбеддинг присоединён к уже существующему профилю.
        """
        if speaker_id is not None:
            return self.register(name, embedding, speaker_id=speaker_id), False

        match = self.identify(embedding, threshold=REGISTER_MATCH_THRESHOLD)
        if match is not None:
            logger.info(
                f"🔗 register_or_merge: голос похож на уже известного "
                f"'{match.name}' (score={match.confidence:.3f} >= "
                f"{REGISTER_MATCH_THRESHOLD}) — дописываю в id={match.speaker_id[:8]} "
                f"вместо нового профиля"
            )
            return self.register(name, embedding, speaker_id=match.speaker_id), True

        new_id = self.register(name, embedding, speaker_id=None)
        return new_id, False

    def merge_speakers(self, src_id: str, dst_id: str) -> int:
        """Слить два профиля одного человека (issue W5-4).

        Переносит ВСЕ эмбеддинги ``src_id`` под ``dst_id`` и удаляет
        профиль ``src_id``. Имя ``dst_id`` не меняется — считается основным
        (тот, под кем профиль решили оставить). Возвращает число
        перенесённых эмбеддингов.

        Идемпотентно/безопасно: возвращает 0 и ничего не делает, если
        ``src_id == dst_id``, любой из id пуст, либо ``dst_id`` не найден
        в БД (защита от опечатки в вызывающем коде — иначе можно случайно
        "потерять" src, не создав валидный dst).

        Факты собеседника (``scope="speaker:<id>"``) живут в отдельном
        слое памяти (``rob_box_harness.memory`` / ``SQLiteVoiceMemory``) —
        этот метод их не трогает. Вызывающий код, которому нужно склеить
        и факты, должен ОТДЕЛЬНО вызвать
        ``rob_box_harness.memory.merge_speaker_facts(store, src_id, dst_id)``
        после (или до) вызова этого метода — SpeakerDatabase намеренно не
        знает о MemoryStore, чтобы не размазывать ответственность между
        слоями (голосовая биометрия vs LLM-память).
        """
        if not src_id or not dst_id or src_id == dst_id:
            return 0
        dst_exists = self._conn.execute(
            "SELECT 1 FROM speakers WHERE speaker_id=?", (dst_id,)
        ).fetchone()
        if not dst_exists:
            logger.warning(
                f"merge_speakers: dst={dst_id[:8]} не найден в БД — отмена, "
                f"src={src_id[:8]} не тронут"
            )
            return 0
        cur = self._conn.execute(
            "UPDATE embeddings SET speaker_id=? WHERE speaker_id=?", (dst_id, src_id)
        )
        moved = cur.rowcount
        self._conn.execute("DELETE FROM speakers WHERE speaker_id=?", (src_id,))
        self._conn.commit()
        logger.info(
            f"🔗 merge_speakers: {src_id[:8]} → {dst_id[:8]} ({moved} эмбеддингов перенесено)"
        )
        return moved

    def rename(self, speaker_id: str, new_name: str) -> bool:
        """Rename an existing speaker."""
        cur = self._conn.execute(
            "UPDATE speakers SET name=? WHERE speaker_id=?", (new_name, speaker_id)
        )
        self._conn.commit()
        return cur.rowcount > 0

    def list_speakers(self) -> List[dict]:
        """Return all registered speakers with embedding count.

        Issue #1787 — в выдаче есть ``epithet``/``tags``: без них список
        спикеров бесполезен ровно в том сценарии, ради которого эпитет
        заводился (два одинаковых ``name`` в таблице неразличимы глазом).
        """
        rows = self._conn.execute(
            "SELECT s.speaker_id, s.name, s.created_at, COUNT(e.id) AS emb_count, "
            "s.epithet, s.tags "
            "FROM speakers s LEFT JOIN embeddings e USING (speaker_id) "
            "GROUP BY s.speaker_id ORDER BY s.created_at"
        ).fetchall()
        return [
            {
                "id": r[0],
                "name": r[1],
                "created_at": r[2],
                "embeddings": r[3],
                "epithet": r[4],
                "tags": _split_tags(r[5]),
            }
            for r in rows
        ]

    def delete_speaker(self, speaker_id: str) -> bool:
        """Remove a speaker and all their embeddings."""
        cur = self._conn.execute("DELETE FROM speakers WHERE speaker_id=?", (speaker_id,))
        self._conn.commit()
        return cur.rowcount > 0

    def rename_by_name(self, old_name: str, new_name: str) -> Optional[str]:
        """Find speaker by ``old_name`` and rename to ``new_name``.

        Returns the speaker_id if renamed, None if not found.
        Case-insensitive match (Cyrillic-aware — SQLite ``LOWER()``
        handles only ASCII, so comparison happens in Python);
        picks the most recent match.
        """
        old_lower = (old_name or "").strip().lower()
        if not old_lower:
            logger.info("rename_by_name: empty old_name ignored")
            return None
        rows = self._conn.execute(
            "SELECT speaker_id, name FROM speakers ORDER BY created_at DESC"
        ).fetchall()
        speaker_id = None
        for sid, name in rows:
            if (name or "").strip().lower() == old_lower:
                speaker_id = sid
                break
        if speaker_id is None:
            logger.info(f"rename_by_name: '{old_name}' not found in DB")
            return None
        self._conn.execute(
            "UPDATE speakers SET name=? WHERE speaker_id=?",
            (new_name, speaker_id),
        )
        self._conn.commit()
        logger.info(f"Renamed speaker '{old_name}' → '{new_name}' (id={speaker_id[:8]})")
        return speaker_id

    # ── Эпитеты (issue #1787) ─────────────────────────────────────────────────

    def get_epithet(self, speaker_id: str) -> Optional[str]:
        """Текущая кличка спикера (``None``, если ещё не назначена)."""
        row = self._conn.execute(
            "SELECT epithet FROM speakers WHERE speaker_id=?", (speaker_id,)
        ).fetchone()
        return row[0] if row and row[0] else None

    def get_speaker_profile(self, speaker_id: str) -> Optional[dict]:
        """Полный профиль спикера, включая эпитет, теги и историю.

        Возвращает ``None``, если спикера нет. ``epithet_history`` всегда
        list (битый/пустой JSON → ``[]``: история — диагностика, а не
        источник истины, ронять из-за неё диалог нельзя).
        """
        row = self._conn.execute(
            "SELECT speaker_id, name, created_at, epithet, epithet_history, "
            "tags, sentiment_score, last_epithet_review "
            "FROM speakers WHERE speaker_id=?",
            (speaker_id,),
        ).fetchone()
        if not row:
            return None
        return {
            "speaker_id": row[0],
            "name": row[1],
            "created_at": row[2],
            "epithet": row[3],
            "epithet_history": _load_history(row[4]),
            "tags": _split_tags(row[5]),
            "sentiment_score": row[6],
            "last_epithet_review": row[7],
        }

    def taken_epithets(self, exclude_speaker_id: Optional[str] = None) -> List[str]:
        """Все занятые клички — вход для ``epithets.choose_epithet(taken=…)``.

        Без этого списка словарный слой снова начал бы выдавать одинаковые
        клички тёзкам с общей темой (research §4.2) — именно ради этого
        аргумента там предлагался LLM. ``exclude_speaker_id`` — чтобы при
        ПЕРЕсмотре спикер не считал занятой собственную текущую кличку.
        """
        rows = self._conn.execute(
            "SELECT speaker_id, epithet FROM speakers WHERE epithet IS NOT NULL"
        ).fetchall()
        return [
            r[1] for r in rows if r[1] and r[0] != exclude_speaker_id
        ]

    def set_epithet(self, speaker_id: str, epithet: str, reason: str) -> bool:
        """Назначить кличку и дописать переход в ``epithet_history``.

        Историю ведём всегда (research §4.1: «эпитет версионируется —
        можно откатить и видеть эволюцию»), поэтому здесь же обновляется
        ``last_epithet_review`` — таймер антидребезга пересмотра.

        Возвращает ``False``, если спикер не найден или ``epithet`` пуст.
        """
        epithet = (epithet or "").strip()
        if not speaker_id or not epithet:
            return False
        profile = self.get_speaker_profile(speaker_id)
        if profile is None:
            logger.warning(f"set_epithet: спикер {speaker_id[:8]} не найден")
            return False
        if profile["epithet"] == epithet:
            # Идемпотентность: та же кличка — не плодим записи в истории,
            # но таймер пересмотра сдвигаем (решение «оставить как есть»
            # тоже является пересмотром).
            self._conn.execute(
                "UPDATE speakers SET last_epithet_review=? WHERE speaker_id=?",
                (time.time(), speaker_id),
            )
            self._conn.commit()
            return True

        now = time.time()
        history = profile["epithet_history"]
        history.append(
            {
                "ts": now,
                "old": profile["epithet"],
                "new": epithet,
                "reason": reason,
            }
        )
        self._conn.execute(
            "UPDATE speakers SET epithet=?, epithet_history=?, "
            "last_epithet_review=? WHERE speaker_id=?",
            (epithet, json.dumps(history, ensure_ascii=False), now, speaker_id),
        )
        self._conn.commit()
        logger.info(
            f"🔤 epithet {speaker_id[:8]}: {profile['epithet']!r} → {epithet!r} "
            f"({reason})"
        )
        return True

    def update_speaker_stats(
        self,
        speaker_id: str,
        tags: Optional[List[str]] = None,
        sentiment_score: Optional[float] = None,
    ) -> bool:
        """Обновить темы и валентность речи спикера.

        Метаданные профиля, на которых строится выбор эпитета. Оба поля
        опциональны — вызов без обоих ничего не делает и возвращает
        ``False`` (нет смысла ходить в БД).
        """
        sets: List[str] = []
        params: List[object] = []
        if tags is not None:
            sets.append("tags=?")
            params.append(",".join(t for t in tags if t))
        if sentiment_score is not None:
            sets.append("sentiment_score=?")
            params.append(float(sentiment_score))
        if not sets or not speaker_id:
            return False
        params.append(speaker_id)
        cur = self._conn.execute(
            f"UPDATE speakers SET {', '.join(sets)} WHERE speaker_id=?", params
        )
        self._conn.commit()
        return cur.rowcount > 0

    def close(self) -> None:
        self._conn.close()


# ── Хелперы сериализации профиля (issue #1787) ───────────────────────────────


def _split_tags(raw: Optional[str]) -> List[str]:
    """CSV-строка тегов → список (пустая/``None`` → ``[]``)."""
    if not raw:
        return []
    return [t.strip() for t in str(raw).split(",") if t.strip()]


def _load_history(raw: Optional[str]) -> List[dict]:
    """JSON-история эпитетов → список dict-ов, толерантно к мусору."""
    if not raw:
        return []
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        logger.warning("epithet_history: битый JSON — читаю как пустую историю")
        return []
    return data if isinstance(data, list) else []
