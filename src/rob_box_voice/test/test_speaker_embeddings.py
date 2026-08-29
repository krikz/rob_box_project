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

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_voice"
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
REGISTER_MATCH_THRESHOLD = _se.REGISTER_MATCH_THRESHOLD


def _random_embedding(seed: int = 0, dim: int = 256) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(dim).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    """Тот же голос, но "в других условиях записи" (issue W5-4).

    base + alpha * (независимый случайный единичный вектор), затем
    ре-нормализация — тот же приём, что и в TestIdentify выше, но с
    параметризуемой амплитудой деградации. Для двух независимых единичных
    векторов в 256D dot(base, noise)~0, поэтому аналитически
    cos(base, degraded) ~= 1/sqrt(1+alpha^2):
        alpha=0.3  -> cos~0.96 (чистая запись)
        alpha=0.6  -> cos~0.86 (шум/дистанция)
        alpha=1.0  -> cos~0.71 (сильно деградированная запись)
        alpha=1.4  -> cos~0.58 (почти неузнаваемо)
    Числа подтверждены синтетическим бенчмарком задачи W5-4
    (docs/plans/2026-08-29-wave2-worker-prompts.md, карточка W5-4).
    """
    noise = _random_embedding(noise_seed, dim=len(base))
    v = base + alpha * noise
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


class TestDuplicateVoiceBug:
    """Issue W5-4 — «один голос заводится как два профиля, и они дрейфуют».

    Симптом с робота: один и тот же человек распознавался то как
    «денчик», то как «эйджик» — завелись ДВЕ записи на одного человека.
    Корневая причина: голый ``register()`` (которым раньше безусловно
    пользовался speaker_id_node на каждый вызов LLM-тула
    register_speaker) НИКОГДА не проверял, похож ли голос на уже
    известный профиль — он просто создавал новый speaker_id.
    """

    def test_raw_register_always_creates_new_profile_even_for_same_voice(self, db):
        """Документирует механизм бага: register() без speaker_id слепой.

        Это ПОВЕДЕНИЕ ПО ДИЗАЙНУ register() (используется, когда
        вызывающий код уже знает id) — но именно это поведение,
        применённое speaker_id_node к КАЖДОМУ вызову register_speaker,
        и порождало дубли. register_or_merge() (ниже) — исправление.
        """
        base = _random_embedding(100)
        # Первая фраза — регистрация "Денчик"
        sid1 = db.register("Денчик", _degraded(base, 0.3, 101))
        # Тот же человек, чуть другие условия записи — LLM снова вызывает
        # register_speaker (например, услышал имя иначе): raw register()
        # НЕ проверяет похожесть голоса и создаёт НОВЫЙ профиль.
        sid2 = db.register("Эйджик", _degraded(base, 0.3, 102))

        assert sid1 != sid2, "raw register() всегда создаёт новый id — это и есть баг"
        speakers = db.list_speakers()
        assert len(speakers) == 2, "один голос представлен ДВУМЯ профилями в БД"

    def test_register_or_merge_reuses_existing_profile_for_same_voice(self, db):
        """Красный→зелёный тест исправления: register_or_merge не плодит дубли."""
        base = _random_embedding(200)
        sid1, reused1 = db.register_or_merge("Денчик", _degraded(base, 0.3, 201))
        assert reused1 is False  # первая регистрация — новый профиль, это ок

        # Тот же человек, деградированная запись (шум/дистанция/громкость) —
        # LLM снова вызывает register_speaker с (возможно) другим именем.
        sid2, reused2 = db.register_or_merge("Эйджик", _degraded(base, 0.45, 202))

        assert reused2 is True, "деградированная запись ТОГО ЖЕ голоса должна слиться с профилем"
        assert sid2 == sid1, "не должно появиться второго speaker_id для одного голоса"
        assert len(db.list_speakers()) == 1, "в БД должен остаться ОДИН профиль, а не два"

    def test_register_or_merge_does_not_drift_across_repeated_degraded_utterances(self, db):
        """Симптом «они дальше расходятся»: серия деградированных фраз ОДНОГО
        человека не должна плодить второй, отдельно растущий профиль."""
        base = _random_embedding(300)
        sid, _ = db.register_or_merge("Денчик", _degraded(base, 0.2, 301))
        for i, alpha in enumerate([0.3, 0.35, 0.4, 0.45], start=1):
            next_id, reused = db.register_or_merge("Денчик", _degraded(base, alpha, 400 + i))
            assert reused is True, f"фраза #{i} (alpha={alpha}) создала отдельный профиль"
            assert next_id == sid

        assert len(db.list_speakers()) == 1
        assert db.list_speakers()[0]["embeddings"] == 5  # 1 исходная + 4 деградированные

    def test_register_or_merge_keeps_different_speakers_separate(self, db):
        """Разные люди НЕ должны склеиваться в один профиль (ложное слияние)."""
        sid1, reused1 = db.register_or_merge("Иван", _random_embedding(1))
        sid2, reused2 = db.register_or_merge("Пётр", _random_embedding(2))
        assert reused1 is False
        assert reused2 is False
        assert sid1 != sid2
        assert len(db.list_speakers()) == 2

    def test_register_or_merge_honours_explicit_speaker_id(self, db):
        """Явный speaker_id (например, из rename-потока) обходит проверку похожести."""
        sid = db.register("Саша", _random_embedding(5))
        sid2, reused = db.register_or_merge(
            "Саша", _random_embedding(6), speaker_id=sid
        )
        assert sid2 == sid
        assert reused is False  # explicit path — не "нашли похожего", а "сказали явно"
        assert len(db.list_speakers()) == 1
        assert db.list_speakers()[0]["embeddings"] == 2


class TestIdentifyCandidatesDiagnostics:
    """Issue W5-4 п.4 — диагностика: best_score И второй кандидат."""

    def test_empty_db_returns_empty_list(self, db):
        assert db.identify_candidates(_random_embedding(1)) == []

    def test_returns_top_n_sorted_desc(self, db):
        base = _random_embedding(10)
        db.register("Первый", base)
        db.register("Второй", _degraded(base, 0.5, 11))
        db.register("Третий", _random_embedding(99))  # непохожий голос

        candidates = db.identify_candidates(base, top_n=2)
        assert len(candidates) == 2
        assert candidates[0].confidence >= candidates[1].confidence
        assert candidates[0].name == "Первый"  # точное совпадение — лучший скор

    def test_identify_accepts_custom_threshold(self, db):
        base = _random_embedding(20)
        db.register("Саша", base)
        far = _degraded(base, 1.2, 21)  # ниже 0.75, но выше низкого порога

        assert db.identify(far, threshold=0.3) is not None
        assert db.identify(far, threshold=0.99) is None


class TestMergeSpeakers:
    """Issue W5-4 — merge_speakers(): ручная склейка уже расползшихся дублей."""

    def test_merge_moves_embeddings_and_deletes_src(self, db):
        base = _random_embedding(30)
        dst = db.register("Денчик", base)
        src = db.register("Эйджик", _degraded(base, 0.4, 31))
        db.register("Эйджик", _degraded(base, 0.5, 32), speaker_id=src)  # 2 embeddings on src

        moved = db.merge_speakers(src, dst)

        assert moved == 2
        speakers = {s["id"]: s for s in db.list_speakers()}
        assert src not in speakers, "src должен быть удалён после слияния"
        assert dst in speakers
        assert speakers[dst]["embeddings"] == 3  # 1 исходный + 2 перенесённых
        assert speakers[dst]["name"] == "Денчик"  # имя dst не меняется

    def test_merged_voice_now_identifies_as_dst(self, db):
        base = _random_embedding(40)
        dst = db.register("Денчик", base)
        src = db.register("Эйджик", _degraded(base, 0.3, 41))

        db.merge_speakers(src, dst)

        match = db.identify(_degraded(base, 0.3, 42))
        assert match is not None
        assert match.speaker_id == dst
        assert match.name == "Денчик"

    def test_merge_noop_when_src_equals_dst(self, db):
        sid = db.register("Саша", _random_embedding(1))
        assert db.merge_speakers(sid, sid) == 0
        assert len(db.list_speakers()) == 1

    def test_merge_noop_when_dst_missing(self, db):
        src = db.register("Саша", _random_embedding(1))
        assert db.merge_speakers(src, "no-such-id") == 0
        # src НЕ должен быть тронут при неудачном слиянии
        assert len(db.list_speakers()) == 1
        assert db.list_speakers()[0]["id"] == src

    def test_merge_noop_when_src_missing(self, db):
        dst = db.register("Саша", _random_embedding(1))
        assert db.merge_speakers("no-such-id", dst) == 0
        assert len(db.list_speakers()) == 1


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
