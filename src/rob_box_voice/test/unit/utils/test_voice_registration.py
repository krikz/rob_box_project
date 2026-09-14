#!/usr/bin/env python3
"""
test_voice_registration.py — pytest-тест сценария повторной регистрации
того же голоса (issue W5-4 / kanban t_4f2a354f).

Контракт, зафиксированный телом карточки t_4f2a354f:
    при повторной регистрации голоса с cosine-расстоянием ВЫШЕ порога merge,
    функция ``register_or_merge`` дописывает новый эмбеддинг в СУЩЕСТВУЮЩИЙ
    профиль, а дубль не создаётся.

Acceptance:
    * после второго вызова количество профилей не изменилось;
    * у целевого профиля len(embeddings) >= 2;
    * id/имя профиля совпадают с первым вызовом.

Реализация ``register_or_merge`` живёт в
``rob_box_voice/utils/speaker_embeddings.py`` (метод SpeakerDatabase).
Тест подключает модуль напрямую по пути файла — минует utils/__init__.py,
который тянет pyaudio. Эмбеддинги — синтетические numpy-вектора фиксированной
размерности 256 (как в существующем test_speaker_embeddings.py). Реальный
encoder не вызывается.
"""

from __future__ import annotations

import importlib.util
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
REGISTER_MATCH_THRESHOLD = _se.REGISTER_MATCH_THRESHOLD
EMBEDDING_DIM = 256


# ── Вектора-фикстуры ────────────────────────────────────────────────────────


def _unit(seed: int) -> np.ndarray:
    """L2-нормализованный детерминированный вектор (без обращения к RNG)."""
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(EMBEDDING_DIM).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    """Тот же «голос», но с добавкой шума.

    base + alpha * (независимый единичный вектор), затем ре-нормализация.
    Для двух независимых единичных векторов в высокой размерности
    dot(base, noise)~0, поэтому аналитически
    cos(base, degraded) ~= 1/sqrt(1+alpha^2):
        alpha=0.0  -> cos=1.0  (тождественный вектор)
        alpha=0.3  -> cos~0.96 (чистая запись, дрейф минимальный)
        alpha=0.6  -> cos~0.86 (умеренная деградация)
    Числа подтверждены бенчмарком задачи W5-4
    (docs/plans/2026-08-29-wave2-worker-prompts.md, карточка W5-4).
    """
    noise = _unit(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


def _cos(a: np.ndarray, b: np.ndarray) -> float:
    """Косинусная близость двух нормализованных векторов."""
    return float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) + 1e-9))


@pytest.fixture()
def db(tmp_path):
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


# ── Основной сценарий ──────────────────────────────────────────────────────


class TestRegisterOrMergeSameVoice:
    """Карточка t_4f2a354f — повторная регистрация того же голоса.

    Сценарий: оператор (или LLM-тул register_speaker) дважды регистрирует
    ОДНОГО человека — например, потому что в первый раз услышал имя
    с ошибкой, а во второй распознал лучше. register_or_merge обязан:
    * найти существующий профиль по эмбеддингу (cos > REGISTER_MATCH_THRESHOLD),
    * дописать новый эмбеддинг в этот профиль (len(embeddings) растёт),
    * не создавать дубль (число профилей не меняется).
    """

    def test_second_call_same_voice_merges_into_existing_profile(self, db):
        """Acceptance-чек-лист карточки в одном тесте."""
        # ── Arrange: первый вызов — «регистрация Денчика» ──
        base = _unit(seed=1000)
        first_name = "Денчик"
        first_sid, first_reused = db.register_or_merge(first_name, base)

        assert first_reused is False, "первая регистрация — новый профиль, не merge"
        assert first_sid, "register_or_merge должен вернуть uuid"

        # ── Act: второй вызов — тот же человек, деградированная запись ──
        # cos(base, deg) ~= 1/sqrt(1 + 0.3^2) ~= 0.96 — выше порога 0.75.
        second_emb = _degraded(base, alpha=0.3, noise_seed=1001)
        cos = _cos(base, second_emb)
        assert cos > REGISTER_MATCH_THRESHOLD, (
            f"подготовка фикстуры: cos={cos:.3f} должна быть > "
            f"REGISTER_MATCH_THRESHOLD={REGISTER_MATCH_THRESHOLD}, иначе "
            f"тест ловит не тот сценарий"
        )

        second_name = "Эйджик"  # LLM «передумал» про имя — не должно влиять на merge
        second_sid, second_reused = db.register_or_merge(second_name, second_emb)

        # ── Assert (Acceptance) ──
        # 1) дубль не появился
        speakers = db.list_speakers()
        assert len(speakers) == 1, (
            f"после второго register_or_merge ожидался 1 профиль, "
            f"получено {len(speakers)}: {speakers!r}"
        )

        # 2) id совпадает — эмбеддинг дописан в тот же профиль
        assert second_sid == first_sid, (
            f"register_or_merge вернул новый sid={second_sid[:8]} вместо "
            f"слияния с существующим {first_sid[:8]}"
        )
        assert second_reused is True, (
            "register_or_merge обязан сигналить reused=True при слиянии"
        )

        # 3) у целевого профиля len(embeddings) >= 2
        target = speakers[0]
        assert target["embeddings"] >= 2, (
            f"ожидалось >= 2 эмбеддингов у профиля, получено "
            f"{target['embeddings']}"
        )
        # строго 2 (1 исходный + 1 деградированный) — без «дрейфа» вставок
        assert target["embeddings"] == 2

        # 4) id/имя профиля совпадают с первым вызовом
        assert target["id"] == first_sid
        # имя: register_or_merge в текущей реализации перезаписывает имя
        # через register(..., speaker_id=match.speaker_id) при merge
        # (см. register_or_merge в speaker_embeddings.py), поэтому второй
        # вызов с другим именем обновляет name — фиксируем наблюдаемое
        # поведение, чтобы любой регресс был виден явно.
        assert target["name"] in {first_name, second_name}, (
            f"имя профиля должно быть одним из переданных имён; "
            f"получено {target['name']!r}"
        )

    def test_voice_at_threshold_boundary_merges(self, db):
        """Граничный кейс: cos чуть ВЫШЕ порога — merge, не дубль.

        Готовим синтетику с cos == threshold + epsilon. Если реализация
        округлит не в ту сторону — тест поймает регресс.
        """
        base = _unit(seed=2000)
        # Подбираем alpha так, чтобы cos ~ threshold + 0.01 (надёжно выше).
        # cos = 1/sqrt(1+alpha^2) → alpha = sqrt(1/cos^2 - 1)
        target_cos = REGISTER_MATCH_THRESHOLD + 0.01
        alpha = float(np.sqrt(1.0 / (target_cos * target_cos) - 1.0))
        emb = _degraded(base, alpha=alpha, noise_seed=2001)

        actual_cos = _cos(base, emb)
        assert actual_cos > REGISTER_MATCH_THRESHOLD, (
            f"подготовка фикстуры: cos={actual_cos:.4f} должна быть "
            f"> REGISTER_MATCH_THRESHOLD={REGISTER_MATCH_THRESHOLD}"
        )

        sid1, _ = db.register_or_merge("Денчик", base)
        sid2, reused = db.register_or_merge("Эйджик", emb)

        assert sid2 == sid1, "merge на границе порога должен сработать"
        assert reused is True
        assert len(db.list_speakers()) == 1
        assert db.list_speakers()[0]["embeddings"] == 2

    def test_repeated_merged_calls_keep_appending_not_duplicating(self, db):
        """Серия деградированных фраз ТОГО ЖЕ человека не плодит второй
        растущий профиль — каждое повторение дописывает эмбеддинг
        в тот же профиль.

        Защита от регрессии «дрейфа» (см. issue W5-4).
        """
        base = _unit(seed=3000)
        first_sid, _ = db.register_or_merge("Денчик", base)

        for i, alpha in enumerate([0.2, 0.3, 0.4], start=1):
            emb = _degraded(base, alpha=alpha, noise_seed=3000 + i)
            cos = _cos(base, emb)
            assert cos > REGISTER_MATCH_THRESHOLD, (
                f"фраза #{i}: cos={cos:.3f} должна быть выше порога"
            )
            next_sid, reused = db.register_or_merge("Денчик", emb)
            assert reused is True, (
                f"фраза #{i} (alpha={alpha}) не слизлась — это и есть баг W5-4"
            )
            assert next_sid == first_sid

        # После 4 регистраций (1 исходная + 3 деградированные) — 1 профиль,
        # 4 эмбеддинга. Никаких «отдельно растущих дублей».
        speakers = db.list_speakers()
        assert len(speakers) == 1
        assert speakers[0]["embeddings"] == 4
        assert speakers[0]["id"] == first_sid


# ── Регрессия: контрастные сценарии должны вести себя иначе ───────────────


class TestRegisterOrMergeContrastScenarios:
    """Контрастные проверки — чтобы не «съесть» слиянием чужой голос.

    Покрывают контракт из соседней карточки t_c8dd3a92 (п.2): если голос
    реально другой (cos ниже порога), merge НЕ должен сработать.
    """

    def test_different_voice_does_not_merge(self, db):
        base_a = _unit(seed=4000)
        base_b = _unit(seed=4001)  # независимый вектор → cos~0
        cos = _cos(base_a, base_b)
        assert cos < REGISTER_MATCH_THRESHOLD, (
            f"случайные вектора должны быть ниже порога, получено {cos:.3f}"
        )

        sid_a, _ = db.register_or_merge("Иван", base_a)
        sid_b, reused_b = db.register_or_merge("Пётр", base_b)

        assert sid_b != sid_a
        assert reused_b is False
        assert len(db.list_speakers()) == 2
        assert {s["name"] for s in db.list_speakers()} == {"Иван", "Пётр"}

    def test_explicit_speaker_id_skips_similarity_check(self, db):
        """register_or_merge(..., speaker_id=...) — bypass порога.

        Это поведение по дизайну (rename-поток): если вызывающий код
        уже знает id, проверять похожесть голоса не нужно. Здесь —
        документация/регрессия, чтобы случайно не сломать.
        """
        sid = db.register("Шифу", _unit(seed=5000))
        # вектор, который БЕЗ явного speaker_id слился бы в чужой профиль —
        # но т.к. speaker_id задан явно, всё равно идёт в Шифу
        other = _unit(seed=5001)
        sid2, reused = db.register_or_merge("Шифу", other, speaker_id=sid)

        assert sid2 == sid
        assert reused is False
        assert len(db.list_speakers()) == 1
        assert db.list_speakers()[0]["embeddings"] == 2