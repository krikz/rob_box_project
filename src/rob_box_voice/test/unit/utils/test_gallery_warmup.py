#!/usr/bin/env python3
"""
test_gallery_warmup.py — растущая галерея БЕЗ адаптивного порога (issue #2747).

ИСТОРИЯ (важно для контекста тестов ниже): первая версия правки issue #2747
вводила ``adaptive_identify_threshold()`` — мягкий порог 0.45 на профиле из
одного эмбеддинга, линейно ужесточающийся до калиброванных 0.72. Она была
ОТКЛОНЕНА в PR #2757 после проверки на реальных данных робота
(``speakers.db.bak-20260921T224624Z``, 44 профиля, 45 эмбеддингов, тот же
пайплайн и микрофон, попарные косинусы):

    ОДНО имя (один человек):  n=277  min=0.329 p50=0.786 p90=0.903
                               доля >= 0.45: 92.4%   доля >= 0.72: 57.0%
    РАЗНЫЕ имена (чужие):     n=669  min=0.293 p50=0.536 p90=0.658
                               доля >= 0.45: 90.0%   доля >= 0.72:  4.9%

При пороге 0.45 совпадает 90% ЧУЖИХ пар. Сведя с замером живого голоса
issue #2747 (один человек против своего профиля: 0.283-0.584, медиана
чужих пар здесь: 0.536) — распределения «свой»/«чужой» лежат друг на
друге, никакое значение порога их не разделяет.

Оговорка о данных: 44 профиля бэкапа — в основном синтетические e2e-дикторы
(TTS звучит однообразнее живых людей), поэтому «90% чужих» — вероятно
ВЕРХНЯЯ оценка ложного приёма. Нижней оценки (косинус между двумя РЕАЛЬНЫМИ
людьми на этом микрофоне) пока нет ни у кого — не измерено на живом
роботе (стенд занят E2E).

Что тестируется здесь:
    1. ``identify()`` БЕЗ явного ``threshold=`` — всегда калиброванный
       ``IDENTIFY_THRESHOLD``, НЕ зависит от размера галереи (регрессионный
       тест на измеренных в issue #2747 числах: одиночный эталон + живая
       вторая реплика по-прежнему НЕ узнаётся — это ожидаемо, проблема
       решается не здесь, а session-anchor growth в speaker_id_node).
    2. ``append_reference_embedding()`` — чисто численный guard на потолок
       GALLERY_WARMUP_SIZE, БЕЗ проверки похожести голоса (это сознательно:
       решение "чей это голос" принимает вызывающий код по внешнему
       свидетельству — см. test_register_publishes_result.py для
       session-anchor логики на уровне ноды).

Модуль подключается напрямую по пути файла — минует utils/__init__.py
(тянет pyaudio), тот же приём, что в test_speaker_embeddings.py /
test_identify_threshold.py. Эмбеддинги — синтетические numpy-вектора,
resemblyzer не вызывается.
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

EMBEDDING_DIM = 256


def _unit(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(EMBEDDING_DIM).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    """base + alpha * (независимый единичный вектор), ре-нормализация.

    cos(base, degraded) ~= 1/sqrt(1+alpha^2) — тот же приём, что в
    остальных тестах биометрии (см. test_speaker_embeddings.py).
    """
    noise = _unit(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path, monkeypatch):
    """Изолированная БД с ДЕФОЛТНЫМИ (не монки-патченными под другой тест)
    константами — явный monkeypatch, чтобы тест не зависел от порядка
    выполнения других test_*.py, которые тоже патчат модульные атрибуты
    speaker_embeddings."""
    monkeypatch.setattr(_se, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(_se, "GALLERY_WARMUP_SIZE", 5)
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


class TestIdentifyIsNeverAdaptive:
    """identify() без явного threshold= — регрессионный барьер против
    повторного введения адаптации по gallery_size (issue #2747, PR #2757)."""

    def test_single_reference_live_second_utterance_stays_unknown(self, db):
        """Регрессионный тест на ИЗМЕРЕННЫХ числах issue #2747.

        Профиль с ОДНИМ эталонным эмбеддингом (как сразу после
        register_speaker) и вторая живая реплика с cos~0.523 к эталону
        (живой лог, 22.09.2026) — калиброванный порог 0.72 её НЕ пропускает.
        Это ОЖИДАЕМОЕ поведение после отката адаптивного порога: задача
        «узнать следующую реплику» решается НЕ здесь (session-anchor
        growth в speaker_id_node решает другую задачу — растит галерею по
        непрерывности сессии, не публикует is_known=true по акустике).
        """
        base = _unit(1)
        db.register("Деньчик", base)

        # cos = 1/sqrt(1+alpha^2) = 0.523 → alpha = sqrt(1/0.523^2 - 1) ~= 1.625
        alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
        second_utterance = _degraded(base, alpha=alpha, noise_seed=2)
        actual_cos = float(np.dot(base, second_utterance))
        assert 0.45 < actual_cos < 0.72, (
            "фикстура должна лежать МЕЖДУ старым мягким порогом и "
            f"калиброванным (issue #2747 измерение), получено {actual_cos:.3f}"
        )

        match = db.identify(second_utterance)

        assert match is None, (
            "калиброванный identify_threshold=0.72 НЕ должен пропускать "
            f"cos~{actual_cos:.3f} на профиле из одного эмбеддинга — "
            "это и есть причина, по которой рост галереи не может "
            "опираться на identify(), см. GALLERY_WARMUP_SIZE"
        )

    def test_identical_vector_still_matches(self, db):
        """Sanity: калибровка не сломала обычный happy-path (cos~1.0)."""
        base = _unit(10)
        sid = db.register("Саша", base)

        match = db.identify(base)

        assert match is not None
        assert match.speaker_id == sid

    def test_explicit_threshold_still_works(self, db):
        """identify(embedding, threshold=X) — явный порог как и раньше
        (register_or_merge() полагается именно на это, ADR-0127)."""
        base = _unit(20)
        db.register("Саша", base)
        alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
        emb = _degraded(base, alpha=alpha, noise_seed=21)

        assert db.identify(emb) is None  # калиброванный порог — не проходит
        assert db.identify(emb, threshold=0.3) is not None  # явный низкий — проходит


class TestAppendReferenceEmbedding:
    """append_reference_embedding() — потолок роста, БЕЗ проверки похожести."""

    def test_appends_while_below_cap(self, db):
        base = _unit(40)
        sid = db.register("Деньчик", base)
        assert db.gallery_size(sid) == 1

        grown = db.append_reference_embedding(sid, "Деньчик", _degraded(base, 0.3, 41))

        assert grown is True
        assert db.gallery_size(sid) == 2

    def test_stops_at_cap(self, db):
        base = _unit(50)
        sid = db.register("Деньчик", base)
        for i in range(1, 5):  # доводим галерею до 5 (GALLERY_WARMUP_SIZE)
            db.register("Деньчик", _degraded(base, 0.1, 500 + i), speaker_id=sid)
        assert db.gallery_size(sid) == 5

        grown = db.append_reference_embedding(sid, "Деньчик", _degraded(base, 0.1, 999))

        assert grown is False
        assert db.gallery_size(sid) == 5, "галерея не должна расти после потолка"

    def test_does_not_check_similarity_at_all(self, db):
        """КЛЮЧЕВОЙ инвариант новой версии: метод не смотрит на cosine —
        решение «тот ли это человек» полностью на вызывающем коде
        (session-anchor в speaker_id_node), не на acoustics здесь.

        Совершенно ортогональный (чужой) вектор дописывается так же
        охотно, как почти идентичный — это ОЖИДАЕМО и документирует,
        почему вызывающий код обязан сам гарантировать анкер (issue
        #2747 / PR #2757 ревью)."""
        base = _unit(60)
        sid = db.register("Деньчик", base)
        orthogonal = _unit(9999)  # независимый вектор, cos~0

        grown = db.append_reference_embedding(sid, "Деньчик", orthogonal)

        assert grown is True
        assert db.gallery_size(sid) == 2

    def test_repeated_calls_converge_to_cap_not_beyond(self, db):
        base = _unit(70)
        sid = db.register("Деньчик", base)

        for i in range(1, 10):  # намного больше, чем нужно для потолка
            db.append_reference_embedding(sid, "Деньчик", _degraded(base, 0.2, 700 + i))

        assert db.gallery_size(sid) == 5


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
