#!/usr/bin/env python3
"""
test_gallery_warmup.py — растущая галерея + адаптивный порог (issue #2747).

Контекст (issue #2747, замер на живом роботе 22.09.2026): после
``register_speaker`` профиль диктора содержит РОВНО ОДИН эмбеддинг.
Восемь следующих живых реплик того же человека против этого единственного
эталона дали cosine 0.283 / 0.523 / 0.531 / 0.584 / 0.463 / 0.473 / 0.457 /
0.513 — то есть КАЖДАЯ реплика оставалась ниже IDENTIFY_THRESHOLD=0.72,
диктор навсегда перестаёт узнаваться сразу после регистрации.

Тесты здесь проверяют исправление:
    1. ``adaptive_identify_threshold()`` — порог как функция размера
       галереи (мягкий на 1 эмбеддинге, калиброванный на
       GALLERY_WARMUP_SIZE и больше, линейная интерполяция между ними).
    2. ``identify()`` без явного ``threshold=`` использует эту адаптацию.
    3. ``grow_gallery_if_warming_up()`` — дописывает эмбеддинг в профиль,
       пока галерея маленькая, и перестаёт, когда она выросла до
       GALLERY_WARMUP_SIZE.
    4. Явный ``threshold=`` (как у register_or_merge/REGISTER_MATCH_THRESHOLD)
       по-прежнему ПОЛНОСТЬЮ обходит адаптацию — ADR-0127 не должен
       смягчаться маленькой галереей.

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
adaptive_identify_threshold = _se.adaptive_identify_threshold

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
    GALLERY_WARMUP_* — явный monkeypatch на модульные значения issue #2747,
    чтобы тест не зависел от порядка выполнения других test_*.py, которые
    тоже патчат модульные атрибуты speaker_embeddings."""
    monkeypatch.setattr(_se, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(_se, "GALLERY_WARMUP_SIZE", 5)
    monkeypatch.setattr(_se, "GALLERY_WARMUP_SOFT_THRESHOLD", 0.45)
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


class TestAdaptiveIdentifyThreshold:
    """Чистая функция — без БД, только арифметика интерполяции."""

    def test_single_embedding_uses_soft_floor(self):
        assert adaptive_identify_threshold(1) == pytest.approx(0.45)

    def test_zero_gallery_also_uses_soft_floor(self):
        """Защита от краевого случая (не должно возникать в проде, но
        функция обязана не упасть и не выдать сумасшедшее число)."""
        assert adaptive_identify_threshold(0) == pytest.approx(0.45)

    def test_warmup_size_and_above_uses_calibrated_threshold(self):
        assert adaptive_identify_threshold(5) == pytest.approx(0.72)
        assert adaptive_identify_threshold(6) == pytest.approx(0.72)
        assert adaptive_identify_threshold(100) == pytest.approx(0.72)

    def test_threshold_is_monotonically_non_decreasing(self):
        thresholds = [adaptive_identify_threshold(n) for n in range(1, 8)]
        assert thresholds == sorted(thresholds), (
            f"порог обязан только расти с размером галереи: {thresholds}"
        )

    def test_intermediate_sizes_interpolate_linearly(self):
        # GALLERY_WARMUP_SIZE=5, soft=0.45, calibrated=0.72 → шаг (0.72-0.45)/4=0.0675
        assert adaptive_identify_threshold(2) == pytest.approx(0.5175, abs=1e-4)
        assert adaptive_identify_threshold(3) == pytest.approx(0.585, abs=1e-4)
        assert adaptive_identify_threshold(4) == pytest.approx(0.6525, abs=1e-4)


class TestIdentifyUsesAdaptiveThresholdByDefault:
    """identify() без явного threshold= — issue #2747 acceptance."""

    def test_single_reference_live_voice_scores_are_now_recognized(self, db):
        """Регрессионный тест на ИЗМЕРЕННЫХ числах issue #2747.

        Профиль с ОДНИМ эталонным эмбеддингом (как сразу после
        register_speaker) и вторая живая реплика с cos~0.523 к эталону
        (второе измерение из живого лога, 22.09.2026) — раньше (порог
        константой 0.72) была бы unknown НАВСЕГДА. С адаптивным порогом
        (0.45 при gallery_size=1) она узнаётся.
        """
        base = _unit(1)
        db.register("Деньчик", base)

        # cos = 1/sqrt(1+alpha^2) = 0.523 → alpha = sqrt(1/0.523^2 - 1) ~= 1.625
        # Аналитическая формула cos~=1/sqrt(1+alpha^2) — приближение (dot
        # base/noise не строго 0 в 256D, см. комментарий в
        # test_identify_threshold.py про "independent-voice separation не
        # идеальна в 256D"); допуск пошире, чем для целевого значения —
        # важно, что итог лежит в зоне между мягким (0.45) и калиброванным
        # (0.72) порогом, как и измеренное issue #2747 значение 0.523.
        alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
        second_utterance = _degraded(base, alpha=alpha, noise_seed=2)
        actual_cos = float(np.dot(base, second_utterance))
        assert abs(actual_cos - 0.523) < 0.05, (
            f"фикстура должна давать cos~0.523 (issue #2747 измерение), "
            f"получено {actual_cos:.3f}"
        )
        assert 0.45 < actual_cos < 0.72, (
            "фикстура должна лежать МЕЖДУ мягким и калиброванным порогом, "
            f"получено {actual_cos:.3f}"
        )

        match = db.identify(second_utterance)

        assert match is not None, (
            "issue #2747: живая вторая реплика (cos~0.523 к единственному "
            "эталону) обязана узнаваться при адаптивном пороге "
            f"({adaptive_identify_threshold(1)}), а не оставаться unknown"
        )
        assert match.name == "Деньчик"
        assert match.gallery_size == 1

    def test_first_utterance_right_after_registration_may_stay_unknown(self, db):
        """Самая первая реплика (cos~0.283, issue #2747) — НАМЕРЕННО ниже
        даже мягкого порога 0.45: она снята практически одновременно с
        эталоном и слишком зашумлена, чтобы доверять ей."""
        base = _unit(10)
        db.register("Деньчик", base)

        alpha = float((1.0 / 0.283 ** 2 - 1.0) ** 0.5)
        very_first = _degraded(base, alpha=alpha, noise_seed=11)
        actual_cos = float(np.dot(base, very_first))
        assert abs(actual_cos - 0.283) < 0.03

        match = db.identify(very_first)
        assert match is None

    def test_explicit_threshold_bypasses_adaptation(self, db):
        """register_or_merge()-style явный порог игнорирует размер галереи
        (ADR-0127 не должен смягчаться маленькой галереей)."""
        base = _unit(20)
        db.register("Саша", base)
        # cos~0.523 — прошёл бы адаптивный (0.45), но НЕ должен проходить
        # явный калиброванный 0.72.
        alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
        emb = _degraded(base, alpha=alpha, noise_seed=21)

        assert db.identify(emb) is not None  # адаптивный порог — matches
        assert db.identify(emb, threshold=0.72) is None  # явный — не matches

    def test_grown_gallery_uses_stricter_threshold(self, db):
        """После роста до GALLERY_WARMUP_SIZE порог — уже калиброванный, и
        такая же cos~0.523 реплика больше НЕ узнаётся по единственному
        совпадению — нужен скор повыше (это ожидаемое ужесточение)."""
        base = _unit(30)
        sid = db.register("Саша", base)
        # Дозаполняем галерею близкими образцами до GALLERY_WARMUP_SIZE.
        for i, alpha in enumerate([0.05, 0.06, 0.07, 0.08], start=1):
            db.register("Саша", _degraded(base, alpha, 300 + i), speaker_id=sid)
        assert db.gallery_size(sid) == 5

        alpha_523 = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
        weak = _degraded(base, alpha=alpha_523, noise_seed=99)
        match = db.identify(weak)
        assert match is None, (
            "галерея из 5 эмбеддингов использует калиброванный порог 0.72 — "
            "cos~0.52 больше не должен проходить"
        )


class TestGrowGalleryIfWarmingUp:
    """grow_gallery_if_warming_up() — дозапись эмбеддинга, issue #2747."""

    def test_appends_embedding_while_below_warmup_size(self, db):
        base = _unit(40)
        sid = db.register("Деньчик", base)
        assert db.gallery_size(sid) == 1

        match = db.identify(_degraded(base, 0.3, 41))
        assert match is not None
        assert match.gallery_size == 1

        grown = db.grow_gallery_if_warming_up(match, _degraded(base, 0.3, 41))
        assert grown is True
        assert db.gallery_size(sid) == 2

    def test_stops_appending_once_warmup_size_reached(self, db):
        base = _unit(50)
        sid = db.register("Деньчик", base)
        for i in range(1, 5):  # доводим галерею до 5 (WARMUP_SIZE)
            db.register("Деньчик", _degraded(base, 0.1, 500 + i), speaker_id=sid)
        assert db.gallery_size(sid) == 5

        match = db.identify(base)  # идентичный вектор — точно найдётся
        assert match is not None
        assert match.gallery_size == 5

        grown = db.grow_gallery_if_warming_up(match, _degraded(base, 0.1, 999))
        assert grown is False
        assert db.gallery_size(sid) == 5, "галерея не должна расти бесконечно"

    def test_repeated_growth_converges_to_warmup_size(self, db):
        """Сквозной сценарий: серия узнанных реплик растит галерею РОВНО до
        GALLERY_WARMUP_SIZE и не дальше — имитирует
        speaker_id_node._process_utterance."""
        base = _unit(60)
        sid = db.register("Деньчик", base)

        for i in range(1, 10):  # намного больше, чем нужно для warmup
            match = db.identify(_degraded(base, 0.2, 600 + i))
            if match is None:
                continue
            db.grow_gallery_if_warming_up(match, _degraded(base, 0.2, 600 + i))

        assert db.gallery_size(sid) == 5


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
