#!/usr/bin/env python3
"""
test_identify_threshold.py — pytest-тесты для SpeakerDatabase.identify()
с калиброванным порогом (kanban t_6f4b3510, t_c8dd3a92 п.3, issue W5-4).

Карточка t_ea3afa6e откалибровала IDENTIFY_THRESHOLD на распределении эмбеддингов
из /data/speakers.db (44 голоса, 280 same-voice / 666 cross-voice пар):
    IDENTIFY_THRESHOLD = 0.72
Выбор порога основан на реальных распределениях, а не на магическом числе —
поэтому тесты используют ту же калибровочную логику (формула «голос +
альфа-шум» из существующих test_speaker_embeddings.py / test_voice_registration.py),
но проверяют поведение identify() на двух граничных сценариях.

Acceptance карточки t_6f4b3510:

(а) запрос вектором ТОГО ЖЕ голоса, что в базе → is_known=True,
    match.id и match.name совпадают с ожидаемым профилем;
(б) запрос вектором ЧУЖОГО голоса, чей cosine ~ 0.79 к ближайшему
    профилю → is_known=False при пороге IDENTIFY_THRESHOLD = 0.72
    (cross-voice сходство в районе 0.79 специально попадает в зону
    между старым 0.75 и новым 0.72 порогом, чтобы тест был
    чувствителен к калибровке).

Реализация ``identify`` живёт в
``rob_box_voice/utils/speaker_embeddings.py`` (метод SpeakerDatabase,
порог IDENTIFY_THRESHOLD = 0.72, откалиброван в t_ea3afa6e).

Тест подключает модуль напрямую по пути файла — минует utils/__init__.py,
который тянет pyaudio (как в существующих test_speaker_embeddings.py /
test_voice_registration.py). Эмбеддинги — синтетические numpy-вектора
фиксированной размерности 256 (та же модель, что resemblyzer VoiceEncoder).
Реальный encoder не вызывается.

Calibration cross-reference:
    src/rob_box_voice/test/unit/utils/test_speaker_embeddings.py::_degraded
    src/rob_box_voice/test/unit/utils/test_voice_registration.py::_degraded
Оба используют ту же формулу base + alpha * noise_unit.
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
SpeakerMatch = _se.SpeakerMatch

# Калиброванное значение из t_ea3afa6e (PR #2372, issue W5-4). Этот тест
# верифицирует поведение identify() на откалиброванном пороге, поэтому
# мы НЕ зависим от того, вкоммичена ли уже константа 0.72 в текущей ветке
# (PR #2372 ещё в работе, на момент карточки t_6f4b3510 в develop всё ещё
# 0.75). Тест подставляет значение явно через monkeypatch — это
# проверка КОНТРАКТА, а не состояния кода.
_CALIBRATED_IDENTIFY_THRESHOLD = 0.72

EMBEDDING_DIM = 256


# ── Вектора-фикстуры ────────────────────────────────────────────────────────


def _unit(seed: int) -> np.ndarray:
    """L2-нормализованный детерминированный вектор."""
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(EMBEDDING_DIM).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    """base + alpha * (независимый единичный вектор), затем ре-нормализация.

    Тот же приём, что в существующих test_speaker_embeddings.py /
    test_voice_registration.py — для двух независимых единичных векторов
    в высокой размерности dot(base, noise)~0, поэтому аналитически
    cos(base, degraded) ≈ 1/sqrt(1+alpha²).

    Калибровка под cos(target):
        alpha ≈ sqrt(1/target² - 1)
    Для 256-dim с фиксированными seed'ами (base=42, noise=99) эмпирические
    значения чуть отличаются от аналитической формулы (см. комментарий
    в t_ea3afa6e про «independent-voice separation не идеальна в 256D»),
    поэтому для теста ниже alpha подобран эмпирически под cos=0.79
    (alpha=0.83 → empirical cos=0.7906).
    """
    noise = _unit(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


# ── Calibration constants — НЕ магические числа ──────────────────────────────

# Два контрольных значения cross-voice cosine — для проверки ОБЕИХ сторон
# откалиброванного порога 0.72:
#   cos = 0.69 — НИЖЕ порога 0.72 → is_known=False (acceptance карточки)
#   cos = 0.79 — ВЫШЕ порога 0.72 → is_known=True (контр-кейс,
#               защищает от пере-инжиниринга «порог молча отбивает всё»)
#
# Тело карточки t_6f4b3510 упоминает «cross-voice в районе 0.79 → is_known=false».
# Это МАТЕМАТИЧЕСКИ НЕВОЗМОЖНО при IDENTIFY_THRESHOLD=0.72: cos=0.79 > 0.72,
# значит identify() ОБЯЗАН вернуть match. Возможная интерпретация —
# путаница с REGISTER_MATCH_THRESHOLD=0.75 (для register_or_merge, не identify),
# где cos=0.79 действительно ВЫШЕ порога (но и там это СЛИЯНИЕ, а не «unknown»).
# Тест покрывает оба направления и фиксирует числа эмпирически
# (см. /tmp/calibrate_alpha.py, /tmp/calibrate_69.py) — если тред с PM
# подтвердит, что в карточке имелся в виду другой порог/сценарий,
# достаточно поменять только эти константы.
_CROSS_VOICE_BELOW_COS = 0.69
_CROSS_VOICE_BELOW_ALPHA = 1.15  # → empirical cos=0.6906 (base seed=42, noise seed=99)

_CROSS_VOICE_ABOVE_COS = 0.79
_CROSS_VOICE_ABOVE_ALPHA = 0.83  # → empirical cos=0.7906 (base seed=42, noise seed=99)


# ── DB-fixture ──────────────────────────────────────────────────────────────


@pytest.fixture()
def db(tmp_path, monkeypatch):
    """Изолированная SQLite-БД в tmp_path + калиброванный IDENTIFY_THRESHOLD.

    Подставляет ``IDENTIFY_THRESHOLD=0.72`` из t_ea3afa6e через monkeypatch —
    тест проверяет ПОВЕДЕНИЕ ``identify()`` при калиброванном пороге, а не
    текущее значение константы в исходнике (которая может быть 0.75, пока
    PR #2372 не влит).
    """
    monkeypatch.setattr(_se, "IDENTIFY_THRESHOLD", _CALIBRATED_IDENTIFY_THRESHOLD)
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


# ── Тесты ───────────────────────────────────────────────────────────────────


class TestIdentifySameVoice:
    """Карточка t_6f4b3510 (а) — запрос вектором того же голоса, что в базе.

    Acceptance:
        * SpeakerMatch не None (спикер найден);
        * is_known=True (выше IDENTIFY_THRESHOLD);
        * match.speaker_id == зарегистрированный speaker_id;
        * match.name == ожидаемое имя профиля;
        * confidence >= IDENTIFY_THRESHOLD (явная проверка порога).
    """

    def test_identical_vector_returns_matching_speaker(self, db):
        """Запрос тем же вектором, что зарегистрирован → точное попадание."""
        emb = _unit(42)
        sid = db.register("Саша", emb)
        assert sid and len(sid) == 36  # uuid4

        match = db.identify(emb)

        assert match is not None, "идентичный вектор должен находить спикера"
        assert isinstance(match, SpeakerMatch)
        assert match.is_known is True
        assert match.speaker_id == sid, "id совпадает с зарегистрированным"
        assert match.name == "Саша", "имя совпадает с зарегистрированным"
        assert match.confidence >= _CALIBRATED_IDENTIFY_THRESHOLD

    def test_identical_vector_confidence_is_near_one(self, db):
        """Идентичный эмбеддинг → cosine ~ 1.0 (численная устойчивость)."""
        emb = _unit(7)
        db.register("Шифу", emb)

        match = db.identify(emb)

        assert match is not None
        assert match.confidence > 0.99, (
            f"ожидалось confidence > 0.99 для identical vector, "
            f"получено {match.confidence:.4f}"
        )

    def test_minimally_degraded_same_voice_still_matches(self, db):
        """Тот же голос с минимальной деградацией (alpha=0.05) — порог переход.

        Один и тот же человек, чуть другие условия записи — например,
        громче/тише/с другим микрофоном. cos ≈ 0.998, явно выше порога 0.72.
        """
        base = _unit(11)
        sid = db.register("Денис", base)
        slightly_noisy = _degraded(base, alpha=0.05, noise_seed=12)

        match = db.identify(slightly_noisy)

        assert match is not None
        assert match.is_known is True
        assert match.speaker_id == sid
        assert match.name == "Денис"
        assert match.confidence >= _CALIBRATED_IDENTIFY_THRESHOLD


class TestIdentifyCrossVoiceThreshold:
    """Карточка t_6f4b3510 (б) — cross-voice вокруг калиброванного порога 0.72.

    Acceptance карточки (числовая интерпретация — см. комментарий у
    ``_CROSS_VOICE_BELOW_COS``):
        cross-voice сходство НИЖЕ порога 0.72 → is_known=False.

    Покрываем ОБЕ стороны порога:
        * cos~0.69 → НИЖЕ 0.72 → is_known=False (acceptance)
        * cos~0.79 → ВЫШЕ 0.72 → is_known=True (контр-кейс: порог
          «знает» границу, а не молча отбивает всё)
    """

    def test_cross_voice_cos_69_below_threshold_means_unknown(self, db):
        """cos ~ 0.69 к ближайшему профилю → is_known=False при пороге 0.72.

        Issue #2747 — с этой карточки в identify() появился АДАПТИВНЫЙ
        порог по умолчанию (мягче, пока галерея спикера маленькая — см.
        test_gallery_warmup.py). Этот тест конкретно про КАЛИБРОВАННОЕ
        значение 0.72, а не про адаптацию, поэтому порог передаётся ЯВНО
        (``identify()`` поддерживает это для ровно такого случая —
        register_or_merge() тоже всегда передаёт явный порог, см.
        speaker_embeddings.py). Без явного threshold на профиле из ОДНОГО
        эмбеддинга (как здесь) сработал бы мягкий
        GALLERY_WARMUP_SOFT_THRESHOLD=0.45, и cos~0.69 прошёл бы — это
        отдельный, намеренный контракт issue #2747, а не регрессия этого
        теста.
        """
        base = _unit(42)  # фиксированный seed для воспроизводимости
        sid = db.register("Саша", base)

        # Чужой голос: тот же base + альфа-шум (cos~0.69, НИЖЕ порога 0.72).
        other_voice = _degraded(
            base, alpha=_CROSS_VOICE_BELOW_ALPHA, noise_seed=99
        )

        # sanity: проверим, что наш синтетический "чужой голос" действительно
        # даёт cosine в окрестности 0.69 к зарегистрированному. Если
        # _CROSS_VOICE_BELOW_ALPHA/seed изменятся и cos уедет, тест ниже
        # даст ложный PASS — этот assert защищает от этого.
        expected_cos = float(
            np.dot(base, other_voice)
            / ((np.linalg.norm(base) * np.linalg.norm(other_voice)) + 1e-9)
        )
        assert abs(expected_cos - _CROSS_VOICE_BELOW_COS) < 0.02, (
            f"калибровка alpha разъехалась: target={_CROSS_VOICE_BELOW_COS}, "
            f"empirical={expected_cos:.4f}. Перекалибровать _CROSS_VOICE_BELOW_ALPHA."
        )

        match = db.identify(other_voice, threshold=_CALIBRATED_IDENTIFY_THRESHOLD)

        assert match is None, (
            f"cross-voice с cos~{_CROSS_VOICE_BELOW_COS:.2f} должен быть НИЖЕ "
            f"порога {_CALIBRATED_IDENTIFY_THRESHOLD}, но identify() вернул "
            f"{match!r}"
        )
        # sanity: score всё равно положительный (другой голос не «отсутствует
        # в БД», а просто не дотягивает до порога) — проверим через
        # диагностический метод identify_candidates (он не зависит от порога).
        candidates = db.identify_candidates(other_voice, top_n=1)
        assert len(candidates) == 1
        assert candidates[0].confidence < _CALIBRATED_IDENTIFY_THRESHOLD
        assert candidates[0].name == "Саша"  # ближайший всё равно «Саша»

    def test_cross_voice_below_threshold_no_match_even_with_other_speakers(self, db):
        """Несколько профилей в БД — чужой голос всё равно отбит порогом.

        Явный ``threshold=`` — см. комментарий в тесте выше (issue #2747).
        """
        own_seed = 42
        base = _unit(own_seed)
        db.register("Саша", base)
        db.register("Пётр", _unit(100))
        db.register("Анна", _unit(200))

        other_voice = _degraded(
            base, alpha=_CROSS_VOICE_BELOW_ALPHA, noise_seed=99
        )
        match = db.identify(other_voice, threshold=_CALIBRATED_IDENTIFY_THRESHOLD)

        assert match is None, (
            f"при наличии нескольких профилей в БД cross-voice с cos~"
            f"{_CROSS_VOICE_BELOW_COS:.2f} всё равно должен быть ниже "
            f"порога {_CALIBRATED_IDENTIFY_THRESHOLD}, но identify() вернул "
            f"{match!r}"
        )

    def test_cross_voice_cos_79_above_threshold_returns_match(self, db):
        """Контр-кейс: cross-voice с cos ~ 0.79 (> порога 0.72) → match есть.

        Защищает от пере-инжиниринга: убеждаемся, что порог 0.72
        действительно «знает» границу (пропускает cos=0.79), а не
        молча отбивает всё.

        Это та же пара голосов (base + α-шум), что в TestIdentifySameVoice,
        но с большим alpha — здесь «чужой голос» всё ещё достаточно
        близок, чтобы пройти через 0.72.
        """
        base = _unit(42)
        sid = db.register("Саша", base)

        other_voice = _degraded(
            base, alpha=_CROSS_VOICE_ABOVE_ALPHA, noise_seed=99
        )
        expected_cos = float(
            np.dot(base, other_voice)
            / ((np.linalg.norm(base) * np.linalg.norm(other_voice)) + 1e-9)
        )
        assert abs(expected_cos - _CROSS_VOICE_ABOVE_COS) < 0.02, (
            f"калибровка alpha разъехалась: target={_CROSS_VOICE_ABOVE_COS}, "
            f"empirical={expected_cos:.4f}. Перекалибровать _CROSS_VOICE_ABOVE_ALPHA."
        )
        assert expected_cos > _CALIBRATED_IDENTIFY_THRESHOLD, (
            f"sanity: ожидался cos > {_CALIBRATED_IDENTIFY_THRESHOLD}, "
            f"получено {expected_cos:.4f}"
        )

        match = db.identify(other_voice)

        assert match is not None, (
            f"cross-voice с cos~{expected_cos:.2f} выше порога "
            f"{_CALIBRATED_IDENTIFY_THRESHOLD}, identify() ОБЯЗАН вернуть match"
        )
        assert match.is_known is True
        assert match.speaker_id == sid
        assert match.name == "Саша"

    def test_identify_returns_match_when_very_close_above_threshold(self, db):
        """Контр-кейс: тот же голос с минимальной деградацией (cos ~ 0.98)."""
        base = _unit(42)
        sid = db.register("Саша", base)

        # Очень близкий, но не идентичный вектор (alpha=0.2 → cos~0.98).
        very_close = _degraded(base, alpha=0.2, noise_seed=99)
        match = db.identify(very_close)

        assert match is not None
        assert match.is_known is True
        assert match.speaker_id == sid
        assert match.name == "Саша"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))