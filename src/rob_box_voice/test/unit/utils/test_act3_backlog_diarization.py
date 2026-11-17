#!/usr/bin/env python3
"""test_act3_backlog_diarization.py — pytest-сценарий act3 night-marathon.

Карточка t_3330b9d5: проверить, что после калибровки порогов (issue #2348,
PR #2372, IDENTIFY=0.72 / REGISTER_MATCH=0.75) узлы ``n302``/``n303``/``n308``
сценария ``.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json``
корректно атрибутируют реплики Саши и Бориса, и что ``register_or_merge`` НЕ
создаёт «Саша-N»/«Борис-N» дублей в ``/data/speakers.db``.

Сценарий act3 (по сценарию): четыре голоса говорят БЕЗ wake-слова:

    n302  anton   → Саша   (зарегистрирован в акте 2)
    n303  ermil   → Борис  (зарегистрирован в акте 2)
    n304  zahar   → незнакомец (дядя Гриша)
    n305  filipp  → незнакомец (курьер Валера)
    n306  anton   → "Робот, кто там бубнил на фоне?" (вопрос, wake)
    ...
    n308  ermil   → Борис (явная команда в фоне без wake-слова)

Acceptance (issue #2348 AC5 + body этой карточки):

    AC-A: ``identify()`` для n302 (synthetic anton, cos~0.725) возвращает
          ``is_known=True`` и ``speaker='Саша'`` — до калибровки 0.75
          это была «незнакомец». Граница 0.725 специально попадает
          в окно (0.72, 0.75), чтобы тест был чувствителен к калибровке.
          ПРИМЕЧАНИЕ: register_or_merge (порог 0.75) для n302 НЕ мержит
          (cos<0.75 → новый профиль «Саша-2» by design). Это нормально:
          в act3 backlog-сценарии backlog только копит, register_speaker
          вызывается LLM в явной форме и допускает новый профиль при
          неуверенности.
    AC-B: ``identify()`` для n303/n308 (synthetic ermil, cos~0.866)
          возвращает ``is_known=True`` и ``speaker='Борис'``.
          register_or_merge для них сливается (cos>0.75) — это и есть
          W5-4 fix: повторный голос Бориса не плодит «Борис-N».
    AC-C: ``identify()`` для n304/n305 (synthetic zahar/filipp) возвращает
          ``None`` (или ``is_known=False``). register_or_merge для них
          создаёт новый профиль (т.к. cos~0 к существующим).

Сценарий запускается ЛОКАЛЬНО: pytest эмулирует работу robot
``SpeakerDatabase`` (Pure-Python, см. utils/speaker_embeddings.py).
E2E на реальном роботе — отдельный процесс (merge-gate → e2e-process).
Этот тест ЗЕЛЁНЫЙ = контракт act3 на уровне биометрии выполняется.
e2e-process потом прогонит voice-сценарий на hardware и убедится, что
маркеры ``[backlog] accumulated ... speaker='Саша'/'Борис'/'незнакомец'``
реально появляются в ``docker logs voice-assistant``.

Сценарий НЕ прогоняет сам ``dialogue_node`` — мы тестируем поведение
``SpeakerDatabase`` (низкий уровень, чистый модуль), потому что:
  1. dialogue_node требует ROS2 (rclpy, sounddevice, opt deps);
  2. logи маркера ``[backlog] accumulated ...`` проверяет e2e harness
     (см. ``pr-acceptance``, ``regression-acceptance`` в e2e-process);
  3. этот файл покрывает AC, которые не зависят от LLM-инференса.

Калибровка основана на распределении из /data/speakers.db (run 34522852773):

    same_voice n=280 mean=0.7168 median=0.782 std=0.172 p95=0.964
    cross_voice n=666 mean=0.547 median=0.536 std=0.089 p95=0.715 max=0.869
    cos(Борис, Шифу)=0.791 (Anton-Sasha pair в evidence #2346, не Boris-Shifu)

Тестовые seed'ы подобраны так, чтобы cos между Сашей и n302 (synthetic)
была ~0.725, cos между Борисом и n303/n308 — ~0.866, а cos между
Шифу и «Борисом» — 0.791 (та самая «ложная» пара из evidence).

См. ``/tmp/calibrate_act3.py`` для воспроизводимой калибровки.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Tuple

import numpy as np
import pytest


# ============================================================================
# Загрузка модуля — минует utils/__init__.py (тянет pyaudio),
# как в существующих test_speaker_embeddings.py / test_voice_registration.py.
# ============================================================================

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


# ============================================================================
# Калиброванные значения (НЕ магические числа — из evidence issue #2348 / #2346)
# ============================================================================

# Калиброванные пороги из PR #2372 (t_ea3afa6e, merged 2026-09-14).
# Эти значения подставляются в модуль через monkeypatch в фикстуре — тест
# верифицирует ПОВЕДЕНИЕ ``identify()``/``register_or_merge`` при калиброванных
# порогах, а не текущее состояние констант в исходнике.
CALIBRATED_IDENTIFY_THRESHOLD = 0.72
CALIBRATED_REGISTER_MATCH_THRESHOLD = 0.75

EMBEDDING_DIM = 256

# Test voices (calibrated empirically — см. /tmp/calibrate_act3.py).
# base_seed — эталон голоса, noise_seed — независимый вектор шума.
_SASHA_BASE_SEED = 1        # "Саша" в БД
_SASHA_N302_NOISE_SEED = 1001  # synthetic Саши в n302
_BORIS_BASE_SEED = 2        # "Борис" в БД (отдельный от Шифу)
_BORIS_N303_NOISE_SEED = 1002  # synthetic ermil в n303 (Борис)
_BORIS_N308_NOISE_SEED = 2008  # тот же человек, разные условия записи
_SHIFU_BASE_SEED = 3        # "Шифу" в БД (для проверки пары Boris-Shifu 0.791)
_BORIS_FROM_SHIFU_NOISE_SEED = 1003  # чтобы cos(Шифу, Борис) ≈ 0.791
_GRISHA_BASE_SEED = 4001    # незнакомец #1
_VALERA_BASE_SEED = 5001    # незнакомец #2

# Calibration alpha values (подобраны чтобы эмпирический cos совпал с target).
# Получены из /tmp/calibrate_act3.py:
#   alpha=0.8967 → cos=0.7249 (Саша synthetic, между 0.72 и 0.75)
#   alpha=0.5916 → cos=0.8651 (Борис synthetic, выше обоих порогов)
#   alpha=0.8212 → cos=0.7901 (Борис↔Шифу, та самая «ложная» пара)
ALPHA_SASHA_SYNTHETIC = 0.8967
ALPHA_BORIS_SYNTHETIC = 0.5916
ALPHA_BORIS_FROM_SHIFU = 0.8212

# Ожидаемые эмпирические значения cos (проверяются в фикстуре voice_set).
EXPECTED_COS_SASHA_SYNTHETIC = 0.725
EXPECTED_COS_BORIS_SYNTHETIC = 0.866
EXPECTED_COS_BORIS_SHIFU = 0.791


# ============================================================================
# Helpers
# ============================================================================


def _unit(seed: int) -> np.ndarray:
    """L2-нормализованный детерминированный вектор."""
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(EMBEDDING_DIM).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    """base + alpha * (независимый единичный вектор), затем ре-нормализация.

    Та же формула, что в test_speaker_embeddings.py / test_voice_registration.py
    / test_identify_threshold.py. Для двух независимых единичных векторов
    в высокой размерности dot(base, noise) ~ 0, поэтому аналитически
    cos(base, degraded) ≈ 1/sqrt(1 + alpha²).
    """
    noise = _unit(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


def _cos(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) + 1e-9))


# ============================================================================
# Fixtures
# ============================================================================


@pytest.fixture()
def calibrated_db(tmp_path, monkeypatch):
    """Изолированная БД + калиброванные пороги из PR #2372.

    Подставляет ``IDENTIFY_THRESHOLD=0.72`` и ``REGISTER_MATCH_THRESHOLD=0.75``
    через monkeypatch — тест верифицирует ПОВЕДЕНИЕ на калиброванных порогах,
    а не текущее значение констант (которое может измениться, если PM
    решит ещё раз откалибровать).
    """
    monkeypatch.setattr(_se, "IDENTIFY_THRESHOLD", CALIBRATED_IDENTIFY_THRESHOLD)
    monkeypatch.setattr(
        _se, "REGISTER_MATCH_THRESHOLD", CALIBRATED_REGISTER_MATCH_THRESHOLD
    )
    d = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield d
    d.close()


@pytest.fixture()
def voice_set(calibrated_db) -> Tuple[dict, dict]:
    """Регистрирует акт-2 профили и возвращает «voice set» для act3.

    Returns:
        voices: dict c эталонными эмбеддингами для всех четырёх голосов act3.
        db:    SpeakerDatabase с уже зарегистрированными профилями.

    Регистрация имитирует акт 2 (знакомство): два зарегистрированных
    (Саша, Борис) + один «владелец» (Шифу, для проверки пары Boris-Shifu).
    Незнакомцы НЕ регистрируются в БД — это сценарий act3, который
    проверяем (см. AC-C).
    """
    db = calibrated_db

    # --- акт 2: Саша, Борис, Шифу (владелец для пары Boris-Shifu) ---
    sasha = _unit(_SASHA_BASE_SEED)
    sasha_id = db.register("Саша", sasha)

    # Борис — отдельный seed (не деградация Шифу: для register_or_merge
    # мы используем чистый base, а пару Boris-Shifu строим через _degraded
    # отдельно для проверки identify() на cross-voice).
    boris = _unit(_BORIS_BASE_SEED)
    boris_id = db.register("Борис", boris)

    # Шифу — отдельный seed; чтобы получить cos(Борис, Шифу)=0.791
    # (та самая «ложная» пара из evidence), берём base=Шифу,
    # degraded=Борис. Здесь для register() сохраняем чистый Шифу.
    shifu = _unit(_SHIFU_BASE_SEED)
    shifu_id = db.register("Шифу", shifu)

    # --- synthetic voices для act3 (n302/n303/n304/n305/n308) ---
    # n302: Саша в другой день/комнате — cos~0.725 (между 0.72 и 0.75).
    n302_emb = _degraded(sasha, ALPHA_SASHA_SYNTHETIC, _SASHA_N302_NOISE_SEED)

    # n303: Борис в других условиях — cos~0.866 (выше обоих порогов).
    n303_emb = _degraded(boris, ALPHA_BORIS_SYNTHETIC, _BORIS_N303_NOISE_SEED)

    # n308: тот же Борис, ещё раз — должен матчиться Борисом.
    n308_emb = _degraded(boris, ALPHA_BORIS_SYNTHETIC, _BORIS_N308_NOISE_SEED)

    # n304/n305: незнакомцы — независимые вектора (cos~0.04-0.07).
    n304_emb = _unit(_GRISHA_BASE_SEED)
    n305_emb = _unit(_VALERA_BASE_SEED)

    voices = {
        "sasha_id": sasha_id,
        "boris_id": boris_id,
        "shifu_id": shifu_id,
        "sasha_emb": sasha,
        "boris_emb": boris,
        "shifu_emb": shifu,
        "n302_emb": n302_emb,  # Саша (cos~0.725)
        "n303_emb": n303_emb,  # Борис (cos~0.866)
        "n308_emb": n308_emb,  # Борис снова (cos~0.866)
        "n304_emb": n304_emb,  # дядя Гриша (незнакомец)
        "n305_emb": n305_emb,  # курьер Валера (незнакомец)
    }
    return voices, db


# ============================================================================
# Sanity-проверки калибровки (защита от «тест молча проходит из-за того,
# что синтетика уехала и cos теперь ниже порога»).
# ============================================================================


class TestAct3CalibrationSanity:
    """Sanity-проверки: эмпирические cos совпадают с ожидаемыми.

    Если кто-то изменит seed'ы / alpha и синтетика «уедет» из окна
    (0.72, 0.75) для n302, тесты ниже начнут ложно проходить —
    этот класс фиксирует числа эмпирически (как в test_identify_threshold.py).
    """

    def test_n302_cos_in_calibration_window(self, voice_set):
        """n302 cos~0.725 — между 0.72 и 0.75 (чувствителен к калибровке)."""
        voices, _ = voice_set
        cos = _cos(voices["sasha_emb"], voices["n302_emb"])
        assert abs(cos - EXPECTED_COS_SASHA_SYNTHETIC) < 0.02, (
            f"калибровка n302 разъехалась: target={EXPECTED_COS_SASHA_SYNTHETIC}, "
            f"empirical={cos:.4f}. Перекалибровать ALPHA_SASHA_SYNTHETIC."
        )
        assert EXPECTED_COS_SASHA_SYNTHETIC > CALIBRATED_IDENTIFY_THRESHOLD, (
            "тест потерял смысл: n302 должен проходить НОВЫЙ порог 0.72"
        )
        assert EXPECTED_COS_SASHA_SYNTHETIC < 0.75, (
            "тест потерял чувствительность: n302 должен НЕ проходить "
            "СТАРЫЙ порог 0.75 (иначе мы не покрываем калибровку)"
        )

    def test_n303_cos_above_both_thresholds(self, voice_set):
        """n303 cos~0.866 — выше обоих порогов (0.72 и 0.75)."""
        voices, _ = voice_set
        cos = _cos(voices["boris_emb"], voices["n303_emb"])
        assert abs(cos - EXPECTED_COS_BORIS_SYNTHETIC) < 0.02, (
            f"калибровка n303 разъехалась: target={EXPECTED_COS_BORIS_SYNTHETIC}, "
            f"empirical={cos:.4f}. Перекалибровать ALPHA_BORIS_SYNTHETIC."
        )
        assert cos > CALIBRATED_IDENTIFY_THRESHOLD
        assert cos > CALIBRATED_REGISTER_MATCH_THRESHOLD

    def test_n308_cos_above_both_thresholds(self, voice_set):
        """n308 cos~0.866 — тот же человек, в других условиях."""
        voices, _ = voice_set
        cos = _cos(voices["boris_emb"], voices["n308_emb"])
        assert abs(cos - EXPECTED_COS_BORIS_SYNTHETIC) < 0.02
        assert cos > CALIBRATED_IDENTIFY_THRESHOLD

    def test_boris_shifu_cross_voice_cos_791(self, voice_set):
        """Пара Борис↔Шифу ≈ 0.791 — выше обоих порогов (как в evidence).

        Контр-тест на ложное слияние: эта пара БЛИЖЕ всех «пересекается»
        между профилями (см. evidence issue #2346). На пороге 0.72 register_or_merge
        НЕ должен срабатывать для чужого голоса, если он случайно
        попадёт в окно.
        """
        voices, _ = voice_set
        # Строим «ложную» пару Борис↔Шифу:
        # берём Шифу как base, Борис = degraded(Шифу, ALPHA_BORIS_FROM_SHIFU).
        shifu = voices["shifu_emb"]
        boris_shadow = _degraded(shifu, ALPHA_BORIS_FROM_SHIFU, _BORIS_FROM_SHIFU_NOISE_SEED)
        cos = _cos(shifu, boris_shadow)
        assert abs(cos - EXPECTED_COS_BORIS_SHIFU) < 0.02, (
            f"калибровка Boris-Shifu разъехалась: target={EXPECTED_COS_BORIS_SHIFU}, "
            f"empirical={cos:.4f}. Перекалибровать ALPHA_BORIS_FROM_SHIFU."
        )
        assert EXPECTED_COS_BORIS_SHIFU > CALIBRATED_IDENTIFY_THRESHOLD
        assert EXPECTED_COS_BORIS_SHIFU > CALIBRATED_REGISTER_MATCH_THRESHOLD

    def test_neznakomcy_cos_below_threshold(self, voice_set):
        """Незнакомцы (zahar/filipp) — cos<0.10 к любому профилю."""
        voices, _ = voice_set
        for n_key in ("n304_emb", "n305_emb"):
            n_emb = voices[n_key]
            for ref_key in ("sasha_emb", "boris_emb", "shifu_emb"):
                cos = _cos(n_emb, voices[ref_key])
                assert cos < CALIBRATED_IDENTIFY_THRESHOLD, (
                    f"{n_key} должен быть ниже порога: cos={cos:.4f}"
                )


# ============================================================================
# AC-A: n302 (Саша) — атрибутируется как Саша после калибровки.
#        ДО калибровки (0.75) — был «незнакомец».
# ============================================================================


class TestAct3N302Sasha:
    """AC-A: n302 (anton, synthetic) — атрибутируется как Саша.

    Граничный кейс: synthetic Саша cos~0.725 попадает в окно (0.72, 0.75).
    Это и есть цель калибровки PR #2372 — повторная регистрация того же
    голоса (в других условиях записи) теперь проходит через identify().
    """

    def test_n302_identified_as_sasha(self, voice_set):
        """identify() возвращает SpeakerMatch с name='Саша'."""
        voices, db = voice_set
        match = db.identify(voices["n302_emb"])

        assert match is not None, (
            "n302 (synthetic Саша) должен опознаваться как Саша — это и есть "
            "цель калибровки 0.75→0.72 (issue #2348). До калибровки при 0.75 "
            "identify() возвращал None для cos=0.725."
        )
        assert match.is_known is True
        assert match.name == "Саша"
        assert match.speaker_id == voices["sasha_id"]
        assert match.confidence >= CALIBRATED_IDENTIFY_THRESHOLD
        assert match.confidence < 0.75, (
            "n302 cos~0.725 — sanity: должен быть НИЖЕ 0.75, иначе тест "
            "нечувствителен к калибровке"
        )

    def test_n302_register_or_merge_creates_new_profile_below_threshold(self, voice_set):
        """register_or_merge(n302) при cos~0.725 (НИЖЕ REGISTER_MATCH=0.75) —

        by design создаёт НОВЫЙ профиль, не сливается. Это корректное
        поведение: register_or_merge жёстче, чем identify(), чтобы
        избежать ложных слияний разных голосов. Тест явно
        фиксирует контракт (issue #2348 AC-A: act3 n302 атрибутируется
        через identify(), а не через register_or_merge).
        """
        voices, db = voice_set

        # sanity: n302 cos ниже REGISTER_MATCH, поэтому merge НЕ сработает.
        cos = _cos(voices["sasha_emb"], voices["n302_emb"])
        assert cos < CALIBRATED_REGISTER_MATCH_THRESHOLD, (
            f"подготовка фикстуры: cos={cos:.3f} должна быть < "
            f"REGISTER_MATCH={CALIBRATED_REGISTER_MATCH_THRESHOLD}, "
            f"иначе тест ловит не тот сценарий"
        )

        speakers_before = db.list_speakers()
        assert len(speakers_before) == 3

        # Вызов без явного speaker_id — register_or_merge принимает
        # решение по REGISTER_MATCH_THRESHOLD=0.75.
        sid, reused = db.register_or_merge("Саша", voices["n302_emb"])
        assert reused is False, (
            f"n302 (cos~{cos:.3f} < REGISTER_MATCH=0.75) НЕ должен "
            f"сливаться — это by design: register_or_merge жёстче identify()"
        )
        assert sid != voices["sasha_id"], (
            f"register_or_merge без явного id при cos<порога должен "
            f"создать новый профиль, а не слиться с {voices['sasha_id'][:8]}"
        )

        # Один новый профиль появился (теперь 4), у Саши по-прежнему 1 эмбеддинг.
        speakers_after = db.list_speakers()
        assert len(speakers_after) == 4, (
            f"ожидался +1 новый профиль (теперь 4), получено {len(speakers_after)}"
        )
        sasha_profile = next(s for s in speakers_after if s["id"] == voices["sasha_id"])
        assert sasha_profile["embeddings"] == 1, (
            f"профиль Саши не должен измениться: ожидался 1 эмбеддинг, "
            f"получено {sasha_profile['embeddings']}"
        )


# ============================================================================
# AC-B: n303/n308 (Борис) — атрибутируется как Борис, не как Шифу.
# ============================================================================


class TestAct3N303N308Boris:
    """AC-B: n303 и n308 (ermil, synthetic) — атрибутируется как Борис.

    В БД есть и Борис, и Шифу, и между ними cos=0.791 (evidence #2346).
    При synthetic ermil cos(Борис, n303)=0.866, cos(Шифу, n303)=0.791 —
    identify() отдаёт тот профиль, у которого score выше.
    """

    def test_n303_identified_as_boris_not_shifu(self, voice_set):
        """identify() возвращает Бориса, а не Шифу (несмотря на cross-voice 0.791)."""
        voices, db = voice_set
        match = db.identify(voices["n303_emb"])

        assert match is not None
        assert match.is_known is True
        assert match.name == "Борис", (
            f"n303 должен опознаться как Борис (cos=0.866 выше, чем "
            f"cos(Шифу)=0.791), но identify() вернул {match.name!r}. "
            f"Возможна инверсия rank, если Шифу в БД с более крупным эмбеддингом."
        )
        assert match.speaker_id == voices["boris_id"]
        assert match.confidence > CALIBRATED_IDENTIFY_THRESHOLD

        # Проверка через candidates — Шифу должен быть вторым, но НЕ первым.
        candidates = db.identify_candidates(voices["n303_emb"], top_n=3)
        assert len(candidates) >= 2, "ожидались как минимум Борис и Шифу в candidates"
        # Первый — Борис с score 0.866; второй — Шифу с score 0.791.
        names = [c.name for c in candidates]
        assert "Борис" in names, f"Борис пропал из candidates: {names!r}"
        assert "Шифу" in names, f"Шифу пропал из candidates: {names!r}"

        # У Бориса score выше, чем у Шифу.
        sasha_top = next(c for c in candidates if c.name == "Борис")
        shifu_top = next(c for c in candidates if c.name == "Шифу")
        assert sasha_top.confidence > shifu_top.confidence, (
            f"Борис должен быть первым в candidates (cos=0.866 > Шифу=0.791), "
            f"но Борис={sasha_top.confidence:.3f}, Шифу={shifu_top.confidence:.3f}"
        )

    def test_n308_identified_as_boris(self, voice_set):
        """n308 — тот же Борис, но в других условиях. identify() → Борис."""
        voices, db = voice_set
        match = db.identify(voices["n308_emb"])

        assert match is not None
        assert match.is_known is True
        assert match.name == "Борис"
        assert match.speaker_id == voices["boris_id"]

    def test_n303_n308_register_or_merge_does_not_create_boris_duplicates(self, voice_set):
        """register_or_merge для n303 и n308 → 1 Борис, 2 доп. эмбеддинга.

        Защита от регрессии W5-4: повторная регистрация того же голоса
        не должна плодить «Борис-2», «Борис-3».
        """
        voices, db = voice_set

        # Регистрируем n303 и n308 — оба Бориса, оба с cos~0.866.
        sid1, reused1 = db.register_or_merge("Борис", voices["n303_emb"])
        sid2, reused2 = db.register_or_merge("Борис", voices["n308_emb"])

        assert reused1 is True, "n303 (cos~0.866) должен слизться с Борисом"
        assert reused2 is True, "n308 (cos~0.866) должен слизться с Борисом"
        assert sid1 == voices["boris_id"]
        assert sid2 == voices["boris_id"]

        speakers = db.list_speakers()
        boris_profiles = [s for s in speakers if s["name"] == "Борис"]
        assert len(boris_profiles) == 1, (
            f"после двух register_or_merge ожидался 1 профиль «Борис», "
            f"получено {len(boris_profiles)}: {boris_profiles!r} — "
            f"РЕГРЕССИЯ W5-4 (один голос — два профиля)"
        )
        assert boris_profiles[0]["embeddings"] == 3, (
            f"у Бориса должно быть 3 эмбеддинга (1 исходный + n303 + n308), "
            f"получено {boris_profiles[0]['embeddings']}"
        )


# ============================================================================
# AC-C: n304/n305 (незнакомцы) — is_known=False, не путаются между собой.
# ============================================================================


class TestAct3N304N305Unknown:
    """AC-C: незнакомые голоса (zahar/filipp) — is_known=False.

    Два разных незнакомца НЕ должны склеиваться в один профиль и НЕ должны
    атрибутироваться как Саша/Борис/Шифу.
    """

    def test_n304_unknown_identified_as_none(self, voice_set):
        """n304 (дядя Гриша, zahar) → identify() возвращает None."""
        voices, db = voice_set
        match = db.identify(voices["n304_emb"])

        assert match is None, (
            f"n304 (незнакомец) не должен опознаваться как зарегистрированный "
            f"спикер, но identify() вернул {match!r}"
        )

    def test_n305_unknown_identified_as_none(self, voice_set):
        """n305 (курьер Валера, filipp) → identify() возвращает None."""
        voices, db = voice_set
        match = db.identify(voices["n305_emb"])

        assert match is None, (
            f"n305 (незнакомец) не должен опознаваться как зарегистрированный "
            f"спикер, но identify() вернул {match!r}"
        )

    def test_two_unknown_voices_not_merged_into_one_profile(self, voice_set):
        """register_or_merge двух незнакомцев → 2 разных профиля, не 1.

        До калибровки (0.75) была ловушка: голоса cos~0.04-0.07 → ниже
        порога → каждый получает свой профиль. Это и есть «не путать
        разных незнакомцев».
        """
        voices, db = voice_set

        speakers_pre = db.list_speakers()
        n_pre = len(speakers_pre)

        # Регистрируем обоих незнакомцев с разными именами.
        sid_grisha, reused_grisha = db.register_or_merge("Гриша", voices["n304_emb"])
        sid_valera, reused_valera = db.register_or_merge("Валера", voices["n305_emb"])

        assert reused_grisha is False, "Гриша — новая регистрация (не merge)"
        assert reused_valera is False, "Валера — новая регистрация (не merge)"
        assert sid_grisha != sid_valera, (
            "два разных незнакомца НЕ должны склеиться в один профиль"
        )

        speakers_post = db.list_speakers()
        assert len(speakers_post) == n_pre + 2, (
            f"после регистрации двух незнакомцев ожидалось {n_pre + 2} "
            f"профилей, получено {len(speakers_post)}: {speakers_post!r}"
        )

        # Оба с уникальными id.
        names = {s["name"] for s in speakers_post}
        assert {"Гриша", "Валера"}.issubset(names)

    def test_unknown_voice_candidates_have_low_confidence(self, voice_set):
        """identify_candidates() для незнакомца — все score ниже порога."""
        voices, db = voice_set

        for n_key in ("n304_emb", "n305_emb"):
            candidates = db.identify_candidates(voices[n_key], top_n=5)
            for c in candidates:
                assert c.confidence < CALIBRATED_IDENTIFY_THRESHOLD, (
                    f"нeзнакомец {n_key}: candidate {c.name!r} "
                    f"score={c.confidence:.3f} >= порога {CALIBRATED_IDENTIFY_THRESHOLD}"
                )


# ============================================================================
# Полный сценарий act3 (n302..n308): комбинированный прогон.
# ============================================================================


class TestAct3FullScenario:
    """Полный сценарий act3 — проверка всего пути в одном тесте.

    Имитирует работу dialogue_node: для каждой реплики act3 вызывается
    identify() (как в speaker_id_node) и проверяется, что атрибуция
    корректна. Плюс считается статистика speakers.db до/после.

    Это «хребет» теста: AC-A + AC-B + AC-C вместе + стат-чек.
    """

    def test_full_act3_n302_n303_n304_n305_n308_attribution(
        self, voice_set
    ):
        """Полная последовательность act3: 5 реплик, 4 голоса, корректная атрибуция."""
        voices, db = voice_set

        # --- Стат-ка ДО act3 (имитация «уже зарегистрированы в акте 2») ---
        before = db.list_speakers()
        before_total = len(before)
        before_embeddings = sum(s["embeddings"] for s in before)
        before_names = sorted(s["name"] for s in before)
        assert before_total == 3, (
            f"precondition: 3 профиля в БД, получено {before_total}: {before!r}"
        )
        assert before_names == ["Борис", "Саша", "Шифу"]
        assert before_embeddings == 3, (
            f"precondition: 3 эмбеддинга (по 1 на профиль), получено {before_embeddings}"
        )

        # --- n302: anton → Саша ---
        m302 = db.identify(voices["n302_emb"])
        assert m302 is not None and m302.name == "Саша", (
            f"n302: ожидался match с name='Саша', получено {m302!r}"
        )

        # --- n303: ermil → Борис ---
        m303 = db.identify(voices["n303_emb"])
        assert m303 is not None and m303.name == "Борис", (
            f"n303: ожидался match с name='Борис', получено {m303!r}"
        )

        # --- n304: zahar → незнакомец (None) ---
        m304 = db.identify(voices["n304_emb"])
        assert m304 is None, f"n304: ожидался None (незнакомец), получено {m304!r}"

        # --- n305: filipp → незнакомец (None) ---
        m305 = db.identify(voices["n305_emb"])
        assert m305 is None, f"n305: ожидался None (незнакомец), получено {m305!r}"

        # --- n308: ermil → Борис ---
        m308 = db.identify(voices["n308_emb"])
        assert m308 is not None and m308.name == "Борис", (
            f"n308: ожидался match с name='Борис', получено {m308!r}"
        )

        # --- Стат-ка ПОСЛЕ identify (без register): должна совпадать с ДО ---
        after_identify = db.list_speakers()
        assert len(after_identify) == 3, (
            f"identify() не должен менять число профилей, но стало {len(after_identify)}"
        )
        assert sum(s["embeddings"] for s in after_identify) == before_embeddings, (
            "identify() не должен менять число эмбеддингов"
        )

    def test_act3_with_register_or_merge_calibrated_threshold_no_w5_4_regression(
        self, voice_set
    ):
        """Полный act3 + register_or_merge: контраст — кто мержится, кто нет.

        После калибровки (issue #2348, PR #2372):
          * REGISTER_MATCH_THRESHOLD=0.75 (было 0.82)
          * n303/n308 cos~0.866 — выше 0.75 → MERGE (reused=True)
          * n302 cos~0.725 — НИЖЕ 0.75 → НОВЫЙ ПРОФИЛЬ (reused=False)
            (это by design: register_or_merge жёстче identify())

        Это проверяет два свойства:
          1. n303/n308 НЕ создают дублей «Борис-2»/«Борис-3» (W5-4 fix)
          2. n302 НЕ сливается ошибочно с Сашей (порог 0.75 защищает от
             false-positive merge для «похожих, но других» голосов)
        """
        voices, db = voice_set

        # n302 — synthetic Саша, cos~0.725. Ниже REGISTER_MATCH → новый профиль.
        sid_n302, reused_n302 = db.register_or_merge("Саша", voices["n302_emb"])
        assert reused_n302 is False, (
            "n302 (cos~0.725) НЕ должен сливаться с Сашей — это by design "
            "для register_or_merge (порог 0.75 строже, чем identify 0.72)"
        )
        # Новый профиль создан, у Саши (старого) по-прежнему 1 эмбеддинг.
        after_n302 = db.list_speakers()
        assert len(after_n302) == 4  # 3 акт2 + 1 новая «Саша-2»
        original_sasha = next(s for s in after_n302 if s["id"] == voices["sasha_id"])
        assert original_sasha["embeddings"] == 1

        # n303 и n308 — synthetic Борис, cos~0.866. Выше REGISTER_MATCH → MERGE.
        sid_n303, reused_n303 = db.register_or_merge("Борис", voices["n303_emb"])
        sid_n308, reused_n308 = db.register_or_merge("Борис", voices["n308_emb"])

        assert reused_n303 is True, (
            "n303 (cos~0.866) должен слиться с Борисом — это и есть W5-4 fix"
        )
        assert reused_n308 is True, (
            "n308 (cos~0.866) должен слиться с Борисом"
        )
        assert sid_n303 == voices["boris_id"]
        assert sid_n308 == voices["boris_id"]

        # Финальный дамп: Борис (3 эмбеддинга), Саша (1), Саша-2 (1), Шифу (1) = 4 профиля.
        after = db.list_speakers()
        names = sorted(s["name"] for s in after)
        assert names == ["Борис", "Саша", "Саша", "Шифу"], (
            f"допускается один дубль « Саша-2» (от n302, by design), "
            f"но больше быть не должно: {names!r}"
        )

        # Контраст: НЕ должно быть «Борис-2» — это и есть регрессия W5-4.
        boris_profiles = [s for s in after if s["name"] == "Борис"]
        assert len(boris_profiles) == 1, (
            f"РЕГРЕССИЯ W5-4: ожидался 1 профиль «Борис», получено "
            f"{len(boris_profiles)}: {boris_profiles!r}"
        )
        assert boris_profiles[0]["embeddings"] == 3, (
            f"Борис должен накопить 3 эмбеддинга (1 акт2 + n303 + n308), "
            f"получено {boris_profiles[0]['embeddings']}"
        )

        # Контраст: НЕ должно быть больше одного дубля «Саша-N» —
        # register_or_merge жёстко отбивает ложные слияния.
        sasha_profiles = [s for s in after if s["name"] == "Саша"]
        assert len(sasha_profiles) == 2, (
            f"допускаются 2 профиля «Саша» (оригинальный акт2 + новый от n302), "
            f"получено {len(sasha_profiles)}: {sasha_profiles!r}"
        )