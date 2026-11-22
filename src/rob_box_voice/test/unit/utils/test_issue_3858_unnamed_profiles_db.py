"""Issue #3858 — профиль без имени в ``SpeakerDatabase`` (решение Шифу 22.10, «режим мастерской» для голоса).

Что фиксируют тесты (БД-уровень; нода и политика — в ``node/test_issue_3858_unknown_voice_nickname_profile.py``):

1. ``db.unnamed.register`` заводит профиль с ``name=''`` и одним эмбеддингом; короткая речь — ``AudioTooShortError``
   до записи в БД.
2. Лимит: сверх ``max_unnamed`` вытесняются самые старые безымянные (каскадом, сирот нет); именованных не трогаем;
   новый профиль не вытесняется.
3. ``register_or_merge`` с именем отдаёт имя ТОМУ ЖЕ безымянному профилю (``claimed_unnamed``), только если
   косинус дотянул до порога слияния 0.75 (#3858). Ниже (полоса обычного опознания 0.72) — конфликт
   имён (ADR-0127, #5019): имя безымянному не пишется, заводится отдельный профиль. Пороги не менялись.
4. Безымянный профиль не имя: тёзка/слияние его не видят, ``SpeakerMatch.named`` — False.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

# Грузим модуль по пути, минуя пакет `rob_box_voice`, чей `__init__` тянет pyaudio — на dev-машине и в CI его нет
# (тот же приём, что в соседних тестах этого каталога).
_SPEAKER_EMB = Path(__file__).resolve().parents[3] / "rob_box_voice" / "utils" / "speaker_embeddings.py"
_spec = importlib.util.spec_from_file_location("rob_box_voice.utils.speaker_embeddings", _SPEAKER_EMB)
se = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_voice.utils.speaker_embeddings"] = se
_spec.loader.exec_module(se)

AudioTooShortError = se.AudioTooShortError
SpeakerDatabase = se.SpeakerDatabase


def _embedding(seed: int) -> np.ndarray:
    v = np.random.default_rng(seed).standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    v = base + alpha * _embedding(noise_seed)
    return (v / np.linalg.norm(v)).astype(np.float32)


def _cos(a, b) -> float:
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))


@pytest.fixture()
def db(tmp_path):
    database = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield database
    database.close()


def test_thresholds_are_untouched():
    """Карточка: порог опознания 0.72 и порог слияния 0.75 НЕ трогать."""
    assert se.IDENTIFY_THRESHOLD == 0.72
    assert se.REGISTER_MATCH_THRESHOLD == 0.75


def test_register_unnamed_creates_nameless_profile(db):
    sid = db.unnamed.register(_embedding(1), duration_sec=4.0).speaker_id

    assert db.unnamed.count() == 1
    profile = db.get_speaker_profile(sid)
    assert profile["name"] == se.UNNAMED == ""
    assert db.gallery_size(sid) == 1


def test_nameless_profile_is_identified_as_unnamed_match(db):
    base = _embedding(2)
    sid = db.unnamed.register(base, duration_sec=4.0).speaker_id

    match = db.identify(_degraded(base, 0.4, noise_seed=77))

    assert match is not None and match.speaker_id == sid
    assert match.named is False and match.name == ""


def test_short_speech_creates_no_profile(db):
    with pytest.raises(AudioTooShortError):
        db.unnamed.register(_embedding(3), duration_sec=se.MIN_REGISTER_AUDIO_DURATION_SEC - 0.1)

    assert db.unnamed.count() == 0
    assert db.list_speakers() == []


def test_limit_evicts_oldest_unnamed_and_keeps_named(db):
    named = db.register("Борис", _embedding(10), duration_sec=5.0)
    ids = [db.unnamed.register(_embedding(100 + i), 4.0, max_unnamed=3).speaker_id for i in range(5)]

    remaining = {row["id"] for row in db.list_speakers()}
    assert named in remaining, "именованные не вытесняются"
    assert remaining - {named} == set(ids[2:]), "остались три самых новых безымянных"
    assert db.unnamed.count() == 3
    assert db.orphaned_embedding_count() == 0, "эмбеддинги вытесненных ушли каскадом"


def test_eviction_reports_victims_and_never_evicts_the_new_profile(db):
    first = db.unnamed.register(_embedding(200), 4.0, max_unnamed=1).speaker_id
    outcome = db.unnamed.register(_embedding(201), 4.0, max_unnamed=1)

    assert outcome.evicted == (first,)
    assert db.get_speaker_profile(first) is None
    assert db.get_speaker_profile(outcome.speaker_id) is not None


def test_introduction_gives_the_name_to_the_same_profile(db):
    base = _embedding(4)
    sid = db.unnamed.register(base, duration_sec=4.0).speaker_id
    db.set_epithet(sid, "Гость", "first_seen")
    phrase = _degraded(base, 0.3, noise_seed=55)
    assert _cos(base, phrase) >= se.REGISTER_MATCH_THRESHOLD

    outcome = db.register_or_merge("гриша", phrase, duration_sec=4.5)

    assert outcome.speaker_id == sid and outcome.claimed_unnamed and outcome.reused
    assert [row["name"] for row in db.list_speakers()] == ["Гриша"], "дубля нет, имя нормализовано"
    assert db.get_epithet(sid) == "Гость", "кличка осталась"
    assert db.gallery_size(sid) == 2, "эмбеддинг представления дописан в галерею"
    assert db.unnamed.count() == 0


def _phrase_at_cosine(base: np.ndarray, target: float) -> np.ndarray:
    """Эмбеддинг с косинусом к ``base`` ровно ``target`` (плоскость base × ортогональный шум)."""
    noise = _embedding(901)
    ortho = noise - float(np.dot(noise, base)) * base
    ortho = ortho / np.linalg.norm(ortho)
    phrase = target * base + (1.0 - target**2) ** 0.5 * ortho
    return (phrase / np.linalg.norm(phrase)).astype(np.float32)


def test_foreign_voice_below_merge_threshold_does_not_claim_unnamed(db):
    """#5019: безымянный + cos 0.73 + чужое имя → конфликт, имя не записано, отдельный профиль."""
    base = _embedding(5)
    sid = db.unnamed.register(base, duration_sec=4.0).speaker_id
    phrase = _phrase_at_cosine(base, 0.73)
    assert se.IDENTIFY_THRESHOLD < _cos(base, phrase) < se.REGISTER_MATCH_THRESHOLD

    outcome = db.register_or_merge("Нина", phrase, duration_sec=4.5)

    assert outcome.name_conflict and not outcome.claimed_unnamed and not outcome.reused
    assert outcome.conflict_speaker_id == sid
    assert outcome.speaker_id != sid, "чужому голосу заведён свой профиль"
    names = {row["id"]: row["name"] for row in db.list_speakers()}
    assert names[sid] == "", "безымянный остался без чужого имени"
    assert names[outcome.speaker_id] == "Нина"


def test_introduction_at_merge_threshold_claims_the_same_profile(db):
    """Регрессия #3858: cos >= порога регистрации — имя тому же безымянному id, дубля нет."""
    base = _embedding(51)
    sid = db.unnamed.register(base, duration_sec=4.0).speaker_id
    phrase = _phrase_at_cosine(base, se.REGISTER_MATCH_THRESHOLD)
    assert _cos(base, phrase) >= se.REGISTER_MATCH_THRESHOLD

    outcome = db.register_or_merge("Гриша", phrase, duration_sec=4.5)

    assert outcome.speaker_id == sid and outcome.claimed_unnamed and outcome.reused
    assert not outcome.name_conflict
    assert [row["name"] for row in db.list_speakers()] == ["Гриша"]


def test_introduction_of_a_different_voice_does_not_touch_nameless_profile(db):
    sid = db.unnamed.register(_embedding(6), duration_sec=4.0).speaker_id

    outcome = db.register_or_merge("Борис", _embedding(7), duration_sec=4.5)

    assert outcome.speaker_id != sid and not outcome.claimed_unnamed
    names = {row["id"]: row["name"] for row in db.list_speakers()}
    assert names[sid] == "" and names[outcome.speaker_id] == "Борис"


def test_claim_reports_namesake_for_the_follow_up_question(db):
    """Имя уже есть у другого профиля (тёзка) — присвоение идёт, наружу уезжает повод переспросить."""
    db.register("Гриша", _embedding(8), duration_sec=5.0)
    base = _embedding(9)
    sid = db.unnamed.register(base, duration_sec=4.0).speaker_id

    outcome = db.register_or_merge("Гриша", _degraded(base, 0.3, noise_seed=58), duration_sec=4.5)

    assert outcome.speaker_id == sid and outcome.claimed_unnamed
    assert outcome.name_twin and outcome.twin_name == "Гриша"


def test_namesake_lookup_ignores_nameless_profiles(db):
    db.unnamed.register(_embedding(11), duration_sec=4.0)

    assert db._find_by_name("") is None
    assert db.rename_by_name("", "Кто-то") is None
