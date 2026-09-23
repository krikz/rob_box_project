#!/usr/bin/env python3
"""
test_issue_2829_register_bound_to_utterance.py

Issue #2829 (ADR-0131), PR-2 -- ``speaker_id_node._on_register_request``
must bind registration to the ``utterance_id`` of the phrase in which the
person introduced themselves, instead of "register whoever speaks next"
(the old ``_pending_register_name`` fallback with no deadline -- live
case 23.09 10:53: "Will register next utterance as 'Дэнчик'" registered
43s later, quite possibly a different person).

Contract after the fix:

1. ``utterance_id`` present + matching embedding already in the ring
   buffer -- register immediately (happy path, the common case: by the
   time the LLM calls register_speaker, dialogue_node's
   UtteranceSpeakerRegistry.resolve() has already waited for the FULL
   /voice/speaker/result of this same utterance, so the embedding is
   already there).
2. ``utterance_id`` present but the embedding hasn't arrived YET -- a
   short BOUNDED wait (``_REGISTER_UTTERANCE_WAIT_SEC``), not an
   unbounded pend. If it shows up within the window, register; otherwise
   honest ``register_error`` (``utterance_not_found``).
3. No ``utterance_id`` at all -- immediate honest ``register_error``
   (``no_utterance_context``), never "register the next utterance from
   anyone".
4. Growth-session (``_apply_growth_session``): a reply that is NOT
   recognised as belonging to the session owner (unknown OR a different
   person, using the unthresholded top-1 candidate from
   ``identify_candidates``) closes the session instead of being
   silently appended -- the "новый незнакомый голос... отравляет чужую
   галерею" bug from the issue.

Same node-construction trick as test_register_publishes_result.py
(``object.__new__(SpeakerIdNode)`` -- no ROS2, no resemblyzer needed).
"""

from __future__ import annotations

import collections
import json
import sys
import threading
import time
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Deque, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402


class _Embed:
    def __init__(self, embedding, voiced_sec: float = 5.0) -> None:
        self.embedding = embedding
        self.voiced_sec = voiced_sec
        self.raw_sec = max(voiced_sec, 0.0)

    @property
    def voiced_ratio(self) -> float:
        return (self.voiced_sec / self.raw_sec) if self.raw_sec > 0 else 0.0


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    noise = _embedding(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(json.loads(msg.data))


def _msg(payload):
    if isinstance(payload, str):
        return type("Msg", (), {"data": payload})()
    return type("Msg", (), {"data": json.dumps(payload, ensure_ascii=False)})()


@pytest.fixture()
def node(tmp_path, monkeypatch):
    import rob_box_voice.utils.speaker_embeddings as se_mod

    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SIZE", 5)
    monkeypatch.setattr(se_mod, "MIN_REGISTER_AUDIO_DURATION_SEC", 3.0)

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._speech_log = {}
    instance._speech_log_lock = threading.Lock()
    instance._sample_rate = 16000
    instance._recent_embeddings: Deque[
        Tuple[float, np.ndarray, float, Optional[str]]
    ] = collections.deque(maxlen=20)
    instance._MAX_EMBED_AGE_SEC = 30.0
    instance._REGISTER_UTTERANCE_WAIT_SEC = 0.2
    instance._REGISTER_UTTERANCE_POLL_SEC = 0.02
    instance._pending_register_name: Optional[str] = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
    instance._epithet_request_pub = None
    instance._growth_session_gap_sec = 30.0
    instance._growth_session = None
    instance._executor = ThreadPoolExecutor(max_workers=2)
    yield instance
    instance._executor.shutdown(wait=True)
    instance._db.close()


def _wait_until(predicate, timeout=1.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ---------------------------------------------------------------------------
# 1. utterance_id present, embedding already there -- happy path
# ---------------------------------------------------------------------------


def test_register_with_matching_embedding_registers_immediately(node):
    emb = _embedding(1)
    node._recent_embeddings.append((time.time(), emb, 4.0, "utt-abc123"))

    node._on_register_request(
        _msg({"name": "Саша", "utterance_id": "utt-abc123"})
    )
    _wait_until(lambda: node._db.list_speakers() != [])

    speakers = node._db.list_speakers()
    assert len(speakers) == 1
    assert speakers[0]["name"] == "Саша"
    registered = [m for m in node._result_pub.messages if m.get("event") == "registered"]
    assert len(registered) == 1
    assert registered[0]["utterance_id"] == "utt-abc123"


def test_register_does_not_pick_a_different_utterance_embedding(node):
    """Two utterances in flight -- register_speaker for utt-2 must use
    utt-2's own embedding, never utt-1's (the old "freshest within 30s"
    logic would have picked whichever was most recent, regardless of
    which phrase actually contained "meня зовут")."""
    emb1 = _embedding(10)
    emb2 = _embedding(20)
    node._recent_embeddings.append((time.time() - 5, emb1, 4.0, "utt-1"))
    node._recent_embeddings.append((time.time(), emb2, 4.0, "utt-2"))

    node._on_register_request(_msg({"name": "Борис", "utterance_id": "utt-2"}))
    _wait_until(lambda: node._db.list_speakers() != [])

    stored = node._db.identify(emb2, threshold=0.99)
    assert stored is not None and stored.name == "Борис"
    # Убедиться, что записан ИМЕННО emb2, а не emb1 -- self-similarity
    # emb1 к сохранённому профилю должна быть заметно ниже.
    other = node._db.identify(emb1, threshold=0.0)
    assert other is not None
    same = node._db.identify(emb2, threshold=0.0)
    assert same.confidence > other.confidence


# ---------------------------------------------------------------------------
# 2. utterance_id present, embedding arrives late (bounded wait)
# ---------------------------------------------------------------------------


def test_register_waits_briefly_for_late_embedding(node):
    node._on_register_request(_msg({"name": "Аня", "utterance_id": "utt-late"}))
    # Эмбеддинг «доезжает» чуть позже, пока идёт bounded retry.
    time.sleep(0.05)
    emb = _embedding(30)
    node._recent_embeddings.append((time.time(), emb, 4.0, "utt-late"))

    _wait_until(lambda: node._db.list_speakers() != [], timeout=1.0)

    speakers = node._db.list_speakers()
    assert len(speakers) == 1 and speakers[0]["name"] == "Аня"


def test_register_gives_up_honestly_after_wait_window(node):
    """Эмбеддинг для этой фразы так и не появился -- честный
    register_error, а НЕ бесконечное ожидание/регистрация чужой фразы."""
    node._on_register_request(
        _msg({"name": "Игорь", "utterance_id": "utt-never-arrives"})
    )

    def _got_error():
        return any(
            m.get("event") == "register_error" for m in node._result_pub.messages
        )

    assert _wait_until(_got_error, timeout=1.0)
    assert node._db.list_speakers() == []
    error = [m for m in node._result_pub.messages if m.get("event") == "register_error"][0]
    assert error["error"] == "utterance_not_found"
    assert error["utterance_id"] == "utt-never-arrives"
    # Ключевая проверка контракта: НЕТ _pending_register_name -- следующая
    # реплика от кого угодно НЕ должна внезапно получить это имя.
    assert node._pending_register_name is None


# ---------------------------------------------------------------------------
# 3. no utterance_id at all -- immediate honest refusal
# ---------------------------------------------------------------------------


def test_register_without_utterance_id_is_honest_refusal_not_pending(node):
    node._on_register_request(_msg({"name": "Denchik"}))

    assert node._db.list_speakers() == []
    error = [m for m in node._result_pub.messages if m.get("event") == "register_error"]
    assert len(error) == 1
    assert error[0]["error"] == "no_utterance_context"
    # САМОЕ ГЛАВНОЕ: старое поведение "запомню следующую фразу кого
    # угодно" отсутствует -- _pending_register_name не выставлен.
    assert node._pending_register_name is None


def test_register_plain_text_name_without_json_is_also_refused_honestly(node):
    """Legacy: plain-text (не-JSON) payload тоже трактуется как имя без
    utterance_id -- тот же честный отказ, не automagic pending."""
    node._on_register_request(_msg("Вася"))

    assert node._db.list_speakers() == []
    assert node._pending_register_name is None


# ---------------------------------------------------------------------------
# 4. growth session: unknown/different speaker closes the session
# ---------------------------------------------------------------------------


def test_growth_session_closes_on_completely_unknown_voice(node):
    """КЛЮЧЕВОЙ тест issue #2829: раньше match=None (голос вообще ни на
    кого не похож) молча ПРОХОДИЛ и дописывался в галерею владельца
    сессии -- "новый незнакомый голос... отравляет чужую галерею". Тест
    строит эмбеддинг заведомо ортогональный владельцу (другой seed,
    полный шум, без примеси base) -- top-1 кандидат в БД из одного
    человека формально останется owner (единственная запись), поэтому
    добавляем ВТОРОГО, отдельно зарегистрированного человека, к кому
    этот случайный голос будет объективно ближе, чтобы top-1 гарантированно
    ушёл не к owner."""
    owner_base = _embedding(100)
    node._do_register("Хозяин", owner_base, speaker_id=None)
    sid_owner = node._growth_session["speaker_id"]

    # Второй, уже известный профиль -- случайный голос-нарушитель будет
    # ближе к нему, чем к владельцу сессии (оба вектора из одного и того
    # же нормального распределения, но разных seed -- в реальности это
    # соответствует "голос отдалённо похож на кого-то ДРУГОГО в базе, не
    # на владельца").
    # Intruder построен ПЕРВЫМ, а "Знакомый2" -- заведомо похожий на
    # intruder (_degraded с маленькой alpha, тот же приём калибровки,
    # что в growth-тестах выше), чтобы top-1 гарантированно ушёл к
    # Знакомому2, а не к owner -- два независимых случайных вектора в
    # 256-мерном пространстве почти ортогональны друг другу (cos~0) вне
    # зависимости от seed, так что "просто разные seed" не гарантирует,
    # какой из них окажется ближе к intruder.
    intruder_embedding = _embedding(999)  # совершенно другой голос
    stranger_lookalike = _degraded(intruder_embedding, alpha=0.3, noise_seed=102)
    node._db.register("Знакомый2", stranger_lookalike)

    # Sanity-check ДО обработки: top-1 кандидат обязан быть НЕ owner --
    # иначе тестовые embeddings подобраны неверно и проверка ниже ничего
    # не доказывает.
    top_before = node._db.identify_candidates(intruder_embedding, top_n=1)
    assert top_before and top_before[0].speaker_id != sid_owner, (
        "тестовые embeddings подобраны неверно -- top-1 должен быть НЕ owner"
    )

    node._result_pub.messages.clear()
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(intruder_embedding))

    node._process_utterance(b"\x00\x00" * 50000, utterance_id="utt-intruder")

    assert node._db.gallery_size(sid_owner) == 1, "чужая реплика не должна попасть в галерею"
    assert node._growth_session is None, "сессия обязана закрыться на неопознанном чужом голосе"


def test_growth_session_still_grows_for_owner_when_identify_fails(node):
    """Регресс-щит issue #2747/#2757: рост должен по-прежнему работать,
    когда identify() честно возвращает None для самого владельца (мало
    эталонов в галерее) -- НЕ требуем полного identify()-совпадения,
    только "топ-1 кандидат в БД -- владелец" (тривиально верно, когда
    других известных профилей ещё нет)."""
    base = _embedding(200)
    node._do_register("Денчик", base, speaker_id=None)
    sid = node._growth_session["speaker_id"]
    node._result_pub.messages.clear()

    # cos ~ 0.523 -- ниже calibrated IDENTIFY_THRESHOLD (0.72), тот же
    # приём калибровки, что в test_register_publishes_result.py.
    alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
    degraded = _degraded(base, alpha=alpha, noise_seed=201)
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(degraded))

    node._process_utterance(b"\x00\x00" * 50000, utterance_id="utt-owner-weak")

    assert node._growth_session is not None, "сессия не должна закрыться на слабом, но своём голосе"
    assert node._db.gallery_size(sid) == 2, "рост обязан сработать даже когда identify() вернул None"


def test_growth_session_still_vetoes_confident_different_speaker(node):
    """Регресс существующего вето (#2747): уверенное опознание ДРУГОГО
    известного спикера по-прежнему закрывает сессию (частный случай
    новой топ-1 проверки)."""
    anchor_base = _embedding(300)
    node._do_register("Деньчик", anchor_base, speaker_id=None)
    sid_anchor = node._growth_session["speaker_id"]

    other_base = _embedding(301)
    sid_other = node._db.register("Пётр", other_base)
    node._result_pub.messages.clear()

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(other_base))
    node._process_utterance(b"\x00\x00" * 1000, utterance_id="utt-other")

    assert node._db.gallery_size(sid_anchor) == 1
    assert node._db.gallery_size(sid_other) == 1
    assert node._growth_session is None
