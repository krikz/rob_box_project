#!/usr/bin/env python3
"""
test_issue_2863_keep_known_speaker.py

Issue #2863 — узнанный диктор не должен сбрасываться событиями, которые
его личность не опровергают (сторона speaker_id_node).

Живой лог (E2E акт 2, run 35886659057)::

    [speaker_id_node] 👤 Speaker: 'Саша' confidence=0.877 … речь=2.67s
    [mcp_server]      register_speaker {'name': 'Саша', 'utterance_id': 'e8f9080d4787'}
    [speaker_id_node] ⚠️ [issue #2769] Registration of 'Саша' rejected — 2.67s < 3.0s
    [mcp_server]      👤 [issue 1770] current_speaker_id: 2b276f43-… → ∅
    [tts_node]        TTS: text='Не расслышал — скажи, пожалуйста, ещё пару слов…'

Контракт после фикса:

1. ``register_speaker`` с именем, под которым ЭТА ЖЕ фраза уже узнана
   (identify_threshold), — ack ``event="registered", already_known=true``,
   а не ``register_error`` и не новый профиль-тёзка.
2. Фраза, узнанная как ДРУГОЕ имя, по-прежнему честно отказывается по
   too_short (узнанность под чужим именем — не «уже знаю»).
3. «Не узнал» по фразе с малым количеством речи (< identify_min_voiced_sec)
   или без эмбеддинга публикуется с ``inconclusive=true``; узнанный
   результат и «не узнал» по достаточной речи — без этой пометки (сброс
   по ним допустим, #2829).

Та же сборка ноды, что в test_issue_2829_register_bound_to_utterance.py
(``object.__new__(SpeakerIdNode)`` — без ROS2 и resemblyzer).
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
    def __init__(self, embedding, voiced_sec: float) -> None:
        self.embedding = embedding
        self.voiced_sec = voiced_sec
        self.raw_sec = max(voiced_sec, 0.0)


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    v = base + alpha * _embedding(noise_seed)
    return (v / np.linalg.norm(v)).astype(np.float32)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(json.loads(msg.data))


def _msg(payload):
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
    instance._growth_owner_min_score = 0.65
    instance._name_confidence_min_gap = 0.15
    instance._name_confidence_band_high = 0.80
    instance._identify_min_voiced_sec = 1.0
    instance._executor = ThreadPoolExecutor(max_workers=2)
    yield instance
    instance._executor.shutdown(wait=True)
    instance._db.close()


def _wait_for_event(node, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(m.get("event") for m in node._result_pub.messages):
            return True
        time.sleep(0.01)
    return False


def _events(node, name):
    return [m for m in node._result_pub.messages if m.get("event") == name]


def _cos(a, b) -> float:
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))


# ---------------------------------------------------------------------------
# 1. Повторная регистрация уже узнанного — «уже знаю», не отказ
# ---------------------------------------------------------------------------


def test_repeat_register_of_recognised_speaker_too_short_is_already_known(node):
    """Живой случай: Саша узнан (0.877), фраза 2.67с < 3.0с, LLM зовёт
    register_speaker('Саша') — раньше register_error too_short (→ «Не
    расслышал» + сброс в mcp_server)."""
    base = _embedding(1)
    sid = node._db.register("Саша", base, duration_sec=5.0)
    phrase = _degraded(base, 0.55, noise_seed=101)
    assert _cos(base, phrase) >= 0.72, "предусловие: фраза узнаётся как Саша"
    node._recent_embeddings.append((time.time(), phrase, 2.67, "e8f9080d4787"))

    node._on_register_request(_msg({"name": "Саша", "utterance_id": "e8f9080d4787"}))
    assert _wait_for_event(node)

    assert _events(node, "register_error") == [], (
        "уже узнанный под этим именем — не отказ регистрации"
    )
    registered = _events(node, "registered")
    assert len(registered) == 1
    assert registered[0]["already_known"] is True
    assert registered[0]["speaker_id"] == sid
    assert registered[0]["utterance_id"] == "e8f9080d4787"
    assert len(node._db.list_speakers()) == 1, "новый якорь не заведён"
    assert node._db.gallery_size(sid) == 1, "короткая фраза в эталон не пишется"


def test_repeat_register_between_identify_and_merge_threshold_no_twin(node):
    """score в [identify_threshold, REGISTER_MATCH_THRESHOLD) при длинной
    фразе: register_or_merge завёл бы профиль-тёзку (новый якорь) —
    человек же уже узнан под этим именем."""
    base = _embedding(2)
    sid = node._db.register("Саша", base, duration_sec=5.0)
    phrase = None
    for seed in range(200, 400):
        cand = _degraded(base, 0.92, noise_seed=seed)
        if 0.725 <= _cos(base, cand) < 0.745:
            phrase = cand
            break
    assert phrase is not None, "предусловие: нашли фразу в полосе 0.72-0.75"
    node._recent_embeddings.append((time.time(), phrase, 4.5, "utt-band"))

    node._on_register_request(_msg({"name": "саша", "utterance_id": "utt-band"}))
    assert _wait_for_event(node)

    assert len(node._db.list_speakers()) == 1, "профиль-тёзка не заведён"
    registered = _events(node, "registered")
    assert len(registered) == 1 and registered[0]["already_known"] is True
    assert registered[0]["speaker_id"] == sid


def test_already_known_opens_growth_session_for_owner(node):
    """Рост галереи — по правилам #2833: через growth-сессию владельца."""
    base = _embedding(3)
    sid = node._db.register("Саша", base, duration_sec=5.0)
    phrase = _degraded(base, 0.55, noise_seed=103)
    node._recent_embeddings.append((time.time(), phrase, 2.0, "utt-g"))

    node._on_register_request(_msg({"name": "Саша", "utterance_id": "utt-g"}))
    assert _wait_for_event(node)

    assert node._growth_session is not None
    assert node._growth_session["speaker_id"] == sid


def test_long_confident_repeat_register_still_merges_into_profile(node):
    """Не ломаем штатный путь: длинная фраза выше порога слияния —
    register_or_merge дописывает её в тот же профиль (reused_profile)."""
    base = _embedding(4)
    sid = node._db.register("Саша", base, duration_sec=5.0)
    phrase = _degraded(base, 0.3, noise_seed=104)
    assert _cos(base, phrase) >= 0.75
    node._recent_embeddings.append((time.time(), phrase, 5.0, "utt-long"))

    node._on_register_request(_msg({"name": "Саша", "utterance_id": "utt-long"}))
    assert _wait_for_event(node)

    registered = _events(node, "registered")
    assert len(registered) == 1
    assert registered[0]["reused_profile"] is True
    assert "already_known" not in registered[0]
    assert node._db.gallery_size(sid) == 2


def test_short_register_under_other_name_still_refused(node):
    """Фраза узнана как Саша, а регистрируют «Борис» — это не «уже знаю»:
    честный too_short, как раньше (#2769)."""
    base = _embedding(5)
    node._db.register("Саша", base, duration_sec=5.0)
    phrase = _degraded(base, 0.55, noise_seed=105)
    node._recent_embeddings.append((time.time(), phrase, 2.0, "utt-other"))

    node._on_register_request(_msg({"name": "Борис", "utterance_id": "utt-other"}))
    assert _wait_for_event(node)

    errors = _events(node, "register_error")
    assert len(errors) == 1 and errors[0]["error"] == "too_short"
    assert _events(node, "registered") == []


# ---------------------------------------------------------------------------
# 2. «Не узнал» по короткой речи — inconclusive, по достаточной — нет
# ---------------------------------------------------------------------------


def _run_utterance(node, embedding, voiced_sec, utterance_id="utt-x"):
    node._db.embed_audio_ex = MagicMock(
        return_value=None if embedding is None else _Embed(embedding, voiced_sec)
    )
    node._process_utterance(b"\x00\x00" * 16000, utterance_id)
    results = [m for m in node._result_pub.messages if "is_known" in m]
    assert len(results) == 1
    return results[0]


def test_short_unknown_phrase_is_inconclusive(node):
    """Живой лог: речь=0.69s, best='Борис' 0.514 → is_known=false →
    mcp_server сбрасывал текущего. Это «не знаю», не «другой человек»."""
    node._db.register("Борис", _embedding(6), duration_sec=5.0)
    payload = _run_utterance(node, _embedding(66), voiced_sec=0.69)

    assert payload["is_known"] is False
    assert payload.get("inconclusive") is True
    assert payload.get("reason") == "too_short_for_biometry"
    assert payload.get("utterance_id") == "utt-x"


def test_no_embedding_is_inconclusive(node):
    payload = _run_utterance(node, None, voiced_sec=0.0)

    assert payload["is_known"] is False
    assert payload.get("inconclusive") is True
    assert payload.get("reason") == "no_embedding"


def test_long_unknown_phrase_is_a_real_evaluation(node):
    """Достаточно речи и «не узнал» — фразу реально оценили: сброс
    допустим (#2829: чужой голос не наследует имя)."""
    node._db.register("Борис", _embedding(7), duration_sec=5.0)
    payload = _run_utterance(node, _embedding(77), voiced_sec=2.5)

    assert payload["is_known"] is False
    assert "inconclusive" not in payload


def test_short_but_recognised_phrase_is_published_as_known(node):
    """Короткая фраза, но identify_threshold пройден — это оценка: смена
    диктора по ней остаётся возможной."""
    base = _embedding(8)
    node._db.register("Борис", base, duration_sec=5.0)
    payload = _run_utterance(node, _degraded(base, 0.2, noise_seed=108), voiced_sec=0.6)

    assert payload["is_known"] is True
    assert "inconclusive" not in payload
