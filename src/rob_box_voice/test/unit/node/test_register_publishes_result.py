#!/usr/bin/env python3
"""
test_register_publishes_result.py — обвязка speaker_id_node поверх
issue #2747 (растущая галерея) и issue #2748 (имя доезжает до лица).

До этой пары исправлений:
  * профиль диктора оставался с ОДНИМ эмбеддингом навсегда (issue #2747);
  * ``_do_register()`` публиковал ТОЛЬКО служебный ack
    (``{"event": "registered", ...}``) — vision_face_node ждёт
    ``is_known=true`` в ``/voice/speaker/result`` и в момент регистрации
    его никогда не получал (issue #2748).

Тесты здесь — про НОДУ (в отличие от test_gallery_warmup.py, который
проверяет чистую логику SpeakerDatabase):
    1. ``_do_register()`` публикует ДВА сообщения: ack (не тронут) и новый
       SpeakerMatch с ``is_known=true``, ``source="register"``.
    2. ``_process_utterance()`` при обычном (не pending) успешном identify()
       дописывает эмбеддинг в галерею, пока она маленькая.
    3. ``_process_utterance()`` в ветке ``pending_name`` НЕ публикует
       второе, потенциально расходящееся сообщение — доверяет
       ``_do_register()``.

Тот же приём, что в test_epithet_wiring.py: ``SpeakerIdNode.__init__`` не
вызывается (там ROS-параметры, ThreadPool, resemblyzer warmup) — нода
собирается через ``object.__new__`` и получает только те поля, которые
нужны проверяемому коду. ``embed_audio`` подменяется на фиксированный
вектор — resemblyzer недоступен на dev-машине / в CI.
"""

from __future__ import annotations

import collections
import json
import sys
import threading
import types
from pathlib import Path
from typing import Deque, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

# ``audio_common_msgs`` не покрыт общим conftest (его тянет только
# speaker_id_node) — доставляем заглушку до импорта ноды (тот же приём,
# что в test_epithet_wiring.py).
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


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    noise = _embedding(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


class _FakePublisher:
    """Собирает опубликованные ``String``-сообщения как декодированный JSON."""

    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(json.loads(msg.data))


@pytest.fixture()
def node(tmp_path, monkeypatch):
    """SpeakerIdNode без ROS-инициализации, с реальной БД во временном файле.

    Калиброванные модульные константы фиксируются monkeypatch'ем — тест не
    должен зависеть от того, что происходит в других test_*.py, которые
    патчат те же атрибуты speaker_embeddings (общий модуль в sys.modules).
    """
    import rob_box_voice.utils.speaker_embeddings as se_mod

    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SIZE", 5)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SOFT_THRESHOLD", 0.45)

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._speech_log = {}
    instance._speech_log_lock = threading.Lock()
    instance._sample_rate = 16000
    instance._recent_embeddings: Deque[Tuple[float, np.ndarray]] = collections.deque(maxlen=20)
    instance._MAX_EMBED_AGE_SEC = 30.0
    instance._pending_register_name: Optional[str] = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
    # epithet_request publisher — _ensure_epithet -> _assign_epithet ->
    # _request_llm_epithet его читает через getattr(..., None).
    instance._epithet_request_pub = None
    yield instance
    instance._db.close()


# ---------------------------------------------------------------------------
# 1. _do_register публикует ack + is_known=true source="register"
# ---------------------------------------------------------------------------


def test_do_register_publishes_ack_and_speaker_match(node):
    emb = _embedding(1)
    node._do_register("Деньчик", emb, speaker_id=None)

    kinds = [m.get("event") for m in node._result_pub.messages]
    assert "registered" in kinds, "старый ack не должен пропасть (обратная совместимость)"

    # Issue #2748 — второе сообщение: полноценный SpeakerMatch.
    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1, (
        f"ожидалось ровно одно is_known=true сообщение, получено "
        f"{len(match_msgs)}: {node._result_pub.messages!r}"
    )
    payload = match_msgs[0]
    assert payload["name"] == "Деньчик"
    assert payload["source"] == "register"
    assert payload["speaker_id"]
    assert payload["confidence"] > 0.99, "self-similarity только что записанного эмбеддинга ~1.0"


def test_do_register_ack_message_unchanged_shape(node):
    """dialogue_node/mcp_server/voice_adapter завязаны на форму ack —
    регрессия здесь тихо сломает их обработку 'event'=='registered'."""
    emb = _embedding(2)
    node._do_register("Саша", emb, speaker_id=None)

    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert set(ack) >= {"event", "name", "speaker_id", "reused_profile"}
    assert ack["name"] == "Саша"
    assert ack["reused_profile"] is False


def test_do_register_on_name_conflict_still_publishes_own_match(node):
    """ADR-0127: конфликт имён заводит ОТДЕЛЬНЫЙ профиль — is_known=true
    обязан указывать на НОВЫЙ профиль (Борис), а не на чужой (Саша)."""
    base = _embedding(700)
    similar = _degraded(base, alpha=0.6, noise_seed=701)  # cos ~0.86 > REGISTER_MATCH_THRESHOLD

    node._do_register("Саша", base, speaker_id=None)
    node._result_pub.messages.clear()

    node._do_register("Борис", similar, speaker_id=None)

    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1
    assert match_msgs[0]["name"] == "Борис"
    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert "voice_conflict" in ack


# ---------------------------------------------------------------------------
# 2. _process_utterance: growing gallery на обычном (не pending) identify()
# ---------------------------------------------------------------------------


def test_process_utterance_grows_gallery_on_successful_identify(node):
    base = _embedding(10)
    sid = node._db.register("Деньчик", base)
    assert node._db.gallery_size(sid) == 1

    # cos~0.523 к единственному эталону — то самое измерение issue #2747,
    # проходит адаптивный порог (0.45 при gallery_size=1).
    alpha = float((1.0 / 0.523 ** 2 - 1.0) ** 0.5)
    second = _degraded(base, alpha=alpha, noise_seed=11)
    node._db.embed_audio = MagicMock(return_value=second)

    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid) == 2, (
        "успешный identify() на маленькой галерее обязан дописать эмбеддинг "
        "(issue #2747)"
    )
    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1
    assert match_msgs[0]["name"] == "Деньчик"
    assert "source" not in match_msgs[0], (
        "обычная идентификация не должна помечаться source='register'"
    )


def test_process_utterance_stops_growing_once_warmup_size_reached(node):
    base = _embedding(20)
    sid = node._db.register("Деньчик", base)
    for i in range(1, 5):
        node._db.register("Деньчик", _degraded(base, 0.1, 2000 + i), speaker_id=sid)
    assert node._db.gallery_size(sid) == 5

    node._db.embed_audio = MagicMock(return_value=_degraded(base, 0.1, 2999))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid) == 5, "галерея не должна расти после WARMUP_SIZE"


def test_process_utterance_publishes_unknown_for_unmatched_voice(node):
    node._db.register("Саша", _embedding(30))
    node._db.embed_audio = MagicMock(return_value=_embedding(31))  # ортогональный голос

    node._process_utterance(b"\x00\x00" * 1000)

    assert node._result_pub.messages == [{"is_known": False}]


# ---------------------------------------------------------------------------
# 3. _process_utterance: pending_name-ветка не дублирует публикацию
# ---------------------------------------------------------------------------


def test_pending_name_registration_publishes_exactly_one_known_result(node):
    emb = _embedding(40)
    node._db.embed_audio = MagicMock(return_value=emb)
    node._pending_register_name = "Эйджик"

    node._process_utterance(b"\x00\x00" * 1000)

    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1, (
        f"ожидалось ровно одно is_known=true сообщение (от _do_register), "
        f"получено {len(match_msgs)}: {node._result_pub.messages!r}"
    )
    assert match_msgs[0]["source"] == "register"
    assert match_msgs[0]["name"] == "Эйджик"
    # pending сброшен, чтобы следующая реплика не перерегистрировалась.
    assert node._pending_register_name is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
