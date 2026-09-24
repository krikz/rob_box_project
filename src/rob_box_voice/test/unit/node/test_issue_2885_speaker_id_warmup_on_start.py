#!/usr/bin/env python3
"""
test_issue_2885_speaker_id_warmup_on_start.py

Issue #2885 — первая фраза после рестарта voice-assistant обрабатывалась
биометрией 40–53 с (живой лог E2E run 35900007906: ``Speaker: unknown
(41027 ms)``, а следующая фраза — 961 ms). Прогрев resemblyzer был
отложен до первого ``embed_audio`` (#2609), и весь холодный старт
доставался первой живой фразе.

Замер (x86, python:3.10 + torch 2.8.0+cpu, resemblyzer 0.1.4, librosa
0.11.0, numba 0.67.0; 9-секундный буфер): первый проход 39 894 ms, из них
``preprocess_wav`` 36 293 ms — а внутри него первый ``librosa.resample``
(ленивый импорт librosa.core/util + numba-компиляция их ufunc'ов) 60 632 ms
в отдельном прогоне; второй проход 307 ms. Загрузка самой модели — 55 ms,
что и объясняет «Loaded the voice encoder model on cpu in 0.05 seconds».

Контракт после фикса:

1. Прогрев запускается при старте ноды по умолчанию (без правки YAML).
2. ``__init__`` его не ждёт: подписки создаются, пока прогрев идёт.
3. Фраза, пришедшая до конца прогрева, не теряется — встаёт в очередь
   того же executor'а и обрабатывается сразу после прогрева.
4. Прогрев идёт через ``embed_audio_ex`` и проверяет результат: если до
   энкодера не дошёл — это видно в логе (warning), а не молча. (Замер:
   прежний буфер 1 с гаусс-шума доходит, речь=0.99s; следующая 9-с фраза
   после прогрева — 315 ms, без прогрева — 47 355 ms.)

Ноду собираем НАСТОЯЩИМ ``SpeakerIdNode()`` поверх общего ``FakeNode``
(``test/ros_stubs.py`` через ``test/unit/node/conftest.py``) — иначе
не проверить, что именно ``__init__`` запускает прогрев. БД и резёмблизер
подменены: тест про порядок и неблокирование, не про точность.
"""

from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path
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


class _FakeDb:
    """Заглушка SpeakerDatabase: без SQLite и resemblyzer."""

    def __init__(self, db_path: str) -> None:
        self.db_path = db_path
        self.embed_calls: list[int] = []
        self.result = types.SimpleNamespace(
            embedding=np.zeros(256, dtype=np.float32), raw_sec=2.0, voiced_sec=2.0
        )

    def embed_audio_ex(self, pcm_bytes: bytes, sample_rate: int = 16000):
        self.embed_calls.append(len(pcm_bytes))
        return self.result

    def close(self) -> None:
        pass


@pytest.fixture()
def gated_node(monkeypatch):
    """Нода, чей прогрев висит, пока тест не отпустит ``release``."""
    started = threading.Event()
    release = threading.Event()
    processed: list[str] = []
    processed_evt = threading.Event()

    def _blocking_warmup(self) -> None:
        started.set()
        release.wait(timeout=10.0)

    def _record_utterance(self, pcm_bytes, utterance_id=None) -> None:
        processed.append(utterance_id)
        processed_evt.set()

    monkeypatch.setattr(sid_node, "SpeakerDatabase", _FakeDb)
    monkeypatch.setattr(sid_node, "start_metrics_server", lambda port: False)
    monkeypatch.setattr(sid_node.SpeakerIdNode, "_warmup", _blocking_warmup)
    monkeypatch.setattr(
        sid_node.SpeakerIdNode, "_process_utterance", _record_utterance
    )

    t0 = time.monotonic()
    node = sid_node.SpeakerIdNode()
    init_sec = time.monotonic() - t0
    ctx = types.SimpleNamespace(
        node=node,
        init_sec=init_sec,
        started=started,
        release=release,
        processed=processed,
        processed_evt=processed_evt,
    )
    yield ctx
    release.set()
    node._executor.shutdown(wait=True)


def _speech_msg(seconds: float = 1.0):
    pcm = (np.random.default_rng(7).normal(0, 0.1, int(16000 * seconds)) * 32767)
    return types.SimpleNamespace(data=pcm.astype(np.int16).tobytes())


def test_warmup_starts_at_startup_by_default(gated_node) -> None:
    """Без правки YAML прогрев запускается сразу после старта ноды."""
    assert gated_node.started.wait(timeout=2.0), (
        "SpeakerIdNode.__init__ did not start the resemblyzer warmup — the "
        "first phrase after restart pays the 40-60 s cold start (issue #2885)"
    )


def test_warmup_does_not_block_init_or_subscriptions(gated_node) -> None:
    """__init__ вернулся и подписался, пока прогрев ещё висит."""
    assert gated_node.started.wait(timeout=2.0), "warmup never started"
    assert not gated_node.release.is_set()
    assert gated_node.init_sec < 2.0, (
        f"__init__ took {gated_node.init_sec:.2f}s — it must not wait for warmup"
    )
    topics = [topic for topic, _ in gated_node.node._subscribers]
    assert "/audio/speech_audio" in topics
    assert "/voice/speaker/register" in topics


def test_phrase_during_warmup_is_queued_not_lost(gated_node) -> None:
    """Фраза до конца прогрева ждёт в очереди и обрабатывается после него."""
    assert gated_node.started.wait(timeout=2.0), "warmup never started"

    msg = _speech_msg()
    gated_node.node._on_speech_audio(msg)
    expected_id = sid_node.compute_utterance_id(bytes(msg.data))

    # Прогрев ещё идёт — фраза не должна обработаться раньше него
    # (один executor = один поток инференса, без гонки _load_resemblyzer).
    assert not gated_node.processed_evt.wait(timeout=0.3)

    gated_node.release.set()
    assert gated_node.processed_evt.wait(timeout=2.0), (
        "phrase received during warmup was lost — never processed"
    )
    assert gated_node.processed == [expected_id]


def test_warmup_logs_warning_when_encoder_not_reached() -> None:
    """Если прогрев не дошёл до энкодера — warning в логе, не молча."""
    node = object.__new__(sid_node.SpeakerIdNode)
    node._db = _FakeDb("unused")
    node._db.result = None
    logger = MagicMock()
    node.get_logger = MagicMock(return_value=logger)

    node._warmup()

    assert node._db.embed_calls, "warmup must go through embed_audio_ex"
    logger.warning.assert_called_once()
    assert "#2885" in logger.warning.call_args[0][0]


def test_warmup_logs_done_with_voiced_duration() -> None:
    node = object.__new__(sid_node.SpeakerIdNode)
    node._db = _FakeDb("unused")
    logger = MagicMock()
    node.get_logger = MagicMock(return_value=logger)

    node._warmup()

    logger.warning.assert_not_called()
    done = [c[0][0] for c in logger.info.call_args_list if "warmup done" in c[0][0]]
    assert done and "речь=2.00s" in done[0]
