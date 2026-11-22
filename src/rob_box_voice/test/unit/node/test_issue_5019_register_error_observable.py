#!/usr/bin/env python3
"""#5019 — сбой регистрации не молчит: ROS-лог и один ``register_error``.

До правки исключение из ``register_or_merge`` (нет таблицы, ``database is locked``)
умирало в ``Future`` пула: ни строки в ``docker logs``, ни ack на
``/voice/speaker/result``. Диалог ждал 3 с и говорил «Не расслышал».

Решение в коде (ADR-0148): ``_observe_register`` ловит любое исключение кроме
уже обработанного ``AudioTooShortError``, пишет ``get_logger().error`` с
``utterance_id`` и публикует ровно один ``register_error`` (``db_error`` для
``sqlite3.Error``, ``internal`` для прочего). Успех пула в error не пишется.
"""

from __future__ import annotations

import collections
import sqlite3
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Deque, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

# Тот же приём, что в test_register_publishes_result.py: audio_common_msgs
# нет в CI, заглушка до импорта ноды.
_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_core.speaker_wire import speaker_result_to_payload  # noqa: E402
from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.utils import speaker_embeddings as se_mod  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(speaker_result_to_payload(msg))


def _embedding_stub() -> np.ndarray:
    v = np.ones(256, dtype=np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def node(tmp_path, monkeypatch):
    """Нода без ROS, как в test_register_publishes_result.py — только поля регистрации."""
    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "MIN_REGISTER_AUDIO_DURATION_SEC", 3.0)

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
    instance._growth_session = None
    instance._recent_embeddings: Deque[Tuple[float, np.ndarray, float]] = (
        collections.deque(maxlen=20)
    )
    instance._pending_register_name: Optional[str] = None
    instance._pending_register_lock = threading.Lock()
    yield instance
    instance._db.close()


def _errors(node) -> list[dict]:
    return [m for m in node._result_pub.messages if m.get("event") == "register_error"]


def _error_texts(node) -> list[str]:
    return [
        str(call.args[0]) for call in node.get_logger.return_value.error.call_args_list
    ]


def test_register_or_merge_operational_error_publishes_one_register_error(
    node, monkeypatch
):
    """sqlite3.OperationalError из register_or_merge — error-лог и ровно один ack."""
    utterance_id = "utt-5019-db"

    def _boom(*_args, **_kwargs):
        raise sqlite3.OperationalError("no such table: speakers")

    monkeypatch.setattr(node._db, "register_or_merge", _boom)

    # speaker_id задан: ветка settled (первый SQL identify) пропускается,
    # исключение приходит именно из register_or_merge.
    result = node._do_register(
        "Шифу",
        _embedding_stub(),
        speaker_id="spk-known",
        duration_sec=4.0,
        utterance_id=utterance_id,
    )

    assert result is False
    texts = _error_texts(node)
    assert len(texts) == 1, texts
    assert utterance_id in texts[0]
    assert "OperationalError" in texts[0]
    assert "no such table: speakers" in texts[0]
    assert "db_error" in texts[0]

    acks = _errors(node)
    assert len(acks) == 1, node._result_pub.messages
    assert acks[0]["error"] == "db_error"
    assert acks[0]["utterance_id"] == utterance_id
    assert acks[0]["name"] == "Шифу"
    node.get_logger.return_value.warning.assert_not_called()


def test_non_sqlite_error_is_internal_and_still_one_ack(node, monkeypatch):
    utterance_id = "utt-5019-internal"

    def _boom(*_args, **_kwargs):
        raise RuntimeError("gallery write failed")

    monkeypatch.setattr(node._db, "register_or_merge", _boom)

    result = node._do_register(
        "Нина",
        _embedding_stub(),
        speaker_id="spk-known",
        duration_sec=4.0,
        utterance_id=utterance_id,
    )

    assert result is False
    texts = _error_texts(node)
    assert len(texts) == 1
    assert "RuntimeError" in texts[0]
    assert "internal" in texts[0]
    assert utterance_id in texts[0]
    acks = _errors(node)
    assert len(acks) == 1
    assert acks[0]["error"] == "internal"
    assert acks[0]["utterance_id"] == utterance_id


def test_too_short_still_warns_once_and_does_not_error(node):
    """Старый путь AudioTooShortError не сломан и не публикуется дважды."""
    result = node._do_register(
        "Шифу",
        _embedding_stub(),
        speaker_id=None,
        duration_sec=1.0,
        utterance_id="utt-short",
    )

    assert result is False
    acks = _errors(node)
    assert len(acks) == 1
    assert acks[0]["error"] == "too_short"
    assert acks[0]["utterance_id"] == "utt-short"
    node.get_logger.return_value.error.assert_not_called()
    warnings = [
        str(c.args[0]) for c in node.get_logger.return_value.warning.call_args_list
    ]
    assert len(warnings) == 1
    assert "too short" in warnings[0]


def test_register_start_is_logged_before_sql(node, monkeypatch):
    """Первая строка воркера — «started» с utterance_id, до первого SQL."""
    seen: list[str] = []

    def _boom(*_args, **_kwargs):
        infos = [
            str(c.args[0]) for c in node.get_logger.return_value.info.call_args_list
        ]
        seen.extend(infos)
        raise sqlite3.OperationalError("database is locked")

    monkeypatch.setattr(node._db, "register_or_merge", _boom)
    node._do_register(
        "Борис",
        _embedding_stub(),
        speaker_id="spk-known",
        duration_sec=4.0,
        utterance_id="utt-start",
    )

    assert any("started" in line and "utt-start" in line for line in seen), seen


def test_pool_callback_logs_failure_and_ignores_success():
    logger = MagicMock()
    ok = MagicMock()
    ok.cancelled.return_value = False
    ok.exception.return_value = None
    sid_node._log_pool_task_error(logger, ok)
    logger.error.assert_not_called()

    bad = MagicMock()
    bad.cancelled.return_value = False
    bad.exception.return_value = sqlite3.OperationalError("database is locked")
    sid_node._log_pool_task_error(logger, bad)
    logger.error.assert_called_once()
    text = str(logger.error.call_args.args[0])
    assert "OperationalError" in text
    assert "database is locked" in text

    cancelled = MagicMock()
    cancelled.cancelled.return_value = True
    logger.error.reset_mock()
    sid_node._log_pool_task_error(logger, cancelled)
    logger.error.assert_not_called()
    cancelled.exception.assert_not_called()


def _raise_locked():
    raise sqlite3.OperationalError("database is locked")


def test_submit_logged_done_callback_sees_raised_task():
    """submit + done-callback: упавшая задача пишет error, успешная — нет."""
    logger = MagicMock()
    with ThreadPoolExecutor(max_workers=1) as pool:
        ok = sid_node._submit_logged(pool, logger, lambda: "fine")
        assert ok.result(timeout=2) == "fine"
        bad = sid_node._submit_logged(pool, logger, _raise_locked)
        with pytest.raises(sqlite3.OperationalError):
            bad.result(timeout=2)
    logger.error.assert_called_once()
    assert "OperationalError" in str(logger.error.call_args.args[0])


def test_failure_reason_splits_sqlite_from_the_rest():
    assert (
        sid_node._register_failure_reason(sqlite3.OperationalError("x")) == "db_error"
    )
    assert sid_node._register_failure_reason(sqlite3.DatabaseError("x")) == "db_error"
    assert sid_node._register_failure_reason(RuntimeError("x")) == "internal"
