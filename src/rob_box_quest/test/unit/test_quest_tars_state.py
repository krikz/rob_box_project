"""Unit-тесты: tars_state relay (issue #2162 follow-up, ADR-0078 §3.6).

Два новых канала в quest_node:

* ``/avatar/tts/audio_meta`` (sub, String JSON) — side-channel
  ``{request_id, sample_rate, ts_ms}`` от tts_node. Публикует
  ``tars_state accepted`` в WS + кеширует ``request_id → sample_rate``
  для следующего ``/avatar/tts/audio`` AudioData.
* ``/avatar/stt/result`` (sub, String JSON) — wake-word принят
  (stt_node публикует после вейка «ТАРС»). Триггерит
  ``tars_state accepted`` с текстом.

Контракт WS-эвента: см. ``webxr_client/src/ui/tars_state_indicator.ts``.

QuestNode импортирует ROS-msg-пакеты (audio_common_msgs и др.) — фикстура
``quest_node_mod`` из conftest.py ставит stubs (issue #2135). В отличие от
test_quest_avatar_command_result.py (Docker-only), этот файл работает и
на dev-env, и в Docker.
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock


# Topic-контракты — ADR-0078 §4.
AVATAR_TTS_AUDIO_META_TOPIC = "/avatar/tts/audio_meta"
AVATAR_STT_RESULT_TOPIC = "/avatar/stt/result"


def _stub_host() -> MagicMock:
    """Stub с минимальным интерфейсом, который требуют handler'ы quest_node."""
    host = MagicMock()
    host.ws_server = MagicMock()
    host.get_logger = MagicMock(return_value=MagicMock())
    # Кеш sample_rate (per-request) — кешируется в _on_avatar_tts_audio_meta.
    host._avatar_request_sample_rate = {}
    return host


def _string_msg(payload) -> MagicMock:
    m = MagicMock()
    if isinstance(payload, str):
        m.data = payload
    else:
        m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _tars_events(host: MagicMock) -> list[dict]:
    """Извлечь все WS-эвенты типа 'tars_state' из broadcast_json_event."""
    out: list[dict] = []
    for call in host.ws_server.broadcast_json_event.call_args_list:
        event = call.args[0]
        if event.get("type") == "tars_state":
            out.append(event)
    return out


def test_avatar_tts_audio_meta_topic_constant():
    assert AVATAR_TTS_AUDIO_META_TOPIC == "/avatar/tts/audio_meta"


def test_avatar_stt_result_topic_constant():
    assert AVATAR_STT_RESULT_TOPIC == "/avatar/stt/result"


def test_on_avatar_tts_audio_meta_caches_sample_rate_and_emits_tars_state(quest_node_mod):
    """ADR-0078 §4: side-channel sample_rate → кеш + tars_state accepted."""
    host = _stub_host()
    host._current_avatar_request_id = "req-meta-1"
    quest_node_mod.QuestNode._on_avatar_tts_audio_meta(
        host,
        _string_msg(
            {
                "request_id": "req-meta-1",
                "sample_rate": 24000,
                "ts_ms": 1234567890,
            }
        ),
    )
    # sample_rate закэширован.
    assert host._avatar_request_sample_rate.get("req-meta-1") == 24000
    # WS-event типа tars_state, stage=accepted, request_id, ts_ms.
    events = _tars_events(host)
    assert len(events) == 1
    assert events[0]["stage"] == "accepted"
    assert events[0]["request_id"] == "req-meta-1"
    assert "ts_ms" in events[0]


def test_on_avatar_tts_audio_meta_without_current_request_id_is_ignored(quest_node_mod):
    """Нет активной сессии — audio_meta кеширует SR, но НЕ публикует в WS.

    Кеш sample_rate нужен для возможных будущих чанков (race-condition
    между meta и audio), а WS-event не публикуем, чтобы клиент не
    показывал принято, когда нет активной сессии.
    """
    host = _stub_host()
    host._current_avatar_request_id = None
    quest_node_mod.QuestNode._on_avatar_tts_audio_meta(
        host,
        _string_msg({"request_id": "x", "sample_rate": 16000, "ts_ms": 0}),
    )
    host.ws_server.broadcast_json_event.assert_not_called()


def test_on_avatar_tts_audio_meta_bad_json_is_dropped(quest_node_mod):
    host = _stub_host()
    quest_node_mod.QuestNode._on_avatar_tts_audio_meta(host, _string_msg("not-json"))
    host.ws_server.broadcast_json_event.assert_not_called()


def test_on_avatar_stt_result_broadcasts_tars_state_accepted_with_text(quest_node_mod):
    """stt_node → /avatar/stt/result → WS tars_state accepted (с текстом)."""
    host = _stub_host()
    quest_node_mod.QuestNode._on_avatar_stt_result(
        host,
        _string_msg(
            {
                "source": "quest",
                "client_id": "session-42",
                "text": "расскажи анекдот",
                "ts_ms": 1234567890,
            }
        ),
    )
    events = _tars_events(host)
    assert len(events) == 1
    assert events[0]["stage"] == "accepted"
    assert events[0]["text"] == "расскажи анекдот"
    assert events[0]["request_id"] == "session-42:1234567890"
    assert "ts_ms" in events[0]


def test_on_avatar_stt_result_empty_text_does_not_broadcast(quest_node_mod):
    host = _stub_host()
    quest_node_mod.QuestNode._on_avatar_stt_result(
        host,
        _string_msg({"source": "quest", "client_id": "x", "text": "", "ts_ms": 0}),
    )
    host.ws_server.broadcast_json_event.assert_not_called()


def test_on_avatar_stt_result_bad_json_is_dropped(quest_node_mod):
    host = _stub_host()
    quest_node_mod.QuestNode._on_avatar_stt_result(host, _string_msg("not-json"))
    host.ws_server.broadcast_json_event.assert_not_called()


def test_on_avatar_tts_audio_uses_cached_sample_rate(quest_node_mod):
    """_on_avatar_tts_audio подставляет sample_rate из кеша audio_meta."""
    host = _stub_host()
    host._current_avatar_request_id = "req-1"
    host._current_avatar_ws = MagicMock()
    host._avatar_request_sample_rate = {"req-1": 24000}
    # AudioData с int16-LE.
    audio_msg = MagicMock()
    audio_msg.data = [0] * 480  # 10мс @ 24kHz моно
    quest_node_mod.QuestNode._on_avatar_tts_audio(host, audio_msg)
    # deliver_audio вызван с sample_rate=24000.
    host.ws_server.deliver_audio.assert_called_once()
    kwargs = host.ws_server.deliver_audio.call_args.kwargs
    assert kwargs.get("sample_rate") == 24000


def test_on_avatar_tts_audio_falls_back_to_16khz_when_no_cache(quest_node_mod):
    host = _stub_host()
    host._current_avatar_request_id = "req-2"
    host._current_avatar_ws = MagicMock()
    host._avatar_request_sample_rate = {}  # пусто
    audio_msg = MagicMock()
    audio_msg.data = [0] * 320
    quest_node_mod.QuestNode._on_avatar_tts_audio(host, audio_msg)
    kwargs = host.ws_server.deliver_audio.call_args.kwargs
    assert kwargs.get("sample_rate") == 16000  # fallback
