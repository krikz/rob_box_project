"""Unit-тесты: tars_state relay (issue #2162 follow-up, ADR-0078 §3.6).

Два новых канала в quest_node:

* ``/avatar/tts/audio_meta`` (sub, String JSON) — side-channel
  ``{request_id, sample_rate, ts_ms}`` от tts_node. Кеширует
  ``request_id → sample_rate`` для следующего ``/avatar/tts/audio``
  AudioData (tars_state НЕ шлёт — issue #3253).
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


def test_on_avatar_tts_audio_meta_caches_sample_rate_without_tars_state(quest_node_mod):
    """ADR-0078 §4: side-channel sample_rate → кеш.

    issue #3253 (Ш2): tars_state accepted отсюда больше НЕ шлётся —
    audio_meta идёт перед КАЖДЫМ чанком, accepted перебивал speaking и
    щёлкал акцепт-тоном посреди фразы.
    """
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
    assert host._avatar_request_sample_rate.get("req-meta-1") == 24000
    assert _tars_events(host) == []


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


# ── issue #3253 (Ш2): сервер сам шлёт стадии accepted → thinking → speaking → idle ──


class _Clock:
    """Монотонные часы реле: сдвигаются вручную."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


def _wired_host(quest_node_mod) -> tuple[MagicMock, _Clock]:
    """Стаб QuestNode с НАСТОЯЩИМ реле стадий (трекер + отправка в WS)."""
    from rob_box_quest.tars_stage_relay import TarsStageRelay

    clock = _Clock()
    host = _stub_host()
    host._tars_relay = TarsStageRelay(
        lambda: host.ws_server, host.get_logger(), clock=clock
    )
    host._current_avatar_request_id = None
    host._current_avatar_ws = None
    host._avatar_request_sample_rate = {}
    host._pick_active_operator_ws = MagicMock(return_value=MagicMock(name="ws"))
    host.ws_server.register_audio_session = MagicMock(return_value=True)
    host.ws_server.deliver_audio = MagicMock(return_value=True)
    return host, clock


def _audio_msg(n_bytes: int) -> MagicMock:
    # rclpy отдаёт unbounded uint8[] (AudioData.data) как array.array, не list.
    import array

    m = MagicMock()
    m.data = array.array("B", bytes(n_bytes))
    return m


def _stage_seq(host: MagicMock) -> list[str]:
    return [e["stage"] for e in _tars_events(host)]


def _speak_reply(qn, host, clock, *, n_chunks: int = 2, chunk_bytes: int = 32000) -> None:
    """supervisor → /avatar/tts/request (speech_id) → чанки в шлем по 1 с звука."""
    qn._on_avatar_tts_request_meta(host, _string_msg({
        "request_id": "abcd1234",
        "speech_id": "tars-abcd1234",
        "ssml": "<speak>готово</speak>",
        "sink": "headset",
    }))
    host._avatar_request_sample_rate["abcd1234"] = 16000
    for _ in range(n_chunks):
        clock.now += 0.1
        qn._on_avatar_tts_audio(host, _audio_msg(chunk_bytes))


def test_server_emits_accepted_thinking_speaking_idle_and_done(quest_node_mod):
    """Полный путь живой реплики: последовательность стадий + operator_tts_done."""
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)

    # 1. stt_node: wake «ТАРС» принят.
    qn._on_avatar_stt_result(host, _string_msg(
        {"source": "quest", "client_id": "c1", "text": "который час", "ts_ms": 1}
    ))
    # 2. supervisor: старт LLM.
    host._tars_relay.on_supervisor_stage(_string_msg(
        {"stage": "thinking", "request_id": "c1:1", "ts_ms": 2}
    ))
    # 3. Ответ ушёл в синтез, 2 чанка по 1 с звука.
    _speak_reply(qn, host, clock)
    # 4. tts_node: синтез закончен — звук в шлеме ещё играет, idle рано.
    host._tars_relay.on_tts_finished(_string_msg(
        {"speech_id": "tars-abcd1234", "success": True, "duration_sec": 2.0}
    ))
    host._tars_relay.tick()
    assert _stage_seq(host) == ["accepted", "thinking", "speaking"]
    host.ws_server.deliver_operator_tts_done.assert_not_called()

    # 5. Звук доиграл (+запас) → idle + operator_tts_done.
    clock.now += 3.0
    host._tars_relay.tick()
    assert _stage_seq(host) == ["accepted", "thinking", "speaking", "idle"]
    host.ws_server.deliver_operator_tts_done.assert_called_once_with("abcd1234")
    idle = _tars_events(host)[-1]
    assert idle["request_id"] == "abcd1234"
    assert idle["reason"] == "done"
    assert isinstance(idle["ts_ms"], int)
    # Дальше тишина: повторного idle/done нет.
    clock.now += 60.0
    host._tars_relay.tick()
    assert _stage_seq(host).count("idle") == 1
    host.ws_server.deliver_operator_tts_done.assert_called_once()


def test_speaking_is_sent_once_per_reply(quest_node_mod):
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)
    _speak_reply(qn, host, clock, n_chunks=5)
    assert _stage_seq(host) == ["speaking"]


def test_undelivered_chunk_does_not_claim_speaking(quest_node_mod):
    """ws закрыт (deliver_audio=False) — в шлеме ничего не звучит."""
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)
    host.ws_server.deliver_audio = MagicMock(return_value=False)
    _speak_reply(qn, host, clock)
    assert _stage_seq(host) == []


def test_barge_in_sends_idle_immediately(quest_node_mod):
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)
    _speak_reply(qn, host, clock, n_chunks=1, chunk_bytes=320000)  # 10 с звука
    quest_node_mod._notify_tars_barge_in(host)
    assert _stage_seq(host) == ["speaking", "idle"]
    assert _tars_events(host)[-1]["reason"] == "barge_in"
    host.ws_server.deliver_operator_tts_done.assert_called_once_with("abcd1234")


def test_synthesis_error_sends_idle_and_operator_tts_error(quest_node_mod):
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)
    _speak_reply(qn, host, clock, n_chunks=0)
    host._tars_relay.on_tts_finished(_string_msg(
        {"speech_id": "tars-abcd1234", "success": False, "error": "all_providers_failed"}
    ))
    assert _stage_seq(host) == ["idle"]
    assert _tars_events(host)[-1]["reason"] == "all_providers_failed"
    host.ws_server.deliver_operator_tts_error.assert_called_once_with(
        "abcd1234", "all_providers_failed"
    )
    host.ws_server.deliver_operator_tts_done.assert_not_called()


def test_tts_finished_noise_is_ignored(quest_node_mod):
    """Общий /voice/tts/finished: не-JSON, queued и чужие speech_id — мимо."""
    qn = quest_node_mod.QuestNode
    host, clock = _wired_host(quest_node_mod)
    _speak_reply(qn, host, clock, n_chunks=0)
    host._tars_relay.on_tts_finished(_string_msg("silero_warming:d1"))
    host._tars_relay.on_tts_finished(_string_msg(
        {"speech_id": "tars-abcd1234", "success": True, "queued": True}
    ))
    host._tars_relay.on_tts_finished(_string_msg({"speech_id": "other", "success": True}))
    assert _stage_seq(host) == []
    host.ws_server.deliver_operator_tts_done.assert_not_called()


def test_supervisor_idle_without_speech_is_relayed(quest_node_mod):
    """Ход без речи в шлем (агент выключен) — idle от supervisor'а."""
    host, _ = _wired_host(quest_node_mod)
    host._tars_relay.on_supervisor_stage(_string_msg(
        {"stage": "idle", "request_id": "c1:1", "reason": "agent_disabled"}
    ))
    events = _tars_events(host)
    assert [e["stage"] for e in events] == ["idle"]
    assert events[0]["reason"] == "agent_disabled"


def test_supervisor_stage_rejects_unknown_and_bad_json(quest_node_mod):
    host, _ = _wired_host(quest_node_mod)
    host._tars_relay.on_supervisor_stage(_string_msg({"stage": "speaking", "request_id": "x"}))
    host._tars_relay.on_supervisor_stage(_string_msg("not-json"))
    host._tars_relay.on_supervisor_stage(_string_msg(["thinking"]))
    assert _tars_events(host) == []


def test_bridge_barge_in_notifies_node(quest_node_mod):
    """QuestBridge.publish_voice_barge_in (PTT start) → реле стадий: idle."""
    node = MagicMock()
    bridge = quest_node_mod.QuestBridge(
        node=node,
        cmd_vel_quest_pub=MagicMock(),
        cmd_vel_emergency_pub=MagicMock(),
        tts_control_pub=MagicMock(),
        sound_stop_pub=MagicMock(),
    )
    bridge.publish_voice_barge_in()
    node._tars_relay.on_barge_in.assert_called_once_with()
