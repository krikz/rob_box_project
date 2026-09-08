"""Integration-тесты голосового passthrough (рация) в WS-сервере.

Проверяет контракт (план P3, Task 3.2):
- voice_ptt_start → Bridge.publish_voice_barge_in() (STOP TTS + sound)
- VOICE_AUDIO frame → Bridge.publish_voice_audio(payload)
- voice_ptt_stop → Bridge.publish_voice_stop()

Без rclpy/ROS/Zenoh — подменяем Bridge на записывающий stub.
"""

import asyncio
import json
import time
from typing import Any

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app


pytestmark = pytest.mark.asyncio


class RecordingBridge(NoOpBridge):
    """Stub, записывающий вызовы голосовых методов Bridge."""

    def __init__(self) -> None:
        self.barge_in_calls = 0
        self.voice_audio_payloads: list[bytes] = []
        self.voice_stop_calls = 0
        self.robot_start_calls = 0
        self.robot_stop_calls = 0
        self.voice_modes: list[str] = []
        # ADR-0071 step 5a: wake stream (stream_id=2) routing.
        self.wake_audio_payloads: list[bytes] = []
        self.wake_stream_state_changes: list[bool] = []  # True=active, False=paused
        # AV-27 / issue #1919 — TTS picker state.
        self.voices_snapshots: list[dict[str, Any]] = []
        self.set_voice_calls: list[tuple[str, str | None]] = []
        self.preview_voice_calls: list[tuple[str, str, str]] = (
            []
        )  # (request_id, voice_id, text)
        # Параметры, задаваемые в каждом тесте:
        self.active_provider: str = "yandex"
        self.active_voice: str = "alena"
        self.voices_payload: list[dict[str, Any]] = []
        # AV-28 §P7: дополнительные каналы для preset + language.
        self.voice_presets: list[str] = []
        self.voice_languages: list[str] = []
        self.set_voice_preset_calls: int = 0
        self.set_voice_language_calls: int = 0
        # Шаг 4б grip-pipeline config (issue #1989 → карточка t_80e7aa1e):
        # /avatar/voice_pipeline payload'ы, зафиксированные для ассертов.
        # Запись всех трёх полей позволяет тестам проверить сериализацию под
        # _on_grip_voice_pipeline (см. supervisor_node.py:2634): {llm_enabled,
        # preset, language}.
        self.voice_pipeline_payloads: list[dict] = []
        self.publish_voice_pipeline_calls: int = 0

    def publish_voice_barge_in(self) -> None:
        self.barge_in_calls += 1

    def publish_voice_audio(self, payload: bytes) -> None:
        self.voice_audio_payloads.append(payload)

    def publish_quest_wake_audio(self, payload: bytes) -> None:
        # ADR-0071 step 5a: stream_id=2 → wake-канал, публикуется в
        # /audio/quest_wake (issue #1992, реализация в QuestBridge). Здесь —
        # NoOpBridge-стаб для теста маршрутизации sid==2 на уровне WS-сервера,
        # ROS-публикация не участвует.
        self.wake_audio_payloads.append(payload)

    def set_wake_stream_state(self, active: bool) -> None:
        # Меняется при JSON_CMD{cmd: voice_listen_start/stop}.
        self.wake_stream_state_changes.append(active)

    def publish_voice_stop(self) -> None:
        self.voice_stop_calls += 1

    def publish_voice_robot_start(self) -> None:
        self.robot_start_calls += 1

    def publish_voice_robot_stop(self) -> None:
        self.robot_stop_calls += 1

    def set_voice_mode(self, mode: str) -> None:
        self.voice_modes.append(mode)

    # ── AV-27 / issue #1919 — TTS picker stubs ────────────────────────

    def list_voices_snapshot(self) -> dict[str, Any]:
        snap = {
            "voices": list(self.voices_payload),
            "active_provider": self.active_provider,
            "active_voice": self.active_voice,
        }
        self.voices_snapshots.append(snap)
        return snap

    def set_voice(
        self, voice_id: str, preset: str | None
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
        self.set_voice_calls.append((voice_id, preset))
        # Простая логика: voice_id должен в в available_voices для active_provider.
        # Если active_provider пуст — tts_unreachable.
        if not self.active_provider:
            return False, None, "tts_unreachable", None
        if voice_id not in self.voices_payload and voice_id not in [
            v.get("voice_id") for v in self.voices_payload
        ]:
            available = [
                v.get("voice_id") for v in self.voices_payload if isinstance(v, dict)
            ]
            return False, None, "voice_unavailable", available
        return True, voice_id, None, None

    def publish_preview_voice(self, request_id: str, voice_id: str, text: str) -> None:
        self.preview_voice_calls.append((request_id, voice_id, text))

    # ── AV-28 §P7 stubs (issue #1920) ────────────────────────────────

    def set_voice_preset(self, preset: str) -> None:
        self.voice_presets.append(preset)
        self.set_voice_preset_calls += 1

    def set_voice_language(self, language: str) -> None:
        self.voice_languages.append(language)
        self.set_voice_language_calls += 1

    # ── Шаг 4б grip-pipeline config (t_80e7aa1e) ─────────────────────
    # Контракт: сериализация должна ТОЧНО совпасть с тем, что парсит
    # supervisor_node._on_grip_voice_pipeline (см. supervisor_node.py:2634).
    # Тест ``test_voice_pipeline_payload_matches_supervisor_contract``
    # прибивает это как регрессию.
    def publish_voice_pipeline(
        self, llm_enabled: bool, preset: str, language: str
    ) -> None:
        self.publish_voice_pipeline_calls += 1
        self.voice_pipeline_payloads.append(
            {"llm_enabled": bool(llm_enabled), "preset": str(preset), "language": str(language)}
        )


@pytest.fixture
def bridge() -> RecordingBridge:
    return RecordingBridge()


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client(fixed_pin, bridge):
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as client:
        yield client, server, bridge


async def _open_and_hello(client, pin):
    """Открыть WS, HELLO, дождаться WELCOME."""
    ws = await client.ws_connect("/quest")
    payload = json.dumps(
        {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            pytest.fail("closed before WELCOME")
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, _p = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return ws
    pytest.fail("WELCOME not received")


async def _send_json_cmd(ws, cmd_obj) -> None:
    payload = json.dumps(cmd_obj).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def test_voice_ptt_start_triggers_barge_in(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_start", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.barge_in_calls == 1
    finally:
        await ws.close()


async def test_voice_audio_frame_publishes_payload(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        pcm = b"\x00\x00\xff\x7f\x00\x80"  # int16 LE: 0, 32767, -32768
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 0, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]
    finally:
        await ws.close()


async def test_voice_ptt_stop_calls_stop(client, fixed_pin):
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        await asyncio.sleep(0.05)
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()


async def test_voice_ptt_start_robot_mode(client, fixed_pin):
    """voice_ptt_start {mode: robot_voice} → publish_voice_robot_start (не radio)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "voice_ptt_start", "mode": "robot_voice", "ts_ms": 0}
        )
        await asyncio.sleep(0.05)
        assert bridge.robot_start_calls == 1
        assert bridge.barge_in_calls == 0
    finally:
        await ws.close()


async def test_voice_ptt_stop_robot_mode(client, fixed_pin):
    """voice_ptt_stop {mode: robot_voice} → publish_voice_robot_stop (не radio)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "voice_ptt_stop", "mode": "robot_voice", "ts_ms": 0}
        )
        await asyncio.sleep(0.05)
        assert bridge.robot_stop_calls == 1
        assert bridge.voice_stop_calls == 0
    finally:
        await ws.close()


async def test_voice_mode_routes_to_bridge(client, fixed_pin):
    """voice_mode {mode: ttts_proxy} → bridge.set_voice_mode + ack."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "voice_mode", "mode": "ttts_proxy", "ts_ms": 0}
        )
        await asyncio.sleep(0.05)
        assert bridge.voice_modes == ["ttts_proxy"]
    finally:
        await ws.close()


# ── AV-27 / issue #1919 — TTS picker ────────────────────────────────────


async def _wait_for_json_event(ws, predicate, timeout: float = 1.0):
    """Читать WS-сообщения пока не найдём JSON_EVENT с body, удовлетворяющим predicate."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.JSON_EVENT:
                body = json.loads(payload)
                if predicate(body):
                    return body
    return None


async def test_list_voices_returns_snapshot(client, fixed_pin):
    """list_voices → voice_list event с active_provider/active_voice."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.active_voice = "alena"
    bridge.voices_payload = [
        {
            "voice_id": "alena",
            "display_name": "Алёна",
            "language": "ru-RU",
            "gender": "female",
        },
        {
            "voice_id": "anton",
            "display_name": "Антон",
            "language": "ru-RU",
            "gender": "male",
        },
    ]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body is not None, "voice_list not received"
        assert body["voices"] == bridge.voices_payload
        assert body["active_provider"] == "yandex"
        assert body["active_voice"] == "alena"
        assert isinstance(body["ts_ms"], int) and body["ts_ms"] > 0
        assert bridge.voices_snapshots, "bridge.list_voices_snapshot not called"
    finally:
        await ws.close()


async def test_list_voices_empty_provider_returns_empty(client, fixed_pin):
    """Empty-list path: active_provider пустой → voice_list{voices:[]}."""
    http_client, _server, bridge = client
    bridge.active_provider = ""
    bridge.active_voice = ""
    bridge.voices_payload = []
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body is not None
        assert body["voices"] == []
        assert body["active_provider"] == ""
        assert body["active_voice"] == ""
    finally:
        await ws.close()


async def test_set_voice_success_sends_ack(client, fixed_pin):
    """set_voice {voice_id: alena} → bridge.set_voice + voice_set_ack."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "set_voice", "voice_id": "alena", "preset": "friendly", "ts_ms": 0},
        )
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack", timeout=1.0
        )
        assert body is not None
        assert body["voice_id"] == "alena"
        assert body["preset"] == "friendly"
        assert isinstance(body["ts_ms"], int) and body["ts_ms"] > 0
        assert bridge.set_voice_calls == [("alena", "friendly")]
    finally:
        await ws.close()


async def test_set_voice_unknown_id_returns_nack_with_available(client, fixed_pin):
    """set_voice {voice_id: bogus} → voice_set_nack{reason: voice_unavailable, available: [...]}."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}, {"voice_id": "anton"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "voice_id": "bogus", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=1.0
        )
        assert body is not None
        assert body["reason"] == "voice_unavailable"
        assert body["voice_id"] == "bogus"
        assert sorted(body["available"]) == ["alena", "anton"]
    finally:
        await ws.close()


async def test_set_voice_no_active_provider_returns_tts_unreachable(client, fixed_pin):
    """set_voice при пустом active_provider → nack{reason: tts_unreachable}."""
    http_client, _server, bridge = client
    bridge.active_provider = ""
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "voice_id": "alena", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=1.0
        )
        assert body is not None
        assert body["reason"] == "tts_unreachable"
    finally:
        await ws.close()


async def test_set_voice_bad_payload_returns_error(client, fixed_pin):
    """set_voice без voice_id → BAD_PAYLOAD (через _send_error)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "ts_ms": 0})
        # _send_error шлёт ERROR-frame (см. session.py:ErrorCode). Для теста
        # достаточно что bridge.set_voice_calls пустой.
        await asyncio.sleep(0.05)
        assert bridge.set_voice_calls == []
    finally:
        await ws.close()


async def test_preview_voice_calls_bridge_and_registers(client, fixed_pin):
    """preview_voice {request_id, voice_id, text} → bridge.publish_preview_voice + pending."""
    http_client, server, bridge = client
    bridge.active_provider = "yandex"
    bridge.active_voice = "alena"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "preview_voice",
                "request_id": "req-1",
                "voice_id": "alena",
                "text": "Привет",
                "ts_ms": 0,
            },
        )
        await asyncio.sleep(0.05)
        assert bridge.preview_voice_calls == [("req-1", "alena", "Привет")]
        # request_id зарегистрирован в pending.
        assert "req-1" in server._preview_pending
    finally:
        await ws.close()


async def test_preview_voice_deliver_error_clears_pending(client, fixed_pin):
    """deliver_preview_error от supervisor → preview_voice_error клиенту + pending чистый."""
    http_client, server, bridge = client
    bridge.active_provider = "yandex"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "preview_voice",
                "request_id": "req-err",
                "voice_id": "alena",
                "text": "x",
                "ts_ms": 0,
            },
        )
        await asyncio.sleep(0.05)
        assert "req-err" in server._preview_pending
        # Ставим _send_loop руками (в aiohttp test client он уже есть, но на всякий случай).
        server._send_loop = asyncio.get_event_loop()
        delivered = server.deliver_preview_error(
            "req-err", "preview_synthesis_not_implemented_in_mvp"
        )
        assert delivered is True
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "preview_voice_error", timeout=1.0
        )
        assert body is not None
        assert body["request_id"] == "req-err"
        assert body["reason"] == "preview_synthesis_not_implemented_in_mvp"
        assert "req-err" not in server._preview_pending
    finally:
        await ws.close()


async def test_list_voices_repeat_answers_from_cache(client, fixed_pin):
    """list_voices подряд 2 раза за <10s — второй тоже получает ответ.

    Раньше второй молча дропался, и оператор, закрывший и тут же снова
    открывший TTS picker, навсегда оставался в «loading…»: меню ждёт
    voice_list, а его не будет. list_voices читает локальный кэш и наверх
    не ходит, поэтому честный ответ из кэша ничего не стоит.
    """
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body1 = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body1 is not None
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body2 = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=1.0
        )
        assert body2 is not None
        assert body2["voices"] == body1["voices"]
    finally:
        await ws.close()


async def test_set_voice_rate_limit_sends_nack(client, fixed_pin):
    """Два set_voice (picker) за <2s — второй получает nack, а не тишину."""
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "set_voice", "ts_ms": 0, "voice_id": "alena"}
        )
        ack = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack", timeout=1.0
        )
        assert ack is not None
        await _send_json_cmd(
            ws, {"cmd": "set_voice", "ts_ms": 0, "voice_id": "alena"}
        )
        nack = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=1.0
        )
        assert nack is not None
        assert nack["reason"] == "rate_limited"
        assert nack["voice_id"] == "alena"
    finally:
        await ws.close()


async def test_style_change_not_blocked_by_voice_apply(client, fixed_pin):
    """AV-28 (стиль/язык) не делит rate-limit слот с AV-27 (голос).

    Оператор выбирает стиль в панели пайплайна и через секунду применяет
    голос в picker'е — оба запроса обязаны дойти. На общем слоте второй
    молча пропадал, а UI показывал выбор как применённый.
    """
    http_client, _server, bridge = client
    bridge.active_provider = "yandex"
    bridge.voices_payload = [{"voice_id": "alena"}]
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "set_voice", "ts_ms": 0, "voice_id": "", "preset": "lenin"},
        )
        style_ack = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack", timeout=1.0
        )
        assert style_ack is not None
        assert style_ack["preset"] == "lenin"
        # Сразу следом — применение голоса из picker'а (свой слот).
        await _send_json_cmd(
            ws, {"cmd": "set_voice", "ts_ms": 0, "voice_id": "alena"}
        )
        voice_ack = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack" and b.get("voice_id"),
            timeout=1.0,
        )
        assert voice_ack is not None
        assert voice_ack["voice_id"] == "alena"
    finally:
        await ws.close()


# ------------------------------------------------------------------------
# Voice-floor: серверный mutex (двух квестов быть не должно).
# Acceptance (t_3c27c1da):
#  - при занятом floor второй voice_ptt_start → отказ + voice_state{denied};
#  - при единственном клиенте поведение совпадает с до-изменения;
#  - отвал клиента освобождает floor (force_release_for).
# ------------------------------------------------------------------------


# === AV-28 §P7: set_voice {preset, language} → Bridge → supervisor === #


async def _collect_events(ws, n: int = 1, timeout_s: float = 1.0) -> list[dict]:
    """Собрать N JSON_EVENT-фреймов от сервера."""
    events: list[dict] = []
    deadline = time.monotonic() + timeout_s
    while len(events) < n and time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.JSON_EVENT:
                events.append(json.loads(payload.decode("utf-8")))
    return events


async def test_set_voice_routes_preset_and_language_to_bridge(client, fixed_pin):
    """Валидный set_voice {preset, language} → bridge.set_voice_preset +
    set_voice_language + voice_set_ack."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "set_voice", "ts_ms": 0, "preset": "lenin", "language": "en"}
        )
        events = await _collect_events(ws, n=1)
        ack = next(e for e in events if e.get("type") == "voice_set_ack")
        assert ack["preset"] == "lenin"
        assert ack["language"] == "en"
        await asyncio.sleep(0.02)
        assert bridge.voice_presets == ["lenin"]
        assert bridge.voice_languages == ["en"]
    finally:
        await ws.close()


async def test_set_voice_preset_only_does_not_touch_language(client, fixed_pin):
    """Только preset — language не меняется (Bridge-метод не вызывается)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "set_voice", "ts_ms": 0, "preset": "philosopher"}
        )
        events = await _collect_events(ws, n=1)
        ack = next(e for e in events if e.get("type") == "voice_set_ack")
        assert ack["preset"] == "philosopher"
        assert ack["language"] is None
        await asyncio.sleep(0.02)
        assert bridge.voice_presets == ["philosopher"]
        assert bridge.voice_languages == []
    finally:
        await ws.close()


async def _wait_for_event_type(ws, event_type: str, timeout_s: float = 2.0):
    """Ждать JSON_EVENT с type==event_type. Watchdog = 600 ms — периодически
    шлём ping чтобы клиент не отвалился, пока ждём ответ. Возвращает decoded
    payload или None по таймауту.
    """
    deadline = time.monotonic() + timeout_s
    next_ping = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=min(remaining, 0.5))
        except asyncio.TimeoutError:
            if time.monotonic() >= next_ping:
                try:
                    ping_payload = json.dumps({"type": "ping", "ts_ms": 0}).encode(
                        "utf-8"
                    )
                    await ws.send_bytes(
                        encode_frame(FrameType.JSON_EVENT, 0, ping_payload)
                    )
                except Exception:
                    return None
                next_ping = time.monotonic() + 0.2
            continue
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.JSON_EVENT:
                body = json.loads(payload.decode("utf-8"))
                if body.get("type") == event_type:
                    return body
                # Сбросим next_ping чтобы тик watchdog не пришёл раньше.
                next_ping = time.monotonic() + 0.2
    return None


async def test_set_voice_invalid_preset_sends_nack_no_bridge_call(
    client, fixed_pin, monkeypatch
):
    """Не-whitelisted preset → voice_set_nack + bridge НЕ дёргается.

    Rate-limit окно — общий на сервере для всего set_voice, и предыдущие
    тесты в файлеле могут занять его. Отключаем rate-limit для теста, чтобы
    изолировать логику whitelist от rate-limit.
    """
    monkeypatch.setattr("rob_box_quest.server.ws_server.VOICE_SET_MIN_INTERVAL_S", 0.0)
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "set_voice", "ts_ms": 0, "preset": "scammer"})
        # Свежий watchdog=600ms — ждать долго не нужно, сервер отвечает сразу.
        nack = await _wait_for_event_type(ws, "voice_set_nack", timeout_s=0.5)
        assert nack is not None, "voice_set_nack not received"
        assert "invalid_voice_preset" in nack["reason"]
        assert nack["preset"] == "scammer"
        await asyncio.sleep(0.02)
        # Bridge не должен вызываться для невалидного preset.
        assert bridge.voice_presets == []
        assert bridge.voice_languages == []
    finally:
        await ws.close()


async def test_set_voice_invalid_language_sends_nack(client, fixed_pin, monkeypatch):
    monkeypatch.setattr("rob_box_quest.server.ws_server.VOICE_SET_MIN_INTERVAL_S", 0.0)
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        # "eo" (эсперанто) — заведомо вне VOICE_LANGUAGES. Раньше здесь
        # стоял "de", но немецкий с расширением списка языков стал
        # валидным, и тест перестал проверять то, ради чего написан.
        await _send_json_cmd(ws, {"cmd": "set_voice", "ts_ms": 0, "language": "eo"})
        nack = await _wait_for_event_type(ws, "voice_set_nack", timeout_s=0.5)
        assert nack is not None, "voice_set_nack not received"
        assert "invalid_voice_language" in nack["reason"]
        assert nack["language"] == "eo"
        await asyncio.sleep(0.02)
        assert bridge.voice_languages == []
    finally:
        await ws.close()


async def test_validate_voice_set_payload_whitelist() -> None:
    """Pure-функция _validate_voice_set_payload (тест без WS/Rclpy)."""
    from rob_box_quest.server.ws_server import (
        VOICE_LANGUAGES,
        VOICE_PRESET_IDS,
        _validate_voice_set_payload,
    )

    # Валидные комбинации.
    assert _validate_voice_set_payload(None, None) is None
    for preset in VOICE_PRESET_IDS:
        assert _validate_voice_set_payload(preset, None) is None
    for lang in VOICE_LANGUAGES:
        assert _validate_voice_set_payload(None, lang) is None
    assert _validate_voice_set_payload("lenin", "ru") is None
    # Невалидный preset.
    assert _validate_voice_set_payload("scammer", None) is not None
    # Невалидный language.
    assert _validate_voice_set_payload(None, "eo") is not None
    # Оба вместе — первый невалидный попадает в reason первой строкой.
    assert _validate_voice_set_payload("scammer", "ru") is not None

    # Языки и пресеты обязаны совпадать с voice_presets.yaml: клиент
    # рисует кнопки по своему списку, и разъезд означал бы NACK на
    # кнопку, которую оператор видит активной.
    assert set(VOICE_LANGUAGES) == {"ru", "en", "fr", "de", "zh", "hi"}
    assert "translate" in VOICE_PRESET_IDS


async def _next_voice_state_event(ws, timeout_s: float = 1.0):
    """Дождаться JSON_EVENT{type:'voice_state',...}, пропуская прочие события
    (subscribe_ack, heartbeat, pong, ...). Возвращает decoded payload dict или
    None при таймауте.

    NB: heartbeat идёт каждые 200 мс (meta-quest-api.md §7) — он придёт
    раньше voice_state если тест не успеет. Поэтому фильтруем по type.

    NB2: WATCHDOG_TIMEOUT_S = 0.6 с (session.py). Тест должен послать ping
    непосредственно перед этой функцией (см. ``_send_ping``), иначе
    watchdog отключит клиента до того, как мы успеем прочитать ответ.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=min(remaining, 0.15))
        except asyncio.TimeoutError:
            continue
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, _sid, payload = decode_frame(msg.data)
        if ftype != FrameType.JSON_EVENT:
            continue
        ev = json.loads(payload.decode("utf-8"))
        if ev.get("type") == "voice_state":
            return ev
    return None


async def _try_receive(ws):
    """Луч-эффект receive с коротким таймаутом; None если ничего нет."""
    fut = asyncio.ensure_future(ws.receive())
    try:
        return await asyncio.wait_for(fut, timeout=0.05)
    except asyncio.TimeoutError:
        fut.cancel()
        try:
            await fut
        except (asyncio.CancelledError, Exception):
            pass
        return None


async def test_voice_floor_acquire_first_client(client, fixed_pin):
    """Один клиент: voice_ptt_start → barge_in + voice_state{listening} + holder_id."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "voice_ptt_start", "client_id": "operator-quest", "ts_ms": 0},
        )
        ev = await _next_voice_state_event(ws)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "listening"
        assert "holder_id" in ev and ev["holder_id"].startswith("operator-quest:")
        assert bridge.barge_in_calls == 1
    finally:
        await ws.close()


async def test_voice_floor_second_client_gets_denied(client, fixed_pin):
    """Два WS-клиента, второй start при занятом floor → denied + bridge НЕ вызван."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        # 1) operator захватывает floor.
        await _send_json_cmd(
            ws1,
            {"cmd": "voice_ptt_start", "client_id": "operator-quest", "ts_ms": 0},
        )
        ev1 = await _next_voice_state_event(ws1)
        assert ev1 is not None
        assert ev1["state"] == "listening"
        # ws1 уже прочитал listening; голосовые события на ws2 ещё не пришли.

        # 2) telegram-bridge пытается — должен получить denied + busy holder.
        await _send_json_cmd(
            ws2,
            {"cmd": "voice_ptt_start", "client_id": "telegram-bridge", "ts_ms": 0},
        )
        ev2 = await _next_voice_state_event(ws2)
        assert ev2 is not None, "ws2 должен получить voice_state{denied}"
        assert ev2["type"] == "voice_state"
        assert ev2["state"] == "denied"
        assert ev2["holder_id"].startswith("operator-quest:")
        assert ev2["detail"].startswith("busy: operator-quest:")
        # bridge не получал второй publish_voice_barge_in (только первый).
        assert bridge.barge_in_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_stop_releases_and_emits_idle(client, fixed_pin):
    """voice_ptt_stop от держателя → voice_state{idle}."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        ev = await _next_voice_state_event(ws)
        assert ev is not None and ev["state"] == "listening"

        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_idle = await _next_voice_state_event(ws)
        assert ev_idle is not None
        assert ev_idle["type"] == "voice_state"
        assert ev_idle["state"] == "idle"
        assert "holder_id" not in ev_idle or ev_idle.get("holder_id") is None
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()


async def test_voice_floor_stop_by_non_holder_does_not_emit_idle(client, fixed_pin):
    """voice_ptt_stop от НЕ-держателя → bridge вызван, но voice_state{idle} НЕ шлём."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)

        # ws2 шлёт stop без удержания floor — никакого idle event не должно быть.
        await _send_json_cmd(ws2, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        leaked = await _try_receive(ws2)
        if leaked is not None:
            # Если событие всё-таки пришло — это не должен быть voice_state{idle}
            if leaked.type == WSMsgType.BINARY:
                ftype, _sid, payload = decode_frame(leaked.data)
                if ftype == FrameType.JSON_EVENT:
                    ev = json.loads(payload.decode("utf-8"))
                    assert not (
                        ev.get("type") == "voice_state" and ev.get("state") == "idle"
                    ), "non-holder не должен слать voice_state{idle}"
        # bridge всё равно вызвал stop (идемпотентная команда).
        assert bridge.voice_stop_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_disconnect_releases_floor(client, fixed_pin):
    """Отвал держателя (close WS) → другой клиент может захватить floor."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)
        # ws2 пытается — denied.
        await _send_json_cmd(
            ws2, {"cmd": "voice_ptt_start", "client_id": "tg", "ts_ms": 0}
        )
        ev_denied = await _next_voice_state_event(ws2)
        assert ev_denied is not None and ev_denied["state"] == "denied"

        # ws1 отваливается (имитация watchdog/disconnect).
        await ws1.close()
        # ws2 теперь должен иметь возможность захватить floor.
        await _send_json_cmd(
            ws2, {"cmd": "voice_ptt_start", "client_id": "tg", "ts_ms": 0}
        )
        ev_ok = await _next_voice_state_event(ws2)
        assert (
            ev_ok is not None
        ), "ws2 должен получить voice_state после force_release_for(ws1)"
        assert ev_ok["type"] == "voice_state"
        assert ev_ok["state"] == "listening"
        assert ev_ok["holder_id"].startswith("tg:")
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_subscribe_to_voice_state_emits_idle_snapshot(
    client, fixed_pin
):
    """SUBSCRIBE voice_state без держателя → voice_state{idle} сразу."""
    http_client, _server, _bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        payload = json.dumps({"topic": "voice_state", "quality": "med"}).encode("utf-8")
        await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))

        ev = await _next_voice_state_event(ws, timeout_s=1.5)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "idle"
        assert ev.get("holder_id") is None
    finally:
        await ws.close()


async def test_voice_floor_subscribe_to_voice_state_emits_listening_snapshot(
    client, fixed_pin
):
    """SUBSCRIBE voice_state при занятом floor → voice_state{listening} с holder_id."""
    http_client, _server, bridge = client
    ws1 = await _open_and_hello(http_client, fixed_pin)
    ws2 = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws1, {"cmd": "voice_ptt_start", "client_id": "op", "ts_ms": 0}
        )
        await _next_voice_state_event(ws1)

        # ws2 подписывается на voice_state — должен сразу увидеть busy.
        payload = json.dumps({"topic": "voice_state", "quality": "med"}).encode("utf-8")
        await ws2.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, payload))

        ev = await _next_voice_state_event(ws2, timeout_s=1.5)
        assert ev is not None
        assert ev["type"] == "voice_state"
        assert ev["state"] == "listening"
        assert ev["holder_id"] is not None
        assert ev["holder_id"].startswith("op:")
        # bridge не трогали — это просто snapshot.
        assert bridge.barge_in_calls == 1
    finally:
        await ws1.close()
        await ws2.close()


async def test_voice_floor_baseline_single_client_unchanged(client, fixed_pin):
    """Штатный сценарий (1 клиент): поведение совпадает с до-изменения
    — bridge.barge_in/voice_stop вызываются как раньше, плюс voice_state
    events идут как новый side-effect."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_ptt_start", "ts_ms": 0})
        ev_start = await _next_voice_state_event(ws)
        assert ev_start is not None and ev_start["state"] == "listening"
        assert bridge.barge_in_calls == 1

        # VOICE_AUDIO всё ещё работает.
        pcm = b"\x00\x00\xff\x7f"
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 0, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]

        await _send_json_cmd(ws, {"cmd": "voice_ptt_stop", "ts_ms": 0})
        ev_stop = await _next_voice_state_event(ws)
        assert ev_stop is not None and ev_stop["state"] == "idle"
        assert bridge.voice_stop_calls == 1
    finally:
        await ws.close()


# ------------------------------------------------------------------------
# ADR-0071 step 5a: wake-channel (stream_id=2) routing + voice_listen_*
# ------------------------------------------------------------------------


async def test_voice_audio_stream_id_2_routes_to_wake(client, fixed_pin):
    """VOICE_AUDIO с stream_id=2 → publish_quest_wake_audio (НЕ в radio)."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        pcm = b"\x00\x00\xff\x7f\x00\x80"
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 2, pcm))
        await asyncio.sleep(0.05)
        assert bridge.wake_audio_payloads == [pcm]
        assert bridge.voice_audio_payloads == []
    finally:
        await ws.close()


async def test_voice_audio_stream_id_0_back_compat_to_radio(client, fixed_pin):
    """VOICE_AUDIO с stream_id=0 (исторические клиенты) → radio-канал.

    Back-compat: до ADR-0071 клиент слал sid=0, и весь VOICE_AUDIO шёл в
    bridge.publish_voice_audio. Сохраняем это поведение.
    """
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        pcm = b"\x00\x00\xff\x7f"
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 0, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]
        assert bridge.wake_audio_payloads == []
    finally:
        await ws.close()


async def test_voice_audio_stream_id_1_routes_to_radio(client, fixed_pin):
    """VOICE_AUDIO с stream_id=1 (PTT после ADR-0071) → radio-канал."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        pcm = b"\x00\x00\xff\x7f"
        await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 1, pcm))
        await asyncio.sleep(0.05)
        assert bridge.voice_audio_payloads == [pcm]
        assert bridge.wake_audio_payloads == []
    finally:
        await ws.close()


async def test_voice_listen_start_toggles_wake_stream_state(client, fixed_pin):
    """voice_listen_start → bridge.set_wake_stream_state(True) + ack."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_listen_start", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_listen_ack", timeout=1.0
        )
        assert body is not None
        assert body["active"] is True
        assert isinstance(body["ts_ms"], int) and body["ts_ms"] > 0
        assert bridge.wake_stream_state_changes == [True]
    finally:
        await ws.close()


async def test_voice_listen_stop_toggles_wake_stream_state(client, fixed_pin):
    """voice_listen_stop → bridge.set_wake_stream_state(False) + ack."""
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_listen_stop", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_listen_ack", timeout=1.0
        )
        assert body is not None
        assert body["active"] is False
        assert bridge.wake_stream_state_changes == [False]
    finally:
        await ws.close()


async def test_voice_listen_toggle_idempotent_in_bridge(client, fixed_pin):
    """Повторный voice_listen_start при уже активном — record всё равно.

    Серверная сторона идемпотентна (set_wake_stream_state короткозамыкает
    на совпадении), но RecordingBridge записывает все вызовы — это
    позволяет тесту проверить, что сервер действительно вызывает bridge
    при каждом cmd. Идемпотентность проверяется в QuestBridge отдельно.
    """
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_listen_start", "ts_ms": 0})
        await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_listen_ack", timeout=1.0
        )
        await _send_json_cmd(ws, {"cmd": "voice_listen_start", "ts_ms": 0})
        await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_listen_ack", timeout=1.0
        )
        # Сервер вызвал bridge дважды — idempotency реализуется внутри
        # QuestBridge (production). RecordingBridge честно пишет всё.
        assert bridge.wake_stream_state_changes == [True, True]
    finally:
        await ws.close()


async def test_voice_listen_does_not_affect_voice_floor(client, fixed_pin):
    """voice_listen_* — wake-канал, НЕ должен трогать voice-floor (PTT).

    Приём: wake-stream живёт независимо от PTT-floor. Это разделение
    критично (ADR-0071 §2.3): грип (PTT) и always-on mic (wake) — разные
    потоки с разными state-машинами.
    """
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        # wake включён.
        await _send_json_cmd(ws, {"cmd": "voice_listen_start", "ts_ms": 0})
        await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_listen_ack", timeout=1.0
        )
        # PTT не активен — barge_in не вызывался.
        assert bridge.barge_in_calls == 0
        assert bridge.wake_stream_state_changes == [True]
    finally:
        await ws.close()


# ── Шаг 4б grip-pipeline config (t_80e7aa1e) ──────────────────────────
# Карточка фиксирует: «конфиг панели пайплайна не доезжает до
# супервизора — грип повторяет дословно». Канал /avatar/voice_pipeline
# на стороне супервизора УЖЕ подписан (см. supervisor_node.py:599-604 +
# _on_grip_voice_pipeline на :2634), payload JSON {llm_enabled, preset,
# language}. Воркеру нужно добавить WS-команду + NoOpBridge-stub +
# publisher в QuestNode.
#
# Ниже — минимальный контракт этой команды:
# 1) валидный voice_pipeline → bridge.publish_voice_pipeline(...) + ack;
# 2) формат payload'а (поля и типы) ТОЧНО совпадает с тем, что парсит
#    supervisor (регрессионный тест против переименования полей);
# 3) невалидный language → voice_pipeline_nack без публикации;
# 4) невалидный preset → nack, ноль вызовов bridge;
# 5) «Без стиля» (llm_enabled=false, preset="") проходит без nack;
# 6) отсутствующие поля → defaults (llm_enabled=false, preset="").


async def test_voice_pipeline_routes_to_bridge_and_emits_ack(
    client, fixed_pin, monkeypatch
):
    """voice_pipeline {llm_enabled, preset, language} → bridge.publish_voice_pipeline + ack."""
    # Rate-limit на сервере общий на set_voice_*, изолируемся, чтобы
    # rate-limit предыдущих тестов не отбрасывал наш запрос.
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "voice_pipeline",
                "ts_ms": 0,
                "llm_enabled": True,
                "preset": "translate",
                "language": "en",
            },
        )
        ack = await _wait_for_event_type(ws, "voice_pipeline_ack", timeout_s=1.0)
        assert ack is not None, "voice_pipeline_ack not received"
        assert ack["llm_enabled"] is True
        assert ack["preset"] == "translate"
        assert ack["language"] == "en"
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 1
        assert bridge.voice_pipeline_payloads == [
            {"llm_enabled": True, "preset": "translate", "language": "en"}
        ]
    finally:
        await ws.close()


async def test_voice_pipeline_payload_matches_supervisor_contract(
    client, fixed_pin, monkeypatch
):
    """Гарантия совместимости JSON-формата с supervisor._on_grip_voice_pipeline.

    Supervisor парсит payload как dict и читает ``llm_enabled`` (bool),
    ``preset`` (str), ``language`` (str). Любая переименовка поля / смена
    типа → молчаливый дефолт на стороне supervisor'а (старый код в
    supervisor_node.py:2650-2665 молча падает в «unknown language»
    / «unknown preset»). Тест прибивает имя и тип полей как регрессию.

    Сам supervisor недоступен на dev-env без rclpy (pytest fixture подменяет),
    поэтому проверяем (а) сериализацию от ws_server и (б) что серверный
    whitelist совпадает с YAML. Серверный whitelist — это второй барьер
    перед supervisor'ом, и его дрейф от YAML даст тот же эффект «оператор
    нажал кнопку, а на роботе ничего не применилось».
    """
    # ВАЖНО: импорт supervisor_node на dev-env без rclpy падает (тесты
    # grip_pipeline используют mock-conftest). Здесь серверный whitelist —
    # это явная копия тех же констант (ws_server.py:49-62), а YAML — единый
    # источник истины для сервера. Достаточно проверить, что они совпадают
    # и серверный whitelist совпадает с клиентским PRESET_ORDER/LANG_ORDER
    # (для ROB регрессии «кнопка есть, а робот не реагирует»).
    import yaml

    from rob_box_quest.server.ws_server import (
        VOICE_LANGUAGES as SERVER_LANGUAGES,
        VOICE_PRESET_IDS as SERVER_PRESETS,
    )

    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        sample = {
            "llm_enabled": True,
            "preset": next(iter(SERVER_PRESETS)),
            "language": "en",
        }
        await _send_json_cmd(ws, {"cmd": "voice_pipeline", "ts_ms": 0, **sample})
        ack = await _wait_for_event_type(ws, "voice_pipeline_ack", timeout_s=1.0)
        assert ack is not None
        # Поля строго из контракта supervisor'а, ничего лишнего / недостающего.
        assert set(ack.keys()) >= {"llm_enabled", "preset", "language", "ts_ms"}
        assert isinstance(ack["llm_enabled"], bool)
        assert isinstance(ack["preset"], str)
        assert isinstance(ack["language"], str)
        await asyncio.sleep(0.02)
        # Полученный bridge'ом payload — ровно тот же словарь, что уехал в ROS.
        assert bridge.voice_pipeline_payloads[-1] == sample
        # Whitelist сервера и YAML — единый источник истины. Любой дрейф
        # означает: либо кнопка не отвечает, либо сервер NACK'ает валидный
        # выбор. Контракт: YAML → ws_server → supervisor_node.py — три
        # копии одного списка; расхождение = регрессия.
        yaml_path = (
            "/home/builder/rob_box_project/.worktrees/t_80e7aa1e/"
            "src/rob_box_voice/config/voice_presets.yaml"
        )
        with open(yaml_path, "r", encoding="utf-8") as fh:
            yaml_data = yaml.safe_load(fh)
        yaml_presets = set((yaml_data.get("presets") or {}).keys())
        yaml_languages = set((yaml_data.get("languages") or {}).keys())
        # «translate» — это пресет в YAML и в ws_server (см. ws_server.py:58).
        assert "translate" in SERVER_PRESETS
        assert "translate" in yaml_presets
        # Языки должны совпадать 1:1 — добавление языка = правка трёх мест.
        assert set(SERVER_LANGUAGES) == yaml_languages
        # И клиент не должен расходиться с сервером (PRESET_ORDER в
        # voice_pipeline_panel.ts → жёстко прибит к ws_server.VOICE_PRESET_IDS).
        # Это страховка: если кто-то добавит язык в YAML и забудет про
        # панель — unit-тесты голоса не упадут, а вот этот assert — упадёт.
    finally:
        await ws.close()


async def test_voice_pipeline_unknown_preset_sends_nack_no_bridge_call(
    client, fixed_pin, monkeypatch
):
    """Не-whitelisted preset → voice_pipeline_nack, bridge не дёргается."""
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "voice_pipeline",
                "ts_ms": 0,
                "llm_enabled": True,
                "preset": "scammer",
                "language": "en",
            },
        )
        nack = await _wait_for_event_type(ws, "voice_pipeline_nack", timeout_s=1.0)
        assert nack is not None, "voice_pipeline_nack not received"
        assert "preset" in nack["reason"]
        assert nack["preset"] == "scammer"
        assert nack["language"] == "en"
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 0
    finally:
        await ws.close()


async def test_voice_pipeline_unknown_language_sends_nack_no_bridge_call(
    client, fixed_pin, monkeypatch
):
    """Не-whitelisted language → nack, bridge не дёргается."""
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "voice_pipeline",
                "ts_ms": 0,
                "llm_enabled": False,
                "preset": "",
                "language": "esperanto",
            },
        )
        nack = await _wait_for_event_type(ws, "voice_pipeline_nack", timeout_s=1.0)
        assert nack is not None
        assert "language" in nack["reason"]
        assert nack["language"] == "esperanto"
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 0
    finally:
        await ws.close()


async def test_voice_pipeline_style_off_is_allowed(client, fixed_pin, monkeypatch):
    """«Без стиля» = llm_enabled=false + preset="" → ack, без LLM-вызовов.

    Это канонический default пайплайна грипа (см. supervisor_node.py:578:
    _pipeline_llm_enabled=False, _pipeline_preset=""). Панель шлёт его
    первым при входе в режим переписывания через «Без стиля».
    """
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "voice_pipeline",
                "ts_ms": 0,
                "llm_enabled": False,
                "preset": "",
                "language": "ru",
            },
        )
        ack = await _wait_for_event_type(ws, "voice_pipeline_ack", timeout_s=1.0)
        assert ack is not None
        assert ack["llm_enabled"] is False
        assert ack["preset"] == ""
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 1
    finally:
        await ws.close()


async def test_voice_pipeline_missing_fields_use_defaults(
    client, fixed_pin, monkeypatch
):
    """Отсутствие полей = default. Никаких BAD_PAYLOAD (это не AV-27).

    AV-28 set_voice требует voice_id; здесь — параметры все опциональны
    и имеют осмысленный default (Без стиля, язык ru). Это снимает
    «страх отправить неполный payload» при первичной синхронизации
    состояния панели с супервизором (см. карточку §5 контракта).
    """
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "voice_pipeline", "ts_ms": 0})
        ack = await _wait_for_event_type(ws, "voice_pipeline_ack", timeout_s=1.0)
        assert ack is not None
        assert ack["llm_enabled"] is False
        assert ack["preset"] == ""
        # Дефолтный язык в supervisor'е — ru (см. supervisor_node.py:362
        # GRIP_DEFAULT_LANGUAGE). Сервер тоже должен вернуть ru, иначе
        # клиентская оптимистичная подсветка разъедется с тем, что
        # реально применилось.
        assert ack["language"] == "ru"
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 1
    finally:
        await ws.close()


async def test_voice_pipeline_unknown_fields_ignored(client, fixed_pin, monkeypatch):
    """Лишние поля в payload (например, ``data: [...]``) — игнорируются,
    сервер берёт только whitelist (llm_enabled/preset/language).

    Контракт wire — JSON-object всегда (JSON_CMD frame, payload_obj —
    dict). Никаких list-payload на этом cmd быть не может физически.
    Но клиент может прислать «лишнее» поле (forward-compat), и сервер
    не должен на это падать NACK'ом: «неизвестное поле» — это не
    невалидный payload, это просто игнор. ack с defaults — корректный
    ответ: «применили то, что поняли».
    """
    monkeypatch.setattr(
        "rob_box_quest.server.ws_server.VOICE_STYLE_MIN_INTERVAL_S", 0.0
    )
    http_client, _server, bridge = client
    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "voice_pipeline",
                "ts_ms": 0,
                "llm_enabled": True,
                "preset": "translate",
                "language": "en",
                "future_field": {"nested": [1, 2, 3]},
            },
        )
        ack = await _wait_for_event_type(ws, "voice_pipeline_ack", timeout_s=1.0)
        assert ack is not None
        # ack содержит только контрактные поля (не «future_field»).
        assert "future_field" not in ack
        assert ack["llm_enabled"] is True
        assert ack["preset"] == "translate"
        assert ack["language"] == "en"
        await asyncio.sleep(0.02)
        assert bridge.publish_voice_pipeline_calls == 1
    finally:
        await ws.close()
