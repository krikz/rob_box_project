"""Integration-тест: VOICE_AUDIO(stream_id=2) через реальный WSSServer доезжает
до реального QuestBridge.publish_quest_wake_audio → mock ROS-publisher'а
/audio/quest_wake.

Зачем отдельный файл, а не только test_ws_server_voice.py (там —
RecordingBridge/NoOpBridge, ROS не участвует вовсе). Issue #1992: карточку уже
один раз закрывали ошибочно, потому что `git grep -l publish_quest_wake_audio`
находил метод, а сам он был `return None` — код существовал, но не работал.
Этот тест гоняет весь путь WS-frame → QuestBridge → publisher.publish(),
которого не было в предыдущей проверке.

Требует rclpy/geometry_msgs/audio_common_msgs (Docker image) — на dev-env без
ROS пропускается через importorskip, как остальные тесты QuestBridge
(см. test/unit/test_quest_bridge.py).
"""

import asyncio
import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import WSSServer, build_app

pytestmark = pytest.mark.asyncio


class _MockPublisher:
    """Имитация rclpy.Publisher.publish(msg) — собирает все сообщения."""

    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _MockNode:
    def get_logger(self):
        return self

    def warning(self, msg: str) -> None:  # pragma: no cover - diagnostics only
        pass


def _make_real_bridge_with_mock_wake_pub():
    """QuestBridge реальный (не NoOpBridge/RecordingBridge), quest_wake_pub —
    mock rclpy Publisher. Остальные publishers — None (не нужны для этого пути)."""
    pytest.importorskip(
        "geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)"
    )
    from rob_box_quest.quest_node import QuestBridge

    quest_wake_pub = _MockPublisher()
    bridge = QuestBridge(
        node=_MockNode(),
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=quest_wake_pub,
    )
    return bridge, quest_wake_pub


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "654321"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


async def _open_and_hello(client, pin):
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


async def test_stream_id_2_reaches_quest_wake_ros_publisher(fixed_pin):
    """Полный путь: WS VOICE_AUDIO(sid=2) → ws_server routing → QuestBridge
    (реальный, не тестовый double) → mock ROS Publisher на /audio/quest_wake,
    с тем же payload, что пришёл по сети."""
    bridge, quest_wake_pub = _make_real_bridge_with_mock_wake_pub()
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as http_client:
        ws = await _open_and_hello(http_client, fixed_pin)
        try:
            pcm = b"\x00\x00\xff\x7f\x00\x80\x01\x00"
            await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 2, pcm))
            await asyncio.sleep(0.05)
            assert len(quest_wake_pub.published) == 1
            assert bytes(quest_wake_pub.published[0].data) == pcm
        finally:
            await ws.close()


async def test_stream_id_1_does_not_reach_quest_wake_publisher(fixed_pin):
    """PTT (sid=1) не должен попадать на wake-publisher (routing по sid)."""
    bridge, quest_wake_pub = _make_real_bridge_with_mock_wake_pub()
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    app = build_app(server)
    async with TestClient(TestServer(app)) as http_client:
        ws = await _open_and_hello(http_client, fixed_pin)
        try:
            pcm = b"\x00\x00\xff\x7f"
            await ws.send_bytes(encode_frame(FrameType.VOICE_AUDIO, 1, pcm))
            await asyncio.sleep(0.05)
            assert len(quest_wake_pub.published) == 0
        finally:
            await ws.close()
