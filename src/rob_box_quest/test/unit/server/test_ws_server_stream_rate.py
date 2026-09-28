"""Integration: SUBSCRIBE.max_hz на живом aiohttp WS (issue #3150)."""

import asyncio
import json
import time

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame
from rob_box_quest.server.ws_server import NoOpBridge, WSSServer, build_app

pytestmark = pytest.mark.asyncio


@pytest.fixture
def fixed_pin(monkeypatch):
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def client(fixed_pin):
    server = WSSServer(bridge=NoOpBridge(), pin=fixed_pin)
    async with TestClient(TestServer(build_app(server))) as http_client:
        yield http_client, server


async def _connect(http_client, pin):
    ws = await http_client.ws_connect("/quest")
    hello = {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, json.dumps(hello).encode()))
    await _wait_for(ws, lambda ft, body: ft == FrameType.WELCOME)
    return ws


async def _subscribe(ws, topic, **extra):
    payload = {"topic": topic, "quality": "med", **extra}
    await ws.send_bytes(encode_frame(FrameType.SUBSCRIBE, 0, json.dumps(payload).encode()))
    return await _wait_for(
        ws,
        lambda ft, body: ft == FrameType.JSON_EVENT
        and body.get("type") == "subscribe_ack"
        and body.get("topic") == topic,
    )


async def _wait_for(ws, pred, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive(timeout=timeout)
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, _sid, payload = decode_frame(msg.data)
        body = None
        if ftype in (FrameType.JSON_EVENT, FrameType.WELCOME):
            body = json.loads(payload.decode("utf-8"))
        if pred(ftype, body):
            return body
    pytest.fail("expected frame not received")


async def _collect_binary(ws, stream_id, duration):
    got = []
    deadline = time.monotonic() + duration
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return got
        try:
            msg = await ws.receive(timeout=left)
        except asyncio.TimeoutError:
            return got
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, sid, payload = decode_frame(msg.data)
        if ftype == FrameType.BINARY_FRAME and sid == stream_id:
            got.append(payload)


async def test_ack_echoes_effective_max_hz(client, fixed_pin):
    http_client, _server = client
    ws = await _connect(http_client, fixed_pin)
    try:
        ack = await _subscribe(ws, "lidar_2d", max_hz=5)
        assert ack["max_hz"] == 5.0
        # re-SUBSCRIBE без max_hz — лимит снят, stream_id тот же
        ack2 = await _subscribe(ws, "lidar_2d")
        assert "max_hz" not in ack2
        assert ack2["stream_id"] == ack["stream_id"]
        # мусор → без лимита
        ack3 = await _subscribe(ws, "lidar_2d", max_hz=-3)
        assert "max_hz" not in ack3
    finally:
        await ws.close()


async def test_max_hz_throttles_and_keeps_last_frame(client, fixed_pin):
    http_client, server = client
    ws = await _connect(http_client, fixed_pin)
    try:
        ack = await _subscribe(ws, "lidar_2d", max_hz=2)
        for i in range(5):
            server.broadcast_frame("lidar_2d", b"f%d" % i)
        got = await _collect_binary(ws, ack["stream_id"], duration=0.8)
        # первый — сразу, последний — через flush (≈0.5 с); середина отброшена
        assert got == [b"f0", b"f4"]
    finally:
        await ws.close()


async def test_resubscribe_without_limit_sends_every_frame(client, fixed_pin):
    http_client, server = client
    ws = await _connect(http_client, fixed_pin)
    try:
        await _subscribe(ws, "lidar_2d", max_hz=1)
        ack = await _subscribe(ws, "lidar_2d")
        for i in range(4):
            server.broadcast_frame("lidar_2d", b"f%d" % i)
        got = await _collect_binary(ws, ack["stream_id"], duration=0.3)
        assert got == [b"f0", b"f1", b"f2", b"f3"]
    finally:
        await ws.close()


async def test_unsubscribe_drops_pending_frame(client, fixed_pin):
    http_client, server = client
    ws = await _connect(http_client, fixed_pin)
    try:
        ack = await _subscribe(ws, "lidar_2d", max_hz=2)
        server.broadcast_frame("lidar_2d", b"a")
        server.broadcast_frame("lidar_2d", b"b")  # в слот ожидания
        payload = json.dumps({"topic": "lidar_2d"}).encode()
        await ws.send_bytes(encode_frame(FrameType.UNSUBSCRIBE, 0, payload))
        got = await _collect_binary(ws, ack["stream_id"], duration=0.8)
        assert b"b" not in got
    finally:
        await ws.close()
