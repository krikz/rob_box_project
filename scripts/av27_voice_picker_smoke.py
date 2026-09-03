#!/usr/bin/env python3
"""AV-27 / issue #1919 — WS smoke-test для list_voices / set_voice / preview_voice.

Использование:
    python3 scripts/av27_voice_picker_smoke.py [HOST] [PORT] [PIN]

По умолчанию HOST=127.0.0.1, PORT=8765, PIN=000000.

Полный wire-contract через encode_frame/decode_frame из rob_box_quest.protocol.frame.
Скрипт НЕ требует rclpy — только websockets + asyncio. Запускается против
любого поднятого WS-сервера (Docker, dev-env с запущенным quest_node, или
локально aiohttp-ws-server из test_ws_server_voice).

В CI не запускается (developer-tool). Запускается вручную против stage-сервера
после `ros2 launch rob_box_quest quest_node.launch.py` или
`pytest src/rob_box_quest/test/unit/server/test_ws_server_voice.py` для in-proc.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import websockets
from websockets.typing import Data

# Поиск rob_box_quest по sys.path или по соседнему src/.
_HERE = Path(__file__).resolve().parent
_SRC = _HERE.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from rob_box_quest.protocol.frame import FrameType, decode_frame, encode_frame  # noqa: E402


HELLO = {
    "client_version": "0.1.0",
    "capabilities": ["webxr", "voice_picker_smoke"],
    "session_pin": "PLACEHOLDER_PIN",
}


async def _send_json(ws, obj: dict[str, Any]) -> None:
    payload = json.dumps(obj).encode("utf-8")
    await ws.send(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _recv_event(ws, event_type: str, timeout: float = 2.0) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.recv()
        ftype, _sid, payload = decode_frame(msg)
        if ftype == FrameType.JSON_EVENT:
            body = json.loads(payload)
            if body.get("type") == event_type:
                return body
        elif ftype == FrameType.WELCOME:
            print(f"[smoke] WELCOME: session_id={payload[:8]}...")
    return None


async def smoke_test(host: str, port: int, pin: str) -> int:
    url = f"ws://{host}:{port}/quest"
    print(f"[smoke] connecting to {url} ...")
    hello = dict(HELLO)
    hello["session_pin"] = pin
    async with websockets.connect(url, max_size=2**20) as ws:
        # 1) HELLO
        await ws.send(encode_frame(FrameType.HELLO, 0, json.dumps(hello).encode("utf-8")))
        welcome_or_event = await _recv_event(ws, "welcome", timeout=2.0)
        # welcome event type отсутствует в messages.ts; ищем WELCOME frame.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = await ws.recv()
            ftype, _sid, payload = decode_frame(msg)
            if ftype == FrameType.WELCOME:
                print(f"[smoke] WELCOME: {payload[:40]}...")
                break
            if ftype == FrameType.ERROR:
                print(f"[smoke] ERROR: {payload}"); return 2

        # 2) list_voices
        print("[smoke] cmd: list_voices")
        await _send_json(ws, {"cmd": "list_voices", "ts_ms": int(time.time() * 1000)})
        body = await _recv_event(ws, "voice_list", timeout=2.0)
        if body is None:
            print("[smoke] FAIL: voice_list not received")
            return 3
        print(f"[smoke] voice_list: {len(body.get('voices', []))} voices, provider={body.get('active_provider', '')!r}")

        # 3) set_voice (known id)
        voices = body.get("voices", [])
        if voices:
            target = voices[0]["voice_id"]
            print(f"[smoke] cmd: set_voice {target!r}")
            await _send_json(ws, {"cmd": "set_voice", "voice_id": target, "ts_ms": int(time.time() * 1000)})
            ack = await _recv_event(ws, "voice_set_ack", timeout=2.0)
            if ack is None:
                print("[smoke] FAIL: voice_set_ack not received")
                return 4
            print(f"[smoke] voice_set_ack: voice_id={ack['voice_id']!r}")
        else:
            print("[smoke] SKIP: set_voice — empty voices list")

        # 4) set_voice (unknown id → nack)
        print("[smoke] cmd: set_voice bogus")
        await _send_json(ws, {"cmd": "set_voice", "voice_id": "definitely-bogus-xyz", "ts_ms": int(time.time() * 1000)})
        nack = await _recv_event(ws, "voice_set_nack", timeout=2.0)
        if nack is None:
            print("[smoke] FAIL: voice_set_nack not received")
            return 5
        print(f"[smoke] voice_set_nack: reason={nack.get('reason', '?')!r}")

        # 5) preview_voice
        rid = f"smoke-{int(time.time() * 1000)}"
        print(f"[smoke] cmd: preview_voice request_id={rid!r}")
        await _send_json(ws, {
            "cmd": "preview_voice",
            "voice_id": voices[0]["voice_id"] if voices else "alena",
            "text": "Привет!",
            "request_id": rid,
            "ts_ms": int(time.time() * 1000),
        })
        preview_err = await _recv_event(ws, "preview_voice_error", timeout=3.0)
        if preview_err is None:
            preview_done = await _recv_event(ws, "preview_voice_done", timeout=1.0)
            if preview_done is not None:
                print(f"[smoke] preview_voice_done (MVP synthesis работает)")
            else:
                print("[smoke] FAIL: ни done, ни error не получены")
                return 6
        else:
            print(f"[smoke] preview_voice_error: reason={preview_err.get('reason', '?')!r} (MVP: expected)")

        print("[smoke] OK: все команды прошли")
        return 0


def main(argv: list[str]) -> int:
    host = argv[1] if len(argv) > 1 else "127.0.0.1"
    port = int(argv[2]) if len(argv) > 2 else 8765
    pin = argv[3] if len(argv) > 3 else "000000"
    try:
        return asyncio.run(smoke_test(host, port, pin))
    except Exception as e:
        print(f"[smoke] EXCEPTION: {type(e).__name__}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))