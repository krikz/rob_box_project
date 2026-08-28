"""Local fake quest WSS server (dev-only self-test).

Зачем: чтобы можно было прогнать `./quest_smoke.sh --all` локально и
получить green-report без поднятого Vision Pi + Phase 1.6. Использует
ТОЛЬКО уже установленный `websockets` lib — никаких aiohttp/multidict/yarl.

Запуск (в отдельном терминале):
    python3 local_test/_dev_fake_quest.py
Потом в другом:
    ./local_test/quest_smoke.sh --all --host 127.0.0.1 --port 18765 --pin 123456

Не часть прод-контракта — оставлен в репо как dev-helper, исключён
через underscore-prefix в имени (не импортируется из других модулей).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import ssl
import struct
import subprocess
import sys
import tempfile
import time


def encode(ftype: int, sid: int, payload: bytes) -> bytes:
    out = struct.pack("<BI", ftype, sid)
    n = len(payload)
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out += bytes([b | 0x80])
        else:
            out += bytes([b])
            break
    return out + payload


def decode(raw: bytes) -> tuple[int, int, bytes]:
    ftype, sid = struct.unpack_from("<BI", raw)
    off = 5
    n = 0
    shift = 0
    while True:
        b = raw[off]
        off += 1
        n |= (b & 0x7F) << shift
        if not (b & 0x80):
            break
        shift += 7
    return ftype, sid, raw[off : off + n]


HELLO = 0x01
WELCOME = 0x02
SUBSCRIBE = 0x03
BINARY_FRAME = 0x10
JSON_EVENT = 0x12
GOODBYE = 0x20
ERROR = 0xFF


async def handle(ws, pin: str) -> None:
    """Per-connection handler."""
    authenticated = False
    subscribed: str | None = None
    push_task: asyncio.Task | None = None

    def _closed() -> bool:
        return ws.state.name == "CLOSED"

    async def heartbeat():
        n = 0
        while not _closed():
            await asyncio.sleep(0.2)
            if _closed():
                return
            try:
                await ws.send(encode(JSON_EVENT, 0, json.dumps(
                    {"type": "heartbeat", "ts_ms": int(time.time() * 1000), "n": n}
                ).encode()))
                n += 1
            except Exception:
                return

    async def push_jpeg():
        if subscribed != "camera_oak_color":
            return
        # 10 fps ~50 KB JPEG payload.
        while not _closed() and subscribed == "camera_oak_color":
            await asyncio.sleep(0.1)
            if _closed():
                return
            payload = b"\xff\xd8\xff\xe0" + b"\x00" * 50000 + b"\xff\xd9"
            try:
                await ws.send(encode(BINARY_FRAME, 0x1003, payload))
            except Exception:
                return

    hb_task = asyncio.create_task(heartbeat())
    # Yield to the loop so hb_task gets scheduled.
    await asyncio.sleep(0)

    try:
        async for raw in ws:
            if isinstance(raw, str):
                continue
            ftype, _sid, payload = decode(raw)
            if ftype == HELLO:
                body = json.loads(payload.decode("utf-8"))
                if body.get("session_pin") != pin:
                    await ws.send(encode(ERROR, 0, json.dumps(
                        {"code": "AUTH_FAIL", "message": "wrong PIN"}
                    ).encode()))
                    await ws.close(code=4001)
                    return
                authenticated = True
                await ws.send(encode(WELCOME, 0, json.dumps({
                    "server_version": "0.1.0",
                    "session_id": "fake-sess-001",
                    "server_time_ms": int(time.time() * 1000),
                }).encode()))
            elif not authenticated:
                continue
            elif ftype == SUBSCRIBE:
                body = json.loads(payload.decode("utf-8"))
                subscribed = body.get("topic")
                await ws.send(encode(JSON_EVENT, 0, json.dumps({
                    "type": "subscribe_ack",
                    "topic": subscribed,
                    "stream_id": 0x1003,
                    "quality": "med",
                    "kind": "camera_direct",
                }).encode()))
                if push_task is None or push_task.done():
                    push_task = asyncio.create_task(push_jpeg())
            elif ftype == JSON_EVENT:
                body = json.loads(payload.decode("utf-8"))
                # Просто сбрасываем watchdog — нам всё равно.
                _ = body
            elif ftype == GOODBYE:
                await ws.close(code=1000)
                return
    finally:
        for t in (hb_task, push_task):
            if t is not None:
                t.cancel()


async def main(args: argparse.Namespace) -> None:
    import websockets
    from websockets.datastructures import Headers
    from websockets.http11 import Response

    # Generate self-signed cert if needed.
    if args.cert is None:
        tmpdir = tempfile.mkdtemp(prefix="quest-fake-")
        cert = os.path.join(tmpdir, "cert.pem")
        key = os.path.join(tmpdir, "key.pem")
        subprocess.check_call([
            "openssl", "req", "-x509", "-newkey", "rsa:2048", "-days", "1",
            "-nodes", "-subj", "/CN=127.0.0.1",
            "-addext", f"subjectAltName=DNS:{args.host},IP:{args.host}",
            "-keyout", key, "-out", cert,
        ], stderr=subprocess.DEVNULL)
        args.cert = cert
        args.key = key
        print(f"(auto-generated self-signed TLS: {cert})", file=sys.stderr)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(args.cert, args.key)

    async def process(ws):
        await handle(ws, args.pin)

    # websockets-сервер не умеет HTTP non-upgrade запросы — это нужно для /healthz.
    # process_request возвращает Response (НЕ пишет в conn напрямую).
    def healthz_request(conn, request):
        path = request.path.decode() if isinstance(request.path, bytes) else request.path
        if path != "/healthz":
            return None  # let websockets do WS handshake
        body = b'{"status":"ok","sessions_active":1,"server_version":"0.1.0"}'
        return Response(
            status_code=200,
            reason_phrase="OK",
            headers=Headers([
                ("Content-Type", "application/json"),
                ("Content-Length", str(len(body))),
                ("Connection", "close"),
            ]),
            body=body,
        )

    print(
        f"fake quest listening on wss://{args.host}:{args.port} (pin={args.pin})",
        file=sys.stderr,
    )
    async with websockets.serve(
        process,
        args.host,
        args.port,
        ssl=ctx,
        process_request=healthz_request,
    ):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=18765)
    ap.add_argument("--pin", default="123456")
    ap.add_argument("--cert", default=None)
    ap.add_argument("--key", default=None)
    args = ap.parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.exit(0)
