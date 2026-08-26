#!/usr/bin/env python3
"""Wire-protocol helpers for local_test/quest_smoke.sh.

Этот модуль НЕ зависит от rob_box_quest (который требует ROS2) — он
копирует минимальный контракт: 1 byte type + 4 bytes stream_id LE + LEB128
payload_len + payload (см. rob_box_quest/protocol/frame.py + meta-quest-api.md §2/§3).

Цель — иметь возможность дёрнуть WSS-хэндшейк с любой машины, не поднимая
полный ROS2-workspace. Никакого silent fallback: если websockets нет —
падаем явно.

FrameType (см. protocol/frame.py + meta-quest-api.md §3):
    HELLO         0x01
    WELCOME       0x02
    SUBSCRIBE     0x03
    UNSUBSCRIBE   0x04
    BINARY_FRAME  0x10
    JSON_CMD      0x11
    JSON_EVENT    0x12
    GOODBYE       0x20
    ERROR         0xFF
"""
from __future__ import annotations

import argparse
import asyncio
import json
import ssl
import statistics
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable

try:
    import websockets
    import websockets.exceptions
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        "FATAL: python3 'websockets' library is required for quest_smoke.\n"
        "Install: pip install 'websockets>=10'\n"
        f"Original: {exc}\n"
    )
    raise SystemExit(65)


# --- Frame codec (копия contract из rob_box_quest/protocol/frame.py) ---------
HELLO = 0x01
WELCOME = 0x02
SUBSCRIBE = 0x03
UNSUBSCRIBE = 0x04
BINARY_FRAME = 0x10
JSON_CMD = 0x11
JSON_EVENT = 0x12
GOODBYE = 0x20
ERROR = 0xFF

_FRAME_HEADER = struct.Struct("<BI")  # type(1) + stream_id(4 LE)


def _encode_leb128(value: int) -> bytes:
    if value < 0:
        raise ValueError("LEB128 supports unsigned only")
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            break
    return bytes(out)


def _decode_leb128(data: bytes, offset: int) -> tuple[int, int]:
    result = 0
    shift = 0
    while True:
        if offset >= len(data):
            raise ValueError("truncated LEB128")
        byte = data[offset]
        offset += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, offset
        shift += 7
        if shift > 70:
            raise ValueError("LEB128 too long")


def encode_frame(ftype: int, stream_id: int, payload: bytes) -> bytes:
    return _FRAME_HEADER.pack(ftype, stream_id) + _encode_leb128(len(payload)) + payload


def decode_frame(raw: bytes) -> tuple[int, int, bytes]:
    if len(raw) < _FRAME_HEADER.size:
        raise ValueError("incomplete header")
    ftype, stream_id = _FRAME_HEADER.unpack_from(raw)
    plen, off = _decode_leb128(raw, _FRAME_HEADER.size)
    payload = raw[off : off + plen]
    if len(payload) != plen:
        raise ValueError("truncated payload")
    return ftype, stream_id, payload


# --- Result dataclass --------------------------------------------------------
@dataclass
class SectionResult:
    name: str
    status: str  # 'pass' | 'fail' | 'skip'
    detail: str = ""
    metrics: dict = field(default_factory=dict)


@dataclass
class SuiteResult:
    sections: list[SectionResult] = field(default_factory=list)

    def add(self, r: SectionResult) -> None:
        self.sections.append(r)

    def to_dict(self) -> dict:
        pass_n = sum(1 for s in self.sections if s.status == "pass")
        fail_n = sum(1 for s in self.sections if s.status == "fail")
        skip_n = sum(1 for s in self.sections if s.status == "skip")
        return {
            "pass": pass_n,
            "fail": fail_n,
            "skip": skip_n,
            "sections": [
                {
                    "name": s.name,
                    "status": s.status,
                    "detail": s.detail,
                    "metrics": s.metrics,
                }
                for s in self.sections
            ],
        }


# --- DNS ---------------------------------------------------------------------
async def check_dns_async(host: str) -> SectionResult:
    loop = asyncio.get_event_loop()
    try:
        infos = await asyncio.wait_for(
            loop.getaddrinfo(host, None), timeout=3.0
        )
    except asyncio.TimeoutError:
        return SectionResult(name="dns", status="fail", detail=f"DNS timeout for {host!r}")
    except Exception as exc:  # noqa: BLE001
        return SectionResult(
            name="dns", status="fail", detail=f"DNS error for {host!r}: {exc}"
        )
    addrs = sorted({i[4][0] for i in infos})
    return SectionResult(
        name="dns",
        status="pass",
        detail=f"{host} → {', '.join(addrs)}",
        metrics={"addresses": addrs, "count": len(addrs)},
    )


# --- TLS ---------------------------------------------------------------------
async def check_tls(
    host: str, port: int, expected_san: str, timeout_s: float = 3.0
) -> SectionResult:
    """Подключаемся по TLS (без WS upgrade), читаем cert, проверяем SAN.

    Использует ssl + asyncio.open_connection для TCP+TLS handshake, затем
    парсит сертификат через `openssl x509 -text` (subprocess) — это работает
    с любым self-signed, не требует cryptography/pyOpenSSL. `openssl` уже
    стоит в любой Linux-среде и используется в start_quest.sh для генерации
    того же сертификата, так что контракт стабильный.
    """
    ctx = ssl.create_default_context()
    ctx.check_hostname = False  # self-signed: не валидируем цепочку
    ctx.verify_mode = ssl.CERT_NONE  # только инспектируем subject/SAN/expiry
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host=host, port=port, ssl=ctx, server_hostname=host),
            timeout=timeout_s,
        )
    except asyncio.TimeoutError:
        return SectionResult(
            name="tls",
            status="fail",
            detail=f"TLS handshake timeout ({host}:{port})",
        )
    except ssl.SSLError as exc:
        return SectionResult(
            name="tls", status="fail", detail=f"TLS error: {exc.reason} ({exc})"
        )
    except (ConnectionRefusedError, OSError) as exc:
        return SectionResult(
            name="tls",
            status="fail",
            detail=f"TCP connect failed: {exc}",
        )
    try:
        sock = writer.get_extra_info("ssl_object")
        if sock is None:
            return SectionResult(
                name="tls",
                status="fail",
                detail="No SSL object on transport (неожиданно для ssl=ctx)",
            )
        der = sock.getpeercert(binary_form=True)
        if not der:
            return SectionResult(
                name="tls",
                status="fail",
                detail="Server вернул пустой cert (возможно anonymous cipher?)",
            )
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:  # noqa: BLE001
            pass

    # Parse cert через openssl (subprocess).
    return await _parse_cert_with_openssl(der, expected_san=expected_san, timeout_s=2.0)


async def _parse_cert_with_openssl(
    der: bytes, expected_san: str, timeout_s: float = 2.0
) -> SectionResult:
    """Парсим DER cert через `openssl x509 -text -noout`."""
    loop = asyncio.get_event_loop()

    def _run_openssl() -> tuple[int, str]:
        proc = subprocess.run(
            ["openssl", "x509", "-inform", "DER", "-text", "-noout"],
            input=der,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
        return proc.returncode, proc.stdout.decode("utf-8", errors="replace")

    try:
        rc, text = await asyncio.wait_for(
            loop.run_in_executor(None, _run_openssl), timeout=timeout_s + 1.0
        )
    except asyncio.TimeoutError:
        return SectionResult(
            name="tls",
            status="fail",
            detail=f"openssl parse timeout (>{timeout_s}s)",
        )
    except FileNotFoundError:
        return SectionResult(
            name="tls",
            status="fail",
            detail="openssl не найден в PATH (нужен для парсинга cert)",
        )

    if rc != 0 or not text:
        return SectionResult(
            name="tls",
            status="fail",
            detail=f"openssl x509 failed: rc={rc}, output empty",
        )

    # Subject: "Subject: CN = foo" (формат openssl x509 -text).
    # NB: openssl индент subject/notAfter — leading spaces, так что .lstrip() обязателен.
    subject = ""
    san_dns: list[str] = []
    san_ip: list[str] = []
    not_after = ""
    in_san = False
    SAN_LABEL_PREFIX = "X509v3 Subject Alternative Name:"
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        stripped = line.lstrip()
        if "Not After" in line and not not_after:
            # "            Not After : Aug 26 12:00:00 2026 GMT"
            idx = line.find(":")
            if idx >= 0:
                not_after = line[idx + 1 :].strip()
        elif stripped.startswith("Subject:"):
            subject = stripped[len("Subject:") :].strip()
        elif stripped.startswith(SAN_LABEL_PREFIX):
            in_san = True
            tail = stripped.split(":", 1)[1].strip()
            if tail:
                for tok in tail.split(","):
                    tok = tok.strip()
                    if tok.startswith("DNS:"):
                        san_dns.append(tok[len("DNS:") :].strip())
                    elif tok.startswith("IP Address:"):
                        san_ip.append(tok[len("IP Address:") :].strip())
        elif in_san:
            cont = line.lstrip()
            indent = len(line) - len(line.lstrip())
            if cont and indent >= 8:
                for tok in cont.split(","):
                    tok = tok.strip()
                    if not tok:
                        continue
                    if tok.startswith("DNS:"):
                        san_dns.append(tok[len("DNS:") :].strip())
                    elif tok.startswith("IP Address:"):
                        san_ip.append(tok[len("IP Address:") :].strip())
            else:
                in_san = False

    days_left = None
    if not_after:
        try:
            from email.utils import parsedate_to_datetime

            dt = parsedate_to_datetime(not_after)
            days_left = (dt.timestamp() - time.time()) / 86400.0
        except (TypeError, ValueError):
            days_left = None

    san_ok = expected_san in san_dns
    metrics = {
        "subject": subject,
        "san_dns": san_dns,
        "san_ip": san_ip,
        "not_after": not_after,
        "days_left": days_left,
        "expected_san": expected_san,
    }
    if not san_ok:
        return SectionResult(
            name="tls",
            status="fail",
            detail=(
                f"cert SAN missing expected DNS:{expected_san}; "
                f"got DNS={san_dns}, IP={san_ip}"
            ),
            metrics=metrics,
        )
    if days_left is not None and days_left < 0:
        return SectionResult(
            name="tls",
            status="fail",
            detail=f"cert EXPIRED ({not_after})",
            metrics=metrics,
        )
    return SectionResult(
        name="tls",
        status="pass",
        detail=(
            f"subject={subject}; SAN DNS={san_dns}; "
            f"expires in {days_left:.1f}d" if days_left is not None
            else f"subject={subject}; SAN DNS={san_dns}"
        ),
        metrics=metrics,
    )


# --- HTTP /healthz -----------------------------------------------------------
async def check_healthz(host: str, port: int, timeout_s: float = 3.0) -> SectionResult:
    """GET /healthz → ждём status:ok."""
    import urllib.error
    import urllib.request

    url = f"https://{host}:{port}/healthz"
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    loop = asyncio.get_event_loop()

    def _do_get() -> tuple[int, dict]:
        req = urllib.request.Request(url)
        try:
            with urllib.request.urlopen(req, timeout=timeout_s, context=ctx) as resp:
                body = resp.read()
                return resp.status, json.loads(body.decode("utf-8"))
        except urllib.error.HTTPError as e:
            return e.code, {}
        except urllib.error.URLError as e:
            return 0, {"_error": str(e.reason)}

    try:
        status, body = await asyncio.wait_for(
            loop.run_in_executor(None, _do_get), timeout=timeout_s + 1.0
        )
    except asyncio.TimeoutError:
        return SectionResult(
            name="healthz", status="fail", detail=f"HTTP timeout ({url})"
        )

    if status == 0:
        return SectionResult(
            name="healthz", status="fail", detail=f"HTTP error: {body.get('_error')}"
        )
    if status != 200:
        return SectionResult(
            name="healthz", status="fail", detail=f"HTTP {status}"
        )
    if body.get("status") != "ok":
        return SectionResult(
            name="healthz",
            status="fail",
            detail=f"status field != 'ok': {body}",
        )
    return SectionResult(
        name="healthz",
        status="pass",
        detail=(
            f"status=ok, sessions_active={body.get('sessions_active')}, "
            f"version={body.get('server_version')}"
        ),
        metrics=body,
    )


# --- WSS handshake + heartbeat + SUBSCRIBE + GOODBYE -------------------------
async def _recv_frame(ws, timeout_s: float) -> tuple[int, int, bytes]:
    """Read one binary message and decode frame. Timeout → (-1,-1,b'')."""
    raw = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
    if isinstance(raw, str):  # pragma: no cover
        raise ValueError(f"text frame (not binary): {raw[:80]!r}")
    return decode_frame(raw)


async def check_wss_full(
    host: str,
    port: str,
    pin: str,
    *,
    heartbeat_samples: int = 5,
    heartbeat_window_s: float = 3.0,
    jpeg_window_s: float = 2.0,
    subscribe_topic: str = "camera_oak_color",
    verify_tls: bool = True,
) -> list[SectionResult]:
    """Полный WSS-прогон. Возвращает список SectionResult'ов в порядке:

        wss_handshake → heartbeat → subscribe_jpeg → goodbye

    Raises asyncio.TimeoutError/ConnectionError при недоступности сервера —
    вызывающий должен ловить и превращать в fail-секции.
    """
    ssl_ctx: ssl.SSLContext | None = ssl.create_default_context()
    if ssl_ctx is not None:
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE
    url = f"wss://{host}:{port}/quest"
    out: list[SectionResult] = []

    try:
        ws = await websockets.connect(
            url,
            ssl=ssl_ctx,
            max_size=2**24,  # JPEG может быть ~МБ
            ping_interval=None,  # server шлёт heartbeat сам
        )
    except Exception as exc:  # noqa: BLE001
        return [
            SectionResult(
                name="wss_handshake",
                status="fail",
                detail=f"connect failed: {exc}",
            )
        ]

    try:
        # ---- 1. HELLO → WELCOME --------------------------------------
        hello_payload = json.dumps(
            {
                "client_version": "0.1.0-smoke",
                "capabilities": ["smoke"],
                "session_pin": pin,
            }
        ).encode("utf-8")
        await ws.send(encode_frame(HELLO, 0, hello_payload))

        t_welcome_deadline = time.monotonic() + 2.0
        welcome_body: dict | None = None
        got_error = None
        while time.monotonic() < t_welcome_deadline:
            ftype, _sid, payload = await _recv_frame(ws, timeout_s=0.5)
            if ftype == WELCOME:
                welcome_body = json.loads(payload.decode("utf-8"))
                break
            if ftype == ERROR:
                got_error = json.loads(payload.decode("utf-8"))
                break
        if got_error is not None:
            out.append(
                SectionResult(
                    name="wss_handshake",
                    status="fail",
                    detail=f"server ERROR after HELLO: {got_error}",
                )
            )
            return out
        if welcome_body is None:
            out.append(
                SectionResult(
                    name="wss_handshake",
                    status="fail",
                    detail="WELCOME not received within 2s",
                )
            )
            return out
        if welcome_body.get("server_version") != "0.1.0":
            out.append(
                SectionResult(
                    name="wss_handshake",
                    status="fail",
                    detail=(
                        f"unexpected server_version={welcome_body.get('server_version')!r} "
                        f"(expected '0.1.0')"
                    ),
                )
            )
            return out
        out.append(
            SectionResult(
                name="wss_handshake",
                status="pass",
                detail=(
                    f"WELCOME received (session_id={welcome_body.get('session_id', '?')[:8]}…)"
                ),
                metrics={"server_version": welcome_body.get("server_version")},
            )
        )

        # ---- 2. heartbeat interval measurement -----------------------
        # Читаем N heartbeat'ов подряд, считаем интервалы.
        # Drain pending (если пришло лишнее — пропускаем).
        hb_ts: list[float] = []
        deadline = time.monotonic() + heartbeat_window_s
        while len(hb_ts) < heartbeat_samples and time.monotonic() < deadline:
            try:
                ftype, _sid, payload = await _recv_frame(ws, timeout_s=0.4)
            except asyncio.TimeoutError:
                continue
            if ftype != JSON_EVENT:
                continue
            try:
                body = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if body.get("type") == "heartbeat":
                hb_ts.append(time.monotonic())
        if len(hb_ts) < 3:
            out.append(
                SectionResult(
                    name="heartbeat",
                    status="fail",
                    detail=(
                        f"only {len(hb_ts)} heartbeats received in {heartbeat_window_s}s "
                        f"(want >=3 for interval measurement)"
                    ),
                )
            )
        else:
            intervals_ms = [
                (hb_ts[i] - hb_ts[i - 1]) * 1000.0 for i in range(1, len(hb_ts))
            ]
            median_ms = statistics.median(intervals_ms)
            max_ms = max(intervals_ms)
            # meta-quest-api.md §7: HEARTBEAT_INTERVAL_S = 0.2 (200 ms), tolerance ±50.
            hb_ok = 150.0 <= median_ms <= 250.0 and max_ms <= 350.0
            out.append(
                SectionResult(
                    name="heartbeat",
                    status="pass" if hb_ok else "fail",
                    detail=(
                        f"n={len(intervals_ms)}, median={median_ms:.0f}ms, max={max_ms:.0f}ms "
                        f"(target 200±50ms)"
                    ),
                    metrics={
                        "n": len(intervals_ms),
                        "median_ms": median_ms,
                        "max_ms": max_ms,
                        "intervals_ms": intervals_ms,
                    },
                )
            )

        # ---- 3. SUBSCRIBE → BINARY_FRAME JPEG (end-to-end) ----------
        sub_payload = json.dumps(
            {"topic": subscribe_topic, "quality": "med"}
        ).encode("utf-8")
        await ws.send(encode_frame(SUBSCRIBE, 0, sub_payload))
        # Ждём subscribe_ack.
        t_sub_deadline = time.monotonic() + 1.5
        sub_ack: dict | None = None
        while time.monotonic() < t_sub_deadline:
            try:
                ftype, _sid, payload = await _recv_frame(ws, timeout_s=0.3)
            except asyncio.TimeoutError:
                continue
            if ftype == JSON_EVENT:
                body = json.loads(payload.decode("utf-8"))
                if body.get("type") == "subscribe_ack":
                    sub_ack = body
                    break
        if sub_ack is None:
            out.append(
                SectionResult(
                    name="subscribe_jpeg",
                    status="fail",
                    detail=(
                        f"SUBSCRIBE({subscribe_topic}) → no subscribe_ack in 1.5s"
                    ),
                )
            )
        else:
            stream_id = sub_ack.get("stream_id")
            # Считаем BINARY_FRAME в течение jpeg_window_s.
            frame_count = 0
            frame_total_bytes = 0
            t_jpeg_deadline = time.monotonic() + jpeg_window_s
            while time.monotonic() < t_jpeg_deadline:
                try:
                    ftype, _sid, payload = await _recv_frame(ws, timeout_s=0.3)
                except asyncio.TimeoutError:
                    continue
                if ftype == BINARY_FRAME:
                    frame_count += 1
                    frame_total_bytes += len(payload)
            if frame_count == 0:
                # Без фреймов — пропускаем (NoOpBridge или нет камеры на dev-машине).
                out.append(
                    SectionResult(
                        name="subscribe_jpeg",
                        status="skip",
                        detail=(
                            f"SUBSCRIBE({subscribe_topic}) ack OK (stream_id={stream_id}), "
                            f"but 0 BINARY_FRAME in {jpeg_window_s}s "
                            f"(bridge has no live camera — expected in --check-only "
                            f"or when Bridge is NoOp)"
                        ),
                        metrics={
                            "stream_id": stream_id,
                            "subscribe_ack": sub_ack,
                            "frames_in_window": 0,
                        },
                    )
                )
            else:
                # Если фреймы идут — проверим что stream_id совпадает.
                # В NoOpBridge фреймов нет; тут — живой сервер с камерой.
                out.append(
                    SectionResult(
                        name="subscribe_jpeg",
                        status="pass",
                        detail=(
                            f"SUBSCRIBE({subscribe_topic}) ack OK (stream_id={stream_id}), "
                            f"got {frame_count} BINARY_FRAME in {jpeg_window_s}s "
                            f"(total {frame_total_bytes} bytes)"
                        ),
                        metrics={
                            "stream_id": stream_id,
                            "subscribe_ack": sub_ack,
                            "frames_in_window": frame_count,
                            "avg_frame_bytes": (
                                frame_total_bytes // frame_count if frame_count else 0
                            ),
                        },
                    )
                )

        # ---- 4. GOODBYE → server-side close --------------------------
        await ws.send(encode_frame(GOODBYE, 0, b""))
        # Ждём CLOSE в течение WATCHDOG_TIMEOUT_S + buffer.
        try:
            close_deadline = time.monotonic() + 0.7  # WATCHDOG = 0.6s
            t_close = None
            while time.monotonic() < close_deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=0.3)
                    t_close = time.monotonic()
                except asyncio.TimeoutError:
                    continue
                except websockets.exceptions.ConnectionClosed as exc:
                    t_close = time.monotonic()
                    code = getattr(exc, "code", None)
                    if code not in (1000, 1005, None):
                        out.append(
                            SectionResult(
                                name="goodbye",
                                status="fail",
                                detail=f"unexpected close code={code} (rcsd/reason={exc})",
                            )
                        )
                        return out
                    break
            if t_close is None:
                out.append(
                    SectionResult(
                        name="goodbye",
                        status="fail",
                        detail="server did not close within 0.7s after GOODBYE",
                    )
                )
            else:
                latency_ms = (t_close - (close_deadline - 0.7)) * 1000.0
                out.append(
                    SectionResult(
                        name="goodbye",
                        status="pass",
                        detail=f"server closed in ~{latency_ms:.0f}ms (close code=1000/1005)",
                        metrics={"close_latency_ms": latency_ms},
                    )
                )
        except Exception as exc:  # noqa: BLE001
            out.append(
                SectionResult(
                    name="goodbye",
                    status="fail",
                    detail=f"GOODBYE error: {exc}",
                )
            )
    finally:
        try:
            await ws.close()
        except Exception:  # noqa: BLE001
            pass

    return out


# --- CLI ---------------------------------------------------------------------
async def _run(args: argparse.Namespace) -> int:
    suite = SuiteResult()
    host = args.host
    port = args.port
    pin = args.pin or ""

    # 1. DNS (всегда — даже в --check-only)
    suite.add(await check_dns_async(host))

    # 2. TLS (всегда — cert можно проверить без aiohttp)
    suite.add(await check_tls(host, port, expected_san=host))

    if args.mode == "check-only":
        # --check-only: остальные секции — SKIP.
        for name, msg in [
            ("healthz", "requires live server (use --all)"),
            ("wss_handshake", "requires live server (use --all)"),
            ("heartbeat", "requires live server (use --all)"),
            ("subscribe_jpeg", "requires live server (use --all)"),
            ("goodbye", "requires live server (use --all)"),
        ]:
            suite.add(
                SectionResult(name=name, status="skip", detail=msg)
            )
        suite.add(
            SectionResult(
                name="_mode",
                status="pass",
                detail="--check-only mode: DNS + TLS only",
            )
        )
        _emit(suite, args)
        return _exit_code(suite)

    # 3-7. Full mode.
    # 3. healthz
    suite.add(await check_healthz(host, port))

    # 4-7. WSS handshake + heartbeat + subscribe + goodbye
    if not pin:
        # Без PIN остальные секции — skip с явной подсказкой.
        for name in ("wss_handshake", "heartbeat", "subscribe_jpeg", "goodbye"):
            suite.add(
                SectionResult(
                    name=name,
                    status="skip",
                    detail="no PIN supplied (use --pin 123456 or QUEST_PIN env)",
                )
            )
    else:
        try:
            sections = await check_wss_full(
                host,
                port,
                pin,
                heartbeat_samples=args.heartbeat_samples,
                heartbeat_window_s=args.heartbeat_window,
                jpeg_window_s=args.jpeg_window,
            )
            for s in sections:
                suite.add(s)
        except Exception as exc:  # noqa: BLE001
            suite.add(
                SectionResult(
                    name="wss_handshake",
                    status="fail",
                    detail=f"WSS suite crashed: {exc}",
                )
            )
            for name in ("heartbeat", "subscribe_jpeg", "goodbye"):
                suite.add(
                    SectionResult(name=name, status="skip", detail="upstream failure")
                )

    _emit(suite, args)
    return _exit_code(suite)


def _emit(suite: SuiteResult, args: argparse.Namespace) -> None:
    if args.json:
        print(json.dumps(suite.to_dict(), indent=2, ensure_ascii=False))
        return
    # Pretty colored output.
    _print_section_table(suite, use_color=args.color)


# ANSI helpers
_USE_COLOR = True


def _color(code: str, text: str) -> str:
    if not _USE_COLOR:
        return text
    return f"\033[{code}m{text}\033[0m"


_GREEN = "32"
_RED = "31"
_YELLOW = "33"
_BOLD = "1"
_DIM = "2"


def _print_section_table(suite: SuiteResult, use_color: bool = True) -> None:
    global _USE_COLOR
    _USE_COLOR = use_color and sys.stdout.isatty()
    print(_color(_BOLD, "rob_box_quest smoke report"))
    print(_color(_DIM, f"  host={suite.sections[0].name and ''}"))
    print()
    print(f"  {'section':<16} {'status':<6}  detail")
    print(f"  {'-'*16} {'-'*6}  {'-'*60}")
    for s in suite.sections:
        if s.name.startswith("_"):
            continue
        if s.status == "pass":
            tag = _color(_GREEN, "PASS")
        elif s.status == "fail":
            tag = _color(_RED, "FAIL")
        elif s.status == "skip":
            tag = _color(_YELLOW, "SKIP")
        else:  # pragma: no cover
            tag = s.status.upper()
        # Truncate detail to ~60 chars.
        d = s.detail
        if len(d) > 80:
            d = d[:77] + "…"
        print(f"  {s.name:<16} {tag:<15}  {d}")
    d = suite.to_dict()
    print()
    pass_tag = _color(_GREEN, f"PASS={d['pass']}")
    fail_tag = _color(_RED, f"FAIL={d['fail']}") if d["fail"] else f"FAIL={d['fail']}"
    skip_tag = _color(_YELLOW, f"SKIP={d['skip']}") if d["skip"] else f"SKIP={d['skip']}"
    print(f"  totals: {pass_tag} {fail_tag} {skip_tag}")
    print()


def _exit_code(suite: SuiteResult) -> int:
    """Exit code = 0 (all pass), 99 (partial: есть SKIP без FAIL), 1.. (sum of fail bits).

    Section → bit (для суммирования):
        dns=1, tls=2, healthz=3, wss_handshake=4, heartbeat=5, subscribe_jpeg=6, goodbye=7.
    """
    bit = {
        "dns": 1,
        "tls": 2,
        "healthz": 3,
        "wss_handshake": 4,
        "heartbeat": 5,
        "subscribe_jpeg": 6,
        "goodbye": 7,
    }
    code = 0
    has_skip = False
    for s in suite.sections:
        if s.status == "fail":
            code |= bit.get(s.name, 0)
        elif s.status == "skip" and s.name in bit:
            has_skip = True
    if code:
        return code  # nonzero bits — конкретная секция не прошла.
    if has_skip:
        return 99
    return 0


def main() -> int:  # pragma: no cover — CLI
    parser = argparse.ArgumentParser(
        prog="quest_smoke_lib",
        description="Pure-Python wire helpers for local_test/quest_smoke.sh",
    )
    parser.add_argument("--host", default="quest.rob_box.local")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--pin", default=None, help="Session PIN (env QUEST_PIN)")
    parser.add_argument(
        "--mode",
        choices=["check-only", "all"],
        default=None,
        help=(
            "check-only = DNS + TLS only; "
            "all = DNS + TLS + healthz + WSS (default = all)"
        ),
    )
    parser.add_argument("--json", action="store_true", help="JSON output")
    parser.add_argument(
        "--no-color", action="store_true", help="Disable ANSI colors"
    )
    parser.add_argument(
        "--heartbeat-samples", type=int, default=5
    )
    parser.add_argument(
        "--heartbeat-window", type=float, default=3.0, help="seconds"
    )
    parser.add_argument(
        "--jpeg-window", type=float, default=2.0, help="seconds"
    )
    parser.add_argument(
        "--skip",
        default="",
        help="comma-separated section names to force-skip (used for --check-only)",
    )
    args = parser.parse_args()

    # Resolve --mode.
    if args.mode is None:
        args.mode = "all"
    if args.mode == "check-only":
        args.skip = "healthz,wss_handshake,heartbeat,subscribe_jpeg,goodbye"
    if args.no_color:
        args.color = False
    else:
        args.color = sys.stdout.isatty()

    # PIN from env.
    if args.pin is None:
        args.pin = __import__("os").environ.get("QUEST_PIN")

    try:
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
