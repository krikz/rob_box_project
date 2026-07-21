#!/usr/bin/env python3
"""Local HTTP mock of the MiniMax T2A v2 endpoint.

This mock is the test-bench substitute for the real ``https://api.minimax.io``.
It accepts the same request shape (URL, headers, JSON body) but serves audio
from a local fixture directory so the bench can run without:

* real MiniMax credentials (the mock accepts any ``Authorization: Bearer …``
  string and any ``?GroupId=…`` value, so test code does not have to embed
  secrets to "make the call work");
* network connectivity;
* paid quota.

The endpoint contract intentionally mirrors the production one (see
``rob_box_llm.providers.minimax_tts`` for the canonical reference):

* ``POST /v1/t2a_v2``
* query param ``GroupId`` (any non-empty value passes)
* ``Authorization: Bearer <key>`` header (any non-empty value passes)
* JSON body with ``model``, ``text``, ``voice_setting.voice_id``,
  ``audio_setting.{format,sample_rate,channel}`` and ``stream`` flag.

Response shape::

    {
      "data": {"audio": "<hex>", "audio_sample_rate": <hz>, "audio_length": <n>},
      "base_resp": {"status_code": 0, "status_msg": "success"}
    }

For ``format=pcm`` the fixture is read as-is; for ``wav/mp3/ogg`` it is
returned verbatim (the production provider's MP3 fallback for OGG is
replicated here). The mock also supports an injected failure mode
(``?fail_status=<int>&fail_msg=<text>`` query param) for negative-path
tests.

Stream-mode (``"stream": true`` in body) emits a single SSE event with
``data:<json>\\n\\n`` followed by ``data:[DONE]\\n\\n``, exactly the shape
the production provider's stream parser consumes.

Run standalone:

    python mock_minimax_server.py --port 18080 --fixtures tts_audio_bench/fixtures

Or programmatically from ``run_bench.py`` via ``start_server()``.
"""
from __future__ import annotations

import argparse
import http.server
import json
import logging
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

_log = logging.getLogger("mock_minimax")

# Default fixture samplerate used to pick the matching reference file.
_DEFAULT_SR = 32000

# Streaming: number of chunks to slice the fixture into, and per-chunk
# emit delay (seconds). Controlled via ?chunk_count=N and ?chunk_delay_ms=NNN
# URL query params so the test bench can prove ``time-to-first-AudioData``
# stays bounded even when the upstream inserts simulated latency.
_STREAM_DEFAULT_CHUNK_COUNT = 1
_STREAM_DEFAULT_CHUNK_DELAY_MS = 0
_STREAM_CHUNK_BYTES = 4096  # ~64 ms @ 16 kHz mono int16 — a realistic TTS chunk

# Format → fixture subdirectory. The MiniMax API exposes ``pcm | wav | mp3``;
# OGG is mapped to MP3 per the provider's documented fallback. We serve
# ``fixtures/ogg/...`` if present for tests that explicitly request OGG.
_FMT_DIRS = {
    "pcm": "pcm",
    "wav": "wav",
    "mp3": "mp3",
    "ogg": "ogg",  # serve real OGG if available, otherwise fall back to MP3
}


def _resolve_fixture(
    fixtures_root: Path,
    voice_id: str,
    fmt: str,
    sample_rate: int,
) -> Optional[bytes]:
    """Return the bytes for the requested fixture, or ``None`` if missing."""
    # We use a deterministic naming scheme from make_fixture.py:
    #   sine_<freq>hz_<dur>s_<sr>.<ext>
    # The bench picks the first 16 kHz fixture by convention (the same SR
    # the production tts_node publishes to /voice/audio/speech).
    ext = fmt
    candidate = fixtures_root / _FMT_DIRS.get(fmt, fmt) / f"sine_440hz_1.00s_{sample_rate}.{ext}"
    if not candidate.exists() and fmt == "ogg":
        # Mirror MiniMax's documented fallback: OGG → MP3.
        candidate = fixtures_root / "mp3" / f"sine_440hz_1.00s_{sample_rate}.mp3"
    if not candidate.exists():
        return None
    return candidate.read_bytes()


class _MockHandler(BaseHTTPRequestHandler):
    """httplib-style handler that returns MiniMax-shaped envelopes."""

    fixtures_root: Path = Path("tts_audio_bench/fixtures")
    fail_status: Optional[int] = None  # base_resp.status_code override
    fail_msg: str = "injected failure"
    # Streaming defaults — request-level query params override these.
    stream_chunk_count: Optional[int] = None
    stream_chunk_delay_ms: Optional[float] = None

    # Silence stderr; the bench log captures the same lines at INFO.
    def log_message(self, fmt: str, *args: Any) -> None:  # noqa: D401
        _log.info("mock_minimax: " + fmt, *args)

    # ------------------------------------------------------------------
    # Request handling
    # ------------------------------------------------------------------

    def do_POST(self) -> None:  # noqa: N802 — http.server API
        parsed = urlparse(self.path)
        if parsed.path != "/v1/t2a_v2":
            self.send_error(404, f"unknown path: {parsed.path}")
            return

        qs = parse_qs(parsed.query)
        group_ids = qs.get("GroupId") or qs.get("group_id") or []
        if not group_ids or not group_ids[0]:
            self._send_json(401, {"base_resp": {"status_code": 1001, "status_msg": "missing GroupId"}})
            return
        auth = self.headers.get("Authorization", "")
        if not auth.startswith("Bearer "):
            self._send_json(401, {"base_resp": {"status_code": 1001, "status_msg": "missing Bearer"}})
            return

        # Read body.
        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except json.JSONDecodeError:
            self._send_json(400, {"base_resp": {"status_code": 1002, "status_msg": "bad json"}})
            return

        # Honor the failure-injection query params (used by negative tests).
        qs_fail = qs.get("fail_status", [None])[0]
        qs_fail_msg = qs.get("fail_msg", [None])[0]
        fail_status = int(qs_fail) if qs_fail is not None else self.fail_status
        fail_msg = qs_fail_msg or self.fail_msg
        if fail_status is not None:
            self._send_json(200, {"base_resp": {"status_code": fail_status, "status_msg": fail_msg}})
            return

        audio_setting = payload.get("audio_setting") or {}
        voice_setting = payload.get("voice_setting") or {}
        fmt = (audio_setting.get("format") or "pcm").lower()
        sr = int(audio_setting.get("sample_rate") or _DEFAULT_SR)
        voice_id = voice_setting.get("voice_id") or "unknown"

        fixture = _resolve_fixture(self.fixtures_root, voice_id, fmt, sr)
        if fixture is None:
            self._send_json(
                200,
                {"base_resp": {"status_code": 1002, "status_msg": f"no fixture for fmt={fmt} sr={sr}"}},
            )
            return

        envelope: Dict[str, Any] = {
            "data": {
                "audio": fixture.hex(),
                "audio_sample_rate": sr,
                "audio_length": len(fixture),
            },
            "extra_info": {"voice_id": voice_id, "format": fmt, "fixture": True},
            "base_resp": {"status_code": 0, "status_msg": "success"},
        }

        # Streaming responses are SSE-shaped (data: …\n\n + [DONE]).
        # ``chunk_count`` (default 1) splits the fixture across that many SSE
        # events so the bench can measure TTFA while the upstream is still
        # streaming the rest of the audio. ``chunk_delay_ms`` (default 0) is
        # the per-chunk sleep before each subsequent event; the FIRST chunk
        # is always sent immediately so TTFA = transport latency, independent
        # of how the upstream paces the remaining data.
        if payload.get("stream"):
            chunk_count_raw = qs.get("chunk_count", [None])[0]
            chunk_delay_raw = qs.get("chunk_delay_ms", [None])[0]
            # Per-request override ⇒ handler-class default ⇒ constant default.
            try:
                chunk_count = (
                    max(1, int(chunk_count_raw))
                    if chunk_count_raw is not None
                    else (
                        self.stream_chunk_count
                        if self.stream_chunk_count is not None
                        else _STREAM_DEFAULT_CHUNK_COUNT
                    )
                )
            except (TypeError, ValueError):
                chunk_count = (
                    self.stream_chunk_count
                    if self.stream_chunk_count is not None
                    else _STREAM_DEFAULT_CHUNK_COUNT
                )
            try:
                chunk_delay_ms = (
                    max(0.0, float(chunk_delay_raw))
                    if chunk_delay_raw is not None
                    else (
                        self.stream_chunk_delay_ms
                        if self.stream_chunk_delay_ms is not None
                        else _STREAM_DEFAULT_CHUNK_DELAY_MS
                    )
                )
            except (TypeError, ValueError):
                chunk_delay_ms = (
                    self.stream_chunk_delay_ms
                    if self.stream_chunk_delay_ms is not None
                    else _STREAM_DEFAULT_CHUNK_DELAY_MS
                )

            # Slice the fixture bytes across ``chunk_count`` events. Each
            # chunk reports ``chunk_index`` and ``chunk_count`` so the
            # provider can reconstruct ``audio_length`` without ambiguity.
            n = chunk_count
            step = max(_STREAM_CHUNK_BYTES, len(fixture) // n)
            pieces = [
                fixture[i : i + step] for i in range(0, len(fixture), step)
            ]
            # Force at least ``n`` chunks (tiny fixtures get padded zeros).
            while len(pieces) < n:
                pieces.append(b"\x00\x00")

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            # Chunked transfer-encoding so we can flush between events.
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            for idx, piece in enumerate(pieces):
                piece_envelope = {
                    "data": {
                        "audio": piece.hex(),
                        "audio_sample_rate": sr,
                        "audio_length": len(piece),
                    },
                    "extra_info": {
                        "voice_id": voice_id,
                        "format": fmt,
                        "fixture": True,
                        "chunk_index": idx,
                        "chunk_count": len(pieces),
                    },
                    "base_resp": {"status_code": 0, "status_msg": "success"},
                }
                event = f"data:{json.dumps(piece_envelope)}\n\n"
                self._write_chunk(event.encode("utf-8"))
                if idx < len(pieces) - 1 and chunk_delay_ms > 0:
                    time.sleep(chunk_delay_ms / 1000.0)
            self._write_chunk(b"data:[DONE]\n\n")
            self._flush_last_chunk()
            return

        body = json.dumps(envelope).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _send_json(self, status: int, body: Dict[str, Any]) -> None:
        data = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    # ------------------------------------------------------------------
    # Chunked transfer-encoding helpers (used by streaming responses).
    # ------------------------------------------------------------------

    def _write_chunk(self, payload: bytes) -> None:
        """Emit one HTTP/1.1 chunked transfer-encoding chunk and flush.

        Hex length prefix + CRLF + bytes + CRLF, followed by ``flush()``
        so the chunk reaches the client immediately — without flushing,
        asyncio / httpx readers may not see the first event until the
        full response is buffered, defeating the point of streaming.
        """
        self.wfile.write(f"{len(payload):X}\r\n".encode("ascii"))
        self.wfile.write(payload)
        self.wfile.write(b"\r\n")
        try:
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            # Client hung up mid-stream; nothing useful we can do here.
            pass

    def _flush_last_chunk(self) -> None:
        """Write the zero-length chunk that terminates the body."""
        try:
            self.wfile.write(b"0\r\n\r\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass


class _ReuseAddrServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Threading HTTPServer with SO_REUSEADDR so bench re-runs don't TIME_WAIT."""

    allow_reuse_address = True
    daemon_threads = True


def start_server(
    *,
    port: int,
    fixtures_root: Path,
    fail_status: Optional[int] = None,
    fail_msg: str = "injected failure",
    stream_chunk_count: Optional[int] = None,
    stream_chunk_delay_ms: Optional[float] = None,
) -> _ReuseAddrServer:
    """Spin up a server on ``port`` in a daemon thread; return the handle.

    ``stream_chunk_count`` / ``stream_chunk_delay_ms`` (when set) are used as
    *defaults* for every streaming response the mock produces. The matching
    URL query params ``?chunk_count=…`` / ``?chunk_delay_ms=…`` override
    these defaults on a per-request basis.
    """
    handler = type(
        "_BoundHandler",
        (_MockHandler,),
        {
            "fixtures_root": fixtures_root,
            "fail_status": fail_status,
            "fail_msg": fail_msg,
            "stream_chunk_count": stream_chunk_count,
            "stream_chunk_delay_ms": stream_chunk_delay_ms,
        },
    )
    server = _ReuseAddrServer(("127.0.0.1", port), handler)
    t = threading.Thread(target=server.serve_forever, name="mock-minimax", daemon=True)
    t.start()
    _log.info("mock_minimax listening on http://127.0.0.1:%d (fixtures=%s)", port, fixtures_root)
    return server


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=18080)
    p.add_argument("--fixtures", default="tts_audio_bench/fixtures")
    p.add_argument("--fail-status", type=int, default=None)
    p.add_argument("--fail-msg", default="injected failure")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    server = start_server(
        port=args.port,
        fixtures_root=Path(args.fixtures).resolve(),
        fail_status=args.fail_status,
        fail_msg=args.fail_msg,
    )
    _log.info("ready (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())