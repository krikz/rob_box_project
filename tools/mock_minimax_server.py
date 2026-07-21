#!/usr/bin/env python3
"""Standalone HTTP/SSE mock of the MiniMax T2A v2 endpoint.

This mock is a local substitute for the real ``https://api.minimax.io``
endpoint.  It accepts the same request shape (URL, headers, JSON body)
and serves audio from a local fixture directory so any caller — the
``MiniMaxTTSProvider``, the ``tts_node``, or a manual ``curl`` — can be
exercised without real credentials, network connectivity, or paid quota.

Design choices
--------------

* **stdlib-only** — uses ``http.server`` + ``socketserver.ThreadingMixIn``.
  No FastAPI, no Starlette, no external dependencies.  This keeps the
  bench easy to run in CI containers and on developer laptops.

* **Wire-compatible with the production endpoint.**  The accepted
  request shape (URL, headers, JSON body) and the emitted response
  envelopes (both JSON for ``stream=false`` and SSE for ``stream=true``)
  are byte-identical to what ``api.minimax.io`` returns, so the existing
  ``MiniMaxTTSProvider`` consumes the mock output without any code
  changes.  The endpoint reference (see ``rob_box_llm.providers.minimax_tts``
  for the canonical documentation):

  - ``POST /v1/t2a_v2``
  - query param ``GroupId`` (any non-empty value passes)
  - ``Authorization: Bearer ***`` header (any non-empty value passes)
  - JSON body with ``model``, ``text``, ``voice_setting.voice_id``,
    ``audio_setting.{format,sample_rate,channel}`` and ``stream`` flag

  Response (non-streaming): ::

      {
        "data": {"audio": "<hex>", "audio_sample_rate": <hz>, "audio_length": <n>},
        "base_resp": {"status_code": 0, "status_msg": "success"}
      }

  Response (streaming): a series of ``data:<json>\\n\\n`` events
  terminating with ``data:[DONE]\\n\\n`` — exactly the shape the
  production ``stream()`` parser consumes.

* **Pre-built scenarios.**  ``--scenario <name>`` wires the query-param
  defaults (``chunk_count``, ``chunk_delay_ms``, ``fail_status``) to a
  documented recipe.  The three required scenarios from the task brief
  are available out of the box:

  - ``pcm-chunked``    — single chunk, 16 kHz PCM (the default tts_node sample_rate)
  - ``wav-streaming``  — 4 SSE chunks @ ~50 ms apart, 16 kHz WAV
  - ``mp3`` / ``ogg``  — non-streaming MP3; ``ogg`` falls back to MP3

  Each scenario prints the exact ``curl`` command that exercises it.

* **Env-injection aware.**  Reads ``MINIMAX_BASE_URL`` and
  ``MINIMAX_API_KEY`` from the environment at start-up (purely for
  logging / acknowledgement — the mock accepts ANY value for both, so a
  caller can switch from the real endpoint to this mock without
  embedding secrets).  See ``BLOCKERS.md`` for the production-side
  changes required to actually consume ``MINIMAX_BASE_URL`` from env.

Run standalone
--------------

::

    # simplest invocation — non-streaming JSON, single chunk, PCM 16 kHz
    python tools/mock_minimax_server.py

    # explicit scenario (wav streaming with 4 chunks)
    python tools/mock_minimax_server.py --scenario wav-streaming

    # explicit port + fixtures
    python tools/mock_minimax_server.py --port 18080 \\
        --fixtures tts_audio_bench/fixtures

    # health-check endpoint is GET /health on the same port.

Verifying it works
------------------

The mock prints the exact ``curl`` invocations it expects.  Use them to
verify after start-up:

::

    $ curl -s http://127.0.0.1:18080/health
    {"status": "ok", "mock": "MiniMax T2A v2"}

    # non-streaming (JSON) — the canonical "single-shot" path
    $ curl -s -X POST "http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test" \\
        -H "Authorization: Bearer anything" -H "Content-Type: application/json" \\
        -d '{"model":"speech-02-hd","text":"hello","voice_setting":{"voice_id":"male-qn-qingse"},"audio_setting":{"format":"pcm","sample_rate":16000,"channel":1}}' | jq

    # streaming (SSE) — set "stream": true in the body and pipe to jq
    $ curl -N -s -X POST "http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test" \\
        -H "Authorization: Bearer anything" -H "Content-Type: application/json" \\
        -d '{"model":"speech-02-hd","text":"hello","voice_setting":{"voice_id":"male-qn-qingse"},"audio_setting":{"format":"wav","sample_rate":16000,"channel":1},"stream":true}'
"""
from __future__ import annotations

import argparse
import http.server
import json
import logging
import os
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

_log = logging.getLogger("mock_minimax")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Default fixture samplerate used to pick the matching reference file.
_DEFAULT_SR = 32000

#: ~64 ms @ 16 kHz mono int16 — a realistic TTS chunk.
_STREAM_CHUNK_BYTES = 4096

#: Format → fixture subdirectory.  ``ogg`` falls back to ``mp3`` per
#: MiniMax's documented behaviour (the production provider surfaces
#: this as ``TTSFormat.MP3`` in the returned :class:`TTSAudio`).
_FMT_DIRS = {
    "pcm": "pcm",
    "wav": "wav",
    "mp3": "mp3",
    "ogg": "ogg",
}

#: Pre-baked scenarios — these set query-param defaults so the caller's
#: request doesn't have to repeat them.  Each entry maps to the kwargs
#: accepted by :meth:`_MockHandler.install_scenario`.
SCENARIOS: Dict[str, Dict[str, Any]] = {
    # Default — single-chunk non-streaming JSON, PCM 16 kHz.
    "pcm-chunked": {
        "description": "Single-chunk non-streaming JSON, PCM 16 kHz.",
        "stream_chunk_count": 1,
        "stream_chunk_delay_ms": 0,
        "fail_status": None,
    },
    # 4-chunk SSE streaming — exercises the time-to-first-audio path.
    "wav-streaming": {
        "description": "4 SSE chunks @ ~50 ms apart, 16 kHz WAV. "
        "Drives time-to-first-audio measurement.",
        "stream_chunk_count": 4,
        "stream_chunk_delay_ms": 50,
        "fail_status": None,
    },
    # MP3 non-streaming — validates that hex-decoded MP3 is consumable.
    "mp3": {
        "description": "Non-streaming MP3, 16 kHz (provider falls back from OGG).",
        "stream_chunk_count": 1,
        "stream_chunk_delay_ms": 0,
        "fail_status": None,
    },
    # OGG: MiniMax doesn't support OGG natively — provider falls back to MP3.
    # The mock serves real OGG if the fixture is present, MP3 otherwise.
    "ogg": {
        "description": "OGG format: real OGG fixture if present, else MP3 fallback.",
        "stream_chunk_count": 1,
        "stream_chunk_delay_ms": 0,
        "fail_status": None,
    },
    # Negative path — auth failure injected as MiniMax base_resp error.
    "auth-error": {
        "description": "Injects base_resp.status_code=1001 (auth) on every request.",
        "stream_chunk_count": 1,
        "stream_chunk_delay_ms": 0,
        "fail_status": 1001,
        "fail_msg": "invalid api key",
    },
}


def _resolve_fixture(
    fixtures_root: Path,
    fmt: str,
    sample_rate: int,
) -> Optional[bytes]:
    """Return the bytes for the requested fixture, or ``None`` if missing.

    Mirrors MiniMax's documented behaviour: ``ogg`` → ``mp3`` fallback
    when no OGG fixture is present.
    """
    ext = fmt
    candidate = (
        fixtures_root
        / _FMT_DIRS.get(fmt, fmt)
        / f"sine_440hz_1.00s_{sample_rate}.{ext}"
    )
    if not candidate.exists() and fmt == "ogg":
        candidate = (
            fixtures_root / "mp3" / f"sine_440hz_1.00s_{sample_rate}.mp3"
        )
    if not candidate.exists():
        return None
    return candidate.read_bytes()


class _MockHandler(BaseHTTPRequestHandler):
    """httplib-style handler that returns MiniMax-shaped envelopes.

    Class-level attributes are overridden by :func:`start_server` per
    invocation — they're not safe to mutate concurrently, but the
    threading server only ever reads them after construction (the request
    path never mutates them).
    """

    fixtures_root: Path = Path("tts_audio_bench/fixtures")
    fail_status: Optional[int] = None
    fail_msg: str = "injected failure"
    stream_chunk_count: Optional[int] = None
    stream_chunk_delay_ms: Optional[float] = None

    # Silence stderr; we route the same lines through the logger at INFO.
    def log_message(self, fmt: str, *args: Any) -> None:  # noqa: D401
        _log.info("mock_minimax: " + fmt, *args)

    # ------------------------------------------------------------------
    # HTTP verbs
    # ------------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802
        """Health-check / introspection endpoint — non-existent in MiniMax.

        Returning useful diagnostics on a side channel keeps the mock
        observable without polluting the documented ``/v1/t2a_v2``
        contract.
        """
        parsed = urlparse(self.path)
        if parsed.path == "/health":
            body = json.dumps(
                {
                    "status": "ok",
                    "mock": "MiniMax T2A v2",
                    "fixtures_root": str(self.fixtures_root),
                    "active_fail_status": self.fail_status,
                    "active_chunk_count": self.stream_chunk_count,
                    "active_chunk_delay_ms": self.stream_chunk_delay_ms,
                }
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404, f"unknown path: {parsed.path}")

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/v1/t2a_v2":
            self._send_json(404, {"base_resp": {"status_code": 404, "status_msg": f"unknown path: {parsed.path}"}})
            return

        qs = parse_qs(parsed.query)
        # GroupId is mandatory — MiniMax requires it as a query param.
        group_ids = qs.get("GroupId") or qs.get("group_id") or []
        if not group_ids or not group_ids[0]:
            self._send_json(
                401,
                {"base_resp": {"status_code": 1001, "status_msg": "missing GroupId"}},
            )
            return
        auth = self.headers.get("Authorization", "")
        if not auth.startswith("Bearer "):
            self._send_json(
                401,
                {"base_resp": {"status_code": 1001, "status_msg": "missing Bearer"}},
            )
            return

        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except json.JSONDecodeError:
            self._send_json(
                400,
                {"base_resp": {"status_code": 1002, "status_msg": "bad json"}},
            )
            return

        # Per-request failure injection (overrides the class default).
        qs_fail = qs.get("fail_status", [None])[0]
        qs_fail_msg = qs.get("fail_msg", [None])[0]
        fail_status = int(qs_fail) if qs_fail is not None else self.fail_status
        fail_msg = qs_fail_msg or self.fail_msg
        if fail_status is not None:
            self._send_json(
                200,
                {"base_resp": {"status_code": fail_status, "status_msg": fail_msg}},
            )
            return

        audio_setting = payload.get("audio_setting") or {}
        voice_setting = payload.get("voice_setting") or {}
        fmt = (audio_setting.get("format") or "pcm").lower()
        sr = int(audio_setting.get("sample_rate") or _DEFAULT_SR)
        voice_id = voice_setting.get("voice_id") or "unknown"

        fixture = _resolve_fixture(self.fixtures_root, fmt, sr)
        if fixture is None:
            self._send_json(
                200,
                {
                    "base_resp": {
                        "status_code": 1002,
                        "status_msg": f"no fixture for fmt={fmt} sr={sr}",
                    }
                },
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

        # Streaming path: SSE with HTTP/1.1 chunked transfer-encoding so
        # each event flushes independently.  ``chunk_count`` (default 1)
        # slices the fixture across N events; the FIRST chunk is sent
        # immediately so TTFA = transport latency, independent of how
        # the upstream paces the remaining data.
        if payload.get("stream"):
            chunk_count_raw = qs.get("chunk_count", [None])[0]
            chunk_delay_raw = qs.get("chunk_delay_ms", [None])[0]
            try:
                chunk_count = (
                    max(1, int(chunk_count_raw))
                    if chunk_count_raw is not None
                    else (
                        self.stream_chunk_count
                        if self.stream_chunk_count is not None
                        else 1
                    )
                )
            except (TypeError, ValueError):
                chunk_count = (
                    self.stream_chunk_count
                    if self.stream_chunk_count is not None
                    else 1
                )
            try:
                chunk_delay_ms = (
                    max(0.0, float(chunk_delay_raw))
                    if chunk_delay_raw is not None
                    else (
                        self.stream_chunk_delay_ms
                        if self.stream_chunk_delay_ms is not None
                        else 0.0
                    )
                )
            except (TypeError, ValueError):
                chunk_delay_ms = (
                    self.stream_chunk_delay_ms
                    if self.stream_chunk_delay_ms is not None
                    else 0.0
                )

            # Slice fixture into ``chunk_count`` equal pieces (with
            # ``_STREAM_CHUNK_BYTES`` minimum granularity so a 4 KB WAV
            # gets >1 chunk instead of collapsing to one).
            n = chunk_count
            step = max(_STREAM_CHUNK_BYTES, len(fixture) // n)
            pieces = [
                fixture[i : i + step] for i in range(0, len(fixture), step)
            ]
            while len(pieces) < n:
                pieces.append(b"\x00\x00")

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
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

    def _write_chunk(self, payload: bytes) -> None:
        """Emit one HTTP/1.1 chunked transfer-encoding chunk and flush.

        Without flushing, asyncio / httpx readers may not see the first
        event until the full response is buffered, defeating the point
        of streaming.  ``BrokenPipeError`` is swallowed because a client
        hangup mid-stream is not actionable from the server side.
        """
        self.wfile.write(f"{len(payload):X}\r\n".encode("ascii"))
        self.wfile.write(payload)
        self.wfile.write(b"\r\n")
        try:
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _flush_last_chunk(self) -> None:
        try:
            self.wfile.write(b"0\r\n\r\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass


class _ReuseAddrServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Threading HTTPServer with SO_REUSEADDR for bench re-runs."""

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

    The handle's ``shutdown()`` cleanly stops the server; the
    accompanying thread exits once the in-flight handler returns.
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
    _log.info(
        "mock_minimax listening on http://127.0.0.1:%d (fixtures=%s)",
        port,
        fixtures_root,
    )
    return server


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _print_curl_examples(port: int, scenario: str) -> None:
    """Print the canonical curl invocations for the active scenario.

    These match the examples in the module docstring; printing them at
    start-up means a developer can copy-paste to verify the mock
    without reading the docs first.
    """
    print(f"\n  # Health check\n"
          f"  curl -s http://127.0.0.1:{port}/health\n")
    print("  # Non-streaming JSON (canonical 'single-shot' path):\n"
          "  curl -s -X POST "
          f"\"http://127.0.0.1:{port}/v1/t2a_v2?GroupId=local-test\" \\\n"
          "    -H \"Authorization: Bearer anything\" "
          "-H \"Content-Type: application/json\" \\\n"
          "    -d '{\"model\":\"speech-02-hd\",\"text\":\"hello\","
          "\"voice_setting\":{\"voice_id\":\"male-qn-qingse\"},"
          "\"audio_setting\":{\"format\":\"pcm\",\"sample_rate\":16000,"
          "\"channel\":1}}'\n")
    print("  # Streaming SSE (set '\"stream\":true' in body):\n"
          "  curl -N -s -X POST "
          f"\"http://127.0.0.1:{port}/v1/t2a_v2?GroupId=local-test\" \\\n"
          "    -H \"Authorization: Bearer anything\" "
          "-H \"Content-Type: application/json\" \\\n"
          "    -d '{\"model\":\"speech-02-hd\",\"text\":\"hello\","
          "\"voice_setting\":{\"voice_id\":\"male-qn-qingse\"},"
          "\"audio_setting\":{\"format\":\"wav\",\"sample_rate\":16000,"
          "\"channel\":1},\"stream\":true}'\n")
    print(f"  # Scenario active: {scenario!r}. Per-request overrides:")
    print("    ?chunk_count=N        — split fixture across N SSE events")
    print("    ?chunk_delay_ms=N     — sleep between SSE events")
    print("    ?fail_status=N        — inject MiniMax base_resp.status_code=N")
    print("    ?fail_msg=...         — accompanying status_msg\n")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Local HTTP/SSE mock for the MiniMax T2A v2 endpoint."
    )
    parser.add_argument(
        "--port",
        type=int,
        default=18080,
        help="TCP port to bind (default: 18080).",
    )
    parser.add_argument(
        "--fixtures",
        default="tts_audio_bench/fixtures",
        help="Path to the directory containing pcm/wav/mp3/ogg fixtures "
        "(default: tts_audio_bench/fixtures, relative to repo root).",
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS.keys()),
        default="pcm-chunked",
        help="Pre-built scenario (default: pcm-chunked).",
    )
    parser.add_argument(
        "--fail-status",
        type=int,
        default=None,
        help="Override base_resp.status_code returned for every request. "
        "Takes precedence over the scenario's fail_status.",
    )
    parser.add_argument(
        "--fail-msg",
        default=None,
        help="Override base_resp.status_msg returned for every request.",
    )
    parser.add_argument(
        "--chunk-count",
        type=int,
        default=None,
        help="Override the SSE chunk count (default: scenario value).",
    )
    parser.add_argument(
        "--chunk-delay-ms",
        type=float,
        default=None,
        help="Override the SSE per-chunk delay (ms, default: scenario value).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable DEBUG-level logging.",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    # Acknowledge env-injected values — these are purely diagnostic since
    # the mock accepts ANY non-empty value for both.  See BLOCKERS.md for
    # the production-side blocker: the provider and tts_node do not
    # currently read ``MINIMAX_BASE_URL`` from env, so a caller cannot
    # switch the live endpoint without a code change.
    base_url_env = os.environ.get("MINIMAX_BASE_URL")
    api_key_env = os.environ.get("MINIMAX_API_KEY")
    group_id_env = os.environ.get("MINIMAX_GROUP_ID")
    if base_url_env:
        _log.info("MINIMAX_BASE_URL=%s (env honoured; mock accepts any value)", base_url_env)
    if api_key_env:
        # Don't log the actual key — just confirm presence for parity with
        # production-side redaction (see test_minimax_tts_logging.py).
        redacted = api_key_env[:4] + "***" if len(api_key_env) >= 4 else "***"
        _log.info("MINIMAX_API_KEY=%s (env honoured; mock accepts any value)", redacted)
    if group_id_env:
        _log.info("MINIMAX_GROUP_ID=%s (env honoured)", group_id_env)

    # Resolve scenario defaults — explicit CLI flags win over scenario defaults.
    scenario_cfg = SCENARIOS[args.scenario]
    fail_status = (
        args.fail_status
        if args.fail_status is not None
        else scenario_cfg.get("fail_status")
    )
    fail_msg = args.fail_msg or scenario_cfg.get("fail_msg", "injected failure")
    stream_chunk_count = (
        args.chunk_count
        if args.chunk_count is not None
        else scenario_cfg.get("stream_chunk_count")
    )
    stream_chunk_delay_ms = (
        args.chunk_delay_ms
        if args.chunk_delay_ms is not None
        else scenario_cfg.get("stream_chunk_delay_ms", 0.0)
    )

    fixtures_root = Path(args.fixtures).resolve()
    if not fixtures_root.is_dir():
        print(
            f"ERROR: fixtures directory not found: {fixtures_root}\n"
            f"  Run from the repo root, or pass --fixtures /absolute/path.",
            file=sys.stderr,
        )
        return 2

    server = start_server(
        port=args.port,
        fixtures_root=fixtures_root,
        fail_status=fail_status,
        fail_msg=fail_msg,
        stream_chunk_count=stream_chunk_count,
        stream_chunk_delay_ms=stream_chunk_delay_ms,
    )

    print(
        f"\nmock_minimax listening on http://127.0.0.1:{args.port}\n"
        f"  scenario:       {args.scenario!r} ({scenario_cfg['description']})\n"
        f"  fixtures_root:  {fixtures_root}\n"
        f"  fail_status:    {fail_status}\n"
        f"  chunk_count:    {stream_chunk_count}\n"
        f"  chunk_delay_ms: {stream_chunk_delay_ms}\n"
        f"  Ctrl-C to stop.\n"
    )
    _print_curl_examples(args.port, args.scenario)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())