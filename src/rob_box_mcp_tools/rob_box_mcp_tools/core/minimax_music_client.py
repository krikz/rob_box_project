#!/usr/bin/env python3
"""
minimax_music_client.py — HTTP client for MiniMax Music Generation API.

Implements a thin synchronous wrapper around
``POST https://api.minimax.io/v1/music_generation`` so the MCP server can call
it via ``asyncio.to_thread`` (the API can take 40-160s wall time per request —
issue #1358 measured avg 88s in-container).

Endpoint contract (from public docs, https://platform.minimax.io/docs/api-reference/music-generation):

    Request:
        POST https://api.minimax.io/v1/music_generation
        Authorization: Bearer $MINIMAX_API_KEY
        Content-Type: application/json
        {
            "model": "music-3.0-free" | "music-3.0",
            "prompt": "<style/mood, 1-2000 chars>",
            "lyrics": "<text with [Verse]/[Chorus]... tags, optional when is_instrumental>",
            "is_instrumental": false,
            "output_format": "hex",       # default — we decode to bytes
            "audio_setting": {
                "sample_rate": 44100,
                "bitrate": 256000,
                "format": "mp3"
            }
        }

    Response:
        {
            "data": {"audio": "<hex>", "status": 2},
            "extra_info": {"music_duration": ms, "music_sample_rate": 44100,
                           "bitrate": 256000, "audio_length_ms": ...},
            "base_resp": {"status_code": 0, "status_msg": "success"}
        }

Failure modes handled:
    * non-2xx HTTP status → MinimaxMusicAPIError with status + body
    * base_resp.status_code != 0 → MinimaxMusicAPIError with status_msg
    * hex decode failure → MinimaxMusicAPIError
    * urllib timeout → wrapped into MinimaxMusicAPIError("timeout")
    * missing MINIMAX_API_KEY → MinimaxMusicConfigError (init-time)

This module deliberately has **no ROS 2 dependencies** so it can be unit-tested
without a running daemon (mirrors the design of ``rob_box_llm.providers.minimax_tts``).
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from urllib import error as urllib_error
from urllib import request as urllib_request


# ── Defaults ────────────────────────────────────────────────────────────────
_DEFAULT_ENDPOINT = os.getenv(
    "MINIMAX_MUSIC_ENDPOINT",
    "https://api.minimax.io/v1/music_generation",
)
_DEFAULT_MODEL_FREE = "music-3.0-free"
_DEFAULT_MODEL_PAID = "music-3.0"
_DEFAULT_TIMEOUT_S = 200.0  # measured avg ~88s + headroom (issue #1358)


class MinimaxMusicError(Exception):
    """Base class for all errors raised by this module."""


class MinimaxMusicConfigError(MinimaxMusicError):
    """Raised at init-time when configuration is unusable (no API key, etc.)."""


class MinimaxMusicAPIError(MinimaxMusicError):
    """Raised when the remote API returns an error or unreachable."""

    def __init__(self, message: str, *, status_code: Optional[int] = None,
                 body: Optional[str] = None, base_resp: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.body = body
        self.base_resp = base_resp or None


@dataclass
class MinimaxMusicTrack:
    """Result of a successful ``generate()`` call.

    Attributes:
        audio_bytes:        decoded mp3 payload (raw bytes).
        duration_ms:        duration in milliseconds (from extra_info.music_duration).
        sample_rate:        sample rate in Hz (typically 44100).
        bitrate:            bitrate in bps (typically 256000).
        model:              model name actually used.
        raw_response:       full decoded JSON response (for debugging / metadata).
    """
    audio_bytes: bytes
    duration_ms: int = 0
    sample_rate: int = 44100
    bitrate: int = 256000
    model: str = ""
    raw_response: Dict[str, Any] = field(default_factory=dict)


class MinimaxMusicClient:
    """Synchronous HTTP client for the MiniMax music_generation endpoint.

    Args:
        api_key:    MiniMax bearer token. Defaults to ``$MINIMAX_API_KEY``.
        endpoint:   API endpoint URL. Defaults to ``$MINIMAX_MUSIC_ENDPOINT``
                    or the public ``https://api.minimax.io/v1/music_generation``.
        timeout_s:  Per-request timeout in seconds (default 200).
        default_model: Model to use when caller doesn't specify. Defaults to
                    ``music-3.0-free`` (RPM 3). Paid model ``music-3.0`` (RPM 120)
                    is also supported.
    """

    _init_lock = threading.Lock()

    def __init__(
        self,
        api_key: Optional[str] = None,
        *,
        endpoint: Optional[str] = None,
        timeout_s: float = _DEFAULT_TIMEOUT_S,
        default_model: str = _DEFAULT_MODEL_FREE,
    ) -> None:
        resolved_key = api_key if api_key is not None else os.getenv("MINIMAX_API_KEY")
        if not resolved_key or not resolved_key.strip():
            raise MinimaxMusicConfigError(
                "MINIMAX_API_KEY is not set — MiniMax music generation is disabled. "
                "Set the env var or pass api_key= explicitly to enable."
            )

        self._api_key = resolved_key.strip()
        self._endpoint = (endpoint or _DEFAULT_ENDPOINT).rstrip("/")
        self._timeout_s = float(timeout_s)
        self._default_model = default_model

    # ── Public API ─────────────────────────────────────────────────────────

    def generate(
        self,
        *,
        prompt: str,
        lyrics: Optional[str] = None,
        is_instrumental: bool = False,
        model: Optional[str] = None,
        sample_rate: int = 44100,
        bitrate: int = 256000,
        audio_format: str = "mp3",
    ) -> MinimaxMusicTrack:
        """Generate a music track from MiniMax Music API.

        Args:
            prompt:           Style / mood description (1-2000 chars).
            lyrics:           Song lyrics with [Verse]/[Chorus] tags. Required unless
                              ``is_instrumental=True``. Pass empty string for
                              fully-instrumental pieces (MiniMax accepts it when
                              is_instrumental is true).
            is_instrumental:  When True, generate instrumental-only (no vocals).
            model:            Model name override. Defaults to ``self._default_model``.
            sample_rate:      44100 (default) / 8000 / 16000 etc.
            bitrate:          256000 (default) / 128000 / 320000 etc.
            audio_format:     "mp3" (default) — MiniMax also supports "wav"/"pcm".

        Returns:
            ``MinimaxMusicTrack`` with decoded audio bytes + metadata.

        Raises:
            MinimaxMusicAPIError: On HTTP/transport failure or non-success base_resp.
            ValueError:           On invalid arguments (empty prompt etc.).
        """
        if not prompt or not prompt.strip():
            raise ValueError("prompt must be a non-empty string")
        if not is_instrumental and (not lyrics or not lyrics.strip()):
            raise ValueError("lyrics must be non-empty when is_instrumental=False")
        if len(prompt) > 2000:
            raise ValueError(f"prompt is {len(prompt)} chars; MiniMax limit is 2000")

        body: Dict[str, Any] = {
            "model": model or self._default_model,
            "prompt": prompt.strip(),
            "is_instrumental": bool(is_instrumental),
            "output_format": "hex",
            "audio_setting": {
                "sample_rate": int(sample_rate),
                "bitrate": int(bitrate),
                "format": str(audio_format),
            },
        }
        if not is_instrumental:
            body["lyrics"] = (lyrics or "").strip()

        payload = json.dumps(body).encode("utf-8")
        req = urllib_request.Request(
            self._endpoint,
            data=payload,
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )

        try:
            with urllib_request.urlopen(req, timeout=self._timeout_s) as resp:
                raw_bytes = resp.read()
                http_status = getattr(resp, "status", 200)
        except urllib_error.HTTPError as exc:
            err_body = ""
            try:
                err_body = exc.read().decode("utf-8", errors="replace")
            except Exception:  # noqa: BLE001
                pass
            raise MinimaxMusicAPIError(
                f"MiniMax music API HTTP {exc.code}: {exc.reason}",
                status_code=exc.code,
                body=err_body,
            ) from exc
        except urllib_error.URLError as exc:
            # includes timeout
            raise MinimaxMusicAPIError(
                f"MiniMax music API transport error: {exc.reason}",
            ) from exc

        # Parse JSON
        try:
            parsed = json.loads(raw_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MinimaxMusicAPIError(
                f"MiniMax music API returned non-JSON body: {exc}",
                status_code=http_status,
                body=raw_bytes[:512].decode("utf-8", errors="replace"),
            ) from exc

        # Inspect base_resp
        base_resp = parsed.get("base_resp") or {}
        if base_resp.get("status_code", 0) != 0:
            raise MinimaxMusicAPIError(
                f"MiniMax music API error: {base_resp.get('status_msg', 'unknown')}",
                status_code=base_resp.get("status_code"),
                body=json.dumps(parsed, ensure_ascii=False)[:512],
                base_resp=base_resp,
            )

        # Decode audio
        data = parsed.get("data") or {}
        audio_hex = data.get("audio")
        if not audio_hex or not isinstance(audio_hex, str):
            raise MinimaxMusicAPIError(
                "MiniMax music API returned empty/invalid audio payload",
                status_code=http_status,
                body=json.dumps(parsed, ensure_ascii=False)[:512],
                base_resp=base_resp,
            )

        try:
            audio_bytes = bytes.fromhex(audio_hex)
        except ValueError as exc:
            raise MinimaxMusicAPIError(
                f"MiniMax music API returned non-hex audio payload: {exc}",
                status_code=http_status,
                body=audio_hex[:64],
                base_resp=base_resp,
            ) from exc

        extra = parsed.get("extra_info") or {}
        return MinimaxMusicTrack(
            audio_bytes=audio_bytes,
            duration_ms=int(extra.get("music_duration") or 0),
            sample_rate=int(extra.get("music_sample_rate") or sample_rate),
            bitrate=int(extra.get("bitrate") or bitrate),
            model=str(body["model"]),
            raw_response=parsed,
        )

    # ── Diagnostics ────────────────────────────────────────────────────────

    @property
    def is_configured(self) -> bool:
        """True if the client was constructed successfully (key present)."""
        return bool(self._api_key)

    @property
    def endpoint(self) -> str:
        return self._endpoint

    @property
    def default_model(self) -> str:
        return self._default_model