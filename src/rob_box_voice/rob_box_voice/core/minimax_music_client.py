"""
minimax_music_client.py — async client for the MiniMax Music Generation API.

Endpoint:  ``POST https://api.minimax.io/v1/music_generation``
Auth:      ``Authorization: Bearer $MINIMAX_API_KEY``
Body:      ``{"model": "music-3.0"|"music-3.0-free",
              "prompt": "<style/mood/genre/instruments>",
              "lyrics": "[Verse 1] ...\n[Chorus] ..." | "[Instrumental]",
              "audio_setting": {"sample_rate": 44100, "bitrate": 256000, "format": "mp3"},
              "refer_voice": "<voice_id>"?, "refer_instrumental": "<file_url>"?  # optional, model 2.5+
              "output_directory": "<path>"?   # optional, server writes there
            }``

Response:  ``{"audio": "hex-encoded-mp3-bytes", "audio_format": "mp3",
              "duration_s": 60, "status": "success", "model": "music-3.0",
              "trace_id": "..."}``  (shape may evolve — we accept
              ``data``/``audio_url``/``audio``/``data.audio`` variants).

Usage::

    import asyncio
    from rob_box_voice.core.minimax_music_client import MinimaxMusicClient, MinimaxMusicError

    client = MinimaxMusicClient(api_key=os.getenv("MINIMAX_API_KEY"))
    try:
        result = await client.generate(
            prompt="warm romantic ballad, soft piano, C minor, 80bpm",
            lyrics="[Verse]\\nВ каплях дождя...\\n[Chorus]\\nТы со мной...",
        )
        # result.audio_bytes — full MP3 binary
        # result.duration_s   — 60
        Path("/tmp/track.mp3").write_bytes(result.audio_bytes)
    finally:
        await client.close()

For dialog-UX the higher-level ``generate_with_progress`` runs the call
in a background ``asyncio.Task`` and invokes a ``progress_cb`` every
``progress_interval`` seconds so the LLM can say "почти готово, ещё
чуть-чуть..." while the API works.

Reference: https://platform.minimax.io/docs/api-reference/music-generation
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Dict, Optional

import httpx

logger = logging.getLogger(__name__)


# Default rate limit: 1 concurrent generation (Free plan: RPM 3, but we
# keep one in flight to avoid hitting the per-minute cap during normal
# user-driven dialog).  The free plan also has TPM (tokens per minute)
# caps; the conservative default keeps the dialog UX safe.
DEFAULT_MAX_CONCURRENT = 1

# Default polling cadence for the dialog progress callback (seconds).
DEFAULT_PROGRESS_INTERVAL = 12.0

# Hard wall-time cap on a single generation (seconds).  Issue #1358
# measured avg 88s (max 163s in free tier).
DEFAULT_TIMEOUT_S = 180.0

# Models supported by the public API.
MODELS = ("music-3.0", "music-3.0-free", "music-2.5", "music-2.5-free")

# Audio settings — fixed to MP3 44.1kHz/256kbps per issue #1358 spec.
DEFAULT_AUDIO_SETTING: Dict[str, Any] = {
    "sample_rate": 44100,
    "bitrate": 256000,
    "format": "mp3",
}


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------


class MinimaxMusicError(RuntimeError):
    """Raised for any non-recoverable failure of the MiniMax Music API."""

    def __init__(
        self,
        message: str,
        *,
        status_code: Optional[int] = None,
        body: Optional[str] = None,
        retry_after_s: Optional[float] = None,
    ) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.body = body
        self.retry_after_s = retry_after_s


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass
class MusicGenerationResult:
    """Successful response from a generation call."""

    audio_bytes: bytes
    audio_format: str = "mp3"
    duration_s: float = 0.0
    model: str = ""
    trace_id: str = ""
    wall_time_s: float = 0.0
    raw: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


class MinimaxMusicClient:
    """Async client for the MiniMax Music Generation API.

    Holds a single :class:`httpx.AsyncClient` and a semaphore to throttle
    concurrent requests.  Reuse the instance for the lifetime of the
    MusicSkill; close it on shutdown.

    Args:
        api_key:          MiniMax API key.  Default: ``$MINIMAX_API_KEY``.
        base_url:         API base URL (override for testing).
        max_concurrent:   Max simultaneous generations (default 1).
        timeout_s:        Per-request HTTP timeout in seconds (default 180).
        progress_interval: How often to invoke the progress callback in
                          :meth:`generate_with_progress` (default 12s).
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        *,
        base_url: str = "https://api.minimax.io",
        max_concurrent: int = DEFAULT_MAX_CONCURRENT,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        progress_interval: float = DEFAULT_PROGRESS_INTERVAL,
        client: Optional[httpx.AsyncClient] = None,
    ) -> None:
        self.api_key = api_key or os.getenv("MINIMAX_API_KEY") or ""
        if not self.api_key:
            # Don't raise at construction time — the skill is loaded at
            # boot and we don't want a missing key to crash the dialog
            # node.  Surface a clear error at call time instead.
            logger.warning(
                "⚠️ MINIMAX_API_KEY is not set — generate_music will fail "
                "with a clear error until the env var is configured."
            )
        self.base_url = base_url.rstrip("/")
        self.max_concurrent = max(1, int(max_concurrent))
        self.timeout_s = float(timeout_s)
        self.progress_interval = float(progress_interval)
        self._client = client  # injected by tests
        self._owns_client = client is None
        self._sem = asyncio.Semaphore(self.max_concurrent)

    async def _ensure_client(self) -> httpx.AsyncClient:
        if self._client is None:
            self._client = httpx.AsyncClient(
                base_url=self.base_url,
                timeout=self.timeout_s,
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        return self._client

    async def close(self) -> None:
        if self._owns_client and self._client is not None:
            try:
                await self._client.aclose()
            except Exception:  # noqa: BLE001
                pass
            self._client = None

    # ------------------------------------------------------------------
    # Public API: generate
    # ------------------------------------------------------------------

    async def generate(
        self,
        prompt: str,
        lyrics: str = "[Instrumental]",
        *,
        model: str = "music-3.0",
        audio_setting: Optional[Dict[str, Any]] = None,
        refer_voice: Optional[str] = None,
        refer_instrumental: Optional[str] = None,
        **extra: Any,
    ) -> MusicGenerationResult:
        """Call the API synchronously and return the audio bytes.

        Acquires the global semaphore first so two concurrent calls are
        serialized.  Raises :class:`MinimaxMusicError` on any failure.
        """
        if not self.api_key:
            raise MinimaxMusicError(
                "MINIMAX_API_KEY is not set.  "
                "Set the env var on the voice-assistant container."
            )

        if model not in MODELS:
            raise MinimaxMusicError(
                f"Unknown model {model!r}. Supported: {', '.join(MODELS)}"
            )

        if not prompt or not prompt.strip():
            raise MinimaxMusicError("`prompt` is required and cannot be empty.")

        payload: Dict[str, Any] = {
            "model": model,
            "prompt": prompt.strip(),
            "lyrics": lyrics or "[Instrumental]",
            "audio_setting": audio_setting or DEFAULT_AUDIO_SETTING,
        }
        if refer_voice:
            payload["refer_voice"] = refer_voice
        if refer_instrumental:
            payload["refer_instrumental"] = refer_instrumental
        payload.update(extra)

        async with self._sem:
            client = await self._ensure_client()
            t0 = time.monotonic()
            try:
                resp = await client.post(
                    "/v1/music_generation",
                    json=payload,
                )
            except httpx.TimeoutException as exc:
                raise MinimaxMusicError(
                    f"MiniMax Music API timed out after {self.timeout_s:.0f}s",
                ) from exc
            except httpx.HTTPError as exc:
                raise MinimaxMusicError(f"HTTP error: {exc}") from exc

            wall = time.monotonic() - t0

            if resp.status_code == 429:
                # Rate limit
                retry_after = resp.headers.get("Retry-After")
                raise MinimaxMusicError(
                    "MiniMax Music API rate-limited (429).  "
                    "Wait a minute or switch to a paid model.",
                    status_code=429,
                    body=resp.text[:500],
                    retry_after_s=float(retry_after) if retry_after else 60.0,
                )
            if resp.status_code >= 400:
                raise MinimaxMusicError(
                    f"MiniMax Music API error {resp.status_code}",
                    status_code=resp.status_code,
                    body=resp.text[:1000],
                )

            # Parse body.  Some MiniMax responses use {data: {...}}; some
            # return the audio inline; some return a URL.  Accept all
            # three shapes.
            try:
                body = resp.json()
            except json.JSONDecodeError as exc:
                raise MinimaxMusicError(
                    "MiniMax returned non-JSON body",
                    status_code=resp.status_code,
                    body=resp.text[:500],
                ) from exc

            audio_bytes, audio_meta = self._extract_audio(body)
            if not audio_bytes:
                # Some MiniMax plans return an `audio_url` instead of
                # inline bytes — download it.
                audio_url = (
                    (body.get("data") or {}).get("audio_url")
                    or body.get("audio_url")
                )
                if audio_url:
                    try:
                        r2 = await client.get(audio_url)
                        r2.raise_for_status()
                        audio_bytes = r2.content
                        audio_meta["audio_format"] = (
                            audio_meta.get("audio_format")
                            or (r2.headers.get("content-type", "").split("/")[-1] or "mp3")
                        )
                    except httpx.HTTPError as exc:
                        raise MinimaxMusicError(
                            f"Failed to download audio_url: {exc}"
                        ) from exc
            if not audio_bytes:
                raise MinimaxMusicError(
                    "MiniMax response has no audio data",
                    status_code=resp.status_code,
                    body=json.dumps(body)[:500],
                )

            return MusicGenerationResult(
                audio_bytes=audio_bytes,
                audio_format=audio_meta.get("audio_format", "mp3"),
                duration_s=float(audio_meta.get("duration_s") or 0.0),
                model=audio_meta.get("model", model),
                trace_id=audio_meta.get("trace_id", ""),
                wall_time_s=wall,
                raw=body,
            )

    # ------------------------------------------------------------------
    # Dialog-UX variant: generate with progress callback
    # ------------------------------------------------------------------

    async def generate_with_progress(
        self,
        prompt: str,
        lyrics: str = "[Instrumental]",
        *,
        progress_cb: Optional[Callable[[Dict[str, Any]], Awaitable[None]]] = None,
        cancel_event: Optional[asyncio.Event] = None,
        **kwargs: Any,
    ) -> MusicGenerationResult:
        """Call ``generate`` and emit progress updates while waiting.

        ``progress_cb`` is an async callable that receives
        ``{"elapsed_s": float, "status": "generating", "hint": "..."}``.
        The callback is invoked every ``progress_interval`` seconds.
        ``cancel_event`` lets the caller abort early (e.g. user said
        "стоп, не надо").  The call still raises :class:`MinimaxMusicError`
        on failure.
        """
        async def _ticker() -> None:
            start = time.monotonic()
            while True:
                await asyncio.sleep(self.progress_interval)
                if cancel_event is not None and cancel_event.is_set():
                    return
                if progress_cb is None:
                    continue
                elapsed = time.monotonic() - start
                try:
                    await progress_cb({
                        "elapsed_s": elapsed,
                        "status": "generating",
                        "hint": self._progress_hint(elapsed),
                    })
                except Exception as exc:  # noqa: BLE001
                    logger.debug("progress_cb raised: %s", exc)

        ticker_task: Optional[asyncio.Task] = None
        if progress_cb is not None:
            ticker_task = asyncio.create_task(_ticker(), name="mm-music-progress")
        try:
            result = await self.generate(prompt=prompt, lyrics=lyrics, **kwargs)
        finally:
            if ticker_task is not None:
                ticker_task.cancel()
                try:
                    await ticker_task
                except (asyncio.CancelledError, Exception):  # noqa: BLE001
                    pass
        return result

    @staticmethod
    def _progress_hint(elapsed: float) -> str:
        """One short Russian hint based on elapsed seconds (dialog UX)."""
        if elapsed < 20:
            return "Подбираю стиль..."
        if elapsed < 50:
            return "Пишу мелодию..."
        if elapsed < 90:
            return "Почти готово, ещё чуть-чуть..."
        if elapsed < 130:
            return "Финальная обработка..."
        return "Долго, но продолжаю ждать..."

    # ------------------------------------------------------------------
    # Audio extraction helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_audio(body: Dict[str, Any]) -> tuple[bytes, Dict[str, Any]]:
        """Find the audio bytes + metadata in a MiniMax response body.

        Accepts several documented and observed response shapes:
          - ``{audio: "<hex>", audio_format: "mp3", duration_s: 60}``
          - ``{data: {audio: "<hex>", ...}}``
          - ``{data: {audio: "<base64>", ...}}``
        Returns ``(bytes, meta_dict)``.  ``bytes`` is empty when the
        response uses ``audio_url`` instead (caller downloads it).
        """
        meta: Dict[str, Any] = {}

        # Flat shape
        if isinstance(body.get("audio"), str):
            audio_field = body["audio"]
            meta["audio_format"] = body.get("audio_format", "mp3")
            meta["duration_s"] = body.get("duration_s", 0.0)
            meta["model"] = body.get("model", "")
            meta["trace_id"] = body.get("trace_id", "")
            return _decode_audio_field(audio_field), meta

        # data.* shape
        data = body.get("data")
        if isinstance(data, dict):
            if isinstance(data.get("audio"), str):
                meta["audio_format"] = data.get("audio_format", "mp3")
                meta["duration_s"] = data.get("duration_s", 0.0)
                meta["model"] = data.get("model", "")
                meta["trace_id"] = data.get("trace_id", "")
                return _decode_audio_field(data["audio"]), meta
            if isinstance(data.get("audio_url"), str):
                meta["audio_format"] = data.get("audio_format", "mp3")
                meta["duration_s"] = data.get("duration_s", 0.0)
                return b"", meta

        return b"", meta


def _decode_audio_field(field: str) -> bytes:
    """Decode either hex (preferred) or base64 audio field."""
    if not field:
        return b""
    # Heuristic: hex strings have even length and only [0-9a-fA-F]
    if len(field) % 2 == 0 and all(c in "0123456789abcdefABCDEF" for c in field[:64]):
        try:
            return bytes.fromhex(field)
        except ValueError:
            pass
    # Fallback: base64
    import base64
    try:
        return base64.b64decode(field, validate=False)
    except Exception:  # noqa: BLE001
        return b""


__all__ = [
    "MinimaxMusicClient",
    "MinimaxMusicError",
    "MusicGenerationResult",
    "MODELS",
    "DEFAULT_AUDIO_SETTING",
    "DEFAULT_PROGRESS_INTERVAL",
    "DEFAULT_TIMEOUT_S",
    "DEFAULT_MAX_CONCURRENT",
]
