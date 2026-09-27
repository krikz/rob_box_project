"""Unit tests for :mod:`rob_box_voice.core.minimax_music_client`."""

from __future__ import annotations

import asyncio
import base64
import json
from typing import Any

import httpx
import pytest

from rob_box_voice.core.minimax_music_client import (
    DEFAULT_AUDIO_SETTING,
    DEFAULT_PROGRESS_INTERVAL,
    DEFAULT_TIMEOUT_S,
    MODELS,
    MinimaxMusicClient,
    MinimaxMusicError,
    MusicGenerationResult,
    _decode_audio_field,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _response(
    status: int,
    body: dict[str, Any],
    *,
    headers: dict[str, str] | None = None,
) -> httpx.Response:
    """Build a real httpx.Response for MockTransport (so .stream works)."""
    req = httpx.Request("POST", "https://api.minimax.io/v1/music_generation")
    return httpx.Response(status, json=body, headers=headers or {}, request=req)


def _audio_hex(payload: bytes) -> str:
    return payload.hex()


def _client_with_mock(
    handler, *, api_key: str = "test-key", **kwargs
) -> tuple[MinimaxMusicClient, httpx.AsyncClient]:
    transport = httpx.MockTransport(handler)
    http = httpx.AsyncClient(transport=transport, base_url="https://api.minimax.io")
    return (
        MinimaxMusicClient(api_key=api_key, client=http, **kwargs),
        http,
    )


def _run(coro):
    """Run an async coroutine inside a sync pytest test method."""
    return asyncio.get_event_loop().run_until_complete(coro)  # type: ignore[deprecated]


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


class TestConstruction:
    def test_warns_when_no_api_key(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
        client = MinimaxMusicClient(api_key=None)
        assert client.api_key == ""

    def test_uses_env_when_api_key_arg_is_none(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setenv("MINIMAX_API_KEY", "from-env")
        c = MinimaxMusicClient(api_key=None)
        assert c.api_key == "from-env"

    def test_custom_base_url_and_timeout(self) -> None:
        c = MinimaxMusicClient(
            api_key="k",
            base_url="https://example.test/",
            timeout_s=42.0,
        )
        assert c.base_url == "https://example.test"
        assert c.timeout_s == 42.0


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


class TestValidation:
    def test_empty_key_raises(self) -> None:
        c = MinimaxMusicClient(api_key="")

        async def go():
            with pytest.raises(MinimaxMusicError, match="MINIMAX_API_KEY"):
                await c.generate(prompt="x")
            await c.close()

        _run(go())

    def test_unknown_model_raises(self) -> None:
        c = MinimaxMusicClient(api_key="k")

        async def go():
            with pytest.raises(MinimaxMusicError, match="Unknown model"):
                await c.generate(prompt="x", model="gpt-99")
            await c.close()

        _run(go())

    def test_empty_prompt_raises(self) -> None:
        c = MinimaxMusicClient(api_key="k")

        async def go():
            with pytest.raises(MinimaxMusicError, match="`prompt` is required"):
                await c.generate(prompt="   ")
            await c.close()

        _run(go())


# ---------------------------------------------------------------------------
# Happy paths
# ---------------------------------------------------------------------------


class TestGenerate:
    def test_hex_audio_inline(self) -> None:
        audio = b"fake mp3 binary 1234"

        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(
                200,
                {
                    "audio": _audio_hex(audio),
                    "audio_format": "mp3",
                    "duration_s": 60.0,
                    "model": "music-3.0",
                    "trace_id": "abc",
                },
            )

        c, http = _client_with_mock(handler)

        async def go():
            return await c.generate(
                prompt="warm romantic ballad", lyrics="[Verse]\nHello"
            )

        try:
            result = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())

        assert isinstance(result, MusicGenerationResult)
        assert result.audio_bytes == audio
        assert result.audio_format == "mp3"
        assert result.duration_s == 60.0
        assert result.model == "music-3.0"
        assert result.trace_id == "abc"
        assert result.wall_time_s >= 0

    def test_base64_audio_inline(self) -> None:
        audio = b"binary base64 payload"

        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(
                200,
                {
                    "data": {
                        "audio": base64.b64encode(audio).decode(),
                        "audio_format": "mp3",
                        "duration_s": 30.0,
                    }
                },
            )

        c, http = _client_with_mock(handler)

        async def go():
            return await c.generate(prompt="x")

        try:
            result = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert result.audio_bytes == audio

    def test_audio_url_triggers_download(self) -> None:
        audio = b"downloaded binary"
        download_calls: list[str] = []

        async def handler(req: httpx.Request) -> httpx.Response:
            if "/track.mp3" in str(req.url):
                download_calls.append(str(req.url))
                return httpx.Response(
                    200,
                    content=audio,
                    headers={"content-type": "audio/mpeg"},
                    request=req,
                )
            return _response(
                200,
                {
                    "data": {
                        "audio_url": "https://cdn.example.test/track.mp3",
                        "duration_s": 30.0,
                    }
                },
            )

        c, http = _client_with_mock(handler)

        async def go():
            return await c.generate(prompt="x")

        try:
            result = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert result.audio_bytes == audio
        # We normalize "audio/mpeg" → "mp3" for the audio_format field
        assert result.audio_format in ("mp3", "mpeg")
        assert download_calls, "expected download of audio_url"

    def test_no_audio_anywhere_raises(self) -> None:
        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(200, {"data": {"status": "ok"}})

        c, http = _client_with_mock(handler)

        async def go():
            with pytest.raises(MinimaxMusicError, match="no audio data"):
                await c.generate(prompt="x")

        try:
            _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())

    def test_non_json_response_raises(self) -> None:
        async def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(
                200, content=b"not json", request=req
            )

        c, http = _client_with_mock(handler)

        async def go():
            with pytest.raises(MinimaxMusicError, match="non-JSON"):
                await c.generate(prompt="x")

        try:
            _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())

    def test_request_payload_uses_required_fields(self) -> None:
        seen: list[dict[str, Any]] = []

        async def handler(req: httpx.Request) -> httpx.Response:
            seen.append(json.loads(req.content))
            return _response(
                200,
                {
                    "audio": _audio_hex(b"ok"),
                    "audio_format": "mp3",
                    "duration_s": 10.0,
                },
            )

        c, http = _client_with_mock(handler)

        async def go():
            await c.generate(
                prompt="romantic ballad",
                lyrics="[Verse]\nhello",
                model="music-3.0",
            )

        try:
            _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())

        assert len(seen) == 1
        payload = seen[0]
        assert payload["model"] == "music-3.0"
        assert payload["prompt"] == "romantic ballad"
        assert payload["lyrics"] == "[Verse]\nhello"
        assert payload["audio_setting"]["format"] == "mp3"
        assert payload["audio_setting"]["sample_rate"] == 44100
        assert payload["audio_setting"]["bitrate"] == 256000

    def test_default_lyrics_is_instrumental(self) -> None:
        seen: list[dict[str, Any]] = []

        async def handler(req: httpx.Request) -> httpx.Response:
            seen.append(json.loads(req.content))
            return _response(
                200,
                {"audio": _audio_hex(b"x"), "audio_format": "mp3", "duration_s": 5.0},
            )

        c, http = _client_with_mock(handler)

        async def go():
            await c.generate(prompt="x")  # no lyrics

        try:
            _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert seen[0]["lyrics"] == "[Instrumental]"


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------


class TestErrorPaths:
    def test_429_carries_retry_after(self) -> None:
        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(
                429,
                {"error": "rate limited"},
                headers={"Retry-After": "60"},
            )

        c, http = _client_with_mock(handler)

        async def go():
            with pytest.raises(MinimaxMusicError) as ei:
                await c.generate(prompt="x")
            return ei.value

        try:
            exc = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert exc.status_code == 429
        assert exc.retry_after_s == 60.0

    def test_500_includes_body(self) -> None:
        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(500, {"error": "boom"})

        c, http = _client_with_mock(handler)

        async def go():
            with pytest.raises(MinimaxMusicError) as ei:
                await c.generate(prompt="x")
            return ei.value

        try:
            exc = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert exc.status_code == 500
        assert "boom" in (exc.body or "")


# ---------------------------------------------------------------------------
# Progress callback
# ---------------------------------------------------------------------------


class TestProgress:
    def test_progress_callback_does_not_crash(self) -> None:
        seen: list[dict[str, Any]] = []

        async def cb(payload: dict[str, Any]) -> None:
            seen.append(payload)

        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(
                200,
                {
                    "audio": _audio_hex(b"x"),
                    "audio_format": "mp3",
                    "duration_s": 10.0,
                },
            )

        c, http = _client_with_mock(handler, progress_interval=0.001)

        async def go():
            return await c.generate_with_progress(prompt="x", progress_cb=cb)

        try:
            result = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert result.audio_bytes == b"x"
        for call in seen:
            assert "elapsed_s" in call
            assert call["status"] == "generating"
            assert isinstance(call["hint"], str)

    def test_no_progress_callback_runs_fine(self) -> None:
        async def handler(req: httpx.Request) -> httpx.Response:
            return _response(
                200,
                {"audio": _audio_hex(b"y"), "audio_format": "mp3", "duration_s": 1.0},
            )

        c, http = _client_with_mock(handler)

        async def go():
            return await c.generate_with_progress(prompt="x", progress_cb=None)

        try:
            result = _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert result.audio_bytes == b"y"


# ---------------------------------------------------------------------------
# Semaphore / concurrency
# ---------------------------------------------------------------------------


class TestConcurrency:
    def test_serializes_concurrent_calls(self) -> None:
        in_flight = 0
        max_in_flight = 0

        async def handler(req: httpx.Request) -> httpx.Response:
            nonlocal in_flight, max_in_flight
            in_flight += 1
            max_in_flight = max(max_in_flight, in_flight)
            await asyncio.sleep(0.05)
            in_flight -= 1
            return _response(
                200,
                {"audio": _audio_hex(b"x"), "audio_format": "mp3", "duration_s": 1.0},
            )

        c, http = _client_with_mock(handler)

        async def go():
            await asyncio.gather(
                c.generate(prompt="a"),
                c.generate(prompt="b"),
            )

        try:
            _run(go())
        finally:
            _run(c.close())
            _run(http.aclose())
        assert max_in_flight == 1, "semaphore should serialize calls"


# ---------------------------------------------------------------------------
# Audio field decoder
# ---------------------------------------------------------------------------


class TestDecodeAudioField:
    def test_hex_decode(self) -> None:
        assert _decode_audio_field(b"hello".hex()) == b"hello"

    def test_base64_decode(self) -> None:
        b64 = base64.b64encode(b"hello world").decode()
        assert _decode_audio_field(b64) == b"hello world"

    def test_empty(self) -> None:
        assert _decode_audio_field("") == b""


# ---------------------------------------------------------------------------
# Progress hint
# ---------------------------------------------------------------------------


class TestProgressHint:
    def test_hints_progress_through_stages(self) -> None:
        h1 = MinimaxMusicClient._progress_hint(5)
        h2 = MinimaxMusicClient._progress_hint(30)
        h3 = MinimaxMusicClient._progress_hint(60)
        h4 = MinimaxMusicClient._progress_hint(100)
        h5 = MinimaxMusicClient._progress_hint(140)
        assert isinstance(h1, str) and h1
        assert isinstance(h2, str) and h2
        assert isinstance(h3, str) and h3
        assert isinstance(h4, str) and h4
        assert isinstance(h5, str) and h5
        assert len({h1, h2, h3, h4, h5}) == 5


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


class TestConstants:
    def test_models_includes_free_and_paid(self) -> None:
        assert "music-3.0" in MODELS
        assert "music-3.0-free" in MODELS

    def test_default_audio_setting(self) -> None:
        assert DEFAULT_AUDIO_SETTING["format"] == "mp3"
        assert DEFAULT_AUDIO_SETTING["sample_rate"] == 44100
        assert DEFAULT_AUDIO_SETTING["bitrate"] == 256000

    def test_progress_interval_default(self) -> None:
        assert DEFAULT_PROGRESS_INTERVAL == 12.0

    def test_timeout_default(self) -> None:
        assert DEFAULT_TIMEOUT_S == 180.0
