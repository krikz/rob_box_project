"""Requirement-focused tests for MiniMax TTS reliability and byte API."""

from __future__ import annotations

import asyncio
import io
import json
import wave
from typing import Any, cast

import httpx
import pytest

from rob_box_llm.errors import TTSAuthError, TTSError
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider


@pytest.fixture
def ok_envelope() -> dict[str, object]:
    return {
        "data": {
            "audio": b"\x00\x01\x02\x03".hex(),
            "audio_sample_rate": 24_000,
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _client(handler: Any) -> httpx.AsyncClient:
    return httpx.AsyncClient(transport=httpx.MockTransport(handler))


def test_base_url_env_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MINIMAX_TTS_BASE_URL", "https://tts.example.test/")

    provider = MiniMaxTTSProvider(api_key="key", group_id="group")

    assert provider._base_url == "https://tts.example.test"


@pytest.mark.asyncio
async def test_bytes_api_maps_format_and_voice(ok_envelope: dict[str, object]) -> None:
    requests: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        return httpx.Response(200, json=ok_envelope)

    provider = MiniMaxTTSProvider(api_key="key", group_id="group", client=_client(handler))

    result = await provider.synthesize_bytes(
        "hello",
        voice="Calm_Woman",
        format="pcm_24000",
        model="speech-02-turbo",
        speed=1.25,
    )

    body = json.loads(requests[0].content)
    assert result == b"\x00\x01\x02\x03"
    assert body["model"] == "speech-02-turbo"
    assert body["voice_setting"]["voice_id"] == "Calm_Woman"
    assert body["voice_setting"]["speed"] == 1.25
    assert body["audio_setting"]["format"] == "pcm"
    assert body["audio_setting"]["sample_rate"] == 24_000


@pytest.mark.asyncio
async def test_wav_format_wraps_raw_pcm_in_riff_container() -> None:
    raw_pcm = b"\x00\x01\x02\x03"

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            json={
                "data": {
                    "audio": raw_pcm.hex(),
                    "audio_sample_rate": 24_000,
                },
                "base_resp": {"status_code": 0, "status_msg": "success"},
            },
        )

    provider = MiniMaxTTSProvider(api_key="key", group_id="group", client=_client(handler))

    result = await provider.synthesize_bytes("hello", format="wav")

    assert result[:4] == b"RIFF"
    assert result[8:12] == b"WAVE"
    with wave.open(io.BytesIO(result), "rb") as wav_file:
        assert wav_file.getframerate() == 24_000
        assert wav_file.getnchannels() == 1
        assert wav_file.getsampwidth() == 2
        assert wav_file.readframes(wav_file.getnframes()) == raw_pcm


@pytest.mark.asyncio
async def test_wav_format_rejects_incomplete_int16_sample() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            json={
                "data": {"audio": b"\x00".hex(), "audio_sample_rate": 24_000},
                "base_resp": {"status_code": 0, "status_msg": "success"},
            },
        )

    provider = MiniMaxTTSProvider(
        api_key="key",
        group_id="group",
        client=_client(handler),
        max_attempts=1,
    )

    with pytest.raises(TTSError, match="16-bit"):
        await provider.synthesize_bytes("hello", format="wav")


@pytest.mark.asyncio
async def test_concurrent_clear_cache_is_serialized(
    ok_envelope: dict[str, object],
) -> None:
    provider = MiniMaxTTSProvider(
        api_key="key",
        group_id="group",
        client=_client(lambda request: httpx.Response(200, json=ok_envelope)),
    )
    await provider.synthesize_bytes("hello")
    lock = cast(Any, provider._cache_lock)
    lock_acquired = asyncio.Event()
    release = asyncio.Event()

    async def hold_cache_lock() -> None:
        async with lock:
            lock_acquired.set()
            await release.wait()

    holder = asyncio.create_task(hold_cache_lock())
    await lock_acquired.wait()
    clear_task = asyncio.create_task(provider.clear_cache())
    await asyncio.sleep(0)

    assert not clear_task.done()
    release.set()
    await asyncio.gather(holder, clear_task)


@pytest.mark.asyncio
async def test_clear_cache_during_inflight_prevents_stale_repopulation(
    ok_envelope: dict[str, object],
) -> None:
    request_count = 0
    request_started = asyncio.Event()
    release = asyncio.Event()

    async def handler(request: httpx.Request) -> httpx.Response:
        nonlocal request_count
        request_count += 1
        request_started.set()
        await release.wait()
        return httpx.Response(200, json=ok_envelope)

    provider = MiniMaxTTSProvider(api_key="key", group_id="group", client=_client(handler))
    first = asyncio.create_task(provider.synthesize_bytes("hello"))
    await request_started.wait()
    await provider.clear_cache()
    release.set()
    await first

    await provider.synthesize_bytes("hello")

    assert request_count == 2


@pytest.mark.asyncio
async def test_cache_avoids_duplicate_http_request(ok_envelope: dict[str, object]) -> None:
    request_count = 0

    def handler(request: httpx.Request) -> httpx.Response:
        nonlocal request_count
        request_count += 1
        return httpx.Response(200, json=ok_envelope)

    provider = MiniMaxTTSProvider(api_key="key", group_id="group", client=_client(handler))

    first = await provider.synthesize_bytes("hello", voice="Calm_Woman", format="wav")
    second = await provider.synthesize_bytes("hello", voice="Calm_Woman", format="wav")

    assert first == second
    assert request_count == 1


@pytest.mark.asyncio
async def test_retries_429_then_succeeds_without_real_sleep(
    ok_envelope: dict[str, object],
) -> None:
    request_count = 0
    delays: list[float] = []

    def handler(request: httpx.Request) -> httpx.Response:
        nonlocal request_count
        request_count += 1
        if request_count < 3:
            return httpx.Response(429, text="slow down")
        return httpx.Response(200, json=ok_envelope)

    async def fake_sleep(delay: float) -> None:
        delays.append(delay)

    provider = MiniMaxTTSProvider(
        api_key="key",
        group_id="group",
        client=_client(handler),
        retry_base_delay=0.5,
        retry_jitter=0.0,
        sleep=fake_sleep,
    )

    result = await provider.synthesize_bytes("hello", format="pcm_24000")

    assert result == b"\x00\x01\x02\x03"
    assert request_count == 3
    assert delays == [0.5, 1.0]


@pytest.mark.asyncio
async def test_non_retryable_401_uses_one_attempt() -> None:
    request_count = 0

    def handler(request: httpx.Request) -> httpx.Response:
        nonlocal request_count
        request_count += 1
        return httpx.Response(401, text="bad key")

    provider = MiniMaxTTSProvider(api_key="key", group_id="group", client=_client(handler))

    with pytest.raises(TTSAuthError):
        await provider.synthesize_bytes("hello")

    assert request_count == 1


@pytest.mark.asyncio
async def test_concurrency_limit_serializes_requests(
    ok_envelope: dict[str, object],
) -> None:
    active = 0
    max_active = 0
    release = asyncio.Event()
    both_started = asyncio.Event()

    async def handler(request: httpx.Request) -> httpx.Response:
        nonlocal active, max_active
        active += 1
        max_active = max(max_active, active)
        if active == 2:
            both_started.set()
        try:
            await release.wait()
            return httpx.Response(200, json=ok_envelope)
        finally:
            active -= 1

    provider = MiniMaxTTSProvider(
        api_key="key",
        group_id="group",
        client=_client(handler),
        max_concurrency=1,
    )

    first = asyncio.create_task(provider.synthesize_bytes("one"))
    second = asyncio.create_task(provider.synthesize_bytes("two"))
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    assert not both_started.is_set()
    release.set()
    await asyncio.gather(first, second)

    assert max_active == 1
