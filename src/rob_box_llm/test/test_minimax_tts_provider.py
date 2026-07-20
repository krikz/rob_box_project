"""Tests for :class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`.

We never hit the network. :mod:`httpx` ships with a ``MockTransport`` that
lets us feed canned request/response pairs to an ``AsyncClient``. That
exercises the provider's full HTTP plumbing — URL, query params, headers,
JSON body, response decoding — without a single socket.

What we cover:

* payload shape (model, voice, language mapping, audio_setting defaults)
* hex-decoded PCM bytes come back as ``TTSAudio.samples``
* ``base_resp.status_code != 0`` raises a typed ``TTSError`` subclass
* HTTP 4xx/5xx → mapped exception (``TTSAuthError`` / ``TTSRateLimitError``
  / ``TTSBadRequestError``)
* httpx ``TimeoutException`` → ``TTSTimeoutError``
* empty text → ``TTSBadRequestError``
* missing API key / group id → ``TTSAuthError``
* ``stream()`` collects SSE chunks into a single ``TTSChunk`` and signals
  end-of-stream with ``finish_reason="stop"``
* ``aclose()`` closes an owned client but NOT an injected one
"""

from __future__ import annotations

import json
from typing import Any

import httpx
import pytest

from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.providers.minimax_tts import (
    MiniMaxTTSProvider,
    _build_payload,
    _map_language,
)
from rob_box_llm.tts import TTSFormat, TTSSettings

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _mock_client(handler) -> httpx.AsyncClient:
    """Build an httpx.AsyncClient whose transport is replaced by ``handler``.

    ``handler`` is a callable ``(httpx.Request) -> httpx.Response``.
    """
    transport = httpx.MockTransport(handler)
    return httpx.AsyncClient(transport=transport, base_url="https://api.MiniMax.io")


def _ok_response(audio_bytes: bytes, sample_rate: int = 32_000) -> dict[str, Any]:
    """Build a MiniMax T2A v2 success envelope."""
    return {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": sample_rate,
            "audio_length": len(audio_bytes),
        },
        "extra_info": None,
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _error_response(status_code: int, status_msg: str) -> dict[str, Any]:
    return {
        "data": None,
        "base_resp": {"status_code": status_code, "status_msg": status_msg},
    }


# ---------------------------------------------------------------------------
# Payload / language mapping (pure functions, no I/O)
# ---------------------------------------------------------------------------


class TestBuildPayload:
    def test_minimal_payload_uses_defaults(self):
        s = TTSSettings()
        payload = _build_payload("hi", s, stream=False, default_voice="default-voice", default_model="default-model")
        assert payload["text"] == "hi"
        assert payload["model"] == "default-model"
        assert payload["stream"] is False
        assert payload["voice_setting"]["voice_id"] == "default-voice"
        assert "speed" not in payload["voice_setting"]
        assert payload["audio_setting"]["sample_rate"] == 32_000  # provider default
        assert payload["audio_setting"]["format"] == "pcm"
        assert payload["audio_setting"]["channel"] == 1

    def test_full_payload_maps_language_and_emotion(self):
        s = TTSSettings(
            voice="Calm_Woman",
            model="speech-02-turbo",
            language="ru",
            speed=1.2,
            volume=2.0,
            pitch=2,
            emotion="happy",
            sample_rate=24_000,
            format=TTSFormat.MP3,
            text_normalization=True,
            extra={"custom": "value"},
        )
        payload = _build_payload("privet", s, stream=True, default_voice="X", default_model="Y")
        assert payload["model"] == "speech-02-turbo"
        assert payload["stream"] is True
        vs = payload["voice_setting"]
        assert vs["voice_id"] == "Calm_Woman"
        assert vs["speed"] == 1.2
        assert vs["vol"] == 2.0
        assert vs["pitch"] == 2
        assert vs["emotion"] == "happy"
        assert vs["language"] == "Russian"  # "ru" → "Russian"
        assert payload["audio_setting"]["sample_rate"] == 24_000
        assert payload["audio_setting"]["format"] == "mp3"
        assert payload["text_normalization"] is True
        assert payload["custom"] == "value"

    def test_ogg_format_falls_back_to_mp3(self):
        # MiniMax doesn't support OGG in audio_setting; we degrade to MP3
        # and surface the chosen format on the returned TTSAudio.
        s = TTSSettings(format=TTSFormat.OGG)
        payload = _build_payload("x", s, stream=False, default_voice="v", default_model="m")
        assert payload["audio_setting"]["format"] == "mp3"


class TestMapLanguage:
    @pytest.mark.parametrize(
        "code,expected",
        [
            ("ru", "Russian"),
            ("RU", "Russian"),
            ("en", "English"),
            ("zh", "Chinese"),
            ("ja", "Japanese"),
            ("ko", "Korean"),
        ],
    )
    def test_iso_code_mapped(self, code, expected):
        assert _map_language(code) == expected

    @pytest.mark.parametrize("full", ["Russian", "English", "Chinese Mandarin"])
    def test_full_name_passes_through(self, full):
        assert _map_language(full) == full

    @pytest.mark.parametrize("empty", [None, "", "   "])
    def test_empty_returns_none(self, empty):
        assert _map_language(empty) is None


# ---------------------------------------------------------------------------
# Constructor
# ---------------------------------------------------------------------------


class TestConstructor:
    def test_name_and_defaults(self):
        p = MiniMaxTTSProvider(api_key="k", group_id="g")
        assert p.name == "minimax"
        assert p._api_key == "k"
        assert p._group_id == "g"
        assert p._base_url == "https://api.MiniMax.io"
        assert p._default_voice == "male-qn-qingse"
        assert p._default_model == "speech-02-hd"

    def test_env_fallback(self, monkeypatch):
        monkeypatch.setenv("MINIMAX_API_KEY", "env-key")
        monkeypatch.setenv("MINIMAX_GROUP_ID", "env-group")
        p = MiniMaxTTSProvider()
        assert p._api_key == "env-key"
        assert p._group_id == "env-group"

    def test_custom_base_url_strips_trailing_slash(self):
        p = MiniMaxTTSProvider(api_key="k", group_id="g", base_url="https://x.example.com/")
        assert p._base_url == "https://x.example.com"

    def test_missing_api_key_raises_on_headers(self):
        # Construction is permissive (defer to runtime) but _headers() must fail.
        p = MiniMaxTTSProvider(api_key="", group_id="g")
        with pytest.raises(TTSAuthError):
            p._headers()

    def test_missing_group_id_raises_on_params(self):
        p = MiniMaxTTSProvider(api_key="k", group_id="")
        with pytest.raises(TTSAuthError):
            p._params()


# ---------------------------------------------------------------------------
# synthesize()
# ---------------------------------------------------------------------------


class TestSynthesize:
    @pytest.mark.asyncio
    async def test_returns_decoded_pcm(self):
        fake_pcm = b"\x00\x01\x02\x03\x04\x05" * 100  # 600 bytes
        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(200, json=_ok_response(fake_pcm, sample_rate=24_000))

        client = _mock_client(handler)
        p = MiniMaxTTSProvider(api_key="k", group_id="g", default_voice="V", default_model="M", client=client)

        out = await p.synthesize("hello", settings=TTSSettings(sample_rate=24_000))
        assert out.samples == fake_pcm
        assert out.sample_rate == 24_000
        assert out.format == TTSFormat.PCM
        assert out.raw is not None

        # Verify the request shape.
        assert len(captured) == 1
        req = captured[0]
        assert req.method == "POST"
        # httpx normalises the host to lowercase and URL-encodes the path.
        assert str(req.url).lower().startswith("https://api.minimax.io/v1/t2a_v2")
        assert req.url.params["GroupId"] == "g"
        assert req.headers["Authorization"] == "Bearer k"
        assert req.headers["Content-Type"] == "application/json"
        body = json.loads(req.content)
        assert body["text"] == "hello"
        assert body["model"] == "M"
        assert body["voice_setting"]["voice_id"] == "V"
        assert body["audio_setting"]["sample_rate"] == 24_000
        assert body["stream"] is False

    @pytest.mark.asyncio
    async def test_empty_text_raises_bad_request(self):
        client = _mock_client(lambda req: httpx.Response(200, json=_ok_response(b"")))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("   ")

    @pytest.mark.asyncio
    async def test_missing_credentials_raises_auth(self):
        # Even before we hit HTTP — _headers() is checked first inside _post.
        client = _mock_client(lambda req: httpx.Response(200, json=_ok_response(b"")))
        p = MiniMaxTTSProvider(api_key="", group_id="g", client=client)
        with pytest.raises(TTSAuthError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_http_401_raises_auth_error(self):
        def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(401, text="bad key")

        client = _mock_client(handler)
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSAuthError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_http_429_raises_rate_limit(self):
        client = _mock_client(lambda req: httpx.Response(429, text="quota"))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSRateLimitError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_http_400_raises_bad_request(self):
        client = _mock_client(lambda req: httpx.Response(400, text="bad voice"))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_http_500_raises_generic_error(self):
        client = _mock_client(lambda req: httpx.Response(500, text="boom"))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSError) as exc:
            await p.synthesize("hi")
        assert exc.value.provider == "minimax"

    @pytest.mark.asyncio
    async def test_timeout_raises_tts_timeout(self):
        def handler(req: httpx.Request) -> httpx.Response:
            raise httpx.ReadTimeout("slow")

        client = _mock_client(handler)
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSTimeoutError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_api_level_error_message_mapped(self):
        # base_resp.status_code != 0 → TTSError. Status_msg drives the category.
        client = _mock_client(lambda req: httpx.Response(200, json=_error_response(1002, "invalid voice_id")))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_api_level_quota_error_mapped(self):
        client = _mock_client(lambda req: httpx.Response(200, json=_error_response(1003, "rate limit exceeded")))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSRateLimitError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_api_level_auth_error_mapped(self):
        client = _mock_client(lambda req: httpx.Response(200, json=_error_response(1001, "invalid api key")))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSAuthError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_non_json_response_raises_error(self):
        client = _mock_client(lambda req: httpx.Response(200, text="<html>not json</html>"))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSError):
            await p.synthesize("hi")

    @pytest.mark.asyncio
    async def test_missing_audio_field_raises_error(self):
        # base_resp says success but data.audio is absent — provider error.
        client = _mock_client(
            lambda req: httpx.Response(200, json={"data": {}, "base_resp": {"status_code": 0, "status_msg": "success"}})
        )
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSError):
            await p.synthesize("hi")


# ---------------------------------------------------------------------------
# stream()
# ---------------------------------------------------------------------------


class TestStream:
    @pytest.mark.asyncio
    async def test_sse_chunks_collected_into_single_chunk(self):
        chunk1 = b"\x00\x01" * 50
        chunk2 = b"\x02\x03" * 50
        body = (
            f"data:{json.dumps(_ok_response(chunk1, sample_rate=24_000))}\n\n"
            f"data:{json.dumps(_ok_response(chunk2, sample_rate=24_000))}\n\n"
            "data:[DONE]\n\n"
        )

        async def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(200, text=body)

        client = _mock_client(handler)
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)

        chunks = [c async for c in p.stream("hello", settings=TTSSettings(sample_rate=24_000))]
        assert len(chunks) == 1
        assert chunks[0].samples == chunk1 + chunk2
        assert chunks[0].sample_rate == 24_000
        assert chunks[0].finish_reason == "stop"

    @pytest.mark.asyncio
    async def test_stream_empty_text(self):
        client = _mock_client(lambda req: httpx.Response(200))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSBadRequestError):
            async for _ in p.stream(""):
                pass

    @pytest.mark.asyncio
    async def test_stream_http_error_raises_before_yield(self):
        # 4xx in streaming — provider must raise BEFORE yielding any chunk.
        client = _mock_client(lambda req: httpx.Response(403, text="nope"))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSAuthError):
            async for _ in p.stream("hi"):
                pass

    @pytest.mark.asyncio
    async def test_stream_no_chunks_raises(self):
        body = "data:[DONE]\n\n"

        async def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(200, text=body)

        client = _mock_client(handler)
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        with pytest.raises(TTSError):
            async for _ in p.stream("hi"):
                pass


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------


class TestAclose:
    @pytest.mark.asyncio
    async def test_aclose_closes_owned_client(self):
        """When the provider creates its own client (no client= passed), aclose()
        must close that client."""
        p = MiniMaxTTSProvider(api_key="k", group_id="g")
        assert p._owns_client is True
        client = p._client  # owned by the provider
        await p.aclose()
        assert client.is_closed

    @pytest.mark.asyncio
    async def test_aclose_does_not_close_injected_client(self):
        """If the caller injects a client, the provider must NOT close it."""
        client = _mock_client(lambda req: httpx.Response(200))
        p = MiniMaxTTSProvider(api_key="k", group_id="g", client=client)
        # We didn't set _owns_client=True; verify aclose is a no-op for the client.
        await p.aclose()
        assert not client.is_closed
        await client.aclose()  # cleanup
