"""Successful MiniMax synthesis contract for supported request formats."""

from __future__ import annotations

import json
from typing import Any, get_type_hints

import httpx
import pytest
import respx

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSAudio, TTSFormat, TTSSettings


_FORMAT_CASES = [
    pytest.param(
        "wav",
        "audio/wav",
        b"RIFF\x24\x00\x00\x00WAVEfmt ",
        TTSFormat.WAV,
        "wav",
        id="wav",
    ),
    pytest.param(
        "mp3",
        "audio/mpeg",
        b"ID3\x04\x00\x00\x00\x00\x00\x00\xff\xfb",
        TTSFormat.MP3,
        "mp3",
        id="mp3",
    ),
    pytest.param(
        "ogg",
        "audio/ogg",
        b"OggS\x00\x02\x00\x00\x00\x00",
        TTSFormat.MP3,
        "mp3",
        id="ogg-falls-back-to-mp3",
    ),
]


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("fmt", "content_type", "audio_bytes", "expected_format", "wire_format"),
    _FORMAT_CASES,
)
async def test_synthesize_formats(
    minimax_provider: MiniMaxTTSProvider,
    mock_minimax_http: respx.Router,
    sample_text: str,
    valid_tts_params: dict[str, Any],
    fmt: str,
    content_type: str,
    audio_bytes: bytes,
    expected_format: TTSFormat,
    wire_format: str,
) -> None:
    """The format request, response bytes and public return contract agree.

    MiniMax's T2A v2 endpoint responds with a JSON envelope containing
    hex-encoded audio, so ``Content-Type`` describes that envelope rather than
    the embedded container. OGG is not accepted by MiniMax and is therefore
    sent and reported as the provider's documented MP3 fallback; distinctive
    OggS bytes are retained here to prove the provider passes payload bytes
    through without attempting container detection.
    """
    response_body = {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": 32_000,
            "audio_length": len(audio_bytes),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }
    response_content = json.dumps(response_body).encode()
    route = mock_minimax_http.post(
        "https://api.minimax.io/v1/t2a_v2",
        params={"GroupId": minimax_provider._group_id},
    ).mock(
        return_value=httpx.Response(
            200,
            headers={
                "Content-Type": content_type,
                "Content-Length": str(len(response_content)),
            },
            content=response_content,
        )
    )
    settings = TTSSettings(
        **{
            **valid_tts_params,
            "format": TTSFormat(fmt),
            "voice": "Calm_Woman",
            "language": "ru",
            "speed": 1.25,
        }
    )

    result: TTSAudio = await minimax_provider.synthesize(
        sample_text,
        settings=settings,
    )

    # Static and runtime return contracts: callers receive one materialized
    # TTSAudio value (not bytes and not an AsyncIterator).
    assert get_type_hints(MiniMaxTTSProvider.synthesize)["return"] is TTSAudio
    assert isinstance(result, TTSAudio)
    assert result.format is expected_format
    assert result.samples == audio_bytes

    assert route.called
    request = route.calls.last.request
    request_body = json.loads(request.content)
    assert request.method == "POST"
    assert request.headers["Authorization"] == f"Bearer {minimax_provider._api_key}"
    assert request.headers["Content-Type"] == "application/json"
    assert request.url.params["GroupId"] == minimax_provider._group_id
    assert request_body["audio_setting"]["format"] == wire_format
    assert request_body["voice_setting"] == {
        "voice_id": "Calm_Woman",
        "speed": 1.25,
        "vol": 5.0,
        "pitch": 0,
        "emotion": "neutral",
        "language": "Russian",
    }

    assert result.raw["data"]["audio_length"] == len(audio_bytes)
    assert route.calls.last.response.status_code == 200
    assert route.calls.last.response.headers["Content-Type"] == content_type
    assert route.calls.last.response.headers["Content-Length"] == str(
        len(response_content)
    )
