"""Wire-level request and API-key redaction tests for MiniMax TTS."""

from __future__ import annotations

import json
import logging
from typing import Any

import httpx
import pytest
import respx

from rob_box_llm.errors import TTSAuthError, TTSError
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSSettings

from conftest import FAKE_API_KEY, FAKE_GROUP_ID, MINIMAX_BASE_URL, MINIMAX_T2A_PATH


_REALISTIC_TEST_KEY = "test-minimax-api-key-do-not-log-7228ce28"


def _ok_response() -> dict[str, Any]:
    return {
        "data": {
            "audio": b"\x00\x01".hex(),
            "audio_sample_rate": 32_000,
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _mock_response(status_code: int) -> httpx.Response:
    if status_code == 200:
        return httpx.Response(status_code, json=_ok_response())
    return httpx.Response(status_code, text="request rejected")


def _captured_log_text(caplog: pytest.LogCaptureFixture) -> str:
    """Render actual messages and interpolation arguments from all records."""
    parts: list[str] = []
    for record in caplog.records:
        parts.extend((record.getMessage(), repr(record.args), repr(record.exc_info)))
    return "\n".join(parts)


def _assert_key_redacted(value: object, key: str) -> None:
    assert key not in str(value)
    assert key not in repr(value)


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("voice", "language", "speed"),
    [
        pytest.param(None, None, None, id="provider-defaults"),
        pytest.param("Calm_Woman", "ru", 0.5, id="lower-speed-bound"),
        pytest.param("Calm_Woman", "Русский", 1.0, id="cyrillic-language"),
        pytest.param("Calm_Woman", "English", 2.0, id="upper-speed-bound"),
    ],
)
async def test_request_payload_params(
    minimax_provider: MiniMaxTTSProvider,
    mock_minimax_http: respx.Router,
    sample_text: str,
    voice: str | None,
    language: str | None,
    speed: float | None,
) -> None:
    route = mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=httpx.Response(200, json=_ok_response()))
    settings = TTSSettings(voice=voice, language=language, speed=speed)

    await minimax_provider.synthesize(sample_text, settings=settings)

    request = route.calls.last.request
    voice_setting = json.loads(request.content)["voice_setting"]
    assert voice_setting["voice_id"] == (
        voice if voice is not None else MiniMaxTTSProvider.DEFAULT_VOICE
    )
    if language is None:
        assert "language" not in voice_setting
    elif language == "ru":
        assert voice_setting["language"] == "Russian"
    else:
        assert voice_setting["language"] == language
    if speed is None:
        assert "speed" not in voice_setting
    else:
        assert voice_setting["speed"] == speed


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_headers_include_auth(
    minimax_provider: MiniMaxTTSProvider,
    mock_minimax_http: respx.Router,
) -> None:
    route = mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=httpx.Response(200, json=_ok_response()))

    await minimax_provider.synthesize("hello")

    assert route.calls.last.request.headers["Authorization"] == f"Bearer {FAKE_API_KEY}"


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize("status_code", [200, 401, 500])
async def test_api_key_not_in_logs(
    caplog: pytest.LogCaptureFixture,
    mock_minimax_http: respx.Router,
    status_code: int,
) -> None:
    route = mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=_mock_response(status_code))
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )
    for logger_name in (
        "rob_box_llm.providers.minimax_tts",
        "httpx",
        "httpcore",
        "aiohttp",
    ):
        caplog.set_level(logging.DEBUG, logger=logger_name)
        logging.getLogger(logger_name).setLevel(logging.DEBUG)

    try:
        if status_code == 200:
            await provider.synthesize("hello")
        else:
            with pytest.raises(TTSError):
                await provider.synthesize("hello")
    finally:
        await provider.aclose()

    assert route.called
    assert caplog.records, "log-leak assertion would be vacuous"
    log_text = _captured_log_text(caplog)
    assert _REALISTIC_TEST_KEY not in log_text
    assert FAKE_API_KEY not in log_text


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize("status_code", [401, 500])
async def test_api_key_not_in_exception_message(
    mock_minimax_http: respx.Router,
    status_code: int,
) -> None:
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=_mock_response(status_code))
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSError) as exc_info:
            await provider.synthesize("hello")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize("status_code", [401, 500])
async def test_api_key_echoed_by_server_is_redacted_from_exception(
    mock_minimax_http: respx.Router,
    status_code: int,
) -> None:
    """An upstream that echoes Authorization must not expose it to callers."""
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        return_value=httpx.Response(
            status_code,
            text=f"rejected Authorization: Bearer {_REALISTIC_TEST_KEY}",
        )
    )
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        expected_error = TTSAuthError if status_code == 401 else TTSError
        with pytest.raises(expected_error) as exc_info:
            await provider.synthesize("hello")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_non_json_success_echoed_credentials_are_redacted_from_exception(
    mock_minimax_http: respx.Router,
) -> None:
    """Malformed success bodies are untrusted and must be redacted."""
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        return_value=httpx.Response(
            200,
            text=f"not-json {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}",
        )
    )
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSError) as exc_info:
            await provider.synthesize("hello")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)
    _assert_key_redacted(exc_info.value, FAKE_GROUP_ID)


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_api_error_envelope_echoed_credentials_are_redacted_from_exception(
    mock_minimax_http: respx.Router,
) -> None:
    """MiniMax error-envelope messages must not expose credentials."""
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        return_value=httpx.Response(
            200,
            json={
                "base_resp": {
                    "status_code": 1001,
                    "status_msg": (
                        f"auth rejected {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}"
                    ),
                }
            },
        )
    )
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSAuthError) as exc_info:
            await provider.synthesize("hello")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)
    _assert_key_redacted(exc_info.value, FAKE_GROUP_ID)


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize("status_code", [401, 500])
async def test_stream_http_error_echoed_credentials_are_redacted_from_exception(
    mock_minimax_http: respx.Router,
    status_code: int,
) -> None:
    """Streaming HTTP errors must redact credentials echoed by upstream."""
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        return_value=httpx.Response(
            status_code,
            text=f"rejected {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}",
        )
    )
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSError) as exc_info:
            async for _ in provider.stream("hello"):
                pytest.fail("stream yielded before HTTP error")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)
    _assert_key_redacted(exc_info.value, FAKE_GROUP_ID)


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_api_error_envelope_echoed_credentials_are_redacted(
    mock_minimax_http: respx.Router,
) -> None:
    """Streaming API error events must redact credential-like content."""
    event = {
        "base_resp": {
            "status_code": 1001,
            "status_msg": f"auth rejected {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}",
        }
    }
    body = f"data:{json.dumps(event)}\n\ndata:[DONE]\n\n"
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=httpx.Response(200, text=body))
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSAuthError) as exc_info:
            async for _ in provider.stream("hello"):
                pytest.fail("stream yielded before API error")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)
    _assert_key_redacted(exc_info.value, FAKE_GROUP_ID)


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_malformed_event_does_not_log_credentials(
    caplog: pytest.LogCaptureFixture,
    mock_minimax_http: respx.Router,
) -> None:
    """Malformed SSE diagnostics must redact configured credentials."""
    body = (
        f"data:not-json {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}\n\n"
        "data:[DONE]\n\n"
    )
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(return_value=httpx.Response(200, text=body))
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    caplog.set_level(logging.DEBUG, logger="rob_box_llm.providers.minimax_tts")
    try:
        with pytest.raises(TTSError):
            async for _ in provider.stream("hello"):
                pytest.fail("malformed stream unexpectedly yielded")
    finally:
        await provider.aclose()

    log_text = _captured_log_text(caplog)
    assert _REALISTIC_TEST_KEY not in log_text
    assert FAKE_GROUP_ID not in log_text


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_transport_error_credentials_are_redacted_from_exception(
    mock_minimax_http: respx.Router,
) -> None:
    """Streaming transport diagnostics must not expose configured secrets."""
    mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        side_effect=httpx.ReadError(
            f"transport echoed {_REALISTIC_TEST_KEY} {FAKE_GROUP_ID}"
        )
    )
    provider = MiniMaxTTSProvider(
        api_key=_REALISTIC_TEST_KEY,
        group_id=FAKE_GROUP_ID,
    )

    try:
        with pytest.raises(TTSError) as exc_info:
            async for _ in provider.stream("hello"):
                pytest.fail("stream yielded before transport error")
    finally:
        await provider.aclose()

    _assert_key_redacted(exc_info.value, _REALISTIC_TEST_KEY)
    _assert_key_redacted(exc_info.value, FAKE_GROUP_ID)
