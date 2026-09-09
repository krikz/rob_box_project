"""Tests for credential-safe logging in MiniMaxTTSProvider.

The provider sends ``GroupId`` as a URL query parameter.  httpx access logs
render the full URL at INFO, so this module verifies the real ``httpx`` logger
rather than only the provider-local logger.
"""

from __future__ import annotations

import logging

from collections.abc import Callable

import httpx
import pytest

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSSettings


SECRET_API_KEY = "sk-MARKER-DO-NOT-LOG-ac5f796b"
SECRET_GROUP_ID = "g-MARKER-DO-NOT-LOG-a51996c7"


def _ok_response() -> dict[str, object]:
    samples = b"\x00\x01" * 20
    return {
        "data": {
            "audio": samples.hex(),
            "audio_sample_rate": 24_000,
            "audio_length": len(samples),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _provider(
    handler: Callable[[httpx.Request], httpx.Response],
) -> MiniMaxTTSProvider:
    client = httpx.AsyncClient(transport=httpx.MockTransport(handler))
    return MiniMaxTTSProvider(
        api_key=SECRET_API_KEY,
        group_id=SECRET_GROUP_ID,
        client=client,
    )


def _assert_credentials_absent(caplog: pytest.LogCaptureFixture) -> None:
    rendered = "\n".join(record.getMessage() for record in caplog.records)
    assert SECRET_API_KEY not in rendered
    assert SECRET_GROUP_ID not in rendered


class TestCredentialLogging:
    """Exercise success and error paths with the global httpx logger enabled."""

    @pytest.mark.asyncio
    async def test_httpx_info_log_redacts_group_id_on_success(
        self,
        caplog: pytest.LogCaptureFixture,
    ) -> None:
        def handler(request: httpx.Request) -> httpx.Response:
            return httpx.Response(200, json=_ok_response())

        provider = _provider(handler)

        with caplog.at_level(logging.INFO, logger="httpx"):
            # Simulate another component re-enabling normal httpx access logs
            # after the provider was constructed.
            logging.getLogger("httpx").setLevel(logging.INFO)
            await provider.synthesize("hi", settings=TTSSettings(sample_rate=24_000))

        assert any(record.name == "httpx" for record in caplog.records)
        _assert_credentials_absent(caplog)
        await provider._client.aclose()

    @pytest.mark.asyncio
    async def test_httpx_info_log_redacts_group_id_on_http_error(
        self,
        caplog: pytest.LogCaptureFixture,
    ) -> None:
        def handler(request: httpx.Request) -> httpx.Response:
            return httpx.Response(401, text="unauthorized")

        provider = _provider(handler)

        with caplog.at_level(logging.INFO, logger="httpx"):
            logging.getLogger("httpx").setLevel(logging.INFO)
            with pytest.raises(Exception):
                await provider.synthesize("hi")

        assert any(record.name == "httpx" for record in caplog.records)
        _assert_credentials_absent(caplog)
        await provider._client.aclose()
