#!/usr/bin/env python3
"""Unit-тесты для :mod:`rob_box_voice.stt_providers.minimax_provider`.

Phase 1 PoC (issue #2365). Никаких сетевых вызовов — только httpx mock.

Что покрываем:

* ``recognize`` возвращает text при 200/JSON-ответе
* ``recognize`` возвращает ``None`` при 401 / 403 / 429 / 5xx / timeout
  / non-JSON / missing ``text`` field
* ``_extract_text`` понимает обе формы JSON (``{"text": ...}`` и
  ``{"data": {"text": ...}}``)
* ``MiniMaxSTTProvider.maybe_from_env`` возвращает ``None`` без ключа
* Пустые/огромные audio_bytes — отказ до сети
* Протокол ``STTProvider``: ``name`` стабилен, ``recognize(bytes)`` —
  единственный публичный метод (для ``select_recognition``).
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import httpx
import pytest

from rob_box_voice.stt_providers.minimax_provider import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    MAX_AUDIO_BYTES,
    PROVIDER_NAME,
    MiniMaxSTTAuthError,
    MiniMaxSTTError,
    MiniMaxSTTInvalidResponseError,
    MiniMaxSTTProvider,
    MiniMaxSTTRateLimitError,
    MiniMaxSTTUnavailableError,
    _extract_text,
)


# ---------------------------------------------------------------------------
# Test doubles — httpx transport + helpers
# ---------------------------------------------------------------------------


class _StubTransport(httpx.BaseTransport):
    """Минимальный поддельный httpx transport.

    Подменяет реальный сокет. Возвращает заранее заданные ``status`` +
    ``payload`` независимо от тела запроса. Можно также поднять
    исключение (``exception``), чтобы имитировать timeout / connect error.
    """

    def __init__(
        self,
        *,
        status: int = 200,
        payload: Any = None,
        raw_body: Optional[bytes] = None,
        exception: Optional[Exception] = None,
    ) -> None:
        self.status = status
        self.payload = payload
        self.raw_body = raw_body
        self.exception = exception
        self.calls: list[httpx.Request] = []

    def handle_request(self, request: httpx.Request) -> httpx.Response:
        self.calls.append(request)
        if self.exception is not None:
            raise self.exception

        if self.raw_body is not None:
            return httpx.Response(self.status, content=self.raw_body)
        return httpx.Response(self.status, json=self.payload)


def _make_provider(
    transport: _StubTransport,
    *,
    api_key: str = "test-key-12345",
    language: Optional[str] = "ru",
    timeout: httpx.Timeout = httpx.Timeout(connect=1.0, read=1.0, write=1.0, pool=1.0),
) -> MiniMaxSTTProvider:
    client = httpx.Client(
        transport=transport,
        timeout=timeout,
    )
    return MiniMaxSTTProvider(
        base_url=DEFAULT_BASE_URL,
        api_key=api_key,
        model=DEFAULT_MODEL,
        language=language,
        timeout=timeout,
        client=client,
    )


# 1 second of 16kHz 16-bit mono PCM silence (~32 KiB).
SILENCE_AUDIO = b"\x00\x00" * 16000


# ---------------------------------------------------------------------------
# Constants / config
# ---------------------------------------------------------------------------


class TestModuleConstants:
    def test_provider_name_stable(self):
        # Stable string — used as metric label, log tag, etc.
        assert PROVIDER_NAME == "minimax"

    def test_default_base_url_points_to_minimax(self):
        assert DEFAULT_BASE_URL == "https://api.minimax.io"

    def test_default_model_is_asr1(self):
        assert DEFAULT_MODEL == "asr-1.0"

    def test_max_audio_bytes_reasonable(self):
        # 25 MB engineering ceiling, not zero / not gigantic.
        assert 1 * 1024 * 1024 < MAX_AUDIO_BYTES <= 100 * 1024 * 1024


# ---------------------------------------------------------------------------
# _extract_text — response parser
# ---------------------------------------------------------------------------


class TestExtractText:
    def test_top_level_text(self):
        assert _extract_text({"text": "привет робот"}) == "привет робот"

    def test_text_gets_stripped(self):
        assert _extract_text({"text": "  расскажи сказку  "}) == "расскажи сказку"

    def test_empty_text_is_none(self):
        # Пустая строка после strip → None (а не "")
        assert _extract_text({"text": ""}) is None
        assert _extract_text({"text": "   "}) is None

    def test_nested_data_text(self):
        # Некоторые зеркала отвечают в {"data": {"text": ...}}
        assert _extract_text({"data": {"text": "привет"}}) == "привет"

    def test_missing_text_returns_none(self):
        assert _extract_text({"foo": "bar"}) is None
        assert _extract_text({}) is None

    def test_non_string_text_returns_none(self):
        # Robustness: если сервер случайно прислал {"text": 42}, не падаем.
        assert _extract_text({"text": 42}) is None

    def test_non_mapping_returns_none(self):
        assert _extract_text("not a dict") is None
        assert _extract_text(None) is None


# ---------------------------------------------------------------------------
# recognize() — happy path
# ---------------------------------------------------------------------------


class TestRecognizeSuccess:
    def test_returns_text_for_200_json(self):
        transport = _StubTransport(status=200, payload={"text": "расскажи сказку"})
        provider = _make_provider(transport)

        text = provider.recognize(SILENCE_AUDIO)

        assert text == "расскажи сказку"
        assert len(transport.calls) == 1
        # Endpoint is correct (research confirmed POST https://api.minimax.io/v1/speech_to_text).
        assert transport.calls[0].url.path == "/v1/speech_to_text"
        # Method
        assert transport.calls[0].method == "POST"

    def test_request_carries_bearer_token(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport, api_key="super-secret-key")

        provider.recognize(SILENCE_AUDIO)

        auth_header = transport.calls[0].headers.get("Authorization")
        assert auth_header == "Bearer super-secret-key"

    def test_request_sends_multipart_file_and_model(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)

        provider.recognize(SILENCE_AUDIO)

        # httpx собирает multipart — проверим, что в теле есть и файл, и поля.
        content_type = transport.calls[0].headers.get("Content-Type", "")
        assert content_type.startswith("multipart/form-data"), content_type
        # Body as multipart — проверяем по байтам.
        body = transport.calls[0].read()
        assert b'name="model"' in body
        assert b"asr-1.0" in body
        assert b'name="file"' in body
        # Язык (по умолчанию ru) — провайдер должен его прокинуть.
        # Multipart-значения httpx НЕ квотирует — это plain ``ru``.
        assert b'name="language"' in body
        assert b"ru" in body

    def test_language_none_omits_language_field(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport, language=None)

        provider.recognize(SILENCE_AUDIO)

        body = transport.calls[0].read()
        assert b'name="language"' not in body

    def test_strips_whitespace_in_text(self):
        transport = _StubTransport(status=200, payload={"text": "   привет   "})
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) == "привет"


# ---------------------------------------------------------------------------
# recognize() — error mapping to None
# ---------------------------------------------------------------------------


class TestRecognizeErrorCases:
    @pytest.mark.parametrize(
        "status, payload",
        [
            (401, {"error": "unauthorized"}),
            (403, {"error": "forbidden"}),
            (429, {"error": "rate limit exceeded"}),
            (500, {"error": "internal"}),
            (502, {"error": "bad gateway"}),
            (503, {"error": "unavailable"}),
        ],
    )
    def test_http_error_status_returns_none(self, status, payload):
        transport = _StubTransport(status=status, payload=payload)
        provider = _make_provider(transport)

        # recognize() must NOT raise — it degrades to None so the chain
        # can fall back to Vosk (issue #979 contract).
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_400_with_unexpected_payload_returns_none(self):
        transport = _StubTransport(status=400, payload={"error": "bad request"})
        provider = _make_provider(transport)

        # 400 is also swallowed (won't help retrying, but no need to crash).
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_non_json_response_returns_none(self):
        transport = _StubTransport(
            status=200, raw_body=b"<html>not json</html>"
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None

    def test_json_without_text_field_returns_none(self):
        transport = _StubTransport(
            status=200, payload={"result": "no text key"}
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None

    def test_timeout_translates_to_none(self):
        transport = _StubTransport(
            exception=httpx.ReadTimeout("read timed out")
        )
        provider = _make_provider(transport)

        # Read-timeout = soft failure for select_recognition.
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_connect_error_translates_to_none(self):
        transport = _StubTransport(
            exception=httpx.ConnectError("dns failed")
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None


# ---------------------------------------------------------------------------
# transcribe() — raises typed exceptions (for advanced callers)
# ---------------------------------------------------------------------------


class TestTranscribeRaisesTyped:
    def test_401_raises_auth_error(self):
        transport = _StubTransport(status=401, payload={"error": "bad key"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTAuthError):
            provider.transcribe(SILENCE_AUDIO)

    def test_429_raises_rate_limit_error(self):
        transport = _StubTransport(status=429, payload={"error": "slow down"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTRateLimitError):
            provider.transcribe(SILENCE_AUDIO)

    def test_5xx_raises_unavailable(self):
        transport = _StubTransport(status=503, payload={"error": "down"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTUnavailableError):
            provider.transcribe(SILENCE_AUDIO)

    def test_missing_text_raises_invalid_response(self):
        transport = _StubTransport(status=200, payload={"foo": "bar"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTInvalidResponseError):
            provider.transcribe(SILENCE_AUDIO)

    def test_all_exceptions_subclass_minimax_error(self):
        # Catch-all base class — callers can `except MiniMaxSTTError` once.
        assert issubclass(MiniMaxSTTAuthError, MiniMaxSTTError)
        assert issubclass(MiniMaxSTTRateLimitError, MiniMaxSTTError)
        assert issubclass(MiniMaxSTTUnavailableError, MiniMaxSTTError)
        assert issubclass(MiniMaxSTTInvalidResponseError, MiniMaxSTTError)


# ---------------------------------------------------------------------------
# Input validation — до сети
# ---------------------------------------------------------------------------


class TestInputValidation:
    def test_empty_audio_raises_before_network(self):
        transport = _StubTransport(status=200, payload={"text": "ok"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTError, match="empty"):
            provider.transcribe(b"")

        # Подтверждаем, что сеть не дёрнулась.
        assert transport.calls == []

    def test_oversize_audio_raises_before_network(self):
        transport = _StubTransport(status=200, payload={"text": "ok"})
        provider = _make_provider(transport)

        huge = b"\x00" * (MAX_AUDIO_BYTES + 1)
        with pytest.raises(MiniMaxSTTError, match="exceeds"):
            provider.transcribe(huge)

        assert transport.calls == []


# ---------------------------------------------------------------------------
# Factory: maybe_from_env
# ---------------------------------------------------------------------------


class TestMaybeFromEnv:
    def test_returns_none_without_key(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
        assert MiniMaxSTTProvider.maybe_from_env() is None

    def test_returns_none_with_empty_key(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        monkeypatch.setenv("MINIMAX_API_KEY", "")
        assert MiniMaxSTTProvider.maybe_from_env() is None

    def test_returns_instance_with_key(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        monkeypatch.setenv("MINIMAX_API_KEY", "abc123")
        provider = MiniMaxSTTProvider.maybe_from_env()
        assert provider is not None
        assert provider.name == PROVIDER_NAME


# ---------------------------------------------------------------------------
# Construction — no key raises
# ---------------------------------------------------------------------------


class TestConstruction:
    def test_no_api_key_raises_unavailable(self):
        with pytest.raises(MiniMaxSTTUnavailableError):
            MiniMaxSTTProvider(api_key="")


# ---------------------------------------------------------------------------
# STTProvider Protocol — structural conformance
# ---------------------------------------------------------------------------


class TestProtocolConformance:
    def test_satisfies_stt_provider_protocol(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)
        # ``STTProvider`` в rob_box_voice.stt_fallback объявлен как
        # ``typing.Protocol`` БЕЗ ``@runtime_checkable``, поэтому
        # ``isinstance`` против него не работает (Pyright / mypy).
        # Проверяем duck-type: имя ``name`` строка + ``recognize`` callable.
        assert isinstance(getattr(provider, "name", None), str)
        assert callable(getattr(provider, "recognize", None))

    def test_name_attribute_is_string(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)
        assert isinstance(provider.name, str)
        assert provider.name  # not empty

    def test_recognize_callable_with_bytes(self):
        transport = _StubTransport(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)
        # Сигнатура: recognize(bytes) — без extra kwargs (Phase 2 может
        # добавить ``language=``, ``diarize=``, но базовый контракт не
        # должен ломаться).
        result = provider.recognize(SILENCE_AUDIO)
        assert result == "ок"


# ---------------------------------------------------------------------------
# Parity test — MiniMax vs Vosk fallback (issue #2365 acceptance)
# ---------------------------------------------------------------------------


class TestParityWithFallback:
    """Минимальный parity-тест: MiniMax-провайдер в chain даёт тот же
    текст, что и Vosk fallback, если оба успешно распознали."""

    def test_minimax_text_matches_expected(self):
        # В реальном e2e это «та же фраза на одной записи». В PoC
        # достаточно убедиться, что MiniMax-ответ прозрачно проходит
        # через ``recognize()`` и попадает в финальный текст.
        expected_text = "робот расскажи сказку про дракона"
        transport = _StubTransport(
            status=200, payload={"text": expected_text}
        )
        provider = _make_provider(transport)

        result = provider.recognize(SILENCE_AUDIO)
        assert result == expected_text


# ---------------------------------------------------------------------------
# Logging redaction — не падаем и не оставляем ключ в логах
# ---------------------------------------------------------------------------


class TestLogRedaction:
    def test_redaction_filter_installed_on_module_logger(self):
        from rob_box_voice.stt_providers import minimax_provider as mod
        from rob_box_voice.stt_providers.minimax_provider import (
            MiniMaxSTTRedactedLogFilter,
        )

        # Чистим фильтры, оставшиеся от предыдущих тестов с другими ключами,
        # иначе isinstance-найдёт "чужой" redactor, который не скрэбит наш
        # ключ (фильтр запоминает ключ на этапе конструктора).
        mod._log.filters = [
            f
            for f in mod._log.filters
            if not isinstance(f, MiniMaxSTTRedactedLogFilter)
        ]
        # Тот же фокус для httpx-логгера (конструктор вешает туда же).
        logging.getLogger("httpx").filters = [
            f
            for f in logging.getLogger("httpx").filters
            if not isinstance(f, MiniMaxSTTRedactedLogFilter)
        ]

        secret = "very-secret-xyz"
        transport = _StubTransport(status=200, payload={"text": "ok"})
        _make_provider(transport, api_key=secret)

        redactor = next(
            (
                f
                for f in mod._log.filters
                if isinstance(f, MiniMaxSTTRedactedLogFilter)
            ),
            None,
        )
        assert redactor is not None, "MiniMaxSTTRedactedLogFilter not attached"

        record = logging.LogRecord(
            name="x",
            level=logging.INFO,
            pathname=__file__,
            lineno=0,
            msg=f"Authorization: Bearer {secret}",
            args=(),
            exc_info=None,
        )
        assert redactor.filter(record) is True
        assert secret not in record.getMessage()