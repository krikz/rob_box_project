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
# Test doubles — httpx client + response stub
# ---------------------------------------------------------------------------
#
# Why we don't subclass ``httpx.BaseTransport`` / ``httpx.MockTransport``:
#
# Some CI test environments monkey-patch ``httpx`` itself (see
# ``test/test_dialogue_node.py:117`` and ``test/unit/node/conftest.py:97``
# — both build ``types.SimpleNamespace(...)`` to skip real network).
# Subclassing ``httpx.BaseTransport`` then blows up at collection time
# with::
#
#     AttributeError: 'types.SimpleNamespace' object has no attribute 'BaseTransport'
#
# which is exactly what the Unit Tests (ROS2 Humble) job saw on this PR.
#
# We instead replace ``MiniMaxSTTProvider._get_client`` / ``client.post``
# with a tiny stub that records the call and returns whatever the test
# wants. This works under both real httpx and the SimpleNamespace shim.


class _StubHTTPResponse:
    """Минимальный поддельный ``httpx.Response``.

    Поддерживает только то, что использует ``MiniMaxSTTProvider.transcribe``:
    ``status_code``, ``text``, ``json()``.
    """

    def __init__(
        self,
        *,
        status: int = 200,
        payload: Any = None,
        raw_body: Optional[bytes] = None,
    ) -> None:
        self.status_code = status
        self._payload = payload
        self._raw_body = raw_body
        if raw_body is not None:
            self.text = raw_body.decode("utf-8", errors="replace")
        elif payload is not None:
            self.text = ""
        else:
            self.text = ""

    def json(self) -> Any:
        if self._payload is None:
            raise ValueError("no json payload configured")
        return self._payload


# These are constructed *after* `httpx` is imported. On the CI shim
# (``unit/node/conftest.py:97`` replaces ``sys.modules["httpx"]`` with
# ``types.SimpleNamespace(Timeout=...)``) ``httpx.TimeoutException`` and
# ``httpx.HTTPError`` are missing, so we fall back to ``MiniMaxSTTUnavailableError``
# — that's the exception :meth:`MiniMaxSTTProvider.transcribe` *maps*
# HTTP-layer failures to anyway, so the observable behaviour of
# :meth:`recognize` (returns ``None``) is identical.
try:
    _HTTP_TIMEOUT_BASE = httpx.TimeoutException  # type: ignore[attr-defined]
    _HTTP_ERROR_BASE = httpx.HTTPError  # type: ignore[attr-defined]
except AttributeError:
    _HTTP_TIMEOUT_BASE = MiniMaxSTTUnavailableError
    _HTTP_ERROR_BASE = MiniMaxSTTUnavailableError


class _FakeReadTimeout(_HTTP_TIMEOUT_BASE):
    """Stand-in for ``httpx.ReadTimeout`` — SimpleNamespace-shim safe.

    Some CI test envs (``unit/node/conftest.py:97``) monkey-patch
    ``sys.modules["httpx"]`` to ``types.SimpleNamespace(Timeout=...)``,
    which has no ``ReadTimeout`` / ``ConnectError`` / ``TimeoutException``
    / ``HTTPError`` attributes. Using real ``httpx.ReadTimeout`` here
    would blow up at collection time.

    Subclassing ``httpx.TimeoutException`` (when available) keeps the
    ``except`` clauses in :meth:`MiniMaxSTTProvider.transcribe` matching
    our stub exception, so the provider's HTTP-error → typed-exception
    mapping still fires. Under the shim, we fall back to subclassing
    :class:`MiniMaxSTTUnavailableError` directly — which is the same
    exception the provider raises when its ``except httpx.HTTPError``
    branch matches, so :meth:`recognize` still returns ``None``.
    """


class _FakeConnectError(_HTTP_ERROR_BASE):
    """Stand-in for ``httpx.ConnectError`` — see :class:`_FakeReadTimeout`."""


class _StubHTTPClient:
    """Заменяет ``httpx.Client`` (или его SimpleNamespace-shim).

    ``post(...)`` возвращает заранее заданный ``_StubHTTPResponse``.
    Любое исключение (``exception=``) поднимается — имитация timeout /
    connect error / и т.п.
    """

    def __init__(
        self,
        *,
        status: int = 200,
        payload: Any = None,
        raw_body: Optional[bytes] = None,
        exception: Optional[Exception] = None,
    ) -> None:
        self._status = status
        self._payload = payload
        self._raw_body = raw_body
        self._exception = exception
        self.calls: list[dict[str, Any]] = []
        self.closed = False

    def post(self, url: str, *, headers: dict, files: dict, data: dict) -> _StubHTTPResponse:
        self.calls.append(
            {
                "url": url,
                "headers": dict(headers),
                "files": {k: (v[0], v[1].read() if hasattr(v[1], "read") else v[1]) for k, v in files.items()},
                "data": dict(data),
            }
        )
        if self._exception is not None:
            raise self._exception
        return _StubHTTPResponse(
            status=self._status, payload=self._payload, raw_body=self._raw_body
        )

    def close(self) -> None:
        self.closed = True

    # httpx.Client context-manager support (если кто-то вызовет ``with``).
    def __enter__(self) -> "_StubHTTPClient":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


def _make_provider(
    stub_client: _StubHTTPClient,
    *,
    api_key: str = "test-key-12345",
    language: Optional[str] = "ru",
    timeout: Any = None,
) -> MiniMaxSTTProvider:
    """Construct a provider wired to ``stub_client`` (bypasses real httpx).

    ``timeout`` accepts either a real ``httpx.Timeout`` or any duck-typed
    stand-in (we don't read its fields). Pass ``None`` for the default.
    A real ``httpx.Timeout`` is used when one is available; on the CI shim
    (``unit/node/conftest.py`` — ``httpx = types.SimpleNamespace(Timeout=...)``)
    we fall back to a plain sentinel, since the provider only stores the
    value and never calls methods on it during these tests.
    """
    if timeout is None:
        try:
            timeout = httpx.Timeout(connect=1.0, read=1.0, write=1.0, pool=1.0)
        except AttributeError:
            # httpx shim: ``Timeout`` may be a MagicMock factory that
            # returns something unusable. Use a plain sentinel — the
            # provider doesn't dereference any fields in tests.
            timeout = object()
    provider = MiniMaxSTTProvider(
        base_url=DEFAULT_BASE_URL,
        api_key=api_key,
        model=DEFAULT_MODEL,
        language=language,
        timeout=timeout,
    )
    # Подменяем внутренний client, чтобы реальный httpx не дёргался.
    # Конструктор не вызывает ``_get_client`` лениво — поэтому присвоение
    # напрямую безопасно.
    provider._client = stub_client  # type: ignore[assignment]
    return provider


# 1 second of 16kHz 16-bit mono PCM silence (~32 KiB).
SILENCE_AUDIO = b"\x00\x00" * 16000


# ---------------------------------------------------------------------------
# autouse fixture — isolate this module from ``unit/node/conftest.py``'s
# ``types.SimpleNamespace`` shim for ``sys.modules['httpx']``.
#
# Background:
#   * ``unit/node/conftest.py:97`` installs ``httpx = SimpleNamespace(Timeout=...)``
#     so the dialogue_node tests don't need real network.
#   * That shim lacks ``TimeoutException`` / ``HTTPError``, so when
#     ``MiniMaxSTTProvider.transcribe`` is invoked and one of the
#     ``except httpx.TimeoutException as exc:`` / ``except httpx.HTTPError``
#     clauses is evaluated at runtime, Python raises
#     ``AttributeError: 'types.SimpleNamespace' object has no attribute 'TimeoutException'``.
#   * The shim is installed with ``setdefault``, so any module that loads
#     real httpx *first* wins — but :mod:`rob_box_voice.stt_providers.minimax_provider`
#     is imported lazily inside the tests, by which time the shim has
#     already won.
#
# Strategy:
#   * Make sure the real httpx package is bound to ``sys.modules['httpx']``
#     before this module's tests run, but **only for this module's lifetime**.
#   * ``addfinalizer`` restores the previous entry so that sibling
#     modules in the same ``pytest test/`` invocation (notably
#     ``unit/tts/test_provider_chain.py``) keep their SimpleNamespace
#     httpx and don't regress.
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _ensure_real_httpx_during_tests():
    """autouse — swap ``sys.modules['httpx']`` for the real package.

    Idempotent: if the real package is already bound, this is a no-op
    apart from the finalizer. Restores the prior entry after each test
    so other tests in the same ``pytest`` run aren't affected.
    """
    import importlib
    import sys as _sys

    saved = _sys.modules.get("httpx")
    is_real = saved is not None and hasattr(saved, "__file__")
    if is_real:
        yield
        return

    # Drop the shim and re-import the real package. If httpx is missing
    # on this image entirely we keep whatever's bound (likely None).
    try:
        _sys.modules.pop("httpx", None)
        importlib.import_module("httpx")
    except ImportError:
        pass
    try:
        yield
    finally:
        if saved is not None:
            _sys.modules["httpx"] = saved
        else:
            _sys.modules.pop("httpx", None)


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

    def test_empty_text_returns_empty_string_not_none(self):
        # ADR-0096: valid empty result (silence recognised as empty)
        # must NOT be coerced to None — that path would raise
        # MiniMaxSTTInvalidResponseError downstream, semantically wrong.
        assert _extract_text({"text": ""}) == ""
        assert _extract_text({"text": "   "}) == ""

    def test_non_string_text_returns_empty_string(self):
        # ADR-0096: key exists but value is not a string → degraded but
        # valid empty (we cannot extract text, but the response shape is
        # not "invalid").
        assert _extract_text({"text": 42}) == ""
        assert _extract_text({"text": ["привет"]}) == ""

    def test_nested_data_text(self):
        # Некоторые зеркала отвечают в {"data": {"text": ...}}
        assert _extract_text({"data": {"text": "привет"}}) == "привет"

    def test_nested_data_empty_text_returns_empty_string(self):
        # ADR-0096: nested mirror with empty text → valid empty.
        assert _extract_text({"data": {"text": ""}}) == ""

    def test_nested_data_without_text_returns_empty_string(self):
        # ADR-0096: mirror shape {"data": {}} — wrapping is valid,
        # but no speech. Valid empty result, not invalid response.
        assert _extract_text({"data": {}}) == ""

    def test_missing_text_returns_none(self):
        # Truly malformed: no ``text`` key at any level → None signals
        # to caller that MiniMaxSTTInvalidResponseError should be raised.
        assert _extract_text({"foo": "bar"}) is None
        assert _extract_text({}) is None

    def test_non_mapping_returns_none(self):
        assert _extract_text("not a dict") is None
        assert _extract_text(None) is None


# ---------------------------------------------------------------------------
# recognize() — happy path
# ---------------------------------------------------------------------------


class TestRecognizeSuccess:
    def test_returns_text_for_200_json(self):
        transport = _StubHTTPClient(status=200, payload={"text": "расскажи сказку"})
        provider = _make_provider(transport)

        text = provider.recognize(SILENCE_AUDIO)

        assert text == "расскажи сказку"
        assert len(transport.calls) == 1
        # Endpoint is correct (research confirmed POST https://api.minimax.io/v1/speech_to_text).
        assert transport.calls[0]["url"].endswith("/v1/speech_to_text")

    def test_request_carries_bearer_token(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
        provider = _make_provider(transport, api_key="super-secret-key")

        provider.recognize(SILENCE_AUDIO)

        auth_header = transport.calls[0]["headers"].get("Authorization")
        assert auth_header == "Bearer super-secret-key"

    def test_request_sends_multipart_file_and_model(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)

        provider.recognize(SILENCE_AUDIO)

        # Упрощённая проверка: stub не собирает multipart-байты, но
        # проверяем, что модель/language/file попали в вызов.
        data = transport.calls[0]["data"]
        files = transport.calls[0]["files"]
        assert data["model"] == "asr-1.0"
        assert data["response_format"] == "json"
        assert data["language"] == "ru"
        assert files["file"][0] == "audio.wav"
        assert files["file"][1] == SILENCE_AUDIO

    def test_language_none_omits_language_field(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
        provider = _make_provider(transport, language=None)

        provider.recognize(SILENCE_AUDIO)

        # Stub хранит data как dict — проверим, что language там нет.
        assert "language" not in transport.calls[0]["data"]

    def test_strips_whitespace_in_text(self):
        transport = _StubHTTPClient(status=200, payload={"text": "   привет   "})
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) == "привет"


# ---------------------------------------------------------------------------
# recognize() — empty text (ADR-0096 + ADR-0091 §7)
# ---------------------------------------------------------------------------


class TestRecognizeEmpty:
    """Полная цепочка ``recognize()`` при пустом ответе ASR.

    Issue #2470: раньше ``_extract_text({"text":""}) → None → raise
    ``MiniMaxSTTInvalidResponseError`` → ``recognize()`` отдавал
    ``None`` через except. Это **семантически неверно** (тишина ≠
    ошибка) и спамит WARNING в логи (alert-fatigue, issue #1193).

    ADR-0096 фиксит: валидный пустой результат возвращается как
    ``""`` и идёт в ``select_recognition`` → ``reason="empty"``
    per ADR-0091 §7.
    """

    def test_recognize_with_empty_text_returns_empty_string(self):
        transport = _StubHTTPClient(status=200, payload={"text": ""})
        provider = _make_provider(transport)

        result = provider.recognize(SILENCE_AUDIO)

        # НЕ None (раньше был None из-за except). НЕ exception.
        # Пустая строка = ASR честно сказал «тишина».
        assert result == ""
        assert result is not None

    def test_recognize_with_whitespace_only_returns_empty_string(self):
        transport = _StubHTTPClient(status=200, payload={"text": "   "})
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) == ""

    def test_recognize_with_nested_empty_data_returns_empty_string(self):
        # Зеркальный формат: {"data": {"text": ""}}.
        transport = _StubHTTPClient(
            status=200, payload={"data": {"text": ""}}
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) == ""

    def test_recognize_with_non_string_text_returns_empty_string(self):
        # ADR-0096: ключ есть, но значение не строка → degraded, но
        # НЕ invalid. Робот молчит (reason="empty"), но без WARNING.
        transport = _StubHTTPClient(status=200, payload={"text": 42})
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) == ""

    def test_no_warning_logged_on_empty_text(self, caplog):
        # Issue #1193: alert-fatigue. Раньше каждый пустой ответ ASR
        # логировал WARNING через ``MiniMaxSTTInvalidResponseError``.
        # Теперь — INFO или ничего (мы хотим отличать реальные сбои
        # от штатной тишины).
        transport = _StubHTTPClient(status=200, payload={"text": ""})
        provider = _make_provider(transport)

        with caplog.at_level(logging.WARNING, logger="rob_box_voice.stt_providers.minimax_provider"):
            result = provider.recognize(SILENCE_AUDIO)

        assert result == ""
        # ``MiniMaxSTTInvalidResponseError`` НЕ должен упоминаться.
        assert "MiniMaxSTTInvalidResponseError" not in caplog.text
        # И никаких WARNING про missing text.
        assert "missing text" not in caplog.text

    def test_transcribe_with_empty_text_returns_response_not_raises(self):
        # ``transcribe()`` тоже не должен raise'ить на пустом тексте —
        # это валидный результат, не invalid response.
        from rob_box_voice.stt_providers.minimax_provider import (
            MiniMaxSTTResponse,
        )

        transport = _StubHTTPClient(status=200, payload={"text": ""})
        provider = _make_provider(transport)

        response = provider.transcribe(SILENCE_AUDIO)

        assert isinstance(response, MiniMaxSTTResponse)
        assert response.text == ""

    def test_transcribe_with_missing_text_key_still_raises(self):
        # Negative test (ADR-0096): действительно невалидный ответ —
        # по-прежнему raise. Это инвариант.
        transport = _StubHTTPClient(status=200, payload={"foo": "bar"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTInvalidResponseError) as excinfo:
            provider.transcribe(SILENCE_AUDIO)

        assert "missing 'text'" in str(excinfo.value)


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
        transport = _StubHTTPClient(status=status, payload=payload)
        provider = _make_provider(transport)

        # recognize() must NOT raise — it degrades to None so the chain
        # can fall back to Vosk (issue #979 contract).
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_400_with_unexpected_payload_returns_none(self):
        transport = _StubHTTPClient(status=400, payload={"error": "bad request"})
        provider = _make_provider(transport)

        # 400 is also swallowed (won't help retrying, but no need to crash).
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_non_json_response_returns_none(self):
        transport = _StubHTTPClient(
            status=200, raw_body=b"<html>not json</html>"
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None

    def test_json_without_text_field_returns_none(self):
        transport = _StubHTTPClient(
            status=200, payload={"result": "no text key"}
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None

    def test_timeout_translates_to_none(self):
        # _FakeReadTimeout instead of httpx.ReadTimeout — CI shim
        # (see unit/node/conftest.py:97) replaces httpx with a
        # SimpleNamespace that lacks ReadTimeout / ConnectError.
        transport = _StubHTTPClient(
            exception=_FakeReadTimeout("read timed out")
        )
        provider = _make_provider(transport)

        # Read-timeout = soft failure for select_recognition.
        assert provider.recognize(SILENCE_AUDIO) is None

    def test_connect_error_translates_to_none(self):
        # See _FakeReadTimeout docstring — same shim story.
        transport = _StubHTTPClient(
            exception=_FakeConnectError("dns failed")
        )
        provider = _make_provider(transport)

        assert provider.recognize(SILENCE_AUDIO) is None


# ---------------------------------------------------------------------------
# transcribe() — raises typed exceptions (for advanced callers)
# ---------------------------------------------------------------------------


class TestTranscribeRaisesTyped:
    def test_401_raises_auth_error(self):
        transport = _StubHTTPClient(status=401, payload={"error": "bad key"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTAuthError):
            provider.transcribe(SILENCE_AUDIO)

    def test_429_raises_rate_limit_error(self):
        transport = _StubHTTPClient(status=429, payload={"error": "slow down"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTRateLimitError):
            provider.transcribe(SILENCE_AUDIO)

    def test_5xx_raises_unavailable(self):
        transport = _StubHTTPClient(status=503, payload={"error": "down"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTUnavailableError):
            provider.transcribe(SILENCE_AUDIO)

    def test_missing_text_raises_invalid_response(self):
        transport = _StubHTTPClient(status=200, payload={"foo": "bar"})
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
        transport = _StubHTTPClient(status=200, payload={"text": "ok"})
        provider = _make_provider(transport)

        with pytest.raises(MiniMaxSTTError, match="empty"):
            provider.transcribe(b"")

        # Подтверждаем, что сеть не дёрнулась.
        assert transport.calls == []

    def test_oversize_audio_raises_before_network(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ok"})
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
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)
        # ``STTProvider`` в rob_box_voice.stt_fallback объявлен как
        # ``typing.Protocol`` БЕЗ ``@runtime_checkable``, поэтому
        # ``isinstance`` против него не работает (Pyright / mypy).
        # Проверяем duck-type: имя ``name`` строка + ``recognize`` callable.
        assert isinstance(getattr(provider, "name", None), str)
        assert callable(getattr(provider, "recognize", None))

    def test_name_attribute_is_string(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
        provider = _make_provider(transport)
        assert isinstance(provider.name, str)
        assert provider.name  # not empty

    def test_recognize_callable_with_bytes(self):
        transport = _StubHTTPClient(status=200, payload={"text": "ок"})
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
        transport = _StubHTTPClient(
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
        transport = _StubHTTPClient(status=200, payload={"text": "ok"})
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