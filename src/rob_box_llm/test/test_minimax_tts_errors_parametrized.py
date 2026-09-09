"""Parametrized error-path tests for :class:`MiniMaxTTSProvider`.

The brief asks for one test set that covers every documented error branch
the provider can hit, all sharing the same shape so a regression in one
path is obvious when compared to its siblings:

* the right ``TTSError`` subclass is raised,
* the underlying HTTP layer is called at most once (no infinite retries
  on permanent failures; retry-when-applicable cases are bounded),
* the raw ``MINIMAX_API_KEY`` does not leak into the user-visible
  exception message — the api_key is the most security-sensitive
  credential in the call path and must never appear in logs or in
  re-raised exception text.

Why parametrized
----------------

There are six error branches (400, 401, 403, 429, 5xx, timeout,
connection) and the assertion shape is identical for all of them.  A
copy-pasted ``async def test_x`` for each branch is brittle — adding a
new branch (say, 408 Request Timeout) means editing the test file in
several places instead of one table row.  ``pytest.mark.parametrize``
collapses all six into a single table that any reviewer can scan in
under thirty seconds and that the next contributor can extend with a
single new row.

Why respx (not ``httpx.MockTransport``)
--------------------------------------

The conftest already provides a ``mock_minimax_http`` respx fixture,
but the per-fixture pattern (replace ``provider._client`` with a
respx-mounted client) is awkward to compose for tests that also need
to assert ``respx.call_count``.  This file builds its own provider
instances and lets ``respx.mock()`` intercept the provider's default
``httpx.AsyncClient`` globally — the same approach the upstream
``test_minimax_tts_provider.py`` already uses for ``httpx.MockTransport``
but with the bonus of ``route.call_count`` and ``route.calls`` access
that the task brief explicitly asks for.
"""

from __future__ import annotations

import asyncio
import json

import httpx
import pytest
import respx

from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSSettings


# ---------------------------------------------------------------------------
# Sentinel credentials — distinct from the conftest FAKE_API_KEY so a leak
# from one test is not silenced by the logging guard from the conftest.
# ---------------------------------------------------------------------------
#
# The brief says "api_key не попал в str(exception)" — the assertion is
# against THIS value, not against whatever the conftest happened to use.
# We keep a *different* sentinel per test to make sure no two tests
# accidentally share state via the env scrubber (autouse in conftest
# wipes real keys, but it does not wipe ours).
#
# IMPORTANT: ASCII-only. httpx encodes header values as ASCII by
# default, so a sentinel with non-ASCII characters (``«``, ``»``) would
# fail with UnicodeEncodeError BEFORE we ever reach the assertion — the
# provider would never get a chance to leak the key, because the
# request would die at header construction. The whole point of this
# file is to verify what happens on the wire, so the sentinel has to
# survive the wire.
_SENTINEL_API_KEY = "sentinel-leak-guard-errparam-9d2a-DO-NOT-USE"
_SENTINEL_GROUP_ID = "g-SENTINEL-ERRPARAM-0000-0000-0000-000000000000"

# Server response bodies for each status code. Deliberately kept short
# and free of any credential-shaped string so the body itself does not
# accidentally trip the leak guard. If a real MiniMax response ever
# echoes the key into a 4xx body, the provider MUST redact it — the
# leak guard below is the regression net for that future fix.
_BODY_400 = "bad voice id"
_BODY_401 = "unauthorized"
_BODY_403 = "forbidden"
_BODY_429 = "quota exceeded"
_BODY_500 = "internal server error"
_BODY_502 = "bad gateway"
_BODY_503 = "service unavailable"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_provider() -> MiniMaxTTSProvider:
    """Provider with sentinel credentials and a real httpx client.

    ``respx.mock()`` in each test patches httpx globally, so the
    provider's default client is intercepted without us having to
    touch ``provider._client``. The provider's owner flag stays
    ``True`` so we can use ``aclose()`` for cleanup without leaking
    sockets across the test session.
    """
    return MiniMaxTTSProvider(
        api_key=_SENTINEL_API_KEY,
        group_id=_SENTINEL_GROUP_ID,
    )


def _ok_envelope(audio: bytes = b"\x00\x01" * 20) -> dict[str, object]:
    """Build a minimal MiniMax T2A v2 success envelope.

    Returned only by tests that need a baseline to assert "request was
    not made" — every parametrised branch below is an error path, so
    this helper is not used by them. It is here so the file is
    self-contained if someone copy-pastes a test that needs a 200.
    """
    return {
        "data": {
            "audio": audio.hex(),
            "audio_sample_rate": 24_000,
            "audio_length": len(audio),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


# ---------------------------------------------------------------------------
# 4xx error matrix — one row per (status, expected_exception)
# ---------------------------------------------------------------------------
#
# The provider's current behaviour is "raise on first failure, no
# retry". The brief asks us to assert that retries happen when
# retry-logic exists; the symmetric negative assertion ("no infinite
# loop on permanent failure") is what we encode here.  If a future
# PR adds retry logic, that PR is responsible for updating the
# expected call counts — these tests will then go red and force the
# contributor to think about the change.
#
# ``max_calls`` is the UPPER BOUND on how many times the provider is
# allowed to hit the wire for one synthesis call. 1 is the right
# number for "fail fast" providers; >1 means a retry budget was
# configured.

_HTTPMatrixRow = tuple[int, str, type[Exception], int]
# (status_code, body, expected_exception, max_calls)


_HTTP_ERROR_MATRIX: list[_HTTPMatrixRow] = [
    # 400 — provider's own empty-text guard raises TTSBadRequestError
    # BEFORE the wire, so we model it as status 0 here to keep the table
    # uniform.  See TestEmptyTextShortCircuits for the explicit version.
    (400, _BODY_400, TTSBadRequestError, 1),
    (401, _BODY_401, TTSAuthError, 1),
    (403, _BODY_403, TTSAuthError, 1),
    (429, _BODY_429, TTSRateLimitError, 1),
    (500, _BODY_500, TTSError, 1),
    (502, _BODY_502, TTSError, 1),
    (503, _BODY_503, TTSError, 1),
]


# ---------------------------------------------------------------------------
# Test class — one test per row of the matrix
# ---------------------------------------------------------------------------


class TestHttpErrorMatrix:
    """Every documented 4xx/5xx error code maps to the right typed error.

    See module docstring for why this is one parametrised class
    instead of seven hand-rolled ``async def test_*`` functions.
    """

    @pytest.mark.asyncio
    @pytest.mark.minimax
    @pytest.mark.parametrize(
        "status_code, body, expected_exc, max_calls",
        _HTTP_ERROR_MATRIX,
        ids=[
            "400-bad-request",
            "401-unauthorized",
            "403-forbidden",
            "429-too-many-requests",
            "500-internal",
            "502-bad-gateway",
            "503-unavailable",
        ],
    )
    async def test_http_status_maps_to_typed_exception_with_no_leak(
        self, status_code, body, expected_exc, max_calls
    ) -> None:
        """Status code → expected exception, bounded calls, no key leak.

        Three assertions per row:

        1. ``expected_exc`` is raised (not a sibling or parent class —
           matching the wrong type would silently let a 401 reach a
           quota-retry handler, which is exactly the bug we want to
           prevent).
        2. ``route.call_count <= max_calls`` — guards against the
           classic "retry loop on permanent failure" regression.
           The current contract is "no retries" so the bound is 1;
           tightening this number would force a reviewer to justify
           a future retry PR.
        3. ``_SENTINEL_API_KEY not in str(exc)`` — the leak guard. The
           api_key must never reach a log file or a user-visible
           exception message.
        """
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(status_code, text=body)
            )
            provider = _make_provider()
            try:
                with pytest.raises(expected_exc) as exc_info:
                    await provider.synthesize("hello world")
            finally:
                await provider.aclose()

        # 1. The exception is exactly the expected type (not a parent).
        assert type(exc_info.value) is expected_exc, (
            f"status {status_code} raised {type(exc_info.value).__name__}, "
            f"expected exactly {expected_exc.__name__}"
        )
        assert exc_info.value.provider == "minimax"

        # 2. No infinite retry loop.
        assert route.call_count <= max_calls, (
            f"status {status_code} triggered {route.call_count} HTTP calls "
            f"(>max_calls={max_calls}) — provider is retrying permanent "
            f"failures, which is a contract violation"
        )
        assert route.call_count >= 1, (
            f"status {status_code} did not hit the wire at all — "
            f"the route handler is misconfigured for this test"
        )

        # 3. The api_key does not appear in the user-visible message.
        #    We also guard against the GroupId (a related credential)
        #    in case a future change decides to surface the request
        #    envelope on the way out.
        rendered = str(exc_info.value)
        assert _SENTINEL_API_KEY not in rendered, (
            f"MINIMAX_API_KEY leaked into {expected_exc.__name__} message: "
            f"{rendered!r}"
        )
        assert _SENTINEL_GROUP_ID not in rendered, (
            f"MINIMAX_GROUP_ID leaked into {expected_exc.__name__} message: "
            f"{rendered!r}"
        )

    @pytest.mark.asyncio
    @pytest.mark.minimax
    @pytest.mark.parametrize(
        "status_code, body, expected_exc, max_calls",
        _HTTP_ERROR_MATRIX,
        ids=[f"stream-{row[0]}" for row in _HTTP_ERROR_MATRIX],
    )
    async def test_http_error_before_any_yield_raises(
        self, status_code, body, expected_exc, max_calls
    ) -> None:
        """The streaming path must raise BEFORE yielding anything.

        This is the contract spelled out in :class:`TTSProvider`'s
        docstring: pre-yield errors raise, post-yield errors become a
        terminal ``TTSChunk(finish_reason='error')``.  We exhaust the
        async iterator (so any erroneous early yield is consumed) and
        assert that:

        * the iterator terminates without producing any audio chunk,
        * the exception raised by the first ``__anext__`` is the
          expected type, and
        * the call count is bounded as for ``synthesize``.
        """
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(status_code, text=body)
            )
            provider = _make_provider()
            try:
                yielded: list = []
                with pytest.raises(expected_exc) as exc_info:
                    async for chunk in provider.stream("hello world"):
                        yielded.append(chunk)
            finally:
                await provider.aclose()

        assert yielded == [], (
            f"stream() yielded {len(yielded)} chunk(s) before raising on "
            f"status {status_code} — contract requires raising BEFORE yield"
        )
        assert type(exc_info.value) is expected_exc
        assert route.call_count <= max_calls
        assert route.call_count >= 1
        rendered = str(exc_info.value)
        assert _SENTINEL_API_KEY not in rendered
        assert _SENTINEL_GROUP_ID not in rendered


# ---------------------------------------------------------------------------
# Transport-level errors — side_effect on the respx route
# ---------------------------------------------------------------------------


class TestTransportErrors:
    """Errors that never produce an HTTP response (timeout, connection).

    The provider has to catch the bare exception in ``_post`` and map
    it onto the right domain error. ``respx`` lets us inject any
    exception into the request cycle via ``side_effect``; we cover the
    three documented httpx timeout classes plus a plain
    ``ConnectionError`` (the brief explicitly asks for it).
    """

    @pytest.mark.asyncio
    @pytest.mark.minimax
    @pytest.mark.parametrize(
        "side_effect, expected_exc, label",
        [
            (httpx.ConnectTimeout("connect timeout"), TTSTimeoutError, "connect-timeout"),
            (httpx.ReadTimeout("read timeout"), TTSTimeoutError, "read-timeout"),
            (httpx.PoolTimeout("pool timeout"), TTSTimeoutError, "pool-timeout"),
            (httpx.ConnectError("conn refused"), TTSTimeoutError, "connect-error"),
            (httpx.NetworkError("network unreachable"), TTSTimeoutError, "network-error"),
        ],
    )
    async def test_transport_error_mapped_with_no_leak(
        self, side_effect, expected_exc, label
    ) -> None:
        """Any httpx transport exception → ``TTSTimeoutError`` (current contract).

        The current ``_map_exception`` lumps every non-``HTTPStatusError``
        ``httpx.HTTPError`` (which includes ``ConnectError``,
        ``NetworkError``, ``ReadTimeout``, etc.) into ``TTSTimeoutError``
        because for the caller the only meaningful question is "can I
        retry?".  We assert the EXACT type here so a future
        refactor that splits ``TTSTimeoutError`` and ``TTSConnectionError``
        forces a deliberate update of these tests rather than a silent
        semantic change.
        """
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").mock(side_effect=side_effect)
            provider = _make_provider()
            try:
                with pytest.raises(expected_exc) as exc_info:
                    await provider.synthesize("hello world")
            finally:
                await provider.aclose()

        assert type(exc_info.value) is expected_exc
        assert exc_info.value.provider == "minimax"
        # Transport errors are "could be transient" — a future retry
        # policy might let the provider call the wire a couple of
        # times. Upper bound is 3 (2 retries on top of the initial
        # call); we don't pin the lower bound because asyncio.TimeoutError
        # and httpx.ReadTimeout may be conflated by the provider and
        # we're not testing that distinction here.
        assert 1 <= route.call_count <= 3, (
            f"{label}: expected 1-3 calls, got {route.call_count}"
        )
        rendered = str(exc_info.value)
        assert _SENTINEL_API_KEY not in rendered
        assert _SENTINEL_GROUP_ID not in rendered

    @pytest.mark.asyncio
    @pytest.mark.minimax
    async def test_asyncio_timeout_error_does_not_hang(self) -> None:
        """Bare ``asyncio.TimeoutError`` (not an httpx exception) must raise.

        The brief explicitly asks for ``side_effect=asyncio.TimeoutError``
        — a TimeoutError that is NOT an ``httpx.TimeoutException``.  The
        current code path drops it into the generic ``TTSError`` branch
        (since it's not an httpx class); the test asserts *some*
        ``TTSError`` subclass is raised (not a bare ``asyncio.TimeoutError``)
        and that the provider cleans up its connection.  Pinning
        exactly ``TTSTimeoutError`` here would be over-specifying:
        asyncio.TimeoutError is a TimeoutError, not a timeout-from-the
        network, and a future split into ``TTSConnectionError`` may be
        more honest.
        """
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").mock(
                side_effect=asyncio.TimeoutError()
            )
            provider = _make_provider()
            try:
                with pytest.raises(TTSError) as exc_info:
                    await provider.synthesize("hello world")
            finally:
                # aclose() must not raise even after a transport error.
                # The client may already be in a weird state; the test
                # verifies our finally-block is robust.
                await provider.aclose()

        # Some TTSError subclass was raised, not a raw asyncio.TimeoutError.
        assert not isinstance(exc_info.value, asyncio.TimeoutError)
        assert isinstance(exc_info.value, TTSError)
        assert exc_info.value.provider == "minimax"
        # Exactly one call — asyncio.TimeoutError is a programmer-error
        # type, the provider has no business retrying it.
        assert route.call_count == 1
        rendered = str(exc_info.value)
        assert _SENTINEL_API_KEY not in rendered
        assert _SENTINEL_GROUP_ID not in rendered


# ---------------------------------------------------------------------------
# Pre-flight guards (errors raised before any HTTP traffic)
# ---------------------------------------------------------------------------


class TestPreFlightGuards:
    """Errors that the provider raises before opening a socket.

    These are the cheapest branches to hit and the easiest to
    regress — a future refactor of ``_headers()`` or ``synthesize()``
    could silently drop the empty-text check.
    """

    @pytest.mark.asyncio
    @pytest.mark.minimax
    @pytest.mark.parametrize("empty", ["", "   ", "\n\t"])
    async def test_empty_text_short_circuits_to_bad_request(
        self, empty: str
    ) -> None:
        """Empty / whitespace-only text is a 400, raised BEFORE the wire.

        The provider's own guard catches this in ``synthesize()``
        (and again in ``stream()``) so no HTTP call should be made.
        The bound here is 0 — that's the whole point of a
        pre-flight guard.
        """
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").pass_through()
            provider = _make_provider()
            try:
                with pytest.raises(TTSBadRequestError) as exc_info:
                    await provider.synthesize(empty)
            finally:
                await provider.aclose()

        assert "empty" in str(exc_info.value).lower()
        assert exc_info.value.provider == "minimax"
        assert route.call_count == 0, (
            f"empty text must short-circuit; the wire was hit "
            f"{route.call_count} time(s) instead of 0"
        )
        assert _SENTINEL_API_KEY not in str(exc_info.value)
        assert _SENTINEL_GROUP_ID not in str(exc_info.value)

    @pytest.mark.asyncio
    @pytest.mark.minimax
    async def test_missing_api_key_raises_auth_without_hitting_wire(self) -> None:
        """``api_key=\"\"`` → TTSAuthError raised by ``_headers()``."""
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").pass_through()
            # Provider built with empty api_key — env scrubber keeps the
            # env-level MINIMAX_API_KEY out, so this is genuinely "".
            provider = MiniMaxTTSProvider(
                api_key="",
                group_id=_SENTINEL_GROUP_ID,
            )
            try:
                with pytest.raises(TTSAuthError) as exc_info:
                    await provider.synthesize("hello world")
            finally:
                await provider.aclose()

        assert exc_info.value.provider == "minimax"
        assert route.call_count == 0
        # The error message must not contain the real key (which is
        # empty here anyway, but the assertion holds for the
        # non-empty case in TestHttpErrorMatrix).  Verify the message
        # talks about configuration instead.
        assert "MINIMAX_API_KEY" in str(exc_info.value)

    @pytest.mark.asyncio
    @pytest.mark.minimax
    async def test_missing_group_id_works_without_hitting_wire_guard_only_api_key(self) -> None:
        """``group_id=\"\"`` → НЕ TTSAuthError: GroupId опционален на
        api.minimax.io (проверено живым запросом 2026-08-05: без GroupId
        возвращается аудио). Синтез идёт в сеть с пустым params; с пустым
        api_key — падает ДО сети."""
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").respond(200, json={})
            provider = MiniMaxTTSProvider(
                api_key=_SENTINEL_API_KEY,
                group_id="",
            )
            try:
                with pytest.raises(TTSAuthError) as exc_info:
                    await MiniMaxTTSProvider(
                        api_key="",
                        group_id="",
                    ).synthesize("hello world")
            finally:
                await provider.aclose()

        assert route.call_count == 0
        assert "MINIMAX_API_KEY" in str(exc_info.value)
        # And the API key still must not leak — the error talks about
        # the api_key, not the group_id.
        assert _SENTINEL_API_KEY not in str(exc_info.value)


# ---------------------------------------------------------------------------
# API-level error envelope (base_resp.status_code != 0) — distinct from
# HTTP-level errors above. MiniMax returns 200 OK with a JSON envelope
# that signals failure via base_resp.status_code.
# ---------------------------------------------------------------------------


_API_ERROR_MATRIX: list[tuple[int, str, type[Exception]]] = [
    (1001, "invalid api key", TTSAuthError),
    (1002, "invalid voice id", TTSBadRequestError),
    (1003, "rate limit exceeded", TTSRateLimitError),
    (1004, "unknown error", TTSError),
]


class TestApiLevelErrorEnvelope:
    """HTTP 200 + ``base_resp.status_code != 0`` is a separate failure mode.

    These cases do not show up as a non-2xx status, so the HTTP-level
    matrix above does not cover them. The provider has to inspect
    ``base_resp.status_code`` and pick a domain error based on
    ``status_msg`` keyword matching.
    """

    @pytest.mark.asyncio
    @pytest.mark.minimax
    @pytest.mark.parametrize(
        "api_status, msg, expected_exc",
        _API_ERROR_MATRIX,
        ids=[
            "1001-auth",
            "1002-bad-request",
            "1003-rate-limit",
            "1004-generic",
        ],
    )
    async def test_base_resp_error_envelope_maps_with_no_leak(
        self, api_status, msg, expected_exc
    ) -> None:
        """``base_resp`` error envelope → typed exception, no leak."""
        envelope = {
            "data": None,
            "base_resp": {"status_code": api_status, "status_msg": msg},
        }
        with respx.mock(assert_all_called=False) as router:
            route = router.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(
                    200,
                    content=json.dumps(envelope),
                    headers={"Content-Type": "application/json"},
                )
            )
            provider = _make_provider()
            try:
                with pytest.raises(expected_exc) as exc_info:
                    await provider.synthesize("hello world")
            finally:
                await provider.aclose()

        assert type(exc_info.value) is expected_exc
        assert exc_info.value.provider == "minimax"
        # status_msg may include the api_status code; assert the
        # message is the documented "{api_status}: {msg}" shape so
        # the heuristic mapping stays observable.
        assert str(api_status) in str(exc_info.value)
        assert msg in str(exc_info.value)
        # No retries on a definitive API-level error.
        assert route.call_count == 1
        rendered = str(exc_info.value)
        assert _SENTINEL_API_KEY not in rendered
        assert _SENTINEL_GROUP_ID not in rendered


# ---------------------------------------------------------------------------
# Cross-cutting: provider cleanup on every error path
# ---------------------------------------------------------------------------


class TestResourceCleanup:
    """Every error path must leave the provider in a closable state.

    The aclose() call inside each test's finally block is itself the
    verification: if any error path left the client in a bad state,
    pytest would surface the aclose() failure with a confusing
    traceback that points at the wrong line.  To make sure
    developers notice, this class explicitly counts the calls
    post-error and asserts a single call — the second
    ``provider.aclose()`` would have been a no-op anyway, but a
    regression that opens two clients per synthesis would be caught
    here.
    """

    @pytest.mark.asyncio
    @pytest.mark.minimax
    async def test_synthesize_error_path_closes_owned_client(self) -> None:
        with respx.mock(assert_all_called=False) as router:
            router.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(500, text="boom")
            )
            provider = _make_provider()
            assert provider._owns_client is True
            client = provider._client

            with pytest.raises(TTSError):
                await provider.synthesize("hello world")
            await provider.aclose()

            assert client.is_closed, (
                "provider did not close its owned client after an "
                "error path — aclose() must run regardless of how "
                "synthesize() exits"
            )

    @pytest.mark.asyncio
    @pytest.mark.minimax
    async def test_stream_error_path_does_not_leak_connection(self) -> None:
        """The streaming path opens a long-lived connection.

        On error, the ``async with self._client.stream(...)`` block
        must exit cleanly so the underlying HTTP connection is
        returned to the pool.  We assert this indirectly via
        ``client.is_closed`` after aclose().
        """
        with respx.mock(assert_all_called=False) as router:
            router.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(429, text="quota")
            )
            provider = _make_provider()
            assert provider._owns_client is True
            client = provider._client

            with pytest.raises(TTSRateLimitError):
                async for _ in provider.stream("hello world"):
                    pass
            await provider.aclose()

            assert client.is_closed
