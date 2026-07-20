"""Shared fixtures for ``rob_box_llm`` tests.

This conftest provides the building blocks that every test for the
:class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider` (and any
sibling provider) can reuse without copy-pasting boilerplate.

The fixtures requested by the task brief:

* ``minimax_provider`` — a provider instance with a **fake** API key and
  group id, with the ``MINIMAX_*`` environment variables scrubbed so a
  stray ``os.getenv`` cannot accidentally pick up a real secret. Tests
  that need to assert behaviour around missing credentials build their
  own provider explicitly.
* ``mock_minimax_http`` — a ``respx`` router with both
  ``POST /v1/t2a_v2`` (synthesis) and ``GET`` (streaming) routes
  pre-registered. Tests populate the routes' responses; the fixture
  activates the router on entry and exits the respx context on teardown
  so global router state is left clean for the next test (avoids
  "respx routes leak across tests" flakes).
* ``sample_text`` — a short, recognisable phrase. Length is fixed so
  snapshot-style tests can assert on a deterministic response size.
* ``valid_tts_params`` — a dict of valid ``voice``/``language``/``speed``
  values that pass provider-side validation, used by any test that needs
  a "happy path" ``TTSSettings`` without re-deriving the constants.

Design notes
------------

* We import from :mod:`rob_box_llm` rather than reaching into
  ``providers.minimax_tts`` directly so that any future rename of the
  provider module only needs a single conftest edit.

* ``respx`` is the priority HTTP-mock library per the task brief; tests
  that prefer constructor-injected ``httpx.MockTransport`` clients can
  build their own provider inline — both styles coexist without
  interference because ``respx`` only intercepts when its router is
  active.

* The fake key is a clearly-marked sentinel — any test that logs it
  must fail (see ``test_minimax_tts_logging.py``). If you ever change
  this string, search the test suite for the old sentinel first.
"""

from __future__ import annotations

from typing import Any, Iterator

import httpx
import pytest
import respx

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSSettings


# ---------------------------------------------------------------------------
# Sentinel credentials
# ---------------------------------------------------------------------------
#
# Distinctive strings chosen so:
#   1. They cannot be confused with a real key by humans or by accident.
#   2. Any log line that contains them fails the credential-leak guard
#      in test_minimax_tts_logging.py with a loud message rather than
#      silently shipping a real secret.
FAKE_API_KEY = "«fake-key:conftest:do-not-use-outside-tests»"
FAKE_GROUP_ID = "g-FAKE-CONFTEST-00000000-0000-0000-0000-000000000000"

# Documented MiniMax T2A v2 endpoint path. Centralising it here keeps
# every test in sync with the provider's _post() implementation and
# makes a future endpoint migration a one-line change.
MINIMAX_T2A_PATH = "/v1/t2a_v2"
MINIMAX_BASE_URL = "https://api.minimax.io"


# ---------------------------------------------------------------------------
# env scrubber — applies to every test in the directory
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _scrub_minimax_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Wipe real MiniMax credentials from the environment.

    Without this, a developer running tests on a workstation that has
    ``MINIMAX_API_KEY`` exported would have that key silently picked up
    by :class:`MiniMaxTTSProvider` constructors that omit the explicit
    ``api_key=`` kwarg. That's a credential-leak waiting to happen —
    fail loudly by removing the env vars at the top of every test.
    """
    for var in ("MINIMAX_API_KEY", "MINIMAX_GROUP_ID"):
        monkeypatch.delenv(var, raising=False)


# ---------------------------------------------------------------------------
# Sample data fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sample_text() -> str:
    """A short phrase used by synthesis tests.

    Chosen to be non-empty (so it doesn't trip the empty-text guard)
    but also non-trivial (so token-counting / duration assertions have
    something to chew on). The phrase itself is meaningless — using a
    memorable constant here would invite copy-paste into production
    code, which is exactly what we want to avoid in test data.
    """
    return "The quick brown fox jumps over the lazy dog."


@pytest.fixture
def valid_tts_params() -> dict[str, Any]:
    """Parameters that pass provider-side validation.

    Use these when you want a "happy path" ``TTSSettings`` but don't
    care about any specific value. The keys mirror ``TTSSettings``
    fields so they can be splatted directly::

        def test_x(minimax_provider, valid_tts_params):
            settings = TTSSettings(**valid_tts_params)
            ...

    Every value is within the bounds enforced by ``_build_payload``
    (volume 0..10, speed sane, language in the alias map).
    """
    return {
        "voice": "Calm_Woman",
        "model": "speech-02-hd",
        "language": "ru",
        "speed": 1.0,
        "volume": 5.0,
        "pitch": 0,
        "emotion": "neutral",
        "sample_rate": 32_000,
        "format": "pcm",
    }


@pytest.fixture
def valid_tts_settings(valid_tts_params: dict[str, Any]) -> TTSSettings:
    """A pre-built ``TTSSettings`` instance for tests that need one.

    Equivalent to ``TTSSettings(**valid_tts_params)`` — convenience for
    tests that want the object without re-typing the constructor call.
    """
    return TTSSettings(**valid_tts_params)


# ---------------------------------------------------------------------------
# Provider fixture
# ---------------------------------------------------------------------------


@pytest.fixture
def minimax_provider() -> MiniMaxTTSProvider:
    """A provider wired with fake credentials and a default base URL.

    We inject a placeholder :class:`httpx.AsyncClient` so the provider
    does NOT own its own client (``_owns_client=False``). Tests that
    exercise HTTP behaviour should pair this with the
    ``mock_minimax_http`` fixture and replace ``provider._client`` with
    a respx-backed client — the provider's HTTP plumbing is decoupled
    from the constructor.

    Injecting a placeholder (rather than letting the provider build
    its own real client) keeps fixture construction cheap: no socket
    setup, no DNS resolution, no "what happens if I run 5000 tests
    in parallel" socket exhaustion.
    """
    placeholder_client = httpx.AsyncClient()
    return MiniMaxTTSProvider(
        api_key=FAKE_API_KEY,
        group_id=FAKE_GROUP_ID,
        base_url=MINIMAX_BASE_URL,
        default_voice="male-qn-qingse",
        default_model="speech-02-hd",
        client=placeholder_client,
    )


# ---------------------------------------------------------------------------
# HTTP mock fixture — respx router (priority per task brief)
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_minimax_http() -> Iterator[respx.Router]:
    """Yield an active ``respx`` router pre-loaded with MiniMax routes.

    Usage::

        def test_x(mock_minimax_http, minimax_provider):
            route = mock_minimax_http.post("/v1/t2a_v2").mock(
                return_value=httpx.Response(200, json={"data": {"audio": "00"}, "base_resp": {"status_code": 0}})
            )
            out = await minimax_provider.synthesize("hi")
            assert route.called

    The router intercepts only while this fixture is active — entering
    ``respx.mock`` on setup and exiting on teardown means global state
    cannot leak across tests, which is the usual cause of "passes in
    isolation, fails in the full suite" respx flakes.

    Both ``POST`` (synthesis) and ``GET`` (server-side events / future
    streaming transports) are pre-registered so a test only needs to
    attach a response to the route it cares about.
    """
    with respx.mock(assert_all_called=False) as router:
        # Pre-register the two documented MiniMax HTTP routes so tests
        # don't have to know the URL. Each starts with no response —
        # tests attach one with .mock(return_value=...) or .mock(side_effect=...).
        router.post(MINIMAX_T2A_PATH).pass_through()
        router.get(MINIMAX_T2A_PATH).pass_through()
        yield router
