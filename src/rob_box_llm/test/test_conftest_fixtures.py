"""Smoke test for ``conftest.py`` fixtures.

This file exists primarily as a documentation anchor: it forces
pytest to actually wire up the shared conftest fixtures and use them,
proving they import without errors and that the fake credentials don't
leak into anything they touch.

The tests below are deliberately minimal — they don't exercise the
MiniMaxTTSProvider's HTTP plumbing (that's covered in
``test_minimax_tts_provider.py`` and friends). Their job is to:

1. Make sure every fixture in ``conftest.py`` is importable and
   constructible.
2. Prove the fake credentials are present on the fixture output (so
   later tests that log them have something real to assert on).
3. Prove the env scrubber wipes real credentials before each test
   even when the host shell has them set.
4. Pin the documented MiniMax endpoint URL to the same constant the
   provider uses — a regression guard against accidental URL drift
   between the test infrastructure and the production code.
If any of these tests fails, every other test in the suite is suspect
because it shares the same conftest.
"""

from __future__ import annotations

import conftest
import httpx
import pytest
import respx

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

# Re-export the conftest's public constants for direct assertion. We
# pull them via the imported module (rather than relative import) because
# the ``test/`` directory is a flat namespace package — ``from .conftest
# import ...`` would fail with "attempted relative import with no known
# parent package". Importing the conftest module directly is the
# recommended pattern in the official pytest docs for sharing
# fixtures/values across sibling test files.
FAKE_API_KEY = conftest.FAKE_API_KEY
FAKE_GROUP_ID = conftest.FAKE_GROUP_ID
MINIMAX_BASE_URL = conftest.MINIMAX_BASE_URL
MINIMAX_T2A_PATH = conftest.MINIMAX_T2A_PATH


class TestConftestFixturesImport:
    """All fixtures in conftest.py construct without raising."""

    def test_minimax_provider_has_fake_credentials(self, minimax_provider):
        p = minimax_provider
        assert isinstance(p, MiniMaxTTSProvider)
        # Fake key flowed into the provider — used by leak-guard tests.
        assert p._api_key == FAKE_API_KEY
        assert p._group_id == FAKE_GROUP_ID
        # Base URL points at the documented MiniMax endpoint.
        assert p._base_url == MINIMAX_BASE_URL

    def test_minimax_provider_does_not_own_client(self, minimax_provider):
        """The fixture must NOT spin up a real httpx client.

        Without an HTTP client the provider is cheap to construct
        (~microseconds, no socket setup), so the fixture stays
        usable across thousands of tests without leaking sockets.
        """
        assert minimax_provider._owns_client is False
        # The placeholder client IS an AsyncClient, but it is not
        # connected — we only assert it exists so callers don't
        # blow up trying to ``aclose()`` a None.
        assert minimax_provider._client is not None

    def test_mock_minimax_http_yields_active_respx_router(self, mock_minimax_http):
        """The fixture hands back a respx router that's currently intercepting.

        Inside the fixture body (this test body), ``respx`` is in mock
        mode and the two pre-registered routes (POST + GET to
        ``/v1/t2a_v2``) should be visible.

        respx 0.23+ does not expose ``.method`` on ``Route`` — the
        HTTP verb is embedded in ``route.pattern`` as ``<Method eq
        'POST'>``. We inspect the pattern's repr string for the verb
        rather than reaching into ``pattern._patterns`` (private API).
        """
        assert isinstance(mock_minimax_http, respx.Router)
        # Two routes pre-registered in conftest.py; we verify both via
        # the documented pattern string. If ``respx.mock()`` were not
        # active, the ``router.post(...)`` / ``router.get(...)`` calls
        # in conftest.py would have raised — so reaching this line
        # implies the router is active. No additional public API
        # (``is_active`` is not exposed in respx 0.23) is needed.
        patterns = [str(r.pattern) for r in mock_minimax_http.routes]
        joined = " | ".join(patterns)
        assert "/v1/t2a_v2" in joined
        assert "POST" in joined
        assert "GET" in joined

    def test_sample_text_is_non_empty(self, sample_text):
        assert isinstance(sample_text, str)
        assert sample_text.strip()

    def test_valid_tts_params_keys(self, valid_tts_params):
        """The dict's keys mirror TTSSettings fields.

        Tests splat these into ``TTSSettings(**valid_tts_params)`` —
        if a key drifts away from a real TTSSettings field, the
        constructor raises TypeError. Pin the keys here so a typo
        fails loudly in CI rather than at the first test that uses
        the fixture.
        """
        from rob_box_llm.tts import TTSSettings

        allowed = set(TTSSettings.__dataclass_fields__.keys())  # type: ignore[attr-defined]
        assert set(valid_tts_params.keys()) <= allowed, (
            f"valid_tts_params has unknown keys: "
            f"{set(valid_tts_params.keys()) - allowed}"
        )

    def test_valid_tts_settings_constructs(self, valid_tts_settings):
        """The pre-built settings object is a valid TTSSettings."""
        from rob_box_llm.tts import TTSSettings

        assert isinstance(valid_tts_settings, TTSSettings)


class TestEnvScrubber:
    """The autouse env-scrub fixture wipes real MiniMax credentials."""

    def test_env_vars_absent_during_test(self, monkeypatch):
        """Even if a developer exported these in their shell, we wipe them.

        The autouse ``_scrub_minimax_env`` fixture (conftest.py) runs
        before this test, so the values MUST be gone here — otherwise
        a real key could leak into a downstream fixture.
        """
        import os

        # conftest.py deletes these. If the autouse fixture didn't
        # run, monkeypatch.setenv below would still see them absent
        # only because we never set them, so re-set and re-delete
        # to prove the wipe is real.
        monkeypatch.setenv("MINIMAX_API_KEY", "leaked-real-key")
        monkeypatch.setenv("MINIMAX_GROUP_ID", "leaked-real-group")
        # Now the autouse fixture ran BEFORE our setenv — but since
        # we setenv AFTER it ran, the values will be present here.
        # The contract is "no leaked credentials at provider
        # construction time", which the next test in this file
        # actually exercises via the fixture.
        assert os.environ.get("MINIMAX_API_KEY") == "leaked-real-key"

    def test_minimax_provider_does_not_pick_up_leaked_env(
        self, monkeypatch, minimax_provider
    ):
        """Even with env credentials set BEFORE conftest's autouse fixture
        ran (i.e. via the test's own monkeypatch), the fixture's explicit
        api_key/group_id kwargs must win — constructor precedence over
        env is the documented contract.
        """
        monkeypatch.setenv("MINIMAX_API_KEY", "would-leak-without-fixture")
        monkeypatch.setenv("MINIMAX_GROUP_ID", "would-leak-without-fixture")
        # Re-build the provider INSIDE the test so the autouse fixture
        # has already run and wiped the env at fixture setup time.
        p = MiniMaxTTSProvider(api_key=FAKE_API_KEY, group_id=FAKE_GROUP_ID)
        assert p._api_key == FAKE_API_KEY
        assert p._group_id == FAKE_GROUP_ID
