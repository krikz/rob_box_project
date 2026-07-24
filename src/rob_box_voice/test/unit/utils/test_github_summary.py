"""Unit tests for :mod:`rob_box_voice.utils.github_summary`.

The tests patch ``requests.patch`` / ``requests.post`` at the module
level (not the underlying HTTP adapter) so we never hit the network and
don't have to spin up a real HTTP server. The convention matches
``test_audio_transcode.py``: bypass ``utils/__init__.py`` (which drags in
``pyaudio``) and import the module directly via ``importlib``.

Coverage targets (per the task acceptance):

* ``existing_summary_comment_id=None`` → POST, ``action="created"``.
* ``existing_summary_comment_id=123`` → PATCH ``/comments/123``,
  ``action="updated"``.
* Repeating the call with the same id stays in PATCH (no duplicate).
* Success / error dicts match the documented schema exactly.
* 4xx → no retry; 5xx → retries with exponential backoff.
* Timeout / ``ConnectionError`` → retry, then surface a structured
  error dict (no exception escapes).
* The ``Authorization`` header is **never** echoed in error reasons.
* The ``markdown_text`` payload is sent verbatim, and the success
  payload's ``body_sent`` is the truncated echo (≤ 200 chars + "…").
"""

from __future__ import annotations

import importlib.util as _ilu
import json
import sys as _sys
from typing import Any, Dict, List, Optional
from unittest import mock

import pytest
import requests

# Bypass rob_box_voice.utils.__init__ to avoid pulling pyaudio on dev
# containers without USB-audio stack. The github_summary module only
# needs stdlib + requests.
_HERE = __file__
_UTILS_DIR = _HERE.rsplit("/test/unit/utils/", 1)[0] + "/rob_box_voice/utils"
_TARGET = _UTILS_DIR + "/github_summary.py"

_spec = _ilu.spec_from_file_location("rob_box_voice_github_summary", _TARGET)
gs = _ilu.module_from_spec(_spec)
_sys.modules["rob_box_voice_github_summary"] = gs
_spec.loader.exec_module(gs)

upsert_summary_comment = gs.upsert_summary_comment
BASE_URL = gs.BASE_URL
REQUEST_TIMEOUT_S = gs.REQUEST_TIMEOUT_S
MAX_RETRIES = gs.MAX_RETRIES
BACKOFF_SCHEDULE = gs.BACKOFF_SCHEDULE
BODY_SENT_TRUNCATE_AT = gs.BODY_SENT_TRUNCATE_AT


# ─────────────────────────────────────────────────────────────────────────────
# Helpers / fixtures
# ─────────────────────────────────────────────────────────────────────────────


class _FakeResponse:
    """Minimal stand-in for ``requests.Response``.

    We only need ``status_code``, ``json()`` and ``text`` to be exercised
    by ``upsert_summary_comment``.
    """

    def __init__(
        self,
        status_code: int,
        json_body: Optional[Dict[str, Any]] = None,
        text: Optional[str] = None,
    ) -> None:
        self.status_code = status_code
        self._json = json_body
        self.text = text if text is not None else (json.dumps(json_body) if json_body is not None else "")

    def json(self) -> Dict[str, Any]:
        if self._json is None:
            raise json.JSONDecodeError("no body", "", 0)
        return self._json


def _ok(comment_id: int = 42, body: str = "hello", html_url: str = "https://example/comment/42") -> _FakeResponse:
    return _FakeResponse(
        status_code=200 if False else 200,  # placeholder, real value set per call site
        json_body={"id": comment_id, "body": body, "html_url": html_url},
    )


@pytest.fixture(autouse=True)
def _token_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Provide a fake GITHUB_TOKEN for every test.

    Tests that want to exercise the "missing token" branch should
    ``del`` the env var themselves.
    """
    monkeypatch.setenv("GITHUB_TOKEN", "test-token-XYZ")


@pytest.fixture
def patch_sleep() -> Any:
    """Make ``time.sleep`` a no-op so backoff tests run instantly."""
    with mock.patch.object(gs, "_sleep_backoff") as fake:
        yield fake


# ─────────────────────────────────────────────────────────────────────────────
# Endpoint selection & success shape
# ─────────────────────────────────────────────────────────────────────────────


def test_create_when_id_is_none_calls_post() -> None:
    sent: Dict[str, Any] = {}

    def fake_post(url: str, *, json: Dict[str, Any], headers: Dict[str, str], timeout: int):
        sent["url"] = url
        sent["json"] = json
        sent["headers"] = headers
        sent["timeout"] = timeout
        return _FakeResponse(201, {"id": 100, "body": json["body"], "html_url": "https://x/100"})

    with mock.patch.object(gs.requests, "post", side_effect=fake_post) as post, \
         mock.patch.object(gs.requests, "patch") as patch:
        result = upsert_summary_comment("## Summary", None)

    assert post.call_count == 1
    assert patch.call_count == 0
    assert sent["url"] == f"{BASE_URL}/issues/comments"
    assert sent["json"] == {"body": "## Summary"}
    assert sent["headers"]["Authorization"] == "Bearer test-token-XYZ"
    assert sent["headers"]["Accept"] == "application/vnd.github+json"
    assert sent["headers"]["X-GitHub-Api-Version"] == "2022-11-28"
    assert sent["headers"]["User-Agent"] == "rob-box-bot"
    assert sent["headers"]["Content-Type"] == "application/json"
    assert sent["timeout"] == REQUEST_TIMEOUT_S
    assert result == {
        "action": "created",
        "comment_id": 100,
        "comment_url": "https://x/100",
        "body_sent": "## Summary",
    }


def test_update_when_id_positive_calls_patch() -> None:
    sent: Dict[str, Any] = {}

    def fake_patch(url: str, *, json: Dict[str, Any], headers: Dict[str, str], timeout: int):
        sent["url"] = url
        sent["json"] = json
        sent["timeout"] = timeout
        return _FakeResponse(200, {"id": 123, "body": json["body"], "html_url": "https://x/123"})

    with mock.patch.object(gs.requests, "patch", side_effect=fake_patch) as patch, \
         mock.patch.object(gs.requests, "post") as post:
        result = upsert_summary_comment("## Updated", 123)

    assert patch.call_count == 1
    assert post.call_count == 0
    assert sent["url"] == f"{BASE_URL}/issues/comments/123"
    assert sent["json"] == {"body": "## Updated"}
    assert result == {
        "action": "updated",
        "comment_id": 123,
        "comment_url": "https://x/123",
        "body_sent": "## Updated",
    }


@pytest.mark.parametrize("bad_id", [0, -1])
def test_id_zero_or_negative_falls_back_to_post(bad_id: int) -> None:
    """``> 0`` is the contract — zero and negatives must POST, not PATCH."""
    with mock.patch.object(gs.requests, "post", return_value=_FakeResponse(201, {"id": 1, "body": "x", "html_url": "u"})) as post, \
         mock.patch.object(gs.requests, "patch") as patch:
        result = upsert_summary_comment("x", bad_id)
    assert post.call_count == 1
    assert patch.call_count == 0
    assert result["action"] == "created"


def test_repeat_call_with_same_id_stays_patch_no_duplicate() -> None:
    """Two calls with the same id must both PATCH (no duplicate POST)."""
    calls: List[str] = []

    def fake_patch(url: str, *, json: Dict[str, Any], headers: Dict[str, str], timeout: int):
        calls.append(url)
        return _FakeResponse(200, {"id": 123, "body": json["body"], "html_url": "https://x/123"})

    with mock.patch.object(gs.requests, "patch", side_effect=fake_patch) as patch, \
         mock.patch.object(gs.requests, "post") as post:
        first = upsert_summary_comment("run 1", 123)
        second = upsert_summary_comment("run 2", 123)

    assert patch.call_count == 2
    assert post.call_count == 0
    assert first["action"] == "updated"
    assert second["action"] == "updated"
    assert all(u.endswith("/issues/comments/123") for u in calls)


# ─────────────────────────────────────────────────────────────────────────────
# Markdown fidelity
# ─────────────────────────────────────────────────────────────────────────────


def test_markdown_text_sent_verbatim_not_modified() -> None:
    captured: Dict[str, Any] = {}

    def fake_post(url: str, *, json: Dict[str, Any], headers: Dict[str, str], timeout: int):
        captured["body"] = json["body"]
        return _FakeResponse(201, {"id": 1, "body": json["body"], "html_url": "u"})

    weird = "  leading\nand trailing  \n\twith\ttabs  \n\n"
    with mock.patch.object(gs.requests, "post", side_effect=fake_post):
        upsert_summary_comment(weird, None)

    assert captured["body"] == weird  # no strip / no replace / no normalize


def test_body_sent_truncated_with_ellipsis_only_when_long() -> None:
    long_body = "x" * 500

    def fake_post(url: str, *, json: Dict[str, Any], headers: Dict[str, str], timeout: int):
        return _FakeResponse(201, {"id": 1, "body": json["body"], "html_url": "u"})

    with mock.patch.object(gs.requests, "post", side_effect=fake_post):
        result = upsert_summary_comment(long_body, None)

    assert len(result["body_sent"]) == BODY_SENT_TRUNCATE_AT + 1  # 200 + '…'
    assert result["body_sent"].endswith("…")
    assert result["body_sent"][:-1] == "x" * BODY_SENT_TRUNCATE_AT


def test_body_sent_short_string_returned_intact() -> None:
    short = "short body"
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(201, {"id": 1, "body": short, "html_url": "u"}),
    ):
        result = upsert_summary_comment(short, None)
    assert result["body_sent"] == short


# ─────────────────────────────────────────────────────────────────────────────
# Validation: response body must echo back exactly what we sent
# ─────────────────────────────────────────────────────────────────────────────


def test_echo_mismatch_returns_validation_error() -> None:
    """GitHub normalizing whitespace would be a regression — reject loudly."""
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(201, {"id": 1, "body": "different", "html_url": "u"}),
    ):
        result = upsert_summary_comment("original", None)
    assert "error" in result
    assert "validation failed" in result["error"]
    assert result["status_code"] == 201
    assert "comment_id" not in result
    assert "comment_url" not in result


def test_missing_id_returns_validation_error() -> None:
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(201, {"body": "x", "html_url": "u"}),
    ):
        result = upsert_summary_comment("x", None)
    assert result["error"].startswith("validation failed")
    assert result["status_code"] == 201


def test_missing_html_url_returns_validation_error() -> None:
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(201, {"id": 1, "body": "x"}),
    ):
        result = upsert_summary_comment("x", None)
    assert result["error"].startswith("validation failed")
    assert result["status_code"] == 201


def test_invalid_json_response_returns_validation_error() -> None:
    bad = _FakeResponse(status_code=201, json_body=None, text="not json")
    bad._json = None  # ensure json() raises
    with mock.patch.object(gs.requests, "post", return_value=bad):
        result = upsert_summary_comment("x", None)
    assert result["error"].startswith("validation failed")
    assert result["status_code"] == 201


# ─────────────────────────────────────────────────────────────────────────────
# Retry behaviour
# ─────────────────────────────────────────────────────────────────────────────


def test_4xx_is_not_retried() -> None:
    """4xx is deterministic — retrying just hides the bug."""
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(401, text="bad credentials"),
    ) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == 1
    assert result["status_code"] == 401
    assert "401" in result["error"]


def test_5xx_is_retried_with_backoff_then_returns_error(patch_sleep: Any) -> None:
    responses = [
        _FakeResponse(503, text="unavailable"),
        _FakeResponse(504, text="timeout"),
        _FakeResponse(500, text="oops"),
    ]
    with mock.patch.object(gs.requests, "post", side_effect=responses) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == MAX_RETRIES + 1
    # Backoff sleeps called for each retry (between attempts 1→2 and 2→3).
    assert patch_sleep.call_count == MAX_RETRIES
    # The mock sees the raw attempt index (0, 1, …). The actual sleep
    # duration comes from BACKOFF_SCHEDULE inside _sleep_backoff itself.
    assert [c.args[0] for c in patch_sleep.call_args_list] == list(range(MAX_RETRIES))
    # And the schedule maps those indices to the documented durations.
    assert [BACKOFF_SCHEDULE[i] for i in range(MAX_RETRIES)] == [1, 2]
    assert result["status_code"] == 500
    assert "500" in result["error"]


def test_5xx_then_success_returns_success(patch_sleep: Any) -> None:
    """A 5xx that recovers on a later attempt must still succeed."""
    responses = [
        _FakeResponse(503, text="temporary"),
        _FakeResponse(201, {"id": 7, "body": "x", "html_url": "u"}),
    ]
    with mock.patch.object(gs.requests, "post", side_effect=responses) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == 2
    assert result["action"] == "created"
    assert result["comment_id"] == 7


def test_timeout_is_retried_then_returns_network_error(patch_sleep: Any) -> None:
    with mock.patch.object(
        gs.requests,
        "post",
        side_effect=requests.exceptions.Timeout("read timed out"),
    ) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == MAX_RETRIES + 1
    assert patch_sleep.call_count == MAX_RETRIES
    assert result["error"].startswith("network error:")
    assert "Timeout" in result["error"]
    assert result.get("status_code") is None


def test_connection_error_is_retried_then_returns_network_error(patch_sleep: Any) -> None:
    with mock.patch.object(
        gs.requests,
        "post",
        side_effect=requests.exceptions.ConnectionError("dns"),
    ) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == MAX_RETRIES + 1
    assert result["error"].startswith("network error:")
    assert "ConnectionError" in result["error"]


def test_timeout_then_success_returns_success(patch_sleep: Any) -> None:
    with mock.patch.object(
        gs.requests,
        "post",
        side_effect=[
            requests.exceptions.Timeout(),
            _FakeResponse(201, {"id": 9, "body": "x", "html_url": "u"}),
        ],
    ) as post:
        result = upsert_summary_comment("x", None)
    assert post.call_count == 2
    assert result["action"] == "created"


# ─────────────────────────────────────────────────────────────────────────────
# Token safety
# ─────────────────────────────────────────────────────────────────────────────


def test_token_never_appears_in_error_reason() -> None:
    """The Authorization request header is never surfaced in error reasons.

    GitHub does NOT echo auth headers in response bodies, so a realistic
    401/403 body looks like ``{"message": "Bad credentials"}``. We assert
    that the request token (from our env var) does not appear anywhere in
    the error string — neither directly nor via headers we might log.
    """
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(
            401,
            text='{"message": "Bad credentials", "documentation_url": "..."}',
        ),
    ):
        result = upsert_summary_comment("x", None)

    assert "test-token-XYZ" not in result["error"]
    assert "Bearer" not in result["error"]
    assert "Authorization" not in result["error"]


def test_missing_token_raises_keyerror_immediately(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail-loud: missing env var should crash, not silently send unauth'd."""
    monkeypatch.delenv("GITHUB_TOKEN", raising=False)
    with mock.patch.object(gs.requests, "post") as post:
        with pytest.raises(KeyError):
            upsert_summary_comment("x", None)
    assert post.call_count == 0


def test_request_uses_30_second_timeout() -> None:
    """The timeout is part of the contract — guard against regressions."""
    with mock.patch.object(
        gs.requests,
        "post",
        return_value=_FakeResponse(201, {"id": 1, "body": "x", "html_url": "u"}),
    ) as post:
        upsert_summary_comment("x", None)
    assert post.call_args.kwargs["timeout"] == 30
