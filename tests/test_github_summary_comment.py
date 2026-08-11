"""Tests for ``pr907.github_summary_comment.upsert_summary_comment``.

We mock ``requests.request`` to cover the four behaviours the function
must guarantee:

* CREATE — POST is issued, returns ``{"action": "created", ...}``.
* UPDATE — PATCH is issued against the existing comment, returns
  ``{"action": "updated", ...}``.
* ERROR — unexpected status, missing fields, body round-trip mismatch
  and 4xx all surface as ``RuntimeError`` with the GitHub error body.
* RETRIES — a transient 5xx is retried up to two extra times (1s, 2s)
  before succeeding or surfacing the error.

The token is read from ``$GITHUB_TOKEN`` by default; every test that
goes through the happy path sets the env var explicitly.
"""

from __future__ import annotations

import json
import os
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
import requests

from pr907.github_summary_comment import (
    _BODY_PREVIEW_CHARS,
    _RETRY_BACKOFF_SECONDS,
    upsert_summary_comment,
)

# A valid PAT-shaped string for tests. We never reach a real network so
# the value is irrelevant; just needs to be non-empty.
_TEST_TOKEN = "ghp_TEST_TOKEN_FOR_UNIT_TESTS_ONLY"


def _mock_response(
    status_code: int,
    json_payload: dict[str, Any] | None = None,
    text: str | None = None,
) -> MagicMock:
    """Build a ``requests.Response``-shaped mock with the requested status."""
    resp = MagicMock(spec=requests.Response)
    resp.status_code = status_code
    if json_payload is None and text is None:
        resp.json.side_effect = ValueError("no json body provided")
        resp.text = ""
    elif json_payload is not None:
        resp.json.return_value = json_payload
        resp.text = json.dumps(json_payload)
    else:
        resp.json.side_effect = ValueError("not json")
        resp.text = text or ""
    return resp


@pytest.fixture(autouse=True)
def _token_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Every test gets a working ``$GITHUB_TOKEN`` by default."""
    monkeypatch.setenv("GITHUB_TOKEN", _TEST_TOKEN)


# ---------------------------------------------------------------------------
# CREATE path (existing_summary_comment_id is None or 0)
# ---------------------------------------------------------------------------


def test_creates_new_comment_when_id_is_none() -> None:
    payload = {
        "id": 4242,
        "html_url": "https://github.com/krikz/rob_box_project/issues/907#issuecomment-4242",
        "body": "## TL;DR\nhello",
    }
    body_text = payload["body"]
    response = _mock_response(201, json_payload=payload)

    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ) as mocked:
        result = upsert_summary_comment(body_text, None)

    assert result == {
        "action": "created",
        "comment_id": 4242,
        "comment_url": payload["html_url"],
        "body_sent": body_text,  # short enough to be unmodified
    }
    # Exactly one HTTP call, POST, against the create endpoint, with the
    # issue_number and the body the caller passed.
    assert mocked.call_count == 1
    method, url = mocked.call_args.args
    kwargs = mocked.call_args.kwargs
    assert method == "POST"
    assert url == "https://api.github.com/repos/krikz/rob_box_project/issues/comments"
    assert kwargs["json"] == {"body": body_text, "issue_number": 907}
    assert kwargs["timeout"] == 30
    assert "Authorization" in kwargs["headers"]
    assert kwargs["headers"]["Authorization"] == f"Bearer {_TEST_TOKEN}"
    assert kwargs["headers"]["Accept"] == "application/vnd.github+json"
    assert kwargs["headers"]["User-Agent"] == "rob-box-bot"
    assert kwargs["headers"]["X-GitHub-Api-Version"] == "2022-11-28"


def test_creates_new_comment_when_id_is_zero() -> None:
    """``0`` is the "no existing id" sentinel — same code path as ``None``."""
    payload = {
        "id": 7,
        "html_url": "https://github.com/krikz/rob_box_project/issues/907#issuecomment-7",
        "body": "short",
    }
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=_mock_response(201, json_payload=payload),
    ) as mocked:
        result = upsert_summary_comment("short", 0)

    assert result["action"] == "created"
    method, _ = mocked.call_args.args
    assert method == "POST"


def test_create_omits_issue_number_when_disabled() -> None:
    payload = {
        "id": 1,
        "html_url": "https://x",
        "body": "x",
    }
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=_mock_response(201, json_payload=payload),
    ) as mocked:
        upsert_summary_comment("x", None, include_issue_number_in_create=False)

    assert mocked.call_args.kwargs["json"] == {"body": "x"}


def test_long_body_is_truncated_in_envelope() -> None:
    long_body = "a" * (_BODY_PREVIEW_CHARS + 50)
    payload = {
        "id": 1,
        "html_url": "https://x",
        "body": long_body,
    }
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=_mock_response(201, json_payload=payload),
    ):
        result = upsert_summary_comment(long_body, None)

    # Truncated preview in the envelope…
    assert result["body_sent"].endswith("…")
    assert len(result["body_sent"]) == _BODY_PREVIEW_CHARS + 1
    # …but the actual body sent over the wire is the full string, unmodified.
    assert result["body_sent"][:-1] == "a" * _BODY_PREVIEW_CHARS


def test_create_does_not_modify_body() -> None:
    """The spec is explicit: no trimming, no rewriting, no escaping."""
    tricky_body = "  leading-space\n\n# heading\n```\nraw block\n```\n\n  trailing-space  "
    payload = {
        "id": 9,
        "html_url": "https://x",
        "body": tricky_body,
    }
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=_mock_response(201, json_payload=payload),
    ) as mocked:
        result = upsert_summary_comment(tricky_body, None)

    assert mocked.call_args.kwargs["json"]["body"] == tricky_body
    assert result["action"] == "created"


# ---------------------------------------------------------------------------
# UPDATE path (existing_summary_comment_id > 0)
# ---------------------------------------------------------------------------


def test_updates_existing_comment() -> None:
    payload = {
        "id": 123,
        "html_url": "https://github.com/krikz/rob_box_project/issues/907#issuecomment-123",
        "body": "## updated",
    }
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=_mock_response(200, json_payload=payload),
    ) as mocked:
        result = upsert_summary_comment("## updated", 123)

    assert result == {
        "action": "updated",
        "comment_id": 123,
        "comment_url": payload["html_url"],
        "body_sent": "## updated",
    }
    method, url = mocked.call_args.args
    kwargs = mocked.call_args.kwargs
    assert method == "PATCH"
    assert url == "https://api.github.com/repos/krikz/rob_box_project/issues/comments/123"
    assert kwargs["json"] == {"body": "## updated"}


# ---------------------------------------------------------------------------
# ERROR paths (4xx, 5xx-after-retries, malformed response)
# ---------------------------------------------------------------------------


def test_missing_token_raises_without_calling_network() -> None:
    with patch.dict(os.environ, {"GITHUB_TOKEN": ""}, clear=False):
        with patch(
            "pr907.github_summary_comment.requests.request",
        ) as mocked:
            with pytest.raises(RuntimeError, match="GITHUB_TOKEN"):
                upsert_summary_comment("body", None)

    mocked.assert_not_called()


def test_unexpected_status_on_create_raises() -> None:
    response = _mock_response(
        500,
        json_payload={"message": "internal server error"},
    )
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ) as mocked:
        with pytest.raises(RuntimeError, match="unexpected status"):
            upsert_summary_comment("body", None)

    # Retried twice (initial + 2 retries) before surfacing.
    assert mocked.call_count == 1 + len(_RETRY_BACKOFF_SECONDS)


def test_4xx_is_not_retried() -> None:
    response = _mock_response(
        401,
        json_payload={"message": "Bad credentials"},
    )
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ) as mocked:
        with pytest.raises(RuntimeError, match="unexpected status"):
            upsert_summary_comment("body", None)

    # No retries on 4xx.
    assert mocked.call_count == 1


def test_unexpected_status_on_update_raises() -> None:
    response = _mock_response(
        404,
        json_payload={"message": "Not Found"},
    )
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ):
        with pytest.raises(RuntimeError, match="expected 200"):
            upsert_summary_comment("body", 999)


def test_body_round_trip_mismatch_raises() -> None:
    payload = {
        "id": 1,
        "html_url": "https://x",
        "body": "## different body from what we sent",
    }
    response = _mock_response(200, json_payload=payload)
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ):
        with pytest.raises(RuntimeError, match="does not match"):
            upsert_summary_comment("## we sent this", 1)


def test_response_missing_id_raises() -> None:
    payload = {"html_url": "https://x", "body": "x"}  # no id
    response = _mock_response(201, json_payload=payload)
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ):
        with pytest.raises(RuntimeError, match="missing integer 'id'"):
            upsert_summary_comment("x", None)


def test_response_missing_html_url_raises() -> None:
    payload = {"id": 1, "body": "x"}  # no html_url
    response = _mock_response(201, json_payload=payload)
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ):
        with pytest.raises(RuntimeError, match="'html_url'"):
            upsert_summary_comment("x", None)


def test_non_json_response_raises() -> None:
    response = _mock_response(201, text="<html>not json</html>")
    with patch(
        "pr907.github_summary_comment.requests.request",
        return_value=response,
    ):
        with pytest.raises(RuntimeError, match="not valid JSON"):
            upsert_summary_comment("x", None)


# ---------------------------------------------------------------------------
# RETRY behaviour
# ---------------------------------------------------------------------------


def test_5xx_is_retried_then_succeeds(monkeypatch: pytest.MonkeyPatch) -> None:
    """One 5xx then a 201: must succeed and only count one retry."""
    sleeps: list[float] = []
    monkeypatch.setattr(
        "pr907.github_summary_comment.time.sleep", lambda s: sleeps.append(s)
    )

    payload = {
        "id": 42,
        "html_url": "https://x",
        "body": "ok",
    }
    responses = [
        _mock_response(503, json_payload={"message": "try again"}),
        _mock_response(201, json_payload=payload),
    ]
    with patch(
        "pr907.github_summary_comment.requests.request",
        side_effect=responses,
    ) as mocked:
        result = upsert_summary_comment("ok", None)

    assert result["action"] == "created"
    assert mocked.call_count == 2
    # First retry waited exactly the first backoff step.
    assert sleeps == [_RETRY_BACKOFF_SECONDS[0]]


def test_two_5xx_then_success(monkeypatch: pytest.MonkeyPatch) -> None:
    sleeps: list[float] = []
    monkeypatch.setattr(
        "pr907.github_summary_comment.time.sleep", lambda s: sleeps.append(s)
    )

    payload = {
        "id": 42,
        "html_url": "https://x",
        "body": "ok",
    }
    responses = [
        _mock_response(500),
        _mock_response(502),
        _mock_response(201, json_payload=payload),
    ]
    with patch(
        "pr907.github_summary_comment.requests.request",
        side_effect=responses,
    ) as mocked:
        result = upsert_summary_comment("ok", None)

    assert result["action"] == "created"
    assert mocked.call_count == 3
    assert sleeps == list(_RETRY_BACKOFF_SECONDS)


def test_persistent_5xx_surfaces_after_all_retries(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("pr907.github_summary_comment.time.sleep", lambda s: None)

    responses = [
        _mock_response(500, json_payload={"message": "fail-1"}),
        _mock_response(502, json_payload={"message": "fail-2"}),
        _mock_response(503, json_payload={"message": "fail-3"}),
    ]
    with patch(
        "pr907.github_summary_comment.requests.request",
        side_effect=responses,
    ) as mocked:
        with pytest.raises(RuntimeError, match="expected 201"):
            upsert_summary_comment("ok", None)

    # 1 initial + 2 retries = 3 calls total.
    assert mocked.call_count == 1 + len(_RETRY_BACKOFF_SECONDS)


def test_network_exception_is_retried(monkeypatch: pytest.MonkeyPatch) -> None:
    sleeps: list[float] = []
    monkeypatch.setattr(
        "pr907.github_summary_comment.time.sleep", lambda s: sleeps.append(s)
    )

    payload = {
        "id": 42,
        "html_url": "https://x",
        "body": "ok",
    }
    side_effects = [
        requests.ConnectionError("network blip"),
        _mock_response(201, json_payload=payload),
    ]
    with patch(
        "pr907.github_summary_comment.requests.request",
        side_effect=side_effects,
    ) as mocked:
        result = upsert_summary_comment("ok", None)

    assert result["action"] == "created"
    assert mocked.call_count == 2
    assert sleeps == [_RETRY_BACKOFF_SECONDS[0]]


# ---------------------------------------------------------------------------
# Type-safety
# ---------------------------------------------------------------------------


def test_non_string_body_rejected() -> None:
    with pytest.raises(TypeError, match="markdown_text must be a str"):
        upsert_summary_comment(b"bytes-not-str", None)  # type: ignore[arg-type]