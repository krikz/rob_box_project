"""GitHub PR summary-comment upsert helper.

Used by CI / harness automation to publish (and refresh) a single summary
comment on a pull request, so reviewers always see the latest run summary
in one canonical place instead of comment spam.

The module is intentionally **dependency-light**: only ``requests`` from
the project-wide ``requirements.txt``. The token is read from the
``GITHUB_TOKEN`` environment variable and is **never** logged or echoed
back in error messages.

Endpoint selection
------------------
``upsert_summary_comment`` decides between two endpoints based on the
caller-supplied ``existing_summary_comment_id``:

* If the id is provided (``not None`` and ``> 0``) we PATCH the existing
  comment. This avoids creating duplicate summaries on every CI run.
* Otherwise we POST a brand-new comment. The PR context (issue_number)
  is supplied implicitly by GitHub — the caller does not need to pass it.

Reliability
-----------
* 30 s timeout per HTTP request.
* Retry on 5xx and on transient network failures (timeout /
  ``ConnectionError``) — up to 2 attempts with exponential backoff
  (1 s, 2 s). 4xx responses are **not** retried because they are
  deterministic (bad token, missing scope, validation, …).
* The response body is round-tripped: we verify that GitHub echoed
  back exactly the markdown we sent, and that the returned ``id`` and
  ``html_url`` are non-empty. Any drift returns a structured error
  dict instead of a fake success.

Output contract
---------------
Success::

    {
        "action": "created" | "updated",
        "comment_id": <int>,
        "comment_url": "<html_url>",
        "body_sent": "<first 200 chars of markdown_text + '…' if longer>",
    }

Error::

    {
        "error": "<short reason — NEVER contains the token>",
        "status_code": <int | None>,
    }

Example
-------
>>> upsert_summary_comment("## Summary\\n- all green", None)
{'action': 'created', 'comment_id': 123, ...}
>>> upsert_summary_comment("## Summary\\n- fixed", 123)
{'action': 'updated', 'comment_id': 123, ...}
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Dict, Optional

import requests

_log = logging.getLogger(__name__)

#: Base URL for the ``krikz/rob_box_project`` repository. The PR context
#: is supplied by the GitHub Actions environment, so we never need an
#: ``issue_number`` here.
BASE_URL = "https://api.github.com/repos/krikz/rob_box_project"

#: Per-request timeout (seconds). Generous enough for a cold start,
#: strict enough that a hung connection fails fast.
REQUEST_TIMEOUT_S = 30

#: Maximum number of retry attempts **on top of** the first try.
#: Two retries → up to three total requests.
MAX_RETRIES = 2

#: Exponential backoff schedule in seconds. The i-th retry sleeps
#: ``BACKOFF_SCHEDULE[i]`` before re-issuing the request. Length must be
#: >= ``MAX_RETRIES``.
BACKOFF_SCHEDULE = (1, 2)

#: When truncating the markdown for the ``body_sent`` field of the
#: success payload, keep the first N characters verbatim and append
#: ``'…'`` if anything was dropped.
BODY_SENT_TRUNCATE_AT = 200


def _build_headers() -> Dict[str, str]:
    """Build the request headers.

    The token is read fresh on every call (cheap) so that tests can
    patch ``os.environ`` without monkey-patching a captured snapshot.
    """
    token = os.environ["GITHUB_TOKEN"]
    return {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "rob-box-bot",
        "Content-Type": "application/json",
    }


def _truncate_body(text: str) -> str:
    """Truncate ``text`` for the ``body_sent`` field, appending ``'…'``.

    The original ``markdown_text`` is **never** modified — only the
    human-readable echo in the success payload is shortened so that
    callers logging the return value don't drown their logs in 50 KB
    of markdown.
    """
    if len(text) <= BODY_SENT_TRUNCATE_AT:
        return text
    return text[:BODY_SENT_TRUNCATE_AT] + "…"


def _error(reason: str, status_code: Optional[int] = None) -> Dict[str, Any]:
    """Build the standard error dict.

    The reason is deliberately generic and **never** includes the
    Authorization header, so the token can't leak via log capture.
    """
    payload: Dict[str, Any] = {"error": reason}
    if status_code is not None:
        payload["status_code"] = status_code
    return payload


def _do_request(
    method: str,
    url: str,
    headers: Dict[str, str],
    json_body: Dict[str, Any],
) -> requests.Response:
    """Issue an HTTP request with retry on 5xx / network failures.

    Dispatches to ``requests.patch`` or ``requests.post`` directly so
    that tests can mock either helper individually (the task contract).
    Returns the **last** response so the caller can inspect its status.
    4xx responses are returned as-is without retry because they are
    deterministic.
    """
    if method == "PATCH":
        sender = requests.patch
    else:
        sender = requests.post

    last_exc: Optional[BaseException] = None
    for attempt in range(MAX_RETRIES + 1):
        try:
            response = sender(
                url,
                headers=headers,
                json=json_body,
                timeout=REQUEST_TIMEOUT_S,
            )
        except (requests.exceptions.Timeout, requests.exceptions.ConnectionError) as exc:
            last_exc = exc
            if attempt < MAX_RETRIES:
                _sleep_backoff(attempt)
                continue
            raise

        if 500 <= response.status_code < 600 and attempt < MAX_RETRIES:
            _sleep_backoff(attempt)
            continue

        return response

    # Defensive: should be unreachable because the loop above either
    # returns a response, retries until exhausted and raises, or returns
    # the last response. If we land here, surface the original exception.
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("github_summary: retry loop exited without a response")


def _sleep_backoff(attempt: int) -> None:
    """Sleep ``BACKOFF_SCHEDULE[attempt]`` seconds.

    Centralised so tests can monkey-patch the sleep and run instantly.
    """
    import time

    time.sleep(BACKOFF_SCHEDULE[attempt])


def upsert_summary_comment(
    markdown_text: str,
    existing_summary_comment_id: Optional[int],
) -> Dict[str, Any]:
    """Create or update the canonical PR summary comment.

    Parameters
    ----------
    markdown_text:
        Full markdown body to publish. Sent verbatim — no trim, no
        normalisation, no header/footer injection. The exact same string
        is returned by GitHub (and we verify it byte-for-byte).
    existing_summary_comment_id:
        If provided and ``> 0``, the comment is updated in place via
        ``PATCH /repos/{owner}/{repo}/issues/comments/{id}``. Otherwise a
        new comment is created via ``POST .../issues/comments``. The PR
        context is supplied by the GitHub Actions runner, not the caller.

    Returns
    -------
    dict
        On success: ``{"action", "comment_id", "comment_url", "body_sent"}``
        exactly as described in the module docstring.
        On failure (4xx / 5xx / validation / network): ``{"error", "status_code"}``
        with a short, **token-free** reason.

    Notes
    -----
    The token is read from ``os.environ["GITHUB_TOKEN"]``. If the env
    var is missing the call raises ``KeyError`` immediately — that is
    intentional: we'd rather crash loudly in CI than silently fail-open
    with an unauthenticated request.

    Examples
    --------
    >>> upsert_summary_comment("## Summary\\n- all green", None)
    {'action': 'created', 'comment_id': 123, 'comment_url': '...', 'body_sent': '## Summary\\n- all green'}
    >>> upsert_summary_comment("## Summary\\n- fixed", 123)
    {'action': 'updated', 'comment_id': 123, 'comment_url': '...', 'body_sent': '## Summary\\n- fixed'}
    """
    headers = _build_headers()
    payload_body = {"body": markdown_text}

    if existing_summary_comment_id is not None and existing_summary_comment_id > 0:
        action = "updated"
        url = f"{BASE_URL}/issues/comments/{existing_summary_comment_id}"
        expected_status = 200
    else:
        action = "created"
        url = f"{BASE_URL}/issues/comments"
        expected_status = 201

    try:
        response = _do_request("PATCH" if action == "updated" else "POST", url, headers, payload_body)
    except (requests.exceptions.Timeout, requests.exceptions.ConnectionError) as exc:
        return _error(f"network error: {type(exc).__name__}", status_code=None)

    if response.status_code != expected_status:
        return _error(_short_reason(response), status_code=response.status_code)

    # Parse JSON. GitHub always returns valid JSON for /issues/comments
    # on a 2xx, but we defend against truncation.
    try:
        data = response.json()
    except json.JSONDecodeError:
        return _error("validation failed: response is not valid JSON", status_code=response.status_code)

    body_echoed = data.get("body")
    if body_echoed != markdown_text:
        return _error("validation failed: echoed body does not match input", status_code=response.status_code)

    comment_id = data.get("id")
    comment_url = data.get("html_url")
    if not isinstance(comment_id, int) or not comment_id:
        return _error("validation failed: missing or empty id", status_code=response.status_code)
    if not isinstance(comment_url, str) or not comment_url:
        return _error("validation failed: missing or empty html_url", status_code=response.status_code)

    return {
        "action": action,
        "comment_id": comment_id,
        "comment_url": comment_url,
        "body_sent": _truncate_body(markdown_text),
    }


def _short_reason(response: requests.Response) -> str:
    """Build a short, token-free reason for an error payload.

    Includes the HTTP status line plus the first 500 chars of the
    response body so callers can see GitHub's error message without
    leaking auth headers.
    """
    text = ""
    try:
        body_text = response.text
    except Exception:  # pragma: no cover - defensive
        body_text = ""
    if body_text:
        text = body_text[:500]
    if not text:
        return f"unexpected status {response.status_code}"
    return f"unexpected status {response.status_code}: {text}"
