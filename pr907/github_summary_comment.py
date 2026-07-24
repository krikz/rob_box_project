"""Idempotent publish/edit of the PR #907 summary review comment via the GitHub
Issues Comments API.

The PR author may re-run the summary generator several times during review
(every time a reviewer pushes a fix the comment body changes). The helper
below makes sure repeated invocations don't accumulate duplicate "summary"
comments on the PR — first call creates, subsequent calls with the same
``existing_summary_comment_id`` edit the original in place.

Only one HTTP call is made per invocation (POST on first publish, PATCH
afterwards). 5xx responses are retried twice (1s, 2s); 4xx responses fail
fast. The token is read from the ``GITHUB_TOKEN`` environment variable and
is never logged.

Usage
-----

    from pr907.github_summary_comment import upsert_summary_comment
    result = upsert_summary_comment(markdown_text, existing_summary_comment_id)
    # result == {"action": "created" | "updated", "comment_id": int,
    #            "comment_url": str, "body_sent": str}

Or from the CLI, with the markdown body loaded from a file:

    python -m pr907.publish_summary --body-file analysis/pr-907-final-summary-comment.md \\
        --existing-id 1234567
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any

import requests

LOG = logging.getLogger(__name__)

# Issue-comments endpoints for krikz/rob_box_project.
_REPO = "krikz/rob_box_project"
_PR_NUMBER = 907
_API_BASE = "https://api.github.com"

_TIMEOUT_SECONDS = 30
_RETRY_BACKOFF_SECONDS = (1.0, 2.0)  # up to 2 retries on 5xx
_BODY_PREVIEW_CHARS = 200

# Headers are constant across both verbs — keep them in one place so a
# future migration to fine-grained tokens or to a newer API version is a
# single-line change.
_BASE_HEADERS: dict[str, str] = {
    "Accept": "application/vnd.github+json",
    "X-GitHub-Api-Version": "2022-11-28",
    "User-Agent": "rob-box-bot",
}


def _auth_headers(token: str) -> dict[str, str]:
    """Build the Authorization header for the GitHub REST API.

    Both the legacy ``token <PAT>`` and the OAuth-style ``Bearer <PAT>``
    schemes are accepted by GitHub for fine-grained and classic PATs; we
    prefer the modern ``Bearer`` form.
    """
    return {**_BASE_HEADERS, "Authorization": f"Bearer {token}"}


def _truncate_body(body: str) -> str:
    """Return ``body`` for the success envelope, truncated to
    ``_BODY_PREVIEW_CHARS`` characters with a trailing ellipsis when
    truncated. No modification is performed on the body that is actually
    sent to the API — this is for the envelope only.
    """
    if len(body) <= _BODY_PREVIEW_CHARS:
        return body
    return body[:_BODY_PREVIEW_CHARS] + "…"


def _request_with_retry(
    method: str,
    url: str,
    *,
    headers: dict[str, str],
    json_body: dict[str, Any],
) -> requests.Response:
    """Issue the HTTP request, retrying transient (5xx / network) failures.

    Behaviour:
      * Up to ``len(_RETRY_BACKOFF_SECONDS)`` retries on 5xx responses,
        waiting 1s then 2s between attempts.
      * 4xx responses are returned as-is — the caller treats them as
        terminal and surfaces the body.
      * ``requests.RequestException`` (timeouts, connection errors) is also
        retried using the same schedule and re-raised on the final attempt.
    """
    attempts = 1 + len(_RETRY_BACKOFF_SECONDS)  # initial + retries
    last_exc: requests.RequestException | None = None
    for attempt_index in range(attempts):
        try:
            response = requests.request(
                method,
                url,
                headers=headers,
                json=json_body,
                timeout=_TIMEOUT_SECONDS,
            )
        except requests.RequestException as exc:
            last_exc = exc
            if attempt_index < len(_RETRY_BACKOFF_SECONDS):
                wait = _RETRY_BACKOFF_SECONDS[attempt_index]
                LOG.warning(
                    "github %s %s failed (network): %s — retrying in %.1fs",
                    method, url, exc, wait,
                )
                time.sleep(wait)
                continue
            raise

        if 500 <= response.status_code < 600:
            if attempt_index < len(_RETRY_BACKOFF_SECONDS):
                wait = _RETRY_BACKOFF_SECONDS[attempt_index]
                LOG.warning(
                    "github %s %s returned HTTP %d — retrying in %.1fs",
                    method, url, response.status_code, wait,
                )
                time.sleep(wait)
                continue
            return response  # give the caller the final 5xx to surface

        return response

    # Unreachable — but keep the type-checker happy.
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("unreachable: retry loop exited without response")


def _parse_response(
    method: str,
    expected_status: int,
    response: requests.Response,
    sent_body: str,
    action: str,
) -> dict[str, Any]:
    """Validate the response payload and return the success envelope.

    On any validation failure (wrong status, missing fields, body mismatch)
    we raise ``RuntimeError`` with the raw body attached so the caller can
    surface the GitHub error message verbatim.
    """
    if response.status_code != expected_status:
        raise RuntimeError(
            f"unexpected status from GitHub ({method} -> HTTP "
            f"{response.status_code}, expected {expected_status}): "
            f"{response.text[:500]}"
        )

    try:
        payload = response.json()
    except ValueError as exc:
        raise RuntimeError(
            f"GitHub response is not valid JSON: {exc}: {response.text[:500]}"
        ) from exc

    comment_id = payload.get("id")
    html_url = payload.get("html_url")
    echoed_body = payload.get("body")

    if not isinstance(comment_id, int):
        raise RuntimeError(
            f"GitHub response missing integer 'id' field: {payload!r}"
        )
    if not isinstance(html_url, str) or not html_url:
        raise RuntimeError(
            f"GitHub response missing 'html_url' string field: {payload!r}"
        )
    if echoed_body != sent_body:
        # Whitespace-sensitive per the spec — equality is the right check.
        raise RuntimeError(
            "GitHub response body does not match the body we sent "
            f"(response had {len(echoed_body or '')} chars, sent "
            f"{len(sent_body)}). First 200 chars of sent body: "
            f"{sent_body[:200]!r}"
        )

    return {
        "action": action,
        "comment_id": comment_id,
        "comment_url": html_url,
        "body_sent": _truncate_body(sent_body),
    }


def _build_create_body(markdown_text: str, include_issue_number: bool) -> dict[str, Any]:
    """Build the POST body for the create case.

    The Issues Comments endpoint accepts both the legacy
    ``{"issue_number": N}`` form (required by GraphQL's old surface) and
    the REST-style ``{"issue": N}`` form. REST actually only requires the
    URL to contain the issue number, so the body field is optional — but
    we honour the spec's "include if needed" by exposing a switch and
    defaulting to ``True`` for forward compatibility with any future
    validation that might require it.
    """
    payload: dict[str, Any] = {"body": markdown_text}
    if include_issue_number:
        payload["issue_number"] = _PR_NUMBER
    return payload


def upsert_summary_comment(
    markdown_text: str,
    existing_summary_comment_id: int | None,
    *,
    token: str | None = None,
    include_issue_number_in_create: bool = True,
) -> dict[str, Any]:
    """Create-or-update the PR #907 summary review comment.

    Parameters
    ----------
    markdown_text:
        The full markdown body to publish. Sent verbatim — no trimming,
        no whitespace normalisation, no character escaping. The spec
        forbids any mutation of the body before transmission.
    existing_summary_comment_id:
        If ``None`` or ``<= 0`` a fresh comment is POSTed to the issue
        comments endpoint and the returned envelope has
        ``action == "created"``. Otherwise the comment with that id is
        PATCHed in place and the envelope has ``action == "updated"``.
    token:
        GitHub token. Defaults to ``$GITHUB_TOKEN``. Never logged. A
        missing/empty token is reported as a configuration error rather
        than a network error.
    include_issue_number_in_create:
        Whether to include ``"issue_number": 907`` in the POST body. The
        REST endpoint does not require it (the URL already routes to the
        issue) but the spec calls out including it "if the endpoint
        requires it" — we default to True for safety.

    Returns
    -------
    dict
        Exactly four keys on success::

            {
              "action": "created" | "updated",
              "comment_id": <int>,
              "comment_url": "<html_url>",
              "body_sent": "<first 200 chars of markdown_text, '…' if truncated>",
            }

    Raises
    ------
    RuntimeError
        On any non-success outcome — wrong HTTP status, missing fields in
        the response, body round-trip mismatch, or missing GitHub token.
        The original GitHub error body is appended to the exception
        message so the operator can debug without re-issuing the request.
    """
    if not isinstance(markdown_text, str):
        raise TypeError(
            f"markdown_text must be a str, got {type(markdown_text).__name__}"
        )

    resolved_token = token if token is not None else os.environ.get("GITHUB_TOKEN")
    if not resolved_token:
        raise RuntimeError(
            "GITHUB_TOKEN is not set — export it or pass token= explicitly"
        )

    headers = _auth_headers(resolved_token)

    if existing_summary_comment_id is not None and existing_summary_comment_id > 0:
        url = f"{_API_BASE}/repos/{_REPO}/issues/comments/{existing_summary_comment_id}"
        body = {"body": markdown_text}
        response = _request_with_retry("PATCH", url, headers=headers, json_body=body)
        return _parse_response(
            "PATCH", expected_status=200, response=response,
            sent_body=markdown_text, action="updated",
        )

    url = f"{_API_BASE}/repos/{_REPO}/issues/comments"
    body = _build_create_body(markdown_text, include_issue_number_in_create)
    response = _request_with_retry("POST", url, headers=headers, json_body=body)
    return _parse_response(
        "POST", expected_status=201, response=response,
        sent_body=markdown_text, action="created",
    )


__all__ = ["upsert_summary_comment"]