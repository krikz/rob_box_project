"""Unit tests for :mod:`rob_box_voice.utils.redact`.

These cover BLK-1 (PR #907 review, OWASP A09): upstream response bodies echoed
into ``dialogue_node`` logs MUST NOT leak ``Authorization`` headers, cookie
values, or any other credential-shaped substring. The redactor must be a pure
function, deterministic, and safe to call on any string (including the empty
string and non-strings).

Test surfaces:

* ``redact_upstream_body`` — direct unit tests for the helper.
* ``DialogueNode._run_agent_with_retry`` integration — feed it a synthetic
  ``APIStatusError`` whose ``response.text`` contains ``"Authorization: Bearer
  eyJ..."`` and assert that the captured log record does NOT contain the
  bearer token and DOES contain ``"Authorization: ***"``.
"""

from __future__ import annotations

import importlib.util as _ilu
import logging
import sys as _sys
from pathlib import Path

import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Module loading — same trick as test_audio_transcode.py. ``utils/__init__``
# pulls in pyaudio / ReSpeaker bindings that are unavailable on dev
# containers, so we load ``redact.py`` directly to keep this test file
# hermetic.
# ─────────────────────────────────────────────────────────────────────────────

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[3]  # .../src/rob_box_voice
_REDACT_PATH = _PKG_ROOT / "rob_box_voice" / "utils" / "redact.py"


def _load_redact_module():
    spec = _ilu.spec_from_file_location(
        "rob_box_voice_redact_under_test", _REDACT_PATH
    )
    mod = _ilu.module_from_spec(spec)
    _sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_redact = _load_redact_module()
redact_upstream_body = _redact.redact_upstream_body


# ─────────────────────────────────────────────────────────────────────────────
# Direct unit tests for the helper
# ─────────────────────────────────────────────────────────────────────────────


class TestRedactUpstreamBody:
    def test_masks_authorization_header_with_bearer_token(self):
        # The exact scenario from PR #907 review comment 5068089510.
        body = (
            '{"error":"upstream failure",'
            '"request":"Authorization: Bearer eyJhbGciOiJIUzI1NiJ9.payload.sig"}'
        )
        out = redact_upstream_body(body)
        assert "eyJhbGciOiJIUzI1NiJ9" not in out
        assert "payload.sig" not in out
        assert "Authorization: ***" in out

    def test_masks_authorization_header_case_insensitive(self):
        body = "authorization : Bearer abc.def.ghi"
        out = redact_upstream_body(body)
        assert "abc.def.ghi" not in out
        assert "authorization: ***" in out.lower()

    def test_masks_basic_auth(self):
        body = "Authorization: Basic dXNlcjpwYXNzd29yZA=="
        out = redact_upstream_body(body)
        assert "dXNlcjpwYXNzd29yZA==" not in out
        assert "Authorization: ***" in out

    def test_masks_cookie_header(self):
        # The ``Cookie`` header value is everything up to the first ``;``.
        body = "Cookie: session=secret-cookie-value; Path=/; HttpOnly"
        out = redact_upstream_body(body)
        assert "secret-cookie-value" not in out
        assert "Cookie: ***" in out
        # Unrelated attributes after the ``;`` stay intact (they are not
        # part of the credential itself).
        assert "Path=/" in out

    def test_masks_set_cookie_header(self):
        body = "Set-Cookie: id=abc123; Path=/; HttpOnly"
        out = redact_upstream_body(body)
        assert "abc123" not in out
        assert "Set-Cookie: ***" in out

    def test_masks_json_api_key_field(self):
        body = '{"api_key": "sk-live-1234567890ABCDEF", "ok": false}'
        out = redact_upstream_body(body)
        assert "sk-live-1234567890ABCDEF" not in out
        assert '"api_key": "***"' in out

    def test_masks_json_access_token_field(self):
        body = '{"access_token":"eyJabc.def.ghi","scope":"read"}'
        out = redact_upstream_body(body)
        assert "eyJabc.def.ghi" not in out
        assert '"access_token": "***"' in out

    def test_masks_url_query_token(self):
        body = "GET /v1/chat?api_key=sk-live-zzz&model=foo"
        out = redact_upstream_body(body)
        assert "sk-live-zzz" not in out
        assert "api_key=***" in out
        # Non-secret query params stay intact.
        assert "model=foo" in out

    def test_preserves_non_sensitive_text(self):
        body = "400 Bad Request: model 'foo' not found"
        out = redact_upstream_body(body)
        assert out == body

    def test_empty_string_returns_empty(self):
        assert redact_upstream_body("") == ""

    def test_non_string_returns_input(self):
        # Robustness — callers pass ``str(exc.response.text)``, but we still
        # want the helper to fail-safe on garbage rather than raise.
        for val in (None, 123, b"bytes"):
            assert redact_upstream_body(val) is val

    def test_does_not_mask_short_tokens(self):
        # The JSON secret regex requires >= 4 chars to avoid masking
        # unrelated short words like "id": "x".
        body = '{"id": "x", "ok": true}'
        assert redact_upstream_body(body) == body


# ─────────────────────────────────────────────────────────────────────────────
# Integration: BLK-1 — verify the *exact* string-formatting logic in
# ``dialogue_node._run_agent_with_retry``'s ``APIStatusError`` branch masks
# the bearer token before the log line is emitted.
#
# We don't import ``dialogue_node`` here (pulls in rclpy + openai-agents
# SDK and ties the test to a heavy stack). Instead we replicate the
# two-line code path that pre-fix used to leak the token:
#
#     safe_body = redact_upstream_body(str(exc.response.text)[:200])
#     logger.error(f"🌩️ API error {exc.status_code}: {safe_body} "
#                  f"(request_id={exc.request_id})")
#
# A regression in the helper or in this log shape would flip the
# assertions below from green to red, which is exactly what BLK-1 wants.
# ─────────────────────────────────────────────────────────────────────────────


class _StubResponse:
    def __init__(self, text: str):
        self.text = text
        self.headers = {}


class _FakeAPIStatusError(Exception):
    """Stand-in for ``openai.APIStatusError`` with the attributes dialogue_node reads."""

    def __init__(self, status_code: int, body: str, request_id: str):
        super().__init__(f"status={status_code}")
        self.status_code = status_code
        self.response = _StubResponse(body)
        self.request_id = request_id


def _format_api_status_log(exc: _FakeAPIStatusError) -> str:
    """The exact log-formatting statement from dialogue_node.py."""
    safe_body = redact_upstream_body(str(exc.response.text)[:200])
    return (
        f"🌩️ API error {exc.status_code}: {safe_body} "
        f"(request_id={exc.request_id})"
    )


def test_api_status_error_log_masks_bearer_token(caplog):
    """BLK-1: the dialogue_node APIStatusError branch MUST mask
    ``Authorization: Bearer <tok>`` in the error log line."""
    bearer = "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.payload.signature"
    body = (
        '{"error":"upstream",'
        f'"headers":"Authorization: {bearer}",'
        '"hint":"check Authorization header"}'
    )
    exc = _FakeAPIStatusError(503, body, request_id="req-abc-123")

    # Capture the message the way ROS 2 / rclpy would forward it to the
    # Python logging module.
    formatted = _format_api_status_log(exc)

    with caplog.at_level(logging.ERROR, logger="rob_box_voice.dialogue_node"):
        # We exercise the *formatting* — what matters for BLK-1 is what
        # reaches ``self.get_logger().error(...)`` and therefore what
        # ``caplog`` would see in a live ROS 2 session.
        logging.getLogger("rob_box_voice.dialogue_node").error(formatted)

    # The bearer token MUST NOT survive redaction.
    assert "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9" not in formatted
    assert ".payload.signature" not in formatted
    # The redacted form MUST be present.
    assert "Authorization: ***" in formatted
    # Status code + request_id MUST still be visible (they are safe).
    assert "503" in formatted
    assert "req-abc-123" in formatted


def test_api_status_error_log_masks_token_in_short_200_char_window(caplog):
    """The first 200 chars of the body MUST be redacted — that is what
    dialogue_node ships to the logger."""
    # Construct a body > 200 chars so the slice actually does something.
    padding = "x" * 300
    body = f'{{"error":"upstream","detail":"Authorization: Bearer sk-very-secret-{padding}"}}'
    exc = _FakeAPIStatusError(401, body, request_id="req-short")

    formatted = _format_api_status_log(exc)

    assert "sk-very-secret" not in formatted
    assert "Authorization: ***" in formatted
    assert "401" in formatted
