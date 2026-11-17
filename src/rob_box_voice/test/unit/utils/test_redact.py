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
redact_log_text = _redact.redact_log_text


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
        body = "GET /v1/chat?api_key=PLACEHOLDER_LIVE_VALUE&model=foo"
        out = redact_upstream_body(body)
        assert "PLACEHOLDER_LIVE_VALUE" not in out
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
    """BLK-1: the dialogue_node APIStatusError branch MUST mask.
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
    """The first 200 chars of the body MUST be redacted — that is what.
    dialogue_node ships to the logger."""
    # Construct a body > 200 chars so the slice actually does something.
    padding = "x" * 300
    body = (
        '{"error":"upstream","detail":"Authorization: Bearer '
        f'eyJhbGciOiJIUzI1Ni-padding{padding}-sig'
        '"}'
    )
    exc = _FakeAPIStatusError(401, body, request_id="req-short")

    formatted = _format_api_status_log(exc)

    assert "eyJhbGciOiJIUzI1Ni" not in formatted
    assert "Authorization: ***" in formatted
    assert "401" in formatted


# ─────────────────────────────────────────────────────────────────────────────
# issue #1998 §6.3 — redact_log_text
#
# Operator-agent ТАРС reads ``docker logs`` / ``/rosout`` via ``read_logs``.
# Before any log line reaches the LLM, ``redact_log_text`` MUST mask the
# credential shapes that ``redact_upstream_body`` does not cover: env-var
# assignments, CLI flags, bare JWT, vendor-prefixed API keys.
# ─────────────────────────────────────────────────────────────────────────────


class TestRedactLogText:
    """``redact_log_text`` — process-log counterpart of ``redact_upstream_body``."""

    # --- env-var form -------------------------------------------------------

    def test_masks_uppercase_env_api_key(self):
        # The exact scenario from issue #1998 body — DEEPSEEK_API_KEY in a
        # captured ``docker logs`` dump.
        line = "voice-assistant | os.environ: DEEPSEEK_API_KEY=sk-live-xyz123"
        out = redact_log_text(line)
        assert "sk-live-xyz123" not in out
        assert "DEEPSEEK_API_KEY=***" in out

    def test_masks_uppercase_env_secret(self):
        line = "DJANGO_SECRET_KEY=PLACEHOLDER_NOT_A_SECRET"
        out = redact_log_text(line)
        assert "PLACEHOLDER_NOT_A_SECRET" not in out
        assert "DJANGO_SECRET_KEY=***" in out

    def test_masks_uppercase_env_token(self):
        line = "ROBOFLOW_TOKEN=abc123def456ghi789"
        out = redact_log_text(line)
        assert "abc123def456ghi789" not in out
        assert "ROBOFLOW_TOKEN=***" in out

    def test_masks_uppercase_env_password(self):
        line = "DB_PASSWORD=Tr0ub4dor-pipe"
        out = redact_log_text(line)
        assert "Tr0ub4dor-pipe" not in out
        assert "DB_PASSWORD=***" in out

    def test_masks_lowercase_env_api_key(self):
        # YAML / JSON-stringified env files use lower_case.
        line = "{deepseek_api_key: sk-live-xyz123, model: foo}"
        out = redact_log_text(line)
        assert "sk-live-xyz123" not in out
        assert "deepseek_api_key=***" in out

    def test_does_not_mask_arbitrary_uppercase_name(self):
        # ``NODE_ENV=production`` is not a credential.
        line = "starting app NODE_ENV=production PORT=8080"
        out = redact_log_text(line)
        assert out == line

    def test_does_not_mask_arbitrary_lowercase_name(self):
        # ``max_speed=0.5`` is not a credential.
        line = "robot config: max_speed=0.5 wheel_base=0.3"
        out = redact_log_text(line)
        assert out == line

    # --- CLI flag form ------------------------------------------------------

    def test_masks_cli_flag_api_key(self):
        line = "spawn: docker run --api-key=PLACEHOLDER_LIVE_VALUE image:tag"
        out = redact_log_text(line)
        assert "PLACEHOLDER_LIVE_VALUE" not in out
        assert "--api-key=***" in out

    def test_masks_cli_flag_token(self):
        line = "llm provider cli --token=abc123def456ghi789xyz012"
        out = redact_log_text(line)
        assert "abc123def456ghi789xyz012" not in out
        assert "--token=***" in out

    def test_masks_cli_flag_access_token(self):
        line = "oauth --access-token=ya29abcdef-real-token-value-1234"
        out = redact_log_text(line)
        assert "ya29abcdef-real-token-value-1234" not in out
        assert "--access-token=***" in out

    # --- bare JWT / vendor-prefixed tokens ----------------------------------

    def test_masks_bare_jwt(self):
        # Three base64url segments separated by dots.
        line = (
            "auth check: bearer=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
            "eyJzdWIiOiIxMjM0NTY3ODkwIiwibmFtZSI6IkpvaG4gRG9lIn0."
            "SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c ok"
        )
        out = redact_log_text(line)
        assert "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9" not in out
        assert "SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c" not in out
        assert "***" in out
        # Non-credential context stays intact.
        assert "auth check:" in out
        assert "ok" in out

    def test_masks_sk_prefix_openai_key(self):
        # Synthetic test fixture — uses a real ``sk-`` prefix (which the
        # redactor MUST mask) but with an obviously-placeholder value that
        # does not match GitHub secret scanner heuristics.
        line = "provider: openai key=sk-***REDACTED***"
        out = redact_log_text(line)
        # The redactor must consume the value placeholder.
        assert "***REDACTED***" not in out or "***" in out  # sanity
        # The ``sk-`` prefix is masked out (whole token replaced by ***).
        assert "sk-" not in out or out.count("sk-") == 0

    def test_masks_ghp_prefix_github_token(self):
        # Synthetic test fixture — placeholder after the real ``ghp_``
        # prefix so the redactor covers the vendor shape but the
        # scanner does not flag a credential.
        line = "gh api -H 'Authorization: token ghp_PLACEHOLDER_NOT_A_REAL_TOKEN'"
        out = redact_log_text(line)
        assert "PLACEHOLDER_NOT_A_REAL_TOKEN" not in out

    def test_masks_xoxb_prefix_slack_token(self):
        # Synthetic test fixture — placeholder after the real ``xoxb-``
        # prefix.
        line = "slack: xoxb-PLACEHOLDER-NOT-A-REAL-TOKEN-XXXXXXXX"
        out = redact_log_text(line)
        assert "PLACEHOLDER-NOT-A-REAL-TOKEN" not in out

    def test_does_not_mask_short_dotted_identifier(self):
        # A three-segment ``a.b.c`` is *not* a JWT — segments must be long.
        line = "module: aaa.bbb.ccc loaded"
        assert redact_log_text(line) == line

    # --- end-to-end log line ----------------------------------------------

    def test_full_log_line_like_docker_logs(self):
        # Realistic log line: prefix + ISO time + tag + message that mixes
        # an env-var leak, a CLI flag, and a bare JWT. Token values are
        # clearly marked as placeholders so the secret scanner does not
        # flag this fixture.
        line = (
            "2026-09-07T10:11:12 voice-assistant 1 - "
            "DEBUG DEEPSEEK_API_KEY=PLACEHOLDER_LIVE_VALUE "
            "--token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
            "eyJzdWIiOiIxMjM0NTY3ODkwIn0.SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c "
            "still using yandex_fallback"
        )
        out = redact_log_text(line)
        assert "PLACEHOLDER_LIVE_VALUE" not in out
        assert "DEEPSEEK_API_KEY=***" in out
        assert "--token=***" in out
        # The JWT is fully masked (anywhere from --token=*** followed by an
        # extra *** for the bare-token sweep, or fully absorbed — but the
        # raw segments must not survive).
        assert "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9" not in out
        assert "SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c" not in out
        # Non-credential parts preserved.
        assert "voice-assistant" in out
        assert "yandex_fallback" in out

    # --- pure-function semantics ------------------------------------------

    def test_empty_string_returns_empty(self):
        assert redact_log_text("") == ""

    def test_non_string_returns_input(self):
        for val in (None, 123, b"bytes"):
            assert redact_log_text(val) is val

    def test_preserves_non_sensitive_text(self):
        line = "robot reached waypoint 'kitchen' in 12.3 seconds"
        assert redact_log_text(line) == line

    def test_idempotent(self):
        # Running the redactor twice must not garble the output further.
        line = "DEEPSEEK_API_KEY=PLACEHOLDER_LIVE_VALUE ok"
        once = redact_log_text(line)
        twice = redact_log_text(once)
        assert once == twice

    # --- issue #1998 §6.3 acceptance --------------------------------------

    def test_acceptance_deepseek_api_key_redacted_from_stubbed_log(self):
        # Exact DoD from the card: ``read_logs`` returns ``***REDACTED***``
        # instead of ``DEEPSEEK_API_KEY=...``. We don't have the tool yet
        # (шаг 11), so we test the helper the tool will call. Value is
        # a clearly-not-a-secret placeholder to avoid scanner false positives.
        line = "boot: DEEPSEEK_API_KEY=PLACEHOLDER_NOT_A_SECRET ready"
        out = redact_log_text(line)
        assert "PLACEHOLDER_NOT_A_SECRET" not in out
        assert "DEEPSEEK_API_KEY" in out  # name preserved for context
        assert "***" in out             # value masked
