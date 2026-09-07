"""Log redaction helpers for untrusted upstream payloads and OS-level log text.

OWASP A09 (Security Logging and Monitoring Failures) — response bodies and
transport error messages echoed by upstream proxies MUST be sanitised before
they reach ``self.get_logger().error(...)`` or any other sink that lands in a
log file shipped off-device. The voice stack talks to a managed LLM provider
that, on 4xx/5xx, occasionally echoes the inbound ``Authorization`` header or
``Cookie`` value verbatim in the JSON error body. A naive
``f"... {exc.response.text} ..."`` would therefore leak the bearer token to
the logger.

There are two complementary helpers:

* :func:`redact_upstream_body` — header-driven redaction for upstream HTTP
  response bodies (Authorization / Cookie / JSON ``api_key`` / URL query
  tokens). Use it on every piece of text that comes back from an external
  service before logging it.

* :func:`redact_log_text` — env-var / docker-args style redaction for OS
  process logs (``DEEPSEEK_API_KEY=sk-…`` in ``docker logs``,
  ``--api-key eyJ…`` in command lines). Use it on every log line *before*
  feeding it to an LLM agent (operator-agent «ТАРС» reading container logs
  via :mod:`rob_box_mcp_tools.read_logs` — see
  ``docs/architecture/target-operator-agent-and-dialogue.md`` §6.3).

Both helpers are pure (no globals, no I/O) and safe to call from any layer.
They are deliberately *shape-driven*: they match on the shape of a
credential (header name, env-var name, JWT prefix) rather than on a specific
secret value, so they keep working across rotations and across keys that the
operator didn't know about at deploy time.
"""

from __future__ import annotations

import re

__all__ = ["redact_log_text", "redact_upstream_body"]


# Header-style credential lines, e.g.:
#     "Authorization: Bearer eyJabc..."
#     "authorization : Basic dXNlcjpwYXNz"
#     "X-Api-Key: sk-live-1234"
# The whole "name: value" pair is collapsed to "Name: ***".
_HEADER_RE = re.compile(
    r"""
    (?P<name>
        Authorization |
        Proxy-Authorization |
        Cookie |
        Set-Cookie |
        X-Api-Key            |
        X-Auth-Token         |
        X-Access-Token
    )
    \s*:\s*
    (?P<value>
        [^\s'",;}\]]+             # plain token (no whitespace, no quotes)
        (?:                        # …optionally followed by additional space-separated
            \s+                    # segments until end-of-line / closing brace.
            [^\n"'};,]+
        )*
    )
    """,
    re.IGNORECASE | re.VERBOSE,
)

# Auth-scheme keywords (Bearer, Basic, Token, Digest, Negotiate) signal that
# the credential is multi-token and extends to end-of-line. Without this,
# ``Authorization: Bearer eyJ...payload.sig`` would mask only ``Bearer``.
_AUTH_SCHEME_RE = re.compile(
    r"\b(Bearer|Basic|Token|Digest|Negotiate)\s+[^\n,;'\"}{]+",
    re.IGNORECASE,
)

# JSON-ish secret fields that sometimes appear in upstream error envelopes,
# e.g. ``"api_key": "sk-live-1234"`` or ``"access_token":"eyJ..."``.
_JSON_SECRET_RE = re.compile(
    r"""
    (?P<key>
        "?(?:api[_-]?key|api[_-]?secret|access[_-]?token|refresh[_-]?token|
          bearer|token|secret|password|passwd)"?
    )
    \s*:\s*
    (?P<quote>["'])
    (?P<value>[^"']{4,})
    (?P=quote)
    """,
    re.IGNORECASE | re.VERBOSE,
)

# URL query parameters carrying tokens (?api_key=…, ?token=…).
_URL_QUERY_RE = re.compile(
    r"((?:api[_-]?key|access[_-]?token|token|sig|signature)=)[^&\s'\"]+",
    re.IGNORECASE,
)

_MASK = "***"


def redact_upstream_body(text: str) -> str:
    """Return ``text`` with upstream credential material masked.

    The function is pure (no globals, no I/O) and safe to call from any
    layer. If ``text`` is empty or not a string, it is returned unchanged.

    The redaction is conservative — it will mask anything that *looks* like
    a credential in the three shapes above. False positives (e.g. the word
    ``"token"`` appearing in an unrelated log message) are an acceptable
    trade-off for never leaking an API key.
    """
    if not isinstance(text, str) or not text:
        return text

    # Header-style — collapse name + value to "Name: ***".
    text = _HEADER_RE.sub(lambda m: f"{m.group('name')}: {_MASK}", text)

    # Auth-scheme tokens (``Bearer eyJ...``, ``Basic dXNlcjpwYXNz``) extend
    # past a single whitespace. Mask the whole scheme+credentials span so
    # JWT-style multi-segment tokens are fully covered.
    text = _AUTH_SCHEME_RE.sub(f"{_MASK} {_MASK}", text)

    # JSON-style — collapse "key": "value" to "key": "***".
    text = _JSON_SECRET_RE.sub(
        lambda m: f'{m.group("key")}: {m.group("quote")}{_MASK}{m.group("quote")}',
        text,
    )

    # URL query-style — collapse "key=val" to "key=***".
    text = _URL_QUERY_RE.sub(lambda m: f"{m.group(1)}{_MASK}", text)

    return text


# ---------------------------------------------------------------------------
# OS / process-log redaction (``docker logs``, ``journalctl``, ROS 2 ``/rosout``)
# ---------------------------------------------------------------------------
#
# Container stderr and ``/rosout`` often carry credential material in three
# additional shapes that ``redact_upstream_body`` does not cover:
#
# 1. ``KEY=VALUE`` — the canonical env-var form, e.g.
#    ``DEEPSEEK_API_KEY=sk-live-xyz`` printed by a Python traceback that
#    captured ``os.environ`` for debugging, or by a bash invocation in a
#    process supervisor.
# 2. ``--api-key=eyJ…`` / ``--token sk-…`` — CLI flags inside ``docker inspect``
#    or ``ps``-style command lines.
# 3. Bare JWT / API-key values pasted into a free-text log line — e.g.
#    ``Authorization succeeded for eyJhbGciOi….signature`` when an
#    intermediate service forgets to format the token as a header.
#
# All three are masked by :func:`redact_log_text`. The helper is the building
# block for ``read_logs`` (operator-admin slice, шаг 11 плана миграции §13):
# before any line of a captured log reaches the LLM agent, this function
# runs over it.

# Env-var form: ``DEEPSEEK_API_KEY=sk-live-abc123``.
# We require the name to end in one of the well-known suffixes so we don't
# mask arbitrary ``NAME=value`` log lines. The list is deliberately broad
# enough to cover every credential we've ever put into ``.env.secrets``.
_ENV_SECRET_RE = re.compile(
    r"""
    (?P<name>
        \b[A-Z][A-Z0-9_]*(?:_API_KEY|_SECRET|_TOKEN|_PASSWORD|_PASSWD|
                          _PRIVATE_KEY|_ACCESS_KEY|_SESSION_KEY)
    )
    \s*=\s*
    (?P<value>[^\s'"\]\)]+)
    """,
    re.VERBOSE,
)

# Lowercase variant of the same: ``deepseek_api_key=sk-live-abc123`` —
# happens in YAML / JSON-stringified env files.
_ENV_SECRET_LC_RE = re.compile(
    r"""
    (?P<name>
        \b[a-z][a-z0-9_]*(?:_api_key|_secret|_token|_password|_passwd|
                          _private_key|_access_key|_session_key)
    )
    \s*=\s*
    (?P<value>[^\s'"\]\)]+)
    """,
    re.VERBOSE,
)

# CLI flag form: ``--api-key=eyJ…`` / ``--token sk-live-abc``.
_CLI_FLAG_RE = re.compile(
    r"""
    (?P<flag>
        --(?:api[_-]?key|token|secret|password|access[_-]?token|sig|signature)
    )
    \s*=\s*
    (?P<value>[^\s'"\]\)]+)
    """,
    re.IGNORECASE | re.VERBOSE,
)

# Bare JWT (three base64url segments separated by dots) — happens when a
# service echoes the token into a log line without wrapping it in a header.
# A JWT segment is [A-Za-z0-9_-]+; we require >= 16 chars per segment to
# avoid masking unrelated dotted identifiers (``foo.bar.baz``).
_BARE_JWT_RE = re.compile(
    r"\beyJ[A-Za-z0-9_-]{16,}\.[A-Za-z0-9_-]{16,}\.[A-Za-z0-9_-]{16,}\b"
)

# ``sk-…`` style OpenAI-compatible API keys; ``xoxb-…`` Slack; ``ghp_…`` GitHub.
# These vendors publish the prefix as part of the secret, so it is safe to
# match on the prefix and the fact that the rest is alphanumeric/dash.
_VENDOR_PREFIX_RE = re.compile(
    r"\b(?:sk-[A-Za-z0-9_-]{16,}|sk_live_[A-Za-z0-9]{16,}|sk_test_[A-Za-z0-9]{16,}"
    r"|xox[baprs]-[A-Za-z0-9-]{16,}|ghp_[A-Za-z0-9]{30,}|gho_[A-Za-z0-9]{30,})\b"
)


def redact_log_text(text: str) -> str:
    """Return ``text`` with env-var / CLI / bare-token credentials masked.

    The function is the process-log counterpart of
    :func:`redact_upstream_body`. Use it on every line of an
    OS-level log (``docker logs``, ``journalctl``, ``/rosout``) **before**
    that line reaches an LLM agent — operator-agent ТАРС reads container
    logs via the planned ``read_logs`` tool (§6.3), and the LLM stack
    otherwise has the same access as ``docker logs``.

    Coverage:

    * ``DEEPSEEK_API_KEY=…`` / ``mimo_api_key=…`` — env-var style.
    * ``--api-key=…`` / ``--token …`` — CLI flag style.
    * Bare JWT (``eyJ…``) and ``sk-…`` / ``xoxb-…`` / ``ghp_…`` tokens.

    Pure function — same call/return semantics as
    :func:`redact_upstream_body`.
    """
    if not isinstance(text, str) or not text:
        return text

    # Env-var assignments, UPPER_CASE first (covers the common .env case
    # before we get to the lower-case heuristic).
    text = _ENV_SECRET_RE.sub(lambda m: f"{m.group('name')}={_MASK}", text)
    text = _ENV_SECRET_LC_RE.sub(lambda m: f"{m.group('name')}={_MASK}", text)

    # CLI flags — drop everything after the ``=``.
    text = _CLI_FLAG_RE.sub(lambda m: f"{m.group('flag')}={_MASK}", text)

    # Bare JWT / vendor-prefixed tokens — no ``name`` to preserve, collapse
    # the whole match to a single mask token.
    text = _BARE_JWT_RE.sub(_MASK, text)
    text = _VENDOR_PREFIX_RE.sub(_MASK, text)

    return text


# Internal alias preserved for documentation/comments that reference the
# task name; functionally identical to ``redact_upstream_body``.
_RedactUpstreamBody = redact_upstream_body
