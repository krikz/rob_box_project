"""Log redaction helpers for untrusted upstream payloads.

OWASP A09 (Security Logging and Monitoring Failures) — response bodies and
transport error messages echoed by upstream proxies MUST be sanitised before
they reach ``self.get_logger().error(...)`` or any other sink that lands in a
log file shipped off-device. The voice stack talks to a managed LLM provider
that, on 4xx/5xx, occasionally echoes the inbound ``Authorization`` header or
``Cookie`` value verbatim in the JSON error body. A naive
``f"... {exc.response.text} ..."`` would therefore leak the bearer token to
the logger.

Use :func:`redact_upstream_body` (a.k.a. ``_RedactUpstreamBody``) on every
piece of untrusted text before logging it. The function is deliberately
header-driven — it matches on the *shape* of a credential (case-insensitive
header name, ``Bearer``/``Basic``/``Token`` keyword) rather than on a
specific secret value, so it keeps working across rotations.
"""

from __future__ import annotations

import re

__all__ = ["redact_upstream_body"]


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


# Internal alias preserved for documentation/comments that reference the
# task name; functionally identical to ``redact_upstream_body``.
_RedactUpstreamBody = redact_upstream_body
