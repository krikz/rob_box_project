"""Typed errors raised by LLMProvider implementations.

All providers MUST wrap the underlying SDK errors into one of these so that
callers (AgentSession, harnesses) can match on a stable type instead of
`openai.APIError` / `httpx` / vendor-specific strings.
"""

from __future__ import annotations


class ProviderError(Exception):
    """Base class for every error a `LLMProvider` may raise.

    `provider` is the canonical provider name (e.g. ``"deepseek"``, ``"mimo"``).
    Subclasses exist for the small handful of categories worth branching on;
    anything else surfaces as bare ``ProviderError``.
    """

    def __init__(self, message: str, *, provider: str | None = None) -> None:
        super().__init__(message)
        self.provider = provider


class RateLimitError(ProviderError):
    """429 / quota exhausted. Caller should back off + retry."""


class TimeoutError(ProviderError):  # noqa: A001 — intentional shadowing
    """Network or read timeout. Safe to retry with the same prompt."""


class ContentFilterError(ProviderError):
    """Provider-side refusal (safety filter / output validation)."""


class AuthError(ProviderError):
    """401 / 403. Indicates bad API key or revoked token."""


# ---------------------------------------------------------------------------
# TTS errors — same shape, separate hierarchy so callers can `except TTSError`
# without accidentally swallowing LLM errors (and vice versa).
# ---------------------------------------------------------------------------


class TTSError(Exception):
    """Base class for every error a :class:`TTSProvider` may raise.

    ``provider`` is the canonical provider name (e.g. ``"minimax"``).
    Subclasses exist for the small handful of categories worth branching on;
    anything else surfaces as bare ``TTSError``.
    """

    def __init__(self, message: str, *, provider: str | None = None) -> None:
        super().__init__(message)
        self.provider = provider


class TTSRateLimitError(TTSError):
    """429 / quota exhausted. Caller should back off + retry."""


class TTSTimeoutError(TTSError):
    """Network or read timeout. Safe to retry with the same prompt."""


class TTSAuthError(TTSError):
    """401 / 403. Indicates bad API key or revoked token."""


class TTSBadRequestError(TTSError):
    """400-class errors: unsupported voice / model / parameter combination."""


__all__ = [
    "ProviderError",
    "RateLimitError",
    "TimeoutError",
    "ContentFilterError",
    "AuthError",
    "TTSError",
    "TTSRateLimitError",
    "TTSTimeoutError",
    "TTSAuthError",
    "TTSBadRequestError",
]
