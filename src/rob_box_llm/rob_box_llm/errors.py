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


class TimeoutError(ProviderError):
    """Network or read timeout. Safe to retry with the same prompt."""


class ContentFilterError(ProviderError):
    """Provider refused the request (safety / content filter). NOT retryable."""


class AuthError(ProviderError):
    """401 / 403. Indicates bad API key or revoked token."""


__all__ = [
    "ProviderError",
    "RateLimitError",
    "TimeoutError",
    "ContentFilterError",
    "AuthError",
]
