"""Typed errors raised by LLMProvider and TTSProvider implementations.

All providers MUST wrap the underlying SDK errors into one of these so that
callers (AgentSession, harnesses) can match on a stable type instead of
`openai.APIError` / `httpx` / vendor-specific strings.
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# LLM errors
# ---------------------------------------------------------------------------


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


class CapabilityUnavailableError(ProviderError):
    """The provider / model cannot fulfil the requested capability.

    Examples: ``image_input=True`` on a text-only model, ``tools=True`` on a
    model that doesn't support function calling, or a request that mixes a
    capability the adapter doesn't expose.

    Callers should branch on this error type to skip the provider during
    fallback selection instead of treating it as transient.
    """


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
    # LLM
    "ProviderError",
    "RateLimitError",
    "TimeoutError",
    "ContentFilterError",
    "AuthError",
    "CapabilityUnavailableError",
    # TTS
    "TTSError",
    "TTSRateLimitError",
    "TTSTimeoutError",
    "TTSAuthError",
    "TTSBadRequestError",
]
