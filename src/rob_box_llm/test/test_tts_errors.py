"""Smoke tests for the TTS error hierarchy in :mod:`rob_box_llm.errors`.

The MiniMaxTTSProvider tests exercise the errors end-to-end (via
``_map_exception`` and the API response handlers), but a few invariants of
the hierarchy itself should not depend on any provider implementation:

* subclass relationships stay consistent (``TTSAuthError`` is a
  ``TTSError``)
* ``provider`` field is settable via ``__init__`` on every subclass
* ``TTSError`` does NOT accidentally inherit from ``ProviderError`` (the
  LLM error hierarchy) — callers' ``except ProviderError`` clauses must
  not catch TTS failures
* ``Exception`` is the common root, so generic ``except Exception`` keeps
  working

If you ever see a test here turn red, fix the hierarchy before shipping —
LLM-side ``except`` clauses are load-bearing.
"""

from __future__ import annotations

import pytest

from rob_box_llm.errors import (
    ProviderError,
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)


class TestHierarchyShape:
    def test_tts_subclasses_inherit_from_tts_error(self):
        for cls in (
            TTSAuthError,
            TTSRateLimitError,
            TTSTimeoutError,
            TTSBadRequestError,
        ):
            assert issubclass(cls, TTSError), f"{cls.__name__} must subclass TTSError"

    def test_tts_error_is_an_exception(self):
        assert issubclass(TTSError, Exception)

    def test_tts_error_does_not_inherit_from_provider_error(self):
        """LLM callers use ``except ProviderError`` — TTS failures MUST NOT
        be silently swallowed by those clauses.
        """
        assert not issubclass(TTSError, ProviderError)

    def test_tts_subclasses_do_not_inherit_from_provider_error(self):
        for cls in (
            TTSAuthError,
            TTSRateLimitError,
            TTSTimeoutError,
            TTSBadRequestError,
        ):
            assert not issubclass(cls, ProviderError), (
                f"{cls.__name__} must not derive from ProviderError — that would let "
                "LLM except-clauses catch TTS failures (or vice versa)."
            )


class TestProviderAttribute:
    """``exc.provider`` is the one bit of structured context every subclass
    carries — without it, log messages lose the source provider name.
    """

    def test_explicit_provider_string_passes_through(self):
        exc = TTSAuthError("oops", provider="minimax")
        assert exc.provider == "minimax"

    def test_default_provider_is_none(self):
        """``TTSError(provider=None)`` is allowed — the base class signature
        is ``*, provider: str | None = None`` and subclasses inherit it.
        Loggers must handle the None case.
        """
        exc = TTSError("oops")
        assert exc.provider is None

    @pytest.mark.parametrize(
        "cls",
        [TTSAuthError, TTSRateLimitError, TTSTimeoutError, TTSBadRequestError],
    )
    def test_each_subclass_carries_provider(self, cls):
        exc = cls("msg", provider="minimax")
        assert exc.provider == "minimax"
        assert str(exc) == "msg"  # message preserved verbatim


class TestExceptionIdentity:
    """Errors raised by one provider should not be caught by except clauses
    intended for another — the only safe match is ``except TTSError``.
    """

    def test_bare_except_tts_error_catches_all_subclasses(self):
        for cls in (
            TTSAuthError,
            TTSRateLimitError,
            TTSTimeoutError,
            TTSBadRequestError,
        ):
            with pytest.raises(TTSError):
                raise cls("x", provider="minimax")

    @pytest.mark.parametrize(
        "exc_cls,handler_cls",
        [
            (TTSRateLimitError, TTSAuthError),
            (TTSAuthError, TTSRateLimitError),
            (TTSTimeoutError, TTSBadRequestError),
            (TTSBadRequestError, TTSAuthError),
        ],
    )
    def test_subclass_handler_does_not_catch_unrelated_subclass(
        self, exc_cls, handler_cls
    ):
        """For unrelated sibling subclasses, ``except X`` must not match Y.

        Verified via issubclass (the actual mechanism ``except`` uses
        under the hood) rather than via try/except — the latter either
        quietly swallows (if it catches) or fails the test (if it
        doesn't), both of which conflate "test setup" with "verification".
        """
        # Build a Y instance; assert the handler X is not in Y's MRO.
        sample = exc_cls("x", provider="minimax")
        assert not isinstance(sample, handler_cls), (
            f"{handler_cls.__name__} should not match {exc_cls.__name__} instances "
            f"— that would defeat the per-category exception matching."
        )
        # And confirm the handler's own class catches itself.
        own = handler_cls("x", provider="minimax")
        assert isinstance(own, handler_cls)
