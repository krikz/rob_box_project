"""Общая политика ретраев для всех harness-провайдеров.

До карточки W6-1 ``RetryPolicy`` лежал двумя дословными копиями —
``providers/deepseek.py`` и ``providers/minimax.py``. Тела совпадали
символ в символ, но классы были РАЗНЫМИ объектами, и харнес растащил их
по половинам: ``rob_box_harness.providers.RetryPolicy`` был deepseek-овой
версией, а ``rob_box_harness.tts.RetryPolicy`` (через
``tts/minimax_tts.py``) — minimax-овой. Работало это только на утиной
типизации: любой ``isinstance``-чек между LLM- и TTS-половиной молча
давал бы ``False``.

Оба прежних модуля теперь ре-экспортируют класс отсюда, поэтому все
существующие импорты (``providers.deepseek``, ``providers.minimax``,
``providers``, ``tts``) продолжают работать и указывают на один объект.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

__all__ = ["RetryPolicy"]


@dataclass(frozen=True)
class RetryPolicy:
    """Retry behaviour for transient provider failures.

    Only transient errors (``RateLimitError`` / ``TimeoutError``) are
    retried. ``AuthError`` / ``ContentFilterError`` /
    ``CapabilityUnavailableError`` always propagate immediately —
    retrying them only hides a real bug.

    ``max_attempts`` is the upper bound including the initial call.
    ``max_attempts=1`` disables retries (the initial call still
    happens). ``backoff_base`` is the first retry delay in seconds;
    each subsequent delay is ``backoff_base * 2 ** (attempt - 1)``
    plus a uniform random jitter in ``[0, backoff_jitter)`` to avoid
    thundering herds. Wait times are therefore bounded by
    ``sum(backoff_base * 2 ** i for i in range(max_attempts - 1))``
    plus jitter.
    """

    max_attempts: int = 3
    backoff_base: float = 0.5
    backoff_jitter: float = 0.25

    def __post_init__(self) -> None:
        if self.max_attempts < 1:
            raise ValueError(
                f"RetryPolicy.max_attempts must be >= 1; got {self.max_attempts}"
            )
        if self.backoff_base < 0:
            raise ValueError(
                f"RetryPolicy.backoff_base must be >= 0; got {self.backoff_base}"
            )
        if self.backoff_jitter < 0:
            raise ValueError(
                f"RetryPolicy.backoff_jitter must be >= 0; got {self.backoff_jitter}"
            )

    def delay_for(self, attempt: int) -> float:
        """Return the sleep duration before retry ``attempt`` (1-based).

        ``attempt=1`` is the first retry (after the initial call).
        Returns 0.0 when ``attempt`` is out of range — callers should
        never invoke ``delay_for`` past ``max_attempts - 1``.
        """
        if attempt < 1:
            return 0.0
        base: float = self.backoff_base * (2 ** (attempt - 1))
        jitter: float = 0.0
        if self.backoff_jitter:
            jitter = float(random.uniform(0.0, self.backoff_jitter))
        return base + jitter
