"""TTS provider fallback chain (issue #1976).

Wraps the ``synthesize(text, ssml, voice)`` contract from :class:`TTSNode`
into a *chain* of N providers ordered by priority. On a transient failure
(``Timeout``, ``RateLimit``, ``QuotaExceeded``, ``ServiceUnavailable``)
the chain walks to the next provider; if all providers fail, the chain
raises :class:`TTSUnavailable`.

The :class:`TTSProviderChain` is a **separate, narrow API** — it does NOT
replace the inline walk inside :meth:`TTSNode._synthesize_and_play`. That
inline walk is the hot path; this module is the canonical, testable,
reusable shape for new call sites (preview-voice, e2e harness fallback,
bench scripts). The two implementations agree on the *contract* (priority
order, dead-cache, error classes) and share the helpers below.

Design constraints:

* ``synthesize_fn`` per provider is a plain callable with signature
  ``(text, ssml, voice) -> Any``. No subclassing required — this
  keeps the module usable both from production (``TTSNode`` synthesizers)
  and from tests (plain functions / ``MagicMock``).
* Dead-cache integration is *opt-in* via ``dead_cache`` argument; the
  chain does not own TTLs (the caller does — :class:`TTSNode` already
  maintains :attr:`_provider_dead_until` and friends).
* Error classification is centralised in :func:`_is_transient_failure`
  so :class:`TTSNode` and the chain agree on which exceptions trigger a
  failover.

Spec (карточка t_b33bfee1, issue #1976):
    Yandex → MiniMax → Silero (по приоритету, по Шифу)
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping, Sequence, cast

logger = logging.getLogger(__name__)


# ── Error taxonomy ──────────────────────────────────────────────────────────


class TTSUnavailable(Exception):
    """Raised when every provider in the chain failed to synthesize.

    Attributes:
        errors: per-provider exception list, in chain order. Stored as
            :class:`BaseException` to capture e.g. ``SystemExit`` if a
            provider's wrapper ever raises one.
    """

    def __init__(self, errors: Sequence[BaseException]) -> None:
        self.errors: list[BaseException] = list(errors)
        super().__init__(
            "TTS chain exhausted: all providers failed: "
            + "; ".join(str(e) for e in errors)
        )


class TransientTTSFailure(Exception):
    """Base class for failures that should trigger failover to next provider.

    Subclasses: :class:`Timeout`, :class:`RateLimit`,
    :class:`QuotaExceeded`, :class:`ServiceUnavailable`. Catch this base
    if you want to treat any recoverable upstream error uniformly.
    """


class Timeout(TransientTTSFailure):
    """Upstream provider did not respond within the deadline."""


class RateLimit(TransientTTSFailure):
    """Upstream returned 429 / Too Many Requests."""


class QuotaExceeded(TransientTTSFailure):
    """Upstream returned 402 / quota / token-plan exhausted (no retry)."""


class ServiceUnavailable(TransientTTSFailure):
    """Upstream returned 5xx / transient server-side error."""


_TRANSIENT_TYPES: tuple[type[BaseException], ...] = (
    Timeout,
    RateLimit,
    QuotaExceeded,
    ServiceUnavailable,
    TransientTTSFailure,
)


def _is_transient_failure(exc: BaseException) -> bool:
    """True if ``exc`` should trigger failover (not abort the whole chain)."""
    if isinstance(exc, _TRANSIENT_TYPES):
        return True
    # Generic 5xx / 429 / 402 surfaced as plain ``Exception`` / ``RuntimeError``
    # in legacy call sites (issue #1976 acceptance criteria). Be conservative:
    # only match explicit known substrings, NOT random RuntimeError — we don't
    # want a stray "AttributeError" to be considered a transient failover.
    msg = str(exc).lower()
    needles = (
        "timeout",
        "timed out",
        "rate limit",
        "rate_limit",
        "ratelimit",
        " 429",
        "http/1.1 429",
        " 402",
        "quota",
        "token plan",
        "service unavailable",
        " 5xx",
        " 500 ",
        " 502 ",
        " 503 ",
        " 504 ",
        "grpc unavailable",
        "unavailable",
        "auth error, no retry",
        # Issue #1976 acceptance — YANDEX_AUTHERROR surfaces as plain
        # ``RuntimeError`` in the upstream wrappers used by the e2e
        # harness. Match the magic-token substring so the chain treats
        # it as recoverable (fall through to MiniMax).
        "yandex_autherror",
        "yandex_auth",
        "yandex auth",
    )
    return any(n in msg for n in needles)


# ── Provider descriptor ────────────────────────────────────────────────────


# Module-level alias — a ``Callable[[str, Mapping | None, str | None], Any]``
# is the documented contract. Defining it here keeps the slot dataclass
# annotation self-documenting without pulling in :mod:`typing.Protocol`.
SynthesizeFn = Callable[[str, Mapping[str, Any] | None, str | None], Any]


@dataclass(frozen=True)
class TTSProviderSlot:
    """A single provider slot in the chain.

    Attributes:
        name: short identifier — one of ``{"yandex", "minimax", "silero"}``.
            Used for log lines, dead-cache keys, and ``provider_chain``
            YAML deserialisation.
        synthesize: callable that performs the actual TTS work.
        priority: 1-based ordering (1 = highest). The chain sorts
            providers by this field; ties keep insertion order via
            :func:`sorted(..., key=...)` stable semantics.
        voice: default voice for this slot — passed through to
            ``synthesize`` when the caller did not override.
    """

    name: str
    synthesize: SynthesizeFn
    priority: int
    voice: str | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("TTSProviderSlot.name must be non-empty")
        if self.priority < 1:
            raise ValueError(
                f"TTSProviderSlot.priority must be >= 1 (got {self.priority})"
            )


# ── Chain ───────────────────────────────────────────────────────────────────


# Default chain (per Шифу, issue #1976 acceptance criteria):
#   Yandex (prio 1) → MiniMax (prio 2) → Silero (prio 3, last fallback)
DEFAULT_CHAIN: tuple[str, ...] = ("yandex", "minimax", "silero")

_KNOWN_PROVIDERS: frozenset[str] = frozenset(DEFAULT_CHAIN)


class TTSProviderChain:
    """Ordered fallback chain over :class:`TTSProviderSlot` instances.

    The chain walks providers in priority order. A *transient* failure
    (per :func:`_is_transient_failure`) skips to the next provider and
    records the exception; any other exception propagates unchanged. If
    every provider failed, :class:`TTSUnavailable` is raised carrying the
    per-provider exception list.

    Example (production, tts_node integration)::

        chain = TTSProviderChain(
            [
                TTSProviderSlot("yandex", self._synthesize_yandex, priority=1,
                                voice=self.yandex_voice),
                TTSProviderSlot("minimax", self._synthesize_minimax, priority=2,
                                voice=self.minimax_voice),
                TTSProviderSlot("silero", self._synthesize_silero, priority=3,
                                voice=self.silero_speaker),
            ],
            dead_cache=self._dead_cache_for,  # optional, see __init__
        )
        audio = chain.synthesize("привет", ssml=None, voice=None)

    Example (test, all three failing)::

        chain = TTSProviderChain([
            TTSProviderSlot("yandex", MagicMock(side_effect=RuntimeError("boom")), 1),
            TTSProviderSlot("minimax", MagicMock(side_effect=RuntimeError("boom")), 2),
            TTSProviderSlot("silero", MagicMock(side_effect=RuntimeError("boom")), 3),
        ])
        with pytest.raises(TTSUnavailable):
            chain.synthesize("hi", None, None)
    """

    def __init__(
        self,
        providers: Sequence[TTSProviderSlot],
        *,
        dead_cache: Callable[[str], bool] | None = None,
        mark_dead: Callable[[str, BaseException], None] | None = None,
    ) -> None:
        if not providers:
            raise ValueError("TTSProviderChain needs at least one provider")
        # Stable sort by priority — preserves insertion order for ties.
        self._providers: tuple[TTSProviderSlot, ...] = tuple(
            sorted(providers, key=lambda p: p.priority)
        )
        # Optional dead-cache integration (caller owns TTL semantics).
        # ``dead_cache(name) -> True`` means "skip this provider this call".
        self._dead_cache = dead_cache
        # ``mark_dead(name, exc)`` is called once per transient failure so
        # the caller can apply its own TTL classification (long for quota,
        # short for network).
        self._mark_dead = mark_dead

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def providers(self) -> tuple[TTSProviderSlot, ...]:
        """Tuple of provider slots, sorted by priority."""
        return self._providers

    @property
    def names(self) -> tuple[str, ...]:
        """Names in chain order — useful for logs and provider_state."""
        return tuple(p.name for p in self._providers)

    def synthesize(
        self,
        text: str,
        ssml: Mapping[str, Any] | None = None,
        voice: str | None = None,
    ) -> Any:
        """Walk the chain; return the first provider's result.

        Raises:
            TTSUnavailable: every provider returned a transient failure.
            BaseException: a provider raised a non-transient error
                (programmer error / bad input / network race that is NOT
                classified as recoverable). Propagates unchanged — the
                chain does not silently swallow non-recoverable errors
                (ADR-0018 honesty culture).
        """
        errors: list[Exception] = []
        last_non_transient: BaseException | None = None

        for slot in self._providers:
            if self._dead_cache is not None and self._dead_cache(slot.name):
                logger.warning(
                    "tts: provider %s в кэше мёртвых — пропускаю",
                    slot.name,
                )
                continue

            chosen_voice = voice or slot.voice
            try:
                audio = slot.synthesize(text, ssml, chosen_voice)
            except _TRANSIENT_TYPES as exc:
                logger.warning(
                    "tts: %s transient failure (%s) — пробую следующий",
                    slot.name,
                    exc,
                )
                if self._mark_dead is not None:
                    self._mark_dead(slot.name, exc)
                errors.append(exc)
                continue
            except Exception as exc:
                # Honour the legacy convention (issue #1976 acceptance):
                # ``YANDEX_AUTHERROR`` / ``grpc unavailable`` / ``quota``
                # surface as plain ``Exception`` / ``RuntimeError`` in some
                # upstream wrappers. Classify by message; only skip when
                # _is_transient_failure agrees.
                if _is_transient_failure(exc):
                    logger.warning(
                        "tts: %s classified-transient failure (%s) — "
                        "пробую следующий",
                        slot.name,
                        exc,
                    )
                    if self._mark_dead is not None:
                        self._mark_dead(slot.name, exc)
                    errors.append(exc)
                    continue
                # Non-transient: surface immediately, do NOT hide it.
                last_non_transient = exc
                break

            logger.info("tts: %s OK", slot.name)
            return audio

        if last_non_transient is not None:
            # Propagate non-transient errors without wrapping — preserves
            # stack traces for the operator.
            raise last_non_transient
        if errors:
            # ``errors`` is built from per-slot ``except`` clauses that
            # type ``exc`` as :class:`Exception`; widen to
            # :class:`BaseException` for :class:`TTSUnavailable` (its
            # API allows :class:`BaseException` to capture e.g.
            # ``SystemExit`` if a wrapper ever raises one).
            raise TTSUnavailable(cast(Sequence[BaseException], errors))
        # No providers were attempted (all skipped via dead-cache).
        raise TTSUnavailable([RuntimeError("no live providers in chain")])


# ── YAML deserialisation helper ─────────────────────────────────────────────


def chain_from_yaml_config(
    config: Iterable[Mapping[str, Any]],
    synthesize_lookup: Callable[[str], SynthesizeFn],
) -> TTSProviderChain:
    """Build a :class:`TTSProviderChain` from a YAML list of providers.

    Expected shape (per карточка, voice_node.yaml / tts_node.yaml)::

        tts:
          provider_chain:
            - provider: yandex
              voice: anton
              priority: 1
            - provider: minimax
              voice: minimax-male-qn-qingse
              priority: 2
            - provider: silero
              voice: silero_ru
              priority: 3

    Args:
        config: iterable of provider entries. Each entry must have at
            least ``provider`` (str) and ``priority`` (int). Unknown
            provider names raise :class:`ValueError`.
        synthesize_lookup: callable that resolves a provider name to its
            synthesize function. Lets the caller inject production
            callables (e.g. ``lambda name: {"yandex": self._synthesize_yandex,
            "minimax": self._synthesize_minimax, ...}[name]``) without
            coupling this module to ROS2 / :class:`TTSNode`.

    Raises:
        ValueError: on unknown provider, missing key, or duplicate
            priority among live (non-Silero) providers.
    """
    slots: list[TTSProviderSlot] = []
    seen_priorities: set[int] = set()
    for entry in config:
        name = entry.get("provider")
        if not name:
            raise ValueError(
                f"provider_chain entry missing 'provider': {entry!r}"
            )
        if name not in _KNOWN_PROVIDERS:
            raise ValueError(
                f"provider_chain: unknown provider '{name}' "
                f"(known: {sorted(_KNOWN_PROVIDERS)})"
            )
        priority = entry.get("priority")
        if not isinstance(priority, int):
            raise ValueError(
                f"provider_chain[{name}] 'priority' must be int (got "
                f"{type(priority).__name__}: {priority!r})"
            )
        if priority in seen_priorities:
            raise ValueError(
                f"provider_chain: duplicate priority {priority} "
                f"(each provider must have a unique priority)"
            )
        seen_priorities.add(priority)
        slots.append(
            TTSProviderSlot(
                name=name,
                synthesize=synthesize_lookup(name),
                priority=priority,
                voice=entry.get("voice"),
            )
        )
    return TTSProviderChain(slots)


__all__ = [
    "DEFAULT_CHAIN",
    "RateLimit",
    "ServiceUnavailable",
    "SynthesizeFn",
    "QuotaExceeded",
    "Timeout",
    "TransientTTSFailure",
    "TTSProviderChain",
    "TTSProviderSlot",
    "TTSUnavailable",
    "chain_from_yaml_config",
]