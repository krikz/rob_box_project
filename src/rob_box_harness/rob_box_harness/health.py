"""Provider health-check / quota cache for LLM fallback chains (issue #1082).

The reactive fallback chain (``[minimax, deepseek]``) wastes 15-19s when
the primary provider's quota is exhausted: MiniMax returns
``429 rate_limit_error Token Plan usage limit reached (2056)`` on every
call, the provider retries with backoff three times, and only then does
the wrapper switch to DeepSeek.

This module fixes the class of problem, not one provider:

* :class:`HealthCache` — per-provider state (``healthy`` / ``unavailable``)
  with a TTL (~5 min). A provider marked ``unavailable`` is skipped by
  the fallback chain until the TTL expires (quota may have been topped
  up). Optional JSON persistence (``~/.rob_box/llm_health.json``) so a
  robot restart inside the TTL window does not re-waste a request on a
  dead provider.
* :func:`check_deepseek_balance` — DeepSeek's public ``/user/balance``
  endpoint (proactive check; TTL-cached so we do not hammer it per
  request).
* MiniMax has **no** public balance endpoint (quota is only visible in
  the console usage bar), so its health is detected reactively: the
  first request that comes back with a quota-exhaustion error
  (``2056`` / ``1008`` / ``token plan ... limit``) marks the provider
  unavailable for the TTL. See :func:`is_quota_exhausted`.
* :class:`HealthAwareFallbackLLM` — an :class:`LLMProvider` wrapper that
  orders the chain as ``[healthy] + [unchecked]``, skips ``unavailable``
  providers, probes balances before the first call, and marks providers
  dead on quota / auth / repeated transient failures.

Edge cases handled (per issue #1082):

* Balance API itself is down → provider stays ``unknown`` (healthy) —
  we never block a provider because the health-check failed.
* Balance is positive but a 429 still arrives (quota changed between
  checks) → the reactive error-code path marks it unavailable, so the
  old behaviour remains as a second line of defence.
* 401/403 (bad key) → marked unavailable immediately, no retry, put at
  the end of the chain (not transient).
* After TTL expiry the state is re-checked (DeepSeek: balance API;
  MiniMax: one fast-fail request), so a refilled quota puts the
  provider back at the front.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, AsyncIterator, Iterable, Mapping, Sequence

import httpx

from rob_box_llm.errors import (
    AuthError,
    ProviderError,
    RateLimitError,
    TimeoutError as LLMTimeoutError,
)
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
)

_log = logging.getLogger(__name__)

#: Default TTL for a provider marked ``unavailable`` (seconds). Matches
#: the "~5 min" requirement in issue #1082 — long enough to avoid
#: re-hammering a dead provider, short enough that a topped-up quota is
#: picked up quickly.
DEFAULT_HEALTH_TTL_S: float = 300.0

#: Shorter TTL for *transient* failures (burst 429 / timeout) that are
#: likely to clear in seconds. The provider's own retry loop handles the
#: first few attempts; this only kicks in when the error persisted long
#: enough to escape the provider.
TRANSIENT_TTL_S: float = 30.0

#: Substrings in the MiniMax error message that identify a quota /
#: balance exhaustion (a *long* usage window, NOT a one-minute burst).
#: ``2056`` = Token Plan usage limit reached; ``1008`` = insufficient
#: balance. These must NOT be retried with a seconds-scale backoff.
QUOTA_EXHAUSTED_HINTS: tuple[str, ...] = (
    "2056",
    "1008",
    "token plan",
    "usage limit",
    "insufficient balance",
)


class ProviderStatus(str, Enum):
    """Health state of one provider in the fallback chain."""

    #: Never probed / cache expired / balance API failed. Treated as
    #: healthy (unchecked) so we never block a provider on a missing
    #: health-check.
    UNKNOWN = "unknown"
    #: Probed and balance > 0 (only providers with a balance API reach
    #: this state; others stay UNKNOWN).
    HEALTHY = "healthy"
    #: Quota exhausted / auth failure / persistent error. Skipped by
    #: the chain while the TTL is fresh.
    UNAVAILABLE = "unavailable"


@dataclass
class HealthRecord:
    """Per-provider health state with a TTL.

    ``checked_at`` uses wall-clock time (``time.time()``) so that
    persisted records survive a restart and the TTL continues from
    where it was — a robot reboot inside the dead window still skips
    the dead provider.
    """

    provider: str
    status: ProviderStatus = ProviderStatus.UNKNOWN
    reason: str | None = None
    balance: float | None = None
    checked_at: float = 0.0
    ttl_s: float = DEFAULT_HEALTH_TTL_S

    @property
    def expired(self) -> bool:
        """True when the record is older than its TTL (real wall clock).

        Prefer :meth:`is_expired` when a custom clock is injected into
        the :class:`HealthCache` — the cache's clock must drive TTL
        comparisons for deterministic tests.
        """
        return self.is_expired(time.time())

    def is_expired(self, now: float) -> bool:
        """True when the record is older than its TTL relative to ``now``."""
        return (now - self.checked_at) > self.ttl_s


class HealthCache:
    """TTL cache of per-provider health states.

    Thread-safe (the ROS2 node runs the async loop on a worker thread
    while other threads may log). Persistence is optional: when
    ``persist_path`` is set, every state change is written atomically to
    a small JSON file so a restart inside the TTL window remembers dead
    providers.

    Parameters
    ----------
    ttl_s:
        Default TTL for ``unavailable`` records (seconds).
    persist_path:
        Optional JSON file to load on init / save on change. ``None``
        (default) disables persistence — used by tests and by callers
        that do not want disk I/O.
    clock:
        Injectable clock for tests (default ``time.time``).
    """

    def __init__(
        self,
        ttl_s: float = DEFAULT_HEALTH_TTL_S,
        persist_path: str | Path | None = None,
        clock: Any = time.time,
    ) -> None:
        self.ttl_s = ttl_s
        self._records: dict[str, HealthRecord] = {}
        self._lock = threading.Lock()
        self._persist_path = Path(persist_path) if persist_path else None
        self._clock = clock
        if self._persist_path is not None:
            self._load()

    # ---- read API ------------------------------------------------------

    def get(self, provider: str) -> HealthRecord:
        """Return the record for ``provider`` (created as UNKNOWN if absent)."""
        with self._lock:
            rec = self._records.get(provider)
            if rec is None:
                rec = HealthRecord(provider=provider, ttl_s=self.ttl_s)
                self._records[provider] = rec
            return rec

    def status(self, provider: str) -> ProviderStatus:
        """Effective status, treating expired records as UNKNOWN.

        A stale ``unavailable`` (TTL passed) flips back to UNKNOWN so
        the provider is re-checked / re-tried — the quota may have been
        topped up.
        """
        rec = self.get(provider)
        if rec.is_expired(self._clock()):
            return ProviderStatus.UNKNOWN
        return rec.status

    def is_unavailable(self, provider: str) -> bool:
        """True when the provider is marked unavailable AND the TTL is fresh."""
        return self.status(provider) == ProviderStatus.UNAVAILABLE

    # ---- write API -----------------------------------------------------

    def mark_healthy(self, provider: str, *, balance: float | None = None) -> HealthRecord:
        """Record a successful probe."""
        rec = HealthRecord(
            provider=provider,
            status=ProviderStatus.HEALTHY,
            balance=balance,
            reason="balance ok" if balance is not None else None,
            checked_at=self._clock(),
            ttl_s=self.ttl_s,
        )
        return self._store(rec)

    def mark_unavailable(
        self,
        provider: str,
        *,
        reason: str,
        ttl_s: float | None = None,
    ) -> HealthRecord:
        """Record that the provider is dead for ``ttl_s`` seconds."""
        rec = HealthRecord(
            provider=provider,
            status=ProviderStatus.UNAVAILABLE,
            reason=reason,
            checked_at=self._clock(),
            ttl_s=self.ttl_s if ttl_s is None else ttl_s,
        )
        return self._store(rec)

    # ---- persistence ---------------------------------------------------

    def _store(self, rec: HealthRecord) -> HealthRecord:
        with self._lock:
            self._records[rec.provider] = rec
            if self._persist_path is not None:
                self._save_locked()
        return rec

    def _load(self) -> None:
        try:
            raw = json.loads(self._persist_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:  # corrupt / unreadable
            _log.warning("[health] cache load failed (%r); starting fresh", exc)
            return
        if not isinstance(raw, dict):
            return
        for name, data in raw.items():
            try:
                status = ProviderStatus(str(data.get("status", "unknown")))
                rec = HealthRecord(
                    provider=name,
                    status=status,
                    reason=data.get("reason"),
                    balance=data.get("balance"),
                    checked_at=float(data.get("checked_at", 0.0)),
                    ttl_s=float(data.get("ttl_s", self.ttl_s)),
                )
                # An already-expired record is kept as UNKNOWN so the
                # provider is re-checked on the next request.
                if rec.is_expired(self._clock()):
                    rec.status = ProviderStatus.UNKNOWN
                self._records[name] = rec
            except (ValueError, TypeError) as exc:  # pragma: no cover — defensive
                _log.warning("[health] skipping bad record %r (%r)", name, exc)

    def _save_locked(self) -> None:
        try:
            self._persist_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                name: {
                    "status": rec.status.value,
                    "reason": rec.reason,
                    "balance": rec.balance,
                    "checked_at": rec.checked_at,
                    "ttl_s": rec.ttl_s,
                }
                for name, rec in self._records.items()
            }
            tmp = self._persist_path.with_suffix(".tmp")
            tmp.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            tmp.replace(self._persist_path)
        except OSError as exc:  # pragma: no cover — best-effort persistence
            _log.warning("[health] cache save failed (%r); continuing in memory", exc)


# ---------------------------------------------------------------------------
# Error classification
# ---------------------------------------------------------------------------


def is_quota_exhausted(exc: BaseException) -> bool:
    """True when the error signals quota / balance exhaustion.

    These errors describe a *long* usage window (MiniMax Token Plan is
    a 5-hour window; ``2056`` / ``1008`` are documented as "do not
    retry aggressively") — retrying them with a seconds-scale backoff
    only wastes the user's time. The fallback chain should switch
    immediately and mark the provider unavailable for the TTL.
    """
    text = str(exc).lower()
    return any(hint in text for hint in QUOTA_EXHAUSTED_HINTS)


def is_auth_failure(exc: BaseException) -> bool:
    """True for 401/403-style errors — a bad key is NOT transient.

    Retrying it is pointless; the provider should go to the end of the
    chain (marked unavailable) so the robot uses a working provider.
    """
    if isinstance(exc, AuthError):
        return True
    text = str(exc).lower()
    return "401" in text or "403" in text or "invalid api key" in text


# ---------------------------------------------------------------------------
# Balance probes
# ---------------------------------------------------------------------------


async def check_deepseek_balance(
    base_url: str,
    api_key: str,
    timeout_s: float = 5.0,
) -> float | None:
    """Query DeepSeek's ``/user/balance`` endpoint.

    Returns the total balance (sum of all ``balance_infos``) or ``None``
    when the balance cannot be determined (endpoint down, network
    error, non-JSON response). ``None`` means "assume healthy" — a
    broken health-check must never block the provider.

    Reference: https://api-docs.deepseek.com/api/get-user-balance
    """
    url = base_url.rstrip("/") + "/user/balance"
    try:
        async with httpx.AsyncClient(timeout=timeout_s) as client:
            resp = await client.get(
                url,
                headers={"Authorization": f"Bearer {api_key}"},
            )
            resp.raise_for_status()
            data = resp.json()
        if not data.get("is_available", True):
            return 0.0
        infos = data.get("balance_infos") or []
        total = 0.0
        for info in infos:
            try:
                total += float(info.get("total_balance") or 0.0)
            except (TypeError, ValueError):
                continue
        return total
    except Exception as exc:  # noqa: BLE001 — any probe failure ⇒ unknown
        _log.warning("[health] deepseek balance check failed: %r", exc)
        return None


# ---------------------------------------------------------------------------
# Health-aware fallback wrapper
# ---------------------------------------------------------------------------


class HealthAwareFallbackLLM(LLMProvider):  # type: ignore[misc]
    """LLMProvider that orders the fallback chain by provider health.

    Replaces the reactive ``_FallbackLLM`` (try primary → it dies →
    switch) with a proactive one: before the first call the wrapper
    probes balances (providers that expose a balance API), skips
    providers marked ``unavailable`` (within TTL), and marks providers
    dead on quota / auth / persistent transient errors. The chain is
    ordered ``[healthy] + [unchecked]``; unavailable providers are left
    out until their TTL expires.

    Parameters
    ----------
    providers:
        Ordered fallback chain: primary first, then fallbacks. Each
        must expose ``name`` and the :class:`LLMProvider` contract.
    cache:
        Shared :class:`HealthCache`. ``None`` creates a private one
        (no persistence).
    balance_checkers:
        Optional mapping ``{provider_name: async_callable}`` where the
        callable returns ``float | None`` (balance, or None when the
        check is inconclusive). Used to probe providers BEFORE the
        first call, e.g. ``{"deepseek": lambda: check_deepseek_balance(...)}``.
    logger:
        Logger for ``[health]`` lines. Defaults to the module logger.
    """

    name = "health-aware-fallback"

    def __init__(
        self,
        providers: Sequence[LLMProvider],
        *,
        cache: HealthCache | None = None,
        balance_checkers: Mapping[str, Any] | None = None,
        logger: Any = None,
    ) -> None:
        self._providers: list[LLMProvider] = list(providers)
        if not self._providers:
            raise ValueError("HealthAwareFallbackLLM requires at least one provider")
        self._cache: HealthCache = cache or HealthCache()
        self._checkers: dict[str, Any] = dict(balance_checkers or {})
        self._log = logger if logger is not None else _log

    # ---- capability introspection --------------------------------------

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Forward to the primary provider (it determines the capability set)."""
        return self._providers[0].capabilities

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        return self._providers[0].capabilities_for(model)

    # ---- chain construction ---------------------------------------------

    def _provider_name(self, provider: LLMProvider) -> str:
        return getattr(provider, "name", type(provider).__name__)

    def _build_chain(self) -> list[LLMProvider]:
        """Order the chain as ``[healthy] + [unchecked]``, skip dead."""
        healthy: list[LLMProvider] = []
        unchecked: list[LLMProvider] = []
        for provider in self._providers:
            name = self._provider_name(provider)
            status = self._cache.status(name)
            if status == ProviderStatus.UNAVAILABLE:
                self._log.debug(
                    "[health] provider=%s unavailable — skip (TTL fresh)", name
                )
                continue
            (healthy if status == ProviderStatus.HEALTHY else unchecked).append(provider)
        return healthy + unchecked

    async def _probe(self, name: str) -> None:
        """Run the balance probe for ``name`` when one is configured.

        Only fires when the cached state is stale/unknown — the TTL
        prevents hammering the balance API on every request.
        """
        checker = self._checkers.get(name)
        if checker is None:
            return  # no probe (e.g. MiniMax) → treat as healthy/unchecked
        # Probe only when the cached state is stale/unknown — the TTL
        # prevents hammering the balance API on every request. Uses the
        # cache's clock so injected-clock tests stay deterministic.
        if self._cache.status(name) != ProviderStatus.UNKNOWN:
            return
        try:
            balance = await checker()
        except Exception as exc:  # noqa: BLE001 — probe failure ⇒ unknown
            self._log.warning(
                "[health] provider=%s balance check error (%r) — считаем рабочим",
                name,
                exc,
            )
            return
        if balance is None:
            self._log.info(
                "[health] provider=%s balance unknown — считаем рабочим", name
            )
            return
        if balance <= 0:
            self._cache.mark_unavailable(
                name, reason=f"balance={balance}", ttl_s=self._cache.ttl_s
            )
            self._log.warning(
                "[health] provider=%s balance=%s → unavailable (TTL %.0fs)",
                name,
                balance,
                self._cache.ttl_s,
            )
        else:
            self._cache.mark_healthy(name, balance=balance)
            self._log.info("[health] provider=%s balance=%s → healthy", name, balance)

    def _handle_failure(self, name: str, exc: BaseException) -> None:
        """Classify a provider failure and update the health cache."""
        exc_type = type(exc).__name__
        exc_msg = str(exc)[:200]
        if is_quota_exhausted(exc):
            self._cache.mark_unavailable(
                name, reason=str(exc)[:300], ttl_s=self._cache.ttl_s
            )
            self._log.warning(
                "[health] provider=%s quota exhausted (%s) → unavailable (TTL %.0fs)",
                name,
                exc,
                self._cache.ttl_s,
            )
        elif is_auth_failure(exc):
            self._cache.mark_unavailable(
                name, reason=f"auth: {exc}"[:300], ttl_s=self._cache.ttl_s
            )
            self._log.warning(
                "[health] provider=%s AUTH failure [%s: %s] → unavailable (TTL %.0fs)",
                name,
                exc_type,
                exc_msg,
                self._cache.ttl_s,
            )
        elif isinstance(exc, (RateLimitError, LLMTimeoutError)):
            self._cache.mark_unavailable(
                name, reason=str(exc)[:300], ttl_s=TRANSIENT_TTL_S
            )
            self._log.warning(
                "[health] provider=%s transient error (%s) → short unavailable (TTL %.0fs)",
                name,
                exc,
                TRANSIENT_TTL_S,
            )
        else:
            self._log.warning(
                "[health] provider=%s UNCLASSIFIED failure [%s: %s] — пробуем следующий",
                name,
                exc_type,
                exc_msg,
            )

    # ---- LLMProvider contract -------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        """Run a completion, skipping providers marked unavailable."""
        chain = self._build_chain()
        if not chain:
            raise ProviderError(
                "health-aware-fallback: все провайдеры unavailable "
                "(TTL ещё не истёк); ждём повторной проверки"
            )
        chain_names = [self._provider_name(p) for p in chain]
        self._log.info(
            "[health] complete: chain=%s active=%s",
            chain_names,
            chain_names[0] if chain_names else "<empty>",
        )
        last_exc: BaseException | None = None
        for provider in chain:
            name = self._provider_name(provider)
            await self._probe(name)
            if self._cache.is_unavailable(name):
                continue  # probe just marked it dead
            try:
                self._log.info("[health] → calling provider=%s", name)
                result = await provider.complete(
                    messages, tools=tools, settings=settings
                )
                self._log.info("[health] ← answered by provider=%s", name)
                return result
            except Exception as exc:  # noqa: BLE001 — any provider error ⇒ next
                last_exc = exc
                self._handle_failure(name, exc)
        if last_exc is not None:
            raise last_exc
        raise ProviderError("health-aware-fallback: chain exhausted without result")

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        """Stream a completion, skipping providers marked unavailable.

        Mirrors the reactive wrapper's semantics: if a provider fails
        before/while streaming, the next healthy provider takes over.
        """
        chain = self._build_chain()
        if not chain:
            raise ProviderError(
                "health-aware-fallback: все провайдеры unavailable "
                "(TTL ещё не истёк); ждём повторной проверки"
            )
        chain_names = [self._provider_name(p) for p in chain]
        self._log.info(
            "[health] stream: chain=%s active=%s",
            chain_names,
            chain_names[0] if chain_names else "<empty>",
        )
        last_exc: BaseException | None = None
        for provider in chain:
            name = self._provider_name(provider)
            await self._probe(name)
            if self._cache.is_unavailable(name):
                continue
            try:
                self._log.info("[health] → streaming from provider=%s", name)
                async for chunk in provider.stream(
                    messages, tools=tools, settings=settings
                ):
                    yield chunk
                self._log.info("[health] ← stream finished by provider=%s", name)
                return
            except Exception as exc:  # noqa: BLE001 — any provider error ⇒ next
                last_exc = exc
                self._handle_failure(name, exc)
        if last_exc is not None:
            raise last_exc
        raise ProviderError("health-aware-fallback: chain exhausted without stream")

    async def aclose(self) -> None:
        """Close every provider in the chain. Idempotent per provider."""
        for provider in self._providers:
            try:
                await provider.aclose()
            except Exception as exc:  # noqa: BLE001 — best-effort close
                self._log.debug(
                    "health-aware-fallback: aclose(%s) raised %r; ignoring",
                    self._provider_name(provider),
                    exc,
                )


__all__ = [
    "ProviderStatus",
    "HealthRecord",
    "HealthCache",
    "HealthAwareFallbackLLM",
    "check_deepseek_balance",
    "is_quota_exhausted",
    "is_auth_failure",
    "DEFAULT_HEALTH_TTL_S",
    "TRANSIENT_TTL_S",
    "QUOTA_EXHAUSTED_HINTS",
]
