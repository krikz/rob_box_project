"""``DecisionOrchestrator`` — fail-open-to-local-fallback (issue #3084).

Алгоритм для одного решения класса ``C``::

    для провайдера P в routing[C].providers:
        P не зарегистрирован        → метрика P_unconfigured, дальше
        circuit P открыт            → метрика P_circuit_open, дальше
        остаток дедлайна исчерпан   → стоп цепочки
        ответ P за остаток дедлайна → валидация → порог confidence
            ок                       → вернуть ответ P
            timeout/ошибка/невалид  → метрика, circuit.record_failure, дальше
            ниже порога              → метрика P_low_confidence, дальше
    → детерминированный fallback вызывающего кода (всегда доступен)

Инварианты (acceptance criteria issue):

* дедлайн — ОДИН на всю цепочку, а не на провайдера: суммарное ожидание
  ограничено ``deadline_ms`` класса;
* ретраев нет вообще (на критическом пути запрещены, а для остальных
  выигрыш не стоит усложнения — следующий провайдер и есть «ретрай»);
* исключения провайдеров наружу не выходят — только fallback;
* fallback выбирает вызывающий код под свой класс решения, оркестратор
  не переигрывает прошлый модельный ответ.

Порог confidence: для Choice/Score — поле ``confidence``; для Noul
отдельной confidence нет, используется ``max(p, 1 - p)`` — насколько
ответ далёк от 0.5. Это наше соглашение, не вендорское.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable, Mapping

from .health import CircuitBreaker, DecisionMetrics
from .providers import DecisionProvider
from .routing import DETERMINISTIC, RoutingConfig
from .types import (
    Answer,
    Decision,
    DecisionClass,
    DecisionProviderError,
    NoulAnswer,
    ProviderTimeout,
    Question,
    State,
    validate_answers,
)

logger = logging.getLogger(__name__)

FallbackFn = Callable[[], Mapping[str, Answer]]
BreakerFactory = Callable[[], CircuitBreaker]


def answer_confidence(answer: Answer) -> float:
    """Уверенность ответа для гейта по порогу (см. docstring модуля)."""
    if isinstance(answer, NoulAnswer):
        return max(answer.probability, 1.0 - answer.probability)
    return answer.confidence


class DecisionOrchestrator:
    """Маршрутизирует решения по цепочкам провайдеров с fallback'ом."""

    def __init__(
        self,
        providers: Mapping[str, DecisionProvider] | None = None,
        *,
        routing: RoutingConfig | None = None,
        metrics: DecisionMetrics | None = None,
        breaker_factory: BreakerFactory = CircuitBreaker,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._providers: dict[str, DecisionProvider] = {}
        self._breakers: dict[str, CircuitBreaker] = {}
        self._breaker_factory = breaker_factory
        self._routing = routing or RoutingConfig.default()
        self.metrics = metrics or DecisionMetrics()
        self._clock = clock
        for provider in (providers or {}).values():
            self.register(provider)

    # ----- runtime switching (без рестарта процесса) ----------------------

    @property
    def routing(self) -> RoutingConfig:
        return self._routing

    def set_routing(self, routing: RoutingConfig) -> None:
        """Атомарно заменить маршрутизацию; валидация — в ``RoutingConfig``."""
        self._routing = routing

    def register(self, provider: DecisionProvider) -> None:
        if provider.name == DETERMINISTIC:
            raise ValueError("deterministic fallback is supplied per call, not registered")
        self._providers[provider.name] = provider
        self._breakers[provider.name] = self._breaker_factory()

    def unregister(self, name: str) -> None:
        self._providers.pop(name, None)
        self._breakers.pop(name, None)

    def breaker(self, name: str) -> CircuitBreaker | None:
        return self._breakers.get(name)

    # ----- основной вызов -----------------------------------------------

    async def decide(
        self,
        decision_class: DecisionClass,
        state: State,
        questions: Mapping[str, Question],
        *,
        fallback: FallbackFn,
    ) -> Decision:
        """Решение с гарантированным ответом за ``deadline_ms`` + время fallback'а."""
        route = self._routing.route(decision_class)
        deadline = self._clock() + route.deadline_ms / 1000.0
        reason = "no_provider_configured"
        for name in route.providers:
            remaining = deadline - self._clock()
            if remaining <= 0:
                reason = "deadline_exhausted"
                break
            decision, reason = await self._try_provider(name, state, questions, remaining, route.min_confidence)
            if decision is not None:
                return decision
        return self._run_fallback(decision_class, questions, fallback, reason)

    async def _try_provider(
        self,
        name: str,
        state: State,
        questions: Mapping[str, Question],
        timeout_s: float,
        min_confidence: float,
    ) -> tuple[Decision | None, str]:
        provider = self._providers.get(name)
        if provider is None:
            self.metrics.count(f"{name}_unconfigured")
            return None, f"{name}_unconfigured"
        breaker = self._breakers[name]
        if not breaker.allow():
            self.metrics.count(f"{name}_circuit_open")
            return None, f"{name}_circuit_open"
        started = self._clock()
        try:
            decision = await asyncio.wait_for(provider.decide(state, questions), timeout_s)
            validate_answers(questions, decision.answers)
        except asyncio.TimeoutError:
            return self._fail(name, breaker, ProviderTimeout(f"{name}: deadline {timeout_s:.3f}s"))
        except DecisionProviderError as exc:
            return self._fail(name, breaker, exc)
        except Exception as exc:  # noqa: BLE001 — клиентский баг тоже не должен ронять решение
            logger.exception("decision provider %s crashed", name)
            return self._fail(name, breaker, DecisionProviderError(type(exc).__name__))
        latency_ms = (self._clock() - started) * 1000.0
        breaker.record_success()
        self.metrics.observe_latency(name, latency_ms)
        low = [qid for qid, a in decision.answers.items() if answer_confidence(a) < min_confidence]
        if low:
            self.metrics.count(f"{name}_low_confidence")
            return None, f"{name}_low_confidence"
        self.metrics.count(f"{name}_success")
        return _with_latency(decision, name, latency_ms), f"{name}_success"

    def _fail(self, name: str, breaker: CircuitBreaker, exc: DecisionProviderError) -> tuple[None, str]:
        breaker.record_failure()
        label = f"{name}_{exc.outcome}"
        self.metrics.count(label)
        # Текст исключения формируют наши провайдеры и не кладут туда ключ.
        logger.warning("decision provider %s failed: %s: %s", name, type(exc).__name__, exc)
        return None, label

    def _run_fallback(
        self,
        decision_class: DecisionClass,
        questions: Mapping[str, Question],
        fallback: FallbackFn,
        reason: str,
    ) -> Decision:
        answers = dict(fallback())
        validate_answers(questions, answers)  # баг в наших правилах — пусть падает громко
        self.metrics.count("fallback_used")
        logger.info("decision %s: deterministic fallback (%s)", decision_class.value, reason)
        return Decision(
            answers=answers,
            provider=DETERMINISTIC,
            fallback_used=True,
            fallback_reason=reason,
        )


def _with_latency(decision: Decision, name: str, latency_ms: float) -> Decision:
    return Decision(
        answers=decision.answers,
        provider=name,
        model=decision.model,
        latency_ms=latency_ms,
        usage=decision.usage,
    )


__all__ = ["DecisionOrchestrator", "FallbackFn", "answer_confidence"]
