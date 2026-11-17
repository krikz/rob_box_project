"""Decision layer: типизированные решения Jev / Laya с детерминированным fallback'ом.

Issue #3084. Модуль НЕ подключён к продовым путям робота: это research
spike + PoC. Jev/Laya — опциональные ускорители; без ключа/сервера
всё работает на детерминированном fallback'е вызывающего кода.
Исследование и go/no-go: ``docs/research/jev/README.md``.
"""

from .acceptance_escalation import escalate_acceptance
from .chat_choice import ChatChoiceProvider, chat_choice_from_env
from .health import CircuitBreaker, CircuitState, DecisionMetrics
from .orchestrator import DecisionOrchestrator, answer_confidence
from .providers import DecisionProvider, DeterministicProvider, FakeDecisionProvider
from .routing import ClassRoute, RoutingConfig
from .systemone_http import SystemOneHttpProvider, jev_from_env, laya_from_env
from .types import (
    ChoiceAnswer,
    ChoiceQuestion,
    Decision,
    DecisionClass,
    DecisionProviderError,
    NoulAnswer,
    NoulQuestion,
    ScoreQuestion,
)

__all__ = [
    "ChatChoiceProvider",
    "ChoiceAnswer",
    "ChoiceQuestion",
    "CircuitBreaker",
    "CircuitState",
    "ClassRoute",
    "Decision",
    "DecisionClass",
    "DecisionMetrics",
    "DecisionOrchestrator",
    "DecisionProvider",
    "DecisionProviderError",
    "DeterministicProvider",
    "FakeDecisionProvider",
    "NoulAnswer",
    "NoulQuestion",
    "RoutingConfig",
    "ScoreQuestion",
    "SystemOneHttpProvider",
    "answer_confidence",
    "chat_choice_from_env",
    "escalate_acceptance",
    "jev_from_env",
    "laya_from_env",
]
