"""Порт ``DecisionProvider`` и локальные реализации (issue #3084).

* :class:`DecisionProvider` — протокол, общий для Jev, Laya и
  детерминированного пути. Ничего не знает про ROS.
* :class:`DeterministicProvider` — обёртка над чистой функцией правил;
  всегда отвечает, не ходит в сеть.
* :class:`FakeDecisionProvider` — сценарный провайдер для офлайн-тестов:
  отдаёт заданный ответ, бросает заданную ошибку или «виснет».

Удалённые провайдеры (Jev / Laya через Ollaya) — в :mod:`.systemone_http`.
"""

from __future__ import annotations

import asyncio
from typing import Callable, Mapping, Protocol, runtime_checkable

from .types import Answer, Decision, DecisionProviderError, Question, State


@runtime_checkable
class DecisionProvider(Protocol):
    """Источник структурированных решений по закрытым вопросам."""

    name: str

    async def decide(self, state: State, questions: Mapping[str, Question]) -> Decision:
        """Ответить на ``questions`` по ``state``.

        Реализация НЕ обязана сама соблюдать дедлайн — оркестратор
        отменяет корутину по таймауту. Любая ошибка — подкласс
        :class:`DecisionProviderError`.
        """
        ...


RulesFn = Callable[[State, Mapping[str, Question]], Mapping[str, Answer]]


class DeterministicProvider:
    """Локальные правила. Не зависит от сети, ключей и моделей."""

    name = "deterministic"

    def __init__(self, rules: RulesFn) -> None:
        self._rules = rules

    async def decide(self, state: State, questions: Mapping[str, Question]) -> Decision:
        return Decision(answers=dict(self._rules(state, questions)), provider=self.name)


class FakeDecisionProvider:
    """Сценарный провайдер для тестов без сети.

    Args:
        name: имя в цепочке маршрутизации (``"jev"``, ``"laya"``...).
        answers: что вернуть (если ``error`` не задан).
        error: исключение, которое бросить.
        delay_s: задержка перед ответом — для проверки дедлайнов.
    """

    def __init__(
        self,
        name: str,
        *,
        answers: Mapping[str, Answer] | None = None,
        error: BaseException | None = None,
        delay_s: float = 0.0,
        model: str | None = "fake-1",
    ) -> None:
        self.name = name
        self._answers = dict(answers or {})
        self._error = error
        self._delay_s = delay_s
        self._model = model
        self.calls = 0

    async def decide(self, state: State, questions: Mapping[str, Question]) -> Decision:
        self.calls += 1
        if self._delay_s:
            await asyncio.sleep(self._delay_s)
        if self._error is not None:
            raise self._error
        return Decision(answers=dict(self._answers), provider=self.name, model=self._model)


__all__ = [
    "DecisionProvider",
    "DecisionProviderError",
    "DeterministicProvider",
    "FakeDecisionProvider",
    "RulesFn",
]
